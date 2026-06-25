#!/usr/bin/env python3
"""
Laberint Solver — TurtleBot3 Burger (Robot Real)

Resuelve un laberinto de pasillos estrechos (~30cm) usando:
  - LIDAR 360° para detección de paredes e intersecciones
  - Wall-following PD para navegación por pasillos
  - Grafo topológico DFS para exploración y backtracking
  - Sensor IR externo (topic /meta) para detectar la meta

Máquina de estados:
  EXPLORING → APPROACHING_INTERSECTION → ANALYZING_NODE → EXECUTING_TURN
  TURN_180 (cul-de-sac / backtracking)
  ESCAPING (desbloqueo por atasco)

Nota LIDAR: El sensor está montado al revés. scan_cb aplica una rotación
virtual de 180° para que 0° = frente del robot en todo el código.
"""
import math
from datetime import datetime
from collections import defaultdict
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

# ═══════════════════════════════════════════════════════
# Utilidades matemáticas
# ═══════════════════════════════════════════════════════

def quaternion_to_yaw(q):
    """Convierte cuaternión a ángulo yaw (radianes)."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)

def angle_diff(a, b):
    """Diferencia angular normalizada a [-π, π]."""
    return math.atan2(math.sin(a - b), math.cos(a - b))

def normalize_angle(a):
    """Normaliza ángulo a [-π, π]."""
    return math.atan2(math.sin(a), math.cos(a))

def dist2d(x1, y1, x2, y2):
    """Distancia euclidiana 2D."""
    return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)


class LaberintSolver(Node):
    def __init__(self):
        super().__init__('laberint_solver')

        self.log_filename = f"turtlebot_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.txt"
        with open(self.log_filename, 'w') as f:
            f.write("=== LOG LABERINT SOLVER ===\n")

        # ─── Velocidades ───
        self.LINEAR_SPEED  = 0.03   # Velocidad de crucero (m/s)
        self.SLOW_SPEED    = 0.01   # Velocidad en curvas y aproximaciones
        self.ANGULAR_SPEED = 0.30   # Velocidad angular máxima (rad/s)

        # ─── Umbrales de proximidad (metros) ───
        self.FRONT_BRAKE   = 0.20   # Freno frontal: detener si pared < 20cm
        self.FRONT_SLOW    = 0.25   # Reducir velocidad si pared < 25cm
        self.WALL_STOP     = 0.20   # Umbral para GATILLO 2 (pared frontal)
        self.CHASSIS_FILTER = 0.05  # Lecturas < 5cm = ruido del chasis

        # ─── Ángulos de giro ───
        self.TURN_ANGLE_LATERAL = 68                                  # Giro estándar en intersecciones (°)
        self.TURN_ANGLE_CURVE   = 82                                  # Giro para curvas L obligatorias (°)
        self.TURN_ANGLE_RAD_LAT   = math.radians(self.TURN_ANGLE_LATERAL)
        self.TURN_ANGLE_RAD_CURVE = math.radians(self.TURN_ANGLE_CURVE)

        # ─── Estado del robot ───
        self.scan_data = None
        self.finished  = False
        self.meta_reached = False
        self.pos_x = self.pos_y = self.yaw = 0.0

        # ─── Mapa de calor (celdas visitadas) ───
        self.grid_res = 0.30
        self.visited_cells = defaultdict(int)

        # ─── Grafo topológico (DFS) ───
        self.topological_graph   = []
        self.current_node        = None
        self.returning_to_parent = False
        self.path_letters        = []

        # ─── Máquina de estados ───
        self.mode               = "EXPLORING"
        self.target_yaw         = 0.0
        self.backtrack_after_turn = False

        # ─── Cooldown de detección (anti-rebote) ───
        self.detection_cooldown = False
        self.cooldown_ticks     = 0
        self.cooldown_x = self.cooldown_y = 0.0
        self.in_intersection_zone = False
        self.verify_start_x = self.verify_start_y = 0.0
        self.post_cooldown_armed = False
        self.cooldown_end_x = self.cooldown_end_y = 0.0

        # ─── Control angular suavizado ───
        self.last_angular   = 0.0
        self.max_ang_step   = 0.12
        self.brake_turn_dir = 0.0   # Hysteresis del freno frontal

        # ─── Detección de atasco ───
        self.last_movement_time = self.get_clock().now()
        self.escape_start_x = self.escape_start_y = 0.0
        self.escape_dir_linear  = -1.0
        self.escape_dir_angular = 0.0
        self.stuck_ref_x = self.stuck_ref_y = 0.0
        self.stuck_timer_start = self.get_clock().now()
        self.STUCK_RADIUS = 0.05

        # ─── ROS2: publicadores y subscripciones ───
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, depth=10)
        self.create_subscription(LaserScan, '/scan', self.scan_cb, qos)
        self.create_subscription(Odometry,  '/odom', self.odom_cb, qos)
        self.create_subscription(Bool,      '/meta', self.meta_cb, 10)
        self.create_timer(0.1, self.control_loop)
        self.log_event("Robot iniciado.")

    # ═══════════════════════════════════════════════════════
    # Callbacks y utilidades de sensores
    # ═══════════════════════════════════════════════════════

    def log_event(self, event, extra=""):
        """Registra evento con lecturas LIDAR en consola y fichero."""
        if not self.scan_data: return
        F  = self.savg(0)
        FL = self.savg(45)
        L  = self.savg(90)
        R  = self.savg(270)
        FR = self.savg(315)
        B  = self.savg(180)
        ts = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        line = (f"[{ts}] {event.ljust(50)} | "
                f"F:{F:.2f} FL:{FL:.2f} L:{L:.2f} R:{R:.2f} FR:{FR:.2f} B:{B:.2f}")
        if extra:
            line += f" | {extra}"
        self.get_logger().info(line)
        try:
            with open(self.log_filename, 'a') as f:
                f.write(line + "\n")
        except Exception:
            pass

    def scan_cb(self, msg):
        """Procesa datos LIDAR con rotación virtual de 180° (LIDAR montado al revés)."""
        if not msg.ranges: return
        raw = [
            float('inf') if (r is None or math.isnan(r) or math.isinf(r)
                             or r <= 0 or r < self.CHASSIS_FILTER)
            else min(r, 2.0)
            for r in msg.ranges
        ]
        # Rotación 180°: tras esto, índice 0 = frente del robot
        n = len(raw)
        half = n // 2
        self.scan_data = raw[half:] + raw[:half]

    def odom_cb(self, msg):
        """Actualiza posición y mapa de calor."""
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.pose.orientation)
        self.visited_cells[self.cell(self.pos_x, self.pos_y)] += 1

    def meta_cb(self, msg):
        """Callback del sensor IR de meta."""
        self.meta_reached = bool(msg.data)

    # ─── Funciones de lectura LIDAR ───

    def svals(self, center_angle, width=10):
        """Devuelve las lecturas válidas en un arco centrado en center_angle."""
        if not self.scan_data: return []
        n = len(self.scan_data)
        ca = int(center_angle) % n
        h = width // 2
        return [
            min(self.scan_data[(ca + o) % n], 2.0)
            for o in range(-h, h + 1)
            if self.scan_data[(ca + o) % n] > 0.01
            and not math.isinf(self.scan_data[(ca + o) % n])
        ]

    def savg(self, ca, w=10):
        """Media de lecturas en un arco."""
        v = self.svals(ca, w)
        return sum(v) / len(v) if v else 2.0

    def smin(self, ca, w=10):
        """Mínimo de lecturas en un arco."""
        v = self.svals(ca, w)
        return min(v) if v else 2.0

    def front_metrics(self):
        """Distancia frontal: mínimo del arco 0° (±10°) y diagonales."""
        fmin = min(self.smin(0, 20), self.smin(10, 14), self.smin(350, 14))
        favg = self.savg(0, 28)
        return fmin, favg

    def side_metrics(self):
        """Distancias laterales y diagonales frontales.
        Retorna: (izq_min, der_min, diag_izq_min, diag_der_min)"""
        return (self.smin(90, 24), self.smin(270, 24),
                self.smin(45, 18),  self.smin(315, 18))

    # ─── Comandos de movimiento ───

    def cmd(self, linear, angular):
        m = Twist()
        m.linear.x = float(linear)
        m.angular.z = float(angular)
        self.cmd_pub.publish(m)

    def stop(self):
        self.cmd(0, 0)

    def clamp(self, v, lo, hi):
        return max(lo, min(hi, v))

    def cell(self, x, y):
        return (round(x / self.grid_res), round(y / self.grid_res))

    def smooth_ang(self, target):
        """Suaviza cambios angulares para evitar movimientos bruscos."""
        step = self.clamp(target - self.last_angular, -self.max_ang_step, self.max_ang_step)
        self.last_angular = self.clamp(
            self.last_angular + step, -self.ANGULAR_SPEED, self.ANGULAR_SPEED)
        return self.last_angular

    def activate_cooldown(self):
        """Activa período de gracia tras procesar una intersección."""
        self.detection_cooldown = True
        self.cooldown_ticks = 15   # 1.5s mínimo (anti-rebote)
        self.cooldown_x = self.pos_x
        self.cooldown_y = self.pos_y
        self.brake_turn_dir = 0.0

    # ═══════════════════════════════════════════════════════
    # Giro seguro
    # ═══════════════════════════════════════════════════════

    def execute_safe_turn(self, target_yaw, angular_speed):
        """Rotación pura hacia target_yaw. Retrocede si hay pared muy cerca."""
        diff = angle_diff(target_yaw, self.yaw)
        if abs(diff) < 0.05:
            self.cmd(0, 0)
            return True
        f = self.smin(0, 20)
        linear = -0.01 if f < 0.12 else 0.0
        speed = min(abs(diff) * 1.5, angular_speed)
        if speed < 0.08:
            speed = 0.08
        self.cmd(linear, speed if diff > 0 else -speed)
        return False

    # ═══════════════════════════════════════════════════════
    # Grafo topológico (DFS)
    # ═══════════════════════════════════════════════════════

    NODE_MERGE_DIST = 0.20   # Radio de deduplicación de nodos (metros)

    def log_turn_letter(self, target_angle):
        """Registra la dirección del giro como letra (L/R/S)."""
        d = math.degrees(angle_diff(target_angle, self.yaw))
        self.path_letters.append('L' if d > 45 else 'R' if d < -45 else 'S')

    def simplify_path(self):
        """Simplifica la ruta aplicando reglas de Trémaux (LBL→S, etc.)."""
        reps = {
            "LBL": "S", "RBR": "S", "LBS": "R", "RBS": "L",
            "SBL": "R", "SBR": "L", "LBR": "B", "RBL": "B", "SBS": "B"
        }
        changed = True
        while changed and len(self.path_letters) >= 3:
            changed = False
            seq = "".join(self.path_letters[-3:])
            if seq in reps:
                self.path_letters = self.path_letters[:-3] + list(reps[seq])
                self.log_event(f"Simplificación: {seq} → {reps[seq]}")
                changed = True

    def process_topological_node(self, paths, came_from):
        """Procesa un nodo del grafo: deduplicar, filtrar, elegir ruta DFS."""
        if self.returning_to_parent:
            self.log_event("Regreso a nodo padre completado.")
            self.returning_to_parent = False
            self.path_letters.append('B')
            self.simplify_path()
            node = self.current_node
            if node is None:
                node = {
                    'id': len(self.topological_graph), 'unexplored': paths.copy(),
                    'parent': None, 'came_from': came_from,
                    'going_to': None, 'x': self.pos_x, 'y': self.pos_y
                }
                self.topological_graph.append(node)
                self.current_node = node
                self.log_event("Nodo raíz creado.")
        else:
            # Deduplicación: buscar nodo existente cerca
            existing = None
            for n in self.topological_graph:
                if dist2d(self.pos_x, self.pos_y, n['x'], n['y']) < self.NODE_MERGE_DIST:
                    existing = n
                    break

            if existing:
                node = existing
                self.current_node = node
                self.log_event(f"Nodo #{node['id']} revisitado ({len(node['unexplored'])} pendientes)")
            else:
                node = {
                    'id': len(self.topological_graph), 'unexplored': paths.copy(),
                    'parent': self.current_node, 'came_from': came_from,
                    'going_to': None, 'x': self.pos_x, 'y': self.pos_y
                }
                self.topological_graph.append(node)
                self.current_node = node
                self.log_event(f"Nodo #{node['id']} creado ({len(paths)} caminos)")

        # Nodo agotado → subir al ancestro con caminos pendientes
        if not node['unexplored']:
            self.backtrack_to_ancestor(node, paths)
            return

        # Filtrar la dirección de donde venimos y validar rutas
        valid = []
        for d in node['unexplored']:
            if abs(angle_diff(d, came_from)) < 0.5:
                continue
            for p in paths:
                if abs(angle_diff(d, p)) < 0.8:
                    valid.append(d)
                    break
        if not valid:
            valid = [p for p in paths if abs(angle_diff(p, came_from)) > 0.5]
        if not valid:
            self.log_event("Sin rutas válidas. Subiendo al ancestro.")
            self.backtrack_to_ancestor(node, paths)
            return

        # Elegir la ruta menos visitada
        best_dir, best_visits = valid[0], float('inf')
        for d in valid:
            cx = self.pos_x + 0.35 * math.cos(d)
            cy = self.pos_y + 0.35 * math.sin(d)
            v = self.visited_cells.get(self.cell(cx, cy), 0)
            if v < best_visits:
                best_visits, best_dir = v, d

        # Marcar como explorado
        for j, d in enumerate(node['unexplored']):
            if abs(angle_diff(d, best_dir)) < 0.8:
                node['unexplored'].pop(j)
                break

        node['going_to'] = best_dir
        self.log_turn_letter(best_dir)
        self.simplify_path()
        self.target_yaw = best_dir
        self.mode = "EXECUTING_TURN"
        self.log_event(f"Decisión: giro {math.degrees(angle_diff(best_dir, self.yaw)):.0f}°")

    def backtrack_to_ancestor(self, exhausted_node, paths):
        """DFS: Sube por el árbol hasta encontrar un nodo con caminos pendientes."""
        ancestor = exhausted_node.get('parent') if exhausted_node else None
        while ancestor is not None:
            if len(ancestor['unexplored']) > 0:
                break
            ancestor = ancestor.get('parent')

        if ancestor is not None:
            self.current_node = ancestor
            self.returning_to_parent = True
            n_pending = len(ancestor['unexplored'])
            d = dist2d(self.pos_x, self.pos_y, ancestor['x'], ancestor['y'])
            self.log_event(f"DFS: subiendo a ancestro #{ancestor['id']} "
                           f"({n_pending} pendientes, {d:.2f}m)")
            self.target_yaw = normalize_angle(self.yaw + math.pi)
            self.backtrack_after_turn = True
            self.mode = "TURN_180"
        else:
            # Raíz agotada: todos los caminos explorados
            self.log_event("DFS: grafo completo. Reset + heurística.")
            self.topological_graph = []
            self.current_node = None
            self.returning_to_parent = False
            self.path_letters = []
            best_dir, best_v = paths[0], float('inf')
            for d in paths:
                cx = self.pos_x + 0.35 * math.cos(d)
                cy = self.pos_y + 0.35 * math.sin(d)
                v = self.visited_cells.get(self.cell(cx, cy), 0)
                if v < best_v:
                    best_v, best_dir = v, d
            self.target_yaw = best_dir
            self.mode = "EXECUTING_TURN"

    # ═══════════════════════════════════════════════════════
    # Bucle principal
    # ═══════════════════════════════════════════════════════

    def control_loop(self):
        if not self.scan_data or self.finished:
            return

        # ── Meta alcanzada ──
        if self.meta_reached:
            self.stop()
            self.finished = True
            self.log_event(f"¡META! Ruta: {''.join(self.path_letters)}")
            return

        fmin, _ = self.front_metrics()
        lmin, rmin, flmin, frmin = self.side_metrics()

        # ══════════════════════════════════════════
        # 0. ESCAPING — Maniobra de desbloqueo
        # ══════════════════════════════════════════
        if self.mode == "ESCAPING":
            adv = dist2d(self.pos_x, self.pos_y,
                         self.escape_start_x, self.escape_start_y)
            if adv >= 0.08:
                self.stop()
                self.mode = "EXPLORING"
                self.activate_cooldown()
                self.log_event("Escape completado.")
                self.last_movement_time = self.get_clock().now()
                return

            back = self.savg(180, 20)
            if back < 0.14 or back >= 2.0:
                self.stop()
                self.mode = "EXPLORING"
                self.activate_cooldown()
                self.log_event("Escape abortado: pared trasera.")
                self.last_movement_time = self.get_clock().now()
                return

            self.cmd(self.SLOW_SPEED * self.escape_dir_linear,
                     self.escape_dir_angular)
            return

        # ── Detección de atasco por tiempo ──
        if self.mode in ["EXECUTING_TURN", "TURN_180"] or fmin < self.FRONT_BRAKE:
            t_stuck = (self.get_clock().now() - self.last_movement_time).nanoseconds / 1e9
            limit = 50.0 if self.mode == "TURN_180" else 25.0
            if t_stuck > limit:
                self.stop()
                self.escape_start_x = self.pos_x
                self.escape_start_y = self.pos_y
                if self.backtrack_after_turn:
                    self.backtrack_after_turn = False
                    self.returning_to_parent = True
                    self.log_event("Atasco en 180°. Memoria salvada.")
                self.escape_dir_linear = -1.0
                self.escape_dir_angular = -0.08 if lmin < rmin else 0.08
                self.log_event(f"¡Atascado! ({t_stuck:.1f}s). Escape.")
                self.mode = "ESCAPING"
                return
        else:
            self.last_movement_time = self.get_clock().now()

        # ── Detección de atasco por odometría ──
        if self.mode != "ESCAPING":
            d_moved = dist2d(self.pos_x, self.pos_y,
                             self.stuck_ref_x, self.stuck_ref_y)
            t_in_zone = (self.get_clock().now() - self.stuck_timer_start).nanoseconds / 1e9

            if d_moved > self.STUCK_RADIUS:
                self.stuck_ref_x = self.pos_x
                self.stuck_ref_y = self.pos_y
                self.stuck_timer_start = self.get_clock().now()
            else:
                limit = 40.0 if self.mode == "TURN_180" else 50.0
                if t_in_zone > limit:
                    self.stop()
                    self.escape_start_x = self.pos_x
                    self.escape_start_y = self.pos_y
                    if self.backtrack_after_turn:
                        self.backtrack_after_turn = False
                        self.returning_to_parent = True
                    self.escape_dir_linear = -1.0
                    self.escape_dir_angular = -0.08 if lmin < rmin else 0.08
                    self.log_event(f"¡Atascado físicamente! ({t_in_zone:.1f}s). Escape.")
                    self.mode = "ESCAPING"
                    self.stuck_ref_x = self.pos_x
                    self.stuck_ref_y = self.pos_y
                    self.stuck_timer_start = self.get_clock().now()
                    return

        # ══════════════════════════════════════════
        # 1. APPROACHING_INTERSECTION — Centrado lateral
        # ══════════════════════════════════════════
        if self.mode == "APPROACHING_INTERSECTION":
            if lmin < rmin and lmin < 0.40:
                err = lmin - 0.155
            elif rmin < 0.40:
                err = 0.155 - rmin
            else:
                err = 0.0
            ac = self.clamp(err * 3.5, -0.40, 0.40)

            dist_frontal = self.smin(0, 10)
            adv = dist2d(self.pos_x, self.pos_y,
                         self.verify_start_x, self.verify_start_y)

            if adv < 0.10 and dist_frontal > 0.18:
                self.cmd(self.SLOW_SPEED, ac)
                return

            self.stop()
            self.mode = "ANALYZING_NODE"
            self.log_event(f"Posición lista (F:{dist_frontal:.2f}m). Evaluando cruce...")
            return

        # ══════════════════════════════════════════
        # 2. ANALYZING_NODE — Evaluación del cruce
        # ══════════════════════════════════════════
        if self.mode == "ANALYZING_NODE":
            frente_abierto = self.savg(0, 15) > 0.40
            izq_abierto = self.savg(90, 15) > 0.3 or self.savg(45, 20) > 0.60
            der_abierto = self.savg(270, 15) > 0.3 or self.savg(315, 20) > 0.60

            # Construir lista de caminos detectados (ángulos absolutos)
            caminos = []
            if frente_abierto:
                caminos.append(0.0)
            if izq_abierto:
                caminos.append(self.TURN_ANGLE_RAD_LAT)
            if der_abierto:
                caminos.append(-self.TURN_ANGLE_RAD_LAT)

            fp = [normalize_angle(self.yaw + ang) for ang in caminos]
            cf = normalize_angle(self.yaw + math.pi)  # Dirección de donde venimos

            # Comprobar si estamos en el nodo padre (durante backtracking)
            es_nodo_padre = False
            if self.returning_to_parent and self.current_node is not None:
                d = dist2d(self.pos_x, self.pos_y,
                           self.current_node['x'], self.current_node['y'])
                if d < 0.45:
                    es_nodo_padre = True

            # ── Decisión topológica ──

            if len(fp) == 0:
                # Salvavidas: buscar aperturas con diagonales
                val_L = max(self.savg(90, 20), self.savg(45, 20))
                val_R = max(self.savg(270, 20), self.savg(315, 20))

                if val_L > 0.30 and val_L > (val_R + 0.10):
                    self.log_event(f"Curva rescatada IZQ (L:{val_L:.2f}).")
                    self.target_yaw = normalize_angle(self.yaw + self.TURN_ANGLE_RAD_CURVE)
                    self.mode = "EXECUTING_TURN"
                elif val_R > 0.30 and val_R > (val_L + 0.10):
                    self.log_event(f"Curva rescatada DER (R:{val_R:.2f}).")
                    self.target_yaw = normalize_angle(self.yaw - self.TURN_ANGLE_RAD_CURVE)
                    self.mode = "EXECUTING_TURN"
                elif val_L > 0.30 and val_R > 0.45:
                    self.log_event(f"T ciega rescatada (L:{val_L:.2f}, R:{val_R:.2f}).")
                    if val_L > val_R:
                        self.target_yaw = normalize_angle(self.yaw + self.TURN_ANGLE_RAD_LAT)
                    else:
                        self.target_yaw = normalize_angle(self.yaw - self.TURN_ANGLE_RAD_LAT)
                    self.mode = "EXECUTING_TURN"
                else:
                    self.log_event(f"Callejón sin salida (L:{val_L:.2f}, R:{val_R:.2f}). 180°.")
                    self.target_yaw = normalize_angle(self.yaw + math.pi)
                    self.activate_cooldown()
                    self.mode = "TURN_180"

            elif len(fp) == 1 and not es_nodo_padre:
                if frente_abierto:
                    # Solo frente libre: falsa alarma
                    self.log_event("Falsa alarma (solo frente). Cooldown.")
                    self.mode = "EXPLORING"
                    self.activate_cooldown()
                    self.cooldown_ticks = 20
                else:
                    # Curva L obligatoria: ángulo amplio para no quedarse mirando la pared
                    diff_to_fp = angle_diff(fp[0], self.yaw)
                    if diff_to_fp > 0:
                        curve_target = normalize_angle(self.yaw + self.TURN_ANGLE_RAD_LAT)
                    else:
                        curve_target = normalize_angle(self.yaw - self.TURN_ANGLE_RAD_LAT)
                    self.log_event(f"Curva obligatoria ({self.TURN_ANGLE_LATERAL}°).")
                    self.target_yaw = curve_target
                    self.mode = "EXECUTING_TURN"

            else:
                # Cruce con múltiples vías (o nodo padre camuflado)
                if es_nodo_padre and len(fp) == 1:
                    self.log_event("Nodo padre camuflado como curva.")
                else:
                    self.log_event(f"Cruce ({len(fp)} vías). "
                                   f"F:{frente_abierto} L:{izq_abierto} R:{der_abierto}")

                # Primer cruce tras cul-de-sac sin grafo
                if self.returning_to_parent and self.current_node is None:
                    self.log_event("Primer cruce tras cul-de-sac. Creando mapa.")
                    self.returning_to_parent = False

                # ── Backtracking DFS ──
                if self.returning_to_parent:
                    dist_al_padre = 999.0
                    es_el_padre_real = False
                    if self.current_node:
                        dist_al_padre = dist2d(
                            self.pos_x, self.pos_y,
                            self.current_node['x'], self.current_node['y'])
                        if dist_al_padre < 0.30:
                            es_el_padre_real = True

                    # CASO A: Llegada al nodo objetivo
                    if self.current_node is None or es_el_padre_real:
                        self.log_event("DFS: llegada a nodo objetivo.")
                        self.returning_to_parent = False
                        self.path_letters.append('B')
                        self.simplify_path()

                        if self.current_node and len(self.current_node['unexplored']) > 0:
                            # Buscar la ruta inexplorada que mejor casa con los caminos visibles
                            best_match_diff = float('inf')
                            best_dir = cf
                            best_idx = -1
                            for current_dir in fp:
                                for i, unexp_dir in enumerate(self.current_node['unexplored']):
                                    diff = abs(angle_diff(current_dir, unexp_dir))
                                    if diff < best_match_diff:
                                        best_match_diff = diff
                                        best_dir = current_dir
                                        best_idx = i

                            if best_match_diff < 0.8:
                                self.current_node['unexplored'].pop(best_idx)
                                self.log_event(f"DFS: ruta recuperada. "
                                               f"Quedan {len(self.current_node['unexplored'])}.")
                            else:
                                self.log_event("Desfase orientación. Alternativa forzada.")
                                best_dir = fp[-1] if len(fp) > 1 else fp[0]

                            self.target_yaw = best_dir
                            self.mode = "EXECUTING_TURN"
                            self.log_event(f"DFS: giro "
                                           f"{math.degrees(angle_diff(best_dir, self.yaw)):.0f}°")
                        else:
                            self.log_event("DFS: nodo objetivo agotado. Subiendo más.")
                            self.backtrack_to_ancestor(self.current_node, fp)

                    # CASO B: Nodo intermedio (no es el objetivo)
                    else:
                        self.log_event(f"DFS: cruce intermedio (dist: {dist_al_padre:.2f}m)")
                        intermediate = None
                        for n in self.topological_graph:
                            if dist2d(self.pos_x, self.pos_y, n['x'], n['y']) < 0.25:
                                intermediate = n
                                break

                        if intermediate and len(intermediate['unexplored']) > 0:
                            self.log_event(f"DFS: intermedio #{intermediate['id']} "
                                           f"tiene {len(intermediate['unexplored'])} caminos.")
                            self.returning_to_parent = False
                            self.current_node = intermediate
                            cf_back = normalize_angle(self.yaw + math.pi)
                            self.process_topological_node(fp, cf_back)
                        else:
                            self.log_event("DFS: intermedio agotado. Sigo hacia objetivo.")
                            cf_back = normalize_angle(self.yaw + math.pi)
                            forward = [p for p in fp if abs(angle_diff(p, cf_back)) > 0.5]
                            if forward:
                                best_dir, best_v = forward[0], float('inf')
                                for d in forward:
                                    cx = self.pos_x + 0.35 * math.cos(d)
                                    cy = self.pos_y + 0.35 * math.sin(d)
                                    v = self.visited_cells.get(self.cell(cx, cy), 0)
                                    if v < best_v:
                                        best_v, best_dir = v, d
                                self.target_yaw = best_dir
                                self.mode = "EXECUTING_TURN"
                                self.activate_cooldown()
                                self.cooldown_ticks = 50
                            else:
                                self.mode = "EXPLORING"
                                self.activate_cooldown()
                                self.cooldown_ticks = 30

                # ── Exploración normal (sin backtracking) ──
                else:
                    self.process_topological_node(fp, cf)
                    # Si la decisión es recto (0°), aplicar cooldown largo
                    if (self.mode == "EXECUTING_TURN"
                            and abs(normalize_angle(self.target_yaw - self.yaw)) < 0.10):
                        self.log_event("Decisión: recto (0°). Cooldown.")
                        self.mode = "EXPLORING"
                        self.activate_cooldown()
                        self.cooldown_ticks = 50
                        return
            return

        # ══════════════════════════════════════════
        # 3. EXECUTING_TURN
        # ══════════════════════════════════════════
        if self.mode == "EXECUTING_TURN":
            if self.execute_safe_turn(self.target_yaw, self.ANGULAR_SPEED):
                self.mode = "EXPLORING"
                self.activate_cooldown()
                self.in_intersection_zone = True
                self.log_event("Giro completado.")
            return

        # ══════════════════════════════════════════
        # 4. TURN_180
        # ══════════════════════════════════════════
        if self.mode == "TURN_180":
            if self.execute_safe_turn(self.target_yaw, self.ANGULAR_SPEED * 0.8):
                if self.backtrack_after_turn:
                    self.backtrack_after_turn = False
                    self.returning_to_parent = True
                    self.log_event("180° OK. Buscando nodo padre.")
                self.mode = "EXPLORING"
                self.activate_cooldown()
                self.in_intersection_zone = True
            return

        # ══════════════════════════════════════════
        # 5. EXPLORING — Detección + Wall Following
        # ══════════════════════════════════════════

        # Detección de aperturas laterales (incluye diagonales)
        izq_abierto = self.savg(90, 20) > 0.40 or self.savg(45, 10) > 0.45
        der_abierto = self.savg(270, 20) > 0.40 or self.savg(315, 10) > 0.45
        has_lat = izq_abierto or der_abierto

        frente_estricto = self.smin(0, 10)

        # Reset de zona de intersección si no hay laterales
        if not has_lat:
            self.in_intersection_zone = False

        # ── Cooldown híbrido (ticks + distancia) ──
        if self.detection_cooldown:
            self.cooldown_ticks = max(0, self.cooldown_ticks - 1)
            d_cool = dist2d(self.pos_x, self.pos_y, self.cooldown_x, self.cooldown_y)
            if self.cooldown_ticks <= 0 and d_cool >= 0.15:
                self.detection_cooldown = False
                self.cooldown_end_x = self.pos_x
                self.cooldown_end_y = self.pos_y
                self.post_cooldown_armed = True
                self.log_event(f"Escudo desactivado ({d_cool:.2f}m).")
            else:
                self.in_intersection_zone = True
                # Corrección suave durante cooldown
                if lmin < 0.20 and rmin < 0.20:
                    pass
                elif flmin < 0.20 or lmin < 0.16:
                    deficit = max(0.20 - flmin, 0.16 - lmin)
                    error = deficit * -4.0
                elif frmin < 0.20 or rmin < 0.16:
                    deficit = max(0.20 - frmin, 0.16 - rmin)
                    error = deficit * 4.0

        # Post-cooldown: gracia de 5cm adicional
        post_cooldown_block = False
        if self.post_cooldown_armed:
            d_post = dist2d(self.pos_x, self.pos_y,
                            self.cooldown_end_x, self.cooldown_end_y)
            if d_post >= 0.05:
                self.post_cooldown_armed = False
                self.in_intersection_zone = False
            else:
                post_cooldown_block = True

        # ── Gatillos de detección de intersecciones ──

        detection_ok = (not self.detection_cooldown
                        and not self.in_intersection_zone
                        and not post_cooldown_block)

        # GATILLO 1: Apertura lateral con frente libre (>80cm)
        if has_lat and detection_ok and frente_estricto > 0.80:
            self.stop()
            self.mode = "APPROACHING_INTERSECTION"
            self.verify_start_x, self.verify_start_y = self.pos_x, self.pos_y
            self.in_intersection_zone = True
            self.log_event("Hueco lateral (frente libre). Acercando...")
            return

        # GATILLO 2: Pared frontal (≤20cm) con apertura lateral
        elif (not self.detection_cooldown and not post_cooldown_block
              and frente_estricto <= self.WALL_STOP and has_lat):
            self.stop()
            self.mode = "ANALYZING_NODE"
            self.verify_start_x, self.verify_start_y = self.pos_x, self.pos_y
            self.in_intersection_zone = True
            self.log_event(f"Pared frontal ({frente_estricto:.2f}m). Evaluando...")
            return

        # GATILLO 3: Zona media (solo durante backtracking)
        elif has_lat and detection_ok and self.returning_to_parent:
            self.stop()
            self.mode = "APPROACHING_INTERSECTION"
            self.verify_start_x, self.verify_start_y = self.pos_x, self.pos_y
            self.in_intersection_zone = True
            self.log_event(f"Intersección en vuelta (F:{frente_estricto:.2f}m).")
            return

        # ── Cul-de-sac ──
        back_dist = self.savg(180, 24)
        if (not self.detection_cooldown
                and fmin < 0.30 and lmin < 0.30 and rmin < 0.30
                and back_dist > 0.30):
            has_exit = False
            for sd in range(0, 360, 15):
                if 140 <= sd <= 220:
                    continue   # Ignorar zona trasera
                if self.savg(sd, 14) > 0.30:
                    has_exit = True
                    break
            if not has_exit:
                self.log_event("Cul-de-sac. 180°.")
                self.target_yaw = normalize_angle(self.yaw + math.pi)
                self.backtrack_after_turn = True
                self.mode = "TURN_180"
                return

        # ── Freno frontal ──
        if fmin < self.FRONT_BRAKE:
            # Si hay apertura lateral: redirigir a evaluación de nodo
            if has_lat and not self.detection_cooldown and not post_cooldown_block:
                self.stop()
                self.mode = "ANALYZING_NODE"
                self.verify_start_x, self.verify_start_y = self.pos_x, self.pos_y
                self.in_intersection_zone = True
                self.log_event(f"Intersección rescatada (F:{fmin:.2f}m).")
                return
            # Rotación con hysteresis (solo fuera de zona post-giro)
            if not self.in_intersection_zone:
                if self.brake_turn_dir == 0.0:
                    space_l = max(lmin, flmin)
                    space_r = max(rmin, frmin)
                    self.brake_turn_dir = 1.0 if space_l >= space_r else -1.0
                    self.log_event(f"Freno frontal: "
                                   f"{'IZQ' if self.brake_turn_dir > 0 else 'DER'} "
                                   f"(L:{space_l:.2f} R:{space_r:.2f})")
                self.cmd(0, self.brake_turn_dir * self.ANGULAR_SPEED * 0.6)
                return

        # ══════════════════════════════════════════
        # Wall Following — Control PD
        # ══════════════════════════════════════════

        # Reset hysteresis del freno cuando hay margen
        if fmin > self.FRONT_BRAKE + 0.06:
            self.brake_turn_dir = 0.0

        TARGET_LAT  = 0.155   # Distancia objetivo a pared lateral
        TARGET_DIAG = 0.220   # Distancia objetivo diagonal
        DIAG_CAP    = 0.26    # Cap para evitar volantazos en aperturas

        c_flmin = min(flmin, DIAG_CAP)
        c_frmin = min(frmin, DIAG_CAP)
        prox_izq = min(lmin, c_flmin)
        prox_der = min(rmin, c_frmin)

        GAIN_LAT  = 1.5
        GAIN_DIAG = 0.8
        error = 0.0

        if prox_izq >= 1.90 and prox_der >= 1.90:
            error = 0.0
        elif prox_izq <= prox_der:
            error = ((lmin - TARGET_LAT) * GAIN_LAT
                     + (c_flmin - TARGET_DIAG) * GAIN_DIAG)
        else:
            error = ((TARGET_LAT - rmin) * GAIN_LAT
                     + (TARGET_DIAG - c_frmin) * GAIN_DIAG)

        # Zona muerta anti-zigzag
        if abs(error) < 0.06:
            error = 0.0

        # Corrección durante cooldown (suave)
        if self.detection_cooldown:
            if flmin < 0.20 or lmin < 0.16:
                deficit = max(0.20 - flmin, 0.16 - lmin)
                error = self.clamp(deficit * -1.5, -0.10, 0.10)
            elif frmin < 0.20 or rmin < 0.16:
                deficit = max(0.20 - frmin, 0.16 - rmin)
                error = self.clamp(deficit * 1.5, -0.10, 0.10)
        else:
            # Pánico: demasiado cerca de una pared
            if lmin < 0.115 or flmin < 0.13:
                error = -0.12
            elif rmin < 0.115 or frmin < 0.13:
                error = 0.12

        angular = self.clamp(error, -0.20, 0.20)
        linear = self.LINEAR_SPEED
        if fmin < self.FRONT_SLOW or abs(angular) > 0.12:
            linear = self.SLOW_SPEED

        self.cmd(linear, angular)


# ═══════════════════════════════════════════════════════

def main(args=None):
    rclpy.init(args=args)
    node = LaberintSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()