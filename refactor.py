#!/usr/bin/env python3

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

def extract_yaw_from_quat(q):
    y_val = 2.0 * (q.w * q.z + q.x * q.y)
    x_val = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(y_val, x_val)

def calc_angle_diff(a1, a2):
    return math.atan2(math.sin(a1 - a2), math.cos(a1 - a2))

def wrap_angle_rads(angle):
    return math.atan2(math.sin(angle), math.cos(angle))

def euclidean_dist(x_a, y_a, x_b, y_b):
    return math.sqrt((x_a - x_b)**2 + (y_a - y_b)**2)


class MazeNavigatorNode(Node):
    def __init__(self):
        super().__init__('maze_navigator')
        
        stamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.file_log = f"nav_record_{stamp}.txt"
        with open(self.file_log, 'w') as doc:
            doc.write("=== INICIO DE REGISTRO ===\n")

        self.vel_fwd = 0.02
        self.vel_fwd_slow = 0.01
        self.vel_rot_max = 0.25
        self.dist_brake_front = 0.12
        self.dist_slow_front = 0.25
        self.filter_chassis_dist = 0.05

        self.dist_wall_stop = 0.20
        self.deg_turn = 55
        self.rad_turn = math.radians(self.deg_turn)

        self.time_last_move = self.get_clock().now()
        self.evade_start_x = 0.0
        self.evade_start_y = 0.0
        self.evade_linear = -1.0
        self.evade_angular = 0.0

        self.lidar_readings = None
        self.is_completed = False
        self.goal_achieved = False
        self.curr_x = 0.0
        self.curr_y = 0.0
        self.curr_yaw = 0.0
        self.map_resolution = 0.30
        self.heat_map = defaultdict(int)

        self.node_network = []
        self.active_node = None
        self.reverting_path = False
        self.route_history = []
        self.gap_threshold = 0.40

        self.nav_state = "STATE_NAVIGATE"
        self.desired_yaw = 0.0
        self.force_backtrack = False

        self.is_cooling_down = False
        self.cd_origin_x = 0.0
        self.cd_origin_y = 0.0
        self.dist_cooldown = 0.10
        self.inside_junction = False
        self.check_x = 0.0
        self.check_y = 0.0

        self.prev_angular_vel = 0.0
        self.ang_accel_limit = 0.12
        self.best_heading = 0.0

        self.jam_ref_x = 0.0
        self.jam_ref_y = 0.0
        self.jam_timer = self.get_clock().now()
        self.jam_radius = 0.05

        self.publisher_cmd = self.create_publisher(Twist, '/cmd_vel', 10)
        custom_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST, 
            depth=10
        )
        self.create_subscription(LaserScan, '/scan', self.callback_lidar, custom_qos)
        self.create_subscription(Odometry, '/odom', self.callback_odom, custom_qos)
        self.create_subscription(Bool, '/meta', self.callback_goal, 10)
        self.create_timer(0.1, self.routine_update)
        
        self.record_event("SISTEMA ARRANCADO. Configuración inicial cargada.")

    def record_event(self, msg, detail=""):
        if not self.lidar_readings: 
            return
        
        f_val, fl_val, l_val = self.get_mean(0), self.get_mean(45), self.get_mean(90)
        r_val, fr_val, b_val = self.get_mean(270), self.get_mean(315), self.get_mean(180)
        
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        log_line = f"[{timestamp}] {msg.ljust(45)} | F:{f_val:.2f} FL:{fl_val:.2f} L:{l_val:.2f} R:{r_val:.2f} FR:{fr_val:.2f} B:{b_val:.2f}"
        if detail: 
            log_line += f" | {detail}"
            
        self.get_logger().info(log_line)
        try:
            with open(self.file_log, 'a') as doc: 
                doc.write(log_line + "\n")
        except Exception: 
            pass

    def callback_lidar(self, msg):
        if not msg.ranges: 
            return
        
        cleaned = []
        for r in msg.ranges:
            if r is None or math.isnan(r) or math.isinf(r) or r <= 0 or r < self.filter_chassis_dist:
                cleaned.append(float('inf'))
            else:
                cleaned.append(min(r, 2.0))
        self.lidar_readings = cleaned

    def callback_odom(self, msg):
        pose = msg.pose.pose
        self.curr_x = pose.position.x
        self.curr_y = pose.position.y
        self.curr_yaw = extract_yaw_from_quat(pose.orientation)
        grid_coord = self.get_grid_coord(self.curr_x, self.curr_y)
        self.heat_map[grid_coord] += 1

    def callback_goal(self, msg): 
        self.goal_achieved = bool(msg.data)

    def fetch_slice(self, angle_deg, size=10):
        if not self.lidar_readings: 
            return []
        
        total_rays = len(self.lidar_readings)
        center = int(angle_deg) % total_rays
        margin = size // 2
        
        valid_rays = []
        for offset in range(-margin, margin + 1):
            idx = (center + offset) % total_rays
            ray = self.lidar_readings[idx]
            if ray > 0.01 and not math.isinf(ray):
                valid_rays.append(min(ray, 2.0))
        return valid_rays

    def get_mean(self, angle_deg, size=10):
        rays = self.fetch_slice(angle_deg, size)
        return sum(rays) / len(rays) if rays else 2.0

    def get_min(self, angle_deg, size=10):
        rays = self.fetch_slice(angle_deg, size)
        return min(rays) if rays else 2.0

    def eval_front(self):
        min_front = min(self.get_min(0, 20), self.get_min(10, 14), self.get_min(350, 14))
        avg_front = self.get_mean(0, 28)
        return min_front, avg_front

    def eval_sides(self):
        return (self.get_min(90, 24), self.get_min(270, 24), 
                self.get_min(45, 18), self.get_min(315, 18))

    def send_vel(self, v_lin, v_ang):
        t = Twist()
        t.linear.x = float(v_lin)
        t.angular.z = float(v_ang)
        self.publisher_cmd.publish(t)

    def halt_robot(self): 
        self.send_vel(0.0, 0.0)

    def restrict(self, val, bottom, top): 
        return max(bottom, min(top, val))

    def get_grid_coord(self, px, py): 
        return (round(px / self.map_resolution), round(py / self.map_resolution))

    def trigger_cooldown(self):
        self.is_cooling_down = True
        self.cd_origin_x = self.curr_x
        self.cd_origin_y = self.curr_y

    def perform_rotation(self, target, max_spin):
        err = calc_angle_diff(target, self.curr_yaw)
        if abs(err) < 0.05:
            self.halt_robot()
            return True
            
        spin = min(abs(err) * 1.5, max_spin)
        if spin < 0.08: 
            spin = 0.08
            
        final_spin = spin if err > 0 else -spin
        self.send_vel(0, final_spin)
        return False

    def register_turn_char(self, target_a):
        deg_diff = math.degrees(calc_angle_diff(target_a, self.curr_yaw))
        if deg_diff > 45: char = 'L'
        elif deg_diff < -45: char = 'R'
        else: char = 'S'
        self.route_history.append(char)

    def compress_history(self):
        modded = True
        while modded and len(self.route_history) >= 3:
            modded = False
            tail = "".join(self.route_history[-3:])
            patterns = {
                "LBL":"S", "RBR":"S", "LBS":"R", "RBS":"L", 
                "SBL":"R", "SBR":"L", "LBR":"B", "RBL":"B", "SBS":"B"
            }
            if tail in patterns:
                self.route_history = self.route_history[:-3] + list(patterns[tail])
                self.record_event(f"COMPRESIÓN DE RUTA: {tail} -> {patterns[tail]}")
                modded = True

    def manage_junction(self, available_paths, origin_path):
        if self.reverting_path:
            self.record_event("Retorno a Intersección Previa finalizado.")
            self.reverting_path = False
            self.route_history.append('B')
            self.compress_history()
            
            if self.active_node is None:
                self.record_event("Generando Nodo Primario.")
                new_node = {
                    'idx': len(self.node_network), 'pending': available_paths.copy(),
                    'parent': None, 'origin': origin_path, 'dest': None,
                    'px': self.curr_x, 'py': self.curr_y
                }
                self.node_network.append(new_node)
                self.active_node = new_node
        else:
            merge_thresh = 0.15
            found_node = None
            for n in self.node_network:
                if euclidean_dist(self.curr_x, self.curr_y, n['px'], n['py']) < merge_thresh:
                    found_node = n
                    break

            if found_node:
                self.active_node = found_node
                self.record_event(f"NODO #{found_node['idx']} revisitado ({len(found_node['pending'])} ramas libres)")
            else:
                new_node = {
                    'idx': len(self.node_network), 'pending': available_paths.copy(),
                    'parent': self.active_node, 'origin': origin_path, 'dest': None,
                    'px': self.curr_x, 'py': self.curr_y
                }
                self.node_network.append(new_node)
                self.active_node = new_node
                self.record_event(f"NODO #{new_node['idx']} registrado ({len(available_paths)} vías)")

        if not self.active_node['pending']:
            self.record_event("Intersección sin salida. Reseteando mapa local.")
            self.node_network.clear()
            self.active_node = None
            self.reverting_path = False
            self.route_history.clear()
            
            opt_dir, min_visits = available_paths[0], float('inf')
            for p_dir in available_paths:
                cx = self.curr_x + 0.35 * math.cos(p_dir)
                cy = self.curr_y + 0.35 * math.sin(p_dir)
                visits = self.heat_map.get(self.get_grid_coord(cx, cy), 0)
                if visits < min_visits: 
                    min_visits, opt_dir = visits, p_dir
                    
            self.desired_yaw = opt_dir
            self.nav_state = "STATE_ROTATE"
            return

        valid_dirs = []
        for pd in self.active_node['pending']:
            if abs(calc_angle_diff(pd, origin_path)) < 0.5:
                continue
            for ap in available_paths:
                if abs(calc_angle_diff(pd, ap)) < 0.8:
                    valid_dirs.append(pd)
                    break
        
        if not valid_dirs:
            valid_dirs = [ap for ap in available_paths if abs(calc_angle_diff(ap, origin_path)) > 0.5]
        
        if not valid_dirs:
            self.record_event("Caminos inválidos detectados. Limpiando memoria.")
            self.node_network.clear()
            self.active_node = None
            self.reverting_path = False
            self.route_history.clear()
            
            opt_dir, min_visits = available_paths[0], float('inf')
            for p_dir in available_paths:
                cx = self.curr_x + 0.35 * math.cos(p_dir)
                cy = self.curr_y + 0.35 * math.sin(p_dir)
                visits = self.heat_map.get(self.get_grid_coord(cx, cy), 0)
                if visits < min_visits: 
                    min_visits, opt_dir = visits, p_dir
                    
            self.desired_yaw = opt_dir
            self.nav_state = "STATE_ROTATE"
            return

        opt_dir, min_visits, opt_idx = valid_dirs[0], float('inf'), 0
        for idx, d_val in enumerate(valid_dirs):
            cx = self.curr_x + 0.35 * math.cos(d_val)
            cy = self.curr_y + 0.35 * math.sin(d_val)
            visits = self.heat_map.get(self.get_grid_coord(cx, cy), 0)
            if visits < min_visits: 
                min_visits, opt_dir, opt_idx = visits, d_val, idx
        
        for k, pd in enumerate(self.active_node['pending']):
            if abs(calc_angle_diff(pd, opt_dir)) < 0.8:
                self.active_node['pending'].pop(k)
                break
        
        self.active_node['dest'] = opt_dir
        self.register_turn_char(opt_dir)
        self.compress_history()
        
        self.desired_yaw = opt_dir
        self.nav_state = "STATE_ROTATE"
        turn_deg = math.degrees(calc_angle_diff(opt_dir, self.curr_yaw))
        self.record_event(f"Resolución de Nodo: Viraje de {turn_deg:.0f}°")

    def routine_update(self):
        if not self.lidar_readings or self.is_completed: 
            return
            
        if self.goal_achieved:
            self.halt_robot()
            self.is_completed = True
            self.record_event(f"¡OBJETIVO ALCANZADO! Secuencia: {''.join(self.route_history)}")
            return

        f_min, f_avg = self.eval_front()
        l_min, r_min, fl_min, fr_min = self.eval_sides()

        if self.nav_state == "STATE_UNSTUCK":
            dist_escaped = euclidean_dist(self.curr_x, self.curr_y, self.evade_start_x, self.evade_start_y)
            if dist_escaped >= 0.08:
                self.halt_robot()
                self.nav_state = "STATE_NAVIGATE"
                self.trigger_cooldown()
                self.record_event("Desatasco finalizado. Retomando operaciones.")
                self.time_last_move = self.get_clock().now()
                return
            
            rear_space = self.get_mean(180, 20)
            if rear_space < 0.14 or rear_space >= 2:
                self.halt_robot()
                self.nav_state = "STATE_NAVIGATE"
                self.trigger_cooldown()
                self.record_event("Precaución: Obstáculo trasero. Cancelando evasión.")
                self.time_last_move = self.get_clock().now()
                return
            
            self.send_vel(self.vel_fwd_slow * self.evade_linear, self.evade_angular)
            return

        is_turning = self.nav_state in ["STATE_ROTATE", "STATE_U_TURN"]
        if is_turning or f_min < self.dist_brake_front:
            time_stuck = (self.get_clock().now() - self.time_last_move).nanoseconds / 1e9
            max_wait = 50.0 if self.nav_state == "STATE_U_TURN" else 25.0
            
            if time_stuck > max_wait: 
                self.halt_robot()
                self.evade_start_x = self.curr_x
                self.evade_start_y = self.curr_y
                
                if self.force_backtrack:
                    self.force_backtrack = False
                    self.reverting_path = True
                    self.record_event("Pánico en curva cerrada. Restaurando historial para retorno.")
                
                self.evade_linear = -1.0 
                self.evade_angular = -0.08 if l_min < r_min else 0.08
                    
                self.record_event(f"ALERTA JAM ({time_stuck:.1f}s). Iniciando maniobra de marcha atrás.")
                self.nav_state = "STATE_UNSTUCK"
                return
        else:
            self.time_last_move = self.get_clock().now()

        if self.nav_state != "STATE_UNSTUCK":
            spatial_dist = euclidean_dist(self.curr_x, self.curr_y, self.jam_ref_x, self.jam_ref_y)
            spatial_time = (self.get_clock().now() - self.jam_timer).nanoseconds / 1e9
            
            if spatial_dist > self.jam_radius:
                self.jam_ref_x, self.jam_ref_y = self.curr_x, self.curr_y
                self.jam_timer = self.get_clock().now()
            else:
                max_spatial_wait = 40.0 if self.nav_state == "STATE_U_TURN" else 50.0 
                
                if spatial_time > max_spatial_wait:
                    self.halt_robot()
                    self.evade_start_x = self.curr_x
                    self.evade_start_y = self.curr_y
                    
                    if self.force_backtrack:
                        self.force_backtrack = False
                        self.reverting_path = True
                        self.record_event("Pánico espacial. Retornando.")
                    
                    self.evade_linear = -1.0 
                    self.evade_angular = -0.08 if l_min < r_min else 0.08
                        
                    self.record_event(f"ALERTA ESPACIAL ({spatial_time:.1f}s en 5cm). Iniciando evasión.")
                    self.nav_state = "STATE_UNSTUCK"
                    
                    self.jam_ref_x, self.jam_ref_y = self.curr_x, self.curr_y
                    self.jam_timer = self.get_clock().now()
                    return

        if self.nav_state == "STATE_APPROACH_JUNCTION":
            shift_err = 0.0
            if l_min < r_min and l_min < 0.40: shift_err = l_min - 0.155
            elif r_min < 0.40: shift_err = 0.155 - r_min
                
            correction = self.restrict(shift_err * 3.5, -0.40, 0.40)
            
            front_gap = self.get_min(0, 10)
            progress = euclidean_dist(self.curr_x, self.curr_y, self.check_x, self.check_y)

            if progress < 0.04 and front_gap > 0.18:
                self.send_vel(self.vel_fwd_slow, correction)
                return

            self.halt_robot()
            self.nav_state = "STATE_EVALUATE_JUNCTION"
            self.record_event(f"Posicionado para escanear (Frente libre: {front_gap:.2f}m).")
            return

        if self.nav_state == "STATE_EVALUATE_JUNCTION":
            
            gap_fwd = self.get_mean(0, 15) > 0.29
            gap_left = self.get_mean(90, 15) > 0.39 or self.get_mean(45, 30) > 0.49
            gap_right = self.get_mean(270, 15) > 0.39 or self.get_mean(315, 30) > 0.49

            rel_paths = []
            if gap_fwd: rel_paths.append(0.0)
            if gap_left: rel_paths.append(math.radians(55))
            if gap_right: rel_paths.append(math.radians(-55))

            abs_paths = [wrap_angle_rads(self.curr_yaw + r) for r in rel_paths]
            backward_path = wrap_angle_rads(self.curr_yaw + math.pi)

            is_parent = False
            if self.reverting_path and self.active_node is not None:
                dist_parent = euclidean_dist(self.curr_x, self.curr_y, self.active_node['px'], self.active_node['py'])
                if dist_parent < 0.45:
                    is_parent = True

            if len(abs_paths) == 0:
                val_izq = max(self.get_mean(90, 20), self.get_mean(45, 20))
                val_der = max(self.get_mean(270, 20), self.get_mean(315, 20))
                
                if val_izq > 0.30 and val_izq > (val_der + 0.10):
                    self.record_event(f"Recuperación por hueco izquierdo ({val_izq:.2f}).")
                    self.desired_yaw = wrap_angle_rads(self.curr_yaw + self.rad_turn)
                    self.nav_state = "STATE_ROTATE"
                elif val_der > 0.30 and val_der > (val_izq + 0.10):
                    self.record_event(f"Recuperación por hueco derecho ({val_der:.2f}).")
                    self.desired_yaw = wrap_angle_rads(self.curr_yaw - self.rad_turn)
                    self.nav_state = "STATE_ROTATE"
                elif val_izq > 0.30 and val_der > 0.45:
                    self.record_event("Cruce en T oculto. Ejecutando giro forzado.")
                    turn_dir = self.rad_turn if val_izq > val_der else -self.rad_turn
                    self.desired_yaw = wrap_angle_rads(self.curr_yaw + turn_dir)
                    self.nav_state = "STATE_ROTATE"
                else:
                    self.record_event("Bucle cerrado. Ejecutando maniobra de 180°.")
                    self.desired_yaw = backward_path
                    self.trigger_cooldown()
                    self.nav_state = "STATE_U_TURN"

            elif len(abs_paths) == 1 and not is_parent:
                if abs_paths[0] == 0.0:
                    self.record_event("Falso positivo frontal. Ignorando.")
                    self.nav_state = "STATE_NAVIGATE"
                    self.trigger_cooldown()
                else:
                    self.record_event("Curva identificada. Procediendo al giro.")
                    self.desired_yaw = abs_paths[0]
                    self.nav_state = "STATE_ROTATE"
                    
            else:
                if is_parent and len(abs_paths) == 1:
                    self.record_event("Nodo principal solapado.")
                else:
                    self.record_event(f"Intersección ({len(abs_paths)} opciones). F:{gap_fwd} L:{gap_left} R:{gap_right}")

                if self.reverting_path and self.active_node is None:
                    if len(self.node_network) > 0:
                        self.record_event("Mapeo agotado. Purga de datos iniciada.")
                        self.node_network.clear()
                        self.active_node = None
                        self.reverting_path = False
                        self.route_history.clear()
                        self.nav_state = "STATE_NAVIGATE"
                        self.trigger_cooldown()
                        return
                    else:
                        self.record_event("Registro del primer nodo post-retorno.")
                        self.reverting_path = False

                if self.reverting_path:
                    is_true_parent = False
                    if self.active_node:
                        dist = euclidean_dist(self.curr_x, self.curr_y, self.active_node['px'], self.active_node['py'])
                        if dist < 0.25:
                            is_true_parent = True
                            
                    if self.active_node is None or is_true_parent:
                        self.record_event("Retorno a base confirmado.")
                        self.reverting_path = False
                        self.route_history.append('B')
                        self.compress_history()
                        
                        if self.active_node and len(self.active_node['pending']) > 0:
                            closest_diff = float('inf')
                            opt_dir = backward_path
                            opt_idx = -1
                            
                            for a_path in abs_paths:
                                for idx, p_dir in enumerate(self.active_node['pending']):
                                    diff = abs(calc_angle_diff(a_path, p_dir))
                                    if diff < closest_diff:
                                        closest_diff, opt_dir, opt_idx = diff, a_path, idx
                                        
                            if closest_diff < 0.8: 
                                self.active_node['pending'].pop(opt_idx)
                                self.record_event("Restaurando camino no visitado.")
                            else:
                                self.record_event("Desviación detectada. Selección forzada.")
                                opt_dir = abs_paths[-1] if len(abs_paths) > 1 else abs_paths[0]
                                
                            self.desired_yaw = opt_dir
                            self.nav_state = "STATE_ROTATE"
                        else:
                            self.record_event("Vías consumidas. Reset total del nodo.")
                            self.node_network.clear()
                            self.active_node = None
                            self.reverting_path = False
                            self.route_history.clear()
                            
                            opt_dir, min_visits = abs_paths[0], float('inf')
                            for d_val in abs_paths:
                                cx = self.curr_x + 0.35 * math.cos(d_val)
                                cy = self.curr_y + 0.35 * math.sin(d_val)
                                visits = self.heat_map.get(self.get_grid_coord(cx, cy), 0)
                                if visits < min_visits: 
                                    min_visits, opt_dir = visits, d_val
                                    
                            self.desired_yaw = opt_dir
                            self.nav_state = "STATE_ROTATE"

                    else:
                        self.record_event("Nodo fantasma hallado durante el retorno. Asimilando.")
                        self.reverting_path = False
                        origin_virtual = self.curr_yaw
                        self.manage_junction(abs_paths, origin_virtual)
                        
                        if self.nav_state == "STATE_ROTATE" and abs(wrap_angle_rads(self.desired_yaw - self.curr_yaw)) < 0.10:
                            self.record_event("Mantenimiento de rumbo. Levantando escudo virtual.")
                            self.nav_state = "STATE_NAVIGATE"
                            self.trigger_cooldown() 
                else:
                    self.manage_junction(abs_paths, backward_path)
                    
                    if self.nav_state == "STATE_ROTATE" and abs(wrap_angle_rads(self.desired_yaw - self.curr_yaw)) < 0.10:
                        self.record_event("Anulación de giro frontal. Escudo activo.")
                        self.nav_state = "STATE_NAVIGATE"
                        self.trigger_cooldown()
                        return

        if self.nav_state == "STATE_ROTATE":
            if self.perform_rotation(self.desired_yaw, self.vel_rot_max):
                self.nav_state = "STATE_NAVIGATE"
                self.trigger_cooldown()
                self.inside_junction = True
                self.record_event("Viraje finalizado correctamente.")
            return

        if self.nav_state == "STATE_U_TURN":
            if self.perform_rotation(self.desired_yaw, self.vel_rot_max * 0.8):
                if self.force_backtrack:
                    self.force_backtrack = False
                    self.reverting_path = True
                    self.record_event("Giro de 180 completado. Buscando matriz parental.")
                self.nav_state = "STATE_NAVIGATE"
                self.trigger_cooldown()
                self.inside_junction = True
            return

        if l_min < 0.30 and r_min < 0.30:
            self.inside_junction = False

        mov_gap_l = self.get_mean(90, 20) > 0.35 or self.get_mean(45, 20) > 0.45
        mov_gap_r = self.get_mean(270, 20) > 0.35 or self.get_mean(315, 20) > 0.45
        wall_breach = mov_gap_l or mov_gap_r

        if abs(self.prev_angular_vel) > 0.15: 
            wall_breach = False
        
        if self.is_cooling_down:
            dist_to_origin = euclidean_dist(self.curr_x, self.curr_y, self.cd_origin_x, self.cd_origin_y)
            
            if dist_to_origin >= self.dist_cooldown:
                self.is_cooling_down = False
                self.inside_junction = False 
                self.get_logger().info("COOLDOWN INACTIVO: Zona despejada.")
            else: 
                wall_breach = False
                self.inside_junction = True
                
                if l_min >= 0.20 or r_min >= 0.20:
                    if fl_min < 0.20 or l_min < 0.16:
                        self.send_vel(self.vel_fwd, max(0.20 - fl_min, 0.16 - l_min) * -4.0)
                        return
                    elif fr_min < 0.20 or r_min < 0.16:
                        self.send_vel(self.vel_fwd, max(0.20 - fr_min, 0.16 - r_min) * 4.0)
                        return

        if wall_breach and not self.is_cooling_down and not self.inside_junction and f_min > 0.60:
            self.halt_robot()
            self.nav_state = "STATE_EVALUATE_JUNCTION" 
            self.check_x, self.check_y = self.curr_x, self.curr_y
            self.inside_junction = True
            self.record_event("Corte en pared detectado. Parada táctica.")
            return

        elif not self.is_cooling_down and f_min <= self.dist_wall_stop and wall_breach:
            self.halt_robot()
            self.nav_state = "STATE_EVALUATE_JUNCTION"
            self.check_x, self.check_y = self.curr_x, self.curr_y
            self.inside_junction = True
            self.record_event(f"Obstáculo inminente a {f_min:.2f}m. Iniciando análisis.")
            return

        space_rear = self.get_mean(180, 24)
        if f_min < 0.20 and l_min < 0.20 and r_min < 0.20 and space_rear > 0.30:
            escape_route = False
            for deg_ray in range(0, 360, 15):
                if 140 <= deg_ray <= 220: continue
                if self.get_mean(deg_ray, 14) > 0.30: 
                    escape_route = True
                    break
            if not escape_route:
                self.record_event("ZONA ESTANCADA (Cul-de-sac). Retirada.")
                self.desired_yaw = wrap_angle_rads(self.curr_yaw + math.pi)
                self.force_backtrack = True
                self.nav_state = "STATE_U_TURN"
                return

        if f_min < self.dist_brake_front:
            rot_mod = 1.0 if l_min > r_min else -1.0
            self.send_vel(0, rot_mod * self.vel_rot_max * 0.6)
            return

        offset_target = 0.155        
        offset_diag = 0.220   
        viz_limit = 0.32   
        
        cap_fl = min(fl_min, 0.26)
        cap_fr = min(fr_min, 0.26)
        
        correction_factor = 0.0

        if l_min < viz_limit and r_min < viz_limit:
            if l_min < r_min:
                correction_factor = (l_min - offset_target) * 3.0 + (cap_fl - offset_diag) * 1.5
            else:
                correction_factor = (offset_target - r_min) * 3.0 + (offset_diag - cap_fr) * 1.5
        elif l_min < viz_limit:
            correction_factor = (l_min - offset_target) * 3.0 + (cap_fl - offset_diag) * 1.5
        elif r_min < viz_limit:
            correction_factor = (offset_target - r_min) * 3.0 + (offset_diag - cap_fr) * 1.5

        if l_min < 0.115 or fl_min < 0.13:
            correction_factor = -0.6  
        elif r_min < 0.115 or fr_min < 0.13:
            correction_factor = 0.6   

        apply_angular = self.restrict(correction_factor, -0.45, 0.45)
        apply_linear = self.vel_fwd
        
        if f_min < self.dist_slow_front or abs(apply_angular) > 0.20:
            apply_linear = self.vel_fwd_slow

        self.send_vel(apply_linear, apply_angular)

def main(args=None):
    rclpy.init(args=args)
    core_node = MazeNavigatorNode()
    try: 
        rclpy.spin(core_node)
    except KeyboardInterrupt: 
        pass
    finally:
        if rclpy.ok(): 
            core_node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()