#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time
from collections import deque

# ─── PARÁMETROS AJUSTABLES ────────────────────────────────────────────────────
DIST_OBSTACULO_FRENTE = 0.35   # Distancia mínima al frente antes de girar
DIST_PARED_DERECHA    = 0.25   # Distancia ideal a la pared derecha
DIST_PASILLO          = 0.45   # Der Y Izq < esto → pasillo (pasillo real ~0.305m + margen)
DIST_ESQUINA_CERRADA  = 0.22   # Frente+der+izq todos < esto → esquina cerrada

VEL_LINEAR_PASILLO    = 0.06   # Velocidad en pasillo
VEL_LINEAR_NORMAL     = 0.08   # Velocidad en espacio abierto
VEL_GIRO_FUERTE       = 0.40   # Velocidad angular al girar (reducida para no volcarse)
KP                    = 1.2    # Ganancia proporcional seguimiento de pared

TIEMPO_GIRO_MINIMO    = 1.2    # Segundos mínimos girando antes de poder cambiar

# Cuántas lecturas LIDAR promediar antes de actuar (más = más estable, más lento)
N_LECTURAS_PROMEDIO   = 5
# ──────────────────────────────────────────────────────────────────────────────

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver_node')

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry,  '/odom', self.odom_callback, 10)
        self.timer    = self.create_timer(0.1, self.control_loop)

        # Buffers de historial para promediar (deque descarta automáticamente los viejos)
        self.buf_front = deque(maxlen=N_LECTURAS_PROMEDIO)
        self.buf_right = deque(maxlen=N_LECTURAS_PROMEDIO)
        self.buf_left  = deque(maxlen=N_LECTURAS_PROMEDIO)
        self.buf_back  = deque(maxlen=N_LECTURAS_PROMEDIO)

        # Valores suavizados (lo que usa la lógica)
        self.d_front = 3.0
        self.d_right = 3.0
        self.d_left  = 3.0
        self.d_back  = 3.0

        # Cuántas lecturas tenemos ya (no actuar hasta tener N_LECTURAS_PROMEDIO)
        self.lecturas_acumuladas = 0

        # Estados: 'esperando' | 'pasillo' | 'avanzar' | 'girar_izq' | 'girar_der' | 'escape'
        self.estado = 'esperando'

        self.tiempo_inicio_giro = 0.0
        self.giro_comprometido  = False

        # Detección de fin de pasillo: contar ticks consecutivos fuera de pasillo
        # antes de cambiar de estado (evita reaccionar a una lectura puntual)
        self.ticks_fuera_pasillo = 0
        self.TICKS_CONFIRMACION  = 4   # ticks consecutivos para confirmar que salimos del pasillo

        self.META_X               = 2.75
        self.META_Y               = 1.71
        self.DISTANCIA_MINIMA_META = 0.25
        self.meta_alcanzada       = False

    # ── Utilidad ──────────────────────────────────────────────────────────────
    def clean(self, v):
        return 3.0 if (math.isinf(v) or math.isnan(v)) else float(v)

    def sector_min(self, ranges, a, b):
        return min(self.clean(ranges[i]) for i in range(a, b))

    def promedio(self, buf):
        return sum(buf) / len(buf) if buf else 3.0

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def scan_callback(self, msg):
        r = msg.ranges
        if len(r) < 360:
            return

        # Leer mínimos de cada sector
        raw_front = min(self.sector_min(r, 350, 360),
                        self.sector_min(r,   0,  10))
        raw_right = self.sector_min(r, 260, 310)
        raw_left  = self.sector_min(r,  50, 110)
        raw_back  = self.sector_min(r, 170, 190)

        # Añadir al buffer histórico
        self.buf_front.append(raw_front)
        self.buf_right.append(raw_right)
        self.buf_left.append(raw_left)
        self.buf_back.append(raw_back)

        self.lecturas_acumuladas += 1

        # Calcular valores suavizados (promedio de las últimas N lecturas)
        self.d_front = self.promedio(self.buf_front)
        self.d_right = self.promedio(self.buf_right)
        self.d_left  = self.promedio(self.buf_left)
        self.d_back  = self.promedio(self.buf_back)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if math.sqrt((x - self.META_X)**2 + (y - self.META_Y)**2) < self.DISTANCIA_MINIMA_META:
            self.meta_alcanzada = True

    # ── Lógica principal ──────────────────────────────────────────────────────
    def control_loop(self):
        twist = Twist()

        if self.meta_alcanzada:
            self.get_logger().info('META ALCANZADA! Robot detenido.')
            self.cmd_pub.publish(twist)
            return

        # Esperar a tener suficientes lecturas antes de moverse
        if self.lecturas_acumuladas < N_LECTURAS_PROMEDIO:
            self.get_logger().info(
                f'Acumulando lecturas... {self.lecturas_acumuladas}/{N_LECTURAS_PROMEDIO}'
            )
            self.cmd_pub.publish(twist)  # quieto
            return

        if self.estado == 'esperando':
            self.estado = 'avanzar'

        d_f = self.d_front
        d_r = self.d_right
        d_l = self.d_left

        ahora          = time.time()
        tiempo_girando = ahora - self.tiempo_inicio_giro

        en_pasillo      = (d_r < DIST_PASILLO and d_l < DIST_PASILLO)
        esquina_cerrada = (d_f < DIST_ESQUINA_CERRADA and
                           d_r < DIST_ESQUINA_CERRADA + 0.05 and
                           d_l < DIST_ESQUINA_CERRADA + 0.05)

        self.get_logger().info(
            f'F:{d_f:.2f} R:{d_r:.2f} L:{d_l:.2f} '
            f'pasillo:{en_pasillo} [{self.estado}] giro:{tiempo_girando:.1f}s'
        )

        # ── MÁQUINA DE ESTADOS ────────────────────────────────────────────────

        if esquina_cerrada:
            self.estado = 'escape'
            self.giro_comprometido = False

        elif self.estado == 'pasillo':
            if en_pasillo and d_f >= DIST_OBSTACULO_FRENTE:
                # Seguimos en pasillo normal
                self.ticks_fuera_pasillo = 0

            elif not en_pasillo:
                # Posible salida de pasillo: confirmar con varios ticks consecutivos
                self.ticks_fuera_pasillo += 1
                if self.ticks_fuera_pasillo >= self.TICKS_CONFIRMACION:
                    self.get_logger().info('Salida de pasillo confirmada -> avanzar')
                    self.estado = 'avanzar'
                    self.ticks_fuera_pasillo = 0
                # Si aún no confirmamos, seguimos en pasillo (ignoramos la lectura puntual)

            elif d_f < DIST_OBSTACULO_FRENTE:
                # Obstáculo al fondo del pasillo
                self._iniciar_giro(d_l, d_r, ahora)

        elif self.estado == 'avanzar':
            if en_pasillo and d_f >= DIST_OBSTACULO_FRENTE:
                self.get_logger().info('Pasillo detectado -> modo pasillo')
                self.estado = 'pasillo'
                self.ticks_fuera_pasillo = 0
            elif d_f < DIST_OBSTACULO_FRENTE:
                self._iniciar_giro(d_l, d_r, ahora)

        elif self.estado in ('girar_izq', 'girar_der'):
            if self.giro_comprometido:
                if tiempo_girando >= TIEMPO_GIRO_MINIMO:
                    self.giro_comprometido = False
            else:
                if d_f >= DIST_OBSTACULO_FRENTE + 0.10:
                    self.estado = 'avanzar'
                elif d_f < DIST_OBSTACULO_FRENTE:
                    self._iniciar_giro(d_l, d_r, ahora)

        elif self.estado == 'escape':
            if d_f > DIST_OBSTACULO_FRENTE + 0.10:
                self.estado = 'avanzar'

        # ── ACCIONES ──────────────────────────────────────────────────────────

        if self.estado == 'pasillo':
            twist.linear.x  = VEL_LINEAR_PASILLO
            twist.angular.z = 0.0

        elif self.estado == 'escape':
            twist.linear.x  = -0.05
            twist.angular.z =  VEL_GIRO_FUERTE

        elif self.estado == 'girar_izq':
            twist.linear.x  = 0.0
            twist.angular.z = VEL_GIRO_FUERTE

        elif self.estado == 'girar_der':
            twist.linear.x  = 0.0
            twist.angular.z = -VEL_GIRO_FUERTE

        else:  # avanzar
            twist.linear.x = VEL_LINEAR_NORMAL
            if d_r > 1.2:
                twist.angular.z = -0.20  # buscar pared derecha suavemente
            else:
                error = DIST_PARED_DERECHA - d_r
                twist.angular.z = max(min(KP * error, 0.40), -0.40)

        self.cmd_pub.publish(twist)

    def _iniciar_giro(self, d_l, d_r, ahora):
        self.estado = 'girar_izq' if d_l >= d_r else 'girar_der'
        self.tiempo_inicio_giro = ahora
        self.giro_comprometido  = True


def main(args=None):
    rclpy.init(args=args)
    nodo = MazeSolver()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.cmd_pub.publish(Twist())
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
