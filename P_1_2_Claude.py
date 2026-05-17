#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time

# ─── PARÁMETROS AJUSTABLES ────────────────────────────────────────────────────
DIST_OBSTACULO_FRENTE = 0.35   # Distancia mínima al frente antes de girar
DIST_PARED_DERECHA    = 0.30   # Distancia ideal a la pared derecha en modo seguimiento
DIST_PASILLO          = 0.60   # Si derecha Y izquierda < esto → estamos en un pasillo
DIST_ESQUINA_CERRADA  = 0.22   # Si frente+der+izq todos < esto → esquina cerrada

VEL_LINEAR_PASILLO    = 0.07   # Velocidad en pasillo (lenta y segura)
VEL_LINEAR_NORMAL     = 0.10   # Velocidad en espacio abierto
VEL_GIRO_FUERTE       = 0.55   # Velocidad angular al girar por obstáculo
KP                    = 1.5    # Ganancia proporcional seguimiento de pared

TIEMPO_GIRO_MINIMO    = 1.2    # Segundos mínimos girando antes de poder cambiar
# ──────────────────────────────────────────────────────────────────────────────

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver_node')

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry,  '/odom', self.odom_callback, 10)
        self.timer    = self.create_timer(0.1, self.control_loop)

        self.d_front = 3.0
        self.d_right = 3.0
        self.d_left  = 3.0
        self.d_back  = 3.0

        # Estados: 'pasillo' | 'avanzar' | 'girar_izq' | 'girar_der' | 'escape' | 'meta'
        self.estado = 'avanzar'

        self.tiempo_inicio_giro = 0.0
        self.giro_comprometido  = False

        self.META_X               = 2.75
        self.META_Y               = 1.71
        self.DISTANCIA_MINIMA_META = 0.25
        self.meta_alcanzada       = False

    def clean(self, v):
        return 3.0 if (math.isinf(v) or math.isnan(v)) else float(v)

    def sector_min(self, ranges, a, b):
        return min(self.clean(ranges[i]) for i in range(a, b))

    def scan_callback(self, msg):
        r = msg.ranges
        if len(r) < 360:
            return
        self.d_front = min(self.sector_min(r, 350, 360),
                           self.sector_min(r,   0,  10))
        self.d_right = self.sector_min(r, 260, 310)
        self.d_left  = self.sector_min(r,  50, 110)
        self.d_back  = self.sector_min(r, 170, 190)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if math.sqrt((x - self.META_X)**2 + (y - self.META_Y)**2) < self.DISTANCIA_MINIMA_META:
            self.meta_alcanzada = True

    def control_loop(self):
        twist = Twist()

        if self.meta_alcanzada:
            self.get_logger().info('META ALCANZADA! Robot detenido.')
            self.cmd_pub.publish(twist)
            return

        d_f = self.d_front
        d_r = self.d_right
        d_l = self.d_left

        ahora          = time.time()
        tiempo_girando = ahora - self.tiempo_inicio_giro

        # ── DETECCIÓN DE SITUACIÓN ────────────────────────────────────────────

        en_pasillo       = (d_r < DIST_PASILLO and d_l < DIST_PASILLO)
        esquina_cerrada  = (d_f < DIST_ESQUINA_CERRADA and
                            d_r < DIST_ESQUINA_CERRADA + 0.05 and
                            d_l < DIST_ESQUINA_CERRADA + 0.05)

        self.get_logger().info(
            f'F:{d_f:.2f} R:{d_r:.2f} L:{d_l:.2f} '
            f'pasillo:{en_pasillo} [{self.estado}]'
        )

        # ── MÁQUINA DE ESTADOS ────────────────────────────────────────────────

        if esquina_cerrada:
            # Prioridad máxima: salir marcha atrás
            self.estado = 'escape'
            self.giro_comprometido = False

        elif en_pasillo and d_f >= DIST_OBSTACULO_FRENTE:
            # Pasillo despejado al frente: ir recto ignorando laterales
            self.estado = 'pasillo'
            self.giro_comprometido = False

        elif self.estado == 'pasillo':
            # Salimos de modo pasillo si ya no hay paredes a ambos lados
            # o si hay obstáculo al frente
            if not en_pasillo or d_f < DIST_OBSTACULO_FRENTE:
                if d_f < DIST_OBSTACULO_FRENTE:
                    self._iniciar_giro(d_l, d_r, ahora)
                else:
                    self.estado = 'avanzar'

        elif self.estado == 'avanzar':
            if d_f < DIST_OBSTACULO_FRENTE:
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
            # Recto, velocidad reducida, corrección mínima para no rozar paredes
            twist.linear.x  = VEL_LINEAR_PASILLO
            twist.angular.z = 0.0  # sin corrección lateral: las paredes están a ambos lados

        elif self.estado == 'escape':
            twist.linear.x  = -0.07
            twist.angular.z =  VEL_GIRO_FUERTE

        elif self.estado == 'girar_izq':
            twist.linear.x  = 0.0
            twist.angular.z = VEL_GIRO_FUERTE

        elif self.estado == 'girar_der':
            twist.linear.x  = 0.0
            twist.angular.z = -VEL_GIRO_FUERTE

        else:  # avanzar: seguimiento de pared derecha
            twist.linear.x = VEL_LINEAR_NORMAL
            if d_r > 1.5:
                twist.angular.z = -0.25  # buscar pared derecha suavemente
            else:
                error = DIST_PARED_DERECHA - d_r
                twist.angular.z = max(min(KP * error, 0.45), -0.45)

        self.cmd_pub.publish(twist)

    def _iniciar_giro(self, d_l, d_r, ahora):
        """Decide el lado y marca el inicio del giro comprometido."""
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
