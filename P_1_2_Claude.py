#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

# ─── PARÁMETROS AJUSTABLES ────────────────────────────────────────────────────
DIST_OBSTACULO_FRENTE = 0.40  # Distancia mínima al frente antes de girar
DIST_PARED_DERECHA    = 0.35  # Distancia ideal a la pared derecha
VEL_LINEAR            = 0.12  # Velocidad de avance
VEL_GIRO_FUERTE       = 0.65  # Velocidad angular al girar por obstáculo
KP                    = 2.0   # Ganancia proporcional (cuanto corrige al desviarse)
# ──────────────────────────────────────────────────────────────────────────────

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver_node')

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry,  '/odom', self.odom_callback, 10)
        self.timer    = self.create_timer(0.1, self.control_loop)

        # Distancias por zona
        self.d_front       = 3.0
        self.d_front_right = 3.0  # zona diagonal delantera-derecha
        self.d_right       = 3.0
        self.d_left        = 3.0

        # Estado de la maquina: 'avanzar' | 'girar_izq' | 'girar_der' | 'meta'
        self.estado = 'avanzar'

        # Meta
        self.META_X               = 2.75
        self.META_Y               = 1.71
        self.DISTANCIA_MINIMA_META = 0.25
        self.meta_alcanzada       = False

    # ── Utilidad ──────────────────────────────────────────────────────────────
    def clean(self, v):
        return 3.0 if (math.isinf(v) or math.isnan(v)) else float(v)

    def sector_min(self, ranges, a, b):
        """Minimo limpio en el sector [a, b) del array ranges."""
        return min(self.clean(ranges[i]) for i in range(a, b))

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def scan_callback(self, msg):
        r = msg.ranges
        if len(r) < 360:
            return

        # Sectores mas estrechos -> decisiones mas precisas
        self.d_front       = min(self.sector_min(r, 350, 360),
                                 self.sector_min(r,   0,  10))  # +-10 al frente
        self.d_front_right = self.sector_min(r, 310, 350)        # diagonal derecha
        self.d_right       = self.sector_min(r, 260, 310)        # derecha
        self.d_left        = self.sector_min(r,  50, 110)        # izquierda

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if math.sqrt((x - self.META_X)**2 + (y - self.META_Y)**2) < self.DISTANCIA_MINIMA_META:
            self.meta_alcanzada = True

    # ── Logica principal ──────────────────────────────────────────────────────
    def control_loop(self):
        twist = Twist()

        # ── META ──────────────────────────────────────────────────────────────
        if self.meta_alcanzada:
            self.get_logger().info('META ALCANZADA! Robot detenido.')
            self.cmd_pub.publish(twist)
            return

        d_f  = self.d_front
        d_fr = self.d_front_right
        d_r  = self.d_right
        d_l  = self.d_left

        self.get_logger().info(
            f'F:{d_f:.2f}  FR:{d_fr:.2f}  R:{d_r:.2f}  L:{d_l:.2f}  [{self.estado}]'
        )

        # ── MAQUINA DE ESTADOS ────────────────────────────────────────────────

        if d_f < DIST_OBSTACULO_FRENTE:
            # Obstaculo al frente: decidir hacia que lado girar
            if d_l > d_r:
                self.estado = 'girar_izq'   # mas espacio a la izquierda
            else:
                self.estado = 'girar_der'   # mas espacio a la derecha

        elif self.estado in ('girar_izq', 'girar_der'):
            # Seguimos girando hasta que el frente quede despejado (con histeresis)
            if d_f >= DIST_OBSTACULO_FRENTE + 0.10:
                self.estado = 'avanzar'

        else:
            self.estado = 'avanzar'

        # ── ACCIONES POR ESTADO ───────────────────────────────────────────────

        if self.estado == 'girar_izq':
            twist.linear.x  = 0.0
            twist.angular.z = VEL_GIRO_FUERTE

        elif self.estado == 'girar_der':
            twist.linear.x  = 0.0
            twist.angular.z = -VEL_GIRO_FUERTE

        else:  # avanzar con seguimiento de pared derecha
            twist.linear.x = VEL_LINEAR

            if d_r > 1.5:
                # Sin pared a la derecha: girar suavemente a la derecha para buscarla
                twist.angular.z = -0.3
            else:
                # Control proporcional: mantener distancia ideal a la pared derecha
                error = DIST_PARED_DERECHA - d_r
                twist.angular.z = KP * error
                twist.angular.z = max(min(twist.angular.z, 0.5), -0.5)

        self.cmd_pub.publish(twist)


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
