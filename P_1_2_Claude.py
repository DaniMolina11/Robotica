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
DIST_PARED_DERECHA    = 0.30   # Distancia ideal a la pared derecha
VEL_LINEAR            = 0.12   # Velocidad de avance
VEL_GIRO_FUERTE       = 0.65   # Velocidad angular al girar por obstáculo
KP                    = 2.0    # Ganancia proporcional seguimiento de pared

TIEMPO_GIRO_MINIMO    = 1.5    # Segundos mínimos girando antes de poder cambiar
DIST_ESQUINA_CERRADA  = 0.20   # Si todo está a menos de esto → esquina cerrada
# ──────────────────────────────────────────────────────────────────────────────

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver_node')

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry,  '/odom', self.odom_callback, 10)
        self.timer    = self.create_timer(0.1, self.control_loop)

        # Distancias por zona
        self.d_front = 3.0
        self.d_right = 3.0
        self.d_left  = 3.0
        self.d_back  = 3.0

        # Máquina de estados
        # Estados: 'avanzar' | 'girar_izq' | 'girar_der' | 'escape' | 'meta'
        self.estado = 'avanzar'

        # Temporizador de giro: evita cambiar de decisión antes de tiempo
        self.tiempo_inicio_giro = 0.0
        self.giro_comprometido  = False   # True mientras estamos en giro mínimo obligatorio

        # Meta
        self.META_X               = 2.75
        self.META_Y               = 1.71
        self.DISTANCIA_MINIMA_META = 0.25
        self.meta_alcanzada       = False

    # ── Utilidad ──────────────────────────────────────────────────────────────
    def clean(self, v):
        return 3.0 if (math.isinf(v) or math.isnan(v)) else float(v)

    def sector_min(self, ranges, a, b):
        return min(self.clean(ranges[i]) for i in range(a, b))

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def scan_callback(self, msg):
        r = msg.ranges
        if len(r) < 360:
            return

        self.d_front = min(self.sector_min(r, 350, 360),
                           self.sector_min(r,   0,  10))   # ±10° al frente
        self.d_right = self.sector_min(r, 260, 310)         # derecha
        self.d_left  = self.sector_min(r,  50, 110)         # izquierda
        self.d_back  = self.sector_min(r, 170, 190)         # detrás

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

        d_f = self.d_front
        d_r = self.d_right
        d_l = self.d_left
        d_b = self.d_back

        ahora = time.time()
        tiempo_girando = ahora - self.tiempo_inicio_giro

        self.get_logger().info(
            f'F:{d_f:.2f} R:{d_r:.2f} L:{d_l:.2f} B:{d_b:.2f} [{self.estado}] '
            f'giro:{tiempo_girando:.1f}s'
        )

        # ── DETECCIÓN DE ESQUINA CERRADA ──────────────────────────────────────
        # Si frente, derecha e izquierda están todos bloqueados → escapar marcha atrás
        if (d_f < DIST_ESQUINA_CERRADA and
            d_r < DIST_ESQUINA_CERRADA + 0.05 and
            d_l < DIST_ESQUINA_CERRADA + 0.05):
            self.estado = 'escape'
            self.giro_comprometido = False

        # ── TRANSICIONES DE ESTADO ────────────────────────────────────────────
        elif self.estado == 'avanzar':
            if d_f < DIST_OBSTACULO_FRENTE:
                # Elegir lado con más espacio y comprometerse
                if d_l >= d_r:
                    self.estado = 'girar_izq'
                else:
                    self.estado = 'girar_der'
                self.tiempo_inicio_giro = ahora
                self.giro_comprometido  = True

        elif self.estado in ('girar_izq', 'girar_der'):
            if self.giro_comprometido:
                # Estamos en el tiempo mínimo obligatorio: no cambiar aunque el frente se despeje
                if tiempo_girando >= TIEMPO_GIRO_MINIMO:
                    self.giro_comprometido = False

            else:
                # Tiempo mínimo cumplido: podemos salir del giro si el frente está libre
                if d_f >= DIST_OBSTACULO_FRENTE + 0.10:
                    self.estado = 'avanzar'
                # Si sigue bloqueado, volvemos a comprometernos con otro giro
                elif d_f < DIST_OBSTACULO_FRENTE:
                    if d_l >= d_r:
                        self.estado = 'girar_izq'
                    else:
                        self.estado = 'girar_der'
                    self.tiempo_inicio_giro = ahora
                    self.giro_comprometido  = True

        elif self.estado == 'escape':
            # Salir del escape cuando haya espacio al frente o detrás
            if d_f > DIST_OBSTACULO_FRENTE + 0.10 or d_b > 0.5:
                self.estado = 'avanzar'

        # ── ACCIONES POR ESTADO ───────────────────────────────────────────────

        if self.estado == 'escape':
            # Marcha atrás girando para salir de la esquina
            twist.linear.x  = -0.08
            twist.angular.z =  VEL_GIRO_FUERTE  # gira mientras retrocede

        elif self.estado == 'girar_izq':
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
                twist.angular.z = max(min(KP * error, 0.5), -0.5)

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
