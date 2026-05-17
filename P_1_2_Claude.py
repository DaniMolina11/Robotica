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
DIST_PARAR_GIRO       = 0.28   # Frente < esto → parar y girar (zona de peligro)
DIST_FRENAR           = 0.55   # Frente < esto → empezar a frenar progresivamente
DIST_PARED_DERECHA    = 0.25   # Distancia ideal a la pared derecha
DIST_PASILLO          = 0.45   # Der Y Izq < esto → pasillo
DIST_ESQUINA_CERRADA  = 0.22   # Frente+der+izq todos < esto → escape

VEL_LINEAR_PASILLO    = 0.06   # Velocidad en pasillo
VEL_LINEAR_NORMAL     = 0.08   # Velocidad en espacio abierto
VEL_GIRO              = 0.38   # Velocidad angular al girar
KP                    = 1.2    # Ganancia proporcional seguimiento de pared

TIEMPO_GIRO_MINIMO    = 1.2    # Segundos mínimos girando antes de poder cambiar
N_LECTURAS_PROMEDIO   = 5      # Lecturas LIDAR a promediar
TICKS_CONFIRMACION    = 4      # Ticks para confirmar salida de pasillo
# ──────────────────────────────────────────────────────────────────────────────

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver_node')

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry,  '/odom', self.odom_callback, 10)
        self.timer    = self.create_timer(0.1, self.control_loop)

        self.buf_front = deque(maxlen=N_LECTURAS_PROMEDIO)
        self.buf_right = deque(maxlen=N_LECTURAS_PROMEDIO)
        self.buf_left  = deque(maxlen=N_LECTURAS_PROMEDIO)
        self.buf_back  = deque(maxlen=N_LECTURAS_PROMEDIO)

        self.d_front = 3.0
        self.d_right = 3.0
        self.d_left  = 3.0
        self.d_back  = 3.0

        self.lecturas_acumuladas = 0

        # Estados: 'esperando'|'pasillo'|'avanzar'|'frenando'|'girar_izq'|'girar_der'|'escape'
        self.estado = 'esperando'

        self.tiempo_inicio_giro  = 0.0
        self.giro_comprometido   = False
        self.ticks_fuera_pasillo = 0

        self.META_X               = 2.75
        self.META_Y               = 1.71
        self.DISTANCIA_MINIMA_META = 0.25
        self.meta_alcanzada       = False

    def clean(self, v):
        return 3.0 if (math.isinf(v) or math.isnan(v)) else float(v)

    def sector_min(self, ranges, a, b):
        return min(self.clean(ranges[i]) for i in range(a, b))

    def promedio(self, buf):
        return sum(buf) / len(buf) if buf else 3.0

    def scan_callback(self, msg):
        r = msg.ranges
        if len(r) < 360:
            return
        self.buf_front.append(min(self.sector_min(r, 350, 360),
                                  self.sector_min(r,   0,  10)))
        self.buf_right.append(self.sector_min(r, 260, 310))
        self.buf_left.append( self.sector_min(r,  50, 110))
        self.buf_back.append( self.sector_min(r, 170, 190))
        self.lecturas_acumuladas += 1
        self.d_front = self.promedio(self.buf_front)
        self.d_right = self.promedio(self.buf_right)
        self.d_left  = self.promedio(self.buf_left)
        self.d_back  = self.promedio(self.buf_back)

    def odom_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if math.sqrt((x - self.META_X)**2 + (y - self.META_Y)**2) < self.DISTANCIA_MINIMA_META:
            self.meta_alcanzada = True

    def velocidad_frenada(self, d_front, vel_max):
        """
        Devuelve una velocidad reducida proporcionalmente a la distancia al frente.
        Entre DIST_FRENAR y DIST_PARAR_GIRO la velocidad baja de vel_max a 0.
        """
        if d_front >= DIST_FRENAR:
            return vel_max
        if d_front <= DIST_PARAR_GIRO:
            return 0.0
        # Interpolación lineal entre los dos umbrales
        ratio = (d_front - DIST_PARAR_GIRO) / (DIST_FRENAR - DIST_PARAR_GIRO)
        return round(vel_max * ratio, 3)

    def control_loop(self):
        twist = Twist()

        if self.meta_alcanzada:
            self.get_logger().info('META ALCANZADA! Robot detenido.')
            self.cmd_pub.publish(twist)
            return

        if self.lecturas_acumuladas < N_LECTURAS_PROMEDIO:
            self.get_logger().info(
                f'Acumulando lecturas {self.lecturas_acumuladas}/{N_LECTURAS_PROMEDIO}')
            self.cmd_pub.publish(twist)
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
            if en_pasillo:
                self.ticks_fuera_pasillo = 0
                if d_f < DIST_PARAR_GIRO:
                    # Obstáculo al fondo del pasillo: parar y girar
                    self._iniciar_giro(d_l, d_r, ahora)
                # Si DIST_FRENAR > d_f > DIST_PARAR_GIRO: la frenada progresiva
                # se aplica en la sección de acciones, seguimos en 'pasillo'
            else:
                self.ticks_fuera_pasillo += 1
                if self.ticks_fuera_pasillo >= TICKS_CONFIRMACION:
                    self.get_logger().info('Salida de pasillo confirmada -> avanzar')
                    self.estado = 'avanzar'
                    self.ticks_fuera_pasillo = 0

        elif self.estado == 'avanzar':
            if en_pasillo and d_f >= DIST_PARAR_GIRO:
                self.get_logger().info('Pasillo detectado -> modo pasillo')
                self.estado = 'pasillo'
                self.ticks_fuera_pasillo = 0
            elif d_f < DIST_PARAR_GIRO:
                self._iniciar_giro(d_l, d_r, ahora)
            # Si DIST_FRENAR > d_f > DIST_PARAR_GIRO: frenar pero seguir en 'avanzar'

        elif self.estado in ('girar_izq', 'girar_der'):
            if self.giro_comprometido:
                if tiempo_girando >= TIEMPO_GIRO_MINIMO:
                    self.giro_comprometido = False
            else:
                if d_f >= DIST_PARAR_GIRO + 0.10:
                    self.estado = 'avanzar'
                elif d_f < DIST_PARAR_GIRO:
                    self._iniciar_giro(d_l, d_r, ahora)

        elif self.estado == 'escape':
            if d_f > DIST_PARAR_GIRO + 0.15:
                self.estado = 'avanzar'

        # ── ACCIONES ──────────────────────────────────────────────────────────

        if self.estado == 'pasillo':
            # Velocidad reducida progresivamente si el frente se acerca
            vel = self.velocidad_frenada(d_f, VEL_LINEAR_PASILLO)
            twist.linear.x  = vel
            twist.angular.z = 0.0

        elif self.estado == 'escape':
            twist.linear.x  = -0.05
            twist.angular.z =  VEL_GIRO

        elif self.estado == 'girar_izq':
            twist.linear.x  = 0.0
            twist.angular.z = VEL_GIRO

        elif self.estado == 'girar_der':
            twist.linear.x  = 0.0
            twist.angular.z = -VEL_GIRO

        else:  # avanzar
            # Frenar progresivamente si el frente se acerca
            vel = self.velocidad_frenada(d_f, VEL_LINEAR_NORMAL)
            twist.linear.x = vel
            if d_r > 1.2:
                twist.angular.z = -0.20
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
