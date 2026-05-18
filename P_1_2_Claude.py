#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time
from collections import deque

# --- PARAMETROS ORIGINALES CALIBRADOS ---
DIST_PARAR_GIRO        = 0.35  # Distancia de seguridad al frente
DIST_PARED_DERECHA     = 0.24  # Distancia ideal a la pared derecha
VEL_LINEAR_NORMAL     = 0.08  # Velocidad de avance segura
VEL_GIRO              = 0.32  # Velocidad de rotacion limpia
KP                    = 1.5   # Fuerza del control proporcional

N_LECTURAS_PROMEDIO   = 5
LOG_FILE = '/home/ros/Escriptori/Robotica/maze_log.txt'


class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver_node')

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry,  '/odom', self.odom_callback, 10)
        self.timer    = self.create_timer(0.1, self.control_loop)

        self.buf_front    = deque(maxlen=N_LECTURAS_PROMEDIO)
        self.buf_right    = deque(maxlen=N_LECTURAS_PROMEDIO)
        self.buf_left     = deque(maxlen=N_LECTURAS_PROMEDIO)
        self.buf_back     = deque(maxlen=N_LECTURAS_PROMEDIO)
        self.buf_diag_izq = deque(maxlen=N_LECTURAS_PROMEDIO)
        self.buf_diag_der = deque(maxlen=N_LECTURAS_PROMEDIO)

        self.d_front    = 3.0
        self.d_right    = 3.0
        self.d_left     = 3.0
        self.d_back     = 3.0
        self.d_diag_izq = 3.0
        self.d_diag_der = 3.0

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.lecturas_acumuladas = 0
        self.estado = 'avanzar'

        self.META_X               = 2.75
        self.META_Y               = 1.71
        self.DISTANCIA_MINIMA_META = 0.25
        self.meta_alcanzada       = False

        self.sim_time    = 0.0
        self.vel_lin_pub = 0.0
        self.vel_ang_pub = 0.0

        self.log_file = open(LOG_FILE, 'w')
        self._log_raw('=== INICIO SESION MAZE SOLVER ===')
        self._log_raw('TSIM      | POS_X  | POS_Y  | ESTADO       | F     | R     | L     | VL      | VA      | EVENTO')
        self._log_raw('-' * 110)

    def _log_raw(self, msg):
        ts = time.strftime('%H:%M:%S')
        self.log_file.write(f'[{ts}] {msg}\n')
        self.log_file.flush()

    def _log_tick(self, evento=''):
        self._log_raw(
            f'{self.sim_time:9.2f} | {self.pos_x:+6.3f} | {self.pos_y:+6.3f} | '
            f'{self.estado:<13}| '
            f'{self.d_front:5.2f} | {self.d_right:5.2f} | {self.d_left:5.2f} | '
            f'{self.vel_lin_pub:+7.3f} | {self.vel_ang_pub:+7.3f} | {evento}'
        )

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
        self.buf_front.append(min(self.sector_min(r, 345, 360), self.sector_min(r, 0, 15)))
        self.buf_right.append(self.sector_min(r, 250, 300))
        self.buf_left.append(self.sector_min(r, 60, 110))
        self.buf_back.append(self.sector_min(r, 170, 190))
        self.buf_diag_izq.append(self.sector_min(r, 30, 60))
        self.buf_diag_der.append(self.sector_min(r, 300, 330))
        
        self.lecturas_acumuladas += 1
        self.d_front    = self.promedio(self.buf_front)
        self.d_right    = self.promedio(self.buf_right)
        self.d_left     = self.promedio(self.buf_left)
        self.d_back     = self.promedio(self.buf_back)
        self.d_diag_izq = self.promedio(self.buf_diag_izq)
        self.d_diag_der = self.promedio(self.buf_diag_der)
        self.sim_time   = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def odom_callback(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        dist = math.sqrt((self.pos_x - self.META_X)**2 + (self.pos_y - self.META_Y)**2)
        if dist < self.DISTANCIA_MINIMA_META and not self.meta_alcanzada:
            self.meta_alcanzada = True

    def control_loop(self):
        twist = Twist()

        if self.meta_alcanzada:
            self._log_tick('META - detenido')
            self.cmd_pub.publish(twist)
            return

        if self.lecturas_acumuladas < N_LECTURAS_PROMEDIO:
            self._log_tick('acumulando lecturas')
            self.cmd_pub.publish(twist)
            return

        d_f = self.d_front
        d_r = self.d_right

        # --- MAQUINA DE DECISIONES OPTIMA ---
        if d_f < DIST_PARAR_GIRO:
            # CASO 1: OBSTACULO AL FRENTE (Esquina o Callejón sin salida)
            # El robot se para por completo (X=0.0) y rota sobre su propio eje.
            # Al no avanzar, es imposible que se choque o vuelque.
            self.estado = 'rotar_izquierda'
            twist.linear.x = 0.0
            twist.angular.z = VEL_GIRO
            evento = 'FRENTE BLOQUEADO - Rotando en el sitio'
        else:
            # CASO 2: CAMINO LIBRE AL FRENTE (Navegacion y seguimiento)
            self.estado = 'avanzar'
            twist.linear.x = VEL_LINEAR_NORMAL
            
            if d_r > 0.55:
                # Si la pared derecha se aleja mucho, significa que hay una salida/cruce a la derecha.
                # Giramos a la derecha para meternos por ese nuevo camino.
                twist.angular.z = -VEL_GIRO
                evento = 'SALIDA DETECTADA - Girando a la derecha'
            else:
                # Si vamos por el pasillo normal, aplicamos control proporcional suave
                # para mantenernos perfectamente paralelos a la pared derecha.
                error = DIST_PARED_DERECHA - d_r
                twist.angular.z = max(min(KP * error, 0.45), -0.45)
                evento = f'Siguiendo pared derecha - Error: {error:.3f}'

        self.vel_lin_pub = twist.linear.x
        self.vel_ang_pub = twist.angular.z
        self._log_tick(evento)
        self.cmd_pub.publish(twist)

    def __del__(self):
        try:
            self.log_file.close()
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    nodo = MazeSolver()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.cmd_pub.publish(Twist())
        nodo.log_file.close()
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()