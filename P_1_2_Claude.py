#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time
from collections import deque

# --- PARÁMETROS DE TU CÓDIGO ORIGINAL ---
DIST_GIRO_PASILLO      = 0.32   
DIST_PARAR_GIRO        = 0.32
DIST_FRENAR            = 0.55   
DIST_PARED_DERECHA     = 0.25   
DIST_PASILLO           = 0.45   
DIST_ESQUINA_CERRADA   = 0.20   
DIST_SEGURIDAD_TRASERA = 0.15   

VEL_LINEAR_PASILLO    = 0.06
VEL_LINEAR_NORMAL     = 0.08
VEL_RETROCESO         = 0.05
VEL_GIRO              = 0.28   
VEL_AVANCE_GIRO       = 0.06   
KP                    = 1.2

TIEMPO_GIRO_MINIMO    = 1.5
N_LECTURAS_PROMEDIO   = 5
TICKS_CONFIRMACION    = 4

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

        self.estado          = 'esperando'
        self.estado_anterior = 'esperando'

        self.tiempo_inicio_giro  = 0.0
        self.giro_comprometido   = False
        self.ticks_fuera_pasillo = 0

        self.META_X               = 2.75
        self.META_Y               = 1.71
        self.DISTANCIA_MINIMA_META = 0.25
        self.meta_alcanzada       = False

        self.sim_time    = 0.0
        self.vel_lin_pub = 0.0
        self.vel_ang_pub = 0.0

        self.log_file = open(LOG_FILE, 'w')

    def _log_tick(self, evento=''):
        ts = time.strftime('%H:%M:%S')
        self.log_file.write(f'[{ts}] {self.sim_time:.2f} | {self.estado} | {evento}\n')
        self.log_file.flush()

    def clean(self, v):
        return 3.0 if (math.isinf(v) or math.isnan(v)) else float(v)

    def sector_min(self, ranges, a, b):
        return min(self.clean(ranges[i]) for i in range(a, b))

    def promedio(self, buf):
        return sum(buf) / len(buf) if buf else 3.0

    def scan_callback(self, msg):
        r = msg.ranges
        if len(r) < 360: return
        self.buf_front.append(min(self.sector_min(r, 350, 360), self.sector_min(r, 0, 10)))
        self.buf_right.append(self.sector_min(r, 260, 310))
        self.buf_left.append(self.sector_min(r, 50, 110))
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

    def _cambiar_estado(self, nuevo, motivo=''):
        if nuevo != self.estado:
            self.estado = nuevo

    def _decidir_lado_giro(self):
        en_pasillo = (self.d_right < DIST_PASILLO and self.d_left < DIST_PASILLO)
        return 'izq' if (not en_pasillo and self.d_left >= self.d_right) or (en_pasillo and self.d_diag_izq >= self.d_diag_der) else 'der'

    def _iniciar_giro(self, ahora):
        lado = self._decidir_lado_giro()
        self.estado = f'girar_{lado}'
        self.tiempo_inicio_giro = ahora
        self.giro_comprometido  = True

    def control_loop(self):
        twist = Twist()
        ahora = time.time()
        en_pasillo = (self.d_right < DIST_PASILLO and self.d_left < DIST_PASILLO)

        if self.estado == 'esperando':
            self.estado = 'avanzar'

        elif self.estado == 'avanzar':
            if self.d_front < DIST_PARAR_GIRO:
                self._iniciar_giro(ahora)
            else:
                twist.linear.x = VEL_LINEAR_NORMAL
                error = DIST_PARED_DERECHA - self.d_right
                twist.angular.z = max(min(KP * error, 0.40), -0.40)

        elif self.estado in ('girar_izq', 'girar_der'):
            # --- TUS GIROS ORIGINALES (INTACTOS) ---
            twist.linear.x  = VEL_AVANCE_GIRO if self.d_front > 0.22 else 0.0
            twist.angular.z = VEL_GIRO if self.estado == 'girar_izq' else -VEL_GIRO
            
            # --- SALVAVIDAS: Si termina el giro y sigue cerrado = Callejón ---
            if (ahora - self.tiempo_inicio_giro) > TIEMPO_GIRO_MINIMO:
                if self.d_front < 0.25 and self.d_left < 0.35 and self.d_right < 0.35:
                    self.estado = 'giro_180'
                    self.tiempo_inicio_giro = ahora
                elif self.d_front > DIST_PARAR_GIRO:
                    self.estado = 'avanzar'

        elif self.estado == 'giro_180':
            # Giro sobre propio eje (Peonza)
            twist.linear.x = 0.0
            twist.angular.z = VEL_GIRO
            # Si tras 3.5s girando detecta salida, vuelve a avanzar
            if (ahora - self.tiempo_inicio_giro) > 3.5:
                self.estado = 'avanzar'

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    nodo = MazeSolver()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()