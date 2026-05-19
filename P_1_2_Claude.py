#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time
from collections import deque

# --- PARÁMETROS ORIGINALES ---
DIST_GIRO_PASILLO      = 0.32
DIST_PARAR_GIRO        = 0.32
DIST_FRENAR            = 0.55
DIST_PARED_DERECHA     = 0.25
DIST_PASILLO           = 0.45
DIST_SEGURIDAD_TRASERA = 0.15

VEL_LINEAR_PASILLO = 0.06
VEL_LINEAR_NORMAL  = 0.08
VEL_GIRO           = 0.28
VEL_AVANCE_GIRO    = 0.06
KP                 = 1.2

TIEMPO_GIRO_MINIMO  = 1.5
N_LECTURAS_PROMEDIO = 5
TICKS_CONFIRMACION  = 4

LOG_FILE = '/home/ros/Escriptori/Robotica/maze_log.txt'

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver_node')
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.timer    = self.create_timer(0.1, self.control_loop)

        self.buf_front = deque(maxlen=N_LECTURAS_PROMEDIO)
        self.buf_right = deque(maxlen=N_LECTURAS_PROMEDIO)
        self.buf_left  = deque(maxlen=N_LECTURAS_PROMEDIO)

        self.d_front = 3.0
        self.d_right = 3.0
        self.d_left  = 3.0
        
        self.estado = 'esperando'
        self.tiempo_inicio_giro = 0.0
        self.lecturas_acumuladas = 0
        self.log_file = open(LOG_FILE, 'w')

    def scan_callback(self, msg):
        r = msg.ranges
        if len(r) < 360: return
        self.buf_front.append(min(min(r[350:360]), min(r[0:10])))
        self.buf_right.append(min(r[260:310]))
        self.buf_left.append(min(r[50:110]))
        self.lecturas_acumuladas += 1
        self.d_front = sum(self.buf_front)/len(self.buf_front)
        self.d_right = sum(self.buf_right)/len(self.buf_right)
        self.d_left  = sum(self.buf_left)/len(self.buf_left)

    def _iniciar_giro(self, ahora):
        lado = 'izq' if self.d_left >= self.d_right else 'der'
        self.estado = f'girar_{lado}'
        self.tiempo_inicio_giro = ahora

    def control_loop(self):
        twist = Twist()
        ahora = time.time()
        
        if self.lecturas_acumuladas < N_LECTURAS_PROMEDIO:
            self.cmd_pub.publish(twist)
            return

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
            # Tus giros originales intactos
            twist.linear.x = VEL_AVANCE_GIRO if self.d_front > 0.22 else 0.0
            twist.angular.z = VEL_GIRO if self.estado == 'girar_izq' else -VEL_GIRO
            
            # Comprobación de callejón tras intentar girar
            if (ahora - self.tiempo_inicio_giro) > TIEMPO_GIRO_MINIMO:
                if self.d_front < 0.25 and self.d_left < 0.35 and self.d_right < 0.35:
                    self.estado = 'giro_180'
                    self.tiempo_inicio_giro = ahora
                elif self.d_front > DIST_PARAR_GIRO:
                    self.estado = 'avanzar'

        elif self.estado == 'giro_180':
            # Pivotaje puro: sin avance lineal, solo rotación
            twist.linear.x = 0.0
            twist.angular.z = VEL_GIRO 
            # Si tras girar un poco ya ve espacio, volvemos a avanzar
            if (ahora - self.tiempo_inicio_giro) > 3.5: # Tiempo aprox para 180 grados
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