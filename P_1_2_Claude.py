#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import time
from collections import deque

# --- PARÁMETROS ---
DIST_PARAR_GIRO    = 0.32
DIST_PARED_DERECHA = 0.25
VEL_LINEAR_NORMAL  = 0.08
VEL_GIRO           = 0.28
KP                 = 0.8  # Reducido para que sea menos agresivo

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver_node')
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.timer    = self.create_timer(0.1, self.control_loop)
        
        self.d_front = 3.0
        self.d_right = 3.0
        self.estado = 'avanzar'

    def scan_callback(self, msg):
        r = msg.ranges
        # Filtro simple para evitar ruido de lectura
        self.d_front = min(min(r[350:360]), min(r[0:10]))
        self.d_right = min(r[260:310])

    def control_loop(self):
        twist = Twist()

        if self.estado == 'avanzar':
            # 1. Si hay pared delante, giramos
            if self.d_front < DIST_PARAR_GIRO:
                twist.linear.x = 0.0
                twist.angular.z = VEL_GIRO
            else:
                # 2. MODO RECTA PURA: Solo corregimos si nos alejamos mucho
                error = DIST_PARED_DERECHA - self.d_right
                
                # ZONA MUERTA: Si estamos a una distancia aceptable (0.20 - 0.30), no giramos
                if abs(error) < 0.05:
                    twist.angular.z = 0.0
                else:
                    # Corrección suave
                    twist.angular.z = KP * error
                
                twist.linear.x = VEL_LINEAR_NORMAL
                
        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    nodo = MazeSolver()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()