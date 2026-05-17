
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import math

# Umbrales de distancia (en metros)
DIST_FRENTE = 0.35    # Si hay pared a menos de esto, girar
DIST_DERECHA = 0.35   # Distancia objetivo a la pared derecha

# Velocidades
VEL_LINEAR = 0.10
VEL_ANGULAR = 0.5

# Posición de salida del laberinto (ajustar con odometría)
META_X = 0.0   # ← lo encontrarás llevando el robot manualmente a la salida
META_Y = 0.0
DIST_META = 0.3  # radio de llegada

class LaberintSolver(Node):
    def __init__(self):
        super().__init__('laberint_solver')
        
        # Publicador de velocidades
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # Suscriptores
        self.create_subscription(LaserScan, 'scan', self.callback_scan, 10)
        self.create_subscription(Odometry, 'odom', self.callback_odom, 10)
        
        # Estado
        self.dist_frente = float('inf')
        self.dist_derecha = float('inf')
        self.pos_x = 0.0
        self.pos_y = 0.0
        self.meta_alcanzada = False

    def callback_scan(self, msg):
        ranges = msg.ranges
        
        # Extraer distancias relevantes (media de varios grados para robustez)
        self.dist_frente  = min(ranges[355:360] + ranges[0:5])
        self.dist_derecha = min(ranges[265:275])
        # También útil: izquierda = ranges[85:95]
        
        if not self.meta_alcanzada:
            self.navegar()

    def callback_odom(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        
        # Comprobar si hemos llegado a la meta
        dist = math.sqrt((self.pos_x - META_X)**2 + (self.pos_y - META_Y)**2)
        if dist < DIST_META:
            self.meta_alcanzada = True
            self.parar()
            self.get_logger().info('¡LABERINTO RESUELTO!')

    def navegar(self):
        twist = Twist()
        
        if self.dist_frente < DIST_FRENTE:
            # Obstáculo al frente → girar izquierda
            twist.linear.x = 0.0
            twist.angular.z = VEL_ANGULAR
        elif self.dist_derecha > DIST_DERECHA * 1.5:
            # Hemos perdido la pared derecha → girar derecha
            twist.linear.x = VEL_LINEAR * 0.5
            twist.angular.z = -VEL_ANGULAR * 0.5
        else:
            # Camino libre → avanzar
            twist.linear.x = VEL_LINEAR
            twist.angular.z = 0.0
        
        self.pub.publish(twist)

    def parar(self):
        self.pub.publish(Twist())  # Todo a 0

def main():
    rclpy.init()
    node = LaberintSolver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.parar()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()