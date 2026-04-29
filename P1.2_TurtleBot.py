#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver_node')
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.regions = {'front': 0.0, 'right': 0.0, 'left': 0.0}
        self.meta_alcanzada = False
        
        # Coordenadas exactas de la meta que encontraste con el odómetro
        self.META_X = 2.75 
        self.META_Y = 1.71 
        self.DISTANCIA_MINIMA_META = 0.25

    def clean_distance(self, distance):
        if math.isinf(distance) or math.isnan(distance):
            return 3.0 
        return distance

    def scan_callback(self, msg):
        # Evita cuelgues si Gazebo envía datos vacíos
        if not msg.ranges or len(msg.ranges) < 360:
            return

        # Visión ampliada para esquinas perfectas
        front_ranges = msg.ranges[0:30] + msg.ranges[330:359]
        left_ranges = msg.ranges[30:110]
        right_ranges = msg.ranges[250:330]

        self.regions['front'] = min(min([self.clean_distance(x) for x in front_ranges]), 3.0)
        self.regions['left'] = min(min([self.clean_distance(x) for x in left_ranges]), 3.0)
        self.regions['right'] = min(min([self.clean_distance(x) for x in right_ranges]), 3.0)

    def odom_callback(self, msg):
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        dist_to_meta = math.sqrt((current_x - self.META_X)**2 + (current_y - self.META_Y)**2)
        
        if dist_to_meta < self.DISTANCIA_MINIMA_META:
            self.meta_alcanzada = True

    def control_loop(self):
        twist = Twist()

        if self.meta_alcanzada:
            self.get_logger().info('¡META ALCANZADA! Deteniendo el robot.')
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return

        # Simplificamos las variables para que sea más fácil de leer
        d_front = self.regions['front']
        d_right = self.regions['right']

        # --- LÓGICA ANTI-TEMBLEQUE ---
        
        # 1. Aumentamos la distancia frontal para que empiece a girar antes de tragarse la esquina
        if d_front < 0.50:
            # PELIGRO FRONTAL: Olvida todo y gira a la izquierda rápidamente
            twist.linear.x = 0.0
            twist.angular.z = 0.6 
            
        else:
            # 2. FRENTE LIBRE: Control de la pared derecha
            if d_right < 0.30:
                # Nos chocamos por la derecha -> Volantazo suave a la izquierda
                twist.linear.x = 0.12
                twist.angular.z = 0.4 
            elif d_right > 0.65:
                # Hemos perdido la pared derecha por completo -> La buscamos
                twist.linear.x = 0.12
                twist.angular.z = -0.4
            else:
                # ZONA NEUTRA (entre 0.30m y 0.65m): ¡Tira recto!
                # Esto es lo que rompe el bucle de oscilación en las curvas
                twist.linear.x = 0.15
                twist.angular.z = 0.0

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    nodo = MazeSolver()
    
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error detectado durante la ejecución: {e}")
    finally:
        # Cierre seguro modificado para evitar errores secundarios al apagar
        if rclpy.ok():
            try:
                twist = Twist()
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                nodo.cmd_pub.publish(twist)
            except Exception:
                pass
            nodo.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()