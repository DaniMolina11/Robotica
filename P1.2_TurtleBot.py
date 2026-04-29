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
        
        # Coordenadas exactas de la meta
        self.META_X = 2.75 
        self.META_Y = 1.71 
        self.DISTANCIA_MINIMA_META = 0.25

    def clean_distance(self, distance):
        if math.isinf(distance) or math.isnan(distance):
            return 3.0 
        return distance

    def scan_callback(self, msg):
        if not msg.ranges or len(msg.ranges) < 360:
            return

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

        d_front = self.regions['front']
        d_right = self.regions['right']

        if d_front < 0.40:
            # PELIGRO FRONTAL: Sigue siendo prioritario frenar y dar la curva cerrada
            twist.linear.x = 0.0
            twist.angular.z = 0.6 
        else:
            # FRENTE LIBRE: Activamos el piloto automático suave
            twist.linear.x = 0.12 # Velocidad constante hacia adelante
            
            # --- CONTROL PROPORCIONAL ANTI-ZIGZAG ---
            distancia_ideal = 0.30 # Queremos ir a 30 cm de la pared exactamente
            
            # Calculamos el error (positivo si nos acercamos a la pared, negativo si nos alejamos)
            error = distancia_ideal - d_right 
            
            # Multiplicamos el error para mover el volante proporcionalmente
            giro = error * 2.5 
            
            # Limitamos el giro máximo a 0.6 para que no haga trompos si pierde la pared
            giro = max(min(giro, 0.6), -0.6) 
            
            twist.angular.z = giro

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