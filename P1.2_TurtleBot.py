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
        
        # Publicador para mover el robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Suscriptores: LIDAR y Odometría
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Timer para el bucle de control (se ejecuta cada 0.1 segundos)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Variables de estado
        self.regions = {
            'front': 0.0,
            'right': 0.0,
            'left': 0.0,
        }
        self.meta_alcanzada = False
        
        # --- CONFIGURACIÓN DE LA META (SIMULACIÓN) ---
        # DEBES CAMBIAR ESTOS VALORES por los que encontraste en la Práctica 1
        self.META_X = 2.0 
        self.META_Y = 2.0 
        self.DISTANCIA_MINIMA_META = 0.25 # Distancia para considerar que ha llegado

    def clean_distance(self, distance):
        # Evitar lecturas infinitas del Lidar
        if math.isinf(distance) or math.isnan(distance):
            return 3.0 # Valor seguro máximo
        return distance

    def scan_callback(self, msg):
        # El LIDAR tiene 360 posiciones (1 grado de resolucion)
        # 0: Frente, 90: Izquierda, 180: Atras, 270: Derecha
        
        # Hacemos una media de un pequeño rango de grados para evitar puntos ciegos
        front_ranges = msg.ranges[0:10] + msg.ranges[350:359]
        left_ranges = msg.ranges[80:100]
        right_ranges = msg.ranges[260:280]

        # Limpiamos los datos y sacamos la distancia minima de cada region
        self.regions['front'] = min(min([self.clean_distance(x) for x in front_ranges]), 3.0)
        self.regions['left'] = min(min([self.clean_distance(x) for x in left_ranges]), 3.0)
        self.regions['right'] = min(min([self.clean_distance(x) for x in right_ranges]), 3.0)

    def odom_callback(self, msg):
        # Obtenemos la posicion actual (x, y) del robot
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        
        # Calculamos la distancia euclidiana hasta la meta
        dist_to_meta = math.sqrt((current_x - self.META_X)**2 + (current_y - self.META_Y)**2)
        
        if dist_to_meta < self.DISTANCIA_MINIMA_META:
            self.meta_alcanzada = True

    def control_loop(self):
        twist = Twist()

        # Si hemos llegado a la meta, paramos el robot y no hacemos nada mas
        if self.meta_alcanzada:
            self.get_logger().info('¡META ALCANZADA! Deteniendo el robot.')
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return

        # Distancias de seguridad
        dist_seguridad = 0.4

        # Lógica del algoritmo: Wall Follower (Priorizar giro a la izquierda si choca, buscar pared a la derecha)
        
        if self.regions['front'] > dist_seguridad:
            if self.regions['right'] < dist_seguridad:
                # Camino libre al frente y pared a la derecha -> Seguir recto siguiendo la pared
                twist.linear.x = 0.15
                twist.angular.z = 0.0
            else:
                # Camino libre pero no hay pared a la derecha -> Girar ligeramente a la derecha para encontrarla
                twist.linear.x = 0.1
                twist.angular.z = -0.3
        else:
            # Hay un obstaculo al frente -> Toca girar
            twist.linear.x = 0.0
            if self.regions['left'] > dist_seguridad:
                # Izquierda libre -> Girar a la izquierda
                twist.angular.z = 0.5
            elif self.regions['right'] > dist_seguridad:
                # Derecha libre -> Girar a la derecha
                twist.angular.z = -0.5
            else:
                # Callejón sin salida -> Dar la vuelta
                twist.angular.z = 0.8

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    nodo = MazeSolver()
    
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        # Detener el robot de forma segura al cerrar
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        nodo.cmd_pub.publish(twist)
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()