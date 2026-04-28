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
        
        # Publicador para enviar comandos de velocidad al robot
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Suscriptores para los sensores LIDAR y Odometría
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        # Bucle de control que se ejecuta cada 0.1 segundos
        self.timer = self.create_timer(0.1, self.control_loop)
        
        # Variables para almacenar el estado de las regiones del láser
        self.regions = {
            'front': 0.0,
            'right': 0.0,
            'left': 0.0,
        }
        self.meta_alcanzada = False
        
        # --- CONFIGURACIÓN DE LA META ---
        # Ajusta estos valores según las coordenadas que encontraste en Gazebo
        self.META_X = 2.0 
        self.META_Y = 2.0 
        self.DISTANCIA_MINIMA_META = 0.25

    def clean_distance(self, distance):
        # Limpia valores infinitos o erróneos del sensor
        if math.isinf(distance) or math.isnan(distance):
            return 3.0 
        return distance

    def scan_callback(self, msg):
        # Ampliamos los rangos de visión para detectar esquinas laterales y evitar bloqueos
        # Frente: de -20 a +20 grados
        front_ranges = msg.ranges[0:20] + msg.ranges[340:359]
        # Lados: conos de visión más amplios (90 grados cada uno)
        left_ranges = msg.ranges[45:135]
        right_ranges = msg.ranges[225:315]

        # Guardamos la distancia mínima detectada en cada región
        self.regions['front'] = min(min([self.clean_distance(x) for x in front_ranges]), 3.0)
        self.regions['left'] = min(min([self.clean_distance(x) for x in left_ranges]), 3.0)
        self.regions['right'] = min(min([self.clean_distance(x) for x in right_ranges]), 3.0)

    def odom_callback(self, msg):
        # Extraemos la posición actual para calcular la distancia a la meta
        current_x = msg.pose.pose.position.x
        current_y = msg.pose.pose.position.y
        
        # Fórmula de distancia euclidiana
        dist_to_meta = math.sqrt((current_x - self.META_X)**2 + (current_y - self.META_Y)**2)
        
        if dist_to_meta < self.DISTANCIA_MINIMA_META:
            self.meta_alcanzada = True

    def control_loop(self):
        twist = Twist()

        # Si el robot llega a la zona de meta, se detiene
        if self.meta_alcanzada:
            self.get_logger().info('¡META ALCANZADA! Deteniendo el robot.')
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_pub.publish(twist)
            return

        # Definimos distancias de seguridad para maniobrar
        dist_seguridad = 0.45 
        dist_critica = 0.28 # Umbral para evitar colisiones laterales en curvas cerradas

        # Lógica de navegación: Seguidor de pared derecha mejorada
        if self.regions['front'] > dist_seguridad:
            if self.regions['right'] < dist_critica:
                # Demasiado cerca de la derecha: corregir alejándose un poco hacia la izquierda
                twist.linear.x = 0.12
                twist.angular.z = 0.3 
            elif self.regions['right'] < dist_seguridad:
                # Distancia óptima: avanzar recto siguiendo la pared
                twist.linear.x = 0.15
                twist.angular.z = 0.0
            else:
                # Se aleja de la pared: girar suavemente a la derecha para recuperarla
                twist.linear.x = 0.12
                twist.angular.z = -0.3
        else:
            # Obstáculo frontal: detener avance y elegir dirección de giro
            twist.linear.x = 0.0
            if self.regions['left'] > dist_seguridad:
                # Prioridad giro a la izquierda si el frente está bloqueado
                twist.angular.z = 0.5
            elif self.regions['right'] > dist_seguridad:
                twist.angular.z = -0.5
            else:
                # Callejón sin salida: giro de emergencia rápido
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
        # Aseguramos que el robot se detenga al cerrar el programa
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        nodo.cmd_pub.publish(twist)
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()