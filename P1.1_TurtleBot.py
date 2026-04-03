#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# Límits de velocitat segons l'enunciat
MAX_LIN_VEL = 0.22   # [cite: 131]
MIN_LIN_VEL = -0.22  # [cite: 131]
MAX_ANG_VEL = 2.84   # [cite: 132]
MIN_ANG_VEL = -2.84  # [cite: 132]

# Increments de velocitat segons l'enunciat
LIN_VEL_STEP = 0.01  # [cite: 126]
ANG_VEL_STEP = 0.1   # [cite: 126]

msg = """
Control de TurtleBot3 - Práctica 1
----------------------------------
Teclas de movimiento:
        w
   a    s    d
        x

w : Incrementar velocidad lineal (adelante)
x : Decrementar velocidad lineal (atrás)
a : Girar hacia la izquierda
d : Girar hacia la derecha
s : Parar el robot

CTRL-C para salir del programa y parar el robot
"""

# Función para capturar la tecla presionada en la terminal de Linux
def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init(args=args)
    # Crear el nodo
    node = rclpy.create_node('turtlebot_teleop_keyboard')
    # Crear el publicador en el tópico cmd_vel con el tipo de mensaje Twist
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    # Inicializar las velocidades a 0.0 según el enunciado [cite: 124, 125]
    target_linear_vel = 0.0
    target_angular_vel = 0.0

    try:
        print(msg)
        while True:
            key = getKey(settings)

            # Lógica de las teclas según el enunciado
            if key == 'w':
                target_linear_vel += LIN_VEL_STEP # [cite: 112, 113]
            elif key == 'x':
                target_linear_vel -= LIN_VEL_STEP # [cite: 114, 115]
            elif key == 'a':
                target_angular_vel += ANG_VEL_STEP # [cite: 116, 117]
            elif key == 'd':
                target_angular_vel -= ANG_VEL_STEP # [cite: 118, 119]
            elif key == 's':
                target_linear_vel = 0.0  # [cite: 120, 121]
                target_angular_vel = 0.0 # [cite: 120, 121]
            elif key == '\x03': # Código para Ctrl-C [cite: 122]
                break

            # Limitar las velocidades para no superar los máximos/mínimos permitidos [cite: 135]
            if target_linear_vel > MAX_LIN_VEL:
                target_linear_vel = MAX_LIN_VEL
            elif target_linear_vel < MIN_LIN_VEL:
                target_linear_vel = MIN_LIN_VEL

            if target_angular_vel > MAX_ANG_VEL:
                target_angular_vel = MAX_ANG_VEL
            elif target_angular_vel < MIN_ANG_VEL:
                target_angular_vel = MIN_ANG_VEL

            # Formatear y publicar el mensaje Twist
            twist = Twist()
            twist.linear.x = target_linear_vel
            twist.linear.y = 0.0
            twist.linear.z = 0.0
            twist.angular.x = 0.0
            twist.angular.y = 0.0
            twist.angular.z = target_angular_vel
            pub.publish(twist)

            # Mostrar la velocidad actual por pantalla para control visual [cite: 136]
            print(f"Velocitat actual -> Lineal: {target_linear_vel:.2f} | Angular: {target_angular_vel:.2f}")

    except Exception as e:
        print(e)

    finally:
        # En caso de salir del programa (Ctrl-C), asegurar que el robot se detiene [cite: 122]
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        # Restaurar la configuración de la terminal y apagar ROS 2
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()