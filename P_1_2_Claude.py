#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time
from collections import deque

# --- PARÁMETROS ---
DIST_GIRO_PASILLO      = 0.32
DIST_PARAR_GIRO        = 0.32
DIST_FRENAR            = 0.55
DIST_PARED_DERECHA     = 0.25
DIST_PASILLO           = 0.45
DIST_SEGURIDAD_TRASERA = 0.15
TICKS_CONFIRMAR_CALLEJON = 15   # REFUERZO: Espera 1.5s real antes de girar 180º
TIEMPO_GIRO_180        = 3.5    

VEL_LINEAR_PASILLO = 0.06
VEL_LINEAR_NORMAL  = 0.08
VEL_RETROCESO      = 0.05
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
        self.odom_sub = self.create_subscription(Odometry,  '/odom', self.odom_callback, 10)
        self.timer    = self.create_timer(0.1, self.control_loop)

        self.buf_front = deque(maxlen=N_LECTURAS_PROMEDIO)
        self.buf_right = deque(maxlen=N_LECTURAS_PROMEDIO)
        self.buf_left  = deque(maxlen=N_LECTURAS_PROMEDIO)
        self.buf_back  = deque(maxlen=N_LECTURAS_PROMEDIO)

        self.d_front = self.d_right = self.d_left = self.d_back = 3.0
        self.lecturas_acumuladas = 0
        self.estado              = 'esperando'
        self.tiempo_inicio_giro  = 0.0
        self.ticks_callejon      = 0 

        self.log_file = open(LOG_FILE, 'w')
        self._log_raw('=== INICIO SESION MAZE SOLVER ===')

    def _log_raw(self, msg):
        ts = time.strftime('%H:%M:%S')
        self.log_file.write(f'[{ts}] {msg}\n')
        self.log_file.flush()

    def _log_tick(self, evento=''):
        self._log_raw(f'{self.estado:<13} | F:{self.d_front:5.2f} R:{self.d_right:5.2f} L:{self.d_left:5.2f} | {evento}')

    def clean(self, v):
        return 3.0 if (math.isinf(v) or math.isnan(v)) else float(v)

    def promedio(self, buf):
        return sum(buf) / len(buf) if buf else 3.0

    def scan_callback(self, msg):
        r = msg.ranges
        if len(r) < 360: return
        self.buf_front.append(min(min(r[350:360]), min(r[0:10])))
        self.buf_right.append(min(r[260:310]))
        self.buf_left.append(min(r[50:110]))
        self.buf_back.append(min(r[170:190]))
        
        self.lecturas_acumuladas += 1
        self.d_front = self.promedio(self.buf_front)
        self.d_right = self.promedio(self.buf_right)
        self.d_left  = self.promedio(self.buf_left)
        self.d_back  = self.promedio(self.buf_back)

    def odom_callback(self, msg):
        pass

    def _cambiar_estado(self, nuevo):
        if nuevo != self.estado:
            self._log_raw(f'>>> CAMBIO ESTADO: {self.estado} -> {nuevo}')
            self.estado = nuevo

    def control_loop(self):
        twist = Twist()
        if self.lecturas_acumuladas < N_LECTURAS_PROMEDIO:
            self.cmd_pub.publish(twist)
            return

        ahora = time.time()
        
        # --- 1. LÓGICA DE DETECCIÓN CALLEJÓN (CONFIRMADA) ---
        # Solo activa si detecta el bloqueo constantemente durante 1.5s
        if self.d_front < 0.20 and self.d_left < 0.25 and self.d_right < 0.25:
            self.ticks_callejon += 1
        else:
            self.ticks_callejon = 0

        if self.ticks_callejon >= TICKS_CONFIRMAR_CALLEJON and self.estado != 'girar_180':
            self._cambiar_estado('girar_180')
            self.tiempo_inicio_giro = ahora
            self.ticks_callejon = 0

        # --- 2. MÁQUINA DE ESTADOS ---
        if self.estado == 'esperando': self._cambiar_estado('avanzar')

        elif self.estado == 'avanzar':
            if self.d_front < DIST_PARAR_GIRO:
                lado = 'izq' if self.d_left >= self.d_right else 'der'
                self._cambiar_estado(f'girar_{lado}')
                self.tiempo_inicio_giro = ahora

        elif self.estado == 'girar_180':
            if (ahora - self.tiempo_inicio_giro) > TIEMPO_GIRO_180:
                self._cambiar_estado('avanzar')

        elif self.estado in ('girar_izq', 'girar_der'):
            if (ahora - self.tiempo_inicio_giro) > TIEMPO_GIRO_MINIMO:
                if self.d_front >= DIST_PARAR_GIRO + 0.10:
                    self._cambiar_estado('avanzar')

        # --- 3. ACCIONES ---
        if self.estado == 'girar_180':
            twist.linear.x = 0.0 # Estático sobre el eje
            twist.angular.z = VEL_GIRO
            
        elif self.estado == 'girar_izq':
            twist.angular.z = VEL_GIRO
            twist.linear.x = VEL_AVANCE_GIRO if self.d_front > 0.22 else 0.0
            
        elif self.estado == 'girar_der':
            twist.angular.z = -VEL_GIRO
            twist.linear.x = VEL_AVANCE_GIRO if self.d_front > 0.22 else 0.0
            
        else: # avanzar
            vel = self.velocidad_frenada(self.d_front, VEL_LINEAR_NORMAL)
            twist.linear.x = vel
            error = DIST_PARED_DERECHA - self.d_right
            twist.angular.z = max(min(KP * error, 0.40), -0.40)

        self.cmd_pub.publish(twist)
        self._log_tick(f"TicksCal:{self.ticks_callejon}")

    def velocidad_frenada(self, d_front, vel_max):
        if d_front >= DIST_FRENAR: return vel_max
        if d_front <= DIST_PARAR_GIRO: return 0.0
        return round(vel_max * ((d_front - DIST_PARAR_GIRO) / (DIST_FRENAR - DIST_PARAR_GIRO)), 3)

def main(args=None):
    rclpy.init(args=args)
    nodo = MazeSolver()
    try: rclpy.spin(nodo)
    except KeyboardInterrupt: pass
    finally:
        nodo.cmd_pub.publish(Twist())
        nodo.log_file.close()
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()