#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time
from collections import deque

# --- PARÁMETROS DE NAVEGACIÓN ---
DIST_GIRO_PASILLO      = 0.22   # Aproximación al muro para detectar mejor los cruces
DIST_PARAR_GIRO        = 0.22   
DIST_FRENAR            = 0.55   
DIST_PARED_DERECHA     = 0.25   
DIST_PASILLO           = 0.45   
DIST_SEGURIDAD_TRASERA = 0.15   
DIST_LATERAL_SEGURIDAD = 0.18   # Distancia crítica para reducir giro

# UMBRALES DE HUECO
DIST_LATERAL_PURO      = 0.45  
TICKS_CONFIRMAR_CALLEJON = 6    # Tiempo de espera (0.6s) para confirmar callejón

VEL_LINEAR_PASILLO    = 0.06
VEL_LINEAR_NORMAL     = 0.08
VEL_RETROCESO         = 0.05
VEL_GIRO              = 0.28   
VEL_AVANCE_GIRO       = 0.06   
KP                    = 1.2

N_LECTURAS_PROMEDIO   = 5
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

        self.d_front = self.d_right = self.d_left = self.d_back = 3.0
        self.d_izq_puro = self.d_der_puro = 3.0

        self.lecturas_acumuladas = 0
        self.estado              = 'esperando'
        self.tiempo_inicio_giro  = 0.0
        self.giro_comprometido   = False
        self.ticks_callejon      = 0 

        self.log_file = open(LOG_FILE, 'w')
        self._log_raw('=== INICIO SESION MAZE SOLVER ===')

    def _log_raw(self, msg):
        ts = time.strftime('%H:%M:%S')
        self.log_file.write(f'[{ts}] {msg}\n')
        self.log_file.flush()

    def clean(self, v):
        return 3.0 if (math.isinf(v) or math.isnan(v)) else float(v)

    def sector_min(self, ranges, a, b):
        return min(self.clean(ranges[i]) for i in range(a, b))

    def promedio(self, buf):
        return sum(buf) / len(buf) if buf else 3.0

    def scan_callback(self, msg):
        r = msg.ranges
        self.buf_front.append(min(self.sector_min(r, 350, 360), self.sector_min(r, 0, 10)))
        self.buf_right.append(self.sector_min(r, 260, 310))
        self.buf_left.append(self.sector_min(r, 50, 110))
        self.buf_back.append(self.sector_min(r, 170, 190))
        
        # Visión de reojo para anticipar cruces
        self.d_izq_puro = min(self.clean(r[i]) for i in range(70, 95))
        self.d_der_puro = min(self.clean(r[i]) for i in range(265, 290))

        self.lecturas_acumuladas += 1
        self.d_front = self.promedio(self.buf_front)
        self.d_right = self.promedio(self.buf_right)
        self.d_left  = self.promedio(self.buf_left)
        self.d_back  = self.promedio(self.buf_back)

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
        
        # --- 1. DETECCIÓN DE CALLEJÓN (GATED) ---
        # Solo se activa si el robot intenta avanzar. Se ignora mientras gira.
        if self.estado in ('avanzar', 'pasillo'):
            if self.d_front <= 0.25 and self.d_left < 0.35 and self.d_right < 0.35:
                self.ticks_callejon += 1
                if self.ticks_callejon >= TICKS_CONFIRMAR_CALLEJON:
                    self._cambiar_estado('retroceder')
            else:
                self.ticks_callejon = 0
        else:
            self.ticks_callejon = 0 

        # --- 2. MÁQUINA DE ESTADOS ---
        if self.estado == 'esperando': self._cambiar_estado('avanzar')

        elif self.estado == 'avanzar':
            if self.d_front < DIST_PARAR_GIRO:
                lado = 'izq' if self.d_left >= self.d_right else 'der'
                self._cambiar_estado(f'girar_{lado}')
                self.tiempo_inicio_giro = ahora
                self.giro_comprometido = True

        elif self.estado == 'retroceder':
            if self.d_left > 0.40 or self.d_right > 0.40:
                lado = 'izq' if self.d_left > self.d_right else 'der'
                self._cambiar_estado(f'girar_{lado}')
                self.tiempo_inicio_giro = ahora
                self.giro_comprometido = True

        elif self.estado in ('girar_izq', 'girar_der'):
            # Permite completar el giro mínimo
            if (ahora - self.tiempo_inicio_giro) > 1.5:
                if self.d_front >= DIST_PARAR_GIRO + 0.10:
                    self._cambiar_estado('avanzar')

        # --- 3. ACCIONES Y CONTROL DE SEGURIDAD ---
        if self.estado == 'retroceder':
            twist.linear.x = -VEL_RETROCESO if self.d_back > DIST_SEGURIDAD_TRASERA else 0.0
            
        elif self.estado == 'girar_izq':
            # Control de seguridad lateral durante el giro
            twist.angular.z = VEL_GIRO if self.d_left > DIST_LATERAL_SEGURIDAD else VEL_GIRO * 0.5
            twist.linear.x = VEL_AVANCE_GIRO if self.d_front > 0.22 else 0.0
            
        elif self.estado == 'girar_der':
            # Control de seguridad lateral durante el giro
            twist.angular.z = -VEL_GIRO if self.d_right > DIST_LATERAL_SEGURIDAD else -VEL_GIRO * 0.5
            twist.linear.x = VEL_AVANCE_GIRO if self.d_front > 0.22 else 0.0
            
        else: # avanzar / pasillo
            twist.linear.x = self.velocidad_frenada(self.d_front, VEL_LINEAR_NORMAL)
            # PD simple para seguir pared derecha
            error = DIST_PARED_DERECHA - self.d_right
            twist.angular.z = max(min(KP * error, 0.40), -0.40)

        self.cmd_pub.publish(twist)

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