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
DIST_GIRO_PASILLO      = 0.22   # Se acerca más para ver mejor el cruce
DIST_PARAR_GIRO        = 0.22   
DIST_FRENAR            = 0.55   
DIST_PARED_DERECHA     = 0.25   
DIST_PASILLO           = 0.45   
DIST_SEGURIDAD_TRASERA = 0.15   
DIST_LATERAL_SEGURIDAD = 0.18   # Distancia mínima de seguridad durante giros

# UMBRALES DE HUECO
DIST_LATERAL_PURO      = 0.45  
DIST_DIAG_LIBRE        = 0.50  

VEL_LINEAR_PASILLO    = 0.06
VEL_LINEAR_NORMAL     = 0.08
VEL_RETROCESO         = 0.05
VEL_GIRO              = 0.28   
VEL_AVANCE_GIRO       = 0.06   
KP                    = 1.2

TIEMPO_GIRO_MINIMO       = 1.5
N_LECTURAS_PROMEDIO      = 5
TICKS_CONFIRMACION       = 4
TICKS_CONFIRMAR_CALLEJON = 5  

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
        self.buf_diag_izq = deque(maxlen=N_LECTURAS_PROMEDIO)
        self.buf_diag_der = deque(maxlen=N_LECTURAS_PROMEDIO)

        self.d_front    = 3.0
        self.d_right    = 3.0
        self.d_left     = 3.0
        self.d_back     = 3.0
        self.d_diag_izq = 3.0
        self.d_diag_der = 3.0
        self.d_izq_puro = 3.0
        self.d_der_puro = 3.0

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.lecturas_acumuladas = 0
        self.estado          = 'esperando'
        self.estado_anterior = 'esperando'
        self.tiempo_inicio_giro  = 0.0
        self.giro_comprometido   = False
        self.ticks_fuera_pasillo = 0
        self.ticks_callejon      = 0  

        self.META_X               = 2.75
        self.META_Y               = 1.71
        self.DISTANCIA_MINIMA_META = 0.25
        self.meta_alcanzada       = False

        self.sim_time    = 0.0
        self.vel_lin_pub = 0.0
        self.vel_ang_pub = 0.0

        self.log_file = open(LOG_FILE, 'w')
        self._log_raw('=== INICIO SESION MAZE SOLVER ===')

    def _log_raw(self, msg):
        ts = time.strftime('%H:%M:%S')
        self.log_file.write(f'[{ts}] {msg}\n')
        self.log_file.flush()

    def _log_tick(self, evento=''):
        self._log_raw(
            f'{self.sim_time:9.2f} | {self.pos_x:+6.3f} | {self.pos_y:+6.3f} | '
            f'{self.estado:<13}| '
            f'{self.d_front:5.2f} | {self.d_right:5.2f} | {self.d_left:5.2f} | '
            f'{self.vel_lin_pub:+7.3f} | {self.vel_ang_pub:+7.3f} | {evento}'
        )

    def clean(self, v):
        return 3.0 if (math.isinf(v) or math.isnan(v)) else float(v)

    def sector_min(self, ranges, a, b):
        return min(self.clean(ranges[i]) for i in range(a, b))

    def promedio(self, buf):
        return sum(buf) / len(buf) if buf else 3.0

    def reset_filtros(self):
        self.buf_front.clear()
        self.buf_right.clear()
        self.buf_left.clear()
        self.buf_back.clear()
        self.buf_diag_izq.clear()
        self.buf_diag_der.clear()
        self.lecturas_acumuladas = 0

    def velocidad_frenada(self, d_front, vel_max):
        if d_front >= DIST_FRENAR: return vel_max
        if d_front <= DIST_PARAR_GIRO: return 0.0
        ratio = (d_front - DIST_PARAR_GIRO) / (DIST_FRENAR - DIST_PARAR_GIRO)
        return round(vel_max * ratio, 3)

    def scan_callback(self, msg):
        r = msg.ranges
        if len(r) < 360: return
        self.buf_front.append(min(self.sector_min(r, 350, 360), self.sector_min(r, 0, 10)))
        self.buf_right.append(self.sector_min(r, 260, 310))
        self.buf_left.append(self.sector_min(r, 50, 110))
        self.buf_back.append(self.sector_min(r, 170, 190))
        self.buf_diag_izq.append(self.sector_min(r, 30, 60))
        self.buf_diag_der.append(self.sector_min(r, 300, 330))
        
        self.d_izq_puro = min(self.clean(r[i]) for i in range(70, 95))
        self.d_der_puro = min(self.clean(r[i]) for i in range(265, 290))

        self.lecturas_acumuladas += 1
        self.d_front    = self.promedio(self.buf_front)
        self.d_right    = self.promedio(self.buf_right)
        self.d_left     = self.promedio(self.buf_left)
        self.d_back     = self.promedio(self.buf_back)
        self.d_diag_izq = self.promedio(self.buf_diag_izq)
        self.d_diag_der = self.promedio(self.buf_diag_der)
        self.sim_time   = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def odom_callback(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y

    def _cambiar_estado(self, nuevo, motivo=''):
        if nuevo != self.estado:
            self.estado = nuevo

    def _decidir_lado_giro(self):
        en_pasillo = (self.d_right < DIST_PASILLO and self.d_left < DIST_PASILLO)
        if en_pasillo:
            return 'izq' if self.d_diag_izq >= self.d_diag_der else 'der'
        return 'izq' if self.d_left >= self.d_right else 'der'

    def _iniciar_giro(self, ahora):
        lado = self._decidir_lado_giro()
        self._cambiar_estado(f'girar_{lado}')
        self.tiempo_inicio_giro = ahora
        self.giro_comprometido  = True

    def control_loop(self):
        twist = Twist()
        if self.lecturas_acumuladas < N_LECTURAS_PROMEDIO:
            self.cmd_pub.publish(twist)
            return

        if self.estado == 'esperando': self._cambiar_estado('avanzar')

        ahora          = time.time()
        tiempo_girando = ahora - self.tiempo_inicio_giro
        en_pasillo     = (self.d_right < DIST_PASILLO and self.d_left < DIST_PASILLO)
        izq_cerrada    = (self.d_izq_puro < DIST_LATERAL_PURO and self.d_diag_izq < DIST_DIAG_LIBRE)
        der_cerrada    = (self.d_der_puro < DIST_LATERAL_PURO and self.d_diag_der < DIST_DIAG_LIBRE)

        # MÁQUINA DE ESTADOS
        if self.estado == 'pasillo':
            if en_pasillo:
                self.ticks_fuera_pasillo = 0
                if self.d_front < DIST_GIRO_PASILLO:
                    if izq_cerrada and der_cerrada:
                        self.ticks_callejon += 1
                        if self.ticks_callejon >= TICKS_CONFIRMAR_CALLEJON:
                            self._cambiar_estado('retroceder')
                            self.reset_filtros()
                    else:
                        self.ticks_callejon = 0
                        self._iniciar_giro(ahora)
            else:
                self.ticks_fuera_pasillo += 1
                if self.ticks_fuera_pasillo >= TICKS_CONFIRMACION:
                    self._cambiar_estado('avanzar')
                    self.ticks_callejon = 0

        elif self.estado == 'avanzar':
            if en_pasillo and self.d_front >= DIST_GIRO_PASILLO:
                self._cambiar_estado('pasillo')
            elif self.d_front < DIST_PARAR_GIRO:
                if izq_cerrada and der_cerrada:
                    self.ticks_callejon += 1
                    if self.ticks_callejon >= TICKS_CONFIRMAR_CALLEJON:
                        self._cambiar_estado('retroceder')
                        self.reset_filtros()
                else:
                    self.ticks_callejon = 0
                    self._iniciar_giro(ahora)

        elif self.estado == 'retroceder':
            if self.d_diag_izq > 0.45 or self.d_diag_der > 0.45 or self.d_back <= DIST_SEGURIDAD_TRASERA:
                self.ticks_callejon = 0  
                self._iniciar_giro(ahora)

        elif self.estado in ('girar_izq', 'girar_der'):
            if self.giro_comprometido and (ahora - self.tiempo_inicio_giro) < TIEMPO_GIRO_MINIMO:
                pass
            elif self.d_front >= DIST_PARAR_GIRO + 0.10:
                self._cambiar_estado('avanzar')
                self.ticks_callejon = 0
            else:
                self.giro_comprometido = False

        # --- APLICACIÓN DE VELOCIDADES ---
        if self.estado == 'pasillo':
            twist.linear.x = VEL_LINEAR_PASILLO if self.d_front >= DIST_GIRO_PASILLO else 0.0
        
        elif self.estado == 'retroceder':
            twist.linear.x = -VEL_RETROCESO if self.d_back > DIST_SEGURIDAD_TRASERA else 0.0
        
        elif self.estado == 'girar_izq':
            # SEGURIDAD GIRO IZQ: Reduce velocidad si la pared izquierda está muy cerca
            twist.angular.z = VEL_GIRO if self.d_left > DIST_LATERAL_SEGURIDAD else VEL_GIRO * 0.5
            twist.linear.x = VEL_AVANCE_GIRO if self.d_front > 0.22 else 0.0
            
        elif self.estado == 'girar_der':
            # SEGURIDAD GIRO DER: Reduce velocidad si la pared derecha está muy cerca
            twist.angular.z = -VEL_GIRO if self.d_right > DIST_LATERAL_SEGURIDAD else -VEL_GIRO * 0.5
            twist.linear.x = VEL_AVANCE_GIRO if self.d_front > 0.22 else 0.0
            
        else: # Avanzar
            vel = self.velocidad_frenada(self.d_front, VEL_LINEAR_NORMAL)
            twist.linear.x = vel
            error = DIST_PARED_DERECHA - self.d_right
            twist.angular.z = max(min(KP * error, 0.40), -0.40)

        self.vel_lin_pub, self.vel_ang_pub = twist.linear.x, twist.angular.z
        self._log_tick()
        self.cmd_pub.publish(twist)

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