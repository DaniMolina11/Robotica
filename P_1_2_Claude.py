#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time
from collections import deque

# --- PARÁMETROS OPTIMIZADOS (MÁS LENTOS Y PRECISOS) ---
DIST_GIRO_PASILLO      = 0.32   
DIST_PARAR_GIRO        = 0.32
DIST_FRENAR            = 0.50   
DIST_PARED_DERECHA     = 0.25   
DIST_PASILLO           = 0.45   

# Umbrales estrictos para el callejón sin salida
CALLEJON_FRONT         = 0.32
CALLEJON_LATERAL       = 0.35

# VELOCIDADES AJUSTADAS (Modo seguro activado)
VEL_LINEAR_PASILLO    = 0.04   # Más lento para un centrado perfecto
VEL_LINEAR_NORMAL     = 0.05   # Velocidad de crucero reducida
VEL_AVANCE_GIRO       = 0.02   # Avance mínimo en curvas para no chocar
VEL_GIRO              = 0.22   # Rotación suave y controlable

KP                    = 1.0
KP_PASILLO            = 0.5    # Ganancia amortiguada para eliminar oscilaciones

TIEMPO_GIRO_MINIMO    = 1.5
N_LECTURAS_PROMEDIO   = 5
TICKS_CONFIRMACION    = 4

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

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw   = 0.0
        self.yaw_inicial = 0.0
        self.lecturas_acumuladas = 0

        self.estado          = 'esperando'
        self.estado_anterior = 'esperando'

        self.tiempo_inicio_giro      = 0.0
        self.giro_comprometido       = False
        self.ticks_fuera_pasillo     = 0

        self.META_X               = 2.75
        self.META_Y               = 1.71
        self.DISTANCIA_MINIMA_META = 0.25
        self.meta_alcanzada       = False

        self.sim_time    = 0.0
        self.vel_lin_pub = 0.0
        self.vel_ang_pub = 0.0

        self.log_file = open(LOG_FILE, 'w')
        self._log_raw('=== INICIO SESION MAZE SOLVER ===')
        self._log_raw(
            'TSIM      | POS_X  | POS_Y  | ESTADO       | '
            'F     | R     | L     | VL      | VA      | EVENTO'
        )
        self._log_raw('-' * 110)

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

    def _log_evento(self, msg):
        self._log_raw(f'*** {msg} | F={self.d_front:.2f} R={self.d_right:.2f} L={self.d_left:.2f}')

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
        if d_front >= DIST_FRENAR:
            return vel_max
        if d_front <= DIST_PARAR_GIRO:
            return 0.0
        ratio = (d_front - DIST_PARAR_GIRO) / (DIST_FRENAR - DIST_PARAR_GIRO)
        return round(vel_max * ratio, 3)

    def scan_callback(self, msg):
        r = msg.ranges
        if len(r) < 360:
            return
        self.buf_front.append(min(self.sector_min(r, 350, 360), self.sector_min(r, 0, 10)))
        self.buf_right.append(   self.sector_min(r, 260, 310))
        self.buf_left.append(    self.sector_min(r,  50, 110))
        self.buf_back.append(    self.sector_min(r, 170, 190))
        self.buf_diag_izq.append(self.sector_min(r,  30,  60))
        self.buf_diag_der.append(self.sector_min(r, 300, 330))
        
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
        
        # Orientación precisa (Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)
        
        dist = math.sqrt((self.pos_x - self.META_X)**2 + (self.pos_y - self.META_Y)**2)
        if dist < self.DISTANCIA_MINIMA_META and not self.meta_alcanzada:
            self.meta_alcanzada = True
            self._log_evento(f'META ALCANZADA dist={dist:.3f}')

    def _cambiar_estado(self, nuevo, motivo=''):
        if nuevo != self.estado:
            self._log_evento(f'ESTADO {self.estado} -> {nuevo}  [{motivo}]')
            self.estado_anterior = self.estado
            self.estado = nuevo

    def _decidir_lado_giro(self):
        # Corregido: Solo se prioriza la derecha si hay un hueco real (no un muro a 0.30m)
        if self.d_right > 0.45:
            lado = 'der'
            self._log_evento(f'Giro decidido: DERECHA LIBRE (d_r={self.d_right:.2f})')
        elif self.d_left > 0.45:
            lado = 'izq'
            self._log_evento(f'Giro decidido: IZQUIERDA LIBRE (d_l={self.d_left:.2f})')
        else:
            # Si ambos lados están ajustados, desempata el que ofrezca más margen real
            lado = 'der' if self.d_right >= self.d_left else 'izq'
            self._log_evento(f'Giro por desempate de muros: lado={lado}')
        return lado

    def _iniciar_giro(self, ahora):
        lado = self._decidir_lado_giro()
        self._cambiar_estado(f'girar_{lado}', f'giro lado={lado}')
        self.tiempo_inicio_giro = ahora
        self.giro_comprometido  = True

    def control_loop(self):
        twist = Twist()

        if self.meta_alcanzada:
            self.cmd_pub.publish(twist)
            return

        if self.lecturas_acumuladas < N_LECTURAS_PROMEDIO:
            self.cmd_pub.publish(twist)
            return

        if self.estado == 'esperando':
            self._cambiar_estado('avanzar', 'lecturas listas')

        d_f = self.d_front
        d_r = self.d_right
        d_l = self.d_left

        ahora          = time.time()
        tiempo_girando = ahora - self.tiempo_inicio_giro
        en_pasillo     = (d_r < DIST_PASILLO and d_l < DIST_PASILLO)

        # --- DETECCIÓN DE CALLEJÓN SIN SALIDA (Prioridad Absoluta) ---
        if self.estado in ('avanzar', 'pasillo'):
            if d_f < CALLEJON_FRONT and d_l < CALLEJON_LATERAL and d_r < CALLEJON_LATERAL:
                self._cambiar_estado('girar_180', 'Callejón cerrado: Activando escape de 180 grados')
                self.yaw_inicial = self.yaw
                self.tiempo_inicio_giro = ahora
                self.reset_filtros()
                return

        # --- MÁQUINA DE ESTADOS ---
        if self.estado == 'pasillo':
            if en_pasillo:
                self.ticks_fuera_pasillo = 0
                if d_f < DIST_GIRO_PASILLO:
                    self._iniciar_giro(ahora)
            else:
                self.ticks_fuera_pasillo += 1
                if self.ticks_fuera_pasillo >= TICKS_CONFIRMACION:
                    # Al salir del pasillo evaluamos de forma inteligente cuál lado se abrió de verdad
                    if d_r > DIST_PASILLO:
                        self._cambiar_estado('girar_der', 'El pasillo se abre a la derecha (Priorizado)')
                        self.tiempo_inicio_giro = ahora
                        self.giro_comprometido = True
                    elif d_l > DIST_PASILLO:
                        self._cambiar_estado('girar_izq', 'El pasillo se abre a la izquierda')
                        self.tiempo_inicio_giro = ahora
                        self.giro_comprometido = True
                    else:
                        self._cambiar_estado('avanzar', 'Salida de pasillo neutra')
                    self.ticks_fuera_pasillo = 0

        elif self.estado == 'avanzar':
            if en_pasillo and d_f >= DIST_GIRO_PASILLO:
                self._cambiar_estado('pasillo', 'Entrando en pasillo')
                self.ticks_fuera_pasillo = 0
            elif d_f < DIST_PARAR_GIRO:
                self._iniciar_giro(ahora)
            elif d_r > 0.55 and d_f > 0.40: # Apertura clara a la derecha en zona abierta
                self._cambiar_estado('girar_der', 'Muro derecho finalizado -> Girar a la derecha')
                self.tiempo_inicio_giro = ahora
                self.giro_comprometido = True

        elif self.estado in ('girar_izq', 'girar_der'):
            if self.giro_comprometido:
                if tiempo_girando >= TIEMPO_GIRO_MINIMO:
                    self.giro_comprometido = False
            else:
                if d_f >= DIST_PARAR_GIRO + 0.10:
                    self._cambiar_estado('avanzar', f'Frente despejado d_f={d_f:.2f}')
                elif d_f < DIST_PARAR_GIRO - 0.05:
                    self._iniciar_giro(ahora)

        elif self.estado == 'girar_180':
            diff_angular = self.yaw - self.yaw_inicial
            diff_angular = math.atan2(math.sin(diff_angular), math.cos(diff_angular))
            
            # Condición de salida: haber rotado ~180° (>2.9 rad) y tener frente libre
            if tiempo_girando > 2.0 and abs(diff_angular) > 2.90 and d_f > 0.45:
                self._cambiar_estado('avanzar', 'Giro de 180 grados completado con éxito')
            elif tiempo_girando > 7.0 and d_f > 0.45: # Seguridad por tiempo
                self._cambiar_estado('avanzar', 'Giro de 180 grados completado por tiempo límite')

        # --- CONFIGURACIÓN DE VELOCIDADES ---
        evento = ''
        if self.estado == 'pasillo':
            # CONTROL DE CENTRADO AMORTIGUADO
            error_pasillo = d_l - d_r
            twist.linear.x = VEL_LINEAR_PASILLO
            
            # Aplicamos zona muerta de 4cm para evitar micro-oscilaciones ruidosas
            if abs(error_pasillo) < 0.04:
                twist.angular.z = 0.0
                evento = f'pasillo_recto_puro (error mínimo: {error_pasillo:.3f})'
            else:
                twist.angular.z = max(min(KP_PASILLO * error_pasillo, 0.12), -0.12)
                evento = f'pasillo_centrando err={error_pasillo:.3f}'
            
        elif self.estado == 'girar_180':
            twist.linear.x  = 0.0
            twist.angular.z = -VEL_GIRO # Gira sobre su propio eje hacia la derecha
            evento = 'Ejecutando rotación 180°'
            
        elif self.estado == 'girar_izq':
            twist.linear.x  = VEL_AVANCE_GIRO if d_f > 0.22 else 0.0
            twist.angular.z = VEL_GIRO
            evento = 'Giro Izquierda'
            
        elif self.estado == 'girar_der':
            twist.linear.x  = VEL_AVANCE_GIRO if d_f > 0.22 else 0.0
            twist.angular.z = -VEL_GIRO
            evento = 'Giro Derecha'
            
        else:  # avanzar
            vel = self.velocidad_frenada(d_f, VEL_LINEAR_NORMAL)
            twist.linear.x = vel
            if d_r > 1.0:
                twist.angular.z = -0.15 # Giro suave a la derecha para recuperar contacto con el muro
                evento = 'Buscando pared derecha'
            else:
                error = DIST_PARED_DERECHA - d_r
                twist.angular.z = max(min(KP * error, 0.25), -0.25)
                evento = f'Siguiendo pared derech err={error:.3f}'

        self.vel_lin_pub = twist.linear.x
        self.vel_ang_pub = twist.angular.z
        self._log_tick(evento)
        self.cmd_pub.publish(twist)

    def __del__(self):
        try:
            self.log_file.close()
        except Exception:
            pass

def main(args=None):
    rclpy.init(args=args)
    nodo = MazeSolver()
    try:
        rclpy.spin(nodo)
    except KeyboardInterrupt:
        pass
    finally:
        nodo.cmd_pub.publish(Twist())
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()