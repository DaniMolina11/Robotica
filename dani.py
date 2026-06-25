#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time
from collections import deque

# --- PARÁMETROS DE TU CÓDIGO ORIGINAL ---
DIST_GIRO_PASILLO      = 0.32   
DIST_PARAR_GIRO        = 0.32
DIST_FRENAR            = 0.55   
DIST_PARED_DERECHA     = 0.25   
DIST_PASILLO           = 0.45   
DIST_ESQUINA_CERRADA   = 0.20   
DIST_SEGURIDAD_TRASERA = 0.15   

VEL_LINEAR_PASILLO    = 0.06
VEL_LINEAR_NORMAL     = 0.08
VEL_RETROCESO         = 0.05
VEL_GIRO              = 0.28   
VEL_AVANCE_GIRO       = 0.06   
KP                    = 1.2

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
        self.yaw   = 0.0  # Orientación precisa para guiar el retroceso exacto
        self.lecturas_acumuladas = 0

        self.estado          = 'esperando'
        self.estado_anterior = 'esperando'

        self.tiempo_inicio_giro      = 0.0
        self.giro_comprometido       = False
        self.ticks_fuera_pasillo     = 0

        # --- SISTEMA DE MEMORIA DE COORDENADAS ---
        self.path_history           = []  # Lista de puntos (x,y) recorridos en positivo
        self.punto_inicio_retroceso = None # Dónde empezó la marcha atrás
        self.callejones_visitados   = []  # Blacklist de callejones completados

        self.META_X               = 2.75
        self.META_Y               = 1.71
        self.DISTANCIA_MINIMA_META = 0.25
        self.meta_alcanzada       = False

        self.sim_time    = 0.0
        self.vel_lin_pub = 0.0
        self.vel_ang_pub = 0.0

        self.log_file = open(LOG_FILE, 'w')
        self._log_raw('=== INICIO SESION MAZE SOLVER TRAZADO REVERSO ===')

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
        self._log_raw(f'*** {msg} | estado={self.estado} pos=({self.pos_x:.2f},{self.pos_y:.2f}) F={self.d_front:.2f}')

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
        
        # Obtención del ángulo Yaw (Orientación) desde la odometría
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        dist = math.sqrt((self.pos_x - self.META_X)**2 + (self.pos_y - self.META_Y)**2)
        if dist < self.DISTANCIA_MINIMA_META and not self.meta_alcanzada:
            self.meta_alcanzada = True
            self._log_evento(f'META ALCANZADA dist={dist:.3f}')

    def _camino_conduce_a_callejon(self, lado):
        """Verifica si tomar un pasillo lateral nos mete de cabeza en un callejón guardado en memoria"""
        angulo = self.yaw + (math.pi/2.0 if lado == 'izq' else -math.pi/2.0)
        px = self.pos_x + 0.55 * math.cos(angulo)
        py = self.pos_y + 0.55 * math.sin(angulo)
        for cx, cy in self.callejones_visitados:
            if math.sqrt((px - cx)**2 + (py - cy)**2) < 0.45:
                return True
        return False

    def _cambiar_estado(self, nuevo, motivo=''):
        if nuevo != self.estado:
            self._log_evento(f'ESTADO {self.estado} -> {nuevo}  [{motivo}]')
            self.estado_anterior = self.estado
            self.estado = nuevo

    def _decidir_lado_giro(self):
        en_pasillo = (self.d_right < DIST_PASILLO and self.d_left < DIST_PASILLO)
        if en_pasillo:
            lado = 'izq' if self.d_diag_izq >= self.d_diag_der else 'der'
        else:
            lado = 'izq' if self.d_left >= self.d_right else 'der'
            
        # Antibucle por memoria de coordenadas
        if lado == 'izq' and self._camino_conduce_a_callejon('izq'):
            self._log_evento('ANTIBUCLE: Izquierda fichada como callejón ciego. Forzando DERECHA.')
            lado = 'der'
        elif lado == 'der' and self._camino_conduce_a_callejon('der'):
            self._log_evento('ANTIBUCLE: Derecha fichada como callejón ciego. Forzando IZQUIERDA.')
            lado = 'izq'
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
            return

        if self.estado == 'esperando':
            self._cambiar_estado('avanzar', 'lecturas listas')

        # --- GRABADORA DE COORDENADAS (HUELLAS) ---
        if self.estado in ('avanzar', 'pasillo'):
            if not self.path_history:
                self.path_history.append((self.pos_x, self.pos_y))
            else:
                lx, ly = self.path_history[-1]
                if math.sqrt((self.pos_x - lx)**2 + (self.pos_y - ly)**2) > 0.05:
                    self.path_history.append((self.pos_x, self.pos_y))

        d_f = self.d_front
        d_r = self.d_right
        d_l = self.d_left

        ahora          = time.time()
        tiempo_girando = ahora - self.tiempo_inicio_giro
        en_pasillo     = (d_r < DIST_PASILLO and d_l < DIST_PASILLO)

        # --- DETECCIÓN DE CALLEJÓN INICIAL ---
        if self.estado in ('avanzar', 'pasillo'):
            callejon_muerto = (d_f <= 0.24 and d_l < 0.30 and d_r < 0.30)
            if callejon_muerto:
                self._cambiar_estado('retroceder', 'callejon detectado de frente')
                self.punto_inicio_retroceso = (self.pos_x, self.pos_y)
                if not any(math.sqrt((self.pos_x - cx)**2 + (self.pos_y - cy)**2) < 0.35 for cx, cy in self.callejones_visitados):
                    self.callejones_visitados.append((self.pos_x, self.pos_y))
                self.giro_comprometido = False
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
                    self._cambiar_estado('avanzar', 'salida pasillo confirmada')
                    self.ticks_fuera_pasillo = 0

        elif self.estado == 'avanzar':
            if en_pasillo and d_f >= DIST_GIRO_PASILLO:
                self._cambiar_estado('pasillo', 'pasillo detectado')
                self.ticks_fuera_pasillo = 0
            elif d_f < DIST_PARAR_GIRO:
                self._iniciar_giro(ahora)

        elif self.estado in ('girar_izq', 'girar_der'):
            if self.giro_comprometido:
                if tiempo_girando >= TIEMPO_GIRO_MINIMO:
                    self.giro_comprometido = False
            else:
                if d_f >= DIST_PARAR_GIRO + 0.10:
                    self._cambiar_estado('avanzar', 'frente libre tras giro')
                elif d_f < DIST_PARAR_GIRO - 0.05:
                    # --- SOLUCIÓN AL BUG DE LA OSCILACIÓN ---
                    # Si ha pasado el tiempo mínimo y el frente sigue colapsado, significa que el giro ha fallado
                    # porque el robot está atrapado. Forzamos la marcha atrás inmediata sin oscilar más.
                    self._cambiar_estado('retroceder', 'Giro fallido por bloqueo físico. Activando retroceso.')
                    self.punto_inicio_retroceso = (self.pos_x, self.pos_y)
                    if not any(math.sqrt((self.pos_x - cx)**2 + (self.pos_y - cy)**2) < 0.35 for cx, cy in self.callejones_visitados):
                        self.callejones_visitados.append((self.pos_x, self.pos_y))
                    self.reset_filtros()
                    return

        elif self.estado == 'retroceder':
            if self.path_history:
                # Vamos extrayendo los puntos de la trayectoria en sentido inverso
                target_x, target_y = self.path_history[-1]
                distance = math.sqrt((target_x - self.pos_x)**2 + (target_y - self.pos_y)**2)
                
                while distance < 0.07 and len(self.path_history) > 1:
                    self.path_history.pop()
                    target_x, target_y = self.path_history[-1]
                    distance = math.sqrt((target_x - self.pos_x)**2 + (target_y - self.pos_y)**2)

                # Comprobamos la distancia recorrida desde el fondo del rincón
                dist_desde_inicio = 0.0
                if self.punto_inicio_retroceso:
                    ix, iy = self.punto_inicio_retroceso
                    dist_desde_inicio = math.sqrt((self.pos_x - ix)**2 + (self.pos_y - iy)**2)

                # Solo evaluamos salir de la marcha atrás si nos hemos alejado más de 0.40m (morro libre)
                if dist_desde_inicio > 0.40:
                    izq_despejada = (d_l > 0.40) and not self._camino_conduce_a_callejon('izq')
                    der_despejada = (d_r > 0.40) and not self._camino_conduce_a_callejon('der')
                    
                    if izq_despejada or der_despejada:
                        lado = 'izq' if d_l > d_r else 'der'
                        self._cambiar_estado(f'girar_{lado}', f'Saliendo del callejón marcha atrás hacia {lado}')
                        self.tiempo_inicio_giro = ahora
                        self.giro_comprometido  = True
                        return

        # --- GENERACIÓN DE VELOCIDADES ---
        evento = ''
        if self.estado == 'pasillo':
            twist.linear.x  = VEL_LINEAR_PASILLO
            twist.angular.z = 0.0
            evento = 'pasillo_recto'
            
        elif self.estado == 'retroceder':
            if self.path_history:
                # Control geométrico de marcha atrás sobre carril de huellas
                target_x, target_y = self.path_history[-1]
                dx = target_x - self.pos_x
                dy = target_y - self.pos_y
                target_angle = math.atan2(dy, dx)
                
                error_angle = target_angle - (self.yaw + math.pi)
                error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))
                
                twist.linear.x = -VEL_RETROCESO
                twist.angular.z = 1.2 * error_angle # Alineación magnética milimétrica con la huella anterior
                evento = 'deshaciendo_pasos'
            else:
                twist.linear.x = -VEL_RETROCESO
                twist.angular.z = 0.0
                evento = 'retro_lineal_fallback'
            
        elif self.estado == 'girar_izq':
            twist.linear.x  = VEL_AVANCE_GIRO if d_f > 0.22 else 0.0
            twist.angular.z = VEL_GIRO
            evento = 'girar_izq'
            
        elif self.estado == 'girar_der':
            twist.linear.x  = VEL_AVANCE_GIRO if d_f > 0.22 else 0.0
            twist.angular.z = -VEL_GIRO
            evento = 'girar_der'
            
        else:  # avanzar
            vel = self.velocidad_frenada(d_f, VEL_LINEAR_NORMAL)
            twist.linear.x = vel
            if d_r > 1.2:
                if self._camino_conduce_a_callejon('der'):
                    twist.angular.z = 0.0  # Pasa de largo ignorando el pasillo ciego fichado
                    evento = 'Ignorando callejón por blacklist'
                else:
                    twist.angular.z = -0.20
                    evento = 'buscando_muro'
            else:
                error = DIST_PARED_DERECHA - d_r
                twist.angular.z = max(min(KP * error, 0.40), -0.40)
                evento = 'siguiendo_muro_der'

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
        nodo._log_raw('=== INTERRUPCION USUARIO ===')
        nodo.cmd_pub.publish(Twist())
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()