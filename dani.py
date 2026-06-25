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
DIST_SEGURIDAD_TRASERA = 0.16   

VEL_LINEAR_PASILLO    = 0.06
VEL_LINEAR_NORMAL     = 0.08
VEL_RETROCESO         = 0.05
VEL_GIRO              = 0.45   # Aumentado para mayor agilidad y respuesta
KP                    = 1.2

TIEMPO_GIRO_MINIMO    = 1.0    # Ajustado a la nueva velocidad de giro acelerada
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
        self.lecturas_acumuladas = 0

        self.estado          = 'esperando'
        self.estado_anterior = 'esperando'

        self.tiempo_inicio_giro      = 0.0
        self.giro_comprometido       = False
        self.ticks_fuera_pasillo     = 0

        # Módulos de memoria de carril trasero
        self.path_history           = []  
        self.punto_inicio_retroceso = None 
        self.callejones_visitados   = []  

        self.META_X               = 2.75
        self.META_Y               = 1.71
        self.DISTANCIA_MINIMA_META = 0.25
        self.meta_alcanzada       = False

        self.sim_time    = 0.0
        self.vel_lin_pub = 0.0
        self.vel_ang_pub = 0.0

        self.log_file = open(LOG_FILE, 'w')
        self._log_raw('=== INICIO SESION MAZE SOLVER PIVOTE FIJO ===')

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
        
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        dist = math.sqrt((self.pos_x - self.META_X)**2 + (self.pos_y - self.META_Y)**2)
        if dist < self.DISTANCIA_MINIMA_META and not self.meta_alcanzada:
            self.meta_alcanzada = True

    def _camino_conduce_a_callejon(self, lado):
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
            
        if lado == 'izq' and self._camino_conduce_a_callejon('izq'):
            lado = 'der'
        elif lado == 'der' and self._camino_conduce_a_callejon('der'):
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

        # Registro continuo de migas de pan (Huellas)
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

        # --- REGLA PROTEGIDA EN LÍNEA RECTA ---
        if self.estado in ('avanzar', 'pasillo'):
            # El callejón verídico detiene el avance lineal inmediatamente y activa retroceder
            callejon_muerto = (d_f <= 0.32 and d_l < 0.35 and d_r < 0.35)
            if callejon_muerto:
                self._cambiar_estado('retroceder', 'callejon detectado de frente')
                self.punto_inicio_retroceso = (self.pos_x, self.pos_y)
                if not any(math.sqrt((self.pos_x - cx)**2 + (self.pos_y - cy)**2) < 0.35 for cx, cy in self.callejones_visitados):
                    self.callejones_visitados.append((self.pos_x, self.pos_y))
                self.giro_comprometido = False
                self.reset_filtros()  
                return

        if self.estado in ('avanzar', 'pasillo'):
            esquina_cerrada = (d_f < DIST_ESQUINA_CERRADA and d_r < DIST_ESQUINA_CERRADA + 0.05 and d_l < DIST_ESQUINA_CERRADA + 0.05)
            if esquina_cerrada:
                self._cambiar_estado('retroceder', 'emergencia: esquina cerrada')
                self.punto_inicio_retroceso = (self.pos_x, self.pos_y)
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
                # REPARADO: Solo salimos si el frente está verdaderamente despejado encarando el pasillo nuevo
                if d_f >= DIST_PARAR_GIRO + 0.08:
                    self._cambiar_estado('avanzar', 'frente libre tras giro')

        elif self.estado == 'retroceder':
            if self.path_history:
                target_x, target_y = self.path_history[-1]
                distance = math.sqrt((target_x - self.pos_x)**2 + (target_y - self.pos_y)**2)
                
                while distance < 0.07 and len(self.path_history) > 1:
                    self.path_history.pop()
                    target_x, target_y = self.path_history[-1]
                    distance = math.sqrt((target_x - self.pos_x)**2 + (target_y - self.pos_y)**2)

                dist_desde_inicio = 0.0
                if self.punto_inicio_retroceso:
                    ix, iy = self.punto_inicio_retroceso
                    dist_desde_inicio = math.sqrt((self.pos_x - ix)**2 + (self.pos_y - iy)**2)

                # Si ya reculó lo suficiente despejando el morro, o el escudo trasero roza un muro, evalúa salir
                if dist_desde_inicio > 0.38 or self.d_back <= DIST_SEGURIDAD_TRASERA:
                    izq_despejada = (d_l > 0.40) and not self._camino_conduce_a_callejon('izq')
                    der_despejada = (d_r > 0.40) and not self._camino_conduce_a_callejon('der')
                    
                    if izq_despejada or der_despejada:
                        lado = 'izq' if d_l > d_r else 'der'
                        self._cambiar_estado(f'girar_{lado}', f'Saliendo del callejón hacia {lado}')
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
                target_x, target_y = self.path_history[-1]
                dx = target_x - self.pos_x
                dy = target_y - self.pos_y
                target_angle = math.atan2(dy, dx)
                
                error_angle = target_angle - (self.yaw + math.pi)
                error_angle = math.atan2(math.sin(error_angle), math.cos(error_angle))
                
                # ESCUDO TRASERO ACTIVO: Si d_back baja del umbral crítico de seguridad, frena en seco linealmente
                if self.d_back > DIST_SEGURIDAD_TRASERA:
                    twist.linear.x = -VEL_RETROCESO
                else:
                    twist.linear.x = 0.0  
                    evento = 'ESCUDO TRASERO ACTIVADO - COLISION EVITADA'
                
                twist.angular.z = 1.2 * error_angle 
                if not evento: evento = 'deshaciendo_pasos'
            else:
                if self.d_back > DIST_SEGURIDAD_TRASERA:
                    twist.linear.x = -VEL_RETROCESO
                else:
                    twist.linear.x = 0.0
                twist.angular.z = 0.0
                evento = 'retro_lineal_fallback'
            
        elif self.estado == 'girar_izq':
            # REPARADO: PIVOTE PURO IN-SITU (linear.x = 0.0). No avanza nada para no colisionar con esquinas
            twist.linear.x  = 0.0  
            twist.angular.z = VEL_GIRO
            evento = 'pivote_izq'
            
        elif self.estado == 'girar_der':
            # REPARADO: PIVOTE PURO IN-SITU (linear.x = 0.0). No avanza nada para no colisionar con esquinas
            twist.linear.x  = 0.0  
            twist.angular.z = -VEL_GIRO
            evento = 'pivote_der'
            
        else:  # avanzar
            vel = self.velocidad_frenada(d_f, VEL_LINEAR_NORMAL)
            twist.linear.x = vel
            if d_r > 1.2:
                if self._camino_conduce_a_callejon('der'):
                    twist.angular.z = 0.0  
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