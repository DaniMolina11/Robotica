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
        self.yaw   = 0.0  # NUEVO: Seguimiento de orientación angular para la proyección de caminos
        self.lecturas_acumuladas = 0

        self.estado          = 'esperando'
        self.estado_anterior = 'esperando'

        self.tiempo_inicio_giro      = 0.0
        self.giro_comprometido       = False
        self.ticks_fuera_pasillo     = 0

        # NUEVO: Memoria espacial para descarte de caminos y antibucle
        self.dead_ends        = []  # Lista de coordenadas (x,y) de callejones confirmados
        self.visited_points   = []  # Historial de posiciones para mapa de calor
        self.last_track_time  = 0.0

        self.META_X               = 2.75
        self.META_Y               = 1.71
        self.DISTANCIA_MINIMA_META = 0.25
        self.meta_alcanzada       = False

        self.sim_time    = 0.0
        self.vel_lin_pub = 0.0
        self.vel_ang_pub = 0.0

        self.log_file = open(LOG_FILE, 'w')
        self._log_raw('=== INICIO SESION MAZE SOLVER CON MEMORIA ESPACIAL ===')
        self._log_raw(
            f'Params: DIST_GIRO={DIST_GIRO_PASILLO} DIST_PARAR={DIST_PARAR_GIRO} '
            f'VEL_NORMAL={VEL_LINEAR_NORMAL} VEL_GIRO={VEL_GIRO} VEL_AVANCE_GIRO={VEL_AVANCE_GIRO}'
        )
        self._log_raw(
            'TSIM      | POS_X  | POS_Y  | ESTADO       | '
            'F     | R     | L     | VL      | VA      | EVENTO'
        )
        self._log_raw('-' * 130)

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
        self._log_raw(
            f'*** {msg} | estado={self.estado} '
            f'pos=({self.pos_x:.3f},{self.pos_y:.3f}) '
            f'F={self.d_front:.2f} R={self.d_right:.2f} L={self.d_left:.2f}'
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
        
        # Extracción matemática del ángulo Yaw (Orientación real del robot en el plano)
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

        dist = math.sqrt((self.pos_x - self.META_X)**2 + (self.pos_y - self.META_Y)**2)
        if dist < self.DISTANCIA_MINIMA_META and not self.meta_alcanzada:
            self.meta_alcanzada = True
            self._log_evento(f'META ALCANZADA dist={dist:.3f}')

    def _registrar_trayectoria(self):
        """NUEVO: Registra la posición del robot cada 0.3s para crear el mapa de calor de visitas"""
        if self.sim_time - self.last_track_time > 0.3:
            if self.estado in ('avanzar', 'pasillo'):
                self.visited_points.append((self.pos_x, self.pos_y))
                self.last_track_time = self.sim_time

        # Mantener el buffer acotado para optimizar rendimiento
        if len(self.visited_points) > 400:
            self.visited_points.pop(0)

    def _obtener_visitas_en_direccion(self, angulo_relativo):
        """NUEVO: Cuenta cuántas veces se ha explorado un pasillo proyectando un punto de mira a 0.55m"""
        ang_pred = self.yaw + angulo_relativo
        x_pred = self.pos_x + 0.55 * math.cos(ang_pred)
        y_pred = self.pos_y + 0.55 * math.sin(ang_pred)
        
        contador = 0
        for vx, vy in self.visited_points:
            if math.sqrt((x_pred - vx)**2 + (y_pred - vy)**2) < 0.35:
                contador += 1
        return contador

    def _ruta_conduce_a_callejon(self, angulo_relativo):
        """NUEVO: Comprueba si un pasillo opcional está en la blacklist de callejones sin salida confirmados"""
        ang_pred = self.yaw + angulo_relativo
        x_pred = self.pos_x + 0.55 * math.cos(ang_pred)
        y_pred = self.pos_y + 0.55 * math.sin(ang_pred)
        
        for cx, cy in self.dead_ends:
            if math.sqrt((x_pred - cx)**2 + (y_pred - cy)**2) < 0.45:
                return True
        return False

    def _cambiar_estado(self, nuevo, motivo=''):
        if nuevo != self.estado:
            self._log_evento(f'ESTADO {self.estado} -> {nuevo}  [{motivo}]')
            self.estado_anterior = self.estado
            self.estado = nuevo

    def _decidir_lado_giro(self):
        en_pasillo = (self.d_right < DIST_PASILLO and self.d_left < DIST_PASILLO)
        
        # Filtros de memoria predictiva antes de girar en cruces abiertos
        izq_bloqueada = self._ruta_conduce_a_callejon(math.pi/2)
        der_bloqueada = self._ruta_conduce_a_callejon(-math.pi/2)
        visitas_izq   = self._obtener_visitas_en_direccion(math.pi/2)
        visitas_der   = self._obtener_visitas_en_direccion(-math.pi/2)

        if en_pasillo:
            lado = 'izq' if self.d_diag_izq >= self.d_diag_der else 'der'
            self._log_evento(f'Giro en PASILLO por diagonal: lado={lado}')
        else:
            lado = 'izq' if self.d_left >= self.d_right else 'der'
            self._log_evento(f'Giro NORMAL: lado={lado}')
        
        # CONTROL ANTIBUCLE: Si la dirección elegida es un callejón o está saturada, cambia al flanco alternativo
        if lado == 'izq' and (izq_bloqueada or visitas_izq > visitas_der + 8):
            if not der_bloqueada:
                self._log_evento(f'ANTIBUCLE: Descartando IZQUIERDA (Callejón o quemado: I:{visitas_izq} vs D:{visitas_der}). Forzando DER.')
                lado = 'der'
        elif lado == 'der' and (der_bloqueada or visitas_der > visitas_izq + 8):
            if not izq_bloqueada:
                self._log_evento(f'ANTIBUCLE: Descartando DERECHA (Callejón o quemado: D:{visitas_der} vs I:{visitas_izq}). Forzando IZQ.')
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
            self._log_tick('META - detenido')
            self.cmd_pub.publish(twist)
            return

        if self.lecturas_acumuladas < N_LECTURAS_PROMEDIO:
            self._log_tick(f'acumulando {self.lecturas_acumuladas}/{N_LECTURAS_PROMEDIO}')
            self.cmd_pub.publish(twist)
            return

        if self.estado == 'esperando':
            self._cambiar_estado('avanzar', 'lecturas listas')

        # Registrar la posición actual en la base de datos temporal
        self._registrar_trayectoria()

        d_f = self.d_front
        d_r = self.d_right
        d_l = self.d_left

        ahora          = time.time()
        tiempo_girando = ahora - self.tiempo_inicio_giro
        en_pasillo     = (d_r < DIST_PASILLO and d_l < DIST_PASILLO)

        # --- REGLA PROTEGIDA EN LÍNEA RECTA ---
        if self.estado in ('avanzar', 'pasillo'):
            callejon_muerto = (d_f <= 0.24 and d_l < 0.32 and d_r < 0.32)
            if callejon_muerto:
                self._cambiar_estado('retroceder', 'callejon detectado (frente y laterales bloqueados)')
                
                # GRABACIÓN DE BLACKLIST: Registramos el punto exacto del callejón sin salida para evitar re-entradas
                if not any(math.sqrt((self.pos_x - cx)**2 + (self.pos_y - cy)**2) < 0.35 for cx, cy in self.dead_ends):
                    self.dead_ends.append((self.pos_x, self.pos_y))
                    self._log_evento(f'AÑADIDO A BLACKLIST DE CALLEJONES: ({self.pos_x:.2f}, {self.pos_y:.2f})')

                self.giro_comprometido = False
                self.reset_filtros()  # Vaciamos los buffers antiguos para que el láser responda al milisegundo
                return

        if self.estado in ('avanzar', 'pasillo'):
            esquina_cerrada = (d_f < DIST_ESQUINA_CERRADA and
                               d_r < DIST_ESQUINA_CERRADA + 0.05 and
                               d_l < DIST_ESQUINA_CERRADA + 0.05)
            if esquina_cerrada:
                self._cambiar_estado('retroceder', 'emergencia: esquina cerrada')
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

        elif self.estado == 'retroceder':
            # SOLUCIÓN CRÍTICA DE RETROCEDER: Obligamos a desahogar el frente (>0.52m) antes de evaluar salidas.
            # Esto evita que el robot lea de forma prematura el pasillo lateral estando encajonado en el fondo.
            if d_f > 0.52:
                if d_l > 0.40 or d_r > 0.40:
                    # Validamos qué caminos son limpios y no conducen de vuelta a la blacklist
                    izq_valida = (d_l > 0.40) and not self._ruta_conduce_a_callejon(math.pi/2)
                    der_valida = (d_r > 0.40) and not self._ruta_conduce_a_callejon(-math.pi/2)
                    
                    visitas_izq = self._obtener_visitas_en_direccion(math.pi/2) if izq_valida else 9999
                    visitas_der = self._obtener_visitas_en_direccion(-math.pi/2) if der_valida else 9999
                    
                    if izq_valida or der_valida:
                        # Seleccionamos la alternativa que menos hayamos pisado (Menos quemada en el mapa de calor)
                        lado = 'izq' if visitas_izq <= visitas_der else 'der'
                        self._cambiar_estado(f'girar_{lado}', f'Salida de descarte encontrada hacia {lado} (Historial I:{visitas_izq} D:{visitas_der})')
                        self.tiempo_inicio_giro = ahora
                        self.giro_comprometido  = True
                    else:
                        # Si ambos pasillos adyacentes mueren en callejones ya completados,
                        # el robot ignora el cruce y continúa retrocediendo en línea recta hacia tramos anteriores del laberinto.
                        pass

        elif self.estado in ('girar_izq', 'girar_der'):
            if self.giro_comprometido:
                if tiempo_girando >= TIEMPO_GIRO_MINIMO:
                    self.giro_comprometido = False
            else:
                if d_f >= DIST_PARAR_GIRO + 0.10:
                    self._cambiar_estado('avanzar', f'frente libre d_f={d_f:.2f}')
                elif d_f < DIST_PARAR_GIRO - 0.05:
                    self._iniciar_giro(ahora)

        elif self.estado == 'escape':
            if d_f > DIST_PARAR_GIRO:
                self._cambiar_estado('avanzar', 'escape completado')

        # --- APLICACIÓN DE VELOCIDADES ---
        evento = ''
        if self.estado == 'pasillo':
            twist.linear.x  = VEL_LINEAR_PASILLO
            twist.angular.z = 0.0
            evento = f'pasillo_recto f={d_f:.2f}'
            
        elif self.estado == 'retroceder':
            if self.d_back > DIST_SEGURIDAD_TRASERA:
                twist.linear.x = -VEL_RETROCESO
            else:
                twist.linear.x = 0.0
            twist.angular.z = 0.0  
            evento = f'retrocediendo_seguro_anti_roce B={self.d_back:.2f}'
            
        elif self.estado == 'escape':
            twist.linear.x  = -0.05
            twist.angular.z = VEL_GIRO
            evento = 'escape'
            
        elif self.estado == 'girar_izq':
            twist.linear.x  = VEL_AVANCE_GIRO if d_f > 0.22 else 0.0
            twist.angular.z = VEL_GIRO
            evento = f'girar_izq t={tiempo_girando:.1f}s'
            
        elif self.estado == 'girar_der':
            twist.linear.x  = VEL_AVANCE_GIRO if d_f > 0.22 else 0.0
            twist.angular.z = -VEL_GIRO
            evento = f'girar_der t={tiempo_girando:.1f}s'
            
        else:  # avanzar
            vel = self.velocidad_frenada(d_f, VEL_LINEAR_NORMAL)
            twist.linear.x = vel
            if d_r > 1.2:
                # Si se abre la derecha, verificamos preventivamente que no sea la boca de entrada a un callejón negro
                if self._ruta_conduce_a_callejon(-math.pi/2):
                    twist.angular.z = 0.0  # Sigue recto e ignora la trampa
                    evento = 'ANTIBUCLE: Ignorando pasillo negro a la derecha'
                else:
                    twist.angular.z = -0.20
                    evento = f'buscando_pared vel={vel:.3f}'
            else:
                error = DIST_PARED_DERECHA - d_r
                twist.angular.z = max(min(KP * error, 0.40), -0.40)
                evento = f'siguiendo_pared_der err={error:.3f}'
            if vel < VEL_LINEAR_NORMAL:
                evento += ' FRENANDO'

        self.vel_lin_pub = twist.linear.x
        self.vel_ang_pub = twist.angular.z
        self._log_tick(evento)
        self.cmd_pub.publish(twist)

    def __del__(self):
        try:
            self._log_raw('=== FIN SESION ===')
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
        nodo.log_file.close()
        nodo.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()