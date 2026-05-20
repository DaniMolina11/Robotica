#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time

# --- PARÁMETROS DE MÁXIMA AGILIDAD (PASILLOS 30-35 CM) ---
DIST_PASILLO           = 0.25   # Un lateral < 0.25m significa que hay muro pegado
DIST_PARAR_GIRO        = 0.24   # Umbral frontal para actuar (un pelo más alto por la velocidad)
UMBRAL_ABIERTO         = 0.30   # Si el canal de apertura supera esto, el pasillo se ha abierto

# VELOCIDADES AGRESIVAS (Modo Carrera Activado)
VEL_LINEAR_PASILLO    = 0.08   # Velocidad alegre y fluida en pasillos
VEL_LINEAR_NORMAL     = 0.10   
VEL_AVANCE_GIRO       = 0.04   
VEL_GIRO              = 0.65   # ¡Giro ultra-rápido!

# PID RECALIBRADO PARA 50 HZ
KP_PASILLO            = 6.5    
KI_PASILLO            = 0.01   
KD_PASILLO            = 2.8    

KP_NORMAL             = 1.8    
TIEMPO_GIRO_MINIMO    = 0.8    # Menos tiempo comprometido para encadenar giros rápido
TICKS_CONFIRMACION    = 1      # Respuesta inmediata al detectar cambios laterales

LOG_FILE = '/home/ros/Escriptori/Robotica/maze_log.txt'

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver_node')

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry,  '/odom', self.odom_callback, 10)
        
        # TIMER ULTRA-RÁPIDO: Actualización de trayectoria a 50 Hz (Cada 0.02 segundos)
        self.timer    = self.create_timer(0.02, self.timer_control_loop)

        # Variables de Percepción Separadas
        self.d_front       = 3.0
        self.d_left_wall   = 0.16  # Para el PID (Mínimo amplio)
        self.d_right_wall  = 0.16  # Para el PID (Mínimo amplio)
        self.d_left_open   = 0.16  # Para detectar huecos (Haz cerrado a 90°)
        self.d_right_open  = 0.16  # Para detectar huecos (Haz cerrado a 270°)
        self.ancho_medio   = 0.16
        self.lecturas_listas = False

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw   = 0.0
        self.yaw_inicial = 0.0

        # Memoria síncrona para el PID del Lidar
        self.last_scan_time       = 0.0
        self.error_anterior_pasillo = 0.0
        self.integral_pasillo       = 0.0
        self.salida_pid           = 0.0

        self.estado              = 'avanzar'
        self.tiempo_inicio_giro  = 0.0
        self.giro_comprometido   = False
        self.ticks_fuera_pasillo = 0

        self.META_X               = 2.75
        self.META_Y               = 1.71
        self.DISTANCIA_MINIMA_META = 0.25
        self.meta_alcanzada       = False
        self.sim_time             = 0.0

        self.log_file = open(LOG_FILE, 'w')
        self._log_raw('=== INICIO SESION: AGILIDAD 50HZ CON FILTROS SEPARADOS ===')

    def _log_raw(self, msg):
        ts = time.strftime('%H:%M:%S')
        self.log_file.write(f'[{ts}] {msg}\n')
        self.log_file.flush()

    def _log_tick(self, evento=''):
        self._log_raw(
            f'{self.sim_time:9.2f} | X:{self.pos_x:+5.2f} Y:{self.pos_y:+5.2f} | '
            f'{self.estado:<13}| F:{self.d_front:.2f} R:{self.d_right_open:.2f} L:{self.d_left_open:.2f} | '
            f'{evento}'
        )

    def clean(self, v):
        return 3.0 if (math.isinf(v) or math.isnan(v)) else float(v)

    def sector_min(self, ranges, a, b):
        return min(self.clean(ranges[i]) for i in range(a, b))

    def scan_callback(self, msg):
        r = msg.ranges
        if len(r) < 360:
            return
        
        self.sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        
        # Calcular el dt real del sensor para mantener el PID matemático perfecto
        if self.last_scan_time == 0.0:
            self.last_scan_time = self.sim_time
            return
        dt_scan = self.sim_time - self.last_scan_time
        self.last_scan_time = self.sim_time
        if dt_scan <= 0.0: dt_scan = 0.05

        # 1. CANAL FRONTAL COLIMADO
        self.d_front = min(self.sector_min(r, 358, 360), self.sector_min(r, 0, 2))

        # 2. CANALES PARA EL PID (Abanico intermedio para no perder la pared si el robot oscila)
        self.d_left_wall  = self.sector_min(r, 60, 120)   
        self.d_right_wall = self.sector_min(r, 240, 300)  

        # 3. CANALES DE APERTURA (Haces estrechos a 90° y 270° para detectar pasillos abiertos)
        self.d_left_open  = self.sector_min(r, 86, 94)
        self.d_right_open = self.sector_min(r, 266, 274)

        # 4. CALCULO DEL PID EN SINTONÍA CON EL SENSOR
        self.ancho_medio = (self.d_left_wall + self.d_right_wall) / 2.0
        error = self.ancho_medio - self.d_right_wall
        
        self.integral_pasillo += error * dt_scan
        self.integral_pasillo = max(min(self.integral_pasillo, 0.15), -0.15)
        derivada = (error - self.error_anterior_pasillo) / dt_scan
        self.error_anterior_pasillo = error
        
        # Guardamos la salida para que el Timer de 50Hz la use al instante
        self.salida_pid = (KP_PASILLO * error) + (KI_PASILLO * self.integral_pasillo) + (KD_PASILLO * derivada)
        self.lecturas_listas = True

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

    def _cambiar_estado(self, nuevo, motivo=''):
        if nuevo != self.estado:
            self._log_raw(f'*** CAMBIO ESTADO: {self.estado} -> {nuevo} [{motivo}]')
            self.estado = nuevo

    def _decidir_lado_giro(self):
        # SOLUCIÓN CRÍTICA: Evaluamos los canales de apertura colimados, no los muros lejanos
        if self.d_right_open > UMBRAL_ABIERTO: 
            lado = 'der' # Derecha libre de verdad -> Gira a la derecha
        elif self.d_left_open > UMBRAL_ABIERTO: 
            lado = 'izq' # Izquierda libre de verdad y derecha cerrada -> ¡Gira a la izquierda obligatorio!
        else:
            # Si ambos lados están físicamente colapsados frente al robot, desempata el PID lateral
            lado = 'der' if self.d_right_wall >= self.d_left_wall else 'izq'
        return lado

    def _iniciar_giro(self, ahora):
        lado = self._decidir_lado_giro()
        self._cambiar_estado(f'girar_{lado}', f'Curva detectada por canales. Lado={lado}')
        self.tiempo_inicio_giro = ahora
        self.giro_comprometido  = True

    def timer_control_loop(self):
        """Bucle de control de trayectoria ejecutado de forma síncrona a 50 Hz"""
        if not self.lecturas_listas or self.meta_alcanzada:
            return

        twist = Twist()
        d_f = self.d_front
        d_r_open = self.d_right_open
        d_l_open = self.d_left_open

        ahora          = time.time()
        tiempo_girando = ahora - self.tiempo_inicio_giro
        
        # Evaluamos si estamos en pasillo usando los canales colimados directos
        en_pasillo = (d_r_open < DIST_PASILLO and d_l_open < DIST_PASILLO)

        # --- MAQUINA DE ESTADOS REACIVA A 50 HZ ---
        if self.estado in ('girar_izq', 'girar_der'):
            if self.giro_comprometido:
                if tiempo_girando >= TIEMPO_GIRO_MINIMO:
                    self.giro_comprometido = False
            else:
                if d_f >= DIST_PARAR_GIRO + 0.06:
                    self._cambiar_estado('avanzar', 'Frente recuperado')
                elif d_f < DIST_PARAR_GIRO - 0.04:
                    self._iniciar_giro(ahora)

        elif self.estado == 'girar_180':
            diff_angular = self.yaw - self.yaw_inicial
            diff_angular = math.atan2(math.sin(diff_angular), math.cos(diff_angular))
            
            # La alta frecuencia del timer permite clavar la salida del callejón al vuelo
            if tiempo_girando > 1.0 and abs(diff_angular) > 2.95 and d_f > 0.30:
                self._cambiar_estado('avanzar', 'Escape de callejón completado')
            elif tiempo_girando > 4.0 and d_f > 0.30:
                self._cambiar_estado('avanzar', 'Escape completado por tiempo')

        else:
            # FILTRO 1: Colisión frontal detectada
            if d_f < DIST_PARAR_GIRO:
                if d_r_open < DIST_PASILLO and d_l_open < DIST_PASILLO:
                    self._cambiar_estado('girar_180', 'CALLEJÓN CAPTURADO: Bloqueo en canales laterales')
                    self.yaw_inicial = self.yaw
                    self.tiempo_inicio_giro = ahora
                else:
                    self._iniciar_giro(ahora)
            
            # FILTRO 2: Navegación libre en recta
            elif self.estado == 'pasillo':
                if en_pasillo:
                    self.ticks_fuera_pasillo = 0
                else:
                    self.ticks_fuera_pasillo += 1
                    if self.ticks_fuera_pasillo >= TICKS_CONFIRMACION:
                        if d_r_open > UMBRAL_ABIERTO:
                            self._cambiar_estado('girar_der', 'Pasillo abierto a la derecha')
                            self.tiempo_inicio_giro = ahora
                            self.giro_comprometido = True
                        elif d_l_open > UMBRAL_ABIERTO:
                            self._cambiar_estado('girar_izq', 'Pasillo abierto a la izquierda')
                            self.tiempo_inicio_giro = ahora
                            self.giro_comprometido = True
                        else:
                            self._cambiar_estado('avanzar', 'Fin de pasillo neutro')
                        self.ticks_fuera_pasillo = 0
                        
            elif self.estado == 'avanzar':
                if en_pasillo:
                    self._cambiar_estado('pasillo', 'Pasillo estrecho detectado')
                    self.ticks_fuera_pasillo = 0
                elif d_r_open > UMBRAL_ABIERTO and d_f > 0.28: 
                    self._cambiar_estado('girar_der', 'Muro derecho cortado en abierto')
                    self.tiempo_inicio_giro = ahora
                    self.giro_comprometido = True

        # --- SELECCIÓN AGIL DE VELOCIDADES ---
        evento = ''
        if self.estado == 'pasillo':
            twist.linear.x  = VEL_LINEAR_PASILLO
            twist.angular.z = max(min(self.salida_pid, 0.60), -0.60) # Mayor autoridad angular para corregir rápido
            evento = f'PID_CENTRO_50HZ centro={self.ancho_medio:.3f}'
            
        elif self.estado == 'girar_180':
            twist.linear.x  = 0.0
            twist.angular.z = -VEL_GIRO 
            evento = 'Rotando in-situ 180°'
            
        elif self.estado == 'girar_izq':
            twist.linear.x  = VEL_AVANCE_GIRO if d_f > 0.15 else 0.0
            twist.angular.z = VEL_GIRO
            evento = 'Curva rápida Izquierda'
            
        elif self.estado == 'girar_der':
            twist.linear.x  = VEL_AVANCE_GIRO if d_f > 0.15 else 0.0
            twist.angular.z = -VEL_GIRO
            evento = 'Curva rápida Derecha'
            
        else:  # avanzar
            if d_r_open > 0.45:
                twist.linear.x  = VEL_LINEAR_NORMAL
                twist.angular.z = -0.30 
                evento = 'Cazando muro'
            else:
                error_pared = (self.ancho_medio if self.ancho_medio < 0.22 else 0.16) - self.d_right_wall
                twist.linear.x  = VEL_LINEAR_NORMAL
                twist.angular.z = max(min(KP_NORMAL * error_pared, 0.45), -0.45)
                evento = f'Seguimiento lateral err={error_pared:+.3f}'

        self.cmd_pub.publish(twist)
        self._log_tick(evento)

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