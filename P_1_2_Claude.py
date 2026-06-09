#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
import math
import time
from collections import deque

# --- PARÁMETROS DE REFUERZO DE LÍNEA RECTA A 15 CM ---
DIST_PARED_DERECHA     = 0.15   # REQUERIMIENTO: Referencia fija y absoluta a 15 cm
DIST_PASILLO           = 0.25   # Un lateral < 0.25m significa que hay muro pegado
DIST_PARAR_GIRO        = 0.24   # Umbral frontal para actuar
UMBRAL_ABIERTO         = 0.30   # Si el canal de apertura supera esto, el pasillo se ha abierto

# VELOCIDADES AGRESIVAS A 50 HZ
VEL_LINEAR_PASILLO    = 0.04   
VEL_LINEAR_NORMAL     = 0.04   
VEL_AVANCE_GIRO       = 0.04   
VEL_GIRO              = 0.65   

# PID DE ALTA CORRECCIÓN PARA CONTROL DE LÍNEA RECTA (50 HZ)
KP_PASILLO            = 7.5    # Copia el error al instante para no dejar que se mueva de los 15 cm
KI_PASILLO            = 0.01   
KD_PASILLO            = 3.2    # Derivativa alta para anticipar el rebote y clavar la recta

KP_NORMAL             = 2.0    
TIEMPO_GIRO_MINIMO    = 0.8    
TICKS_CONFIRMACION    = 1      

LOG_FILE = '/home/ros/Escriptori/Robotica/maze_log.txt'

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver_node')

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry,  '/odom', self.odom_callback, 10)
        
        # TIMER ULTRA-RÁPIDO: Trayectoria a 50 Hz (Cada 0.02 segundos)
        self.timer    = self.create_timer(0.02, self.timer_control_loop)

        # Variables de Percepción
        self.d_front       = 3.0
        self.d_left_wall   = 0.15  
        self.d_right_wall  = 0.15  
        self.d_left_open   = 0.15  
        self.d_right_open  = 0.15  
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

        # self.log_file = open(LOG_FILE, 'w')
        # self._log_raw('=== INICIO SESION: RECTAS A 15CM FIJOS ===')

    def _log_raw(self, msg):
        ts = time.strftime('%H:%M:%S')
        self.log_file.write(f'[{ts}] {msg}\n')
        self.log_file.flush()

    def _log_tick(self, evento=''):
        self._log_raw(
            f'{self.sim_time:9.2f} | X:{self.pos_x:+5.2f} Y:{self.pos_y:+5.2f} | '
            f'{self.estado:<13}| F:{self.d_front:.2f} R:{self.d_right_wall:.2f} | '
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
        
        if self.last_scan_time == 0.0:
            self.last_scan_time = self.sim_time
            return
        dt_scan = self.sim_time - self.last_scan_time
        self.last_scan_time = self.sim_time
        if dt_scan <= 0.0: dt_scan = 0.02

        # 1. Frente colimado
        self.d_front = min(self.sector_min(r, 358, 360), self.sector_min(r, 0, 2))

        # 2. Hemisferios de muros para el PID (Búsqueda del punto más cercano)
        self.d_left_wall  = self.sector_min(r, 60, 120)   
        self.d_right_wall = self.sector_min(r, 240, 300)  

        # 3. Canales directos a 90° para abrir pasillos
        self.d_left_open  = self.sector_min(r, 86, 94)
        self.d_right_open = self.sector_min(r, 266, 274)

        # 4. PID ENFOCADO EN MANTENER LOS 15 CM DE LA PARED DERECHA
        # Error = Objetivo (0.15) - Distancia Real. 
        # Si d_right_wall es 0.12 (muy pegado), error = +0.03 -> salida positiva -> gira a la izquierda (+) para alejarse.
        error = DIST_PARED_DERECHA - self.d_right_wall
        
        self.integral_pasillo += error * dt_scan
        self.integral_pasillo = max(min(self.integral_pasillo, 0.10), -0.10)
        derivada = (error - self.error_anterior_pasillo) / dt_scan
        self.error_anterior_pasillo = error
        
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
        # Certeza absoluta de canales colimados: si un lado está abierto, se tira por ahí sin dudar
        if self.d_right_open > UMBRAL_ABIERTO: 
            lado = 'der' 
        elif self.d_left_open > UMBRAL_ABIERTO: 
            lado = 'izq' 
        else:
            lado = 'der' if self.d_right_wall >= self.d_left_wall else 'izq'
        return lado

    def _iniciar_giro(self, ahora):
        lado = self._decidir_lado_giro()
        self._cambiar_estado(f'girar_{lado}', f'Curva detectada. Lado={lado}')
        self.tiempo_inicio_giro = ahora
        self.giro_comprometido  = True

    def timer_control_loop(self):
        if not self.lecturas_listas or self.meta_alcanzada:
            return

        twist = Twist()
        d_f = self.d_front
        d_r_open = self.d_right_open
        d_l_open = self.d_left_open

        ahora          = time.time()
        tiempo_girando = ahora - self.tiempo_inicio_giro
        en_pasillo     = (d_r_open < DIST_PASILLO and d_l_open < DIST_PASILLO)

        # --- MÁQUINA DE ESTADOS CRÍTICA ---
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
            
            if tiempo_girando > 1.0 and abs(diff_angular) > 2.95 and d_f > 0.30:
                self._cambiar_estado('avanzar', 'Callejón resuelto')
            elif tiempo_girando > 4.0 and d_f > 0.30:
                self._cambiar_estado('avanzar', 'Callejón resuelto por tiempo')

        else:
            # JERARQUÍA 1: Bloqueo de frente
            if d_f < DIST_PARAR_GIRO:
                if d_r_open < DIST_PASILLO and d_l_open < DIST_PASILLO:
                    self._cambiar_estado('girar_180', 'CALLEJÓN CONFIRMADO')
                    self.yaw_inicial = self.yaw
                    self.tiempo_inicio_giro = ahora
                else:
                    self._iniciar_giro(ahora)
            
            # JERARQUÍA 2: Avance libre
            elif self.estado == 'pasillo':
                if en_pasillo:
                    self.ticks_fuera_pasillo = 0
                else:
                    self.ticks_fuera_pasillo += 1
                    if self.ticks_fuera_pasillo >= TICKS_CONFIRMACION:
                        if d_r_open > UMBRAL_ABIERTO:
                            self._cambiar_estado('girar_der', 'Abertura pasillo derecha')
                            self.tiempo_inicio_giro = ahora
                            self.giro_comprometido = True
                        elif d_l_open > UMBRAL_ABIERTO:
                            self._cambiar_estado('girar_izq', 'Abertura pasillo izquierda')
                            self.tiempo_inicio_giro = ahora
                            self.giro_comprometido = True
                        else:
                            self._cambiar_estado('avanzar', 'Fin pasillo')
                        self.ticks_fuera_pasillo = 0
                        
            elif self.estado == 'avanzar':
                if en_pasillo:
                    self._cambiar_estado('pasillo', 'Pasillo estrecho detectado')
                    self.ticks_fuera_pasillo = 0
                elif d_r_open > UMBRAL_ABIERTO and d_f > 0.28: 
                    self._cambiar_estado('girar_der', 'Muro derecho terminado')
                    self.tiempo_inicio_giro = ahora
                    self.giro_comprometido = True

        # --- GENERACIÓN DE VELOCIDADES ---
        evento = ''
        if self.estado == 'pasillo':
            twist.linear.x  = VEL_LINEAR_PASILLO
            # La salida del PID persigue mantener los 15cm clavados de la derecha
            twist.angular.z = max(min(self.salida_pid, 0.60), -0.60) 
            evento = f'PID_RECTA_15CM err={DIST_PARED_DERECHA - self.d_right_wall:+.3f}'
            
        elif self.estado == 'girar_180':
            twist.linear.x  = 0.0
            twist.angular.z = -VEL_GIRO 
            evento = 'Rotando 180°'
            
        elif self.estado == 'girar_izq':
            twist.linear.x  = VEL_AVANCE_GIRO if d_f > 0.15 else 0.0
            twist.angular.z = VEL_GIRO
            evento = 'Giro Izquierda'
            
        elif self.estado == 'girar_der':
            twist.linear.x  = VEL_AVANCE_GIRO if d_f > 0.15 else 0.0
            twist.angular.z = -VEL_GIRO
            evento = 'Giro Derecha'
            
        else:  # avanzar
            if d_r_open > 0.45:
                twist.linear.x  = VEL_LINEAR_NORMAL
                twist.angular.z = -0.30 
                evento = 'Cazando muro'
            else:
                error_pared = DIST_PARED_DERECHA - self.d_right_wall
                twist.linear.x  = VEL_LINEAR_NORMAL
                twist.angular.z = max(min(KP_NORMAL * error_pared, 0.45), -0.45)
                evento = f'Pared der err={error_pared:+.3f}'

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
