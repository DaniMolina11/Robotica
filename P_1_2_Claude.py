#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time

# --- PARÁMETROS DE AGILIDAD PARA PASILLOS DE 30-35 CM ---
DIST_PASILLO           = 0.25   # Un lateral < 0.25m significa que hay muro pegado
DIST_PARAR_GIRO        = 0.22   # Umbral frontal de colisión real
UMBRAL_ABIERTO         = 0.35   # Más de esto significa que el pasillo se ha abierto

# VELOCIDADES DE AGILIDAD (Movimiento dinámico y despierto)
VEL_LINEAR_PASILLO    = 0.06   # Duplicada para avanzar con soltura
VEL_LINEAR_NORMAL     = 0.08   # Velocidad alegre en abierto
VEL_AVANCE_GIRO       = 0.03   # Avance ágil en curvas
VEL_GIRO              = 0.50   # ¡Giro rápido! (180° en poco más de 3 segundos)

# PID ULTRA-REACTIVO (Calculado para 20 Hz)
KP_PASILLO            = 5.0    # Fuerza proporcional para corregir al instante
KI_PASILLO            = 0.02   
KD_PASILLO            = 2.2    # Amortiguación matemática para absorber inercias rápidas

KP_NORMAL             = 1.8    
TIEMPO_GIRO_MINIMO    = 1.0    # Reducido para no perder tiempo en las esquinas
TICKS_CONFIRMACION    = 2      # Menos ticks = decisión más rápida al salir de pasillos

LOG_FILE = '/home/ros/Escriptori/Robotica/maze_log.txt'

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver_node')

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry,  '/odom', self.odom_callback, 10)
        
        # TEMPORIZADOR ALTA FRECUENCIA: Actualiza la trayectoria cada 0.05 segundos (20 Hz)
        self.timer    = self.create_timer(0.05, self.timer_control_loop)

        # Variables globales de sensores (Lectura instantánea)
        self.d_front    = 3.0
        self.d_right    = 3.0
        self.d_left     = 3.0
        self.d_back     = 3.0
        self.ancho_medio = 0.16
        self.lecturas_listas = False

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw   = 0.0
        self.yaw_inicial = 0.0

        # Control del PID de tiempo continuo
        self.last_time            = None
        self.error_anterior_pasillo = 0.0
        self.integral_pasillo       = 0.0

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
        self._log_raw('=== INICIO SESION: MÁXIMA AGILIDAD 20 HZ ===')

    def _log_raw(self, msg):
        ts = time.strftime('%H:%M:%S')
        self.log_file.write(f'[{ts}] {msg}\n')
        self.log_file.flush()

    def _log_tick(self, evento=''):
        self._log_raw(
            f'{self.sim_time:9.2f} | X:{self.pos_x:+5.2f} Y:{self.pos_y:+5.2f} | '
            f'{self.estado:<13}| F:{self.d_front:.2f} R:{self.d_right:.2f} L:{self.d_left:.2f} | '
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

        # 1. HEMISFERIO FRONTAL ULTRA-CERRADO (Filtro óptico para no rozar paredes en recta)
        self.d_front = min(self.sector_min(r, 358, 360), self.sector_min(r, 0, 2))
        self.d_back  = self.sector_min(r, 178, 182)

        # 2. PARTICIÓN EN DOS HEMISFERIOS LATERALES MASIVOS (Mínimos absolutos reales)
        self.d_left  = self.sector_min(r, 15, 165)   
        self.d_right = self.sector_min(r, 195, 345)  

        # 3. PROMEDIO DINÁMICO DEL ANCHO
        self.ancho_medio = (self.d_left + self.d_right) / 2.0
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
        if self.d_right > UMBRAL_ABIERTO: 
            lado = 'der' 
        elif self.d_left > UMBRAL_ABIERTO: 
            lado = 'izq' 
        else:
            lado = 'der' if self.d_right >= self.d_left else 'izq'
        return lado

    def _iniciar_giro(self, ahora):
        lado = self._decidir_lado_giro()
        self._cambiar_estado(f'girar_{lado}', f'Curva rápida. Lado={lado}')
        self.tiempo_inicio_giro = ahora
        self.giro_comprometido  = True

    def timer_control_loop(self):
        """Bucle de trayectoria asíncrono y ultra-rápido (Ejecutado por el Timer)"""
        if not self.lecturas_listas or self.meta_alcanzada:
            return

        # Calcular DT exacto usando el reloj del sistema de ROS2 (Evita desajustes por lag)
        now_time = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now_time
            return
        dt = (now_time - self.last_time).nanoseconds / 1e9
        self.last_time = now_time
        if dt <= 0.0: dt = 0.05

        twist = Twist()
        d_f = self.d_front
        d_r = self.d_right
        d_l = self.d_left

        ahora          = time.time()
        tiempo_girando = ahora - self.tiempo_inicio_giro
        en_pasillo     = (d_r < DIST_PASILLO and d_l < DIST_PASILLO)

        # --- MÁQUINA DE ESTADOS DINÁMICA (JERARQUÍA MÁXIMA) ---
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
            
            # Condición de salida ágil del callejón
            if tiempo_girando > 1.2 and abs(diff_angular) > 2.95 and d_f > 0.30:
                self._cambiar_estado('avanzar', 'Salida rápida de callejón exitosa')
            elif tiempo_girando > 5.0 and d_f > 0.30:
                self._cambiar_estado('avanzar', 'Giro 180 completado por tiempo')

        else:
            # FILTRO CRÍTICO 1: Bloqueo frontal inminente
            if d_f < DIST_PARAR_GIRO:
                if d_r < DIST_PASILLO and d_l < DIST_PASILLO:
                    self._cambiar_estado('girar_180', 'CALLEJÓN CONFIRMADO POR HEMISFERIOS')
                    self.yaw_inicial = self.yaw
                    self.tiempo_inicio_giro = ahora
                else:
                    self._iniciar_giro(ahora)
            
            # FILTRO CRÍTICO 2: Navegación estándar en recta
            elif self.estado == 'pasillo':
                if en_pasillo:
                    self.ticks_fuera_pasillo = 0
                else:
                    self.ticks_fuera_pasillo += 1
                    if self.ticks_fuera_pasillo >= TICKS_CONFIRMACION:
                        if d_r > UMBRAL_ABIERTO:
                            self._cambiar_estado('girar_der', 'Abertura derecha en carrera')
                            self.tiempo_inicio_giro = ahora
                            self.giro_comprometido = True
                        elif d_l > UMBRAL_ABIERTO:
                            self._cambiar_estado('girar_izq', 'Abertura izquierda en carrera')
                            self.tiempo_inicio_giro = ahora
                            self.giro_comprometido = True
                        else:
                            self._cambiar_estado('avanzar', 'Fin de pasillo')
                        self.ticks_fuera_pasillo = 0
                        
            elif self.estado == 'avanzar':
                if en_pasillo:
                    self._cambiar_estado('pasillo', 'Entrando en pasillo')
                    self.error_anterior_pasillo = 0.0
                    self.integral_pasillo = 0.0
                    self.ticks_fuera_pasillo = 0
                elif d_r > UMBRAL_ABIERTO and d_f > 0.28: 
                    self._cambiar_estado('girar_der', 'Muro derecho roto')
                    self.tiempo_inicio_giro = ahora
                    self.giro_comprometido = True

        # --- GENERACIÓN ÁGIL DE VELOCIDADES (PID DE ALTA PRIORIDAD) ---
        evento = ''
        if self.estado == 'pasillo':
            # Control de centrado milimétrico por hemisferios masivos con el centro ideal
            error = self.ancho_medio - d_r  
            
            self.integral_pasillo += error * dt
            self.integral_pasillo = max(min(self.integral_pasillo, 0.15), -0.15)
            derivada = (error - self.error_anterior_pasillo) / dt
            self.error_anterior_pasillo = error
            
            salida_pid = (KP_PASILLO * error) + (KI_PASILLO * self.integral_pasillo) + (KD_PASILLO * derivada)
            
            twist.linear.x  = VEL_LINEAR_PASILLO
            twist.angular.z = max(min(salida_pid, 0.55), -0.55) # Mayor margen angular para corregir rápido
            evento = f'PID_AGIL err={error:+.3f} D={derivada:+.2f}'
            
        elif self.estado == 'girar_180':
            twist.linear.x  = 0.0
            twist.angular.z = -VEL_GIRO # Rotación pura y rápida in-situ
            evento = 'Limpiando callejón (180°)'
            
        elif self.estado == 'girar_izq':
            twist.linear.x  = VEL_AVANCE_GIRO if d_f > 0.15 else 0.0
            twist.angular.z = VEL_GIRO
            evento = 'Girando a la Izquierda'
            
        elif self.estado == 'girar_der':
            twist.linear.x  = VEL_AVANCE_GIRO if d_f > 0.15 else 0.0
            twist.angular.z = -VEL_GIRO
            evento = 'Girando a la Derecha'
            
        else:  # avanzar
            if d_r > 0.45:
                twist.linear.x  = VEL_LINEAR_NORMAL
                twist.angular.z = -0.25 # Recuperación rápida de pared
                evento = 'Cazando muro'
            else:
                error_pared = (self.ancho_medio if self.ancho_medio < 0.22 else 0.16) - d_r
                twist.linear.x  = VEL_LINEAR_NORMAL
                twist.angular.z = max(min(KP_NORMAL * error_pared, 0.40), -0.40)
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