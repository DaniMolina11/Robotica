#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time

# --- PARÁMETROS DE SEGURIDAD Y LÍNEA RECTA ---
DIST_PARED_IDEAL       = 0.15   # REQUERIMIENTO: Distancia de seguridad estricta a 15cm
DIST_PASILLO           = 0.26   # Umbral para considerar que un lateral es un muro cerrado
DIST_PARAR_GIRO        = 0.21   # Distancia al frente para avanzar lo necesario y librar la esquina
UMBRAL_ABIERTO         = 0.32   # Distancia que confirma que un pasillo se ha abierto

# VELOCIDADES AGRESIVAS Y ÁGILES A 50 HZ
VEL_LINEAR_PASILLO    = 0.08   
VEL_LINEAR_NORMAL     = 0.10   
VEL_GIRO              = 0.70   # Giro rápido sobre su propio eje

# PID DE ALTA FIDELIDAD (50 HZ)
KP_PASILLO            = 7.5    
KI_PASILLO            = 0.01   
KD_PASILLO            = 3.2    

KP_NORMAL             = 2.0    
TIEMPO_GIRO_MINIMO    = 0.7    
TICKS_CONFIRMACION    = 1      

LOG_FILE = '/home/ros/Escriptori/Robotica/maze_log.txt'

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver_node')

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry,  '/odom', self.odom_callback, 10)
        
        # TIMER RECONSTRUIDO A 50 HZ (Trayectoria en tiempo real cada 0.02s)
        self.timer    = self.create_timer(0.02, self.timer_control_loop)

        # Percepción Dinámica
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

        # Memoria del PID
        self.last_scan_time       = 0.0
        self.error_anterior_pasillo = 0.0
        self.integral_pasillo       = 0.0
        self.salida_pid           = 0.0
        self.evento_pid           = 'INICIANDO'

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
        self._log_raw('=== INICIO SESION: IMPEDIMENTO DE GIRO PREMATURO ===')

    def _log_raw(self, msg):
        ts = time.strftime('%H:%M:%S')
        self.log_file.write(f'[{ts}] {msg}\n')
        self.log_file.flush()

    def _log_tick(self, evento=''):
        self._log_raw(
            f'{self.sim_time:9.2f} | X:{self.pos_x:+5.2f} Y:{self.pos_y:+5.2f} | '
            f'{self.estado:<13}| F:{self.d_front:.2f} R:{self.d_right_wall:.2f} L:{self.d_left_wall:.2f} | '
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

        # 1. Colimador frontal ultra-preciso (No lee las paredes laterales en rectas largas)
        self.d_front = min(self.sector_min(r, 359, 360), self.sector_min(r, 0, 1))

        # 2. Haces colimados a 90° y 270° para detectar aperturas de pasillos al instante
        self.d_left_open  = self.sector_min(r, 85, 95)
        self.d_right_open = self.sector_min(r, 265, 275)

        # 3. Hemisferios laterales para medir la distancia real perpendicular a los muros
        self.d_left_wall  = self.sector_min(r, 75, 105)   
        self.d_right_wall = self.sector_min(r, 255, 285)  

        # 4. CONTROL ASIMÉTRICO DE CONTROL DE LÍNEA RECTA ANTIPREMATURO
        left_closed  = (self.d_left_open < DIST_PASILLO)
        right_closed = (self.d_right_open < DIST_PASILLO)

        if left_closed and right_closed:
            # Caso A: Pasillo totalmente cerrado -> Centrado bilateral perfecto
            error = self.d_left_wall - self.d_right_wall
            self.evento_pid = "CENTRADO_SIMETRICO_PURA_RECTA"
        elif right_closed:
            # Caso B: La izquierda se abre -> Ignoramos el hueco izquierdo para NO girar antes de tiempo.
            # Nos enganchamos a la pared derecha obligándolo a ir en línea recta a 15 cm.
            error = (DIST_PARED_IDEAL - self.d_right_wall) * 2.0  # Multiplicador de refuerzo de carril
            self.evento_pid = "FIJANDO_RECTA_PARED_DERECHA_15CM"
        elif left_closed:
            # Caso C: La derecha se abre -> Nos enganchamos a la pared izquierda a 15 cm para ir rectos.
            error = (self.d_left_wall - DIST_PARED_IDEAL) * 2.0
            self.evento_pid = "FIJANDO_RECTA_PARED_IZQUIERDA_15CM"
        else:
            error = 0.0
            self.evento_pid = "CRUCE_ABIERTO_SIN_PAREDES"
        
        # Ejecución del bucle PID continuo
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
        if self.d_right_open > UMBRAL_ABIERTO: 
            lado = 'der' 
        elif self.d_left_open > UMBRAL_ABIERTO: 
            lado = 'izq' 
        else:
            lado = 'der' if self.d_right_wall >= self.d_left_wall else 'izq'
        return lado

    def _iniciar_giro(self, ahora):
        lado = self._decidir_lado_giro()
        self._cambiar_estado(f'girar_{lado}', f'Iniciando pivote hacia: {lado}')
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
        en_pasillo     = (self.d_right_wall < DIST_PASILLO and self.d_left_wall < DIST_PASILLO)

        # --- MÁQUINA DE ESTADOS RECOUPERADA A 50 HZ ---
        if self.estado in ('girar_izq', 'girar_der'):
            if self.giro_comprometido:
                if tiempo_girando >= TIEMPO_GIRO_MINIMO:
                    self.giro_comprometido = False
            else:
                if d_f >= DIST_PARAR_GIRO + 0.08:
                    self._cambiar_estado('avanzar', 'Frente despejado tras curva')
                elif d_f < DIST_PARAR_GIRO - 0.04:
                    self._iniciar_giro(ahora)

        elif self.estado == 'girar_180':
            diff_angular = self.yaw - self.yaw_inicial
            diff_angular = math.atan2(math.sin(diff_angular), math.cos(diff_angular))
            
            if tiempo_girando > 1.0 and abs(diff_angular) > 2.95 and d_f > 0.30:
                self._cambiar_estado('avanzar', 'Callejón evitado con éxito')
            elif tiempo_girando > 4.0 and d_f > 0.30:
                self._cambiar_estado('avanzar', 'Callejón resuelto por tiempo')

        else:
            # FILTRO JERÁRQUICO 1: Llegada al final del pasillo (Muro frontal detectado)
            if d_f < DIST_PARAR_GIRO:
                if d_r_open < DIST_PASILLO and d_l_open < DIST_PASILLO:
                    self._cambiar_estado('girar_180', 'CALLEJÓN CONFIRMADO')
                    self.yaw_inicial = self.yaw
                    self.tiempo_inicio_giro = ahora
                else:
                    self._iniciar_giro(ahora)
            
            # FILTRO JERÁRQUICO 2: Avance y monitorización en recta
            elif self.estado == 'pasillo':
                if en_pasillo:
                    self.ticks_fuera_pasillo = 0
                else:
                    self.ticks_fuera_pasillo += 1
                    if self.ticks_fuera_pasillo >= TICKS_CONFIRMACION:
                        if d_r_open > UMBRAL_ABIERTO:
                            self._cambiar_estado('girar_der', 'Cruce lateral derecho detectado')
                            self.tiempo_inicio_giro = ahora
                            self.giro_comprometido = True
                        elif d_l_open > UMBRAL_ABIERTO:
                            self._cambiar_estado('girar_izq', 'Cruce lateral izquierdo detectado')
                            self.tiempo_inicio_giro = ahora
                            self.giro_comprometido = True
                        else:
                            self._cambiar_estado('avanzar', 'Saliendo de pasillo')
                        self.ticks_fuera_pasillo = 0
                        
            elif self.estado == 'avanzar':
                if en_pasillo:
                    self._cambiar_estado('pasillo', 'Pasillo cerrado detectado')
                    self.ticks_fuera_pasillo = 0
                elif d_r_open > UMBRAL_ABIERTO and d_f > 0.28: 
                    self._cambiar_estado('girar_der', 'Muro derecho finalizado')
                    self.tiempo_inicio_giro = ahora
                    self.giro_comprometido = True

        # --- GENERACIÓN DE VELOCIDADES DE PIVOTE ---
        evento = self.evento_pid
        if self.estado == 'pasillo':
            twist.linear.x  = VEL_LINEAR_PASILLO
            twist.angular.z = max(min(self.salida_pid, 0.60), -0.60) 
            
        elif self.estado == 'girar_180':
            twist.linear.x  = 0.0
            twist.angular.z = -VEL_GIRO 
            evento = 'Pivoteando 180°'
            
        elif self.estado == 'girar_izq':
            twist.linear.x  = 0.0  # Pivote puro in-situ para asegurar que el chasis libre la esquina
            twist.angular.z = VEL_GIRO
            evento = 'Rotando a la Izquierda'
            
        elif self.estado == 'girar_der':
            twist.linear.x  = 0.0  # Pivote puro in-situ para asegurar que el chasis libre la esquina
            twist.angular.z = -VEL_GIRO
            evento = 'Rotando a la Derecha'
            
        else:  # avanzar
            if d_r_open > 0.45:
                twist.linear.x  = VEL_LINEAR_NORMAL
                twist.angular.z = -0.30 
                evento = 'Buscando contacto de pared'
            else:
                error_pared = DIST_PARED_IDEAL - self.d_right_wall
                twist.linear.x  = VEL_LINEAR_NORMAL
                twist.angular.z = max(min(KP_NORMAL * error_pared, 0.45), -0.45)
                evento = f'Seguimiento normal err={error_pared:+.3f}'

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