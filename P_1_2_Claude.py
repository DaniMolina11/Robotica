#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time

# --- PARÁMETROS CONFIGURADOS PARA PASILLOS DE 30-35 CM ---
DIST_PASILLO           = 0.25   # Un lateral < 0.25m significa que hay muro pegado
DIST_PARED_DERECHA     = 0.16   # Centro ideal en pasillo de ~32cm
DIST_PARAR_GIRO        = 0.22   # Umbral frontal de colisión para tomar una acción
DIST_GIRO_PASILLO      = 0.22   
UMBRAL_ABIERTO         = 0.32   # Más de esto significa que el pasillo se ha abierto

# VELOCIDADES DE PRECISIÓN (Calma y control)
VEL_LINEAR_PASILLO    = 0.03   
VEL_LINEAR_NORMAL     = 0.04   
VEL_AVANCE_GIRO       = 0.01   
VEL_GIRO              = 0.18   

# PID DE ALTA PRIORIDAD PARA PASILLO
KP_PASILLO            = 3.8    
KI_PASILLO            = 0.02   
KD_PASILLO            = 1.6    

KP_NORMAL             = 1.5    
TIEMPO_GIRO_MINIMO    = 1.5    
TICKS_CONFIRMACION    = 3

LOG_FILE = '/home/ros/Escriptori/Robotica/maze_log.txt'

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver_node')

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry,  '/odom', self.odom_callback, 10)

        # Variables de lectura instantánea por rango amplio
        self.d_front    = 3.0
        self.d_right    = 3.0
        self.d_left     = 3.0
        self.d_back     = 3.0

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw   = 0.0
        self.yaw_inicial = 0.0

        # Historial del PID
        self.sim_time             = 0.0
        self.last_sim_time        = 0.0
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

        self.log_file = open(LOG_FILE, 'w')
        self._log_raw('=== INICIO SESION JERARQUÍA CORREGIDA ===')

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
        if self.last_sim_time == 0.0:
            self.last_sim_time = self.sim_time
            return
        
        dt = self.sim_time - self.last_sim_time
        self.last_sim_time = self.sim_time
        
        if dt <= 0.0: 
            dt = 0.05

        # Obtención de la distancia perpendicular real mediante escaneo por abanicos (Evita el error por desalineación)
        self.d_front = min(self.sector_min(r, 350, 360), self.sector_min(r, 0, 10))
        self.d_back  = self.sector_min(r, 170, 190)
        self.d_left  = self.sector_min(r, 60, 120)    
        self.d_right = self.sector_min(r, 240, 300)  

        self.control_loop(dt)

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
        # Si la derecha está libre por encima del umbral, giramos a la derecha (Prioridad reglamentaria)
        if self.d_right > UMBRAL_ABIERTO: 
            lado = 'der' 
        # Si la derecha está bloqueada pero la izquierda está libre, giramos a la izquierda obligatoriamente
        elif self.d_left > UMBRAL_ABIERTO: 
            lado = 'izq' 
        else:
            lado = 'der' if self.d_right >= self.d_left else 'izq'
        return lado

    def _iniciar_giro(self, ahora):
        lado = self._decidir_lado_giro()
        self._cambiar_estado(f'girar_{lado}', f'Curva normal detectada. Lado={lado}')
        self.tiempo_inicio_giro = ahora
        self.giro_comprometido  = True

    def control_loop(self, dt):
        twist = Twist()

        if self.meta_alcanzada:
            self.cmd_pub.publish(twist)
            return

        d_f = self.d_front
        d_r = self.d_right
        d_l = self.d_left

        ahora          = time.time()
        tiempo_girando = ahora - self.tiempo_inicio_giro
        en_pasillo     = (d_r < DIST_PASILLO and d_l < DIST_PASILLO)

        # --- MÁQUINA DE ESTADOS Y JERARQUÍA DE DISTANCIAS ---
        
        # 1. Si el robot está ejecutando activamente un giro, la máquina sigue su curso sin interrumpirse
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
            
            if tiempo_girando > 2.5 and abs(diff_angular) > 2.95 and d_f > 0.35:
                self._cambiar_estado('avanzar', 'Giro 180 completado')
            elif tiempo_girando > 8.0 and d_f > 0.35:
                self._cambiar_estado('avanzar', 'Giro 180 abortado')

        # 2. Si NO está girando (está navegando en línea recta), evaluamos la jerarquía de sensores
        else:
            # PRIMER FILTRO JERÁRQUICO: Muro al frente detectado
            if d_f < DIST_PARAR_GIRO:
                # Desambiguación inmediata basada en el entorno real de pasillo:
                if d_r < DIST_PASILLO and d_l < DIST_PASILLO:
                    # Ambos lados cerrados de forma claustrofóbica -> ¡Es un callejón sin salida real!
                    self._cambiar_estado('girar_180', 'CALLEJÓN CONFIRMADO: Frente y laterales cerrados')
                    self.yaw_inicial = self.yaw
                    self.tiempo_inicio_giro = ahora
                else:
                    # Uno de los lados está libre -> Es una esquina o intersección normal
                    self._iniciar_giro(ahora)
            
            # SEGUNDO FILTRO JERÁRQUICO: Navegación con frente libre
            elif self.estado == 'pasillo':
                if en_pasillo:
                    self.ticks_fuera_pasillo = 0
                else:
                    self.ticks_fuera_pasillo += 1
                    if self.ticks_fuera_pasillo >= TICKS_CONFIRMACION:
                        if d_r > UMBRAL_ABIERTO:
                            self._cambiar_estado('girar_der', 'El pasillo termina abriéndose a la derecha')
                            self.tiempo_inicio_giro = ahora
                            self.giro_comprometido = True
                        elif d_l > UMBRAL_ABIERTO:
                            self._cambiar_estado('girar_izq', 'El pasillo termina abriéndose a la izquierda')
                            self.tiempo_inicio_giro = ahora
                            self.giro_comprometido = True
                        else:
                            self._cambiar_estado('avanzar', 'Salida neutral de pasillo')
                        self.ticks_fuera_pasillo = 0
                        
            elif self.estado == 'avanzar':
                if en_pasillo:
                    self._cambiar_estado('pasillo', 'Configuración de pasillo detectada')
                    self.error_anterior_pasillo = 0.0
                    self.integral_pasillo = 0.0
                    self.ticks_fuera_pasillo = 0
                elif d_r > UMBRAL_ABIERTO and d_f > 0.30: 
                    self._cambiar_estado('girar_der', 'Muro derecho finalizado en zona abierta')
                    self.tiempo_inicio_giro = ahora
                    self.giro_comprometido = True

        # --- ENVÍO DE VELOCIDADES ---
        evento = ''
        if self.estado == 'pasillo':
            error = d_l - d_r  
            self.integral_pasillo += error * dt
            self.integral_pasillo = max(min(self.integral_pasillo, 0.10), -0.10)
            derivada = (error - self.error_anterior_pasillo) / dt
            self.error_anterior_pasillo = error
            
            salida_pid = (KP_PASILLO * error) + (KI_PASILLO * self.integral_pasillo) + (KD_PASILLO * derivada)
            
            twist.linear.x  = VEL_LINEAR_PASILLO
            twist.angular.z = max(min(salida_pid, 0.40), -0.40) 
            evento = f'PID_PASILLO err={error:+.3f}'
            
        elif self.estado == 'girar_180':
            twist.linear.x  = 0.0
            twist.angular.z = -VEL_GIRO 
            evento = 'Rotando 180 grados'
            
        elif self.estado == 'girar_izq':
            twist.linear.x  = VEL_AVANCE_GIRO if d_f > 0.18 else 0.0
            twist.angular.z = VEL_GIRO
            evento = 'Curva Izquierda'
            
        elif self.estado == 'girar_der':
            twist.linear.x  = VEL_AVANCE_GIRO if d_f > 0.18 else 0.0
            twist.angular.z = -VEL_GIRO
            evento = 'Curva Derecha'
            
        else:  # avanzar (seguimiento de pared derecha fuera de pasillo)
            if d_r > 0.45:
                twist.linear.x  = VEL_LINEAR_NORMAL
                twist.angular.z = -0.12 
                evento = 'Buscando contacto con muro'
            else:
                error_pared = DIST_PARED_DERECHA - d_r
                twist.linear.x  = VEL_LINEAR_NORMAL
                twist.angular.z = max(min(KP_NORMAL * error_pared, 0.22), -0.22)
                evento = f'Siguiendo pared der err={error_pared:+.3f}'

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