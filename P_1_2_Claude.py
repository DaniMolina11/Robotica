#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time

# --- PARÁMETROS CONFIGURADOS PARA PASILLOS DE 30-35 CM ---
DIST_PASILLO           = 0.25   # Ambos lados por debajo de esto = Pasillo
DIST_PARED_DERECHA     = 0.16   # Centro ideal en pasillo de ~32cm
DIST_GIRO_PASILLO      = 0.22   # Distancia al frente para actuar
DIST_PARAR_GIRO        = 0.22   

# Umbrales estrictos para callejón sin salida
CALLEJON_FRONT         = 0.24
CALLEJON_LATERAL       = 0.22

# VELOCIDADES DE PRECISIÓN (Calma y control)
VEL_LINEAR_PASILLO    = 0.03   
VEL_LINEAR_NORMAL     = 0.04   
VEL_AVANCE_GIRO       = 0.01   
VEL_GIRO              = 0.18   

# PID AGRESIVO Y SIN RETARDO PARA PASILLO
KP_PASILLO            = 3.8    # Respuesta inmediata al descentrado
KI_PASILLO            = 0.02   
KD_PASILLO            = 1.6    # Amortiguación real para eliminar la oscilación

KP_NORMAL             = 1.5    
TIEMPO_GIRO_MINIMO    = 1.5    
TICKS_CONFIRMACION    = 3

LOG_FILE = '/home/ros/Escriptori/Robotica/maze_log.txt'

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver_node')

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        # Suscripción al Scan: Aquí nacerá el pulso de control del robot
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry,  '/odom', self.odom_callback, 10)

        # Variables de lectura instantánea (Sin deques/retardos)
        self.d_front    = 3.0
        self.d_right    = 3.0
        self.d_left     = 3.0
        self.d_back     = 3.0

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.yaw   = 0.0
        self.yaw_inicial = 0.0

        # Control de tiempo real para el PID
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
        self._log_raw('=== INICIO SESION CONTROL SINCRONO SENSOR-MOTOR ===')

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
        
        # 1. MARCAR EL TIEMPO DE SIMULACIÓN Y CALCULAR DT REAL
        self.sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.last_sim_time == 0.0:
            self.last_sim_time = self.sim_time
            return
        
        dt = self.sim_time - self.last_sim_time
        self.last_sim_time = self.sim_time
        
        if dt <= 0.0: 
            dt = 0.05 # Salvaguarda por si el reloj se pisa

        # 2. CAPTURAR LAS DISTANCIAS REALES INSTANTÁNEAS (CERO RETARDO)
        self.d_front = min(self.sector_min(r, 350, 360), self.sector_min(r, 0, 10))
        self.d_back  = self.sector_min(r, 170, 190)
        self.d_left  = self.sector_min(r, 60, 120)    # Rango amplio anti-giros
        self.d_right = self.sector_min(r, 240, 300)  # Rango amplio anti-giros

        # 3. LANZAR EL CONTROLADOR INMEDIATAMENTE CON EL DT REAL
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
        if self.d_right > 0.35: 
            lado = 'der' 
        elif self.d_left > 0.35: 
            lado = 'izq' 
        else:
            lado = 'der' if self.d_right >= self.d_left else 'izq'
        return lado

    def _iniciar_giro(self, ahora):
        lado = self._decidir_lado_giro()
        self._cambiar_estado(f'girar_{lado}', f'Cruce detectado. Lado={lado}')
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

        # --- DETECCIÓN DE CALLEJÓN ---
        if self.estado in ('avanzar', 'pasillo'):
            if d_f < CALLEJON_FRONT and d_l < CALLEJON_LATERAL and d_r < CALLEJON_LATERAL:
                self._cambiar_estado('girar_180', 'CALLEJÓN CONFIRMADO')
                self.yaw_inicial = self.yaw
                self.tiempo_inicio_giro = ahora
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
                    if d_r > DIST_PASILLO:
                        self._cambiar_estado('girar_der', 'El pasillo se abre a la derecha')
                        self.tiempo_inicio_giro = ahora
                        self.giro_comprometido = True
                    elif d_l > DIST_PASILLO:
                        self._cambiar_estado('girar_izq', 'El pasillo se abre a la izquierda')
                        self.tiempo_inicio_giro = ahora
                        self.giro_comprometido = True
                    else:
                        self._cambiar_estado('avanzar', 'Salida neutral')
                    self.ticks_fuera_pasillo = 0

        elif self.estado == 'avanzar':
            if en_pasillo and d_f >= DIST_GIRO_PASILLO:
                self._cambiar_estado('pasillo', 'Entrando en pasillo')
                self.error_anterior_pasillo = 0.0
                self.integral_pasillo = 0.0
                self.ticks_fuera_pasillo = 0
            elif d_f < DIST_PARAR_GIRO:
                self._iniciar_giro(ahora)
            elif d_r > 0.38 and d_f > 0.30: 
                self._cambiar_estado('girar_der', 'Muro derecho termina')
                self.tiempo_inicio_giro = ahora
                self.giro_comprometido = True

        elif self.estado in ('girar_izq', 'girar_der'):
            if self.giro_comprometido:
                if tiempo_girando >= TIEMPO_GIRO_MINIMO:
                    self.giro_comprometido = False
            else:
                if d_f >= DIST_PARAR_GIRO + 0.08:
                    self._cambiar_estado('avanzar', 'Frente despejado')
                elif d_f < DIST_PARAR_GIRO - 0.04:
                    self._iniciar_giro(ahora)

        elif self.estado == 'girar_180':
            diff_angular = self.yaw - self.yaw_inicial
            diff_angular = math.atan2(math.sin(diff_angular), math.cos(diff_angular))
            
            if tiempo_girando > 2.5 and abs(diff_angular) > 2.95 and d_f > 0.35:
                self._cambiar_estado('avanzar', 'Giro 180 completado')
            elif tiempo_girando > 8.0 and d_f > 0.35:
                self._cambiar_estado('avanzar', 'Giro 180 abortado')

        # --- APLICACIÓN DE VELOCIDADES Y CONTROL PID REAL ---
        evento = ''
        if self.estado == 'pasillo':
            error = d_l - d_r  
            
            # Integral calculada con el dt real de la simulación
            self.integral_pasillo += error * dt
            self.integral_pasillo = max(min(self.integral_pasillo, 0.10), -0.10)
            
            # Derivada exacta basada en el tiempo real entre muestras del sensor
            derivada = (error - self.error_anterior_pasillo) / dt
            self.error_anterior_pasillo = error
            
            salida_pid = (KP_PASILLO * error) + (KI_PASILLO * self.integral_pasillo) + (KD_PASILLO * derivada)
            
            twist.linear.x  = VEL_LINEAR_PASILLO
            twist.angular.z = max(min(salida_pid, 0.40), -0.40) 
            evento = f'PID_PURO dt={dt:.3f} err={error:+.3f} D={derivada:+.3f}'
            
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
            
        else:  
            if d_r > 0.45:
                twist.linear.x  = VEL_LINEAR_NORMAL
                twist.angular.z = -0.12 
                evento = 'Buscando muro'
            else:
                error_pared = DIST_PARED_DERECHA - d_r
                twist.linear.x  = VEL_LINEAR_NORMAL
                twist.angular.z = max(min(KP_NORMAL * error_pared, 0.22), -0.22)
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