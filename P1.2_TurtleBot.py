#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
# --- PERFIL QoS Estándar para Redes de Robots Reales ---
from rclpy.qos import qos_profile_sensor_data
import math
import time
from collections import deque

# --- PARÁMETROS OPTIMIZADOS PARA EL ENTORNO REAL ---
DIST_GIRO_PASILLO      = 0.24   # Tu distancia perfecta de giro
DIST_PARAR_GIRO        = 0.24   # Tu distancia perfecta de giro
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
KD_ALINEAR            = 1.0     

TIEMPO_GIRO_MINIMO    = 1.5
N_LECTURAS_PROMEDIO   = 5
TICKS_CONFIRMACION    = 4

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver_node')

        # --- EQUILIBRIO QoS: Motores en Reliable (10), Láser y Odom en Best Effort ---
        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, qos_profile_sensor_data)
        self.odom_sub = self.create_subscription(Odometry,  '/odom', self.odom_callback, qos_profile_sensor_data)
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

        self.sim_time = 0.0
        self.vel_lin_pub = 0.0
        self.vel_ang_pub = 0.0

        self.get_logger().info('=== NODO MAZE SOLVER INICIADO COMPLETO ===')

    def clean(self, v):
        return 3.0 if (math.isinf(v) or math.isnan(v)) else float(v)

    def sector_min(self, ranges, a, b):
        return min(self.clean(ranges[i]) for i in range(a, b))

    def sector_promedio(self, ranges, a, b):
        valid_vals = [self.clean(ranges[i]) for i in range(a, b)]
        return sum(valid_vals) / len(valid_vals) if valid_vals else 3.0

    def promedio(self, buf):
        return sum(buf) / len(buf) if buf else 3.0

    def reset_filtros(self):
        for buf, val in [(self.buf_front, self.d_front), (self.buf_right, self.d_right),
                         (self.buf_left, self.d_left), (self.buf_back, self.d_back),
                         (self.buf_diag_izq, self.d_diag_izq), (self.buf_diag_der, self.d_diag_der)]:
            buf.clear()
            for _ in range(N_LECTURAS_PROMEDIO):
                buf.append(val)
        self.lecturas_acumuladas = N_LECTURAS_PROMEDIO

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
            
        # --- CORRECCIÓN INTEGRAL EN EL LÁSER REAL (EVITA PILARES Y CABLES) ---
        # Pasamos de min() a un promedio robusto de sector para limpiar ruidos espaciales
        front_rays = [self.clean(r[i]) for i in range(355, 360)] + [self.clean(r[i]) for i in range(0, 5)]
        self.buf_front.append(sum(front_rays) / len(front_rays))
        
        self.buf_right.append(self.sector_promedio(r, 265, 275)) 
        self.buf_left.append(self.sector_promedio(r, 85, 95))    
        
        back_rays = [self.clean(r[i]) for i in range(175, 185)]
        self.buf_back.append(sum(back_rays) / len(back_rays))
        
        self.buf_diag_izq.append(self.sector_promedio(r, 40, 50))
        self.buf_diag_der.append(self.sector_promedio(r, 310, 320))
        
        self.lecturas_acumuladas += 1
        self.d_front    = self.promedio(self.buf_front)
        self.d_right    = self.promedio(self.buf_right)
        self.d_left     = self.promedio(self.buf_left)
        self.d_back     = self.promedio(self.buf_back)
        self.d_diag_izq = self.promedio(self.buf_diag_izq)
        self.d_diag_der = self.promedio(self.buf_diag_der)
        
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            self.sim_time += 0.1
        else:
            self.sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def odom_callback(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        dist = math.sqrt((self.pos_x - self.META_X)**2 + (self.pos_y - self.META_Y)**2)
        if dist < self.DISTANCIA_MINIMA_META and not self.meta_alcanzada:
            self.meta_alcanzada = True
            self.get_logger().info(f'*** META ALCANZADA! Distancia a meta: {dist:.3f}m')

    def _cambiar_estado(self, nuevo, motivo=''):
        if nuevo != self.estado:
            self.get_logger().info(f'*** CAMBIO ESTADO: {self.estado} -> {nuevo} [{motivo}]')
            self.estado_anterior = self.estado
            self.estado = nuevo

    def _decidir_lado_giro(self):
        en_pasillo = (self.d_right < DIST_PASILLO and self.d_left < DIST_PASILLO)
        if en_pasillo:
            lado = 'izq' if self.d_diag_izq >= self.d_diag_der else 'der'
        else:
            lado = 'izq' if self.d_left >= self.d_right else 'der'
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

        d_f = self.d_front
        d_r = self.d_right
        d_l = self.d_left
        d_di = self.d_diag_izq
        d_dd = self.d_diag_der

        ahora          = time.time()
        tiempo_girando = ahora - self.tiempo_inicio_giro
        en_pasillo     = (d_r < DIST_PASILLO and d_l < DIST_PASILLO)

        # --- TELEMETRÍA EN CONSOLA PARA COMPROBAR EL LIDAR EN DIRECTO ---
        # Pon el robot en el pasillo abierto de clase y lee estos números en la pantalla:
        self.get_logger().info(f"TELEMETRIA -> Estado: {self.estado} | F: {d_f:.2f} | R: {d_r:.2f} | L: {d_l:.2f} | B: {self.d_back:.2f}")

        # -------------------------------------------------------------------
        # RADAR DE CALLEJÓN ABSOLUTO
        # -------------------------------------------------------------------
        if self.estado != 'giro_180':
            tiene_muro_delante   = (d_f <= 0.18)
            tiene_muro_izquierda = (d_l < 0.32)
            tiene_muro_derecha   = (d_r < 0.32)

            if tiene_muro_delante and tiene_muro_izquierda and tiene_muro_derecha:
                self._cambiar_estado('giro_180', 'CRÍTICO: Callejón sin salida (activando peonza bloqueante)')
                self.tiempo_inicio_giro = ahora
                self.giro_comprometido = False
                self.reset_filtros()
                return

        # -------------------------------------------------------------------
        # MAQUINA DE ESTADOS
        # -------------------------------------------------------------------
        if self.estado == 'giro_180':
            if tiempo_girando >= 5.6:
                self._cambiar_estado('avanzar', 'Giro completo de 180 grados terminado con exito')
                self.reset_filtros()
        
        else:
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
                        self._cambiar_estado('avanzar', f'frente libre d_f={d_f:.2f}')
                    elif d_f < DIST_PARAR_GIRO - 0.05:
                        self._iniciar_giro(ahora)

            elif self.estado == 'escape':
                if d_f > DIST_PARAR_GIRO:
                    self._cambiar_estado('avanzar', 'escape completado')

        # -------------------------------------------------------------------
        # APLICACIÓN DE VELOCIDADES MOTOR
        # -------------------------------------------------------------------
        if self.estado == 'pasillo':
            twist.linear.x  = VEL_LINEAR_PASILLO
            twist.angular.z = 0.0
            
        elif self.estado == 'giro_180':
            twist.linear.x = 0.0        
            twist.angular.z = VEL_GIRO  
            
        elif self.estado == 'escape':
            twist.linear.x  = -0.05
            twist.angular.z = VEL_GIRO
            
        elif self.estado == 'girar_izq':
            twist.linear.x  = VEL_AVANCE_GIRO if d_f > 0.22 else 0.0
            twist.angular.z = VEL_GIRO
            
        elif self.estado == 'girar_der':
            twist.linear.x  = VEL_AVANCE_GIRO if d_f > 0.22 else 0.0
            twist.angular.z = -VEL_GIRO
            
        else:  # avanzar
            vel = self.velocidad_frenada(d_f, VEL_LINEAR_NORMAL)
            twist.linear.x = vel
            
            if d_r < 0.50 and d_l < 0.50:
                error_centrado = d_l - d_r        
                error_angular  = d_di - d_dd      
                
                giro_total = (KP * error_centrado) + (KD_ALINEAR * error_angular)
                twist.angular.z = max(min(giro_total, 0.35), -0.35) 
            
            elif d_r < 0.50 and d_l >= 0.50:
                error = DIST_PARED_DERECHA - d_r
                twist.angular.z = max(min(KP * error, 0.25), -0.25)
            
            elif d_l < 0.40 and d_r >= 0.50:
                error = d_l - DIST_PARED_DERECHA  
                twist.angular.z = max(min(KP * error, 0.25), -0.25)
                
            else:
                if d_r > 1.2:
                    twist.angular.z = -0.15
                else:
                    error = DIST_PARED_DERECHA - d_r
                    twist.angular.z = max(min(KP * error, 0.25), -0.25)

        self.vel_lin_pub = twist.linear.x
        self.vel_ang_pub = twist.angular.z
        self.cmd_pub.publish(twist)

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