#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time
from collections import deque

# ---------------------------------------------------------------------------
# PARÁMETROS
# ---------------------------------------------------------------------------
DIST_GIRO_PASILLO      = 0.32
DIST_PARAR_GIRO        = 0.32
DIST_FRENAR            = 0.55
DIST_PARED_DERECHA     = 0.25
DIST_PASILLO           = 0.45
DIST_SEGURIDAD_TRASERA = 0.15

VEL_LINEAR_PASILLO = 0.06
VEL_LINEAR_NORMAL  = 0.08
VEL_RETROCESO      = 0.05
VEL_GIRO           = 0.28
VEL_AVANCE_GIRO    = 0.06
KP                 = 1.2

TIEMPO_GIRO_MINIMO  = 1.5
N_LECTURAS_PROMEDIO = 5
TICKS_CONFIRMACION  = 4

# --- PARÁMETROS DEL CALLEJÓN SIN SALIDA ---
# Tras MAX_GIROS_FALLIDOS giros donde al terminar frente Y laterales siguen cerrados
# se confirma callejón → se activa marcha atrás.
MAX_GIROS_FALLIDOS    = 2
DIST_LATERAL_LIBRE    = 0.38  # lateral >= esto → hay camino, NO es callejón
DIST_FRENTE_BLOQUEADO = 0.32  # frente < esto → frente cerrado

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
        self.lecturas_acumuladas = 0

        self.estado          = 'esperando'
        self.estado_anterior = 'esperando'

        self.tiempo_inicio_giro  = 0.0
        self.giro_comprometido   = False
        self.ticks_fuera_pasillo = 0

        # Contador de giros fallidos consecutivos en callejón
        self.giros_fallidos_callejon = 0

        # Tiempo en que empezamos a retroceder (para timeout de seguridad)
        self.tiempo_inicio_retroceso = 0.0

        self.META_X               = 2.75
        self.META_Y               = 1.71
        self.DISTANCIA_MINIMA_META = 0.25
        self.meta_alcanzada        = False

        self.sim_time    = 0.0
        self.vel_lin_pub = 0.0
        self.vel_ang_pub = 0.0

        self.log_file = open(LOG_FILE, 'w')
        self._log_raw('=== INICIO SESION MAZE SOLVER ===')
        self._log_raw(
            f'Params: DIST_GIRO={DIST_GIRO_PASILLO} DIST_PARAR={DIST_PARAR_GIRO} '
            f'VEL_NORMAL={VEL_LINEAR_NORMAL} VEL_GIRO={VEL_GIRO} '
            f'VEL_AVANCE_GIRO={VEL_AVANCE_GIRO} MAX_GIROS_FALLIDOS={MAX_GIROS_FALLIDOS}'
        )
        self._log_raw(
            'TSIM      | POS_X  | POS_Y  | ESTADO       | '
            'F     | R     | L     | DI    | DD    | VL      | VA      | EVENTO'
        )
        self._log_raw('-' * 130)

    # -----------------------------------------------------------------------
    # Logging
    # -----------------------------------------------------------------------

    def _log_raw(self, msg):
        ts = time.strftime('%H:%M:%S')
        self.log_file.write(f'[{ts}] {msg}\n')
        self.log_file.flush()

    def _log_tick(self, evento=''):
        self._log_raw(
            f'{self.sim_time:9.2f} | {self.pos_x:+6.3f} | {self.pos_y:+6.3f} | '
            f'{self.estado:<13}| '
            f'{self.d_front:5.2f} | {self.d_right:5.2f} | {self.d_left:5.2f} | '
            f'{self.d_diag_izq:5.2f} | {self.d_diag_der:5.2f} | '
            f'{self.vel_lin_pub:+7.3f} | {self.vel_ang_pub:+7.3f} | {evento}'
        )

    def _log_evento(self, msg):
        self._log_raw(
            f'*** {msg} | estado={self.estado} '
            f'pos=({self.pos_x:.3f},{self.pos_y:.3f}) '
            f'F={self.d_front:.2f} R={self.d_right:.2f} L={self.d_left:.2f} '
            f'DI={self.d_diag_izq:.2f} DD={self.d_diag_der:.2f}'
        )

    # -----------------------------------------------------------------------
    # Utilidades
    # -----------------------------------------------------------------------

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

    def _callejon_confirmado(self):
        """True si frente Y ambos laterales están cerrados (callejón real, no curva)."""
        return (self.d_front < DIST_FRENTE_BLOQUEADO and
                self.d_left  < DIST_LATERAL_LIBRE and
                self.d_right < DIST_LATERAL_LIBRE)

    # -----------------------------------------------------------------------
    # Callbacks
    # -----------------------------------------------------------------------

    def scan_callback(self, msg):
        r = msg.ranges
        if len(r) < 360:
            return
        self.buf_front.append(min(self.sector_min(r, 350, 360),
                                  self.sector_min(r,   0,  10)))
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
        dist = math.sqrt((self.pos_x - self.META_X)**2 + (self.pos_y - self.META_Y)**2)
        if dist < self.DISTANCIA_MINIMA_META and not self.meta_alcanzada:
            self.meta_alcanzada = True
            self._log_evento(f'META ALCANZADA dist={dist:.3f}')

    # -----------------------------------------------------------------------
    # Gestión de estados
    # -----------------------------------------------------------------------

    def _cambiar_estado(self, nuevo, motivo=''):
        if nuevo != self.estado:
            self._log_evento(f'ESTADO {self.estado} -> {nuevo}  [{motivo}]')
            self.estado_anterior = self.estado
            self.estado = nuevo

    def _decidir_lado_giro(self):
        en_pasillo = (self.d_right < DIST_PASILLO and self.d_left < DIST_PASILLO)
        if en_pasillo:
            lado = 'izq' if self.d_diag_izq >= self.d_diag_der else 'der'
            self._log_evento(
                f'Giro en PASILLO por diagonal: lado={lado} '
                f'DI={self.d_diag_izq:.2f} DD={self.d_diag_der:.2f}'
            )
        else:
            lado = 'izq' if self.d_left >= self.d_right else 'der'
            self._log_evento(f'Giro NORMAL: lado={lado} L={self.d_left:.2f} R={self.d_right:.2f}')
        return lado

    def _iniciar_giro(self, ahora):
        lado = self._decidir_lado_giro()
        self._cambiar_estado(f'girar_{lado}', f'giro lado={lado}')
        self.tiempo_inicio_giro = ahora
        self.giro_comprometido  = True

    def _iniciar_retroceso(self, ahora, motivo=''):
        """Activa marcha atrás. SOLO se llama cuando el callejón está confirmado."""
        self._cambiar_estado('retroceder', motivo)
        self.giro_comprometido       = False
        self.giros_fallidos_callejon = 0
        self.tiempo_inicio_retroceso = ahora
        self.reset_filtros()

    # -----------------------------------------------------------------------
    # Bucle de control
    # -----------------------------------------------------------------------

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

        d_f = self.d_front
        d_r = self.d_right
        d_l = self.d_left

        ahora          = time.time()
        tiempo_girando = ahora - self.tiempo_inicio_giro
        en_pasillo     = (d_r < DIST_PASILLO and d_l < DIST_PASILLO)

        # -------------------------------------------------------------------
        # MÁQUINA DE ESTADOS
        # -------------------------------------------------------------------

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
                # No interrumpimos el giro hasta que pase el tiempo mínimo
                if tiempo_girando >= TIEMPO_GIRO_MINIMO:
                    self.giro_comprometido = False

                    if d_f >= DIST_PARAR_GIRO + 0.10:
                        # ✓ Giro exitoso: frente libre → avanzar
                        self.giros_fallidos_callejon = 0
                        self._cambiar_estado('avanzar', f'frente libre tras giro d_f={d_f:.2f}')

                    elif self._callejon_confirmado():
                        # ✗ Giro fallido en callejón: frente Y laterales cerrados
                        self.giros_fallidos_callejon += 1
                        self._log_evento(
                            f'GIRO FALLIDO EN CALLEJON #{self.giros_fallidos_callejon} '
                            f'F={d_f:.2f} L={self.d_left:.2f} R={self.d_right:.2f}'
                        )
                        if self.giros_fallidos_callejon >= MAX_GIROS_FALLIDOS:
                            # Callejón confirmado → marcha atrás
                            self._iniciar_retroceso(
                                ahora,
                                f'CALLEJON CONFIRMADO tras {self.giros_fallidos_callejon} giros fallidos'
                            )
                        else:
                            # Aún no suficientes fallos: intentar girar al otro lado
                            self._iniciar_giro(ahora)

                    else:
                        # Frente cerrado pero hay hueco lateral → es una curva normal
                        # No contabilizamos como fallo de callejón
                        self.giros_fallidos_callejon = 0
                        self._iniciar_giro(ahora)

            else:
                # Fuera del tiempo comprometido: si el frente ya está libre, avanzar
                if d_f >= DIST_PARAR_GIRO + 0.10:
                    self.giros_fallidos_callejon = 0
                    self._cambiar_estado('avanzar', f'frente libre d_f={d_f:.2f}')

        elif self.estado == 'retroceder':
            # Durante el retroceso solo miramos si aparece hueco lateral
            # Si la pared trasera está pegada, paramos de retroceder pero NO giramos
            # (no hay espacio). Esperamos al timeout de seguridad si todo falla.
            tiempo_retrocediendo = ahora - self.tiempo_inicio_retroceso

            if self.d_left >= DIST_LATERAL_LIBRE or self.d_right >= DIST_LATERAL_LIBRE:
                lado = 'izq' if self.d_left >= self.d_right else 'der'
                self._log_evento(
                    f'Hueco lateral al retroceder: lado={lado} '
                    f'L={self.d_left:.2f} R={self.d_right:.2f}'
                )
                self._cambiar_estado(f'girar_{lado}', f'salida lateral retrocediendo hacia {lado}')
                self.tiempo_inicio_giro = ahora
                self.giro_comprometido  = True

            elif tiempo_retrocediendo >= 4.0:
                # Timeout de seguridad: 4 s retrocediendo sin encontrar hueco lateral.
                # Giramos a la fuerza hacia el lado con más espacio (aunque sea poco).
                lado = 'izq' if self.d_left >= self.d_right else 'der'
                self._log_evento(
                    f'TIMEOUT RETROCESO 4s → giro forzado {lado} '
                    f'L={self.d_left:.2f} R={self.d_right:.2f} B={self.d_back:.2f}'
                )
                self._cambiar_estado(f'girar_{lado}', f'timeout retroceso forzado {lado}')
                self.tiempo_inicio_giro = ahora
                self.giro_comprometido  = True

        elif self.estado == 'escape':
            if d_f > DIST_PARAR_GIRO:
                self._cambiar_estado('avanzar', 'escape completado')

        # -------------------------------------------------------------------
        # APLICACIÓN DE VELOCIDADES
        # -------------------------------------------------------------------
        evento = ''

        if self.estado == 'pasillo':
            twist.linear.x  = VEL_LINEAR_PASILLO
            twist.angular.z = 0.0
            evento = f'pasillo_recto f={d_f:.2f}'

        elif self.estado == 'retroceder':
            # Retrocede recto. Se para si la pared trasera está pegada.
            # NO gira en este bloque: el giro lo decide la máquina de estados arriba.
            if self.d_back > DIST_SEGURIDAD_TRASERA:
                twist.linear.x = -VEL_RETROCESO
                evento = f'retrocediendo B={self.d_back:.2f} t={ahora - self.tiempo_inicio_retroceso:.1f}s'
            else:
                twist.linear.x = 0.0
                evento = f'retroceso_parado_pared_trasera B={self.d_back:.2f}'
            twist.angular.z = 0.0

        elif self.estado == 'escape':
            twist.linear.x  = -0.05
            twist.angular.z = VEL_GIRO
            evento = 'escape'

        elif self.estado == 'girar_izq':
            twist.linear.x  = VEL_AVANCE_GIRO if d_f > 0.22 else 0.0
            twist.angular.z = VEL_GIRO
            evento = f'girar_izq arco={twist.linear.x > 0} t={tiempo_girando:.1f}s'

        elif self.estado == 'girar_der':
            twist.linear.x  = VEL_AVANCE_GIRO if d_f > 0.22 else 0.0
            twist.angular.z = -VEL_GIRO
            evento = f'girar_der arco={twist.linear.x > 0} t={tiempo_girando:.1f}s'

        else:  # avanzar
            vel = self.velocidad_frenada(d_f, VEL_LINEAR_NORMAL)
            twist.linear.x = vel
            if d_r > 1.2:
                twist.angular.z = -0.20
                evento = f'buscando_pared vel={vel:.3f}'
            else:
                error = DIST_PARED_DERECHA - d_r
                twist.angular.z = max(min(KP * error, 0.40), -0.40)
                evento = f'siguiendo_pared_der err={error:.3f} vel={vel:.3f}'
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