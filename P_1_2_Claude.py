#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import math
import time
from collections import deque

# ─── PARÁMETROS AJUSTABLES ────────────────────────────────────────────────────
DIST_PARAR_GIRO       = 0.28   # Frente < esto → parar (zona de peligro)
DIST_FRENAR           = 0.55   # Frente < esto → empezar a frenar
DIST_PARED_DERECHA    = 0.25   # Distancia ideal a pared derecha
DIST_PASILLO          = 0.45   # Der Y Izq < esto → pasillo
DIST_ESQUINA_CERRADA  = 0.22   # Todo < esto → escape

# Para girar necesitamos espacio libre en el lado al que vamos a girar
# El robot burger tiene ~0.18m de radio, necesitamos al menos 0.35m libres
ESPACIO_MINIMO_GIRO   = 0.35

VEL_LINEAR_PASILLO    = 0.06
VEL_LINEAR_NORMAL     = 0.08
VEL_RETROCESO         = 0.05   # Velocidad de retroceso antes de girar sin espacio
VEL_GIRO              = 0.38
KP                    = 1.2

TIEMPO_GIRO_MINIMO    = 1.2
TIEMPO_RETROCESO_MAX  = 1.5    # Máximo segundos retrocediendo para buscar espacio
N_LECTURAS_PROMEDIO   = 5
TICKS_CONFIRMACION    = 4

LOG_FILE = '/home/ros/Escriptori/Robotica/maze_log.txt'
# ──────────────────────────────────────────────────────────────────────────────

class MazeSolver(Node):
    def __init__(self):
        super().__init__('maze_solver_node')

        self.cmd_pub  = self.create_publisher(Twist, '/cmd_vel', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry,  '/odom', self.odom_callback, 10)
        self.timer    = self.create_timer(0.1, self.control_loop)

        self.buf_front = deque(maxlen=N_LECTURAS_PROMEDIO)
        self.buf_right = deque(maxlen=N_LECTURAS_PROMEDIO)
        self.buf_left  = deque(maxlen=N_LECTURAS_PROMEDIO)
        self.buf_back  = deque(maxlen=N_LECTURAS_PROMEDIO)

        self.d_front = 3.0
        self.d_right = 3.0
        self.d_left  = 3.0
        self.d_back  = 3.0

        self.pos_x = 0.0
        self.pos_y = 0.0

        self.lecturas_acumuladas = 0

        # Estados:
        # 'esperando' | 'pasillo' | 'avanzar' | 'retroceder'
        # | 'girar_izq' | 'girar_der' | 'escape' | 'meta'
        self.estado          = 'esperando'
        self.estado_anterior = 'esperando'

        self.tiempo_inicio_giro      = 0.0
        self.tiempo_inicio_retroceso = 0.0
        self.giro_comprometido       = False
        self.lado_giro_pendiente     = None   # 'izq' o 'der', guardado durante retroceso
        self.ticks_fuera_pasillo     = 0

        self.META_X               = 2.75
        self.META_Y               = 1.71
        self.DISTANCIA_MINIMA_META = 0.25
        self.meta_alcanzada       = False

        # Log
        self.log_file = open(LOG_FILE, 'w')
        self._escribir_log('=== INICIO SESION MAZE SOLVER ===')
        self._escribir_log(
            f'Parametros: DIST_PARAR={DIST_PARAR_GIRO} DIST_FRENAR={DIST_FRENAR} '
            f'DIST_PASILLO={DIST_PASILLO} ESPACIO_GIRO={ESPACIO_MINIMO_GIRO} '
            f'VEL_NORMAL={VEL_LINEAR_NORMAL} VEL_PASILLO={VEL_LINEAR_PASILLO} '
            f'VEL_GIRO={VEL_GIRO} KP={KP} T_GIRO_MIN={TIEMPO_GIRO_MINIMO}'
        )
        self._escribir_log(
            'TIEMPO_SIM | POS_X  | POS_Y  | ESTADO      | '
            'F     | R     | L     | B     | VEL_LIN | VEL_ANG | EVENTO'
        )
        self._escribir_log('-' * 120)

        self.tick_count  = 0
        self.sim_time    = 0.0
        self.vel_lin_pub = 0.0
        self.vel_ang_pub = 0.0

    # ── Log ───────────────────────────────────────────────────────────────────
    def _escribir_log(self, msg):
        ts = time.strftime('%H:%M:%S')
        linea = f'[{ts}] {msg}'
        self.log_file.write(linea + '\n')
        self.log_file.flush()

    def _log_tick(self, evento=''):
        linea = (
            f'{self.sim_time:10.2f} | '
            f'{self.pos_x:+6.3f} | {self.pos_y:+6.3f} | '
            f'{self.estado:<12}| '
            f'{self.d_front:5.2f} | {self.d_right:5.2f} | '
            f'{self.d_left:5.2f} | {self.d_back:5.2f} | '
            f'{self.vel_lin_pub:+7.3f} | {self.vel_ang_pub:+7.3f} | '
            f'{evento}'
        )
        self._escribir_log(linea)

    def _log_evento(self, evento):
        """Log de eventos importantes (cambios de estado, decisiones clave)."""
        self._escribir_log(
            f'*** EVENTO: {evento} | estado={self.estado} '
            f'pos=({self.pos_x:.3f},{self.pos_y:.3f}) '
            f'F={self.d_front:.2f} R={self.d_right:.2f} L={self.d_left:.2f}'
        )

    # ── Utilidad ──────────────────────────────────────────────────────────────
    def clean(self, v):
        return 3.0 if (math.isinf(v) or math.isnan(v)) else float(v)

    def sector_min(self, ranges, a, b):
        return min(self.clean(ranges[i]) for i in range(a, b))

    def promedio(self, buf):
        return sum(buf) / len(buf) if buf else 3.0

    def velocidad_frenada(self, d_front, vel_max):
        if d_front >= DIST_FRENAR:
            return vel_max
        if d_front <= DIST_PARAR_GIRO:
            return 0.0
        ratio = (d_front - DIST_PARAR_GIRO) / (DIST_FRENAR - DIST_PARAR_GIRO)
        return round(vel_max * ratio, 3)

    # ── Callbacks ─────────────────────────────────────────────────────────────
    def scan_callback(self, msg):
        r = msg.ranges
        if len(r) < 360:
            return
        self.buf_front.append(min(self.sector_min(r, 350, 360),
                                  self.sector_min(r,   0,  10)))
        self.buf_right.append(self.sector_min(r, 260, 310))
        self.buf_left.append( self.sector_min(r,  50, 110))
        self.buf_back.append( self.sector_min(r, 170, 190))
        self.lecturas_acumuladas += 1
        self.d_front = self.promedio(self.buf_front)
        self.d_right = self.promedio(self.buf_right)
        self.d_left  = self.promedio(self.buf_left)
        self.d_back  = self.promedio(self.buf_back)
        self.sim_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

    def odom_callback(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        dist = math.sqrt((self.pos_x - self.META_X)**2 + (self.pos_y - self.META_Y)**2)
        if dist < self.DISTANCIA_MINIMA_META:
            if not self.meta_alcanzada:
                self._log_evento(f'META ALCANZADA a distancia={dist:.3f}')
            self.meta_alcanzada = True

    # ── Lógica principal ──────────────────────────────────────────────────────
    def _cambiar_estado(self, nuevo, motivo=''):
        if nuevo != self.estado:
            self._log_evento(f'{self.estado} -> {nuevo}  motivo: {motivo}')
            self.estado_anterior = self.estado
            self.estado = nuevo

    def _iniciar_retroceso(self, lado_pendiente):
        """Retroceder para ganar espacio antes de girar."""
        self.lado_giro_pendiente     = lado_pendiente
        self.tiempo_inicio_retroceso = time.time()
        self._cambiar_estado('retroceder',
                             f'sin espacio para girar_{lado_pendiente} '
                             f'(L={self.d_left:.2f} R={self.d_right:.2f})')

    def _iniciar_giro(self, d_l, d_r, ahora):
        """
        Decide el lado del giro. Si no hay espacio suficiente en ese lado,
        retrocede primero para crear distancia.
        """
        lado = 'izq' if d_l >= d_r else 'der'
        espacio = d_l if lado == 'izq' else d_r

        if espacio < ESPACIO_MINIMO_GIRO:
            # No hay espacio: retroceder para crearlo
            self._iniciar_retroceso(lado)
        else:
            # Hay espacio: girar directamente
            self.estado = f'girar_{lado}'
            self.tiempo_inicio_giro = ahora
            self.giro_comprometido  = True
            self._log_evento(
                f'Giro {lado} directo, espacio={espacio:.2f}m')

    def control_loop(self):
        twist = Twist()
        self.tick_count += 1

        if self.meta_alcanzada:
            self._log_tick('META - robot detenido')
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
        tiempo_retro   = ahora - self.tiempo_inicio_retroceso

        en_pasillo      = (d_r < DIST_PASILLO and d_l < DIST_PASILLO)
        esquina_cerrada = (d_f < DIST_ESQUINA_CERRADA and
                           d_r < DIST_ESQUINA_CERRADA + 0.05 and
                           d_l < DIST_ESQUINA_CERRADA + 0.05)

        # ── MÁQUINA DE ESTADOS ────────────────────────────────────────────────

        if esquina_cerrada:
            self._cambiar_estado('escape', 'esquina cerrada')
            self.giro_comprometido = False

        elif self.estado == 'pasillo':
            if en_pasillo:
                self.ticks_fuera_pasillo = 0
                if d_f < DIST_PARAR_GIRO:
                    self._iniciar_giro(d_l, d_r, ahora)
            else:
                self.ticks_fuera_pasillo += 1
                if self.ticks_fuera_pasillo >= TICKS_CONFIRMACION:
                    self._cambiar_estado('avanzar', 'salida de pasillo confirmada')
                    self.ticks_fuera_pasillo = 0

        elif self.estado == 'avanzar':
            if en_pasillo and d_f >= DIST_PARAR_GIRO:
                self._cambiar_estado('pasillo', 'pasillo detectado')
                self.ticks_fuera_pasillo = 0
            elif d_f < DIST_PARAR_GIRO:
                self._iniciar_giro(d_l, d_r, ahora)

        elif self.estado == 'retroceder':
            # Retrocedemos hasta que haya espacio suficiente en el lado pendiente
            espacio_actual = d_l if self.lado_giro_pendiente == 'izq' else d_r
            if espacio_actual >= ESPACIO_MINIMO_GIRO:
                self._log_evento(
                    f'Espacio conseguido tras retroceso: {espacio_actual:.2f}m')
                self.estado = f'girar_{self.lado_giro_pendiente}'
                self.tiempo_inicio_giro = ahora
                self.giro_comprometido  = True
            elif tiempo_retro >= TIEMPO_RETROCESO_MAX:
                # Tiempo máximo agotado: intentar girar igualmente
                self._log_evento(
                    f'Retroceso timeout ({tiempo_retro:.1f}s), forzando giro')
                self.estado = f'girar_{self.lado_giro_pendiente}'
                self.tiempo_inicio_giro = ahora
                self.giro_comprometido  = True

        elif self.estado in ('girar_izq', 'girar_der'):
            if self.giro_comprometido:
                if tiempo_girando >= TIEMPO_GIRO_MINIMO:
                    self.giro_comprometido = False
            else:
                if d_f >= DIST_PARAR_GIRO + 0.10:
                    self._cambiar_estado('avanzar', f'frente despejado tras giro ({d_f:.2f}m)')
                elif d_f < DIST_PARAR_GIRO:
                    self._iniciar_giro(d_l, d_r, ahora)

        elif self.estado == 'escape':
            if d_f > DIST_PARAR_GIRO + 0.15:
                self._cambiar_estado('avanzar', 'escape completado')

        # ── ACCIONES ──────────────────────────────────────────────────────────
        evento_tick = ''

        if self.estado == 'pasillo':
            vel = self.velocidad_frenada(d_f, VEL_LINEAR_PASILLO)
            twist.linear.x  = vel
            twist.angular.z = 0.0
            if vel < VEL_LINEAR_PASILLO:
                evento_tick = f'frenando_pasillo vel={vel:.3f}'

        elif self.estado == 'retroceder':
            twist.linear.x  = -VEL_RETROCESO
            twist.angular.z = 0.0
            evento_tick = f'retrocediendo t={tiempo_retro:.1f}s lado={self.lado_giro_pendiente}'

        elif self.estado == 'escape':
            twist.linear.x  = -0.05
            twist.angular.z =  VEL_GIRO
            evento_tick = 'escape activo'

        elif self.estado == 'girar_izq':
            twist.linear.x  = 0.0
            twist.angular.z = VEL_GIRO
            evento_tick = f'girando_izq t={tiempo_girando:.1f}s comprometido={self.giro_comprometido}'

        elif self.estado == 'girar_der':
            twist.linear.x  = 0.0
            twist.angular.z = -VEL_GIRO
            evento_tick = f'girando_der t={tiempo_girando:.1f}s comprometido={self.giro_comprometido}'

        else:  # avanzar
            vel = self.velocidad_frenada(d_f, VEL_LINEAR_NORMAL)
            twist.linear.x = vel
            if d_r > 1.2:
                twist.angular.z = -0.20
                evento_tick = 'buscando_pared_der'
            else:
                error = DIST_PARED_DERECHA - d_r
                twist.angular.z = max(min(KP * error, 0.40), -0.40)
                evento_tick = f'siguiendo_pared err={error:.3f}'
            if vel < VEL_LINEAR_NORMAL:
                evento_tick += f' frenando vel={vel:.3f}'

        self.vel_lin_pub = twist.linear.x
        self.vel_ang_pub = twist.angular.z

        # Log cada tick (cada 0.1s) y siempre en eventos importantes
        self._log_tick(evento_tick)

        self.cmd_pub.publish(twist)

    def __del__(self):
        try:
            self._escribir_log('=== FIN SESION ===')
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
        nodo._escribir_log('=== INTERRUPCION USUARIO ===')
        nodo.cmd_pub.publish(Twist())
        nodo.log_file.close()
        nodo.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
