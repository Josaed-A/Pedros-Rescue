"""
sim_driver_node.py
==================
Driver simulado — emula AX-12A o EX-106+ sin hardware real.

Expone exactamente los mismos topics y servicios que los drivers reales,
de modo que la GUI y cinematica_node no necesitan distinguir entre
backend real y backend simulado.

Parametros ROS:
  driver_ns    (string, 'ax12a')  — prefijo de topics/servicios
  loop_rate_hz (int,    66)       — frecuencia del loop de control
  servos       (list)             — lista de servos (mismo formato que drivers reales)
  port         (string)           — ignorado en sim, declarado para compatibilidad YAML
  baudrate     (int)              — ignorado en sim

Comportamiento:
  - Se conecta automaticamente al arrancar
  - Mueve articulaciones gradualmente hacia el target (no teleporta)
  - Simula calibracion, estop, resume, jog y rescue_pulse
"""

import time
import math
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from rescue_robot_core.servos.params import leer_servos_params
from rescue_robot_core.servos.wheel_encoder import WheelEncoder
from rescue_interfaces.msg import ArmStatus
from rescue_interfaces.srv import ServoCommand, RegisterServo, ServoStatus

VEL_MAX_DEG_PER_SEC = 120.0


class SimDriverNode(Node):

    def __init__(self):
        super().__init__('sim_driver')

        self.declare_parameter('driver_ns',    'ax12a')
        self.declare_parameter('loop_rate_hz', 66)
        self.declare_parameter('port',         '/dev/ttyUSB0')
        self.declare_parameter('baudrate',     1_000_000)

        self._ns     = self.get_parameter('driver_ns').value
        self._rate_s = 1.0 / self.get_parameter('loop_rate_hz').value

        self._conectado  = False
        self._emergencia = False
        self._modo_calib = False
        self._running    = False

        self._pos:     dict[str, float]            = {}
        self._target:  dict[str, float | None]     = {}
        self._vel_pct: dict[str, float]            = {}
        self._id_map:  dict[int, str]              = {}
        self._lock = threading.RLock()
        self._shutdown_event = threading.Event()

        self._cargar_servos_yaml()

        self._pub_js     = self.create_publisher(JointState, '/joint_states', 10)
        self._pub_status = self.create_publisher(
            ArmStatus, f'/{self._ns}/status', 10)
        self._last_status_message = ''
        self._status_heartbeat = self.create_timer(
            0.5, lambda: self._publicar_status(self._last_status_message))

        self.create_subscription(
            JointState, f'/{self._ns}/joint_cmd', self._cb_joint_cmd, 10)

        self._crear_servicios()

        self._conectar()

        self._ctrl_thread = threading.Thread(
            target=self._bucle_control, daemon=True,
            name=f'{self._ns}-sim-ctrl')
        self._ctrl_thread.start()

        self.get_logger().info(
            f'[SIM] Driver simulado "{self._ns}" listo. '
            f'Joints: {list(self._pos.keys())}')

    # ── Carga desde YAML ─────────────────────────────────────────────

    def _cargar_servos_yaml(self):
        try:
            for s in leer_servos_params(self):
                with self._lock:
                    self._pos[s['nombre']]     = 0.0
                    self._target[s['nombre']]  = None
                    self._vel_pct[s['nombre']] = 30.0
                    self._id_map[s['id']]      = s['nombre']
        except Exception as e:
            self.get_logger().warn(f'[SIM] No se cargaron servos del YAML: {e}')

    # ── Servicios ────────────────────────────────────────────────────

    def _crear_servicios(self):
        ns = self._ns
        self.create_service(Trigger,       f'/{ns}/connect',           self._srv_connect)
        self.create_service(Trigger,       f'/{ns}/disconnect',        self._srv_disconnect)
        self.create_service(Trigger,       f'/{ns}/emergency_stop',    self._srv_estop)
        self.create_service(Trigger,       f'/{ns}/resume',            self._srv_resume)
        self.create_service(Trigger,       f'/{ns}/calibrate_start',   self._srv_calib_start)
        self.create_service(Trigger,       f'/{ns}/calibrate_confirm', self._srv_calib_confirm)
        self.create_service(ServoCommand,  f'/{ns}/jog',               self._srv_jog)
        self.create_service(ServoCommand,  f'/{ns}/rescue_pulse',      self._srv_rescue_pulse)
        self.create_service(RegisterServo, f'/{ns}/register_servo',    self._srv_register_servo)
        self.create_service(ServoStatus, f'/{ns}/servo_status', self._srv_status)
        self.create_service(Trigger, f'/{ns}/reset_alerts', self._srv_reset_alerts)

    def _srv_status(self, req, res):
        with self._lock:
            res.ids = list(self._id_map)
            res.nombres = list(self._id_map.values())
        n = len(res.ids)
        res.success = True
        res.responde = [self._conectado] * n
        res.torque_on = [self._conectado and not self._emergencia] * n
        res.error_byte = [0] * n
        res.alerta = ['SIM: temperatura/voltaje no modelados'] * n
        res.temperatura = [0] * n
        res.voltaje = [0.0] * n
        res.mensaje = 'Estado simulado; no representa telemetria fisica'
        return res

    def _srv_reset_alerts(self, req, res):
        res.success = True
        res.message = 'SIM: no se modelan alarmas fisicas; emergencia no modificada'
        return res

    def _publicar_status(self, mensaje: str = ''):
        self._last_status_message = mensaje
        msg = ArmStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.conectado    = self._conectado
        msg.emergencia   = self._emergencia
        msg.modo_calib   = self._modo_calib
        msg.mensaje      = mensaje
        self._pub_status.publish(msg)

    # ── Conexion ─────────────────────────────────────────────────────

    def _conectar(self):
        self._conectado  = True
        self._emergencia = False
        self._running    = True
        self._publicar_status(f'[SIM] {self._ns} conectado')

    def _desconectar(self):
        self._running   = False
        self._conectado = False
        self._publicar_status('Desconectado')

    # ── Servicios Trigger ────────────────────────────────────────────

    def _srv_connect(self, req, res):
        self._conectar()
        res.success = True
        res.message = f'[SIM] {self._ns} conectado'
        return res

    def _srv_disconnect(self, req, res):
        self._desconectar()
        res.success = True
        res.message = 'Desconectado'
        return res

    def _srv_estop(self, req, res):
        self._emergencia = True
        with self._lock:
            for k in self._target:
                self._target[k] = None
        res.success = True
        res.message = 'Paro de emergencia activo'
        self._publicar_status('Paro de emergencia activo')
        return res

    def _srv_resume(self, req, res):
        self._emergencia = False
        res.success = True
        res.message = 'Control reanudado'
        self._publicar_status('Control reanudado')
        return res

    def _srv_calib_start(self, req, res):
        with self._lock:
            for k in self._target:
                self._target[k] = None
        self._modo_calib = True
        res.success = True
        res.message = 'Calibracion iniciada'
        self._publicar_status('Calibracion iniciada')
        return res

    def _srv_calib_confirm(self, req, res):
        with self._lock:
            for k in self._pos:
                self._pos[k]    = 0.0
                self._target[k] = None
        self._modo_calib = False
        res.success = True
        res.message = 'Home = 0 confirmado'
        self._publicar_status('Home = 0 confirmado')
        return res

    def _srv_jog(self, req, res):
        if self._emergencia:
            res.success = False
            res.mensaje = 'Paro de emergencia activo'
            return res
        if not self._conectado:
            res.success = False
            res.mensaje = 'Driver no conectado'
            return res
        nombre = self._id_map.get(req.id)
        if nombre is None:
            res.success = False
            res.mensaje = f'ID {req.id} no registrado'
            return res
        with self._lock:
            self._target[nombre] = float(req.target_deg) % 360.0
            if req.vel_pct > 0:
                self._vel_pct[nombre] = float(req.vel_pct)
        res.success = True
        res.mensaje = f'Jog {nombre} → {req.target_deg:.1f}°'
        return res

    def _srv_rescue_pulse(self, req, res):
        if self._emergencia:
            res.success = False
            res.mensaje = 'Paro de emergencia activo'
            return res
        if not self._conectado:
            res.success = False
            res.mensaje = 'Driver no conectado'
            return res
        nombre = self._id_map.get(req.id)
        if nombre is None:
            res.success = False
            res.mensaje = f'ID {req.id} no registrado'
            return res
        d   = 1 if req.target_deg >= 0 else -1
        vel = max(5.0, min(30.0, float(req.vel_pct) if req.vel_pct > 0 else 15.0))
        with self._lock:
            self._target[nombre] = None
            self._pos[nombre]    = (self._pos[nombre] + d * vel * 0.25) % 360.0
        res.success = True
        res.mensaje = f'Pulso rescate {nombre} OK'
        return res

    def _srv_register_servo(self, req, res):
        nombre = req.nombre
        with self._lock:
            if nombre in self._pos:
                res.success = False
                res.mensaje = f'"{nombre}" ya registrado'
                return res
            self._pos[nombre]     = 0.0
            self._target[nombre]  = None
            self._vel_pct[nombre] = 30.0
            self._id_map[req.id]  = nombre
        res.success = True
        res.mensaje = f'Servo "{nombre}" (ID {req.id}) registrado'
        return res

    # ── Callback comandos ────────────────────────────────────────────

    def _cb_joint_cmd(self, msg: JointState):
        if self._emergencia or not self._conectado or self._modo_calib:
            return
        with self._lock:
            for i, (name, pos) in enumerate(zip(msg.name, msg.position)):
                if name in self._target:
                    self._target[name] = np.degrees(pos) % 360.0
                    if i < len(msg.velocity) and msg.velocity[i] > 0:
                        self._vel_pct[name] = float(msg.velocity[i])

    # ── Loop de control ──────────────────────────────────────────────

    def _bucle_control(self):
        while not self._shutdown_event.is_set():
            if not self._running or not self._conectado or self._emergencia:
                time.sleep(0.05)
                continue

            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()

            with self._lock:
                for nombre in list(self._pos.keys()):
                    current = self._pos[nombre]
                    target  = self._target[nombre]

                    if target is not None:
                        error = WheelEncoder.error_circular(target, current)
                        if abs(error) < 0.5:
                            self._pos[nombre]    = target
                            self._target[nombre] = None
                        else:
                            vel_deg = (self._vel_pct[nombre] / 100.0) * VEL_MAX_DEG_PER_SEC
                            step    = min(abs(error), vel_deg * self._rate_s)
                            self._pos[nombre] = (current + math.copysign(step, error)) % 360.0

                    js.name.append(nombre)
                    # Match the signed feedback convention of the real encoder.
                    js.position.append(np.radians((self._pos[nombre] + 180.0) % 360.0 - 180.0))

            if js.name:
                self._pub_js.publish(js)

            time.sleep(self._rate_s)

    # ── Cleanup ──────────────────────────────────────────────────────

    def destroy_node(self):
        self._shutdown_event.set()
        self._ctrl_thread.join(timeout=1)
        self._desconectar()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SimDriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
