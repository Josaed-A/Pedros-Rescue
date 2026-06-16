"""
ex106_driver_node.py
====================
Nodo ROS2 — driver para servos Dynamixel EX-106+ conectados via RS485.

Logica identica a ax12a_driver_node con las diferencias:
  - Baudrate 57600
  - Puerto RS485 independiente (/dev/ttyUSB1)
  - Namespace de servicios /ex106/...
  - Topic de status /ex106/status

Topics publicados:
  /joint_states           (sensor_msgs/JointState)
  /ex106/status           (control_brazo/ArmStatus)

Topics suscritos:
  /ex106/joint_cmd        (sensor_msgs/JointState)
    name[]     — nombres de joints
    position[] — angulos en radianes
    velocity[] — vel_pct por joint (opcional)

Servicios:
  /ex106/connect           (std_srvs/Trigger)
  /ex106/disconnect        (std_srvs/Trigger)
  /ex106/emergency_stop    (std_srvs/Trigger)
  /ex106/resume            (std_srvs/Trigger)
  /ex106/calibrate_start   (std_srvs/Trigger)
  /ex106/calibrate_confirm (std_srvs/Trigger)
  /ex106/jog               (control_brazo/ServoCommand)
  /ex106/rescue_pulse      (control_brazo/ServoCommand)
  /ex106/register_servo    (control_brazo/RegisterServo)
"""

import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

from control_brazo.servo_params import leer_servos_params
from control_brazo.wheel_encoder import WheelEncoder
from control_brazo.msg import ArmStatus
from control_brazo.srv import ServoCommand, RegisterServo, ServoStatus

ADDR_CW_LIMIT     =  6
ADDR_CCW_LIMIT    =  8
ADDR_TORQUE_EN    = 24
ADDR_MOVING_SPEED = 32
ADDR_PRESENT_POS  = 36
ADDR_PRESENT_VOLT = 42
ADDR_PRESENT_TEMP = 43

PROTOCOL = 1.0


def _describir_error(err: int) -> str:
    """Decodifica el byte de error del status (protocolo Dynamixel 1.0)."""
    if not err:
        return ''
    bits = [
        (0x01, 'voltaje'),
        (0x02, 'limite de angulo'),
        (0x04, 'sobrecalentamiento'),
        (0x08, 'rango'),
        (0x10, 'checksum'),
        (0x20, 'sobrecarga'),
        (0x40, 'instruccion'),
    ]
    return ', '.join(txt for mask, txt in bits if err & mask)


class EX106DriverNode(Node):

    def __init__(self):
        super().__init__('ex106_driver')

        self.declare_parameter('port',         '/dev/ttyUSB1')
        self.declare_parameter('baudrate',     57_600)
        self.declare_parameter('loop_rate_hz', 66)

        self._port_name = self.get_parameter('port').value
        self._baudrate  = self.get_parameter('baudrate').value

        self._port      : PortHandler = None
        self._ph         = PacketHandler(PROTOCOL)
        self._lock       = threading.RLock()
        self._conectado  = False
        self._emergencia = False
        self._modo_calib = False
        self._running    = False

        self._encoders: dict[int, WheelEncoder] = {}
        self._nombres:  dict[int, str]          = {}
        self._activos:  set[int]                = set()
        self._enc_lock  = threading.RLock()

        self._cargar_servos_yaml()

        self._pub_joint_states = self.create_publisher(
            JointState, '/joint_states', 10)
        self._pub_status = self.create_publisher(
            ArmStatus, '/ex106/status', 10)

        self.create_subscription(
            JointState, '/ex106/joint_cmd', self._cb_joint_cmd, 10)

        self._crear_servicios()
        self._ctrl_thread = None

        self.get_logger().info(
            f'EX-106+ driver listo. Puerto: {self._port_name} | '
            f'Servos en YAML: {list(self._encoders.keys())}')

    # ------------------------------------------------------------------
    #  Inicializacion desde YAML
    # ------------------------------------------------------------------

    def _cargar_servos_yaml(self):
        try:
            for s in leer_servos_params(self):
                enc = WheelEncoder(reduccion=s['reduccion'])
                enc.deadband_deg  = s['deadband_deg']
                enc.invertir_giro = s['invertir_giro']
                with self._enc_lock:
                    self._encoders[s['id']] = enc
                    self._nombres[s['id']]  = s['nombre']
        except Exception as e:
            self.get_logger().warn(f'No se cargaron servos del YAML: {e}')

    # ------------------------------------------------------------------
    #  Registro de servicios
    # ------------------------------------------------------------------

    def _crear_servicios(self):
        self.create_service(Trigger,       '/ex106/connect',           self._srv_connect)
        self.create_service(Trigger,       '/ex106/disconnect',        self._srv_disconnect)
        self.create_service(Trigger,       '/ex106/emergency_stop',    self._srv_estop)
        self.create_service(Trigger,       '/ex106/resume',            self._srv_resume)
        self.create_service(Trigger,       '/ex106/calibrate_start',   self._srv_calib_start)
        self.create_service(Trigger,       '/ex106/calibrate_confirm', self._srv_calib_confirm)
        self.create_service(ServoCommand,  '/ex106/jog',               self._srv_jog)
        self.create_service(ServoCommand,  '/ex106/rescue_pulse',      self._srv_rescue_pulse)
        self.create_service(RegisterServo, '/ex106/register_servo',    self._srv_register_servo)
        self.create_service(ServoStatus,   '/ex106/servo_status',      self._srv_status)
        self.create_service(Trigger,       '/ex106/reset_alerts',      self._srv_reset_alerts)

    def _publicar_status(self, mensaje: str = ''):
        msg = ArmStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.conectado    = self._conectado
        msg.emergencia   = self._emergencia
        msg.modo_calib   = self._modo_calib
        msg.mensaje      = mensaje
        self._pub_status.publish(msg)

    # ------------------------------------------------------------------
    #  Servicios Trigger
    # ------------------------------------------------------------------

    def _srv_connect(self, req, res):
        try:
            ok, msg = self._conectar()
        except Exception as e:
            ok, msg = False, f'Error al conectar: {e}'
        res.success = ok
        res.message = msg
        self._publicar_status(msg)
        return res

    def _srv_disconnect(self, req, res):
        try:
            self._desconectar()
        except Exception as e:
            self.get_logger().warn(f'Error al desconectar: {e}')
        res.success = True
        res.message = 'Desconectado'
        self._publicar_status('Desconectado')
        return res

    def _srv_estop(self, req, res):
        self._emergencia = True
        self._stop_all()
        with self._enc_lock:
            for enc in self._encoders.values():
                enc.target_deg = None
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
        self._stop_all()
        with self._enc_lock:
            for enc in self._encoders.values():
                enc.target_deg = None
        self._modo_calib = True
        res.success = True
        res.message = 'Calibracion iniciada'
        self._publicar_status('Calibracion iniciada')
        return res

    def _srv_calib_confirm(self, req, res):
        self._stop_all()
        with self._enc_lock:
            for enc in self._encoders.values():
                enc.set_zero()
                enc.target_deg = None
        self._modo_calib = False
        res.success = True
        res.message = 'Home = 0 confirmado'
        self._publicar_status('Home = 0 confirmado')
        return res

    # ------------------------------------------------------------------
    #  Servicio jog
    # ------------------------------------------------------------------

    def _srv_jog(self, req, res):
        if self._emergencia:
            res.success = False
            res.mensaje = 'Paro de emergencia activo'
            return res
        if not self._conectado:
            res.success = False
            res.mensaje = 'Driver no conectado'
            return res
        sid = req.id
        with self._enc_lock:
            if sid not in self._encoders:
                res.success = False
                res.mensaje = f'Servo ID {sid} no registrado'
                return res
            if sid not in self._activos:
                res.success = False
                res.mensaje = f'Servo ID {sid} sin respuesta en el bus'
                return res
            enc = self._encoders[sid]
            enc.target_deg = float(req.target_deg) % 360.0
            if req.vel_pct > 0:
                enc.vel_pct = float(req.vel_pct)
        res.success = True
        res.mensaje = f'Jog ID {sid} → {req.target_deg:.1f}°'
        return res

    # ------------------------------------------------------------------
    #  Servicio rescue_pulse
    # ------------------------------------------------------------------

    def _srv_rescue_pulse(self, req, res):
        if not self._conectado:
            res.success = False
            res.mensaje = 'Driver no conectado'
            return res
        sid = req.id
        with self._enc_lock:
            if sid not in self._encoders:
                res.success = False
                res.mensaje = f'Servo ID {sid} no registrado'
                return res
            if sid not in self._activos:
                res.success = False
                res.mensaje = f'Servo ID {sid} sin respuesta en el bus'
                return res
            self._encoders[sid].target_deg = None

        d   = 1 if req.target_deg >= 0 else -1
        vel = max(5.0, min(30.0, float(req.vel_pct) if req.vel_pct > 0 else 15.0))
        self._write2(sid, ADDR_MOVING_SPEED, self._vel_raw(d * vel))
        time.sleep(0.25)
        self._write2(sid, ADDR_MOVING_SPEED, 0)

        res.success = True
        res.mensaje = f'Pulso rescate ID {sid} OK'
        return res

    # ------------------------------------------------------------------
    #  Servicio register_servo
    # ------------------------------------------------------------------

    def _srv_register_servo(self, req, res):
        sid = req.id
        with self._enc_lock:
            if sid in self._encoders:
                res.success = False
                res.mensaje = f'ID {sid} ya registrado como "{self._nombres.get(sid)}"'
                return res
            enc = WheelEncoder(reduccion=float(req.reduccion))
            enc.deadband_deg  = float(req.deadband_deg)
            enc.invertir_giro = bool(req.invertir_giro)
            self._encoders[sid] = enc
            self._nombres[sid]  = req.nombre

        if self._conectado:
            with self._lock:
                _, comm, _ = self._ph.ping(self._port, sid)
            if comm != COMM_SUCCESS:
                with self._enc_lock:
                    del self._encoders[sid]
                    del self._nombres[sid]
                res.success = False
                res.mensaje = f'ID {sid} no responde en el bus'
                return res
            self._write2(sid, ADDR_CW_LIMIT,  0)
            self._write2(sid, ADDR_CCW_LIMIT, 0)
            self._write1(sid, ADDR_TORQUE_EN, 1)
            with self._enc_lock:
                self._activos.add(sid)
                enc.reset_full()

        res.success = True
        res.mensaje = f'Servo "{req.nombre}" (ID {sid}) registrado'
        return res

    # ------------------------------------------------------------------
    #  Servicio servo_status — lee alarmas/temperatura/torque por servo
    # ------------------------------------------------------------------

    def _srv_status(self, req, res):
        with self._enc_lock:
            items = [(sid, self._nombres.get(sid, str(sid)))
                     for sid in self._encoders]

        for sid, nombre in items:
            responde, torque_on, err_byte, temp, volt = False, False, 0, 0, 0.0
            if self._conectado and self._port is not None:
                try:
                    with self._lock:
                        ten, comm,   err   = self._ph.read1ByteTxRx(
                            self._port, sid, ADDR_TORQUE_EN)
                        t,   comm_t, _     = self._ph.read1ByteTxRx(
                            self._port, sid, ADDR_PRESENT_TEMP)
                        v,   comm_v, _     = self._ph.read1ByteTxRx(
                            self._port, sid, ADDR_PRESENT_VOLT)
                except Exception:
                    comm = -1
                if comm == COMM_SUCCESS:
                    responde  = True
                    torque_on = (ten == 1)
                    err_byte  = int(err)
                    if comm_t == COMM_SUCCESS: temp = int(t)
                    if comm_v == COMM_SUCCESS: volt = v / 10.0

            res.ids.append(sid)
            res.nombres.append(nombre)
            res.responde.append(responde)
            res.torque_on.append(torque_on)
            res.error_byte.append(err_byte)
            res.alerta.append(_describir_error(err_byte) if responde
                              else 'sin respuesta')
            res.temperatura.append(temp)
            res.voltaje.append(float(volt))

        res.success = True
        res.mensaje = f'{len(items)} servos'
        return res

    # ------------------------------------------------------------------
    #  Servicio reset_alerts — rearma torque tras una alarma de shutdown
    # ------------------------------------------------------------------

    def _srv_reset_alerts(self, req, res):
        if not self._conectado or self._port is None:
            res.success = False
            res.message = 'Driver no conectado'
            return res
        with self._enc_lock:
            ids = list(self._activos)
        rearmados = []
        for sid in ids:
            # Reasegura modo rueda y rehabilita el torque (recuperacion tras
            # un apagado por alarma de sobrecarga/temperatura/voltaje).
            self._write2(sid, ADDR_CW_LIMIT,  0)
            self._write2(sid, ADDR_CCW_LIMIT, 0)
            self._write1(sid, ADDR_TORQUE_EN, 1)
            rearmados.append(self._nombres.get(sid, str(sid)))
        res.success = True
        res.message = ('Torque rearmado: ' + ', '.join(rearmados)
                       if rearmados else 'No hay servos activos')
        self._publicar_status(res.message)
        return res

    # ------------------------------------------------------------------
    #  Callback comandos de movimiento
    # ------------------------------------------------------------------

    def _cb_joint_cmd(self, msg: JointState):
        if self._emergencia or not self._conectado or self._modo_calib:
            return
        with self._enc_lock:
            for i, (name, pos) in enumerate(zip(msg.name, msg.position)):
                vel = msg.velocity[i] if i < len(msg.velocity) else None
                for sid, nombre in self._nombres.items():
                    if (nombre == name and sid in self._encoders
                            and sid in self._activos):
                        self._encoders[sid].target_deg = np.degrees(pos) % 360.0
                        if vel is not None and vel > 0:
                            self._encoders[sid].vel_pct = float(vel)

    # ------------------------------------------------------------------
    #  Conexion / desconexion
    # ------------------------------------------------------------------

    def _conectar(self):
        self._port = PortHandler(self._port_name)
        if not self._port.openPort():
            return False, f'No se pudo abrir {self._port_name}'
        if not self._port.setBaudRate(self._baudrate):
            self._port.closePort()
            return False, f'Baudrate {self._baudrate} fallo'

        with self._enc_lock:
            ids = list(self._encoders.keys())

        # Conexion tolerante: se trabaja con los servos que respondan
        activos, faltantes = [], []
        for sid in ids:
            with self._lock:
                _, comm, _ = self._ph.ping(self._port, sid)
            if comm == COMM_SUCCESS:
                activos.append(sid)
            else:
                faltantes.append(sid)
                self.get_logger().warn(
                    f'Servo ID {sid} ({self._nombres.get(sid)}) no responde; '
                    'queda inactivo')

        if not activos:
            self._port.closePort()
            return False, 'Ningun servo EX-106 respondio en el bus'

        for sid in activos:
            self._write2(sid, ADDR_CW_LIMIT,  0)
            self._write2(sid, ADDR_CCW_LIMIT, 0)
            self._write1(sid, ADDR_TORQUE_EN, 1)

        with self._enc_lock:
            self._activos = set(activos)
            for sid in activos:
                self._encoders[sid].reset_full()

        self._conectado  = True
        self._emergencia = False
        self._running    = True
        self._ctrl_thread = threading.Thread(
            target=self._bucle_control, daemon=True, name='ex106-ctrl')
        self._ctrl_thread.start()

        nombres_on = [self._nombres.get(s, str(s)) for s in activos]
        msg = f'EX-106 OK ({self._port_name}): {", ".join(nombres_on)}'
        if faltantes:
            nombres_off = [self._nombres.get(s, str(s)) for s in faltantes]
            msg += f' | sin respuesta: {", ".join(nombres_off)}'
        return True, msg

    def _desconectar(self):
        self._running = False
        if self._ctrl_thread and self._ctrl_thread.is_alive():
            self._ctrl_thread.join(timeout=2.0)
        if self._conectado:
            self._stop_all()
        if self._port:
            self._port.closePort()
        with self._enc_lock:
            self._activos = set()
        self._conectado = False

    # ------------------------------------------------------------------
    #  Escrituras al bus
    # ------------------------------------------------------------------

    def _write1(self, sid, addr, val):
        try:
            with self._lock:
                self._ph.write1ByteTxRx(self._port, sid, addr, val)
        except Exception as e:
            self.get_logger().warn(f'Escritura al bus fallo (ID {sid}): {e}',
                                   throttle_duration_sec=5.0)

    def _write2(self, sid, addr, val):
        try:
            with self._lock:
                self._ph.write2ByteTxRx(self._port, sid, addr, val)
        except Exception as e:
            self.get_logger().warn(f'Escritura al bus fallo (ID {sid}): {e}',
                                   throttle_duration_sec=5.0)

    def _vel_raw(self, pct: float) -> int:
        cruda = int(abs(pct) / 100.0 * 1023)
        if pct == 0:  return 0
        if pct > 0:   return max(1, cruda)
        return 1024 + max(1, cruda)

    def _stop_all(self):
        if not self._port:
            return
        with self._enc_lock:
            ids = list(self._activos)
        for sid in ids:
            self._write2(sid, ADDR_MOVING_SPEED, 0)

    # ------------------------------------------------------------------
    #  Loop de control (~66 Hz)
    # ------------------------------------------------------------------

    def _bucle_control(self):
        rate_s = 1.0 / self.get_parameter('loop_rate_hz').value
        while self._running:
            if not self._conectado or self._emergencia:
                time.sleep(0.05)
                continue

            with self._enc_lock:
                items = [(sid, enc) for sid, enc in self._encoders.items()
                         if sid in self._activos]

            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()

            for sid, enc in items:
                try:
                    with self._lock:
                        pos, res, err = self._ph.read2ByteTxRx(
                            self._port, sid, ADDR_PRESENT_POS)
                except Exception as e:
                    # Puerto perdido (ej. USB desconectado): no matar el nodo
                    self.get_logger().error(f'Puerto serie perdido: {e}')
                    self._conectado = False
                    self._running   = False
                    self._publicar_status('Puerto serie perdido')
                    return
                if res != COMM_SUCCESS or err != 0:
                    continue

                enc.update(pos)
                js.name.append(self._nombres.get(sid, str(sid)))
                js.position.append(np.radians(enc.angulo_salida))

                if enc.target_deg is None:
                    continue

                error = WheelEncoder.error_circular(enc.target_deg, enc.angulo_salida)

                if enc.control_estancado(abs(error), time.monotonic()):
                    # Tracking perdido: detener antes de que gire sin control
                    self._write2(sid, ADDR_MOVING_SPEED, 0)
                    enc.target_deg = None
                    nombre = self._nombres.get(sid, str(sid))
                    self.get_logger().error(
                        f'{nombre} (ID {sid}): sin progreso hacia el objetivo, '
                        'movimiento abortado por seguridad')
                    self._publicar_status(
                        f'{nombre}: movimiento abortado (tracking perdido)')
                    continue

                if abs(error) < enc.deadband_deg:
                    self._write2(sid, ADDR_MOVING_SPEED, 0)
                    enc.target_deg = None
                else:
                    d  = 1 if error > 0 else -1
                    if enc.invertir_giro: d *= -1
                    ae = abs(error)
                    # Minimos altos: con reduccion alta, por debajo de ~9%
                    # el servo no vence la friccion y se atasca cerca del objetivo
                    if   ae < 1:  vel = max(9,  int(enc.vel_pct * 0.12))
                    elif ae < 3:  vel = max(11, int(enc.vel_pct * 0.22))
                    elif ae < 8:  vel = max(14, int(enc.vel_pct * 0.42))
                    elif ae < 20: vel = max(16, int(enc.vel_pct * 0.65))
                    else:         vel = enc.vel_pct
                    self._write2(sid, ADDR_MOVING_SPEED, self._vel_raw(d * vel))

            if js.name:
                self._pub_joint_states.publish(js)

            time.sleep(rate_s)

    # ------------------------------------------------------------------

    def destroy_node(self):
        self._desconectar()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EX106DriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
