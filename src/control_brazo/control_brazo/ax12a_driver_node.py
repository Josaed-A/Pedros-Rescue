"""
ax12a_driver_node.py
====================
Nodo ROS2 — driver para servos Dynamixel AX-12A conectados via U2D2.

Responsabilidades:
  - Manejar el puerto serie U2D2 (un solo puerto para todos los AX-12A del robot)
  - Registrar servos dinamicamente via servicio /ax12a/register_servo
  - Loop de lectura/control por servo (WheelEncoder)
  - Publicar posicion actual en /joint_states
  - Suscribir comandos de movimiento en /ax12a/joint_cmd
  - Servicios: connect, disconnect, emergency_stop, resume,
               calibrate_start, calibrate_confirm, jog, rescue_pulse,
               register_servo

Topics publicados:
  /joint_states           (sensor_msgs/JointState)
  /ax12a/status           (control_brazo/ArmStatus)

Topics suscritos:
  /ax12a/joint_cmd        (sensor_msgs/JointState)
    name[]     — nombres de joints
    position[] — angulos en radianes
    velocity[] — vel_pct por joint (opcional, si vacio usa el default del encoder)

Servicios:
  /ax12a/connect           (std_srvs/Trigger)
  /ax12a/disconnect        (std_srvs/Trigger)
  /ax12a/emergency_stop    (std_srvs/Trigger)
  /ax12a/resume            (std_srvs/Trigger)
  /ax12a/calibrate_start   (std_srvs/Trigger)
  /ax12a/calibrate_confirm (std_srvs/Trigger)
  /ax12a/jog               (control_brazo/ServoCommand)
  /ax12a/rescue_pulse      (control_brazo/ServoCommand)
  /ax12a/register_servo    (control_brazo/RegisterServo)

Notas de diseno:
  - _cb_joint_cmd esta bloqueado en modo calibracion — solo jog puede
    mover servos durante la calibracion.
  - El loop de control ejecuta target_deg independientemente de modo_calib,
    lo que permite que los jogs se ejecuten correctamente.
"""

import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger

from dynamixel_sdk import PortHandler, PacketHandler, COMM_SUCCESS

from control_brazo.servo_params import leer_servos_params, leer_wheels_params
from control_brazo.wheel_encoder import WheelEncoder
from control_brazo.msg import ArmStatus
from control_brazo.srv import ServoCommand, RegisterServo, ServoStatus

# Registros Dynamixel protocolo 1.0
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


class AX12ADriverNode(Node):

    def __init__(self):
        super().__init__('ax12a_driver')

        self.declare_parameter('port',         '/dev/ttyUSB0')
        self.declare_parameter('baudrate',     1_000_000)
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

        # ── Patas (modo rueda, giro continuo) ──────────────────────────
        self._leg_ids:     set[int]        = set()
        self._leg_cmd:     dict[int, int]  = {}    # direccion deseada -1/0/+1
        self._leg_written: dict[int, int]  = {}    # ultima direccion escrita al bus
        self._leg_last_cmd = 0.0                   # monotonic del ultimo /legs/cmd

        self._cargar_servos_yaml()
        self._cargar_patas_yaml()

        self._pub_joint_states = self.create_publisher(
            JointState, '/joint_states', 10)
        self._pub_status = self.create_publisher(
            ArmStatus, '/ax12a/status', 10)
        # Estado de las patas: position[] = grados de salida ACUMULADOS (rad)
        self._pub_legs_state = self.create_publisher(
            JointState, '/legs/state', 10)

        self.create_subscription(
            JointState, '/ax12a/joint_cmd', self._cb_joint_cmd, 10)
        # Comando de patas: velocity[] = direccion (signo); 0 = detener
        self.create_subscription(
            JointState, '/legs/cmd', self._cb_legs_cmd, 10)

        self._crear_servicios()
        self._ctrl_thread = None

        self.get_logger().info(
            f'AX12A driver listo. Puerto: {self._port_name} | '
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

    def _cargar_patas_yaml(self):
        """Carga las patas (modo rueda) en los mismos diccionarios de encoders,
        marcando sus IDs en self._leg_ids para tratarlas distinto en el lazo."""
        self.declare_parameter('wheel_speed_pct', 40.0)
        self.declare_parameter('wheel_cmd_timeout_s', 0.4)
        self._wheel_speed_pct = float(self.get_parameter('wheel_speed_pct').value)
        self._wheel_timeout   = float(self.get_parameter('wheel_cmd_timeout_s').value)
        try:
            for w in leer_wheels_params(self):
                enc = WheelEncoder(reduccion=w['reduccion'])
                enc.invertir_giro = w['invertir_giro']
                with self._enc_lock:
                    self._encoders[w['id']]    = enc
                    self._nombres[w['id']]     = w['nombre']
                    self._leg_ids.add(w['id'])
                    self._leg_cmd[w['id']]     = 0
                    self._leg_written[w['id']] = None
        except Exception as e:
            self.get_logger().warn(f'No se cargaron patas del YAML: {e}')

    # ------------------------------------------------------------------
    #  Patas (modo rueda): comando, calibracion, torque
    # ------------------------------------------------------------------

    def _cb_legs_cmd(self, msg: JointState):
        """name[] = nombres de pata, velocity[] = direccion (signo). 0 = detener."""
        name_to_id = {self._nombres[i]: i for i in self._leg_ids}
        for k, nombre in enumerate(msg.name):
            sid = name_to_id.get(nombre)
            if sid is None:
                continue
            v = float(msg.velocity[k]) if k < len(msg.velocity) else 0.0
            self._leg_cmd[sid] = 1 if v > 0.1 else (-1 if v < -0.1 else 0)
        self._leg_last_cmd = time.monotonic()

    def _srv_legs_calibrate(self, req, res):
        """Pone la posicion actual de todas las patas como 0° (acumulado)."""
        with self._enc_lock:
            for sid in self._leg_ids:
                self._encoders[sid].set_zero()
        res.success = True
        res.message = 'Patas: posicion actual = 0°'
        return res

    def _srv_legs_enable_torque(self, req, res):
        """Rehabilita el torque de las patas (tras una alarma de sobrecarga)."""
        if not self._conectado:
            res.success = False
            res.message = 'Driver no conectado'
            return res
        n = 0
        for sid in list(self._leg_ids):
            if sid in self._activos:
                self._write1(sid, ADDR_TORQUE_EN, 1)
                n += 1
        res.success = True
        res.message = f'Torque rehabilitado en {n} pata(s)'
        return res

    def _aplicar_pata(self, sid: int, enc: WheelEncoder, now_mono: float):
        """Aplica la velocidad de giro de una pata segun su direccion deseada.

        Velocidad FIJA (wheel_speed_pct); el signo decide la direccion. Si no
        llegan comandos hace wheel_cmd_timeout_s, la pata se detiene. Solo se
        escribe al bus cuando la direccion cambia (evita saturarlo)."""
        d = self._leg_cmd.get(sid, 0)
        if (now_mono - self._leg_last_cmd) > self._wheel_timeout:
            d = 0
        if d == self._leg_written.get(sid):
            return
        if d == 0:
            self._write2(sid, ADDR_MOVING_SPEED, 0)
        else:
            d_eff = -d if enc.invertir_giro else d
            self._write2(sid, ADDR_MOVING_SPEED,
                         self._vel_raw(d_eff * self._wheel_speed_pct))
        self._leg_written[sid] = d

    # ------------------------------------------------------------------
    #  Registro de servicios
    # ------------------------------------------------------------------

    def _crear_servicios(self):
        self.create_service(Trigger,       '/ax12a/connect',           self._srv_connect)
        self.create_service(Trigger,       '/ax12a/disconnect',        self._srv_disconnect)
        self.create_service(Trigger,       '/ax12a/emergency_stop',    self._srv_estop)
        self.create_service(Trigger,       '/ax12a/resume',            self._srv_resume)
        self.create_service(Trigger,       '/ax12a/calibrate_start',   self._srv_calib_start)
        self.create_service(Trigger,       '/ax12a/calibrate_confirm', self._srv_calib_confirm)
        self.create_service(ServoCommand,  '/ax12a/jog',               self._srv_jog)
        self.create_service(ServoCommand,  '/ax12a/rescue_pulse',      self._srv_rescue_pulse)
        self.create_service(RegisterServo, '/ax12a/register_servo',    self._srv_register_servo)
        self.create_service(ServoStatus,   '/ax12a/servo_status',      self._srv_status)
        self.create_service(Trigger,       '/ax12a/reset_alerts',      self._srv_reset_alerts)
        # ── Patas ──
        self.create_service(Trigger,       '/legs/calibrate',          self._srv_legs_calibrate)
        self.create_service(Trigger,       '/legs/enable_torque',      self._srv_legs_enable_torque)

    def _publicar_status(self, mensaje: str = ''):
        msg = ArmStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.conectado    = self._conectado
        msg.emergencia   = self._emergencia
        msg.modo_calib   = self._modo_calib
        msg.mensaje      = mensaje
        self._pub_status.publish(msg)

    # ------------------------------------------------------------------
    #  Servicios Trigger (connect / disconnect / estop / resume / calib)
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
    #  Servicio jog — mover un servo a angulo absoluto
    #  Funciona incluso en modo calibracion (es el proposito del jog).
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
    #  Servicio rescue_pulse — pulso de velocidad abierta
    #  Sirve para liberar un servo mecanicamente atascado.
    #  target_deg >= 0 → giro positivo; < 0 → giro negativo.
    #  vel_pct se acota a [5, 30] para evitar daños.
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
            self._encoders[sid].target_deg = None  # Pausa control de posicion

        d   = 1 if req.target_deg >= 0 else -1
        vel = max(5.0, min(30.0, float(req.vel_pct) if req.vel_pct > 0 else 15.0))
        self._write2(sid, ADDR_MOVING_SPEED, self._vel_raw(d * vel))
        time.sleep(0.25)
        self._write2(sid, ADDR_MOVING_SPEED, 0)

        res.success = True
        res.mensaje = f'Pulso rescate ID {sid} OK'
        return res

    # ------------------------------------------------------------------
    #  Servicio register_servo — agregar servo en runtime
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

        # Si ya esta conectado, verificar presencia y habilitar el servo
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
    #  Bloqueado en modo calibracion — durante calib solo se usa jog.
    #  Usa msg.velocity[i] como vel_pct por joint si esta presente.
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
            return False, 'Ningun servo AX-12A respondio en el bus'

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
            target=self._bucle_control, daemon=True, name='ax12a-ctrl')
        self._ctrl_thread.start()

        nombres_on  = [self._nombres.get(s, str(s)) for s in activos]
        msg = f'AX-12A OK ({self._port_name}): {", ".join(nombres_on)}'
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
    #
    #  Ejecuta target_deg si esta seteado, independientemente de modo_calib.
    #  Esto permite que los jogs de calibracion funcionen correctamente.
    #  Lo que NO ejecuta targets en calib es _cb_joint_cmd (bloqueado arriba).
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
            legs_js = JointState()
            legs_js.header.stamp = js.header.stamp
            now_mono = time.monotonic()

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

                # ── Patas: giro continuo a velocidad fija + grados acumulados ──
                if sid in self._leg_ids:
                    legs_js.name.append(self._nombres.get(sid, str(sid)))
                    legs_js.position.append(np.radians(enc.angulo_salida_acum))
                    self._aplicar_pata(sid, enc, now_mono)
                    continue

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
                    # Minimos altos: con reduccion 27:1, por debajo de ~9%
                    # el servo no vence la friccion y se atasca cerca del objetivo
                    if   ae < 1:  vel = max(9,  int(enc.vel_pct * 0.12))
                    elif ae < 3:  vel = max(11, int(enc.vel_pct * 0.22))
                    elif ae < 8:  vel = max(14, int(enc.vel_pct * 0.42))
                    elif ae < 20: vel = max(16, int(enc.vel_pct * 0.65))
                    else:         vel = enc.vel_pct
                    self._write2(sid, ADDR_MOVING_SPEED, self._vel_raw(d * vel))

            if js.name:
                self._pub_joint_states.publish(js)
            if legs_js.name:
                self._pub_legs_state.publish(legs_js)

            time.sleep(rate_s)

    # ------------------------------------------------------------------

    def destroy_node(self):
        self._desconectar()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = AX12ADriverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
