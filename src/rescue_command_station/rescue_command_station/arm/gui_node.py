"""
gui_node.py
===========
Nodo ROS2 — interfaz grafica de control del brazo 6-DOF.

No contiene logica de hardware ni de cinematica.
Se comunica con el resto del sistema exclusivamente via ROS.

Topics suscritos:
  /joint_states           (sensor_msgs/JointState)
  /end_effector_pose      (geometry_msgs/PoseStamped)
  /fk_points              (sensor_msgs/JointState)   — puntos 3D del brazo
  /sim/fk_points_preview  (sensor_msgs/JointState)   — preview IK
  /ax12a/status           (rescue_interfaces/ArmStatus)
  /ex106/status           (rescue_interfaces/ArmStatus)

Topics publicados:
  /ax12a/joint_cmd        (sensor_msgs/JointState)
  /ex106/joint_cmd        (sensor_msgs/JointState)
  /joint_states_preview   (sensor_msgs/JointState)

Servicios llamados:
  /ax12a|ex106/{connect,disconnect,emergency_stop,resume,
                calibrate_start,calibrate_confirm,jog,rescue_pulse}
  /compute_ik             (rescue_interfaces/ComputeIK)
"""

import math
import os
import signal
import threading
import time
import tkinter as tk
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState, Joy, CompressedImage
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger

from std_msgs.msg import Bool, Float64, String
from rescue_interfaces.msg import ArmStatus, CartesianState
from rescue_interfaces.srv import (ComputeIK, ComputeIKPose, ServoCommand, ServoStatus,
                                CartesianGoto, CartesianTrajectory)
from rescue_interfaces.msg import CartesianWaypoint

from rescue_command_station.control import config as cfg
from rescue_command_station.arm.kinematics import (
    rotx3 as _rotx3, roty3 as _roty3, rotz3 as _rotz3,
)
from rescue_command_station.vision.qr_detector import QrDetector
from rescue_command_station.vision.ros_image import compressed_msg_to_numpy
from rescue_command_station.vision.tk_image import bgr_frame_to_png_data

import customtkinter as ctk

import matplotlib
matplotlib.use('TkAgg')
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 — registra la proyeccion '3d'

ctk.set_appearance_mode('Dark')
ctk.set_default_color_theme('blue')

# Tamano FIJO de la imagen de la camara frontal (px). Fijo a proposito: si se
# dimensiona al widget, el Label crece con la imagen y viceversa, y el video
# "se actualiza de tamano" constantemente.
CAM_IMG_W = 520
CAM_IMG_H = 340

# Defaults usados si no se pasan parametros desde YAML
_DEFAULT_JOINT_ORDER = ['Base', 'Hombro', 'Codo', 'Munieca_P',
                        'Munieca_Y', 'Munieca_R']
_DEFAULT_JOINT_SERVO = {
    'Base':      ('ax', 16),
    'Hombro':    ('ex', 1),
    'Codo':      ('ax', 18),
    'Munieca_P': ('ax', 30),
    'Munieca_Y': ('ax', 4),
    'Munieca_R': ('ax', 5),
}

# Paleta alineada con el dashboard (rescue_command_station/nodes/dashboard_node.py)
COL = {
    'panel_bg':  '#0b0f14',   # bg del dashboard
    'surface':   '#111821',   # surface del dashboard
    'surface_high': '#17212d',
    'accent':    '#38bdf8',   # cyan
    'ok':        '#22c55e',   # green
    'warn':      '#f59e0b',   # amber
    'err':       '#ef4444',   # red
    'muted':     '#94a3b8',
    'text':      '#e7edf4',
    'real':      '#60a5fa',   # blue
    'real_dot':  '#ef4444',
    'prev':      '#f59e0b',
    'prev_dot':  '#f59e0b',
}

# Colores extra de la paleta del dashboard para el tema CTk
_BG          = '#0b0f14'
_SURFACE     = '#111821'
_SURFACE_HI  = '#17212d'
_SURFACE_SOFT = '#1c2733'
_SURFACE_BTN = '#1e2d3d'
_BORDER      = '#263544'
_TEXT        = '#e7edf4'
_MUTED       = '#94a3b8'
_CYAN        = '#38bdf8'


def _aplicar_tema_dashboard():
    """Reescribe el tema global de customtkinter para que TODOS los widgets
    (frames, botones, tabs, sliders, entries...) sigan la paleta oscura del
    dashboard. Cada color del tema es [claro, oscuro]; forzamos ambos al mismo
    valor oscuro para que el look sea identico al dashboard."""
    T = ctk.ThemeManager.theme

    def setc(widget, key, color):
        if widget in T and key in T[widget]:
            T[widget][key] = [color, color]

    # Contenedores
    setc('CTk', 'fg_color', _BG)
    setc('CTkToplevel', 'fg_color', _BG)
    setc('CTkFrame', 'fg_color', _SURFACE)
    setc('CTkFrame', 'top_fg_color', _SURFACE_HI)
    setc('CTkFrame', 'border_color', _BORDER)
    setc('CTkScrollableFrame', 'label_fg_color', _SURFACE_HI)

    # Botones (los que fijan fg_color propio —estop/resume/back— lo conservan)
    setc('CTkButton', 'fg_color', _SURFACE_BTN)
    setc('CTkButton', 'hover_color', _SURFACE_SOFT)
    setc('CTkButton', 'border_color', _BORDER)
    setc('CTkButton', 'text_color', _TEXT)

    # Texto
    setc('CTkLabel', 'text_color', _TEXT)

    # Tabs (submodos del brazo)
    setc('CTkTabview', 'fg_color', _SURFACE)
    setc('CTkTabview', 'segmented_button_fg_color', _SURFACE_HI)
    setc('CTkTabview', 'segmented_button_selected_color', _CYAN)
    setc('CTkTabview', 'segmented_button_selected_hover_color', _CYAN)
    setc('CTkTabview', 'segmented_button_unselected_color', _SURFACE_HI)
    setc('CTkTabview', 'segmented_button_unselected_hover_color', _SURFACE_SOFT)
    setc('CTkTabview', 'text_color', _TEXT)
    setc('CTkSegmentedButton', 'fg_color', _SURFACE_HI)
    setc('CTkSegmentedButton', 'selected_color', _CYAN)
    setc('CTkSegmentedButton', 'selected_hover_color', _CYAN)
    setc('CTkSegmentedButton', 'unselected_color', _SURFACE_HI)
    setc('CTkSegmentedButton', 'unselected_hover_color', _SURFACE_SOFT)
    setc('CTkSegmentedButton', 'text_color', _TEXT)

    # Controles
    setc('CTkSlider', 'fg_color', _SURFACE_SOFT)
    setc('CTkSlider', 'progress_color', _CYAN)
    setc('CTkSlider', 'button_color', _CYAN)
    setc('CTkSlider', 'button_hover_color', _TEXT)
    setc('CTkProgressBar', 'fg_color', _SURFACE_SOFT)
    setc('CTkProgressBar', 'progress_color', _CYAN)
    setc('CTkEntry', 'fg_color', _SURFACE_HI)
    setc('CTkEntry', 'border_color', _BORDER)
    setc('CTkEntry', 'text_color', _TEXT)
    setc('CTkOptionMenu', 'fg_color', _SURFACE_BTN)
    setc('CTkOptionMenu', 'button_color', _SURFACE_SOFT)
    setc('CTkOptionMenu', 'button_hover_color', _SURFACE_HI)
    setc('CTkOptionMenu', 'text_color', _TEXT)
    setc('CTkComboBox', 'fg_color', _SURFACE_HI)
    setc('CTkComboBox', 'border_color', _BORDER)
    setc('CTkComboBox', 'button_color', _SURFACE_SOFT)
    setc('CTkCheckBox', 'fg_color', _CYAN)
    setc('CTkCheckBox', 'text_color', _TEXT)
    setc('CTkSwitch', 'progress_color', _CYAN)


_aplicar_tema_dashboard()


def quaternion_to_rpy(q) -> tuple[float, float, float]:
    """geometry_msgs/Quaternion → (roll, pitch, yaw) en grados."""
    sinr = 2.0 * (q.w * q.x + q.y * q.z)
    cosr = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr, cosr)

    sinp = 2.0 * (q.w * q.y - q.z * q.x)
    pitch = (math.copysign(math.pi / 2, sinp)
             if abs(sinp) >= 1 else math.asin(sinp))

    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny, cosy)

    return math.degrees(roll), math.degrees(pitch), math.degrees(yaw)


def quaternion_to_matrix(q) -> np.ndarray:
    """Matriz R que transforma vectores del frame herramienta al frame base."""
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if n < 1e-9:
        return np.eye(3)
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1.0 - 2.0*(y*y + z*z), 2.0*(x*y - z*w),       2.0*(x*z + y*w)],
        [2.0*(x*y + z*w),       1.0 - 2.0*(x*x + z*z), 2.0*(y*z - x*w)],
        [2.0*(x*z - y*w),       2.0*(y*z + x*w),       1.0 - 2.0*(x*x + y*y)],
    ], dtype=float)


def _parse_points(msg: JointState) -> np.ndarray:
    """Decodifica position=[x0,y0,z0, x1,y1,z1,...] a array (N,3)."""
    flat = list(msg.position)
    n = len(flat) // 3
    return np.array(flat).reshape(n, 3)


def _future_result(future):
    """Resultado de un future de servicio, o None si fallo."""
    try:
        return future.result()
    except Exception:
        return None


# ─────────────────────────────────────────────────────────────
#  Nodo ROS2
# ─────────────────────────────────────────────────────────────

class GUINode(Node):

    def __init__(self):
        super().__init__('gui_control')

        self._joint_order, self._joint_servo = self._leer_config_joints()
        self.declare_parameter('reach', 0.74)
        self.reach = float(self.get_parameter('reach').value)

        self._q_actual    = np.zeros(len(self._joint_order))
        self._pose_actual = None
        self._joint_map   = {name: i for i, name in enumerate(self._joint_order)}
        self._pts_real    : np.ndarray | None = None
        self._pts_preview : np.ndarray | None = None
        self._pts_version = 0

        self._ax_status = {'conectado': False, 'emergencia': False,
                           'modo_calib': False, 'mensaje': ''}
        self._ex_status = {'conectado': False, 'emergencia': False,
                           'modo_calib': False, 'mensaje': ''}

        self._lock = threading.Lock()

        # ── Publishers ─────────────────────────────────────────────
        self._pub_ax_cmd = self.create_publisher(
            JointState, '/ax12a/joint_cmd', 10)
        self._pub_ex_cmd = self.create_publisher(
            JointState, '/ex106/joint_cmd', 10)
        self._pub_joint_preview = self.create_publisher(
            JointState, '/joint_states_preview', 10)

        # ── Subscribers ────────────────────────────────────────────
        self.create_subscription(
            JointState,  '/joint_states',          self._cb_joint_states, 10)
        self.create_subscription(
            PoseStamped, '/end_effector_pose',     self._cb_pose,         10)
        self.create_subscription(
            JointState,  '/fk_points',             self._cb_fk_points,    10)
        self.create_subscription(
            JointState,  '/sim/fk_points_preview', self._cb_fk_preview,   10)
        self.create_subscription(
            ArmStatus,      '/ax12a/status',        self._cb_ax_status,    10)
        self.create_subscription(
            ArmStatus,      '/ex106/status',        self._cb_ex_status,    10)
        self.create_subscription(
            CartesianState, '/cartesian/state',     self._cb_cart_state,   10)

        self._cartesian_state = {
            'in_progress': False, 'progress': 0.0,
            'pos_error': 0.0, 'within_path_tol': True,
            'waypoint_idx': 0, 'total_waypoints': 0,
        }

        self._pub_speed_scale = self.create_publisher(Float64, '/cartesian/speed_scale', 10)

        # ── Clientes de servicios ───────────────────────────────────
        self._cli = {
            'ax_connect'   : self.create_client(Trigger,      '/ax12a/connect'),
            'ax_disconnect': self.create_client(Trigger,      '/ax12a/disconnect'),
            'ax_estop'     : self.create_client(Trigger,      '/ax12a/emergency_stop'),
            'ax_resume'    : self.create_client(Trigger,      '/ax12a/resume'),
            'ax_cal_start' : self.create_client(Trigger,      '/ax12a/calibrate_start'),
            'ax_cal_ok'    : self.create_client(Trigger,      '/ax12a/calibrate_confirm'),
            'ex_connect'   : self.create_client(Trigger,      '/ex106/connect'),
            'ex_disconnect': self.create_client(Trigger,      '/ex106/disconnect'),
            'ex_estop'     : self.create_client(Trigger,      '/ex106/emergency_stop'),
            'ex_resume'    : self.create_client(Trigger,      '/ex106/resume'),
            'ex_cal_start' : self.create_client(Trigger,      '/ex106/calibrate_start'),
            'ex_cal_ok'    : self.create_client(Trigger,      '/ex106/calibrate_confirm'),
            'ax_jog'       : self.create_client(ServoCommand, '/ax12a/jog'),
            'ex_jog'       : self.create_client(ServoCommand, '/ex106/jog'),
            'ax_rescue'    : self.create_client(ServoCommand, '/ax12a/rescue_pulse'),
            'ex_rescue'    : self.create_client(ServoCommand, '/ex106/rescue_pulse'),
            'ax_status_srv': self.create_client(ServoStatus,  '/ax12a/servo_status'),
            'ex_status_srv': self.create_client(ServoStatus,  '/ex106/servo_status'),
            'ax_reset'     : self.create_client(Trigger,      '/ax12a/reset_alerts'),
            'ex_reset'     : self.create_client(Trigger,      '/ex106/reset_alerts'),
            'compute_ik'     : self.create_client(ComputeIK,      '/compute_ik'),
            'compute_ik_pose': self.create_client(ComputeIKPose,  '/compute_ik_pose'),
            'cartesian_goto'  : self.create_client(CartesianGoto,       '/cartesian/goto'),
            'cartesian_traj'  : self.create_client(CartesianTrajectory, '/cartesian/trajectory'),
        }

        # ── Integracion con el dashboard ───────────────────────────
        #   /arm_active: el dashboard dice si el brazo esta al frente (mostrar/ocultar)
        #   /joy: L1/R1 alternan submodos (tabs) cuando el brazo esta al frente
        #   /gui_switch_request: pide volver al dashboard (boton "volver")
        self._req_active = False        # ultimo /arm_active recibido
        self._submode_step = 0          # pasos L1/R1 pendientes de aplicar
        self._prev_buttons: list = []
        self._joy_axes: list = []       # ultimos ejes del mando (teleop cartesiano)
        self._bus_reset_flag = False    # el dashboard pidio reiniciar los buses
        self.create_subscription(Bool, '/arm_active', self._cb_arm_active, 10)
        self.create_subscription(Joy, '/joy', self._cb_joy, 10)
        self.create_subscription(Bool, '/bus_reset', self._cb_bus_reset, 10)
        self._pub_switch = self.create_publisher(Bool, '/gui_switch_request', 10)

        # ── Camara frontal (NO la Orbbec) + deteccion QR / senales ──
        self.declare_parameter('front_camera_topic', '/robot/camera/front/image_raw/compressed')
        front_topic = self.get_parameter('front_camera_topic').value
        self.qr_detector = QrDetector()
        self._cam_lock = threading.Lock()
        self._cam_frame = None
        self._cam_version = 0
        self._qr_text = ''
        self._last_detection = ''
        self._last_qr_scan = 0.0
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(CompressedImage, front_topic, self._cb_front_cam, sensor_qos)
        self.create_subscription(String, '/object_detections', self._cb_detections, 10)

        self.get_logger().info(f'Nodo GUI listo. Joints: {self._joint_order}')

    # ── Integracion dashboard: visibilidad, mando, camara ─────────

    def _cb_arm_active(self, msg):
        self._req_active = bool(msg.data)

    def _cb_bus_reset(self, msg):
        if msg.data:
            self._bus_reset_flag = True

    def _cb_joy(self, msg):
        """L1/R1 → alternar submodos (tabs). Ejes → teleop cartesiano (los lee
        la App). Solo con el brazo al frente."""
        if not self._req_active:
            self._prev_buttons = list(msg.buttons)
            self._joy_axes = []
            return
        self._joy_axes = list(msg.axes)
        btn = msg.buttons

        def pressed(idx):
            return (idx < len(btn) and btn[idx] and
                    (idx >= len(self._prev_buttons) or not self._prev_buttons[idx]))

        if pressed(cfg.BUTTON_R1):
            self._submode_step += 1
        if pressed(cfg.BUTTON_L1):
            self._submode_step -= 1
        self._prev_buttons = list(btn)

    def pedir_dashboard(self):
        """Publica la peticion de volver al dashboard (boton 'volver')."""
        m = Bool(); m.data = True
        self._pub_switch.publish(m)

    def _cb_front_cam(self, msg):
        try:
            frame = compressed_msg_to_numpy(msg)
            now = time.time()
            if now - self._last_qr_scan >= 0.25:
                self._last_qr_scan = now
                frame, qr = self.qr_detector.detect_and_annotate(frame)
                if qr:
                    self._qr_text = qr
            with self._cam_lock:
                self._cam_frame = frame
                self._cam_version += 1
        except Exception as exc:
            self.get_logger().warn(f'front_camera: {exc}', throttle_duration_sec=5.0)

    def _cb_detections(self, msg):
        if msg.data:
            self._last_detection = msg.data

    # ── Lectura de configuracion desde YAML ───────────────────────

    def _leer_config_joints(self):
        """Lee joint_order y joint_drivers.<nombre>.* desde params ROS."""
        self.declare_parameter('joint_order', _DEFAULT_JOINT_ORDER)
        joint_order = list(self.get_parameter('joint_order').value)

        joint_servo: dict[str, tuple[str, int]] = {}
        for name in joint_order:
            try:
                self.declare_parameter(f'joint_drivers.{name}.driver', '')
                self.declare_parameter(f'joint_drivers.{name}.id', 0)
                driver = self.get_parameter(f'joint_drivers.{name}.driver').value
                sid    = int(self.get_parameter(f'joint_drivers.{name}.id').value)
                if driver in ('ax', 'ex'):
                    joint_servo[name] = (driver, sid)
            except Exception as e:
                self.get_logger().warn(f'joint_drivers.{name}: {e}')

        for name, val in _DEFAULT_JOINT_SERVO.items():
            if name in joint_order and name not in joint_servo:
                joint_servo[name] = val

        return joint_order, joint_servo

    # ── Propiedades de configuracion ──────────────────────────────

    @property
    def joint_order(self) -> list[str]:
        return self._joint_order

    @property
    def joint_servo(self) -> dict[str, tuple[str, int]]:
        return self._joint_servo

    # ── Callbacks topics ───────────────────────────────────────────

    def _cb_joint_states(self, msg: JointState):
        with self._lock:
            for name, pos in zip(msg.name, msg.position):
                if name in self._joint_map:
                    self._q_actual[self._joint_map[name]] = pos

    def _cb_pose(self, msg: PoseStamped):
        with self._lock:
            self._pose_actual = msg

    def _cb_fk_points(self, msg: JointState):
        with self._lock:
            self._pts_real = _parse_points(msg)
            self._pts_version += 1

    def _cb_fk_preview(self, msg: JointState):
        with self._lock:
            self._pts_preview = _parse_points(msg)
            self._pts_version += 1

    def _cb_ax_status(self, msg: ArmStatus):
        with self._lock:
            self._ax_status = {
                'conectado':  msg.conectado,
                'emergencia': msg.emergencia,
                'modo_calib': msg.modo_calib,
                'mensaje':    msg.mensaje,
            }

    def _cb_ex_status(self, msg: ArmStatus):
        with self._lock:
            self._ex_status = {
                'conectado':  msg.conectado,
                'emergencia': msg.emergencia,
                'modo_calib': msg.modo_calib,
                'mensaje':    msg.mensaje,
            }

    def _cb_cart_state(self, msg: CartesianState):
        with self._lock:
            self._cartesian_state = {
                'in_progress':     msg.in_progress,
                'progress':        msg.progress,
                'pos_error':       msg.pos_error,
                'within_path_tol': msg.within_path_tol,
                'waypoint_idx':    msg.waypoint_idx,
                'total_waypoints': msg.total_waypoints,
            }

    # ── Estado ────────────────────────────────────────────────────

    @property
    def cartesian_state(self) -> dict:
        with self._lock:
            return dict(self._cartesian_state)

    def publicar_speed_scale(self, pct: float):
        msg = Float64()
        msg.data = float(np.clip(pct, 0.0, 100.0))
        self._pub_speed_scale.publish(msg)

    @property
    def ax_status(self) -> dict:
        with self._lock:
            return dict(self._ax_status)

    @property
    def ex_status(self) -> dict:
        with self._lock:
            return dict(self._ex_status)

    @property
    def conectado(self) -> bool:
        """Al menos un bus conectado — permite trabajar con hardware parcial."""
        with self._lock:
            return self._ax_status['conectado'] or self._ex_status['conectado']

    @property
    def emergencia(self) -> bool:
        with self._lock:
            return self._ax_status['emergencia'] or self._ex_status['emergencia']

    @property
    def modo_calib(self) -> bool:
        with self._lock:
            return self._ax_status['modo_calib'] or self._ex_status['modo_calib']

    @property
    def q_actual(self) -> np.ndarray:
        with self._lock:
            return self._q_actual.copy()

    @property
    def pose_actual(self):
        with self._lock:
            return self._pose_actual

    def puntos_brazo(self):
        with self._lock:
            return self._pts_real, self._pts_preview, self._pts_version

    # ── Publicar comandos de movimiento ───────────────────────────

    def publicar_joint_cmd(self, q_rad: np.ndarray, vel_pct: float = 30.0):
        """Separa q[] por driver segun la config de YAML y envia a cada bus."""
        ax_names = [n for n in self._joint_order
                    if self._joint_servo.get(n, ('ax',))[0] == 'ax']
        ex_names = [n for n in self._joint_order
                    if self._joint_servo.get(n, ('ax',))[0] == 'ex']

        for driver, names, pub in (('ax', ax_names, self._pub_ax_cmd),
                                   ('ex', ex_names, self._pub_ex_cmd)):
            if not names:
                continue
            idx = [self._joint_order.index(n) for n in names]
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name     = names
            msg.position = [float(q_rad[i]) for i in idx]
            msg.velocity = [float(vel_pct)] * len(names)
            pub.publish(msg)
            if driver == 'ex':
                self._jog_ex_fallback(names, idx, q_rad, vel_pct)

    def _jog_ex_fallback(self, names: list[str], idx: list[int],
                         q_rad: np.ndarray, vel_pct: float):
        """Respaldo para EX-106+: usa el servicio por ID si esta disponible."""
        cli = self._cli.get('ex_jog')
        if cli is None or not cli.service_is_ready():
            return

        for name, i in zip(names, idx):
            driver, sid = self._joint_servo.get(name, ('', 0))
            if driver != 'ex' or sid <= 0:
                continue
            req = ServoCommand.Request()
            req.id = int(sid)
            req.target_deg = float(np.degrees(q_rad[i]))
            req.vel_pct = float(vel_pct)

            def _done(f, joint=name):
                res = _future_result(f)
                if res is None:
                    self.get_logger().warn(
                        f'Fallback EX jog para {joint}: llamada fallida')
                elif not res.success:
                    self.get_logger().warn(
                        f'Fallback EX jog para {joint}: {res.mensaje}')

            cli.call_async(req).add_done_callback(_done)

    def publicar_preview(self, q_rad):
        """Publica q_rad para que cinematica_node calcule el preview."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name     = list(self._joint_order)
        msg.position = [float(q) for q in q_rad]
        self._pub_joint_preview.publish(msg)

    # ── Llamadas a servicios Trigger ──────────────────────────────

    def _call_trigger(self, key: str):
        cli = self._cli.get(key)
        if cli is None or not cli.service_is_ready():
            self.get_logger().warn(f'Servicio {key} no disponible')
            return

        def _done(f):
            res = _future_result(f)
            if res is not None:
                self.get_logger().info(f'{key}: {res.message}')
            else:
                self.get_logger().warn(f'{key}: la llamada fallo')

        cli.call_async(Trigger.Request()).add_done_callback(_done)

    def conectar(self):
        self._call_trigger('ax_connect')
        self._call_trigger('ex_connect')

    def desconectar(self):
        self._call_trigger('ax_disconnect')
        self._call_trigger('ex_disconnect')

    def emergencia_stop(self):
        self._call_trigger('ax_estop')
        self._call_trigger('ex_estop')

    def reanudar(self):
        self._call_trigger('ax_resume')
        self._call_trigger('ex_resume')

    def calib_start(self):
        self._call_trigger('ax_cal_start')
        self._call_trigger('ex_cal_start')

    def calib_confirm(self):
        self._call_trigger('ax_cal_ok')
        self._call_trigger('ex_cal_ok')

    # ── ServoCommand (jog / rescue) ───────────────────────────────

    def _call_servo_cmd(self, key: str, sid: int, target_deg: float,
                        vel_pct: float, on_result):
        cli = self._cli.get(key)
        if cli is None or not cli.service_is_ready():
            self.get_logger().warn(f'Servicio {key} no disponible')
            if on_result:
                on_result(False, f'{key} no disponible')
            return
        req = ServoCommand.Request()
        req.id         = sid
        req.target_deg = float(target_deg)
        req.vel_pct    = float(vel_pct)

        def _done(f):
            res = _future_result(f)
            if on_result:
                if res is not None:
                    on_result(res.success, res.mensaje)
                else:
                    on_result(False, 'la llamada al servicio fallo')

        cli.call_async(req).add_done_callback(_done)

    def jog(self, joint_name: str, target_deg: float, vel_pct: float = 10.0,
            on_result=None):
        driver, sid = self._joint_servo.get(joint_name, ('ax', 0))
        self._call_servo_cmd(f'{driver}_jog', sid, target_deg, vel_pct, on_result)

    def rescue_pulse(self, joint_name: str, direction: int, vel_pct: float = 15.0,
                     on_result=None):
        driver, sid = self._joint_servo.get(joint_name, ('ax', 0))
        self._call_servo_cmd(f'{driver}_rescue', sid, direction, vel_pct, on_result)

    # ── Estado de servos / alarmas ────────────────────────────────

    def pedir_servo_status(self, on_result):
        """Pide estado a ambos drivers. Llama on_result(lista_dicts) por driver."""
        for key in ('ax_status_srv', 'ex_status_srv'):
            cli = self._cli.get(key)
            if cli is None or not cli.service_is_ready():
                continue

            def _done(f):
                res = _future_result(f)
                if res is None:
                    return
                filas = [{
                    'id':        res.ids[i],
                    'nombre':    res.nombres[i],
                    'responde':  res.responde[i],
                    'torque_on': res.torque_on[i],
                    'alerta':    res.alerta[i],
                    'temp':      res.temperatura[i],
                    'volt':      res.voltaje[i],
                } for i in range(len(res.ids))]
                on_result(filas)

            cli.call_async(ServoStatus.Request()).add_done_callback(_done)

    def reset_alerts(self):
        self._call_trigger('ax_reset')
        self._call_trigger('ex_reset')

    # ── IK asincrono ──────────────────────────────────────────────

    def pedir_ik(self, x, y, z, beta_deg, q5_deg, q6_deg, elbow, vel_pct,
                 on_result):
        cli = self._cli.get('compute_ik')
        if cli is None or not cli.service_is_ready():
            on_result(False, [], '/compute_ik no disponible')
            return
        req = ComputeIK.Request()
        req.x        = float(x)
        req.y        = float(y)
        req.z        = float(z)
        req.beta_deg = float(beta_deg)
        req.q5_deg   = float(q5_deg)
        req.q6_deg   = float(q6_deg)
        req.elbow    = elbow

        def _done(f):
            res = _future_result(f)
            if res is not None:
                on_result(res.success, list(res.q_rad), res.mensaje)
            else:
                on_result(False, [], 'la llamada a /compute_ik fallo')

        cli.call_async(req).add_done_callback(_done)

    def pedir_ik_pose(self, p, R, elbow, on_result):
        """IK desde una pose completa (R 3x3, p 3). Para teleoperacion cartesiana."""
        cli = self._cli.get('compute_ik_pose')
        if cli is None or not cli.service_is_ready():
            on_result(False, [], '/compute_ik_pose no disponible')
            return
        req = ComputeIKPose.Request()
        req.x     = float(p[0])
        req.y     = float(p[1])
        req.z     = float(p[2])
        req.r     = [float(v) for v in np.asarray(R).reshape(9)]
        req.elbow = elbow

        def _done(f):
            res = _future_result(f)
            if res is not None:
                on_result(res.success, list(res.q_rad), res.mensaje)
            else:
                on_result(False, [], 'la llamada a /compute_ik_pose fallo')

        cli.call_async(req).add_done_callback(_done)

    def pedir_cartesian_goto(self, p, R, n_steps: int, step_dt: float,
                              vel_pct: float, elbow: str, on_result):
        """Trayectoria cartesiana via cartesian_node (/cartesian/goto)."""
        cli = self._cli.get('cartesian_goto')
        if cli is None or not cli.service_is_ready():
            on_result(False, '/cartesian/goto no disponible')
            return
        req = CartesianGoto.Request()
        req.x       = float(p[0])
        req.y       = float(p[1])
        req.z       = float(p[2])
        req.r       = [float(v) for v in np.asarray(R).reshape(9)]
        req.elbow   = elbow
        req.n_steps = int(n_steps)
        req.step_dt = float(step_dt)
        req.vel_pct = float(vel_pct)

        def _done(f):
            res = _future_result(f)
            if res is not None:
                on_result(res.success, res.mensaje)
            else:
                on_result(False, 'la llamada a /cartesian/goto fallo')

        cli.call_async(req).add_done_callback(_done)

    def pedir_cartesian_trajectory(
        self, p0, R0, p1, R1,
        n_steps: int, step_dt: float,
        vel_pct: float, elbow: str,
        path_tol_pos: float, goal_tol_pos: float,
        on_result,
    ):
        """Trayectoria multi-waypoint via /cartesian/trajectory con tolerancias."""
        cli = self._cli.get('cartesian_traj')
        if cli is None or not cli.service_is_ready():
            on_result(False, '/cartesian/trajectory no disponible')
            return

        req = CartesianTrajectory.Request()
        req.elbow        = elbow
        req.vel_pct      = float(vel_pct)
        req.path_tol_pos = float(path_tol_pos)
        req.path_tol_rot = 0.0
        req.goal_tol_pos = float(goal_tol_pos)
        req.goal_tol_rot = 0.0

        # Waypoint 0: pose actual (inicio)
        wp0 = CartesianWaypoint()
        wp0.x, wp0.y, wp0.z = float(p0[0]), float(p0[1]), float(p0[2])
        wp0.r       = [float(v) for v in np.asarray(R0).reshape(9)]
        wp0.n_steps = 1
        wp0.duration = 0.0

        # Waypoint 1: pose objetivo
        wp1 = CartesianWaypoint()
        wp1.x, wp1.y, wp1.z = float(p1[0]), float(p1[1]), float(p1[2])
        wp1.r       = [float(v) for v in np.asarray(R1).reshape(9)]
        wp1.n_steps = int(n_steps)
        wp1.duration = float(step_dt * n_steps)

        req.waypoints = [wp0, wp1]

        def _done(f):
            res = _future_result(f)
            if res is not None:
                on_result(res.success, res.mensaje)
            else:
                on_result(False, 'la llamada a /cartesian/trajectory fallo')

        cli.call_async(req).add_done_callback(_done)


# ─────────────────────────────────────────────────────────────
#  Ventana principal
# ─────────────────────────────────────────────────────────────

class App(ctk.CTk):

    def __init__(self, node: GUINode):
        super().__init__()
        self._node = node

        # Resultados pendientes del thread ROS para aplicar en _loop_ui
        self._pending_ik_q      : list | None  = None
        self._pending_ik_msg    : tuple | None = None   # (texto, color)
        self._pending_vel       : float        = 30.0
        self._pending_jog_msg   : tuple | None = None
        self._pending_rescue_msg: tuple | None = None
        self._pending_status    : dict        = {}
        self._status_poll_count = 0
        self._cart_delta_buttons: list[ctk.CTkButton] = []
        self._drawn_pts_version = -1

        # Teleoperacion cartesiana: pose objetivo acumulada (R 3x3, p 3)
        self._tp_R              : np.ndarray | None = None
        self._tp_p              : np.ndarray | None = None
        self._stab_R            : np.ndarray | None = None   # R fija para estabilizacion
        self._pending_teleop_q  : list | None  = None
        self._pending_teleop_msg: tuple | None = None
        self._pending_teleop_commit: tuple | None = None
        self._joy_ik_inflight    = False   # 1 sola peticion IK por joystick a la vez
        self._joy_inflight_ticks = 0       # timeout de seguridad si la IK no responde
        self._bus_reset_ticks    = 0       # ticks restantes del banner de reinicio de buses

        # Jog manual por servo con el mando (pestana 'Jog')
        self._jog_inflight: set[int] = set()   # indices de joint con /..._jog en curso
        self._jog_gp_indicator_on: bool | None = None
        self._last_cart_in_prog  = False   # detecta flanco bajada para re-habilitar btn

        self.title('Brazo 6-DOF — Control ROS2')
        self.geometry('1380x820')
        self.minsize(1100, 700)
        self.resizable(True, True)
        self.configure(fg_color=COL['panel_bg'])

        # Arranca OCULTA: el dashboard la precarga y la muestra con la flecha
        # abajo (via /arm_active). Asi alternar es instantaneo.
        self.withdraw()
        self._is_shown = False

        self._build_ui()
        self.after(120, self._loop_ui)

    # ── Construccion de la UI ──────────────────────────────────────

    def _build_ui(self):
        self.grid_columnconfigure(0, weight=0, minsize=430)
        self.grid_columnconfigure(1, weight=1)
        self.grid_rowconfigure(0, weight=1)

        self._build_left_panel()
        self._build_right_panel()

    # ── Columna izquierda: estado + controles ─────────────────────

    def _build_left_panel(self):
        left = ctk.CTkFrame(self)
        left.grid(row=0, column=0, sticky='nsew', padx=(8, 4), pady=8)
        left.grid_rowconfigure(3, weight=1)
        left.grid_columnconfigure(0, weight=1)

        header = ctk.CTkFrame(left, fg_color='transparent')
        header.grid(row=0, column=0, sticky='ew', pady=(8, 2))
        header.grid_columnconfigure(1, weight=1)
        self.btn_back = ctk.CTkButton(
            header, text='⟵ Dashboard', width=120, height=30,
            fg_color='#5d6d7e', hover_color='#34495e', command=self._node.pedir_dashboard)
        self.btn_back.grid(row=0, column=0, padx=(8, 6))
        ctk.CTkLabel(header, text='Brazo 6-DOF',
                     font=('Roboto', 18, 'bold')).grid(row=0, column=1)

        # ── Estado por driver ─────────────────────────────────────
        fr_st = ctk.CTkFrame(left)
        fr_st.grid(row=1, column=0, sticky='ew', padx=8, pady=4)
        fr_st.grid_columnconfigure((0, 1), weight=1)

        self.lbl_ax = ctk.CTkLabel(fr_st, text='AX-12A: —', corner_radius=6,
                                   fg_color='#333', font=('Roboto', 12, 'bold'))
        self.lbl_ax.grid(row=0, column=0, sticky='ew', padx=4, pady=4)
        self.lbl_ex = ctk.CTkLabel(fr_st, text='EX-106: —', corner_radius=6,
                                   fg_color='#333', font=('Roboto', 12, 'bold'))
        self.lbl_ex.grid(row=0, column=1, sticky='ew', padx=4, pady=4)

        self.lbl_driver_msg = ctk.CTkLabel(fr_st, text='', text_color=COL['muted'],
                                           font=('Roboto', 11), wraplength=380)
        self.lbl_driver_msg.grid(row=1, column=0, columnspan=2, padx=4)

        self.lbl_global = ctk.CTkLabel(fr_st, text='', text_color=COL['err'],
                                       font=('Roboto', 13, 'bold'))
        self.lbl_global.grid(row=2, column=0, columnspan=2, pady=(0, 2))

        # ── Conexion / E-STOP ─────────────────────────────────────
        fr_btns = ctk.CTkFrame(left, fg_color='transparent')
        fr_btns.grid(row=2, column=0, sticky='ew', padx=8, pady=2)
        fr_btns.grid_columnconfigure((0, 1), weight=1)

        self.btn_conn = ctk.CTkButton(fr_btns, text='Conectar',
                                      command=self._toggle_conn)
        self.btn_conn.grid(row=0, column=0, sticky='ew', padx=2, pady=2)

        self.btn_resume = ctk.CTkButton(
            fr_btns, text='Reanudar', state='disabled',
            fg_color='#1e8449', hover_color='#145a32',
            command=self._node.reanudar)
        self.btn_resume.grid(row=0, column=1, sticky='ew', padx=2, pady=2)

        self.btn_estop = ctk.CTkButton(
            fr_btns, text='PARO DE EMERGENCIA',
            font=('Roboto', 14, 'bold'),
            fg_color='#c0392b', hover_color='#922b21',
            height=46, state='disabled',
            command=self._node.emergencia_stop)
        self.btn_estop.grid(row=1, column=0, columnspan=2,
                            sticky='ew', padx=2, pady=4)

        # Velocidad global
        vel_row = ctk.CTkFrame(fr_btns, fg_color='transparent')
        vel_row.grid(row=2, column=0, columnspan=2, sticky='ew', pady=2)
        vel_row.grid_columnconfigure(1, weight=1)
        ctk.CTkLabel(vel_row, text='Velocidad:').grid(row=0, column=0, padx=4)
        self.sld_vel = ctk.CTkSlider(vel_row, from_=1, to=100,
                                     command=self._on_vel_slider)
        self.sld_vel.set(100)
        self.sld_vel.grid(row=0, column=1, sticky='ew', padx=4)
        self.lbl_vel = ctk.CTkLabel(vel_row, text='100 %', width=46)
        self.lbl_vel.grid(row=0, column=2, padx=4)

        # ── Pestanas de control (submodos; se ciclan con L1/R1) ────
        self.tabs = ctk.CTkTabview(left)
        self.tabs.grid(row=3, column=0, sticky='nsew', padx=8, pady=4)
        self._tab_order = ['Mover', 'Teleop', 'Jog', 'IK', 'Calibrar',
                           'Rescate', 'Estado']
        for name in self._tab_order:
            self.tabs.add(name)

        self._build_tab_fk(self.tabs.tab('Mover'))
        self._build_tab_teleop(self.tabs.tab('Teleop'))
        self._build_tab_jog(self.tabs.tab('Jog'))
        self._build_tab_ik(self.tabs.tab('IK'))
        self._build_tab_calib(self.tabs.tab('Calibrar'))
        self._build_tab_rescue(self.tabs.tab('Rescate'))
        self._build_tab_estado(self.tabs.tab('Estado'))

    def _refresh_camera(self):
        """Renderiza la ultima imagen de la camara frontal + QR/senal."""
        n = self._node
        if n._cam_version != self._drawn_cam_version:
            with n._cam_lock:
                frame = None if n._cam_frame is None else n._cam_frame.copy()
                self._drawn_cam_version = n._cam_version
            if frame is not None:
                # Tamano FIJO: no leemos winfo del Label. Dimensionar la imagen
                # al tamano del widget realimenta el layout (el Label crece con
                # la imagen y la imagen con el Label) y el video "se actualiza de
                # tamano" constantemente. Con una caja fija queda estable.
                png = bgr_frame_to_png_data(frame, max_width=CAM_IMG_W,
                                            max_height=CAM_IMG_H)
                if png is not None:
                    self.cam_photo = tk.PhotoImage(data=png, format='png')
                    self.cam_label.configure(image=self.cam_photo, text='')
        self.lbl_qr.configure(text=f'QR: {n._qr_text or "—"}')
        self.lbl_det.configure(text=f'Senal: {n._last_detection or "—"}')

    # ── Tab FK: sliders por articulacion ──────────────────────────

    def _build_tab_fk(self, tab):
        joint_order = self._node.joint_order
        frame = ctk.CTkScrollableFrame(tab, fg_color='transparent')
        frame.pack(fill='both', expand=True)

        self._fk_sliders: list[ctk.CTkSlider] = []
        self._fk_val_lbls: list[ctk.CTkLabel] = []
        self._lbl_angs:   list[ctk.CTkLabel] = []

        for i, name in enumerate(joint_order):
            fr = ctk.CTkFrame(frame)
            fr.pack(fill='x', pady=3, padx=2)
            fr.grid_columnconfigure(1, weight=1)

            ctk.CTkLabel(fr, text=name, width=90, anchor='w',
                         font=('Roboto', 12, 'bold')).grid(
                             row=0, column=0, padx=6, pady=(4, 0), sticky='w')
            lbl_act = ctk.CTkLabel(fr, text='actual: 0.00°', anchor='e',
                                   text_color=COL['accent'], font=('Roboto', 11))
            lbl_act.grid(row=0, column=1, columnspan=2, padx=6,
                         pady=(4, 0), sticky='e')
            self._lbl_angs.append(lbl_act)

            sld = ctk.CTkSlider(fr, from_=-180, to=180, number_of_steps=720,
                                command=lambda v, i=i: self._on_fk_slider(i, v))
            sld.set(0)
            sld.grid(row=1, column=0, columnspan=2, sticky='ew',
                     padx=6, pady=(2, 6))
            self._fk_sliders.append(sld)

            lbl_val = ctk.CTkLabel(fr, text='0.0°', width=52)
            lbl_val.grid(row=1, column=2, padx=6)
            self._fk_val_lbls.append(lbl_val)

        btn_row = ctk.CTkFrame(frame, fg_color='transparent')
        btn_row.pack(fill='x', pady=6)
        btn_row.grid_columnconfigure((0, 1), weight=1)

        self.btn_fk_sync = ctk.CTkButton(
            btn_row, text='Copiar actual', fg_color='#5d6d7e',
            hover_color='#34495e', command=self._sync_sliders_to_actual)
        self.btn_fk_sync.grid(row=0, column=0, sticky='ew', padx=2)

        self.btn_fk = ctk.CTkButton(btn_row, text='MOVER (FK)',
                                    state='disabled', command=self._move_fk)
        self.btn_fk.grid(row=0, column=1, sticky='ew', padx=2)

        self.btn_home = ctk.CTkButton(
            frame, text='IR A HOME  (0° todos)',
            font=('Roboto', 13, 'bold'),
            fg_color='#1a5276', hover_color='#154360',
            height=40, state='disabled', command=self._home)
        self.btn_home.pack(fill='x', pady=4)

    # ── Tab Teleop: control cartesiano en ejes de la camara ───────

    def _build_tab_teleop(self, tab):
        frame = ctk.CTkScrollableFrame(tab, fg_color='transparent')
        frame.pack(fill='both', expand=True)

        ctk.CTkLabel(frame, text='Teleoperacion de camara (ejes locales)',
                     font=('Roboto', 13, 'bold')).pack(pady=(4, 2))

        # Indicador de joystick: se ilumina (verde) cuando esta moviendo el brazo.
        self.lbl_joy_state = ctk.CTkLabel(
            frame, text='● Joystick:  Izq → X/Y   Gatillos → Z   Der → rotacion',
            font=('Roboto', 12, 'bold'), text_color=COL['muted'],
            fg_color=COL['surface'], corner_radius=6, height=30)
        self.lbl_joy_state.pack(fill='x', padx=6, pady=(2, 6))

        # Selector de modo
        self.var_teleop_mode = ctk.StringVar(value='Translacion')
        ctk.CTkSegmentedButton(
            frame, values=['Translacion', 'Orientacion'],
            variable=self.var_teleop_mode,
            command=self._teleop_switch_mode).pack(fill='x', padx=6, pady=4)

        # Pasos
        step_row = ctk.CTkFrame(frame, fg_color='transparent')
        step_row.pack(fill='x', padx=4, pady=2)
        ctk.CTkLabel(step_row, text='Paso lineal (m):').pack(side='left')
        self.entry_tp_lin = ctk.CTkEntry(step_row, width=58)
        self.entry_tp_lin.insert(0, '0.01')
        self.entry_tp_lin.pack(side='left', padx=4)
        ctk.CTkLabel(step_row, text='angular (°):').pack(side='left', padx=(10, 0))
        self.entry_tp_ang = ctk.CTkEntry(step_row, width=52)
        self.entry_tp_ang.insert(0, '5')
        self.entry_tp_ang.pack(side='left', padx=4)

        # ── Estabilizacion de orientacion ─────────────────────────────
        fr_stab = ctk.CTkFrame(frame, fg_color='transparent')
        fr_stab.pack(fill='x', padx=4, pady=(4, 2))
        self.var_stab = ctk.BooleanVar(value=False)
        self.chk_stab = ctk.CTkCheckBox(
            fr_stab, text='Estabilizar orientacion de camara',
            variable=self.var_stab, command=self._teleop_toggle_stab)
        self.chk_stab.pack(side='left')
        self.lbl_stab_estado = ctk.CTkLabel(
            fr_stab, text='', font=('Roboto', 11), text_color=COL['ok'])
        self.lbl_stab_estado.pack(side='left', padx=8)

        # Botones de translacion de la PUNTA (efector final) en linea recta
        self._tp_frame_trans = ctk.CTkFrame(frame, fg_color='transparent')

        # Marco de referencia: Mundo (base) = lineas rectas X/Y/Z fijas;
        # Camara = ejes locales de la punta (seguir la orientacion actual).
        self.var_tp_frame = ctk.StringVar(value='Camara')
        fr_sel = ctk.CTkFrame(self._tp_frame_trans, fg_color='transparent')
        fr_sel.pack(fill='x', pady=(0, 4))
        ctk.CTkLabel(fr_sel, text='Ejes:').pack(side='left', padx=(2, 4))
        ctk.CTkSegmentedButton(
            fr_sel, values=['Mundo', 'Camara'],
            variable=self.var_tp_frame).pack(side='left', fill='x', expand=True)

        btn_grid = ctk.CTkFrame(self._tp_frame_trans, fg_color='transparent')
        btn_grid.pack(fill='x')
        btn_grid.grid_columnconfigure((0, 1), weight=1)
        self._tp_trans_btns: list[ctk.CTkButton] = []
        trans = [('Adelante (+X)', 0, +1), ('Atras (-X)', 0, -1),
                 ('Izquierda (+Y)', 1, +1), ('Derecha (-Y)', 1, -1),
                 ('Subir (+Z)', 2, +1), ('Bajar (-Z)', 2, -1)]
        for i, (txt, axis, sign) in enumerate(trans):
            b = ctk.CTkButton(
                btn_grid, text=txt, state='disabled',
                command=lambda a=axis, s=sign: self._teleop_translate(a, s))
            b.grid(row=i // 2, column=i % 2, sticky='ew', padx=3, pady=3)
            self._tp_trans_btns.append(b)

        # Botones de orientacion (incremento en el frame de la camara)
        self._tp_frame_orient = ctk.CTkFrame(frame, fg_color='transparent')
        self._tp_frame_orient.grid_columnconfigure((0, 1), weight=1)
        self._tp_orient_btns: list[ctk.CTkButton] = []
        orient = [('Pitch +', 1, +1), ('Pitch -', 1, -1),   # eje local Y
                  ('Yaw +',   2, +1), ('Yaw -',   2, -1),    # eje local Z
                  ('Roll +',  0, +1), ('Roll -',  0, -1)]    # eje local X
        for i, (txt, axis, sign) in enumerate(orient):
            b = ctk.CTkButton(
                self._tp_frame_orient, text=txt, state='disabled',
                command=lambda a=axis, s=sign: self._teleop_rotate(a, s))
            b.grid(row=i // 2, column=i % 2, sticky='ew', padx=3, pady=3)
            self._tp_orient_btns.append(b)

        self._tp_frame_trans.pack(fill='x', padx=4, pady=4)  # modo por defecto

        # ── Speed scale ───────────────────────────────────────────────
        fr_speed = ctk.CTkFrame(frame, fg_color='transparent')
        fr_speed.pack(fill='x', padx=4, pady=(8, 2))
        ctk.CTkLabel(fr_speed, text='Velocidad tray.:',
                     font=('Roboto', 11, 'bold')).pack(side='left')
        self.lbl_speed_val = ctk.CTkLabel(fr_speed, text='100%',
                                          width=42, font=('Roboto', 11),
                                          text_color=COL['accent'])
        self.lbl_speed_val.pack(side='right', padx=4)
        self.sld_speed = ctk.CTkSlider(
            frame, from_=5, to=100, number_of_steps=19,
            command=self._on_speed_change)
        self.sld_speed.set(100)
        self.sld_speed.pack(fill='x', padx=4, pady=(0, 6))

        self.btn_tp_sync = ctk.CTkButton(
            frame, text='Sincronizar con pose actual', fg_color='#5d6d7e',
            hover_color='#34495e', state='disabled', command=self._teleop_sync)
        self.btn_tp_sync.pack(fill='x', padx=4, pady=(6, 2))

        self.lbl_tp_pose = ctk.CTkLabel(frame, text='objetivo: —',
                                        text_color=COL['accent'],
                                        font=('Roboto', 11))
        self.lbl_tp_pose.pack(pady=2)
        self.lbl_tp_msg = ctk.CTkLabel(frame, text='', text_color=COL['warn'],
                                       wraplength=360)
        self.lbl_tp_msg.pack(pady=2)

    # ── Speed scale ───────────────────────────────────────────────

    def _on_speed_change(self, value):
        pct = float(value)
        self.lbl_speed_val.configure(text=f'{pct:.0f}%')
        self._node.publicar_speed_scale(pct)

    # ── Logica de teleoperacion ───────────────────────────────────

    def _teleop_switch_mode(self, _value=None):
        if self.var_teleop_mode.get() == 'Translacion':
            self._tp_frame_orient.pack_forget()
            self._tp_frame_trans.pack(fill='x', padx=4, pady=4)
        else:
            self._tp_frame_trans.pack_forget()
            self._tp_frame_orient.pack(fill='x', padx=4, pady=4)

    def _tp_lin_step(self) -> float:
        try:    return max(0.001, min(0.10, float(self.entry_tp_lin.get())))
        except ValueError: return 0.01

    def _tp_ang_step(self) -> float:
        try:    return max(0.5, min(30.0, float(self.entry_tp_ang.get())))
        except ValueError: return 5.0

    def _update_tp_label(self):
        if self._tp_p is None:
            return
        p = self._tp_p
        txt = f'objetivo  x {p[0]:+.3f}  y {p[1]:+.3f}  z {p[2]:+.3f} m'
        if self._tp_R is not None:
            R = self._tp_R
            pitch = math.degrees(-math.asin(max(-1.0, min(1.0, R[2, 0]))))
            roll  = math.degrees(math.atan2(R[2, 1], R[2, 2]))
            yaw   = math.degrees(math.atan2(R[1, 0], R[0, 0]))
            txt += f'\norient  R {roll:+.0f}  P {pitch:+.0f}  Y {yaw:+.0f}°'
        self.lbl_tp_pose.configure(text=txt)

    def _teleop_toggle_stab(self):
        """Activa/desactiva estabilizacion de orientacion de la camara."""
        if self.var_stab.get():
            pose = self._node.pose_actual
            if pose is None:
                self.var_stab.set(False)
                self.lbl_tp_msg.configure(
                    text='Sin pose actual para estabilizar.',
                    text_color=COL['err'])
                return
            self._stab_R = quaternion_to_matrix(pose.pose.orientation)
            self.lbl_stab_estado.configure(text='R bloqueada')
            self.lbl_tp_msg.configure(
                text='Estabilizacion activa: orientacion fija al mundo.',
                text_color=COL['ok'])
        else:
            self._stab_R = None
            self.lbl_stab_estado.configure(text='')
            self.lbl_tp_msg.configure(
                text='Estabilizacion desactivada.',
                text_color=COL['muted'])

    def _teleop_sync(self) -> bool:
        """Siembra la pose objetivo desde la pose real actual del efector."""
        pose = self._node.pose_actual
        if pose is None:
            self.lbl_tp_msg.configure(text='Aun no hay pose actual.',
                                      text_color=COL['warn'])
            return False
        p = pose.pose.position
        self._tp_p = np.array([p.x, p.y, p.z], float)
        # Con estabilizacion activa, no se toca la R bloqueada
        if not self.var_stab.get():
            self._tp_R = quaternion_to_matrix(pose.pose.orientation)
        self._update_tp_label()
        self.lbl_tp_msg.configure(text='Pose objetivo sincronizada.',
                                  text_color=COL['ok'])
        return True

    def _teleop_ensure_seed(self) -> bool:
        if self._tp_R is not None and self._tp_p is not None:
            return True
        return self._teleop_sync()

    def _teleop_translate(self, axis: int, sign: int):
        """Mueve la PUNTA en linea recta sobre un eje, manteniendo la orientacion.

        Marco 'Mundo': eje fijo de la base (X/Y/Z), p.ej. -Z baja siempre recto.
        Marco 'Camara': eje local de la punta (sigue la orientacion actual).
        La IK recalcula las demas articulaciones para mantener la punta en la
        recta; R se mantiene fija para que sea translacion pura.
        """
        if not self._teleop_ensure_seed():
            return
        step = sign * self._tp_lin_step()
        if self.var_tp_frame.get() == 'Camara':
            direccion = self._tp_R[:, axis]        # eje local de la punta
        else:
            direccion = np.eye(3)[:, axis]         # eje del mundo (base)
        p_new = self._tp_p + step * direccion
        self._teleop_send(p_new, self._tp_R)

    def _teleop_rotate(self, axis: int, sign: int):
        """Modo Orientacion: R_new = R · R_inc (incremento en frame local), p fija."""
        if not self._teleop_ensure_seed():
            return
        ang   = math.radians(sign * self._tp_ang_step())
        Rloc  = (_rotx3, _roty3, _rotz3)[axis](ang)
        R_new = self._tp_R @ Rloc
        self._teleop_send(self._tp_p, R_new)

    def _teleop_send(self, p_new, R_new):
        """Arma Td=[R|p] y resuelve via IK. Solo confirma el objetivo si la IK lo alcanza."""
        # Con estabilizacion activa, la orientacion siempre es la R bloqueada.
        # Las rotaciones manuales actualizan _stab_R para poder reorientar
        # intencionalmente mientras se mantiene la estabilizacion.
        if self.var_stab.get() and self._stab_R is not None:
            R_send = self._stab_R
            if not np.allclose(R_new, self._tp_R if self._tp_R is not None else R_new):
                self._stab_R = R_new   # rotacion intencional → actualiza R fija
                R_send = R_new
        else:
            R_send = R_new
        elbow = getattr(self, 'var_elbow', None)
        elbow = elbow.get() if elbow is not None else 'down'
        self.lbl_tp_msg.configure(text='Calculando IK...', text_color=COL['warn'])

        def _cb(ok, q_rad, msg):
            if ok:
                self._pending_teleop_commit = (p_new, R_send)
                self._pending_teleop_q      = q_rad
                self._pending_teleop_msg    = ('Objetivo alcanzado', COL['ok'])
            else:
                self._pending_teleop_q   = None
                self._pending_teleop_msg = (f'No alcanzable: {msg}', COL['err'])

        self._node.pedir_ik_pose(p_new, R_send, elbow, _cb)

    def _joystick_teleop_step(self):
        """Teleop cartesiano con el mando (pestana Teleop, brazo al frente).
        SIEMPRE en ejes de la CAMARA (locales a la punta) y combina traslacion
        + orientacion en una sola IK:
          - stick IZQUIERDO → traslacion X / Y
          - gatillos L2/R2  → traslacion Z (R2 acerca/aleja segun signo)
          - stick DERECHO   → rotacion (yaw / pitch)
        Mientras mantengas el stick, se va desplazando (1 paso por ciclo).
        Manda 1 sola peticion IK a la vez."""
        axes = self._node._joy_axes
        dz = cfg.ARM_DEADZONE

        def ax(i):
            v = axes[i] if (axes and i < len(axes)) else 0.0
            return v if abs(v) > dz else 0.0

        def trig(i):                    # gatillo: reposo +1, presionado -1 → 0..1
            v = axes[i] if (axes and i < len(axes)) else 1.0
            return max(0.0, (1.0 - v) / 2.0)

        # Stick IZQUIERDO → X/Y ;  gatillos → Z ;  stick DERECHO → rotacion
        tx = ax(cfg.AXIS_LEFT_X)
        ty = ax(cfg.AXIS_LEFT_Y)
        tz = trig(cfg.AXIS_R2) - trig(cfg.AXIS_L2)
        ryaw   = ax(cfg.AXIS_RIGHT_X)
        rpitch = ax(cfg.AXIS_RIGHT_Y)

        active = bool(tx or ty or tz or ryaw or rpitch)
        self._set_joy_indicator(active)
        if not active:
            return

        # 1 sola IK a la vez (con timeout de seguridad si no responde)
        if self._joy_ik_inflight:
            self._joy_inflight_ticks += 1
            if self._joy_inflight_ticks < 16:   # ~2 s a 120 ms/loop
                return
            self._joy_ik_inflight = False
        if not self._teleop_ensure_seed():
            return

        # Traslacion en ejes de la CAMARA (columnas de R = ejes locales de la punta)
        lin = self._tp_lin_step()
        basis = self._tp_R
        # stick Y suele venir invertido (arriba = -1)
        dp = lin * (tx * basis[:, 0] - ty * basis[:, 1] + tz * basis[:, 2])
        p_new = self._tp_p + dp

        # Orientacion: incremento en el frame local de la punta
        ang = math.radians(self._tp_ang_step())
        R_new = self._tp_R @ _rotz3(ryaw * ang) @ _roty3(-rpitch * ang)

        self._joy_ik_inflight = True
        self._joy_inflight_ticks = 0
        self._teleop_send(p_new, R_new)

    def _set_joy_indicator(self, active):
        """Ilumina (verde) el indicador del teleop cuando el mando mueve el brazo."""
        if getattr(self, '_joy_ind_on', None) == active:
            return
        self._joy_ind_on = active
        if active:
            self.lbl_joy_state.configure(
                text='● MOVIENDO  —  Izq X/Y · Gatillos Z · Der rotacion',
                text_color=COL['panel_bg'], fg_color=COL['ok'])
        else:
            self.lbl_joy_state.configure(
                text='● Joystick:  Izq → X/Y   Gatillos → Z   Der → rotacion',
                text_color=COL['muted'], fg_color=COL['surface'])

    # ── Tab Jog: mueve cada servo individualmente con el mando ─────
    #   L1/R1        -> Base        (paso por pulsacion)
    #   L2/R2        -> Hombro      (continuo mientras se mantiene)
    #   Stick Izq X  -> Codo
    #   Stick Izq Y  -> Munieca_P
    #   Stick Der X  -> Munieca_Y
    #   Stick Der Y  -> Munieca_R

    _JOG_GP_LABELS = ('L1 ◄ / ► R1', 'L2 ◄ / ► R2', 'Stick Izq X',
                       'Stick Izq Y', 'Stick Der X', 'Stick Der Y')

    def _build_tab_jog(self, tab):
        frame = ctk.CTkScrollableFrame(tab, fg_color='transparent')
        frame.pack(fill='both', expand=True)

        ctk.CTkLabel(
            frame, text='Jog manual por servo (mando)',
            font=('Roboto', 14, 'bold')).pack(pady=(4, 2))

        self.lbl_jog_gp_state = ctk.CTkLabel(
            frame, text='● Joystick:  L1/R1 Base · L2/R2 Hombro · '
                        'sticks Codo/Munieca_P/Y/R',
            fg_color=COL['surface'], text_color=COL['muted'],
            corner_radius=6, height=28)
        self.lbl_jog_gp_state.pack(fill='x', padx=4, pady=(0, 8))

        step_row = ctk.CTkFrame(frame, fg_color='transparent')
        step_row.pack(fill='x', padx=4, pady=4)
        ctk.CTkLabel(step_row, text='Paso (°/ciclo):').pack(side='left')
        self.entry_gp_jog_step = ctk.CTkEntry(step_row, width=55)
        self.entry_gp_jog_step.insert(0, '2')
        self.entry_gp_jog_step.pack(side='left', padx=6)
        ctk.CTkLabel(step_row, text='Vel %:').pack(side='left', padx=(12, 0))
        self.entry_gp_jog_vel = ctk.CTkEntry(step_row, width=55)
        self.entry_gp_jog_vel.insert(0, '15')
        self.entry_gp_jog_vel.pack(side='left', padx=6)

        self._lbl_gp_jog_angs: list[ctk.CTkLabel] = []
        for name, ctrl in zip(self._node.joint_order, self._JOG_GP_LABELS):
            row = ctk.CTkFrame(frame)
            row.pack(fill='x', padx=2, pady=2)
            ctk.CTkLabel(row, text=name, width=90,
                         anchor='w').pack(side='left', padx=6)
            lbl = ctk.CTkLabel(row, text='0.00°', width=64, anchor='e',
                               font=('Roboto', 12, 'bold'),
                               text_color=COL['accent'])
            lbl.pack(side='left')
            self._lbl_gp_jog_angs.append(lbl)
            ctk.CTkLabel(row, text=ctrl, width=110, anchor='e',
                         text_color=COL['muted']).pack(side='right', padx=6)

        self.lbl_gp_jog_msg = ctk.CTkLabel(frame, text='',
                                           text_color=COL['warn'], wraplength=340)
        self.lbl_gp_jog_msg.pack(pady=2)

    def _gp_jog_step(self) -> float:
        try:    return max(0.2, min(10.0, float(self.entry_gp_jog_step.get())))
        except ValueError: return 2.0

    def _gp_jog_vel(self) -> float:
        try:    return max(1.0, min(50.0, float(self.entry_gp_jog_vel.get())))
        except ValueError: return 15.0

    def _set_gp_jog_indicator(self, active):
        if self._jog_gp_indicator_on == active:
            return
        self._jog_gp_indicator_on = active
        if active:
            self.lbl_jog_gp_state.configure(
                text='● MOVIENDO SERVO', text_color=COL['panel_bg'],
                fg_color=COL['ok'])
        else:
            self.lbl_jog_gp_state.configure(
                text='● Joystick:  L1/R1 Base · L2/R2 Hombro · '
                     'sticks Codo/Munieca_P/Y/R',
                text_color=COL['muted'], fg_color=COL['surface'])

    def _jog_by_index(self, idx: int, sign: float):
        """Jog directo del servo `idx` (indice en joint_order), sin pasar por
        IK: pide al driver un nuevo target = angulo actual + paso, a la
        velocidad configurada. Guardia por-joint (`_jog_inflight`) para no
        acumular llamadas de servicio mientras el mando se mantiene pulsado."""
        if idx in self._jog_inflight:
            return
        joint_order = self._node.joint_order
        if idx >= len(joint_order):
            return
        name = joint_order[idx]
        current_deg = np.degrees(self._node.q_actual[idx])
        target = (current_deg + sign * self._gp_jog_step()) % 360.0
        self._jog_inflight.add(idx)

        def _cb(ok, msg):
            self._jog_inflight.discard(idx)
            if not ok:
                self._set_gp_jog_msg(f'{name}: {msg}', COL['err'])

        self._node.jog(name, target, self._gp_jog_vel(), on_result=_cb)

    def _set_gp_jog_msg(self, text: str, color: str):
        self.lbl_gp_jog_msg.configure(text=text, text_color=color)

    def _joystick_jog_step(self):
        """Lee el mando cada ciclo (~120 ms) y jogea el servo mapeado a cada
        control mientras la pestana 'Jog' este activa. L1/R1 llegan como
        pasos ya contados por el nodo (_submode_step, edge-triggered); los
        gatillos y los sticks se leen como ejes continuos (activos mientras
        se mantienen)."""
        ax_st, ex_st = self._node.ax_status, self._node.ex_status
        if not (ax_st['conectado'] or ex_st['conectado']) or \
           ax_st['emergencia'] or ex_st['emergencia']:
            self._node._submode_step = 0
            return

        axes = self._node._joy_axes
        dz = cfg.ARM_DEADZONE

        def ax(i):
            v = axes[i] if (axes and i < len(axes)) else 0.0
            return v if abs(v) > dz else 0.0

        def trig(i):
            v = axes[i] if (axes and i < len(axes)) else 1.0
            return max(0.0, (1.0 - v) / 2.0)

        # L1/R1 -> Base: pasos discretos ya contados por el nodo (evita que
        # ademas ciclen de pestana mientras estamos en 'Jog').
        base_step = self._node._submode_step
        if base_step:
            self._node._submode_step = 0

        hombro_delta = trig(cfg.AXIS_R2) - trig(cfg.AXIS_L2)
        codo_delta      = ax(cfg.AXIS_LEFT_X)
        munieca_p_delta = -ax(cfg.AXIS_LEFT_Y)   # stick Y viene invertido
        munieca_y_delta = ax(cfg.AXIS_RIGHT_X)
        munieca_r_delta = -ax(cfg.AXIS_RIGHT_Y)

        deltas = [base_step, hombro_delta, codo_delta,
                  munieca_p_delta, munieca_y_delta, munieca_r_delta]

        active = any(abs(d) > 1e-6 for d in deltas)
        self._set_gp_jog_indicator(active)
        if not active:
            return

        for idx, d in enumerate(deltas):
            if abs(d) <= 1e-6:
                continue
            sign = 1.0 if d > 0 else -1.0
            self._jog_by_index(idx, sign)

    # ── Tab IK ────────────────────────────────────────────────────

    def _build_tab_ik(self, tab):
        frame = ctk.CTkScrollableFrame(tab, fg_color='transparent')
        frame.pack(fill='both', expand=True)

        ik_rows = [('X (m):', '0.30'), ('Y (m):', '0.0'), ('Z (m):', '0.20'),
                   ('Beta (°):', '0'), ('q5 (°):', '0'), ('q6 (°):', '0')]
        self._ik_ent: list[ctk.CTkEntry] = []
        for lbl_t, val in ik_rows:
            e = self._row_entry(frame, lbl_t, val, lw=80)
            self._ik_ent.append(e)

        elbow_row = ctk.CTkFrame(frame, fg_color='transparent')
        elbow_row.pack(fill='x', padx=4, pady=2)
        ctk.CTkLabel(elbow_row, text='Codo:').pack(side='left')
        self.var_elbow = ctk.StringVar(value='down')
        ctk.CTkRadioButton(elbow_row, text='Abajo', variable=self.var_elbow,
                           value='down').pack(side='left', padx=8)
        ctk.CTkRadioButton(elbow_row, text='Arriba', variable=self.var_elbow,
                           value='up').pack(side='left')

        step_row = ctk.CTkFrame(frame, fg_color='transparent')
        step_row.pack(fill='x', padx=4, pady=2)
        ctk.CTkLabel(step_row, text='Paso (m):').pack(side='left')
        self.entry_cart_step = ctk.CTkEntry(step_row, width=60)
        self.entry_cart_step.insert(0, '0.005')
        self.entry_cart_step.pack(side='left', padx=6)

        traj_row = ctk.CTkFrame(frame, fg_color='transparent')
        traj_row.pack(fill='x', padx=4, pady=2)
        ctk.CTkLabel(traj_row, text='Pasos tray. (0=directo):').pack(side='left')
        self.entry_traj_steps = ctk.CTkEntry(traj_row, width=46)
        self.entry_traj_steps.insert(0, '0')
        self.entry_traj_steps.pack(side='left', padx=4)
        ctk.CTkLabel(traj_row, text='dt(s):').pack(side='left', padx=(10, 0))
        self.entry_traj_dt = ctk.CTkEntry(traj_row, width=46)
        self.entry_traj_dt.insert(0, '0.10')
        self.entry_traj_dt.pack(side='left', padx=4)

        tol_row = ctk.CTkFrame(frame, fg_color='transparent')
        tol_row.pack(fill='x', padx=4, pady=2)
        ctk.CTkLabel(tol_row, text='Tol. ruta (m):').pack(side='left')
        self.entry_path_tol = ctk.CTkEntry(tol_row, width=54)
        self.entry_path_tol.insert(0, '0.0')
        self.entry_path_tol.pack(side='left', padx=4)
        ctk.CTkLabel(tol_row, text='meta (m):').pack(side='left', padx=(10, 0))
        self.entry_goal_tol = ctk.CTkEntry(tol_row, width=54)
        self.entry_goal_tol.insert(0, '0.02')
        self.entry_goal_tol.pack(side='left', padx=4)

        self.btn_use_pose = ctk.CTkButton(
            step_row, text='Usar pose actual', width=130,
            state='disabled', command=self._copy_current_pose_to_ik)
        self.btn_use_pose.pack(side='left', padx=6)

        cart_btn_row = ctk.CTkFrame(frame, fg_color='transparent')
        cart_btn_row.pack(pady=4)
        for text, delta in [
            ('X-', (-1, 0, 0)), ('X+', (1, 0, 0)),
            ('Y-', (0, -1, 0)), ('Y+', (0, 1, 0)),
            ('Z-', (0, 0, -1)), ('Z+', (0, 0, 1)),
        ]:
            btn = ctk.CTkButton(
                cart_btn_row, text=text, width=48, state='disabled',
                command=lambda d=delta: self._move_cart_delta(*d))
            btn.pack(side='left', padx=2)
            self._cart_delta_buttons.append(btn)

        self.btn_ik = ctk.CTkButton(frame, text='MOVER (IK)',
                                    state='disabled', command=self._move_ik)
        self.btn_ik.pack(fill='x', pady=6, padx=4)
        self.lbl_ik_warn = ctk.CTkLabel(frame, text='', text_color=COL['warn'],
                                        wraplength=340)
        self.lbl_ik_warn.pack()

        # ── Panel de estado de trayectoria cartesiana ─────────────────
        fr_traj = ctk.CTkFrame(frame)
        fr_traj.pack(fill='x', pady=(4, 2), padx=2)
        ctk.CTkLabel(fr_traj, text='Trayectoria cartesiana',
                     font=('Roboto', 11, 'bold')).pack(pady=(4, 2))
        self.traj_progress = ctk.CTkProgressBar(fr_traj, height=14)
        self.traj_progress.set(0)
        self.traj_progress.pack(fill='x', padx=8, pady=2)
        self.lbl_traj_state = ctk.CTkLabel(
            fr_traj, text='En espera', font=('Roboto', 11),
            text_color=COL['muted'])
        self.lbl_traj_state.pack(pady=(0, 4))

        # Pose actual del efector
        fr_pose = ctk.CTkFrame(frame)
        fr_pose.pack(fill='x', pady=6, padx=2)
        ctk.CTkLabel(fr_pose, text='Efector (tiempo real)',
                     font=('Roboto', 12, 'bold')).pack(pady=(4, 2))
        self.lbl_pose_xyz = ctk.CTkLabel(fr_pose, text='x — | y — | z —',
                                         text_color=COL['accent'],
                                         font=('Roboto', 12))
        self.lbl_pose_xyz.pack()
        self.lbl_pose_rpy = ctk.CTkLabel(fr_pose, text='R — | P — | Y —',
                                         text_color=COL['accent'],
                                         font=('Roboto', 12))
        self.lbl_pose_rpy.pack(pady=(0, 4))

    # ── Tab calibracion ───────────────────────────────────────────

    def _build_tab_calib(self, tab):
        frame = ctk.CTkScrollableFrame(tab, fg_color='transparent')
        frame.pack(fill='both', expand=True)

        self.btn_calib_start = ctk.CTkButton(
            frame, text='Iniciar calibracion',
            fg_color='#7d6608', hover_color='#5a4b08',
            state='disabled', command=self._node.calib_start)
        self.btn_calib_start.pack(fill='x', pady=4, padx=4)

        self.lbl_calib_hint = ctk.CTkLabel(
            frame, text='Durante la calibracion, mueve cada joint con ◄ ► '
                        'hasta su posicion home y confirma.',
            wraplength=340, text_color=COL['muted'])
        self.lbl_calib_hint.pack(pady=2)

        step_row = ctk.CTkFrame(frame, fg_color='transparent')
        step_row.pack(fill='x', padx=4, pady=4)
        ctk.CTkLabel(step_row, text='Paso (°):').pack(side='left')
        self.entry_jog_step = ctk.CTkEntry(step_row, width=55)
        self.entry_jog_step.insert(0, '5')
        self.entry_jog_step.pack(side='left', padx=6)
        ctk.CTkLabel(step_row, text='Vel %:').pack(side='left', padx=(12, 0))
        self.entry_jog_vel = ctk.CTkEntry(step_row, width=55)
        self.entry_jog_vel.insert(0, '10')
        self.entry_jog_vel.pack(side='left', padx=6)

        self._lbl_jog_angs: list[ctk.CTkLabel] = []
        self._jog_buttons:  list[ctk.CTkButton] = []
        for name in self._node.joint_order:
            row = ctk.CTkFrame(frame)
            row.pack(fill='x', padx=2, pady=2)
            ctk.CTkLabel(row, text=name, width=90,
                         anchor='w').pack(side='left', padx=6)
            lbl = ctk.CTkLabel(row, text='0.00°', width=64, anchor='e',
                               font=('Roboto', 12, 'bold'),
                               text_color=COL['accent'])
            lbl.pack(side='left')
            self._lbl_jog_angs.append(lbl)
            for txt, sign in (('◄', -1), ('►', +1)):
                btn = ctk.CTkButton(
                    row, text=txt, width=40, state='disabled',
                    fg_color='#5d6d7e', hover_color='#34495e',
                    command=lambda n=name, s=sign: self._jog(n, s))
                btn.pack(side='left', padx=4, pady=2)
                self._jog_buttons.append(btn)

        self.lbl_jog_msg = ctk.CTkLabel(frame, text='',
                                        text_color=COL['warn'], wraplength=340)
        self.lbl_jog_msg.pack(pady=2)

        self.btn_calib_ok = ctk.CTkButton(
            frame, text='CONFIRMAR HOME (actual = 0°)',
            font=('Roboto', 13, 'bold'),
            fg_color='#1e8449', hover_color='#145a32',
            height=40, state='disabled',
            command=self._node.calib_confirm)
        self.btn_calib_ok.pack(fill='x', padx=4, pady=8)

    # ── Tab rescue ────────────────────────────────────────────────

    def _build_tab_rescue(self, tab):
        frame = ctk.CTkFrame(tab, fg_color='transparent')
        frame.pack(fill='both', expand=True)

        ctk.CTkLabel(frame, text='Pulso corto de velocidad para liberar\n'
                                 'un servo mecanicamente atascado.',
                     text_color=COL['muted']).pack(pady=6)

        row = ctk.CTkFrame(frame, fg_color='transparent')
        row.pack(pady=4)
        ctk.CTkLabel(row, text='Joint:').pack(side='left')
        self.var_rescue_joint = ctk.StringVar(value=self._node.joint_order[0])
        ctk.CTkOptionMenu(row, values=self._node.joint_order,
                          variable=self.var_rescue_joint,
                          width=130).pack(side='left', padx=6)
        ctk.CTkLabel(row, text='Vel %:').pack(side='left', padx=(8, 0))
        self.entry_rescue_vel = ctk.CTkEntry(row, width=50)
        self.entry_rescue_vel.insert(0, '15')
        self.entry_rescue_vel.pack(side='left', padx=4)

        btn_row = ctk.CTkFrame(frame, fg_color='transparent')
        btn_row.pack(pady=6)
        self.btn_rescue_neg = ctk.CTkButton(
            btn_row, text='◄ Pulso −', width=130, state='disabled',
            fg_color='#922b21', hover_color='#6e1f16',
            command=lambda: self._rescue_pulse(-1))
        self.btn_rescue_neg.pack(side='left', padx=4)
        self.btn_rescue_pos = ctk.CTkButton(
            btn_row, text='Pulso + ►', width=130, state='disabled',
            fg_color='#922b21', hover_color='#6e1f16',
            command=lambda: self._rescue_pulse(+1))
        self.btn_rescue_pos.pack(side='left', padx=4)

        self.lbl_rescue_msg = ctk.CTkLabel(frame, text='',
                                           text_color=COL['warn'],
                                           wraplength=340)
        self.lbl_rescue_msg.pack(pady=4)

    # ── Tab estado de servos ──────────────────────────────────────

    def _build_tab_estado(self, tab):
        frame = ctk.CTkScrollableFrame(tab, fg_color='transparent')
        frame.pack(fill='both', expand=True)

        hdr = ctk.CTkFrame(frame, fg_color='transparent')
        hdr.pack(fill='x', pady=(2, 2))
        for txt, w in (('Joint', 86), ('Torque', 56), ('T°', 42), ('V', 46)):
            ctk.CTkLabel(hdr, text=txt, width=w, anchor='w',
                         font=('Roboto', 11, 'bold')).pack(side='left', padx=2)
        ctk.CTkLabel(hdr, text='Alerta', anchor='w',
                     font=('Roboto', 11, 'bold')).pack(side='left', padx=4)

        self._estado_rows: dict[str, dict] = {}
        for name in self._node.joint_order:
            row = ctk.CTkFrame(frame)
            row.pack(fill='x', padx=2, pady=2)
            ctk.CTkLabel(row, text=name, width=86, anchor='w',
                         font=('Roboto', 12, 'bold')).pack(side='left', padx=2)
            l_tor = ctk.CTkLabel(row, text='—', width=56)
            l_tor.pack(side='left', padx=2)
            l_tmp = ctk.CTkLabel(row, text='—', width=42)
            l_tmp.pack(side='left', padx=2)
            l_vol = ctk.CTkLabel(row, text='—', width=46)
            l_vol.pack(side='left', padx=2)
            l_alr = ctk.CTkLabel(row, text='—', anchor='w',
                                 text_color=COL['muted'])
            l_alr.pack(side='left', padx=4)
            self._estado_rows[name] = {'torque': l_tor, 'temp': l_tmp,
                                       'volt': l_vol, 'alerta': l_alr}

        btns = ctk.CTkFrame(frame, fg_color='transparent')
        btns.pack(fill='x', pady=8)
        btns.grid_columnconfigure((0, 1), weight=1)
        self.btn_status_refresh = ctk.CTkButton(
            btns, text='Refrescar', fg_color='#5d6d7e',
            hover_color='#34495e', command=self._refresh_status)
        self.btn_status_refresh.grid(row=0, column=0, sticky='ew', padx=2)
        self.btn_reset_alerts = ctk.CTkButton(
            btns, text='Reiniciar alertas (rearmar torque)',
            fg_color='#1e8449', hover_color='#145a32', state='disabled',
            command=self._node.reset_alerts)
        self.btn_reset_alerts.grid(row=0, column=1, sticky='ew', padx=2)

        self.lbl_estado_hint = ctk.CTkLabel(
            frame, text='Una alarma (sobrecarga/temperatura) apaga el torque. '
                        'Quita la causa y pulsa "Reiniciar alertas".',
            wraplength=360, text_color=COL['muted'])
        self.lbl_estado_hint.pack(pady=2)

    def _refresh_status(self):
        self._node.pedir_servo_status(self._on_status_result)

    def _on_status_result(self, filas):
        """Llamado desde el thread ROS — guarda filas para aplicar en _loop_ui."""
        for fila in filas:
            self._pending_status[fila['nombre']] = fila

    # ── Columna derecha: visualizacion 3D embebida ─────────────────

    def _build_right_panel(self):
        right = ctk.CTkFrame(self)
        right.grid(row=0, column=1, sticky='nsew', padx=(4, 8), pady=8)
        right.grid_rowconfigure(0, weight=3)   # viz 3D
        right.grid_rowconfigure(1, weight=2)   # camara frontal
        right.grid_columnconfigure(0, weight=1)

        reach = self._node.reach

        self._fig = Figure(figsize=(7, 7), facecolor=COL['panel_bg'])
        gs = self._fig.add_gridspec(2, 2, hspace=0.32, wspace=0.28,
                                    height_ratios=[1.6, 1.0])

        self._ax3d = self._fig.add_subplot(gs[0, :], projection='3d')
        self._ax_fr = self._fig.add_subplot(gs[1, 0])
        self._ax_tp = self._fig.add_subplot(gs[1, 1])

        self._ax3d.set_facecolor(COL['panel_bg'])
        self._ax3d.set_xlim(-reach, reach)
        self._ax3d.set_ylim(-reach, reach)
        self._ax3d.set_zlim(0, reach * 1.2)
        self._ax3d.set_title('Brazo (azul = real, naranja = preview IK)',
                             color='white', fontsize=10)
        self._ax3d.tick_params(colors='gray', labelsize=7)
        self._ax3d.set_xlabel('X', color='gray', fontsize=8)
        self._ax3d.set_ylabel('Y', color='gray', fontsize=8)
        self._ax3d.set_zlabel('Z', color='gray', fontsize=8)
        self._ax3d.view_init(elev=22, azim=-58)

        for ax, title, xlim, ylim in [
            (self._ax_fr, 'Frontal XZ', (-reach, reach), (0, reach * 1.2)),
            (self._ax_tp, 'Superior XY', (-reach, reach), (-reach, reach)),
        ]:
            ax.set_facecolor(COL['panel_bg'])
            ax.set_title(title, color='white', fontsize=9)
            ax.tick_params(colors='gray', labelsize=7)
            ax.grid(True, color='#333', lw=0.5)
            ax.set_xlim(*xlim)
            ax.set_ylim(*ylim)
            ax.set_aspect('equal')

        kw = dict(color=COL['real'], lw=3, ms=6,
                  markerfacecolor=COL['real_dot'], marker='o')
        kw2 = dict(color=COL['real'], lw=2, ms=4,
                   markerfacecolor=COL['real_dot'], marker='o')
        self._ln3d,  = self._ax3d.plot([], [], [], **kw)
        self._ln_fr, = self._ax_fr.plot([], [], **kw2)
        self._ln_tp, = self._ax_tp.plot([], [], **kw2)

        # Pinza (ultimo segmento muñeca→punta) en verde, mas gruesa
        kwg = dict(color=COL['ok'], lw=5, ms=7,
                   markerfacecolor=COL['ok'], marker='o', solid_capstyle='round')
        self._ln3d_grip, = self._ax3d.plot([], [], [], **kwg)
        self._ln_fr_grip, = self._ax_fr.plot([], [], **dict(kwg, lw=4, ms=5))
        self._ln_tp_grip, = self._ax_tp.plot([], [], **dict(kwg, lw=4, ms=5))

        kwp = dict(color=COL['prev'], lw=2, ms=5,
                   markerfacecolor=COL['prev_dot'], marker='o',
                   linestyle='--', alpha=0.7)
        self._ln3d_prev,  = self._ax3d.plot([], [], [], **kwp)
        self._ln_fr_prev, = self._ax_fr.plot([], [], **kwp)
        self._ln_tp_prev, = self._ax_tp.plot([], [], **kwp)

        self._canvas = FigureCanvasTkAgg(self._fig, master=right)
        self._canvas.get_tk_widget().grid(row=0, column=0, sticky='nsew',
                                          padx=4, pady=4)

        # ── Camara frontal (NO la Orbbec) + QR/senales, lado derecho ──
        cam = ctk.CTkFrame(right, fg_color=COL['surface'])
        cam.grid(row=1, column=0, sticky='nsew', padx=4, pady=(0, 4))
        cam.grid_columnconfigure(0, weight=1)
        cam.grid_rowconfigure(1, weight=1)
        ctk.CTkLabel(cam, text='Camara frontal', anchor='w',
                     text_color=COL['muted'], font=('Roboto', 12, 'bold')
                     ).grid(row=0, column=0, sticky='ew', padx=8, pady=(6, 2))
        self.cam_label = tk.Label(cam, text='Sin imagen de camara frontal',
                                  bg=COL['surface'], fg=COL['muted'])
        self.cam_label.grid(row=1, column=0, sticky='nsew', padx=8, pady=2)
        self.cam_photo = None
        self._drawn_cam_version = -1
        self.lbl_qr = ctk.CTkLabel(cam, text='QR: —', anchor='w',
                                   text_color=COL['accent'], font=('Roboto', 12, 'bold'))
        self.lbl_qr.grid(row=2, column=0, sticky='ew', padx=8)
        self.lbl_det = ctk.CTkLabel(cam, text='Senal: —', anchor='w',
                                    text_color=COL['warn'], font=('Roboto', 12, 'bold'))
        self.lbl_det.grid(row=3, column=0, sticky='ew', padx=8, pady=(0, 6))

    def _redraw_arm(self):
        pts, pts_pre, version = self._node.puntos_brazo()
        if version == self._drawn_pts_version:
            return
        self._drawn_pts_version = version

        if pts is not None:
            self._ln3d.set_data_3d(pts[:, 0], pts[:, 1], pts[:, 2])
            self._ln_fr.set_data(pts[:, 0], pts[:, 2])
            self._ln_tp.set_data(pts[:, 0], pts[:, 1])
            # Pinza = ultimo segmento (muñeca → punta)
            g = pts[-2:]
            self._ln3d_grip.set_data_3d(g[:, 0], g[:, 1], g[:, 2])
            self._ln_fr_grip.set_data(g[:, 0], g[:, 2])
            self._ln_tp_grip.set_data(g[:, 0], g[:, 1])
        if pts_pre is not None:
            self._ln3d_prev.set_data_3d(pts_pre[:, 0], pts_pre[:, 1],
                                        pts_pre[:, 2])
            self._ln_fr_prev.set_data(pts_pre[:, 0], pts_pre[:, 2])
            self._ln_tp_prev.set_data(pts_pre[:, 0], pts_pre[:, 1])
        self._canvas.draw_idle()

    # ── Helpers ───────────────────────────────────────────────────

    def _row_entry(self, parent, label, default, lw=140):
        row = ctk.CTkFrame(parent, fg_color='transparent')
        row.pack(fill='x', padx=4, pady=2)
        ctk.CTkLabel(row, text=label, width=lw, anchor='w').pack(side='left')
        e = ctk.CTkEntry(row, width=90)
        e.insert(0, default)
        e.pack(side='left', padx=4)
        return e

    def _vel(self) -> float:
        return max(1.0, min(100.0, float(self.sld_vel.get())))

    def _jog_step(self) -> float:
        try:    return max(0.5, min(45.0, float(self.entry_jog_step.get())))
        except ValueError: return 5.0

    def _jog_vel(self) -> float:
        try:    return max(1.0, min(50.0, float(self.entry_jog_vel.get())))
        except ValueError: return 10.0

    def _rescue_vel(self) -> float:
        try:    return max(5.0, min(30.0, float(self.entry_rescue_vel.get())))
        except ValueError: return 15.0

    def _cart_step(self) -> float:
        try:    return max(0.001, min(0.20, float(self.entry_cart_step.get())))
        except ValueError: return 0.005

    def _on_vel_slider(self, value):
        self.lbl_vel.configure(text=f'{value:.0f} %')

    def _on_fk_slider(self, idx, value):
        self._fk_val_lbls[idx].configure(text=f'{value:.1f}°')

    def _sync_sliders_to_actual(self):
        q = self._node.q_actual
        for i, sld in enumerate(self._fk_sliders):
            deg = math.degrees(q[i])
            deg = ((deg + 180.0) % 360.0) - 180.0
            sld.set(deg)
            self._fk_val_lbls[i].configure(text=f'{deg:.1f}°')

    def _set_ik_xyz(self, x: float, y: float, z: float):
        for entry, value in zip(self._ik_ent[:3], [x, y, z]):
            entry.delete(0, 'end')
            entry.insert(0, f'{value:.4f}')

    def _set_ik_orientation(self, beta_deg: float, q5_deg: float,
                            q6_deg: float):
        for entry, value in zip(self._ik_ent[3:6], [beta_deg, q5_deg, q6_deg]):
            entry.delete(0, 'end')
            entry.insert(0, f'{value:.2f}')

    def _current_orientation_deg(self) -> tuple[float, float, float]:
        q = self._node.q_actual
        if len(q) < 6:
            return 0.0, 0.0, 0.0
        beta = q[1] - math.pi/2 + q[2] + q[3]
        return math.degrees(beta), math.degrees(q[4]), math.degrees(q[5])

    def _copy_current_pose_to_ik(self):
        pose = self._node.pose_actual
        if pose is None:
            self.lbl_ik_warn.configure(text='Aun no hay pose actual.',
                                       text_color=COL['warn'])
            return
        p = pose.pose.position
        self._set_ik_xyz(p.x, p.y, p.z)
        self._set_ik_orientation(*self._current_orientation_deg())

    def _move_cart_delta(self, dx: int, dy: int, dz: int):
        """Mueve la punta UN paso sobre un eje del MUNDO (base), manteniendo
        la orientacion actual EXACTA (translacion pura).

        Usa /compute_ik_pose con la matriz R completa de la pose actual — NO la
        reparametrizacion beta/q5/q6+atan2(y,x), que acoplaba la orientacion a la
        posicion y hacia que el movimiento en un solo eje no funcionara.
        La IK numerica se siembra con la configuracion actual => movimiento
        continuo y suave sin volteos de munieca.
        """
        pose = self._node.pose_actual
        if pose is None:
            self.lbl_ik_warn.configure(text='Aun no hay pose actual.',
                                       text_color=COL['warn'])
            return
        step = self._cart_step()
        p = pose.pose.position
        R = quaternion_to_matrix(pose.pose.orientation)   # orientacion a fijar
        p_new = np.array([p.x + dx * step,
                          p.y + dy * step,
                          p.z + dz * step], float)
        self._set_ik_xyz(*p_new)
        self._set_ik_orientation(*self._current_orientation_deg())
        self._pending_vel = self._vel()
        self.lbl_ik_warn.configure(text='Calculando IK...',
                                   text_color=COL['warn'])

        def _cb(ok, q_rad, msg):
            if ok:
                self._pending_ik_q   = q_rad
                self._pending_ik_msg = (
                    f'Movido {step*1000:.0f} mm (orientacion fija)', COL['ok'])
                self._node.publicar_preview(q_rad)
            else:
                self._pending_ik_q   = None
                self._pending_ik_msg = (f'No alcanzable: {msg}', COL['err'])

        self._node.pedir_ik_pose(p_new, R, self.var_elbow.get(), _cb)

    # ── Callbacks de acciones ─────────────────────────────────────

    def _toggle_conn(self):
        if self._node.conectado:
            self._node.desconectar()
        else:
            self._node.conectar()

    def _move_fk(self):
        q_rad = np.radians([s.get() for s in self._fk_sliders])
        self._node.publicar_joint_cmd(q_rad, self._vel())
        self._node.publicar_preview(q_rad)

    def _traj_steps(self) -> int:
        try:    return max(0, int(self.entry_traj_steps.get()))
        except ValueError: return 0

    def _traj_dt(self) -> float:
        try:    return max(0.02, min(2.0, float(self.entry_traj_dt.get())))
        except ValueError: return 0.10

    def _move_ik(self):
        try:
            x, y, z, beta_deg, q5_deg, q6_deg = [
                float(e.get()) for e in self._ik_ent]
        except ValueError:
            self.lbl_ik_warn.configure(text='Valores invalidos.',
                                       text_color=COL['err'])
            return

        self.btn_ik.configure(state='disabled')
        self._pending_vel = self._vel()
        n_steps = self._traj_steps()

        if n_steps > 0:
            # Modo trayectoria: usa CartesianTrajectory (tolerancias + feedback)
            pose = self._node.pose_actual
            if pose is None:
                self.lbl_ik_warn.configure(
                    text='Sin pose actual para iniciar trayectoria.',
                    text_color=COL['err'])
                self.btn_ik.configure(state='normal')
                return
            p0 = np.array([pose.pose.position.x,
                            pose.pose.position.y,
                            pose.pose.position.z])
            R0 = quaternion_to_matrix(pose.pose.orientation)
            p1 = np.array([x, y, z])
            q1t = math.atan2(y, x)
            R1 = (_rotz3(q1t) @ _roty3(math.radians(beta_deg))
                  @ _rotx3(math.radians(q5_deg)) @ _rotz3(math.radians(q6_deg)))

            try:
                path_tol = max(0.0, float(self.entry_path_tol.get()))
                goal_tol = max(0.0, float(self.entry_goal_tol.get()))
            except ValueError:
                path_tol, goal_tol = 0.0, 0.02

            self.lbl_ik_warn.configure(text='Enviando trayectoria...',
                                        text_color=COL['warn'])
            self.traj_progress.set(0)
            self.lbl_traj_state.configure(text='Iniciando...', text_color=COL['warn'])

            def _on_traj(ok, msg):
                self._pending_ik_msg = (
                    f'Trayectoria: {msg}' if ok else f'Error: {msg}',
                    COL['ok'] if ok else COL['err'])

            self._node.pedir_cartesian_trajectory(
                p0, R0, p1, R1,
                n_steps, self._traj_dt(),
                self._pending_vel, self.var_elbow.get(),
                path_tol, goal_tol, _on_traj)
        else:
            self.lbl_ik_warn.configure(text='Calculando IK...',
                                        text_color=COL['warn'])
            self._node.pedir_ik(
                x, y, z, beta_deg, q5_deg, q6_deg,
                elbow     = self.var_elbow.get(),
                vel_pct   = self._pending_vel,
                on_result = self._on_ik_result,
            )

    def _on_ik_result(self, success: bool, q_rad: list, mensaje: str):
        """Llamado desde el thread de ROS — guarda resultado para _loop_ui."""
        if success:
            self._pending_ik_q   = q_rad
            self._pending_ik_msg = ('Pose calculada — enviando...', COL['ok'])
            self._node.publicar_preview(q_rad)
        else:
            self._pending_ik_q   = None
            self._pending_ik_msg = (f'IK fallido: {mensaje}', COL['err'])

    def _home(self):
        q_zero = np.zeros(len(self._node.joint_order))
        self._node.publicar_joint_cmd(q_zero, self._vel())
        self._node.publicar_preview(q_zero)
        for i, sld in enumerate(self._fk_sliders):
            sld.set(0)
            self._fk_val_lbls[i].configure(text='0.0°')

    def _jog(self, joint_name: str, sign: int):
        idx = self._node.joint_order.index(joint_name)
        current_deg = np.degrees(self._node.q_actual[idx])
        target = (current_deg + sign * self._jog_step()) % 360.0
        self._node.jog(
            joint_name, target, self._jog_vel(),
            on_result=lambda ok, msg: self._set_jog_msg(
                msg if ok else f'Error: {msg}',
                COL['muted'] if ok else COL['err']))

    def _set_jog_msg(self, text: str, color: str):
        self._pending_jog_msg = (text, color)

    def _rescue_pulse(self, direction: int):
        self._node.rescue_pulse(
            self.var_rescue_joint.get(), direction, self._rescue_vel(),
            on_result=lambda ok, msg: self._set_rescue_msg(
                msg if ok else f'Error: {msg}',
                COL['ok'] if ok else COL['err']))

    def _set_rescue_msg(self, text: str, color: str):
        self._pending_rescue_msg = (text, color)

    # ── Loop UI (120 ms) ──────────────────────────────────────────

    def _loop_ui(self):
        # ── Mostrar/ocultar segun /arm_active (lo manda el dashboard) ──
        want = self._node._req_active
        if want and not self._is_shown:
            self.deiconify(); self.lift(); self._is_shown = True
            try:    self.tabs.set('Teleop')   # al entrar al brazo, Teleop primero
            except Exception: pass
        elif not want and self._is_shown:
            self.withdraw(); self._is_shown = False

        # Si esta oculta, no gastamos en UI ni en el 3D: reagendar y salir.
        if not self._is_shown:
            self._node._submode_step = 0   # descartar pasos L1/R1 mientras oculta
            self.after(150, self._loop_ui)
            return

        # ── L1/R1 → ciclar submodos (tabs), EXCEPTO en 'Jog' donde L1/R1
        # jogean el servo Base y no deben ademas cambiar de pestana ──
        if self.tabs.get() != 'Jog':
            step = self._node._submode_step
            if step:
                self._node._submode_step = 0
                try:
                    cur = self.tabs.get()
                    i = (self._tab_order.index(cur) + step) % len(self._tab_order)
                    self.tabs.set(self._tab_order[i])
                except Exception:
                    pass

        # ── Camara frontal + QR/senal (lado derecho, siempre visible) ──
        self._refresh_camera()

        joint_order = self._node.joint_order
        ax_st = self._node.ax_status
        ex_st = self._node.ex_status
        # Basta un bus conectado para operar (hardware parcial en pruebas)
        con   = ax_st['conectado'] or ex_st['conectado']
        eme   = ax_st['emergencia'] or ex_st['emergencia']
        cal   = ax_st['modo_calib'] or ex_st['modo_calib']
        listo = con and not eme and not cal

        # Estado por driver
        self.lbl_ax.configure(
            text='AX-12A: conectado' if ax_st['conectado'] else 'AX-12A: desconectado',
            fg_color='#14532d' if ax_st['conectado'] else '#333',
            text_color=COL['ok'] if ax_st['conectado'] else COL['muted'])
        self.lbl_ex.configure(
            text='EX-106: conectado' if ex_st['conectado'] else 'EX-106: desconectado',
            fg_color='#14532d' if ex_st['conectado'] else '#333',
            text_color=COL['ok'] if ex_st['conectado'] else COL['muted'])

        msgs = [m for m in (ax_st['mensaje'], ex_st['mensaje']) if m]
        self.lbl_driver_msg.configure(text=' | '.join(dict.fromkeys(msgs)))

        # Boton conectar / desconectar
        self.btn_conn.configure(
            text='Desconectar' if con else 'Conectar',
            fg_color='#922b21' if con else ['#3a7ebf', '#1f538d'])

        # Botones segun estado
        self.btn_estop.configure(      state='normal' if con   else 'disabled')
        self.btn_resume.configure(     state='normal' if eme   else 'disabled')
        self.btn_home.configure(       state='normal' if listo else 'disabled')
        self.btn_fk.configure(         state='normal' if listo else 'disabled')
        self.btn_calib_start.configure(state='normal' if (con and not eme and not cal)
                                                       else 'disabled')
        self.btn_calib_ok.configure(   state='normal' if cal   else 'disabled')
        self.btn_rescue_neg.configure( state='normal' if con   else 'disabled')
        self.btn_rescue_pos.configure( state='normal' if con   else 'disabled')
        self.btn_reset_alerts.configure(state='normal' if con  else 'disabled')
        for btn in self._jog_buttons:
            btn.configure(state='normal' if cal else 'disabled')

        # IK: solo si listo y no hay IK en curso
        ik_en_curso = self.lbl_ik_warn.cget('text') == 'Calculando IK...'
        pose = self._node.pose_actual
        self.btn_ik.configure(
            state='normal' if (listo and not ik_en_curso) else 'disabled')
        cart_state = ('normal' if (listo and not ik_en_curso and pose is not None)
                      else 'disabled')
        for btn in self._cart_delta_buttons:
            btn.configure(state=cart_state)
        self.btn_use_pose.configure(
            state='normal' if pose is not None else 'disabled')

        # Teleop: botones cartesianos activos si listo y hay pose
        tp_state = 'normal' if (listo and pose is not None) else 'disabled'
        for btn in (self._tp_trans_btns + self._tp_orient_btns):
            btn.configure(state=tp_state)
        self.btn_tp_sync.configure(
            state='normal' if pose is not None else 'disabled')

        # Aviso de reinicio de buses (flecha derecha del D-pad) — ~2.4 s
        if self._node._bus_reset_flag:
            self._node._bus_reset_flag = False
            self._bus_reset_ticks = 20

        # Banner de estado global (el paro de emergencia tiene prioridad)
        if eme:
            self.lbl_global.configure(text='PARO DE EMERGENCIA',
                                      text_color=COL['err'])
        elif self._bus_reset_ticks > 0:
            self._bus_reset_ticks -= 1
            self.lbl_global.configure(text='↻ REINICIANDO BUSES (AX + EX)...',
                                      text_color=COL['accent'])
        elif cal:
            self.lbl_global.configure(text='CALIBRACION ACTIVA',
                                      text_color=COL['warn'])
        else:
            self.lbl_global.configure(text='')

        # Angulos actuales
        q = self._node.q_actual
        for i, lbl in enumerate(self._lbl_angs):
            lbl.configure(text=f'actual: {np.degrees(q[i]):.2f}°')
        for i, lbl in enumerate(self._lbl_jog_angs):
            lbl.configure(text=f'{np.degrees(q[i]):.2f}°')
        for i, lbl in enumerate(self._lbl_gp_jog_angs):
            lbl.configure(text=f'{np.degrees(q[i]):.2f}°')

        # Pose del efector
        if pose is not None:
            p = pose.pose.position
            roll, pitch, yaw = quaternion_to_rpy(pose.pose.orientation)
            self.lbl_pose_xyz.configure(
                text=f'x {p.x:+.3f} | y {p.y:+.3f} | z {p.z:+.3f} m')
            self.lbl_pose_rpy.configure(
                text=f'R {roll:+.1f}° | P {pitch:+.1f}° | Y {yaw:+.1f}°')

        # Mensajes pendientes del thread ROS
        if self._pending_jog_msg is not None:
            text, color = self._pending_jog_msg
            self.lbl_jog_msg.configure(text=text, text_color=color)
            self._pending_jog_msg = None

        if self._pending_rescue_msg is not None:
            text, color = self._pending_rescue_msg
            self.lbl_rescue_msg.configure(text=text, text_color=color)
            self._pending_rescue_msg = None

        if self._pending_teleop_msg is not None:
            text, color = self._pending_teleop_msg
            self.lbl_tp_msg.configure(text=text, text_color=color)
            self._pending_teleop_msg = None
            self._joy_ik_inflight = False   # llego resultado: liberar para el siguiente paso

        if self._pending_teleop_q is not None:
            if self._pending_teleop_commit is not None:
                self._tp_p, self._tp_R = self._pending_teleop_commit
                self._pending_teleop_commit = None
                self._update_tp_label()
            self._node.publicar_joint_cmd(
                np.array(self._pending_teleop_q), self._vel())
            self._node.publicar_preview(np.array(self._pending_teleop_q))
            self._pending_teleop_q = None

        # ── Teleop por joystick (solo en la pestana Teleop) ──
        if self.tabs.get() == 'Teleop':
            self._joystick_teleop_step()

        # ── Jog manual por servo con el mando (solo en la pestana Jog) ──
        if self.tabs.get() == 'Jog':
            self._joystick_jog_step()
        elif self._jog_gp_indicator_on:
            self._set_gp_jog_indicator(False)

        if self._pending_ik_msg is not None:
            texto, color = self._pending_ik_msg
            self.lbl_ik_warn.configure(text=texto, text_color=color)
            self._pending_ik_msg = None

        if self._pending_ik_q is not None:
            self._node.publicar_joint_cmd(
                np.array(self._pending_ik_q), self._pending_vel)
            self._pending_ik_q = None
            self.after(3000, lambda: self.lbl_ik_warn.configure(text=''))

        # ── Estado trayectoria cartesiana (feedback /cartesian/state) ──
        cs = self._node.cartesian_state
        in_prog = cs['in_progress']
        if in_prog:
            self.traj_progress.set(cs['progress'])
            err_mm = cs['pos_error'] * 1000.0
            tol_ok = cs['within_path_tol']
            wp_idx = cs['waypoint_idx']
            wp_tot = cs['total_waypoints']
            wp_txt = f'  wp {wp_idx+1}/{wp_tot}' if wp_tot > 0 else ''
            col = COL['ok'] if tol_ok else COL['err']
            self.lbl_traj_state.configure(
                text=f'{cs["progress"]*100:.0f}%  err {err_mm:.1f}mm'
                     + ('  [OK]' if tol_ok else '  [FUERA TOL]') + wp_txt,
                text_color=col)
            self.btn_ik.configure(state='disabled')
        elif self._last_cart_in_prog and not in_prog:
            # Flanco bajada: trayectoria terminó → re-habilitar btn y marcar 100%
            self.traj_progress.set(1.0)
            self.lbl_traj_state.configure(
                text=f'Completada  err {cs["pos_error"]*1000:.1f}mm',
                text_color=COL['ok'])
            self.btn_ik.configure(state='normal')
        self._last_cart_in_prog = in_prog

        # Estado de servos: sondear ~cada 1.4 s solo si la pestaña esta activa
        if con and self.tabs.get() == 'Estado':
            self._status_poll_count += 1
            if self._status_poll_count >= 12:
                self._status_poll_count = 0
                self._refresh_status()
        else:
            self._status_poll_count = 12   # refresca al entrar a la pestaña

        if self._pending_status:
            for name, fila in self._pending_status.items():
                row = self._estado_rows.get(name)
                if row is None:
                    continue
                if not fila['responde']:
                    row['torque'].configure(text='—', text_color=COL['muted'])
                    row['temp'].configure(text='—', text_color=COL['muted'])
                    row['volt'].configure(text='—', text_color=COL['muted'])
                    row['alerta'].configure(text='sin respuesta',
                                            text_color=COL['muted'])
                    continue
                row['torque'].configure(
                    text='ON' if fila['torque_on'] else 'OFF',
                    text_color=COL['ok'] if fila['torque_on'] else COL['err'])
                temp = fila['temp']
                row['temp'].configure(
                    text=f'{temp}°',
                    text_color=COL['err'] if temp >= 70 else COL['accent'])
                row['volt'].configure(text=f"{fila['volt']:.1f}V",
                                      text_color=COL['accent'])
                alerta = fila['alerta']
                row['alerta'].configure(
                    text=alerta if alerta else 'OK',
                    text_color=COL['err'] if alerta else COL['ok'])
            self._pending_status = {}

        # Visualizacion 3D
        self._redraw_arm()

        self.after(120, self._loop_ui)

    def on_closing(self):
        self._node.desconectar()
        self.destroy()
        # Si esta GUI fue lanzada desde el dashboard (via `ros2 launch ...` con
        # start_new_session), baja TODO el grupo de proceso del arm_station
        # (cinematica + cartesian + gui) para que el dashboard reaparezca.
        # En ejecucion standalone (sim_only) tambien cierra el launch, que es
        # el comportamiento esperado al cerrar la ventana.
        try:
            os.killpg(os.getpgid(os.getpid()), signal.SIGINT)
        except Exception:
            pass


# ─────────────────────────────────────────────────────────────
#  Entry point
# ─────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    node = GUINode()

    ros_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    app = App(node)
    # La X de la ventana del brazo NO mata el proceso (el dashboard lo precarga
    # y es dueno del ciclo de vida): solo pide volver al dashboard (se oculta).
    app.protocol('WM_DELETE_WINDOW', node.pedir_dashboard)
    app.mainloop()

    # Detener primero el contexto para que el thread de spin salga limpio
    rclpy.try_shutdown()
    ros_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
