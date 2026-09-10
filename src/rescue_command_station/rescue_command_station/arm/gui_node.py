"""Arm station GUI: the delivered Python model and original Canvas.

Joint feedback and active TCP pose arrive from ROS. Preview is local drawing
data, never a deferred command. All Cartesian moves use the shared planner;
manual joint commands are admitted by cartesian_node.
"""

import math
import threading
import time
import tkinter as tk
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import JointState, Joy, CompressedImage
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, SetBool

from std_msgs.msg import Bool, Float64, String
from rescue_interfaces.msg import ArmStatus, CartesianState
from rescue_interfaces.srv import (ComputeIKPose, ServoCommand, ServoStatus,
                                CartesianGoto, CartesianTrajectory)
from rescue_interfaces.msg import CartesianWaypoint

from rescue_command_station.control import config as cfg
from rescue_command_station.vision.qr_detector import QrDetector
from rescue_command_station.vision.ros_image import compressed_msg_to_numpy
from rescue_command_station.vision.tk_image import bgr_frame_to_png_data

import customtkinter as ctk

from .web_view import ArmCanvas, scene_payload
from .kinematics import rotx, roty, rotz, rot_to_rpy
from .configuration import settings, joint_drivers, configured_arm

ctk.set_appearance_mode('Dark')
ctk.set_default_color_theme('blue')

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
    return rot_to_rpy(quaternion_to_matrix(q))


def quaternion_to_matrix(q) -> np.ndarray:
    """Matriz R que transforma vectores del frame herramienta al frame base."""
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    n = math.sqrt(x*x + y*y + z*z + w*w)
    if not np.isfinite(n) or n < 1e-9:
        raise ValueError("Quaternion invalido")
    x, y, z, w = x/n, y/n, z/n, w/n
    return np.array([
        [1.0 - 2.0*(y*y + z*z), 2.0*(x*y - z*w),       2.0*(x*z + y*w)],
        [2.0*(x*y + z*w),       1.0 - 2.0*(x*x + z*z), 2.0*(y*z - x*w)],
        [2.0*(x*z - y*w),       2.0*(y*z + x*w),       1.0 - 2.0*(x*x + y*y)],
    ], dtype=float)


# UI increments reuse the rotations in the supplied model.
def _rotx3(a): return rotx(a)[:3, :3]
def _roty3(a): return roty(a)[:3, :3]
def _rotz3(a): return rotz(a)[:3, :3]


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
        self.arm_model, self.model_sign = configured_arm(self)

        self._joint_order, self._joint_servo = self._leer_config_joints()
        self.declare_parameter('managed_by_dashboard', False)
        self.managed_by_dashboard = bool(self.get_parameter('managed_by_dashboard').value)

        self._q_actual    = np.zeros(len(self._joint_order))
        self._joint_seen = np.zeros(len(self._joint_order))
        self._preview_q = None
        self._pose_actual = None
        self._pose_seen = 0.0
        self._cartesian_seen = 0.0
        self._bus_seen = {"ax": 0.0, "ex": 0.0}
        self._motion_pending = False
        self._maintenance = False
        self._maintenance_held = False
        self._maintenance_pending = False
        self._joint_map   = {name: i for i, name in enumerate(self._joint_order)}

        self._ax_status = {'conectado': False, 'emergencia': False,
                           'modo_calib': False, 'mensaje': ''}
        self._ex_status = {'conectado': False, 'emergencia': False,
                           'modo_calib': False, 'mensaje': ''}

        self._lock = threading.Lock()

        # ── Publishers ─────────────────────────────────────────────
        self._pub_manual_cmd = self.create_publisher(
            JointState, '/arm/manual_joint_cmd', 10)

        # ── Subscribers ────────────────────────────────────────────
        self.create_subscription(
            JointState,  '/joint_states',          self._cb_joint_states, 10)
        self.create_subscription(
            PoseStamped, '/end_effector_pose',     self._cb_pose,         10)
        self.create_subscription(
            ArmStatus,      '/ax12a/status',        self._cb_ax_status,    10)
        self.create_subscription(
            ArmStatus,      '/ex106/status',        self._cb_ex_status,    10)
        self.create_subscription(
            CartesianState, '/cartesian/state',     self._cb_cart_state,   10)

        self._cartesian_state = {
            'maintenance': False, 'feedback_valid': False,
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
            'compute_ik_pose': self.create_client(ComputeIKPose,  '/compute_ik_pose'),
            'cartesian_cancel': self.create_client(Trigger, '/cartesian/cancel'),
            'cartesian_goto'  : self.create_client(CartesianGoto,       '/cartesian/goto'),
            'cartesian_traj'  : self.create_client(CartesianTrajectory, '/cartesian/trajectory'),
        }

        self._maintenance_cli = self.create_client(SetBool, '/cartesian/maintenance')

        # ── Integracion con el dashboard ───────────────────────────
        #   /arm_active: el dashboard dice si el brazo esta al frente (mostrar/ocultar)
        #   /joy: L1/R1 alternan submodos (tabs) cuando el brazo esta al frente
        #   /gui_switch_request: pide volver al dashboard (boton "volver")
        self._req_active = not self.managed_by_dashboard        # ultimo /arm_active recibido
        self._submode_step = 0          # pasos L1/R1 pendientes de aplicar
        self._prev_buttons: list = []
        self._joy_stamp = 0.0
        self._joy_axes: list = []       # ultimos ejes del mando (teleop cartesiano)
        self._bus_reset_flag = False    # el dashboard pidio reiniciar los buses
        self.create_subscription(Bool, '/arm_active', self._cb_arm_active, 10)
        self.create_subscription(Joy, '/joy', self._cb_joy, 10)
        self.create_subscription(Bool, '/bus_reset', self._cb_bus_reset, 10)
        self._pub_switch = self.create_publisher(Bool, '/gui_switch_request', 10)
        self._pub_arm_active = None
        if not self.managed_by_dashboard:
            # Match the dashboard's base-teleop inhibit when running standalone.
            self._pub_arm_active = self.create_publisher(Bool, '/arm_active', 10)
            self._active_timer = self.create_timer(
                0.2, lambda: self._pub_arm_active.publish(Bool(data=True)))

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
        if not self.managed_by_dashboard:
            return
        if self._req_active and not msg.data:
            self._joy_axes = []
            self._joy_stamp = 0.0
            self._call_trigger('cartesian_cancel')
        self._req_active = bool(msg.data)

    def _cb_bus_reset(self, msg):
        if msg.data:
            self._bus_reset_flag = True

    def _cb_joy(self, msg):
        self._joy_stamp = time.monotonic()
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
        return list(settings(self)['joint_order']), joint_drivers(self)

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
                if name in self._joint_map and np.isfinite(pos):
                    i = self._joint_map[name]
                    self._q_actual[i] = pos
                    self._joint_seen[i] = time.monotonic()

    def _cb_pose(self, msg: PoseStamped):
        with self._lock:
            self._pose_actual = msg
            self._pose_seen = time.monotonic()





    def _cb_ax_status(self, msg: ArmStatus):
        with self._lock:
            self._bus_seen["ax"] = time.monotonic()
            self._ax_status = {
                'conectado':  msg.conectado,
                'emergencia': msg.emergencia,
                'modo_calib': msg.modo_calib,
                'mensaje':    msg.mensaje,
            }

    def _cb_ex_status(self, msg: ArmStatus):
        with self._lock:
            self._bus_seen["ex"] = time.monotonic()
            self._ex_status = {
                'conectado':  msg.conectado,
                'emergencia': msg.emergencia,
                'modo_calib': msg.modo_calib,
                'mensaje':    msg.mensaje,
            }

    def _cb_cart_state(self, msg: CartesianState):
        with self._lock:
            self._cartesian_seen = time.monotonic()
            self._cartesian_state = {
                'maintenance': msg.maintenance,
                'feedback_valid': msg.feedback_valid,
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
            status = dict(self._ax_status)
            if time.monotonic() - self._bus_seen['ax'] > 2.0:
                status.update(conectado=False, mensaje='Sin estado reciente del bus ax')
            return status

    @property
    def ex_status(self) -> dict:
        with self._lock:
            status = dict(self._ex_status)
            if time.monotonic() - self._bus_seen['ex'] > 2.0:
                status.update(conectado=False, mensaje='Sin estado reciente del bus ex')
            return status

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
            fresh = time.monotonic() - self._pose_seen < 1.0 and np.all(time.monotonic() - self._joint_seen < 1.0)
            return self._pose_actual if fresh else None

    @property
    def motion_busy(self):
        with self._lock:
            return self._motion_pending or self._cartesian_state['in_progress']

    @property
    def motion_ready(self):
        with self._lock:
            now = time.monotonic()
            return (not self._maintenance and not self._cartesian_state['maintenance']
                    and self._cartesian_state['feedback_valid']
                    and not self._motion_pending and not self._cartesian_state['in_progress']
                    and now - self._cartesian_seen < 1.0
                    and np.all(now - self._joint_seen < 1.0)
                    and all(now - self._bus_seen[d] < 2.0 for d in ('ax', 'ex'))
                    and all(s['conectado'] and not s['emergencia'] and not s['modo_calib']
                            for s in (self._ax_status, self._ex_status)))



    @property
    def maintenance_ready(self):
        with self._lock:
            now = time.monotonic()
            return (self._maintenance_held and not self._maintenance_pending
                    and not self._motion_pending and not self._cartesian_state['in_progress']
                    and now - self._cartesian_seen < 1.0
                    and all(now - self._bus_seen[d] < 2.0 for d in ('ax', 'ex'))
                    and all(v['conectado'] and v['modo_calib'] and not v['emergencia']
                            for v in (self._ax_status, self._ex_status)))

    # ── Publicar comandos de movimiento ───────────────────────────

    def publicar_joint_cmd(self, q_rad: np.ndarray, vel_pct: float = 30.0):
        """Submit a complete driver-angle command to the station arbiter."""
        q_rad=np.asarray(q_rad,float)
        if q_rad.shape!=(6,) or not np.all(np.isfinite(q_rad)):
            self.get_logger().error('Comando articular invalido'); return
        model=q_rad*self.model_sign
        if np.any(model<self.arm_model.limits[:,0]) or np.any(model>self.arm_model.limits[:,1]):
            self.get_logger().error('Comando rechazado: fuera de limites, sin recorte'); return
        if not self.motion_ready:
            self.get_logger().error('Comando rechazado: control ocupado o feedback/buses no disponibles'); return
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(self._joint_order)
        msg.position = q_rad.tolist()
        msg.velocity = [float(vel_pct)] * 6
        self._pub_manual_cmd.publish(msg)

    def mostrar_preview(self, q_rad):
        """Local view only; it cannot publish a movement command."""
        q_rad = np.asarray(q_rad, float)
        if q_rad.shape == (6,) and np.all(np.isfinite(q_rad)):
            with self._lock:
                self._preview_q = q_rad.copy()

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
        self._call_trigger('cartesian_cancel')
        self._call_trigger('ax_disconnect')
        self._call_trigger('ex_disconnect')

    def emergencia_stop(self):
        self._call_trigger('cartesian_cancel')
        self._call_trigger('ax_estop')
        self._call_trigger('ex_estop')

    def reanudar(self):
        self._call_trigger('ax_resume')
        self._call_trigger('ex_resume')

    def _set_maintenance(self, enabled, on_success):
        cli = self._maintenance_cli
        if not cli.service_is_ready():
            self.get_logger().error('/cartesian/maintenance no disponible; control bloqueado')
            self._maintenance_pending = False
            return
        req = SetBool.Request(); req.data = enabled
        def done(f):
            res = _future_result(f)
            self._maintenance_pending = False
            if res is not None and res.success:
                self._maintenance = enabled
                self._maintenance_held = enabled
                on_success()
            else:
                self.get_logger().error('Mantenimiento no confirmado; control bloqueado')
        try:
            cli.call_async(req).add_done_callback(done)
        except Exception as exc:
            self._maintenance_pending = False
            self.get_logger().error(str(exc))

    def calib_start(self):
        if self.motion_busy or self._maintenance_pending:
            return
        self._maintenance = True
        self._maintenance_pending = True
        def start():
            self._call_trigger('ax_cal_start')
            self._call_trigger('ex_cal_start')
        self._set_maintenance(True, start)

    def calib_confirm(self):
        if not self.maintenance_ready or self._maintenance_pending:
            return
        clients = [self._cli[k] for k in ('ax_cal_ok', 'ex_cal_ok')]
        if not all(c.service_is_ready() for c in clients):
            return
        self._maintenance_pending = True
        replies = []; gate = threading.Lock()
        def done(f):
            res = _future_result(f)
            with gate:
                replies.append(res is not None and res.success)
                if len(replies) != 2:
                    return
            if all(replies):
                self._set_maintenance(False, lambda: None)
            else:
                self._maintenance_pending = False
                self.get_logger().error('Calibracion parcial; repetir inicio y confirmar ambos buses')
        for cli in clients:
            try:
                cli.call_async(Trigger.Request()).add_done_callback(done)
            except Exception:
                done(None)

    # ── ServoCommand (jog / rescue) ───────────────────────────────

    def _call_servo_cmd(self, key: str, sid: int, target_deg: float,
                        vel_pct: float, on_result):
        if not self.maintenance_ready or not np.all(np.isfinite([target_deg, vel_pct])):
            if on_result:
                on_result(False, 'Requiere mantenimiento y calibracion de ambos buses, sin emergencia')
            return
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
        i = self._joint_map[joint_name]
        if time.monotonic() - self._joint_seen[i] > 1.0:
            if on_result:
                on_result(False, 'Jog requiere un angulo medido reciente')
            return
        driver, sid = self._joint_servo[joint_name]
        self._call_servo_cmd(f'{driver}_jog', sid, target_deg, vel_pct, on_result)

    def rescue_pulse(self, joint_name: str, direction: int, vel_pct: float = 15.0,
                     on_result=None):
        driver, sid = self._joint_servo[joint_name]
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

        try:
            self._motion_request(cli, req, _done,
                                 lambda ok, text: on_result(ok, [], text), cancel_on_timeout=False)
        except Exception as exc:
            on_result(False, [], str(exc))

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

        try:
            self._motion_request(cli, req, _done, on_result)
        except Exception as exc:
            on_result(False, str(exc))

    def _motion_request(self, cli, req, on_done, on_result, cancel_on_timeout=True):
        """Bound client wait; a timed-out plan must be cancelled, never retried silently."""
        with self._lock:
            if self._motion_pending:
                raise ValueError('Hay una peticion pendiente; espere o cancele')
            self._motion_pending = True
        try:
            future = cli.call_async(req)
        except Exception:
            with self._lock:
                self._motion_pending = False
            raise
        gate = threading.Lock()
        finished = False
        deadline = time.monotonic() + 60.0
        timer = None

        def finish(f=None):
            nonlocal finished
            with gate:
                if finished:
                    return
                finished = True
            with self._lock:
                self._motion_pending = False
            if timer is not None:
                self.destroy_timer(timer)
            if f is not None:
                on_done(f)
            else:
                if cancel_on_timeout:
                    self._call_trigger('cartesian_cancel')
                future.cancel()
                on_result(False, 'Sin respuesta en 60 s; verifique el estado antes de mover.')

        timer = self.create_timer(0.2, lambda: finish() if time.monotonic() >= deadline else None)
        future.add_done_callback(finish)

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

        try:
            self._motion_request(cli, req, _done, on_result)
        except Exception as exc:
            on_result(False, str(exc))


# ─────────────────────────────────────────────────────────────
#  Ventana principal
# ─────────────────────────────────────────────────────────────

class App(ctk.CTk):

    def __init__(self, node: GUINode):
        super().__init__()
        self._node = node

        # Resultados pendientes del thread ROS para aplicar en _loop_ui
        self._pending_ik_msg    : tuple | None = None   # (texto, color)
        self._pending_vel       : float        = 30.0
        self._pending_jog_msg   : tuple | None = None
        self._pending_rescue_msg: tuple | None = None
        self._pending_status    : dict        = {}
        self._pending_lock = threading.Lock()
        self._status_poll_count = 0
        self._cart_delta_buttons: list[ctk.CTkButton] = []

        # Teleoperacion cartesiana: pose objetivo acumulada (R 3x3, p 3)
        self._tp_R              : np.ndarray | None = None
        self._tp_p              : np.ndarray | None = None
        self._stab_R            : np.ndarray | None = None   # R fija para estabilizacion
        self._pending_teleop_msg: tuple | None = None
        self._pending_teleop_commit: tuple | None = None
        self._bus_reset_ticks    = 0       # ticks restantes del banner de reinicio de buses
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
        if not self._node.managed_by_dashboard:
            self.btn_back.grid_remove()
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
        self._tab_order = ['Mover', 'Teleop', 'IK', 'Calibrar',
                           'Rescate', 'Estado']
        for name in self._tab_order:
            self.tabs.add(name)

        self._build_tab_fk(self.tabs.tab('Mover'))
        self._build_tab_teleop(self.tabs.tab('Teleop'))
        self._build_tab_ik(self.tabs.tab('IK'))
        self._build_tab_calib(self.tabs.tab('Calibrar'))
        self._build_tab_rescue(self.tabs.tab('Rescate'))
        self._build_tab_estado(self.tabs.tab('Estado'))
        self.tabs.add('Simulador 6R')
        ttk_text = 'Simulador sin hardware: misma cinemática y planificador que ROS.\nGeometría paramétrica, TCP herramienta/cámara, marcos locales y CSV.'
        ctk.CTkLabel(self.tabs.tab('Simulador 6R'), text=ttk_text,
                    wraplength=300).pack(padx=10,pady=15)
        ctk.CTkButton(self.tabs.tab('Simulador 6R'), text='Abrir simulador',
                      command=self._open_simulator).pack(pady=10)

    def _open_simulator(self):
        import subprocess
        import sys
        subprocess.Popen([sys.executable, '-m', 'rescue_command_station.arm.simulator'])

    def _refresh_camera(self):
        """Renderiza la ultima imagen de la camara frontal + QR/senal."""
        n = self._node
        if n._cam_version != self._drawn_cam_version:
            with n._cam_lock:
                frame = None if n._cam_frame is None else n._cam_frame.copy()
                self._drawn_cam_version = n._cam_version
            if frame is not None:
                w = self.cam_label.winfo_width()
                h = self.cam_label.winfo_height()
                png = bgr_frame_to_png_data(frame, max_width=w if w > 10 else 520,
                                            max_height=h if h > 10 else 280)
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

            driver_bounds = np.sort(self._node.arm_model.limits[i] * self._node.model_sign[i])
            sld = ctk.CTkSlider(fr, from_=float(np.degrees(driver_bounds[0])),
                                to=float(np.degrees(driver_bounds[1])),
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
        ctk.CTkButton(tab, text='Cancelar trayectoria (no sustituye emergencia)',
                      command=lambda: self._node._call_trigger('cartesian_cancel')).pack(fill='x', padx=8, pady=4)
        frame = ctk.CTkScrollableFrame(tab, fg_color='transparent')
        frame.pack(fill='both', expand=True)

        ctk.CTkLabel(frame, text='Teleoperacion del TCP activo',
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
            fr_stab, text='Fijar orientación del TCP activo',
            variable=self.var_stab, command=self._teleop_toggle_stab)
        self.chk_stab.pack(side='left')
        self.lbl_stab_estado = ctk.CTkLabel(
            fr_stab, text='', font=('Roboto', 11), text_color=COL['ok'])
        self.lbl_stab_estado.pack(side='left', padx=8)

        # Botones de translacion de la PUNTA (efector final) en linea recta
        self._tp_frame_trans = ctk.CTkFrame(frame, fg_color='transparent')

        # Marco de referencia: Mundo (base) = lineas rectas X/Y/Z fijas;
        # Camara = ejes del montaje de camara, distintos de la herramienta.
        self.var_tp_frame = ctk.StringVar(value='Herramienta')
        fr_sel = ctk.CTkFrame(self._tp_frame_trans, fg_color='transparent')
        fr_sel.pack(fill='x', pady=(0, 4))
        ctk.CTkLabel(fr_sel, text='Ejes:').pack(side='left', padx=(2, 4))
        ctk.CTkSegmentedButton(
            fr_sel, values=['Mundo', 'Herramienta', 'Camara'],
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
            frame, from_=0, to=100, number_of_steps=20,
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
            roll, pitch, yaw = rot_to_rpy(R)
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
        self._tp_measured_R = quaternion_to_matrix(pose.pose.orientation)
        # Con estabilizacion activa, no se toca la R bloqueada
        if self.var_stab.get() and self._stab_R is not None:
            self._tp_R = self._stab_R.copy()
        else:
            self._tp_R = quaternion_to_matrix(pose.pose.orientation)
        self._update_tp_label()
        self.lbl_tp_msg.configure(text='Pose objetivo sincronizada.',
                                  text_color=COL['ok'])
        return True

    def _teleop_ensure_seed(self) -> bool:
        if not self._node.motion_ready:
            return False
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
        frame={'Mundo':'base','Herramienta':'tool','Camara':'camera'}[self.var_tp_frame.get()]
        direccion = self._node.arm_model.relative_basis(self._tp_measured_R,frame)[:,axis]
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
        """Valida el recorrido completo antes de confirmar el objetivo solicitado."""
        # Con estabilizacion activa, la orientacion siempre es la R bloqueada.
        # Las rotaciones manuales actualizan _stab_R para poder reorientar
        # intencionalmente mientras se mantiene la estabilizacion.
        if self.var_stab.get() and self._stab_R is not None:
            R_send = self._stab_R
            if not np.allclose(R_new, self._tp_R if self._tp_R is not None else R_new):
                R_send = R_new
        else:
            R_send = R_new
        elbow = ''  # Rama continua desde feedback
        self.lbl_tp_msg.configure(text='Validando trayectoria...', text_color=COL['warn'])

        def _cb(ok, msg):
            if ok:
                self._pending_teleop_commit = (p_new, R_send)
                self._pending_teleop_msg    = ('Trayectoria validada y aceptada', COL['ok'])
            else:
                self._pending_teleop_msg = (f'No alcanzable: {msg}', COL['err'])

        self._node.pedir_cartesian_goto(p_new, R_send, 30, 0.02, self._vel(), elbow, _cb)

    def _joystick_teleop_step(self):
        """Teleop cartesiano con el mando (pestana Teleop, brazo al frente).
        SIEMPRE en ejes de la CAMARA (locales a la punta) y combina traslacion
        + orientacion en una sola IK:
          - stick IZQUIERDO → traslacion X / Y
          - gatillos L2/R2  → traslacion Z (R2 acerca/aleja segun signo)
          - stick DERECHO   → rotacion (yaw / pitch)
        Mientras mantengas el stick, se va desplazando (1 paso por ciclo).
        Manda 1 sola peticion IK a la vez."""
        if time.monotonic() - self._node._joy_stamp > 0.7:
            self._set_joy_indicator(False)
            return
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

        # The service callback releases the request; long preflight is still active.
        if self._node.motion_busy:
            return
        if not self._teleop_ensure_seed():
            return

        # Traslacion en ejes de la CAMARA (columnas de R = ejes locales de la punta)
        lin = self._tp_lin_step()
        frame={'Mundo':'base','Herramienta':'tool','Camara':'camera'}[self.var_tp_frame.get()]
        basis = self._node.arm_model.relative_basis(self._tp_measured_R,frame)
        # stick Y suele venir invertido (arriba = -1)
        dp = lin * (tx * basis[:, 0] - ty * basis[:, 1] + tz * basis[:, 2])
        p_new = self._tp_p + dp

        # Orientacion: incremento en el frame local de la punta
        ang = math.radians(self._tp_ang_step())
        R_new = self._tp_R @ _rotz3(ryaw * ang) @ _roty3(-rpitch * ang)

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

    # ── Tab IK ────────────────────────────────────────────────────

    def _build_tab_ik(self, tab):
        frame = ctk.CTkScrollableFrame(tab, fg_color='transparent')
        frame.pack(fill='both', expand=True)

        ik_rows = [('X (m):', '0.30'), ('Y (m):', '0.0'), ('Z (m):', '0.20'),
                   ('Roll (°):', '0'), ('Pitch (°):', '0'), ('Yaw (°):', '0')]
        self._ik_ent: list[ctk.CTkEntry] = []
        for lbl_t, val in ik_rows:
            e = self._row_entry(frame, lbl_t, val, lw=80)
            self._ik_ent.append(e)

        ctk.CTkLabel(frame, text='TCP: ' + self._node.arm_model.p.control_frame +
                    ' · pose completa XYZ/RPY · rama continua desde feedback',
                    wraplength=340).pack(pady=4)

        step_row = ctk.CTkFrame(frame, fg_color='transparent')
        step_row.pack(fill='x', padx=4, pady=2)
        ctk.CTkLabel(step_row, text='Paso (m):').pack(side='left')
        self.entry_cart_step = ctk.CTkEntry(step_row, width=60)
        self.entry_cart_step.insert(0, '0.005')
        self.entry_cart_step.pack(side='left', padx=6)

        traj_row = ctk.CTkFrame(frame, fg_color='transparent')
        traj_row.pack(fill='x', padx=4, pady=2)
        ctk.CTkLabel(traj_row, text='Muestras base (>=1):').pack(side='left')
        self.entry_traj_steps = ctk.CTkEntry(traj_row, width=46)
        self.entry_traj_steps.insert(0, '30')
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

        self.btn_ik = ctk.CTkButton(frame, text='PLANIFICAR Y MOVER (pose)',
                                    state='disabled', command=self._move_ik)
        self.btn_ik.pack(fill='x', pady=6, padx=4)
        ctk.CTkButton(frame, text='Resolver IK: solo preview',
                      command=self._preview_ik).pack(fill='x', padx=4)
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
            frame, text='Mantenimiento bloquea los movimientos normales. Durante la calibracion, mueve cada joint con ◄ ► '
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
        with self._pending_lock:
            for fila in filas:
                self._pending_status[fila['nombre']] = fila

    # ── Columna derecha: visualizacion 3D embebida ─────────────────

    def _build_right_panel(self):
        right = ctk.CTkFrame(self)
        right.grid(row=0, column=1, sticky='nsew', padx=(4, 8), pady=8)
        right.grid_rowconfigure(0, weight=3)   # viz 3D
        right.grid_rowconfigure(1, weight=2)   # camara frontal
        right.grid_columnconfigure(0, weight=1)

        self._arm_canvas = ArmCanvas(right)
        self._arm_canvas.grid(row=0, column=0, sticky='nsew', padx=4, pady=4)

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
        with self._node._lock:
            q = self._node._q_actual.copy()
            seen = self._node._joint_seen.copy()
            preview = None if self._node._preview_q is None else self._node._preview_q.copy()
        fresh = bool(np.all(time.monotonic() - seen < 1.0))
        status = ('Feedback de seis articulaciones · naranja: preview IK' if fresh else
                  'SIN FEEDBACK COMPLETO RECIENTE · última pose / cero de referencia')
        signs = self._node.model_sign
        self._arm_canvas.submit(scene_payload(
            self._node.arm_model, q * signs,
            None if preview is None else preview * signs, status))

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
        model = q * self._node.model_sign
        if np.any(model < self._node.arm_model.limits[:, 0]) or np.any(model > self._node.arm_model.limits[:, 1]):
            self.lbl_global.configure(text='Feedback fuera de limites: revise calibracion')
            return
        for i, sld in enumerate(self._fk_sliders):
            deg = math.degrees(q[i])
            sld.set(deg)
            self._fk_val_lbls[i].configure(text=f'{deg:.1f}°')

    def _set_ik_xyz(self, x: float, y: float, z: float):
        for entry, value in zip(self._ik_ent[:3], [x, y, z]):
            entry.delete(0, 'end')
            entry.insert(0, f'{value:.8f}')

    def _set_ik_orientation(self, roll_deg, pitch_deg, yaw_deg):
        for entry, value in zip(self._ik_ent[3:6], [roll_deg, pitch_deg, yaw_deg]):
            entry.delete(0, 'end')
            entry.insert(0, f'{value:.7f}')



    def _copy_current_pose_to_ik(self):
        pose = self._node.pose_actual
        if pose is None:
            self.lbl_ik_warn.configure(text='Aun no hay pose actual.',
                                       text_color=COL['warn'])
            return
        p = pose.pose.position
        self._set_ik_xyz(p.x, p.y, p.z)
        self._set_ik_orientation(*quaternion_to_rpy(pose.pose.orientation))

    def _move_cart_delta(self, dx: int, dy: int, dz: int):
        """Validate the complete Cartesian segment before sending any point."""
        if not self._node.motion_ready:
            return
        pose = self._node.pose_actual
        if pose is None:
            self.lbl_ik_warn.configure(text='Sin pose reciente', text_color=COL['warn'])
            return
        p = pose.pose.position
        R = quaternion_to_matrix(pose.pose.orientation)
        p_new = np.array([p.x, p.y, p.z]) + np.array([dx, dy, dz]) * self._cart_step()
        self._set_ik_xyz(*p_new)
        self._set_ik_orientation(*rot_to_rpy(R))
        self._node.pedir_cartesian_goto(p_new, R, self._traj_steps(), self._traj_dt(),
                                       self._vel(), '', self._on_cartesian_result)

    # ── Callbacks de acciones ─────────────────────────────────────

    def _toggle_conn(self):
        if self._node.conectado:
            self._node.desconectar()
        else:
            self._node.conectar()

    def _move_fk(self):
        q_rad = np.radians([s.get() for s in self._fk_sliders])
        self._node.publicar_joint_cmd(q_rad, self._vel())
        self._node.mostrar_preview(q_rad)

    def _traj_steps(self) -> int:
        try: return max(1, min(3000, int(self.entry_traj_steps.get())))
        except ValueError: return 30

    def _traj_dt(self) -> float:
        try:    return max(0.02, min(2.0, float(self.entry_traj_dt.get())))
        except ValueError: return 0.10

    def _read_pose_target(self):
        values = np.array([float(e.get()) for e in self._ik_ent])
        if not np.all(np.isfinite(values)):
            raise ValueError('La pose debe ser finita')
        r, p, y = np.radians(values[3:])
        return values[:3], (rotz(y) @ roty(p) @ rotx(r))[:3, :3]

    def _on_cartesian_result(self, ok, message):
        self._pending_ik_msg = (message, COL['ok'] if ok else COL['err'])

    def _move_ik(self):
        if not self._node.motion_ready:
            return
        try:
            p1, R1 = self._read_pose_target()
            pose = self._node.pose_actual
            if pose is None:
                raise ValueError('Sin pose actual reciente')
            p = pose.pose.position
            p0 = np.array([p.x, p.y, p.z])
            R0 = quaternion_to_matrix(pose.pose.orientation)
            ptol, gtol = float(self.entry_path_tol.get()), float(self.entry_goal_tol.get())
            if not np.all(np.isfinite([ptol, gtol])) or min(ptol, gtol) < 0:
                raise ValueError('Tolerancias invalidas')
            self._node.pedir_cartesian_trajectory(
                p0, R0, p1, R1, self._traj_steps(), self._traj_dt(), self._vel(), '',
                ptol, gtol, self._on_cartesian_result)
        except (ValueError, TypeError) as exc:
            self.lbl_ik_warn.configure(text=str(exc), text_color=COL['err'])

    def _preview_ik(self):
        try:
            p, R = self._read_pose_target()
            self._node.pedir_ik_pose(p, R, '', self._on_ik_result)
        except (ValueError, TypeError) as exc:
            self.lbl_ik_warn.configure(text=str(exc), text_color=COL['err'])

    def _on_ik_result(self, success: bool, q_rad: list, mensaje: str):
        """Preview only: an IK response never queues or publishes a move."""
        if success:
            self._node.mostrar_preview(q_rad)
        self._pending_ik_msg = (('Preview calculado; no se ha movido el brazo' if success else mensaje),
                                COL['ok'] if success else COL['err'])

    def _home(self):
        q_zero = np.zeros(len(self._node.joint_order))
        self._node.publicar_joint_cmd(q_zero, self._vel())
        self._node.mostrar_preview(q_zero)
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

        # ── L1/R1 → ciclar submodos (tabs) ──
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

        ax_st = self._node.ax_status
        ex_st = self._node.ex_status
        # Basta un bus conectado para operar (hardware parcial en pruebas)
        con   = ax_st['conectado'] or ex_st['conectado']
        eme   = ax_st['emergencia'] or ex_st['emergencia']
        cal   = ax_st['modo_calib'] or ex_st['modo_calib']
        listo = self._node.motion_ready

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
        self.btn_estop.configure(      state='normal')
        self.btn_resume.configure(     state='normal' if eme   else 'disabled')
        self.btn_home.configure(       state='normal' if listo else 'disabled')
        self.btn_fk.configure(         state='normal' if listo else 'disabled')
        self.btn_calib_start.configure(state='normal' if (con and not eme and not self._node.motion_busy and not self._node._maintenance_pending)
                                                       else 'disabled')
        self.btn_calib_ok.configure(   state='normal' if self._node.maintenance_ready else 'disabled')
        self.btn_rescue_neg.configure(state='normal' if self._node.maintenance_ready else 'disabled')
        self.btn_rescue_pos.configure(state='normal' if self._node.maintenance_ready else 'disabled')
        self.btn_reset_alerts.configure(state='normal' if con  else 'disabled')
        for btn in self._jog_buttons:
            btn.configure(state='normal' if self._node.maintenance_ready else 'disabled')

        # IK: solo si listo y no hay IK en curso
        ik_en_curso = self._node.motion_busy
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
        elif self._node._maintenance or self._node.cartesian_state['maintenance']:
            self.lbl_global.configure(text='MANTENIMIENTO: iniciar y confirmar calibracion para liberar',
                                      text_color=COL['warn'])
        else:
            self.lbl_global.configure(text='')

        # Angulos actuales
        q = self._node.q_actual
        with self._node._lock:
            recent = time.monotonic() - self._node._joint_seen < 1.0
        for i, lbl in enumerate(self._lbl_angs):
            lbl.configure(text=f'actual: {np.degrees(q[i]):.2f}°' if recent[i] else 'sin feedback reciente')
        for i, lbl in enumerate(self._lbl_jog_angs):
            lbl.configure(text=f'{np.degrees(q[i]):.2f}°' if recent[i] else '—')

        # Pose del efector
        if pose is not None:
            p = pose.pose.position
            roll, pitch, yaw = quaternion_to_rpy(pose.pose.orientation)
            self.lbl_pose_xyz.configure(
                text=f'x {p.x:+.3f} | y {p.y:+.3f} | z {p.z:+.3f} m')
            self.lbl_pose_rpy.configure(
                text=f'R {roll:+.1f}° | P {pitch:+.1f}° | Y {yaw:+.1f}°')

        else:
            self.lbl_pose_xyz.configure(text='Sin pose reciente')
            self.lbl_pose_rpy.configure(text='R — | P — | Y —')

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

        if self._pending_teleop_commit is not None:
            self._tp_p, self._tp_R = self._pending_teleop_commit
            if self.var_stab.get():
                self._stab_R = self._tp_R.copy()
            self._pending_teleop_commit = None
            self._update_tp_label()

        # ── Teleop por joystick (solo en la pestana Teleop) ──
        if self.tabs.get() == 'Teleop':
            self._joystick_teleop_step()

        if self._pending_ik_msg is not None:
            texto, color = self._pending_ik_msg
            self.lbl_ik_warn.configure(text=texto, text_color=color)
            self._pending_ik_msg = None

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
            # A cancelled/failed plan must not be reported as successfully completed.
            completed = cs['progress'] >= 1.0 and cs['within_path_tol']
            self.traj_progress.set(cs['progress'])
            self.lbl_traj_state.configure(
                text=('Completada' if completed else 'Detenida/cancelada; revisar estado')
                     + f'  err {cs["pos_error"]*1000:.1f}mm',
                text_color=COL['ok'] if completed else COL['warn'])
            self.btn_ik.configure(state='normal' if self._node.motion_ready else 'disabled')
        self._last_cart_in_prog = in_prog

        # Estado de servos: sondear ~cada 1.4 s solo si la pestaña esta activa
        if con and self.tabs.get() == 'Estado':
            self._status_poll_count += 1
            if self._status_poll_count >= 12:
                self._status_poll_count = 0
                self._refresh_status()
        else:
            self._status_poll_count = 12   # refresca al entrar a la pestaña

        with self._pending_lock:
            pending_status, self._pending_status = self._pending_status, {}
        if pending_status:
            for name, fila in pending_status.items():
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

        # Visualizacion 3D
        self._redraw_arm()

        self.after(120, self._loop_ui)

    def on_closing(self):
        if self._node._pub_arm_active is not None:
            self._node.destroy_timer(self._node._active_timer)
            self._node._pub_arm_active.publish(Bool(data=False))
        self._node.desconectar()
        self._arm_canvas.close()
        self.destroy()


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
    app.protocol('WM_DELETE_WINDOW', node.pedir_dashboard if node.managed_by_dashboard else app.on_closing)
    try:
        app.mainloop()
    finally:
        app._arm_canvas.close()
        app._arm_canvas._worker.join(timeout=8)

    # Detener primero el contexto para que el thread de spin salga limpio
    rclpy.try_shutdown()
    ros_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
