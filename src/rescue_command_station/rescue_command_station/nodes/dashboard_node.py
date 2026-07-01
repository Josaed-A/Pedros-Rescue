import base64
import csv
import datetime
import json
import math
import os
import signal
import subprocess
import threading
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Joy, JointState, PointCloud2
from std_msgs.msg import Bool, Float32, String
from std_srvs.srv import Trigger
import tf2_ros

from rescue_command_station.control import config as cfg

from rescue_command_station.vision.qr_detector import QrDetector
from rescue_command_station.vision.ros_image import compressed_msg_to_numpy
from rescue_command_station.vision.tk_image import bgr_frame_to_png_data, depth_frame_to_color


COLORS = {
    'bg':           '#0b0f14',
    'surface':      '#111821',
    'surface_high': '#17212d',
    'surface_soft': '#1c2733',
    'border':       '#263544',
    'text':         '#e7edf4',
    'muted':        '#94a3b8',
    'cyan':         '#38bdf8',
    'green':        '#22c55e',
    'green_bg':     '#0f3a20',
    'amber':        '#f59e0b',
    'amber_bg':     '#3d2507',
    'red':          '#ef4444',
    'blue':         '#60a5fa',
    'blue_bg':      '#0c2340',
    'black':        '#030712',
    'surface_btn':  '#1e2d3d',
}

FONT = 'Segoe UI'

TEAM_NAME  = 'SabanaHerons'
COUNTRY    = 'Colombia'
OUTPUT_DIR = '/workspace/maps'

DET_TYPE_OPTIONS = ['ar_code', 'hazmat_sign', 'real_object']

_PODMAN_SOCK = '/tmp/podman.sock'


# ─── ROS Node ────────────────────────────────────────────────────────────────

# ─── Patas (control con stick derecho) ───────────────────────────────────────
LEG_FRONT  = ['PataDelIzq', 'PataDelDer']    # eje X del stick derecho
LEG_REAR   = ['PataTrasIzq', 'PataTrasDer']  # eje Y del stick derecho
LEG_NAMES  = LEG_FRONT + LEG_REAR
LEG_LABELS = {
    'PataDelIzq': 'Del · Izq', 'PataDelDer': 'Del · Der',
    'PataTrasIzq': 'Tras · Izq', 'PataTrasDer': 'Tras · Der',
}
# Simbolo del boton cara del mando segun su indice (X, O, cuadrado, triangulo).
_BTN_SYMBOL = {
    cfg.BUTTON_CROSS: '✕', cfg.BUTTON_CIRCLE: '○',
    cfg.BUTTON_SQUARE: '□', cfg.BUTTON_TRIANGLE: '△',
}


class DashboardRosNode(Node):
    def __init__(self):
        super().__init__('drive_dashboard_node')

        self.declare_parameter('front_camera_topic',    '/robot/camera/front/image_raw/compressed')
        self.declare_parameter('astra_color_topic',     '/robot/camera/astra/color/image_raw/compressed')
        self.declare_parameter('astra_annotated_topic', '/camera/color/image_annotated/compressed')
        self.declare_parameter('astra_depth_topic',     '/robot/camera/astra/depth/image_raw/compressed')
        self.declare_parameter('point_cloud_topic',     '/robot/camera/astra/points')
        self.declare_parameter('raspberry_timeout_seconds', 2.5)

        self.front_camera_topic    = self.get_parameter('front_camera_topic').value
        self.astra_color_topic     = self.get_parameter('astra_color_topic').value
        self.astra_annotated_topic = self.get_parameter('astra_annotated_topic').value
        self.astra_depth_topic     = self.get_parameter('astra_depth_topic').value
        self.point_cloud_topic     = self.get_parameter('point_cloud_topic').value
        self.raspberry_timeout_seconds = float(
            self.get_parameter('raspberry_timeout_seconds').value)

        self.qr_detector = QrDetector()

        self.latest_front_frame           = None
        self.latest_astra_color_frame     = None
        self.latest_astra_annotated_frame = None
        self.front_camera_frames          = 0
        self.astra_color_frames           = 0
        self.astra_annotated_frames       = 0
        self.latest_qr_text               = ''
        self.last_qr_scan_time            = 0.0
        self.qr_scan_interval             = 0.25
        self.last_raspberry_msg_time      = None
        self.last_raspberry_source        = 'sin datos'

        # Popup queue: dicts pushed by callbacks, consumed by the GUI thread
        self._popup_queue: list = []
        self._shown_detection_keys: dict = {}   # key -> last_queued_timestamp
        self._detection_popup_cooldown = 90.0   # seconds before re-alerting same detection
        self._qr_popup_cooldown = 10.0          # QR se re-muestra más seguido que hazmat/AprilTag

        # Detection list (from /object_detections + manual injections)
        self.latest_detections: list = []

        self.status = {
            'gear': 1, 'gear_limit': 0.20,
            'target_speed': 0.0, 'real_speed_abs': 0.0,
            'linear_x': 0.0, 'angular_z': 0.0,
            'left_track': 0.0, 'right_track': 0.0,
            'status_text': 'SIN DATOS',
            'joy_x': 0.0, 'joy_y': 0.0,
            'l1_pressed': 0, 'r1_pressed': 0,
        }

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(String,          '/drive_status',            self.drive_status_callback, 10)
        self.create_subscription(Float32,         '/real_speed_abs',          self.real_speed_callback, 10)
        self.create_subscription(CompressedImage, self.front_camera_topic,    self.front_camera_callback, sensor_qos)
        self.create_subscription(CompressedImage, self.astra_color_topic,     self.astra_color_callback, sensor_qos)
        self.create_subscription(CompressedImage, self.astra_annotated_topic, self.astra_annotated_callback, sensor_qos)
        self.create_subscription(CompressedImage, self.astra_depth_topic,     self._noop, sensor_qos)
        self.create_subscription(PointCloud2,     self.point_cloud_topic,     self._noop_pc, sensor_qos)
        self.create_subscription(String,          '/object_detections',       self.detections_callback, 10)

        self._save_csv_client     = self.create_client(Trigger, '/save_detection_csv')
        self._save_ply_client     = self.create_client(Trigger, '/save_pointcloud_ply')
        self._save_geotiff_client = self.create_client(Trigger, '/save_geotiff')

        # Publisher para inyectar detecciones manuales al geotiff_writer
        self._manual_det_pub = self.create_publisher(String, '/object_detections', 10)

        # TF para obtener posición actual del robot en el mapa
        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        # ── Patas (stick derecho) ─────────────────────────────────────
        # enabled: si una pata esta deshabilitada el joystick no la mueve
        # (permite controlar solo una). pos_deg: grados acumulados en vivo.
        self.legs_enabled = {n: True for n in LEG_NAMES}
        self.legs_pos_deg = {n: 0.0 for n in LEG_NAMES}
        self._legs_cmd_pub = self.create_publisher(JointState, '/legs/cmd', 10)
        self.create_subscription(JointState, '/legs/state', self.legs_state_callback, 10)
        self.create_subscription(Joy, '/joy', self.joy_legs_callback, 10)
        self._legs_calib_client  = self.create_client(Trigger, '/legs/calibrate')
        self._legs_torque_client = self.create_client(Trigger, '/legs/enable_torque')
        # Conexion/desconexion de los buses de servos (AX-12A brazo+patas y EX-106)
        self._connect_clients    = [self.create_client(Trigger, '/ax12a/connect'),
                                    self.create_client(Trigger, '/ex106/connect')]
        self._disconnect_clients = [self.create_client(Trigger, '/ax12a/disconnect'),
                                    self.create_client(Trigger, '/ex106/disconnect')]
        self._ax_connect_client  = self._connect_clients[0]   # compat

        # ── Estado de botones del mando (deteccion de flanco) ──────────
        self._prev_buttons = []
        self._dpad_prev = False          # flanco flecha ABAJO (cambio de GUI)
        self._dpad_x_prev = False        # flanco flecha DERECHA (reconectar buses)
        self.reconnect_requested = False # la GUI lo lee para reconectar AX+EX
        self.switch_gui_requested = False   # la GUI lo lee para alternar interfaz
        self.legs_enabled_dirty   = True    # fuerza al GUI a sincronizar checkboxes
        # True solo cuando el dashboard esta al frente: con el brazo abierto el
        # mando controla el brazo (no las patas). La cruz funciona en ambos.
        self.legs_control_active  = True

        # Publica si el brazo esta al frente. El teleop del vehiculo lo respeta
        # (manda cero), y la GUI del brazo lo usa para mostrarse/ocultarse.
        self._arm_active_pub = self.create_publisher(Bool, '/arm_active', 10)
        self.create_timer(0.2, self._publish_arm_active)
        # El boton "volver" de la GUI del brazo pide regresar al dashboard.
        self.create_subscription(Bool, '/gui_switch_request', self._cb_gui_switch, 10)
        # Aviso de reinicio de buses (lo muestra tambien la GUI del brazo).
        self._bus_reset_pub = self.create_publisher(Bool, '/bus_reset', 10)

        # ── Auto-conexion de los buses al arrancar (sin boton "conectar") ──
        self._autoconnected = False
        self.create_timer(1.0, self._auto_connect_once)

        self.get_logger().info('Dashboard iniciado — vision, mapeo y control')

    def _publish_arm_active(self):
        m = Bool()
        m.data = not self.legs_control_active   # brazo al frente == patas off
        self._arm_active_pub.publish(m)

    def _cb_gui_switch(self, msg):
        # La GUI del brazo pidio volver al dashboard (boton "volver").
        if msg.data:
            self.switch_gui_requested = True

    def _auto_connect_once(self):
        """Conecta los buses de servos automaticamente cuando los servicios
        esten listos (una sola vez)."""
        if self._autoconnected:
            return
        ready = [c for c in self._connect_clients if c.service_is_ready()]
        if not ready:
            return
        for c in ready:
            c.call_async(Trigger.Request())
        self._autoconnected = True
        self.get_logger().info('Auto-conexion de servos enviada.')

    def connect_all(self):
        for c in self._connect_clients:
            if c.service_is_ready():
                c.call_async(Trigger.Request())

    def disconnect_all(self):
        for c in self._disconnect_clients:
            if c.service_is_ready():
                c.call_async(Trigger.Request())

    # ── Helpers ──────────────────────────────────────────────────
    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def mark_raspberry_seen(self, source):
        self.last_raspberry_msg_time = self.now_seconds()
        self.last_raspberry_source = source

    def get_raspberry_connection(self):
        if self.last_raspberry_msg_time is None:
            return False, None, self.last_raspberry_source
        age = self.now_seconds() - self.last_raspberry_msg_time
        return age <= self.raspberry_timeout_seconds, age, self.last_raspberry_source

    def get_robot_pose_map(self):
        """Devuelve (x, y, z) del robot en el frame 'map', o None si el TF no está disponible."""
        try:
            t = self._tf_buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            return (t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z)
        except Exception:
            return None

    def _noop(self, _msg):
        self.mark_raspberry_seen(self.astra_depth_topic)

    def _noop_pc(self, _msg):
        self.mark_raspberry_seen(self.point_cloud_topic)

    # ── Callbacks ────────────────────────────────────────────────
    def drive_status_callback(self, msg):
        try:
            self.status.update(json.loads(msg.data))
        except json.JSONDecodeError:
            pass

    def real_speed_callback(self, msg):
        self.mark_raspberry_seen('/real_speed_abs')
        self.status['real_speed_abs'] = msg.data

    def front_camera_callback(self, msg):
        try:
            self.mark_raspberry_seen(self.front_camera_topic)
            frame = compressed_msg_to_numpy(msg)
            now = self.now_seconds()
            if now - self.last_qr_scan_time >= self.qr_scan_interval:
                self.last_qr_scan_time = now
                frame, qr_text = self.qr_detector.detect_and_annotate(frame)
                if qr_text:
                    self.latest_qr_text = qr_text

                    # Se marca SIEMPRE que se lee, ya se haya visto antes o no
                    # (igual que hazmat/AprilTag/YOLO en detections_callback).
                    det = {
                        'type': 'qr_code',
                        'name': qr_text,
                        '_time': datetime.datetime.now().strftime('%H:%M:%S'),
                    }
                    self.latest_detections.insert(0, det)
                    self.latest_detections = self.latest_detections[:50]

                    # El popup sí se limita con el mismo cooldown que el resto
                    # de detecciones, para no bloquear la GUI cada 0.25s
                    # mientras el mismo QR sigue en cuadro.
                    key = f'qr_code:{qr_text}'
                    if now - self._shown_detection_keys.get(key, 0.0) > self._qr_popup_cooldown:
                        self._shown_detection_keys[key] = now
                        self._popup_queue.append({
                            'kind': 'qr',
                            'text': qr_text,
                            'frame': frame.copy(),
                        })
            self.latest_front_frame = frame
            self.front_camera_frames += 1
        except Exception as exc:
            self.get_logger().warn(f'front_camera: {exc}')

    def astra_color_callback(self, msg):
        try:
            self.mark_raspberry_seen(self.astra_color_topic)
            self.latest_astra_color_frame = compressed_msg_to_numpy(msg)
            self.astra_color_frames += 1
        except Exception as exc:
            self.get_logger().warn(f'astra_color: {exc}')

    def astra_annotated_callback(self, msg):
        try:
            self.latest_astra_annotated_frame = compressed_msg_to_numpy(msg)
            self.astra_annotated_frames += 1
        except Exception as exc:
            self.get_logger().warn(f'astra_annotated: {exc}')

    def detections_callback(self, msg):
        try:
            det = json.loads(msg.data)
            det['_time'] = datetime.datetime.now().strftime('%H:%M:%S')
            self.latest_detections.insert(0, det)
            self.latest_detections = self.latest_detections[:50]

            key = f"{det.get('type', '?')}:{det.get('name', '?')}"
            now = self.now_seconds()
            if now - self._shown_detection_keys.get(key, 0.0) > self._detection_popup_cooldown:
                self._shown_detection_keys[key] = now
                snap = (self.latest_astra_annotated_frame
                        if self.latest_astra_annotated_frame is not None
                        else self.latest_astra_color_frame)
                self._popup_queue.append({
                    'kind': 'detection',
                    'det': det,
                    'frame': snap.copy() if snap is not None else None,
                })
        except Exception:
            pass

    def call_service_async(self, client, on_result):
        if not client.service_is_ready():
            on_result(False, 'Servicio no disponible (no iniciado?)')
            return None
        future = client.call_async(Trigger.Request())
        return future

    # ── Patas ────────────────────────────────────────────────────
    def legs_state_callback(self, msg):
        """/legs/state: position[] = grados de salida acumulados (rad)."""
        import math
        for k, name in enumerate(msg.name):
            if name in self.legs_pos_deg and k < len(msg.position):
                self.legs_pos_deg[name] = math.degrees(msg.position[k])

    def joy_legs_callback(self, msg):
        """Stick derecho → giro de pares de patas. Eje X = delanteras,
        eje Y = traseras; el signo decide la direccion. Las patas
        deshabilitadas no se mueven (para controlar solo una).
        Botones cara → habilitan/deshabilitan cada pata.

        La CRUZ (cambio de GUI) funciona SIEMPRE. El resto del control de
        patas (sticks y botones cara) solo cuando el dashboard esta al frente:
        si la GUI del brazo esta activa, el mando controla el brazo, no las
        patas (legs_control_active=False)."""
        # 1) Cruz -> alternar interfaz (siempre, en cualquier GUI)
        self._handle_dpad(msg)

        # 2) Si el brazo esta al frente, el dashboard NO toca las patas.
        if not self.legs_control_active:
            self._prev_buttons = list(msg.buttons)   # mantener estado de flanco
            return

        # 3) Botones cara -> toggle habilitar/deshabilitar cada pata
        self._handle_leg_buttons(msg)
        self._prev_buttons = list(msg.buttons)

        # 4) Stick derecho -> giro de pares de patas
        if len(msg.axes) <= max(cfg.AXIS_RIGHT_X, cfg.AXIS_RIGHT_Y):
            return
        rx = msg.axes[cfg.AXIS_RIGHT_X]
        ry = msg.axes[cfg.AXIS_RIGHT_Y]
        dz = cfg.LEGS_DEADZONE
        front_dir = 1 if rx > dz else (-1 if rx < -dz else 0)
        rear_dir  = 1 if ry > dz else (-1 if ry < -dz else 0)

        cmd = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        for n in LEG_FRONT:
            cmd.name.append(n)
            cmd.velocity.append(float(front_dir if self.legs_enabled[n] else 0))
        for n in LEG_REAR:
            cmd.name.append(n)
            cmd.velocity.append(float(rear_dir if self.legs_enabled[n] else 0))
        self._legs_cmd_pub.publish(cmd)

    def _handle_dpad(self, msg):
        """Flechas del D-pad (activas siempre, en cualquier GUI):
          - ABAJO  → alternar entre dashboard (movimiento) y GUI del brazo
          - DERECHA → reiniciar la conexion de los buses (AX-12A y EX-106)."""
        ax = msg.axes
        down = (cfg.DPAD_AXIS_Y < len(ax) and
                ax[cfg.DPAD_AXIS_Y] * cfg.DPAD_Y_DOWN > 0.5)
        if down and not self._dpad_prev:
            self.switch_gui_requested = True
        self._dpad_prev = down

        right = (cfg.DPAD_AXIS_X < len(ax) and
                 ax[cfg.DPAD_AXIS_X] * cfg.DPAD_X_RIGHT > 0.5)
        if right and not self._dpad_x_prev:
            self.reconnect_requested = True
        self._dpad_x_prev = right

    def _handle_leg_buttons(self, msg):
        """Flanco de subida de los botones cara → toggle de cada pata."""
        btn = msg.buttons

        def pressed(idx):
            return (idx < len(btn) and btn[idx] and
                    (idx >= len(self._prev_buttons) or not self._prev_buttons[idx]))

        for leg, b in cfg.LEG_BUTTONS.items():
            if leg in self.legs_enabled and pressed(b):
                self.legs_enabled[leg] = not self.legs_enabled[leg]
                self.legs_enabled_dirty = True
                self.get_logger().info(
                    f'{leg}: {"habilitada" if self.legs_enabled[leg] else "deshabilitada"} (boton)')


# ─── Dashboard App ───────────────────────────────────────────────────────────

class ModernDashboardApp:
    def __init__(self, root, ros_node):
        self.root     = root
        self.ros_node = ros_node

        # Camera render tracking
        self.front_camera_photo    = None
        self.astra_camera_photo    = None
        self.rendered_front_frames = -1
        self.rendered_astra_frames = -1
        self._last_astra_annotated = -1

        # Mission state
        self.mapping_active     = False
        self.mapping_start_time: datetime.datetime | None = None

        # Manual detections list
        self.manual_detections: list = []

        # Async service futures
        self._pending_futures: list = []

        # RViz subprocess
        self._rviz_proc: subprocess.Popen | None = None
        # GUI del brazo (arm_station) — proceso aparte; se alterna con el dashboard
        self._arm_proc: subprocess.Popen | None = None

        # Popup state — at most one modal open at a time
        self._popup_open = False

        self.root.title('Pedro Rescue - Estacion de Mando')
        self.root.geometry('1460x840')
        self.root.minsize(1280, 720)
        self.root.configure(bg=COLORS['bg'])

        self._setup_treeview_style()

        self.vars = {
            'gear':         tk.StringVar(),
            'gear_limit':   tk.StringVar(),
            'status':       tk.StringVar(),
            'target_speed': tk.StringVar(),
            'real_speed':   tk.StringVar(),
            'left_track':   tk.StringVar(),
            'right_track':  tk.StringVar(),
            'linear':       tk.StringVar(),
            'angular':      tk.StringVar(),
            'joystick':     tk.StringVar(),
            'shift':        tk.StringVar(),
            'front_camera': tk.StringVar(),
            'astra_camera': tk.StringVar(),
            'qr':           tk.StringVar(),
            'raspberry':    tk.StringVar(),
            'save_status':  tk.StringVar(value=''),
            'manual_count': tk.StringVar(value='0 detecciones manuales'),
            'quick_status': tk.StringVar(value=''),
        }

        self.build_ui()
        self.refresh_ui()
        # Precargar la GUI del brazo (oculta) ~1s despues de mostrar el dashboard,
        # para que el primer cambio con la flecha abajo sea instantaneo.
        self.root.after(1200, self._preload_arm)

    def _setup_treeview_style(self):
        style = ttk.Style()
        style.theme_use('clam')
        style.configure('Det.Treeview',
                        background=COLORS['black'],
                        foreground=COLORS['text'],
                        fieldbackground=COLORS['black'],
                        rowheight=20,
                        font=(FONT, 9),
                        borderwidth=0)
        style.configure('Det.Treeview.Heading',
                        background=COLORS['surface_high'],
                        foreground=COLORS['cyan'],
                        font=(FONT, 8, 'bold'),
                        relief='flat')
        style.map('Det.Treeview',
                  background=[('selected', COLORS['surface_soft'])],
                  foreground=[('selected', COLORS['text'])])
        style.map('Det.Treeview.Heading', relief=[('active', 'flat')])

    # ─── Build layout ─────────────────────────────────────────────────────

    def build_ui(self):
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(1, weight=1)

        self.build_header()

        body = tk.Frame(self.root, bg=COLORS['bg'])
        body.grid(row=1, column=0, sticky='nsew', padx=18, pady=(0, 18))
        body.grid_columnconfigure(0, weight=0, minsize=330)
        body.grid_columnconfigure(1, weight=1)
        body.grid_columnconfigure(2, weight=0, minsize=460)
        body.grid_rowconfigure(0, weight=1)

        self.build_drive_panel(body)
        self.build_video_panel(body)
        self.build_mapeo_panel(body)

    def build_header(self):
        header = tk.Frame(self.root, bg=COLORS['bg'])
        header.grid(row=0, column=0, sticky='ew', padx=18, pady=18)
        header.grid_columnconfigure(0, weight=1)

        title_block = tk.Frame(header, bg=COLORS['bg'])
        title_block.grid(row=0, column=0, sticky='w')
        tk.Label(title_block, text='Pedro Rescue', bg=COLORS['bg'], fg=COLORS['text'],
                 font=(FONT, 24, 'bold')).pack(anchor='w')
        tk.Label(title_block, text='Estacion de mando | traccion, vision y telemetria',
                 bg=COLORS['bg'], fg=COLORS['muted'], font=(FONT, 10)).pack(anchor='w', pady=(2, 0))

        self.status_pill = tk.Label(header, text='SIN DATOS', bg=COLORS['surface_high'],
                                    fg=COLORS['muted'], font=(FONT, 10, 'bold'), padx=16, pady=8)
        self.status_pill.grid(row=0, column=1, sticky='e')

        self.raspberry_pill = tk.Label(header, text='RASP DESCONECTADA',
                                       bg=COLORS['surface_high'], fg=COLORS['red'],
                                       font=(FONT, 10, 'bold'), padx=16, pady=8)
        self.raspberry_pill.grid(row=0, column=2, sticky='e', padx=(10, 0))

    # ─── Drive panel (unchanged) ──────────────────────────────────────────

    def build_drive_panel(self, parent):
        panel = self.card(parent)
        panel.grid(row=0, column=0, sticky='nsew', padx=(0, 14))

        self.card_title(panel, 'Control de orugas', 'Caja, velocidad y mezcla diferencial')

        gear_row = tk.Frame(panel, bg=COLORS['surface'])
        gear_row.pack(fill='x', pady=(12, 14))
        self.big_metric(gear_row, 'Caja',   self.vars['gear'],       COLORS['cyan']).pack(side='left', fill='x', expand=True, padx=(0, 8))
        self.big_metric(gear_row, 'Limite', self.vars['gear_limit'], COLORS['blue']).pack(side='left', fill='x', expand=True)

        self.target_canvas = self.gauge(panel, 'Velocidad objetivo', self.vars['target_speed'], COLORS['cyan'])
        self.real_canvas   = self.gauge(panel, 'Velocidad real',     self.vars['real_speed'],   COLORS['green'])

        self.section(panel, 'Orugas')
        self.track_canvas = tk.Canvas(panel, width=290, height=104, bg=COLORS['surface'], highlightthickness=0)
        self.track_canvas.pack(fill='x', pady=(4, 10))
        self.small_label(panel, self.vars['left_track'])
        self.small_label(panel, self.vars['right_track'])

        self.section(panel, 'Comando ROS')
        self.small_label(panel, self.vars['linear'])
        self.small_label(panel, self.vars['angular'])
        self.small_label(panel, self.vars['joystick'])
        self.small_label(panel, self.vars['shift'])

    # ─── Video panel (2 cameras) ──────────────────────────────────────────

    def build_video_panel(self, parent):
        panel = self.card(parent)
        panel.grid(row=0, column=1, sticky='nsew', padx=(0, 14))
        panel.grid_rowconfigure(1, weight=1)
        panel.grid_columnconfigure(0, weight=1)

        self.card_title(panel, 'Vision en vivo', 'Frontal (QR) + Astra color (deteccion IA)')

        cameras_frame = tk.Frame(panel, bg=COLORS['surface'])
        cameras_frame.pack(fill='both', expand=True, pady=(10, 0))
        cameras_frame.grid_columnconfigure(0, weight=3)
        cameras_frame.grid_columnconfigure(1, weight=2)
        cameras_frame.grid_rowconfigure(0, weight=1)

        front_slot = self.video_slot(cameras_frame, 'Camara frontal (Logitech)',
                                     self.vars['front_camera'], 'Esperando camara frontal...')
        front_slot.grid(row=0, column=0, sticky='nsew', padx=(0, 10))
        self.front_camera_label = front_slot.image_label

        astra_slot = self.video_slot(cameras_frame, 'Astra color (YOLO activo)',
                                     self.vars['astra_camera'], 'Esperando Astra...')
        astra_slot.grid(row=0, column=1, sticky='nsew')
        self.astra_camera_label = astra_slot.image_label

        qr_box = tk.Frame(panel, bg=COLORS['surface_high'])
        qr_box.pack(fill='x', pady=(10, 0))
        tk.Label(qr_box, text='QR / codigo detectado', bg=COLORS['surface_high'],
                 fg=COLORS['muted'], font=(FONT, 9, 'bold'), padx=12, pady=5).pack(anchor='w')
        tk.Label(qr_box, textvariable=self.vars['qr'], bg=COLORS['surface_high'],
                 fg=COLORS['cyan'], font=(FONT, 12, 'bold'), padx=12, pady=6,
                 wraplength=500, justify='left').pack(anchor='w', fill='x')

    def video_slot(self, parent, title, status_var, waiting_text):
        frame = tk.Frame(parent, bg=COLORS['black'],
                         highlightbackground=COLORS['border'], highlightthickness=1)
        frame.grid_columnconfigure(0, weight=1)
        frame.grid_rowconfigure(2, weight=1)
        tk.Label(frame, text=title, bg=COLORS['black'], fg=COLORS['text'],
                 font=(FONT, 10, 'bold'), padx=10, pady=6).grid(row=0, column=0, sticky='ew')
        tk.Label(frame, textvariable=status_var, bg=COLORS['black'], fg=COLORS['muted'],
                 font=(FONT, 8), padx=10).grid(row=1, column=0, sticky='ew')
        frame.image_label = tk.Label(frame, text=waiting_text, bg=COLORS['black'],
                                     fg=COLORS['muted'], font=(FONT, 11))
        frame.image_label.grid(row=2, column=0, sticky='nsew')
        return frame

    # ─── Mapeo panel ──────────────────────────────────────────────────────────

    def build_mapeo_panel(self, parent):
        panel = self.card(parent)
        panel.grid(row=0, column=2, sticky='nsew')
        panel.pack_propagate(False)

        self.card_title(panel, 'Mapeo', 'SabanaHerons · RoboCup 2026D')

        # ── RVIZ ──────────────────────────────────────────────────
        tk.Frame(panel, bg=COLORS['border'], height=1).pack(fill='x', pady=(10, 8))

        self.btn_rviz = tk.Button(
            panel, text='LANZAR RVIZ 3D',
            bg=COLORS['blue_bg'], fg=COLORS['blue'],
            font=(FONT, 12, 'bold'), relief='flat', bd=0,
            pady=13, cursor='hand2',
            activebackground=COLORS['surface_soft'], activeforeground=COLORS['text'],
            command=self._on_launch_rviz)
        self.btn_rviz.pack(fill='x')

        # ── Control del brazo 6-DOF ───────────────────────────────
        self.btn_arm = tk.Button(
            panel, text='CONTROL DEL BRAZO',
            bg=COLORS['surface_high'], fg=COLORS['cyan'],
            font=(FONT, 12, 'bold'), relief='flat', bd=0,
            pady=13, cursor='hand2',
            activebackground=COLORS['surface_soft'], activeforeground=COLORS['text'],
            command=self._toggle_arm_visibility)
        self.btn_arm.pack(fill='x', pady=(8, 0))

        # ── Patas (stick derecho) ─────────────────────────────────
        tk.Frame(panel, bg=COLORS['border'], height=1).pack(fill='x', pady=(10, 6))
        tk.Label(panel, text='PATAS · stick derecho (X=delanteras, Y=traseras)',
                 bg=COLORS['surface'], fg=COLORS['muted'],
                 font=(FONT, 9, 'bold'), anchor='w').pack(fill='x')

        # ── Diagrama VISTA LATERAL: cuerpo del robot + patas como lineas ──
        # Vista de costado = el plano en que giran las patas (referencia clara
        # de donde esta cada una, como manecilla de reloj). FRENTE a la derecha.
        # 0° = delanteras hacia ADELANTE (derecha), traseras hacia ATRAS (izq).
        # Color por lado: Izq = cyan, Der = ambar (contraste para ver cuando
        # estan en angulos distintos); gris si la pata esta deshabilitada.
        self._legs_canvas = tk.Canvas(panel, width=290, height=120,
                                      bg=COLORS['surface'], highlightthickness=0)
        self._legs_canvas.pack(pady=(4, 6))
        bx0, by0, bx1, by1 = 78, 44, 212, 80
        cy = (by0 + by1) // 2
        self._legs_canvas.create_rectangle(bx0, by0, bx1, by1,
                                           outline=COLORS['muted'], width=2,
                                           fill=COLORS['surface_high'])
        self._legs_canvas.create_text(bx1 + 24, cy, text='FRENTE',
                                      fill=COLORS['muted'], font=(FONT, 7, 'bold'))
        self._legs_canvas.create_text(bx0 - 22, cy, text='ATRAS',
                                      fill=COLORS['muted'], font=(FONT, 7, 'bold'))
        self._leg_len = 34
        # hip (x, y) y angulo base (canvas: x derecha, y abajo).
        #   0   = hacia adelante (derecha)  -> patas delanteras
        #   180 = hacia atras (izquierda)   -> patas traseras
        self._leg_geom = {
            'PataDelIzq':  (bx1 - 26, cy,   0.0),
            'PataDelDer':  (bx1 - 12, cy,   0.0),
            'PataTrasIzq': (bx0 + 26, cy, 180.0),
            'PataTrasDer': (bx0 + 12, cy, 180.0),
        }
        self._leg_color = {
            'PataDelIzq':  COLORS['cyan'], 'PataDelDer':  COLORS['amber'],
            'PataTrasIzq': COLORS['cyan'], 'PataTrasDer': COLORS['amber'],
        }
        self._leg_lines = {}
        for name, (hx, hy, base) in self._leg_geom.items():
            self._legs_canvas.create_oval(hx - 2, hy - 2, hx + 2, hy + 2,
                                          fill=COLORS['muted'], outline='')
            a = math.radians(base)
            ln = self._legs_canvas.create_line(
                hx, hy, hx + self._leg_len * math.cos(a), hy + self._leg_len * math.sin(a),
                fill=self._leg_color[name], width=3, capstyle='round')
            self._leg_lines[name] = ln
        # Leyenda de lados
        self._legs_canvas.create_line(bx0, by1 + 16, bx0 + 12, by1 + 16,
                                      fill=COLORS['cyan'], width=3)
        self._legs_canvas.create_text(bx0 + 16, by1 + 16, text='Izq',
                                      fill=COLORS['muted'], font=(FONT, 7), anchor='w')
        self._legs_canvas.create_line(bx0 + 48, by1 + 16, bx0 + 60, by1 + 16,
                                      fill=COLORS['amber'], width=3)
        self._legs_canvas.create_text(bx0 + 64, by1 + 16, text='Der',
                                      fill=COLORS['muted'], font=(FONT, 7), anchor='w')

        self._leg_val_labels = {}
        self._leg_vars = {}
        for name in LEG_NAMES:
            row = tk.Frame(panel, bg=COLORS['surface'])
            row.pack(fill='x', pady=1)
            var = tk.BooleanVar(value=True)
            self._leg_vars[name] = var
            tk.Checkbutton(
                row, variable=var, bg=COLORS['surface'],
                activebackground=COLORS['surface'], selectcolor=COLORS['surface_high'],
                fg=COLORS['text'], bd=0, highlightthickness=0,
                command=lambda n=name: self._on_leg_toggle(n)).pack(side='left')
            tk.Label(row, text=LEG_LABELS[name], bg=COLORS['surface'],
                     fg=COLORS['text'], width=10, anchor='w',
                     font=(FONT, 9)).pack(side='left')
            tk.Label(row, text=_BTN_SYMBOL.get(cfg.LEG_BUTTONS.get(name), '?'),
                     bg=COLORS['surface'], fg=COLORS['muted'], width=2,
                     font=(FONT, 11, 'bold')).pack(side='left')
            lbl = tk.Label(row, text='—', bg=COLORS['surface'], fg=COLORS['cyan'],
                           width=9, anchor='e', font=(FONT, 11, 'bold'))
            lbl.pack(side='right')
            self._leg_val_labels[name] = lbl

        leg_btns = tk.Frame(panel, bg=COLORS['surface'])
        leg_btns.pack(fill='x', pady=(5, 0))
        for txt, cmd, fg in [
            ('RECONECTAR', self._on_legs_reconnect,  COLORS['blue']),
            ('DESCONECT.', self._on_legs_disconnect, COLORS['red']),
            ('CALIB 0°',   self._on_legs_calibrate,  COLORS['green']),
            ('TORQUE',     self._on_legs_torque,     COLORS['amber']),
        ]:
            tk.Button(leg_btns, text=txt, bg=COLORS['surface_high'], fg=fg,
                      font=(FONT, 9, 'bold'), relief='flat', bd=0, pady=7,
                      cursor='hand2', activebackground=COLORS['surface_soft'],
                      activeforeground=COLORS['text'], command=cmd).pack(
                          side='left', fill='x', expand=True, padx=2)
        self._legs_status_label = tk.Label(
            panel, text='Calibrar: posiciona con el stick y pon 0° aquí.',
            bg=COLORS['surface'], fg=COLORS['muted'], font=(FONT, 8),
            anchor='w', wraplength=420, justify='left')
        self._legs_status_label.pack(fill='x', pady=(2, 0))

        # ── Mision ────────────────────────────────────────────────
        tk.Frame(panel, bg=COLORS['border'], height=1).pack(fill='x', pady=(10, 8))

        mission_row = tk.Frame(panel, bg=COLORS['surface'])
        mission_row.pack(fill='x', pady=(0, 8))
        self._mission_indicator = tk.Canvas(mission_row, width=12, height=12,
                                             bg=COLORS['surface'], highlightthickness=0)
        self._mission_indicator.pack(side='left', padx=(0, 6))
        self._mission_dot = self._mission_indicator.create_oval(1, 1, 11, 11,
                                                                  fill=COLORS['muted'], outline='')
        self._mission_time_label = tk.Label(
            mission_row, text='Sin mision activa',
            bg=COLORS['surface'], fg=COLORS['muted'],
            font=(FONT, 10, 'bold'))
        self._mission_time_label.pack(side='left')

        self.btn_start = tk.Button(
            panel, text='INICIAR MISION',
            bg=COLORS['green_bg'], fg=COLORS['green'],
            font=(FONT, 12, 'bold'), relief='flat', bd=0,
            pady=13, cursor='hand2',
            activebackground='#14532d', activeforeground='#4ade80',
            command=self._on_start_mission)
        self.btn_start.pack(fill='x', pady=(0, 6))

        self.btn_stop = tk.Button(
            panel, text='DETENER MISION',
            bg=COLORS['surface_high'], fg=COLORS['muted'],
            font=(FONT, 12, 'bold'), relief='flat', bd=0,
            pady=13, cursor='hand2',
            activebackground=COLORS['amber_bg'], activeforeground=COLORS['amber'],
            state='disabled', command=self._on_stop_mission)
        self.btn_stop.pack(fill='x')

        # ── Guardar RoboCup ───────────────────────────────────────
        tk.Frame(panel, bg=COLORS['border'], height=1).pack(fill='x', pady=(10, 8))

        self.btn_save_all = tk.Button(
            panel, text='GUARDAR MISION',
            bg=COLORS['surface_high'], fg=COLORS['muted'],
            font=(FONT, 12, 'bold'), relief='flat', bd=0,
            pady=14, cursor='hand2',
            activebackground=COLORS['green_bg'], activeforeground=COLORS['green'],
            state='disabled', command=self._on_save_all_robocup)
        self.btn_save_all.pack(fill='x')

        self._save_status_label = tk.Label(
            panel, textvariable=self.vars['save_status'],
            bg=COLORS['surface'], fg=COLORS['muted'],
            font=(FONT, 8), anchor='w', wraplength=420, justify='left')
        self._save_status_label.pack(fill='x', pady=(4, 0))

        # ── Detecciones ───────────────────────────────────────────
        tk.Frame(panel, bg=COLORS['border'], height=1).pack(fill='x', pady=(10, 6))

        det_hdr = tk.Frame(panel, bg=COLORS['surface'])
        det_hdr.pack(fill='x', pady=(0, 4))
        tk.Label(det_hdr, text='DETECCIONES', bg=COLORS['surface'],
                 fg=COLORS['cyan'], font=(FONT, 8, 'bold')).pack(side='left')
        self._det_count_label = tk.Label(det_hdr, text='0',
                                          bg=COLORS['surface_high'], fg=COLORS['text'],
                                          font=(FONT, 8, 'bold'), padx=6, pady=1)
        self._det_count_label.pack(side='left', padx=(8, 0))

        tree_frame = tk.Frame(panel, bg=COLORS['black'],
                              highlightbackground=COLORS['border'], highlightthickness=1)
        tree_frame.pack(fill='x')

        self.det_tree = ttk.Treeview(tree_frame,
                                      columns=('hora', 'tipo', 'nombre'),
                                      show='headings', height=5,
                                      style='Det.Treeview', selectmode='browse')
        self.det_tree.heading('hora',   text='Hora')
        self.det_tree.heading('tipo',   text='Tipo')
        self.det_tree.heading('nombre', text='Nombre')
        self.det_tree.column('hora',   width=60,  anchor='center', stretch=False)
        self.det_tree.column('tipo',   width=110, anchor='w',      stretch=False)
        self.det_tree.column('nombre', width=150, anchor='w',      stretch=True)
        sb = ttk.Scrollbar(tree_frame, orient='vertical', command=self.det_tree.yview)
        self.det_tree.configure(yscrollcommand=sb.set)
        self.det_tree.pack(side='left', fill='x', expand=True)
        sb.pack(side='right', fill='y')
        self._tree_ids: list = []

        # ── Marcado rápido de detecciones ─────────────────────────
        tk.Frame(panel, bg=COLORS['border'], height=1).pack(fill='x', pady=(10, 6))

        tk.Label(panel, text='MARCAR DETECCION',
                 bg=COLORS['surface'], fg=COLORS['cyan'],
                 font=(FONT, 8, 'bold')).pack(anchor='w', pady=(0, 6))

        # Campo nombre / ID (Enter también lanza el último tipo seleccionado)
        qname_row = tk.Frame(panel, bg=COLORS['surface'])
        qname_row.pack(fill='x', pady=(0, 6))
        tk.Label(qname_row, text='ID / Nombre:',
                 bg=COLORS['surface'], fg=COLORS['muted'],
                 font=(FONT, 9), anchor='w').pack(side='left')
        self._quick_name_entry = tk.Entry(
            qname_row, bg=COLORS['surface_high'], fg=COLORS['text'],
            insertbackground=COLORS['text'], font=(FONT, 11), relief='flat', bd=4)
        self._quick_name_entry.pack(side='left', fill='x', expand=True, padx=(8, 0))

        # 3 botones coloreados — un clic = marca posición actual del robot
        # Colores según spec RoboCup 2026: AR amarillo, Hazmat naranja, Objeto rojo
        qbtn_row = tk.Frame(panel, bg=COLORS['surface'])
        qbtn_row.pack(fill='x', pady=(0, 4))
        _QDET = [
            ('ar_code',     '#2d2500', '#ffc800', 'AR CODE'),
            ('hazmat_sign', '#2d1200', '#ff641e', 'HAZMAT'),
            ('real_object', '#200202', '#f00a0a', 'OBJETO'),
        ]
        for dtype, bg, fg, lbl in _QDET:
            tk.Button(
                qbtn_row, text=lbl,
                bg=bg, fg=fg,
                font=(FONT, 10, 'bold'), relief='flat', bd=0,
                pady=12, cursor='hand2',
                activebackground=COLORS['surface_soft'],
                activeforeground=fg,
                command=lambda t=dtype: self._on_quick_mark(t),
            ).pack(side='left', fill='x', expand=True, padx=(0, 3))

        # Estado del último marcado
        self._quick_status_label = tk.Label(
            panel, textvariable=self.vars['quick_status'],
            bg=COLORS['surface'], fg=COLORS['muted'],
            font=(FONT, 8), anchor='w', wraplength=420, justify='left')
        self._quick_status_label.pack(fill='x', pady=(2, 2))

        tk.Label(panel, textvariable=self.vars['manual_count'],
                 bg=COLORS['surface'], fg=COLORS['muted'],
                 font=(FONT, 8), anchor='w').pack(fill='x')

        tk.Button(panel, text='GUARDAR CSV MANUAL',
                  bg=COLORS['surface_btn'], fg=COLORS['amber'],
                  font=(FONT, 10, 'bold'), relief='flat', bd=0,
                  pady=10, cursor='hand2',
                  command=self._on_save_manual_csv).pack(fill='x', pady=(6, 0))

    # ─── Widget helpers ───────────────────────────────────────────────────

    def card(self, parent):
        return tk.Frame(parent, bg=COLORS['surface'],
                        highlightbackground=COLORS['border'], highlightthickness=1,
                        padx=16, pady=16)

    def card_title(self, parent, title, subtitle):
        tk.Label(parent, text=title, bg=COLORS['surface'], fg=COLORS['text'],
                 font=(FONT, 15, 'bold')).pack(anchor='w')
        tk.Label(parent, text=subtitle, bg=COLORS['surface'], fg=COLORS['muted'],
                 font=(FONT, 9)).pack(anchor='w', pady=(2, 0))

    def section(self, parent, title):
        tk.Label(parent, text=title.upper(), bg=COLORS['surface'], fg=COLORS['cyan'],
                 font=(FONT, 8, 'bold')).pack(anchor='w', pady=(14, 4))

    def big_metric(self, parent, label, variable, accent):
        frame = tk.Frame(parent, bg=COLORS['surface_high'], padx=12, pady=10)
        tk.Label(frame, text=label, bg=COLORS['surface_high'], fg=COLORS['muted'],
                 font=(FONT, 8, 'bold')).pack(anchor='w')
        tk.Label(frame, textvariable=variable, bg=COLORS['surface_high'], fg=accent,
                 font=(FONT, 22, 'bold')).pack(anchor='w')
        return frame

    def gauge(self, parent, title, variable, accent):
        self.section(parent, title)
        tk.Label(parent, textvariable=variable, bg=COLORS['surface'], fg=COLORS['text'],
                 font=(FONT, 11, 'bold')).pack(anchor='w')
        canvas = tk.Canvas(parent, width=290, height=18, bg=COLORS['surface'], highlightthickness=0)
        canvas.pack(fill='x', pady=(5, 2))
        canvas.accent = accent
        return canvas

    def small_label(self, parent, variable):
        tk.Label(parent, textvariable=variable, bg=COLORS['surface'], fg=COLORS['muted'],
                 font=(FONT, 10), anchor='w').pack(fill='x', pady=2)

    def _save_btn(self, parent, text, fg, command):
        return tk.Button(parent, text=text, bg=COLORS['surface_high'], fg=fg,
                         font=(FONT, 8, 'bold'), relief='flat', bd=0,
                         padx=6, pady=8, cursor='hand2', command=command,
                         activebackground=COLORS['surface_soft'],
                         activeforeground=COLORS['text'], wraplength=80)

    # ─── RViz ─────────────────────────────────────────────────────────────

    def _on_launch_rviz(self):
        if self._rviz_proc is not None and self._rviz_proc.poll() is None:
            self.vars['save_status'].set('RViz ya está corriendo.')
            self._save_status_label.configure(fg=COLORS['muted'])
            return

        try:
            from ament_index_python.packages import get_package_share_directory
            rviz_config = os.path.join(
                get_package_share_directory('rescue_bringup'), 'config', 'slam_rviz.rviz')
        except Exception:
            rviz_config = '/workspace/src/rescue_bringup/config/slam_rviz.rviz'

        env = os.environ.copy()
        env.setdefault('DISPLAY', ':0')

        try:
            self._rviz_proc = subprocess.Popen(
                ['rviz2', '-d', rviz_config],
                env=env,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.PIPE,
            )
            self.btn_rviz.configure(text='RVIZ CORRIENDO  ●',
                                    fg=COLORS['green'], bg=COLORS['green_bg'])
        except Exception as exc:
            self.vars['save_status'].set(f'RViz error: {exc}')
            self._save_status_label.configure(fg=COLORS['red'])

    # ─── Control del brazo 6-DOF (alternar GUI) ───────────────────────────

    def _preload_arm(self):
        """Lanza la GUI del brazo (arm_station) UNA vez al inicio, oculta.

        La GUI del brazo arranca retraida y solo se muestra cuando /arm_active
        es True. Al tenerla precargada, alternar dashboard<->brazo es
        instantaneo (solo mostrar/ocultar ventanas), sin relanzar nada."""
        if self._arm_proc is not None and self._arm_proc.poll() is None:
            return
        try:
            self._arm_proc = subprocess.Popen(
                ['ros2', 'launch', 'rescue_command_station', 'arm_station.launch.py'],
                start_new_session=True)
            self.ros_node.get_logger().info('Precargando GUI del brazo (oculta)...')
        except Exception as e:
            self.ros_node.get_logger().error(f'No se pudo precargar el brazo: {e}')

    def _toggle_arm_visibility(self):
        """Flecha ABAJO / boton: alterna entre dashboard (movimiento) y brazo.
        El brazo ya corre oculto; solo cambiamos que ventana se ve. La GUI del
        brazo se muestra/oculta sola al seguir /arm_active."""
        if self._arm_proc is None or self._arm_proc.poll() is not None:
            # Aun no precargado (o se cayo): precargar ahora.
            self._preload_arm()
        show_arm = self.ros_node.legs_control_active   # hoy el dashboard al frente?
        # legs_control_active True = dashboard al frente. Al mostrar el brazo se
        # apaga el control de patas y el del vehiculo (via /arm_active).
        self.ros_node.legs_control_active = not show_arm
        if show_arm:
            self.root.withdraw()      # ocultar dashboard; el brazo aparece via /arm_active
        else:
            self.root.deiconify()     # volver al dashboard
            self.root.lift()

    # ─── Patas ────────────────────────────────────────────────────────────

    def _on_leg_toggle(self, name):
        self.ros_node.legs_enabled[name] = bool(self._leg_vars[name].get())

    def _legs_call(self, client, accion):
        def cb(success, msg):
            self._legs_status_label.configure(
                text=f'{accion}: {msg}',
                fg=COLORS['green'] if success else COLORS['red'])
        future = self.ros_node.call_service_async(client, cb)
        if future is not None:
            self._pending_futures.append((future, cb))
        self._legs_status_label.configure(text=f'{accion}...', fg=COLORS['muted'])

    def _on_legs_reconnect(self):
        self.ros_node.connect_all()
        self._legs_status_label.configure(text='Reconectando buses de servos...',
                                          fg=COLORS['muted'])

    def _on_legs_reset_buses(self):
        """Reinicia la conexion de AMBOS buses (AX-12A y EX-106): desconecta y
        vuelve a conectar ~0.6 s despues. Util para recuperarse de un error."""
        self.ros_node.disconnect_all()
        self._legs_status_label.configure(text='Reiniciando buses (AX + EX)...',
                                          fg=COLORS['amber'])
        self.root.after(600, self.ros_node.connect_all)
        # Avisar tambien a la GUI del brazo (puede estar al frente).
        try:
            m = Bool(); m.data = True
            self.ros_node._bus_reset_pub.publish(m)
        except Exception:
            pass

    def _on_legs_disconnect(self):
        self.ros_node.disconnect_all()
        self._legs_status_label.configure(text='Buses de servos DESCONECTADOS.',
                                          fg=COLORS['amber'])

    def _on_legs_calibrate(self):
        self._legs_call(self.ros_node._legs_calib_client, 'Calibrar patas')

    def _on_legs_torque(self):
        self._legs_call(self.ros_node._legs_torque_client, 'Rehabilitar torque')

    def _refresh_legs(self):
        # Sincronizar checkboxes con el estado (el mando tambien los cambia)
        if getattr(self.ros_node, 'legs_enabled_dirty', False):
            for name, var in self._leg_vars.items():
                var.set(bool(self.ros_node.legs_enabled.get(name, True)))
            self.ros_node.legs_enabled_dirty = False
        # Cambio de interfaz pedido por la flecha abajo (o el boton "volver")
        if getattr(self.ros_node, 'switch_gui_requested', False):
            self.ros_node.switch_gui_requested = False
            self._toggle_arm_visibility()
        # Flecha derecha → reiniciar conexion de los buses (AX + EX) por si hay error
        if getattr(self.ros_node, 'reconnect_requested', False):
            self.ros_node.reconnect_requested = False
            self._on_legs_reset_buses()
        for name, lbl in self._leg_val_labels.items():
            deg = self.ros_node.legs_pos_deg.get(name, 0.0)
            lbl.configure(text=f'{deg:+.1f}°')
            # Rotar la linea de la pata en el diagrama de vista superior
            line = self._leg_lines.get(name)
            if line is not None:
                hx, hy, base = self._leg_geom[name]
                a = math.radians(base + deg)
                self._legs_canvas.coords(
                    line, hx, hy,
                    hx + self._leg_len * math.cos(a),
                    hy + self._leg_len * math.sin(a))
                enabled = self.ros_node.legs_enabled.get(name, True)
                self._legs_canvas.itemconfig(
                    line, fill=self._leg_color[name] if enabled else COLORS['muted'])

    def _check_rviz_status(self):
        if self._rviz_proc is not None and self._rviz_proc.poll() is not None:
            rc = self._rviz_proc.returncode
            if rc != 0:
                err = (self._rviz_proc.stderr.read() or b'').decode()[:120]
                self.vars['save_status'].set(f'RViz error: {err}')
                self._save_status_label.configure(fg=COLORS['red'])
                self.btn_rviz.configure(text='LANZAR RVIZ 3D',
                                        fg=COLORS['blue'], bg=COLORS['blue_bg'])
            # Success: keep button green (rviz is running in slam container)

    # ─── Mission control ──────────────────────────────────────────────────

    def _on_start_mission(self):
        self.mapping_active     = True
        self.mapping_start_time = datetime.datetime.now()
        self.btn_start.configure(state='disabled', fg=COLORS['muted'],
                                 bg=COLORS['surface_high'])
        self.btn_stop.configure(state='normal', fg=COLORS['amber'],
                                bg=COLORS['amber_bg'])
        self.btn_save_all.configure(
            state='normal',
            fg=COLORS['green'], bg=COLORS['green_bg'])
        self.vars['save_status'].set('')

    def _on_stop_mission(self):
        self.mapping_active = False
        self.btn_start.configure(state='normal', fg=COLORS['green'],
                                 bg=COLORS['green_bg'])
        self.btn_stop.configure(state='disabled', fg=COLORS['muted'],
                                bg=COLORS['surface_high'])
        self.btn_save_all.configure(
            state='normal',
            text='GUARDAR MISION ROBOCUP',
            fg=COLORS['amber'], bg=COLORS['amber_bg'])

    def _update_mission_indicator(self):
        if self.mapping_active and self.mapping_start_time:
            elapsed = datetime.datetime.now() - self.mapping_start_time
            h, rem  = divmod(int(elapsed.total_seconds()), 3600)
            m, s    = divmod(rem, 60)
            t_str   = f'{h:02d}:{m:02d}:{s:02d}' if h else f'{m:02d}:{s:02d}'
            self._mission_time_label.configure(
                text=f'En mision: {t_str}', fg=COLORS['green'])
            self._mission_indicator.itemconfigure(self._mission_dot, fill=COLORS['green'])
        elif not self.mapping_active and self.mapping_start_time is not None:
            self._mission_time_label.configure(text='Mision detenida', fg=COLORS['amber'])
            self._mission_indicator.itemconfigure(self._mission_dot, fill=COLORS['amber'])
        else:
            self._mission_time_label.configure(text='Sin mision activa', fg=COLORS['muted'])
            self._mission_indicator.itemconfigure(self._mission_dot, fill=COLORS['muted'])

    # ─── Save callbacks ───────────────────────────────────────────────────

    def _on_save_result(self, success: bool, message: str):
        color  = COLORS['green'] if success else COLORS['red']
        prefix = 'OK' if success else 'ERROR'
        self.vars['save_status'].set(f'{prefix}: {message[:80]}')
        self._save_status_label.configure(fg=color)

    def _on_save_all_robocup(self):
        """Call CSV + PLY + TIFF services simultaneously, then open the output folder."""
        self._robocup_results  = {}
        self._robocup_expected = {'CSV', 'PLY', 'TIFF'}
        self.vars['save_status'].set('Guardando CSV + PLY + TIFF...')
        self._save_status_label.configure(fg=COLORS['muted'])
        self.btn_save_all.configure(state='disabled',
                                    text='Guardando...', fg=COLORS['muted'],
                                    bg=COLORS['surface_high'])

        services = [
            (self.ros_node._save_csv_client,     'CSV'),
            (self.ros_node._save_ply_client,     'PLY'),
            (self.ros_node._save_geotiff_client, 'TIFF'),
        ]
        for client, name in services:
            if not client.service_is_ready():
                self._robocup_results[name] = (False, f'{name}: servicio no listo')
                continue
            future = client.call_async(Trigger.Request())

            def make_cb(n):
                def cb(success, msg):
                    self._robocup_results[n] = (success, msg)
                    if self._robocup_expected.issubset(set(self._robocup_results.keys())):
                        self._on_robocup_all_done()
                return cb

            self._pending_futures.append((future, make_cb(name)))

        if self._robocup_expected.issubset(set(self._robocup_results.keys())):
            self._on_robocup_all_done()

    def _on_robocup_all_done(self):
        label = 'GUARDAR MISION ROBOCUP'
        self.btn_save_all.configure(state='normal', text=label)
        all_ok = all(v[0] for v in self._robocup_results.values())
        if all_ok:
            self.vars['save_status'].set(
                f'Guardado en {OUTPUT_DIR} · abriendo carpeta...')
            self._save_status_label.configure(fg=COLORS['green'])
            self.btn_save_all.configure(fg=COLORS['green'], bg=COLORS['green_bg'])
            try:
                subprocess.Popen(['xdg-open', OUTPUT_DIR])
            except Exception:
                pass
        else:
            bad = [n for n, (ok, _) in self._robocup_results.items() if not ok]
            self.vars['save_status'].set(
                f'Error en: {", ".join(bad)} — inicia la mision primero')
            self._save_status_label.configure(fg=COLORS['red'])
            self.btn_save_all.configure(fg=COLORS['amber'], bg=COLORS['amber_bg'])

    def _check_service_futures(self):
        still_pending = []
        for future, callback in self._pending_futures:
            if future.done():
                try:
                    if future.exception():
                        callback(False, str(future.exception()))
                    else:
                        r = future.result()
                        callback(r.success, r.message)
                except Exception as exc:
                    callback(False, str(exc))
            else:
                still_pending.append((future, callback))
        self._pending_futures = still_pending

    # ─── Detections table ─────────────────────────────────────────────────

    def _refresh_det_tree(self):
        dets = self.ros_node.latest_detections
        self._det_count_label.configure(text=str(len(dets)))

        current_ids = self.det_tree.get_children()
        # Only rebuild when count changes
        if len(current_ids) == len(dets) and len(dets) > 0:
            return

        for iid in current_ids:
            self.det_tree.delete(iid)

        TYPE_ABBREV = {
            'ar_code':     'AR Code',
            'hazmat_sign': 'Hazmat',
            'real_object': 'Objeto',
            'qr_code':     'QR',
        }
        for det in dets[:25]:
            hora   = det.get('_time', '--:--:--')
            tipo   = TYPE_ABBREV.get(det.get('type', '?'), det.get('type', '?'))
            nombre = det.get('name', '?')
            tag    = '[M] ' if det.get('_manual') else ''
            self.det_tree.insert('', 'end', values=(hora, f'{tag}{tipo}', nombre))

    # ─── Marcado rápido de detecciones ───────────────────────────────────────

    _QDET_PREFIX = {'ar_code': 'AR', 'hazmat_sign': 'HZ', 'real_object': 'OBJ'}
    _QDET_LABEL  = {'ar_code': 'AR Code', 'hazmat_sign': 'Hazmat', 'real_object': 'Objeto'}
    _QDET_COLOR  = {'ar_code': '#ffc800', 'hazmat_sign': '#ff641e', 'real_object': '#f00a0a'}

    def _on_quick_mark(self, det_type: str):
        name = self._quick_name_entry.get().strip()
        if not name:
            count  = sum(1 for d in self.manual_detections if d['type'] == det_type)
            prefix = self._QDET_PREFIX.get(det_type, 'X')
            name   = f'{prefix}{count + 1}'

        pos = self.ros_node.get_robot_pose_map()
        x, y, z = pos if pos else (0.0, 0.0, 0.0)

        now   = datetime.datetime.now()
        t_str = now.strftime('%H:%M:%S')

        record = {
            'detection': len(self.manual_detections) + 1,
            'time':   t_str,
            'type':   det_type,
            'name':   name,
            'x': round(x, 4),
            'y': round(y, 4),
            'z': round(z, 4),
            'robot':  'Pedro',
            'mode':   'T',
        }
        self.manual_detections.append(record)

        # Muestra en la tabla de detecciones
        self.ros_node.latest_detections.insert(0, {
            'type':    det_type,
            'name':    name,
            'wx':      x,
            'wy':      y,
            '_manual': True,
            '_time':   t_str,
        })

        # Publica al geotiff_writer para que aparezca en el mapa 2D
        geo = String()
        geo.data = json.dumps({'type': det_type, 'name': name, 'wx': x, 'wy': y})
        self.ros_node._manual_det_pub.publish(geo)

        # Limpia el campo nombre para la siguiente detección
        self._quick_name_entry.delete(0, 'end')

        # Feedback visual
        pos_str = f'({x:.2f}, {y:.2f})' if pos else '(sin TF — posición en 0,0)'
        label   = self._QDET_LABEL.get(det_type, det_type)
        color   = self._QDET_COLOR.get(det_type, COLORS['green'])
        self.vars['quick_status'].set(f'✓  {label}  "{name}"  @  {pos_str}')
        self._quick_status_label.configure(fg=color)
        self.vars['manual_count'].set(f'{len(self.manual_detections)} detecciones manuales')

    def _on_save_manual_csv(self):
        out_dir = OUTPUT_DIR
        os.makedirs(out_dir, exist_ok=True)
        now = datetime.datetime.now()
        ts         = now.strftime('%H-%M-%S')
        start_date = (self.mapping_start_time or now).strftime('%Y-%m-%d')
        start_hms  = (self.mapping_start_time or now).strftime('%H:%M:%S')
        path = os.path.join(out_dir, f'RoboCup2026-{TEAM_NAME}-manual-{ts}-pois.csv')
        try:
            with open(path, 'w', newline='') as f:
                # Preamble obligatorio RoboCup 2026 (spec pág. 20)
                f.write(f'"pois"\n')
                f.write(f'"1.3"\n')
                f.write(f'"{TEAM_NAME}"\n')
                f.write(f'"{COUNTRY}"\n')
                f.write(f'"{start_date}"\n')
                f.write(f'"{start_hms}"\n')
                f.write(f'"manual"\n')
                f.write('\n')
                writer = csv.writer(f, quoting=csv.QUOTE_NONNUMERIC)
                writer.writerow(['detection', 'time', 'type', 'name',
                                 'x', 'y', 'z', 'robot', 'mode'])
                for det in self.manual_detections:
                    writer.writerow([
                        det['detection'], det['time'],
                        det['type'],      det['name'],
                        det['x'],         det['y'],    det['z'],
                        det['robot'],     det['mode'],
                    ])
            n = len(self.manual_detections)
            msg = f'CSV guardado ({n} detecciones):\n{path}'
            if not self.manual_detections:
                msg = f'CSV de cabecera guardado (sin detecciones):\n{path}'
            messagebox.showinfo('Guardado', msg)
        except Exception as exc:
            messagebox.showerror('Error', f'No se pudo guardar:\n{exc}')

    # ─── Main refresh loop ────────────────────────────────────────────────

    def refresh_ui(self):
        self._check_service_futures()
        self._check_rviz_status()
        self._update_mission_indicator()
        self.refresh_raspberry_status()
        self.refresh_drive_status()
        self.refresh_cameras()
        self._refresh_det_tree()
        self._refresh_legs()
        self._check_popup_queue()
        self.root.after(50, self.refresh_ui)

    def refresh_raspberry_status(self):
        connected, _age, _src = self.ros_node.get_raspberry_connection()
        if connected:
            self.raspberry_pill.configure(text='RASP CONECTADA', fg=COLORS['green'])
        else:
            self.raspberry_pill.configure(text='RASP DESCONECTADA', fg=COLORS['red'])

    def refresh_drive_status(self):
        s = self.ros_node.status
        gear        = int(s.get('gear', 1))
        gear_limit  = float(s.get('gear_limit', 0.20))
        status_text = s.get('status_text', 'SIN DATOS')
        target      = float(s.get('target_speed', 0.0))
        real        = float(s.get('real_speed_abs', 0.0))
        left        = float(s.get('left_track', 0.0))
        right       = float(s.get('right_track', 0.0))
        lin         = float(s.get('linear_x', 0.0))
        ang         = float(s.get('angular_z', 0.0))
        jx          = float(s.get('joy_x', 0.0))
        jy          = float(s.get('joy_y', 0.0))
        l1          = int(s.get('l1_pressed', 0))
        r1          = int(s.get('r1_pressed', 0))

        self.vars['gear'].set(str(gear))
        self.vars['gear_limit'].set(f'{gear_limit * 100.0:.0f}%')
        self.vars['status'].set(status_text)
        self.vars['target_speed'].set(f'{target * 100.0:.1f}%')
        self.vars['real_speed'].set(f'{real * 100.0:.1f}%')
        self.vars['left_track'].set(f'Oruga izquierda: {left:.3f}')
        self.vars['right_track'].set(f'Oruga derecha: {right:.3f}')
        self.vars['linear'].set(f'linear.x: {lin:.3f}')
        self.vars['angular'].set(f'angular.z: {ang:.3f}')
        self.vars['joystick'].set(f'Joystick X: {jx:.3f} | Y: {jy:.3f}')
        self.vars['shift'].set(f'R1: {r1} | L1: {l1}')
        self.update_status_pill(status_text)
        self.draw_gauge(self.target_canvas, target)
        self.draw_gauge(self.real_canvas, real)
        self.draw_tracks(left, right)

    def refresh_cameras(self):
        self.vars['front_camera'].set(
            f'{self.ros_node.front_camera_topic} | {self.ros_node.front_camera_frames} fr')
        if self.ros_node.front_camera_frames != self.rendered_front_frames:
            self.update_video_image(self.ros_node.latest_front_frame,
                                    self.front_camera_label, 'front_camera_photo', 560, 390)
            self.rendered_front_frames = self.ros_node.front_camera_frames

        ann = self.ros_node.astra_annotated_frames
        raw = self.ros_node.astra_color_frames
        if ann != self._last_astra_annotated and ann > 0:
            self.vars['astra_camera'].set(f'Annotated YOLO | {ann} fr')
            self.update_video_image(self.ros_node.latest_astra_annotated_frame,
                                    self.astra_camera_label, 'astra_camera_photo', 380, 290)
            self._last_astra_annotated = ann
            self.rendered_astra_frames = raw
        elif raw != self.rendered_astra_frames:
            self.vars['astra_camera'].set(f'{self.ros_node.astra_color_topic} | {raw} fr')
            self.update_video_image(self.ros_node.latest_astra_color_frame,
                                    self.astra_camera_label, 'astra_camera_photo', 380, 290)
            self.rendered_astra_frames = raw

        self.vars['qr'].set(self.ros_node.latest_qr_text or 'Sin QR detectado')

    # ─── Popup modal de detección / QR ───────────────────────────────────────

    def _check_popup_queue(self):
        if self._popup_open or not self.ros_node._popup_queue:
            return
        self._popup_open = True
        item = self.ros_node._popup_queue.pop(0)
        if item['kind'] == 'detection':
            self._show_detection_popup(item)
        elif item['kind'] == 'qr':
            self._show_qr_popup(item)

    def _make_popup(self, title):
        popup = tk.Toplevel(self.root)
        popup.title(title)
        popup.configure(bg=COLORS['bg'])
        popup.resizable(False, False)
        popup.transient(self.root)
        popup.grab_set()
        popup.focus_force()
        return popup

    def _center_popup(self, popup):
        popup.update_idletasks()
        sw = self.root.winfo_screenwidth()
        sh = self.root.winfo_screenheight()
        w = popup.winfo_reqwidth()
        h = popup.winfo_reqheight()
        popup.geometry(f'+{max(0, (sw - w) // 2)}+{max(0, (sh - h) // 2)}')

    def _show_detection_popup(self, item):
        det = item['det']
        det_type = det.get('type', '?')
        name = det.get('name', '?')
        confidence = det.get('confidence', None)
        frame = item.get('frame')

        TYPE_INFO = {
            'hazmat_sign': (COLORS['amber'],  COLORS['amber_bg'],  '⚠  HAZMAT DETECTADO'),
            'ar_code':     (COLORS['cyan'],   COLORS['blue_bg'],   '▣  CÓDIGO AR DETECTADO'),
            'real_object': (COLORS['green'],  COLORS['green_bg'],  '◎  OBJETO DETECTADO'),
        }
        fg, bg, header_text = TYPE_INFO.get(
            det_type, (COLORS['text'], COLORS['surface_high'], 'DETECCIÓN'))

        popup = self._make_popup('Detección')

        tk.Label(popup, text=header_text, bg=bg, fg=fg,
                 font=(FONT, 16, 'bold'), padx=30, pady=14).pack(fill='x')

        tk.Label(popup, text=name, bg=COLORS['bg'], fg=COLORS['text'],
                 font=(FONT, 32, 'bold'), pady=8).pack()

        if confidence is not None:
            tk.Label(popup, text=f'Confianza: {confidence * 100:.0f}%',
                     bg=COLORS['bg'], fg=COLORS['muted'],
                     font=(FONT, 12)).pack()

        if frame is not None:
            try:
                png_data = bgr_frame_to_png_data(frame, max_width=500, max_height=320)
                if png_data:
                    photo = tk.PhotoImage(data=png_data, format='png')
                    img_lbl = tk.Label(popup, image=photo, bg=COLORS['bg'], pady=6)
                    img_lbl.image = photo
                    img_lbl.pack()
            except Exception:
                pass

        def close():
            self._popup_open = False
            popup.grab_release()
            popup.destroy()

        popup.protocol('WM_DELETE_WINDOW', close)
        tk.Button(popup, text='CONTINUAR  ▶', bg=fg, fg=COLORS['black'],
                  font=(FONT, 14, 'bold'), relief='flat', bd=0,
                  padx=40, pady=14, cursor='hand2',
                  command=close).pack(pady=(8, 20))

        self._center_popup(popup)

    def _show_qr_popup(self, item):
        text = item['text']
        frame = item.get('frame')

        popup = self._make_popup('QR Detectado')

        tk.Label(popup, text='▣  QR DETECTADO', bg=COLORS['blue_bg'], fg=COLORS['cyan'],
                 font=(FONT, 16, 'bold'), padx=30, pady=14).pack(fill='x')

        # Try to decode base64 image payload (data URL format)
        qr_image_photo = None
        try:
            if text.startswith('data:image/'):
                _, b64data = text.split(',', 1)
                import numpy as _np
                import cv2 as _cv2
                arr = _np.frombuffer(base64.b64decode(b64data), dtype=_np.uint8)
                decoded_img = _cv2.imdecode(arr, _cv2.IMREAD_COLOR)
                if decoded_img is not None:
                    png_data = bgr_frame_to_png_data(decoded_img, max_width=420, max_height=300)
                    if png_data:
                        qr_image_photo = tk.PhotoImage(data=png_data, format='png')
        except Exception:
            pass

        if qr_image_photo:
            img_lbl = tk.Label(popup, image=qr_image_photo, bg=COLORS['bg'], pady=10)
            img_lbl.image = qr_image_photo
            img_lbl.pack()
            tk.Label(popup, text='Imagen decodificada del QR',
                     bg=COLORS['bg'], fg=COLORS['muted'], font=(FONT, 9)).pack()
        else:
            content = tk.Frame(popup, bg=COLORS['surface_high'], padx=20, pady=16)
            content.pack(fill='x', padx=20, pady=(10, 0))
            tk.Label(content, text=text, bg=COLORS['surface_high'], fg=COLORS['text'],
                     font=(FONT, 13), wraplength=520, justify='left').pack()

        if frame is not None:
            try:
                png_data = bgr_frame_to_png_data(frame, max_width=420, max_height=260)
                if png_data:
                    photo = tk.PhotoImage(data=png_data, format='png')
                    tk.Label(popup, text='Imagen de la cámara:',
                             bg=COLORS['bg'], fg=COLORS['muted'],
                             font=(FONT, 9)).pack(pady=(10, 2))
                    img_lbl2 = tk.Label(popup, image=photo, bg=COLORS['bg'])
                    img_lbl2.image = photo
                    img_lbl2.pack()
            except Exception:
                pass

        def close():
            self._popup_open = False
            popup.grab_release()
            popup.destroy()

        popup.protocol('WM_DELETE_WINDOW', close)
        tk.Button(popup, text='CERRAR  ✕', bg=COLORS['cyan'], fg=COLORS['black'],
                  font=(FONT, 14, 'bold'), relief='flat', bd=0,
                  padx=40, pady=14, cursor='hand2',
                  command=close).pack(pady=(12, 20))

        self._center_popup(popup)

    def update_video_image(self, frame, label, photo_attr, max_w, max_h):
        if frame is None:
            return
        w = label.winfo_width()
        h = label.winfo_height()
        target_w = w if w > 10 else max_w
        target_h = h if h > 10 else max_h
        png_data = bgr_frame_to_png_data(frame, max_width=target_w, max_height=target_h)
        if png_data is None:
            return
        photo = tk.PhotoImage(data=png_data, format='png')
        setattr(self, photo_attr, photo)
        label.configure(image=photo, text='')

    # ─── Drawing ──────────────────────────────────────────────────────────

    def update_status_pill(self, status_text):
        color = COLORS['green'] if status_text == 'CONTROL ACTIVO' else COLORS['amber']
        if status_text == 'SIN DATOS':
            color = COLORS['red']
        self.status_pill.configure(text=status_text, fg=color)

    def draw_gauge(self, canvas, value):
        canvas.delete('all')
        w = max(canvas.winfo_width(), 290)
        value = max(0.0, min(float(value), 1.0))
        canvas.create_rectangle(0, 4, w, 14, fill=COLORS['surface_soft'], outline='')
        canvas.create_rectangle(0, 4, w * value, 14, fill=canvas.accent, outline='')

    def draw_tracks(self, left_track, right_track):
        canvas   = self.track_canvas
        canvas.delete('all')
        width    = max(canvas.winfo_width(), 290)
        center_x = width / 2
        self._draw_track_bar(canvas, center_x, 26, left_track, 'Izq')
        self._draw_track_bar(canvas, center_x, 74, right_track, 'Der')

    def _draw_track_bar(self, canvas, center_x, y, value, label):
        bar_w  = max(canvas.winfo_width(), 290) - 92
        half   = bar_w / 2
        x0, x1 = center_x - half, center_x + half
        color  = COLORS['green'] if value >= 0.0 else COLORS['amber']
        canvas.create_text(20, y, text=label, fill=COLORS['muted'], font=(FONT, 9, 'bold'), anchor='w')
        canvas.create_rectangle(x0, y-8, x1, y+8, fill=COLORS['surface_soft'], outline='')
        canvas.create_line(center_x, y-12, center_x, y+12, fill=COLORS['border'])
        fe = center_x + (half * value)
        canvas.create_rectangle(min(center_x, fe), y-8, max(center_x, fe), y+8, fill=color, outline='')


# ─── Entry point ─────────────────────────────────────────────────────────────

def main(args=None):
    rclpy.init(args=args)
    ros_node = DashboardRosNode()

    root = tk.Tk()
    app = ModernDashboardApp(root, ros_node)

    def request_close(_signum=None, _frame=None):
        try:
            root.after(0, root.quit)
        except tk.TclError:
            pass

    signal.signal(signal.SIGINT, request_close)
    signal.signal(signal.SIGTERM, request_close)

    def spin_ros():
        if not rclpy.ok():
            request_close()
            return
        try:
            for _ in range(8):
                rclpy.spin_once(ros_node, timeout_sec=0.0)
            root.after(10, spin_ros)
        except (KeyboardInterrupt, tk.TclError):
            request_close()

    root.after(20, spin_ros)

    try:
        root.mainloop()
    finally:
        # Bajar la GUI del brazo precargada (todo su grupo de proceso).
        arm_proc = getattr(app, '_arm_proc', None)
        if arm_proc is not None and arm_proc.poll() is None:
            try:
                os.killpg(os.getpgid(arm_proc.pid), signal.SIGINT)
            except Exception:
                pass
        try:
            ros_node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()
