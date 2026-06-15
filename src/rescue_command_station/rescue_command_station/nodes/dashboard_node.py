import csv
import datetime
import json
import os
import subprocess
import tkinter as tk
from tkinter import messagebox, ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, PointCloud2
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

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
OUTPUT_DIR = '/workspace/maps'

DET_TYPE_OPTIONS = ['ar_code', 'hazmat_sign', 'real_object']

_RVIZ_BASH = (
    '. /opt/ros/jazzy/setup.bash && '
    '. /workspace/install/setup.bash && '
    'rviz2 -d /workspace/src/rescue_bringup/config/slam_rviz.rviz'
)
_PODMAN_SOCK = '/tmp/podman.sock'


# ─── ROS Node ────────────────────────────────────────────────────────────────

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

        self.get_logger().info('Dashboard iniciado — vision, mapeo y control')

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
        except Exception:
            pass

    def call_service_async(self, client, on_result):
        if not client.service_is_ready():
            on_result(False, 'Servicio no disponible (no iniciado?)')
            return None
        future = client.call_async(Trigger.Request())
        return future


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
            'manual_type':  tk.StringVar(value=DET_TYPE_OPTIONS[0]),
            'manual_camera': tk.StringVar(value='Astra'),
        }

        self.build_ui()
        self.refresh_ui()

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

        # ── Deteccion manual ──────────────────────────────────────
        tk.Frame(panel, bg=COLORS['border'], height=1).pack(fill='x', pady=(10, 6))

        tk.Label(panel, text='DETECCION MANUAL', bg=COLORS['surface'],
                 fg=COLORS['cyan'], font=(FONT, 8, 'bold')).pack(anchor='w', pady=(0, 6))

        cam_row = tk.Frame(panel, bg=COLORS['surface'])
        cam_row.pack(fill='x', pady=(0, 4))
        tk.Label(cam_row, text='Camara:', bg=COLORS['surface'], fg=COLORS['muted'],
                 font=(FONT, 9), width=8, anchor='e').pack(side='left')
        for cam in ('Logitech', 'Astra'):
            tk.Radiobutton(cam_row, text=cam, variable=self.vars['manual_camera'],
                           value=cam, bg=COLORS['surface'], fg=COLORS['text'],
                           selectcolor=COLORS['surface_soft'],
                           activebackground=COLORS['surface'],
                           font=(FONT, 9)).pack(side='left', padx=(8, 0))

        type_row = tk.Frame(panel, bg=COLORS['surface'])
        type_row.pack(fill='x', pady=(0, 4))
        tk.Label(type_row, text='Tipo:', bg=COLORS['surface'], fg=COLORS['muted'],
                 font=(FONT, 9), width=8, anchor='e').pack(side='left')
        self._type_menu = tk.OptionMenu(type_row, self.vars['manual_type'], *DET_TYPE_OPTIONS)
        self._type_menu.configure(bg=COLORS['surface_high'], fg=COLORS['text'],
                                  activebackground=COLORS['surface_soft'],
                                  font=(FONT, 9), bd=0, highlightthickness=0,
                                  relief='flat', width=14)
        self._type_menu['menu'].configure(bg=COLORS['surface_high'], fg=COLORS['text'])
        self._type_menu.pack(side='left', padx=(6, 0), fill='x', expand=True)

        name_row = tk.Frame(panel, bg=COLORS['surface'])
        name_row.pack(fill='x', pady=(0, 8))
        tk.Label(name_row, text='Nombre:', bg=COLORS['surface'], fg=COLORS['muted'],
                 font=(FONT, 9), width=8, anchor='e').pack(side='left')
        self._manual_name_entry = tk.Entry(
            name_row, bg=COLORS['surface_high'], fg=COLORS['text'],
            insertbackground=COLORS['text'], font=(FONT, 10), relief='flat', bd=4)
        self._manual_name_entry.pack(side='left', padx=(6, 0), fill='x', expand=True)
        self._manual_name_entry.bind('<Return>', lambda _e: self._on_add_manual())

        btn_row = tk.Frame(panel, bg=COLORS['surface'])
        btn_row.pack(fill='x')
        tk.Button(btn_row, text='+ AGREGAR',
                  bg=COLORS['surface_btn'], fg=COLORS['cyan'],
                  font=(FONT, 10, 'bold'), relief='flat', bd=0,
                  pady=10, cursor='hand2',
                  command=self._on_add_manual).pack(side='left', fill='x', expand=True, padx=(0, 6))
        tk.Button(btn_row, text='GUARDAR CSV',
                  bg=COLORS['surface_btn'], fg=COLORS['amber'],
                  font=(FONT, 10, 'bold'), relief='flat', bd=0,
                  pady=10, cursor='hand2',
                  command=self._on_save_manual_csv).pack(side='left', fill='x', expand=True)

        tk.Label(panel, textvariable=self.vars['manual_count'],
                 bg=COLORS['surface'], fg=COLORS['muted'],
                 font=(FONT, 8), anchor='w').pack(fill='x', pady=(6, 0))

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
        display = os.environ.get('DISPLAY', ':0')
        cmd = f'DISPLAY={display} {_RVIZ_BASH}'

        if not os.path.exists(_PODMAN_SOCK):
            self.vars['save_status'].set(
                'RViz: socket no disponible. Relanza el dashboard con: '
                './scripts/run_slam_container.sh dashboard')
            self._save_status_label.configure(fg=COLORS['red'])
            return

        # Use Podman REST API via Unix socket to exec rviz2 in pedros_slam
        py = f"""
import socket, http.client, json, sys

class _U(http.client.HTTPConnection):
    def connect(self):
        self.sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        self.sock.connect('{_PODMAN_SOCK}')

def api(method, path, body=None):
    c = _U('localhost')
    h = {{'Content-Type': 'application/json'}} if body else {{}}
    c.request(method, path, json.dumps(body) if body else None, h)
    return json.loads(c.getresponse().read() or b'{{}}')

cmd = {json.dumps(cmd)}
r = api('POST', '/v4.0.0/libpod/containers/pedros_slam/exec',
        {{'AttachStdin': False, 'AttachStdout': False, 'AttachStderr': False,
          'Detach': True, 'Cmd': ['bash', '-c', cmd]}})
exec_id = r.get('Id', '')
if exec_id:
    api('POST', f'/v4.0.0/libpod/exec/{{exec_id}}/start', {{'Detach': True}})
    print('OK')
else:
    print('ERROR:', r, file=sys.stderr)
    sys.exit(1)
"""
        self._rviz_proc = subprocess.Popen(
            ['python3', '-c', py],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.btn_rviz.configure(text='RVIZ CORRIENDO  ●',
                                fg=COLORS['green'], bg=COLORS['green_bg'])

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
        }
        for det in dets[:25]:
            hora   = det.get('_time', '--:--:--')
            tipo   = TYPE_ABBREV.get(det.get('type', '?'), det.get('type', '?'))
            nombre = det.get('name', '?')
            tag    = '[M] ' if det.get('_manual') else ''
            self.det_tree.insert('', 'end', values=(hora, f'{tag}{tipo}', nombre))

    # ─── Manual detections ────────────────────────────────────────────────

    def _on_add_manual(self):
        det_type = self.vars['manual_type'].get()
        name     = self._manual_name_entry.get().strip()
        camera   = self.vars['manual_camera'].get()

        if not name:
            messagebox.showwarning('Nombre vacio', 'Escribe un nombre para la deteccion')
            return

        now    = datetime.datetime.now()
        t_str  = now.strftime('%H:%M:%S')
        record = {
            'detection': len(self.manual_detections) + 1,
            'time': t_str,
            'type': det_type,
            'name': name,
            'camera': camera,
            'x': 0.0, 'y': 0.0, 'z': 0.0,
            'robot': 'Pedro',
            'mode': 'manual',
        }
        self.manual_detections.append(record)
        self.vars['manual_count'].set(f'{len(self.manual_detections)} detecciones manuales')
        self._manual_name_entry.delete(0, 'end')

        # Inject into shared detection log so it appears in the table
        self.ros_node.latest_detections.insert(0, {
            'type': det_type, 'name': name,
            'wx': 0.0, 'wy': 0.0,
            '_manual': True, '_camera': camera,
            '_time': t_str,
        })

    def _on_save_manual_csv(self):
        if not self.manual_detections:
            messagebox.showinfo('Sin datos', 'No hay detecciones manuales que guardar')
            return
        out_dir = OUTPUT_DIR
        os.makedirs(out_dir, exist_ok=True)
        ts   = datetime.datetime.now().strftime('%H-%M-%S')
        path = os.path.join(out_dir, f'RoboCup2026-{TEAM_NAME}-manual-{ts}-pois.csv')
        fieldnames = ['detection', 'time', 'type', 'name', 'camera', 'x', 'y', 'z', 'robot', 'mode']
        try:
            with open(path, 'w', newline='') as f:
                writer = csv.DictWriter(f, fieldnames=fieldnames)
                writer.writeheader()
                writer.writerows(self.manual_detections)
            messagebox.showinfo('Guardado', f'CSV guardado en:\n{path}')
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

    def update_video_image(self, frame, label, photo_attr, max_w, max_h):
        if frame is None:
            return
        png_data = bgr_frame_to_png_data(frame, max_width=max_w, max_height=max_h)
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
    ModernDashboardApp(root, ros_node)

    def spin_ros():
        for _ in range(8):
            rclpy.spin_once(ros_node, timeout_sec=0.0)
        root.after(10, spin_ros)

    root.after(20, spin_ros)

    try:
        root.mainloop()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
