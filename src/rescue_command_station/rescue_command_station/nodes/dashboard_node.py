import json
import tkinter as tk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, PointCloud2
from std_msgs.msg import Float32, String

from rescue_command_station.vision.qr_detector import QrDetector
from rescue_command_station.vision.ros_image import compressed_msg_to_numpy
from rescue_command_station.vision.tk_image import bgr_frame_to_png_data, depth_frame_to_color


COLORS = {
    'bg': '#0b0f14',
    'surface': '#111821',
    'surface_high': '#17212d',
    'surface_soft': '#1c2733',
    'border': '#263544',
    'text': '#e7edf4',
    'muted': '#94a3b8',
    'cyan': '#38bdf8',
    'green': '#22c55e',
    'amber': '#f59e0b',
    'red': '#ef4444',
    'blue': '#60a5fa',
    'black': '#030712',
}

FONT = 'Segoe UI'


class DashboardRosNode(Node):
    def __init__(self):
        super().__init__('drive_dashboard_node')

        self.declare_parameter('front_camera_topic', '/robot/camera/front/image_raw/compressed')
        self.declare_parameter('astra_color_topic', '/robot/camera/astra/color/image_raw/compressed')
        self.declare_parameter('astra_depth_topic', '/robot/camera/astra/depth/image_raw/compressed')
        self.declare_parameter('point_cloud_topic', '/robot/camera/astra/points')
        self.declare_parameter('raspberry_timeout_seconds', 2.5)

        self.front_camera_topic = self.get_parameter('front_camera_topic').value
        self.astra_color_topic = self.get_parameter('astra_color_topic').value
        self.astra_depth_topic = self.get_parameter('astra_depth_topic').value
        self.point_cloud_topic = self.get_parameter('point_cloud_topic').value
        self.raspberry_timeout_seconds = float(
            self.get_parameter('raspberry_timeout_seconds').value
        )

        self.qr_detector = QrDetector()

        self.latest_front_frame = None
        self.latest_astra_color_frame = None
        self.latest_astra_depth_frame = None
        self.latest_qr_text = ''
        self.front_camera_frames = 0
        self.astra_color_frames = 0
        self.astra_depth_frames = 0
        self.point_cloud_frames = 0
        self.last_point_cloud_width = 0
        self.last_point_cloud_frame_id = ''
        self.last_qr_scan_time = 0.0
        self.qr_scan_interval = 0.25
        self.last_raspberry_msg_time = None
        self.last_raspberry_source = 'sin datos'

        self.status = {
            'gear': 1,
            'gear_limit': 0.20,
            'target_speed': 0.0,
            'real_speed_abs': 0.0,
            'linear_x': 0.0,
            'angular_z': 0.0,
            'left_track': 0.0,
            'right_track': 0.0,
            'status_text': 'SIN DATOS',
            'joy_x': 0.0,
            'joy_y': 0.0,
            'l1_pressed': 0,
            'r1_pressed': 0,
        }

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(String, '/drive_status', self.drive_status_callback, 10)
        self.create_subscription(Float32, '/real_speed_abs', self.real_speed_callback, 10)
        self.create_subscription(CompressedImage, self.front_camera_topic, self.front_camera_callback, sensor_qos)
        self.create_subscription(CompressedImage, self.astra_color_topic, self.astra_color_callback, sensor_qos)
        self.create_subscription(CompressedImage, self.astra_depth_topic, self.astra_depth_callback, sensor_qos)
        self.create_subscription(PointCloud2, self.point_cloud_topic, self.point_cloud_callback, sensor_qos)

        self.get_logger().info(f'Dashboard escuchando camara frontal en {self.front_camera_topic}')
        self.get_logger().info(f'Dashboard escuchando Astra color en {self.astra_color_topic}')
        self.get_logger().info(f'Dashboard escuchando Astra profundidad en {self.astra_depth_topic}')
        self.get_logger().info(f'Dashboard escuchando nube de puntos en {self.point_cloud_topic}')

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

    def drive_status_callback(self, msg):
        try:
            self.status.update(json.loads(msg.data))
        except json.JSONDecodeError:
            self.get_logger().warn('No se pudo leer JSON de /drive_status')

    def real_speed_callback(self, msg):
        self.mark_raspberry_seen('/real_speed_abs')
        self.status['real_speed_abs'] = msg.data

    def front_camera_callback(self, msg):
        try:
            self.mark_raspberry_seen(self.front_camera_topic)
            frame = compressed_msg_to_numpy(msg)
            annotated_frame = frame
            now = self.now_seconds()

            if now - self.last_qr_scan_time >= self.qr_scan_interval:
                self.last_qr_scan_time = now
                annotated_frame, qr_text = self.qr_detector.detect_and_annotate(frame)

                if qr_text:
                    self.latest_qr_text = qr_text
                    self.get_logger().info(f'[QR DETECTADO]: {qr_text}')

            self.latest_front_frame = annotated_frame
            self.front_camera_frames += 1
        except Exception as exc:
            self.get_logger().warn(f'No se pudo procesar camara frontal: {exc}')

    def astra_color_callback(self, msg):
        try:
            self.mark_raspberry_seen(self.astra_color_topic)
            self.latest_astra_color_frame = compressed_msg_to_numpy(msg)
            self.astra_color_frames += 1
        except Exception as exc:
            self.get_logger().warn(f'No se pudo procesar color Astra: {exc}')

    def astra_depth_callback(self, msg):
        try:
            self.mark_raspberry_seen(self.astra_depth_topic)
            depth_frame = compressed_msg_to_numpy(msg, depth=True)
            self.latest_astra_depth_frame = depth_frame_to_color(depth_frame)
            self.astra_depth_frames += 1
        except Exception as exc:
            self.get_logger().warn(f'No se pudo procesar profundidad Astra: {exc}')

    def point_cloud_callback(self, msg):
        self.mark_raspberry_seen(self.point_cloud_topic)
        self.point_cloud_frames += 1
        self.last_point_cloud_width = msg.width
        self.last_point_cloud_frame_id = msg.header.frame_id


class ModernDashboardApp:
    def __init__(self, root, ros_node):
        self.root = root
        self.ros_node = ros_node
        self.front_camera_photo = None
        self.astra_color_photo = None
        self.astra_depth_photo = None
        self.rendered_front_camera_frames = -1
        self.rendered_astra_color_frames = -1
        self.rendered_astra_depth_frames = -1

        self.root.title('Pedro Rescue - Estacion de Mando')
        self.root.geometry('1180x720')
        self.root.minsize(1040, 640)
        self.root.configure(bg=COLORS['bg'])

        self.vars = {
            'gear': tk.StringVar(),
            'gear_limit': tk.StringVar(),
            'status': tk.StringVar(),
            'target_speed': tk.StringVar(),
            'real_speed': tk.StringVar(),
            'left_track': tk.StringVar(),
            'right_track': tk.StringVar(),
            'linear': tk.StringVar(),
            'angular': tk.StringVar(),
            'joystick': tk.StringVar(),
            'shift': tk.StringVar(),
            'front_camera': tk.StringVar(),
            'astra_color': tk.StringVar(),
            'astra_depth': tk.StringVar(),
            'qr': tk.StringVar(),
            'point_cloud': tk.StringVar(),
            'raspberry': tk.StringVar(),
        }

        self.build_ui()
        self.refresh_ui()

    def build_ui(self):
        self.root.grid_columnconfigure(0, weight=1)
        self.root.grid_rowconfigure(1, weight=1)

        self.build_header()

        body = tk.Frame(self.root, bg=COLORS['bg'])
        body.grid(row=1, column=0, sticky='nsew', padx=18, pady=(0, 18))
        body.grid_columnconfigure(0, weight=0, minsize=340)
        body.grid_columnconfigure(1, weight=1)
        body.grid_columnconfigure(2, weight=0, minsize=280)
        body.grid_rowconfigure(0, weight=1)

        self.build_drive_panel(body)
        self.build_video_panel(body)
        self.build_system_panel(body)

    def build_header(self):
        header = tk.Frame(self.root, bg=COLORS['bg'])
        header.grid(row=0, column=0, sticky='ew', padx=18, pady=18)
        header.grid_columnconfigure(0, weight=1)

        title_block = tk.Frame(header, bg=COLORS['bg'])
        title_block.grid(row=0, column=0, sticky='w')

        tk.Label(
            title_block,
            text='Pedro Rescue',
            bg=COLORS['bg'],
            fg=COLORS['text'],
            font=(FONT, 24, 'bold')
        ).pack(anchor='w')
        tk.Label(
            title_block,
            text='Estacion de mando | traccion, vision y telemetria',
            bg=COLORS['bg'],
            fg=COLORS['muted'],
            font=(FONT, 10)
        ).pack(anchor='w', pady=(2, 0))

        self.status_pill = tk.Label(
            header,
            text='SIN DATOS',
            bg=COLORS['surface_high'],
            fg=COLORS['muted'],
            font=(FONT, 10, 'bold'),
            padx=16,
            pady=8
        )
        self.status_pill.grid(row=0, column=1, sticky='e')

        self.raspberry_pill = tk.Label(
            header,
            text='RASP DESCONECTADA',
            bg=COLORS['surface_high'],
            fg=COLORS['red'],
            font=(FONT, 10, 'bold'),
            padx=16,
            pady=8
        )
        self.raspberry_pill.grid(row=0, column=2, sticky='e', padx=(10, 0))

    def build_drive_panel(self, parent):
        panel = self.card(parent)
        panel.grid(row=0, column=0, sticky='nsew', padx=(0, 14))

        self.card_title(panel, 'Control de orugas', 'Caja, velocidad y mezcla diferencial')

        gear_row = tk.Frame(panel, bg=COLORS['surface'])
        gear_row.pack(fill='x', pady=(12, 14))

        self.big_metric(gear_row, 'Caja', self.vars['gear'], COLORS['cyan']).pack(side='left', fill='x', expand=True, padx=(0, 8))
        self.big_metric(gear_row, 'Limite', self.vars['gear_limit'], COLORS['blue']).pack(side='left', fill='x', expand=True)

        self.target_canvas = self.gauge(panel, 'Velocidad objetivo', self.vars['target_speed'], COLORS['cyan'])
        self.real_canvas = self.gauge(panel, 'Velocidad real', self.vars['real_speed'], COLORS['green'])

        self.section(panel, 'Orugas')
        self.track_canvas = tk.Canvas(
            panel,
            width=300,
            height=104,
            bg=COLORS['surface'],
            highlightthickness=0
        )
        self.track_canvas.pack(fill='x', pady=(4, 10))

        self.small_label(panel, self.vars['left_track'])
        self.small_label(panel, self.vars['right_track'])

        self.section(panel, 'Comando ROS')
        self.small_label(panel, self.vars['linear'])
        self.small_label(panel, self.vars['angular'])
        self.small_label(panel, self.vars['joystick'])
        self.small_label(panel, self.vars['shift'])

    def build_video_panel(self, parent):
        panel = self.card(parent)
        panel.grid(row=0, column=1, sticky='nsew', padx=(0, 14))
        panel.grid_rowconfigure(2, weight=1)
        panel.grid_columnconfigure(0, weight=1)

        self.card_title(panel, 'Vision en vivo', 'Frontal con QR + Astra color/profundidad')

        video_grid = tk.Frame(panel, bg=COLORS['surface'])
        video_grid.pack(fill='both', expand=True, pady=(10, 0))
        video_grid.grid_columnconfigure(0, weight=3)
        video_grid.grid_columnconfigure(1, weight=2)
        video_grid.grid_rowconfigure(0, weight=1)

        front_panel = self.video_slot(
            video_grid,
            'Camara frontal',
            self.vars['front_camera'],
            'Esperando camara frontal...'
        )
        front_panel.grid(row=0, column=0, sticky='nsew', padx=(0, 10))
        self.front_camera_label = front_panel.image_label

        side_panel = tk.Frame(video_grid, bg=COLORS['surface'])
        side_panel.grid(row=0, column=1, sticky='nsew')
        side_panel.grid_rowconfigure(0, weight=1)
        side_panel.grid_rowconfigure(1, weight=1)
        side_panel.grid_columnconfigure(0, weight=1)

        astra_color_panel = self.video_slot(
            side_panel,
            'Astra color',
            self.vars['astra_color'],
            'Esperando Astra color...'
        )
        astra_color_panel.grid(row=0, column=0, sticky='nsew', pady=(0, 10))
        self.astra_color_label = astra_color_panel.image_label

        astra_depth_panel = self.video_slot(
            side_panel,
            'Astra profundidad',
            self.vars['astra_depth'],
            'Esperando profundidad...'
        )
        astra_depth_panel.grid(row=1, column=0, sticky='nsew')
        self.astra_depth_label = astra_depth_panel.image_label

        qr_box = tk.Frame(panel, bg=COLORS['surface_high'])
        qr_box.pack(fill='x', pady=(12, 0))

        tk.Label(
            qr_box,
            text='QR detectado',
            bg=COLORS['surface_high'],
            fg=COLORS['muted'],
            font=(FONT, 9, 'bold'),
            padx=12,
            pady=6
        ).pack(anchor='w')
        tk.Label(
            qr_box,
            textvariable=self.vars['qr'],
            bg=COLORS['surface_high'],
            fg=COLORS['text'],
            font=(FONT, 12, 'bold'),
            padx=12,
            pady=8,
            wraplength=460,
            justify='left'
        ).pack(anchor='w', fill='x')

    def video_slot(self, parent, title, status_var, waiting_text):
        frame = tk.Frame(parent, bg=COLORS['black'], highlightbackground=COLORS['border'], highlightthickness=1)
        frame.grid_columnconfigure(0, weight=1)
        frame.grid_rowconfigure(2, weight=1)

        tk.Label(
            frame,
            text=title,
            bg=COLORS['black'],
            fg=COLORS['text'],
            font=(FONT, 10, 'bold'),
            padx=10,
            pady=6
        ).grid(row=0, column=0, sticky='ew')
        tk.Label(
            frame,
            textvariable=status_var,
            bg=COLORS['black'],
            fg=COLORS['muted'],
            font=(FONT, 8),
            padx=10
        ).grid(row=1, column=0, sticky='ew')
        frame.image_label = tk.Label(
            frame,
            text=waiting_text,
            bg=COLORS['black'],
            fg=COLORS['muted'],
            font=(FONT, 11)
        )
        frame.image_label.grid(row=2, column=0, sticky='nsew')
        return frame

    def build_system_panel(self, parent):
        panel = self.card(parent)
        panel.grid(row=0, column=2, sticky='nsew')

        self.card_title(panel, 'Sistema', 'Vision 3D y estado general')

        self.section(panel, 'Estado')
        tk.Label(
            panel,
            textvariable=self.vars['raspberry'],
            bg=COLORS['surface_high'],
            fg=COLORS['text'],
            font=(FONT, 11, 'bold'),
            padx=12,
            pady=10,
            anchor='w',
            justify='left'
        ).pack(fill='x', pady=(4, 8))

        tk.Label(
            panel,
            textvariable=self.vars['status'],
            bg=COLORS['surface_high'],
            fg=COLORS['text'],
            font=(FONT, 12, 'bold'),
            padx=12,
            pady=10,
            anchor='w'
        ).pack(fill='x', pady=(4, 12))

        self.section(panel, 'Nube de puntos')
        tk.Label(
            panel,
            textvariable=self.vars['point_cloud'],
            bg=COLORS['surface'],
            fg=COLORS['muted'],
            font=(FONT, 10),
            justify='left',
            wraplength=230
        ).pack(anchor='w', fill='x')

        self.point_cloud_canvas = tk.Canvas(
            panel,
            width=230,
            height=120,
            bg=COLORS['surface'],
            highlightthickness=0
        )
        self.point_cloud_canvas.pack(fill='x', pady=(12, 16))

        self.section(panel, 'Atajos')
        for text in [
            'R1: subir caja',
            'L1: bajar caja',
            'Joystick: control tipo tanque',
            'Dashboard: video + QR',
            'Astra: depth image + PointCloud2',
        ]:
            self.small_static(panel, text)

    def card(self, parent):
        frame = tk.Frame(
            parent,
            bg=COLORS['surface'],
            highlightbackground=COLORS['border'],
            highlightthickness=1,
            padx=16,
            pady=16
        )
        return frame

    def card_title(self, parent, title, subtitle):
        tk.Label(
            parent,
            text=title,
            bg=COLORS['surface'],
            fg=COLORS['text'],
            font=(FONT, 15, 'bold')
        ).pack(anchor='w')
        tk.Label(
            parent,
            text=subtitle,
            bg=COLORS['surface'],
            fg=COLORS['muted'],
            font=(FONT, 9)
        ).pack(anchor='w', pady=(2, 0))

    def section(self, parent, title):
        tk.Label(
            parent,
            text=title.upper(),
            bg=COLORS['surface'],
            fg=COLORS['cyan'],
            font=(FONT, 8, 'bold')
        ).pack(anchor='w', pady=(14, 4))

    def big_metric(self, parent, label, variable, accent):
        frame = tk.Frame(parent, bg=COLORS['surface_high'], padx=12, pady=10)
        tk.Label(frame, text=label, bg=COLORS['surface_high'], fg=COLORS['muted'], font=(FONT, 8, 'bold')).pack(anchor='w')
        tk.Label(frame, textvariable=variable, bg=COLORS['surface_high'], fg=accent, font=(FONT, 22, 'bold')).pack(anchor='w')
        return frame

    def gauge(self, parent, title, variable, accent):
        self.section(parent, title)
        tk.Label(parent, textvariable=variable, bg=COLORS['surface'], fg=COLORS['text'], font=(FONT, 11, 'bold')).pack(anchor='w')
        canvas = tk.Canvas(parent, width=300, height=18, bg=COLORS['surface'], highlightthickness=0)
        canvas.pack(fill='x', pady=(5, 2))
        canvas.accent = accent
        return canvas

    def small_label(self, parent, variable):
        tk.Label(
            parent,
            textvariable=variable,
            bg=COLORS['surface'],
            fg=COLORS['muted'],
            font=(FONT, 10),
            anchor='w'
        ).pack(fill='x', pady=2)

    def small_static(self, parent, text):
        tk.Label(
            parent,
            text=text,
            bg=COLORS['surface'],
            fg=COLORS['muted'],
            font=(FONT, 10),
            anchor='w'
        ).pack(fill='x', pady=3)

    def refresh_ui(self):
        self.refresh_raspberry_status()
        self.refresh_drive_status()
        self.refresh_camera_status()
        self.refresh_point_cloud_status()
        self.root.after(50, self.refresh_ui)

    def refresh_raspberry_status(self):
        connected, age, source = self.ros_node.get_raspberry_connection()
        if connected:
            pill_text = 'RASP CONECTADA'
            detail_text = 'Raspberry: conectada'
            color = COLORS['green']
        else:
            pill_text = 'RASP DESCONECTADA'
            detail_text = 'Raspberry: desconectada'
            color = COLORS['red']

        self.vars['raspberry'].set(detail_text)
        self.raspberry_pill.configure(text=pill_text, fg=color)

    def refresh_drive_status(self):
        status = self.ros_node.status

        gear = int(status.get('gear', 1))
        gear_limit = float(status.get('gear_limit', 0.20))
        status_text = status.get('status_text', 'SIN DATOS')
        target_speed = float(status.get('target_speed', 0.0))
        real_speed = float(status.get('real_speed_abs', 0.0))
        left_track = float(status.get('left_track', 0.0))
        right_track = float(status.get('right_track', 0.0))
        linear_x = float(status.get('linear_x', 0.0))
        angular_z = float(status.get('angular_z', 0.0))
        joy_x = float(status.get('joy_x', 0.0))
        joy_y = float(status.get('joy_y', 0.0))
        l1 = int(status.get('l1_pressed', 0))
        r1 = int(status.get('r1_pressed', 0))

        self.vars['gear'].set(str(gear))
        self.vars['gear_limit'].set(f'{gear_limit * 100.0:.0f}%')
        self.vars['status'].set(status_text)
        self.vars['target_speed'].set(f'{target_speed * 100.0:.1f}%')
        self.vars['real_speed'].set(f'{real_speed * 100.0:.1f}%')
        self.vars['left_track'].set(f'Oruga izquierda: {left_track:.3f}')
        self.vars['right_track'].set(f'Oruga derecha: {right_track:.3f}')
        self.vars['linear'].set(f'linear.x: {linear_x:.3f}')
        self.vars['angular'].set(f'angular.z: {angular_z:.3f}')
        self.vars['joystick'].set(f'Joystick X: {joy_x:.3f} | Y: {joy_y:.3f}')
        self.vars['shift'].set(f'R1: {r1} | L1: {l1}')

        self.update_status_pill(status_text)
        self.draw_gauge(self.target_canvas, target_speed)
        self.draw_gauge(self.real_canvas, real_speed)
        self.draw_tracks(left_track, right_track)

    def refresh_camera_status(self):
        self.vars['front_camera'].set(
            f'{self.ros_node.front_camera_topic} | {self.ros_node.front_camera_frames} frames'
        )
        self.vars['astra_color'].set(
            f'{self.ros_node.astra_color_topic} | {self.ros_node.astra_color_frames} frames'
        )
        self.vars['astra_depth'].set(
            f'{self.ros_node.astra_depth_topic} | {self.ros_node.astra_depth_frames} frames'
        )
        self.vars['qr'].set(self.ros_node.latest_qr_text or 'Sin QR detectado')

        if self.ros_node.front_camera_frames != self.rendered_front_camera_frames:
            self.update_video_image(
                self.ros_node.latest_front_frame,
                self.front_camera_label,
                'front_camera_photo',
                560,
                390
            )
            self.rendered_front_camera_frames = self.ros_node.front_camera_frames

        if self.ros_node.astra_color_frames != self.rendered_astra_color_frames:
            self.update_video_image(
                self.ros_node.latest_astra_color_frame,
                self.astra_color_label,
                'astra_color_photo',
                330,
                170
            )
            self.rendered_astra_color_frames = self.ros_node.astra_color_frames

        if self.ros_node.astra_depth_frames != self.rendered_astra_depth_frames:
            self.update_video_image(
                self.ros_node.latest_astra_depth_frame,
                self.astra_depth_label,
                'astra_depth_photo',
                330,
                170
            )
            self.rendered_astra_depth_frames = self.ros_node.astra_depth_frames

    def update_video_image(self, frame, label, photo_attribute, max_width, max_height):
        if frame is None:
            return

        png_data = bgr_frame_to_png_data(frame, max_width=max_width, max_height=max_height)
        if png_data is None:
            return

        photo = tk.PhotoImage(data=png_data, format='png')
        setattr(self, photo_attribute, photo)
        label.configure(image=photo, text='')

    def refresh_point_cloud_status(self):
        text = (
            f'Topico: {self.ros_node.point_cloud_topic}\n'
            f'Nubes recibidas: {self.ros_node.point_cloud_frames}\n'
            f'Puntos ult. nube: {self.ros_node.last_point_cloud_width}\n'
            f'Frame: {self.ros_node.last_point_cloud_frame_id or "sin datos"}'
        )
        self.vars['point_cloud'].set(text)
        self.draw_point_cloud_status()

    def update_status_pill(self, status_text):
        color = COLORS['green'] if status_text == 'CONTROL ACTIVO' else COLORS['amber']
        if status_text == 'SIN DATOS':
            color = COLORS['red']

        self.status_pill.configure(text=status_text, fg=color)

    def draw_gauge(self, canvas, value):
        canvas.delete('all')
        width = max(canvas.winfo_width(), 300)
        height = 18
        value = max(0.0, min(float(value), 1.0))

        canvas.create_rectangle(0, 4, width, height - 4, fill=COLORS['surface_soft'], outline='')
        canvas.create_rectangle(0, 4, width * value, height - 4, fill=canvas.accent, outline='')

    def draw_tracks(self, left_track, right_track):
        canvas = self.track_canvas
        canvas.delete('all')
        width = max(canvas.winfo_width(), 300)
        center_x = width / 2

        self.draw_track_bar(canvas, center_x, 26, left_track, 'Izq')
        self.draw_track_bar(canvas, center_x, 74, right_track, 'Der')

    def draw_track_bar(self, canvas, center_x, y, value, label):
        bar_width = max(canvas.winfo_width(), 300) - 92
        half = bar_width / 2
        x0 = center_x - half
        x1 = center_x + half
        color = COLORS['green'] if value >= 0.0 else COLORS['amber']

        canvas.create_text(20, y, text=label, fill=COLORS['muted'], font=(FONT, 9, 'bold'), anchor='w')
        canvas.create_rectangle(x0, y - 8, x1, y + 8, fill=COLORS['surface_soft'], outline='')
        canvas.create_line(center_x, y - 12, center_x, y + 12, fill=COLORS['border'])
        fill_end = center_x + (half * value)
        fill_start = min(center_x, fill_end)
        fill_stop = max(center_x, fill_end)
        canvas.create_rectangle(fill_start, y - 8, fill_stop, y + 8, fill=color, outline='')

    def draw_point_cloud_status(self):
        canvas = self.point_cloud_canvas
        canvas.delete('all')
        width = max(canvas.winfo_width(), 230)
        height = 120
        frames = self.ros_node.point_cloud_frames
        color = COLORS['green'] if frames > 0 else COLORS['surface_soft']

        canvas.create_oval(18, 18, 102, 102, fill=COLORS['surface_high'], outline=COLORS['border'])
        canvas.create_oval(42, 42, 78, 78, fill=color, outline='')
        canvas.create_text(130, 44, text='PointCloud2', fill=COLORS['text'], font=(FONT, 11, 'bold'), anchor='w')
        canvas.create_text(
            130,
            70,
            text='Activo' if frames > 0 else 'Esperando datos',
            fill=color if frames > 0 else COLORS['muted'],
            font=(FONT, 10),
            anchor='w'
        )
        canvas.create_line(18, height - 10, width - 18, height - 10, fill=COLORS['border'])


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
