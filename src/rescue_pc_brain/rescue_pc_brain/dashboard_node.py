import json
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32, String


class DashboardRosNode(Node):
    def __init__(self):
        super().__init__('drive_dashboard_node')

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

        self.drive_status_subscription = self.create_subscription(
            String,
            '/drive_status',
            self.drive_status_callback,
            10
        )

        self.real_speed_subscription = self.create_subscription(
            Float32,
            '/real_speed_abs',
            self.real_speed_callback,
            10
        )

    def drive_status_callback(self, msg):
        try:
            data = json.loads(msg.data)
            self.status.update(data)
        except json.JSONDecodeError:
            self.get_logger().warn('No se pudo leer JSON de /drive_status')

    def real_speed_callback(self, msg):
        self.status['real_speed_abs'] = msg.data


class DriveDashboardApp:
    def __init__(self, root, ros_node):
        self.root = root
        self.ros_node = ros_node

        self.root.title('Rescue Robot Dashboard')
        self.root.geometry('540x470')

        self.gear_var = tk.StringVar()
        self.gear_limit_var = tk.StringVar()
        self.status_var = tk.StringVar()
        self.target_speed_var = tk.StringVar()
        self.real_speed_var = tk.StringVar()
        self.left_track_var = tk.StringVar()
        self.right_track_var = tk.StringVar()
        self.linear_var = tk.StringVar()
        self.angular_var = tk.StringVar()
        self.joy_var = tk.StringVar()
        self.l1_var = tk.StringVar()
        self.r1_var = tk.StringVar()

        self.build_ui()
        self.refresh_ui()

    def build_ui(self):
        main_frame = ttk.Frame(self.root, padding=16)
        main_frame.pack(fill='both', expand=True)

        title = ttk.Label(
            main_frame,
            text='Rescue Robot - Panel de Control',
            font=('Arial', 18, 'bold')
        )
        title.pack(pady=(0, 16))

        top_frame = ttk.Frame(main_frame)
        top_frame.pack(fill='x', pady=8)

        ttk.Label(top_frame, text='Caja:', font=('Arial', 12, 'bold')).grid(row=0, column=0, sticky='w')
        ttk.Label(top_frame, textvariable=self.gear_var, font=('Arial', 12)).grid(row=0, column=1, sticky='w', padx=8)

        ttk.Label(top_frame, text='Limite:', font=('Arial', 12, 'bold')).grid(row=1, column=0, sticky='w')
        ttk.Label(top_frame, textvariable=self.gear_limit_var, font=('Arial', 12)).grid(row=1, column=1, sticky='w', padx=8)

        ttk.Label(top_frame, text='Estado:', font=('Arial', 12, 'bold')).grid(row=2, column=0, sticky='w')
        ttk.Label(top_frame, textvariable=self.status_var, font=('Arial', 12)).grid(row=2, column=1, sticky='w', padx=8)

        ttk.Separator(main_frame).pack(fill='x', pady=12)

        ttk.Label(main_frame, text='Velocidad objetivo', font=('Arial', 12, 'bold')).pack(anchor='w')
        self.target_bar = ttk.Progressbar(
            main_frame,
            orient='horizontal',
            length=460,
            mode='determinate',
            maximum=100
        )
        self.target_bar.pack(anchor='w', pady=4)

        ttk.Label(main_frame, textvariable=self.target_speed_var).pack(anchor='w')

        ttk.Label(main_frame, text='Velocidad real motores', font=('Arial', 12, 'bold')).pack(anchor='w', pady=(14, 0))
        self.real_bar = ttk.Progressbar(
            main_frame,
            orient='horizontal',
            length=460,
            mode='determinate',
            maximum=100
        )
        self.real_bar.pack(anchor='w', pady=4)

        ttk.Label(main_frame, textvariable=self.real_speed_var).pack(anchor='w')

        ttk.Separator(main_frame).pack(fill='x', pady=12)

        data_frame = ttk.Frame(main_frame)
        data_frame.pack(fill='x')

        ttk.Label(data_frame, textvariable=self.left_track_var).grid(row=0, column=0, sticky='w', padx=4, pady=2)
        ttk.Label(data_frame, textvariable=self.right_track_var).grid(row=1, column=0, sticky='w', padx=4, pady=2)
        ttk.Label(data_frame, textvariable=self.linear_var).grid(row=2, column=0, sticky='w', padx=4, pady=2)
        ttk.Label(data_frame, textvariable=self.angular_var).grid(row=3, column=0, sticky='w', padx=4, pady=2)
        ttk.Label(data_frame, textvariable=self.joy_var).grid(row=4, column=0, sticky='w', padx=4, pady=2)
        ttk.Label(data_frame, textvariable=self.r1_var).grid(row=5, column=0, sticky='w', padx=4, pady=2)
        ttk.Label(data_frame, textvariable=self.l1_var).grid(row=6, column=0, sticky='w', padx=4, pady=2)

        ttk.Separator(main_frame).pack(fill='x', pady=12)

        help_text = (
            'Controles: joystick izquierdo mueve orugas | R1 sube caja | '
            'L1 baja caja'
        )

        ttk.Label(
            main_frame,
            text=help_text,
            wraplength=490,
            font=('Arial', 9)
        ).pack(anchor='w')

    def refresh_ui(self):
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

        self.gear_var.set(str(gear))
        self.gear_limit_var.set(f'{gear_limit * 100.0:.0f}%')
        self.status_var.set(status_text)

        self.target_bar['value'] = target_speed * 100.0
        self.real_bar['value'] = real_speed * 100.0

        self.target_speed_var.set(f'Objetivo: {target_speed * 100.0:.1f}%')
        self.real_speed_var.set(f'Real: {real_speed * 100.0:.1f}%')

        self.left_track_var.set(f'Oruga izquierda: {left_track:.3f}')
        self.right_track_var.set(f'Oruga derecha: {right_track:.3f}')
        self.linear_var.set(f'linear.x: {linear_x:.3f}')
        self.angular_var.set(f'angular.z: {angular_z:.3f}')
        self.joy_var.set(f'Joystick X: {joy_x:.3f} | Joystick Y: {joy_y:.3f}')
        self.r1_var.set(f'R1 sube caja: {r1}')
        self.l1_var.set(f'L1 baja caja: {l1}')

        self.root.after(100, self.refresh_ui)


def main(args=None):
    rclpy.init(args=args)

    ros_node = DashboardRosNode()

    root = tk.Tk()
    DriveDashboardApp(root, ros_node)

    def spin_ros():
        rclpy.spin_once(ros_node, timeout_sec=0.0)
        root.after(20, spin_ros)

    root.after(20, spin_ros)

    try:
        root.mainloop()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
