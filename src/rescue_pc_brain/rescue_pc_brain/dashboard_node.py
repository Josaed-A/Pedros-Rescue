import json
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32


class DashboardRosNode(Node):
    def __init__(self):
        super().__init__('drive_dashboard_node')

        self.status = {
            'gear': 1,
            'direction': 'FORWARD',
            'target_speed': 0.0,
            'real_speed_abs': 0.0,
            'linear_x': 0.0,
            'angular_z': 0.0,
            'movement_enabled': False,
            'status_text': 'SIN DATOS',
            'r2': 0.0,
            'joy_x': 0.0,
            'joy_y': 0.0,
            'l1_pressed': 0,
            'l2_pressed': 0,
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
        self.root.geometry('520x460')

        self.gear_var = tk.StringVar()
        self.direction_var = tk.StringVar()
        self.status_var = tk.StringVar()
        self.target_speed_var = tk.StringVar()
        self.real_speed_var = tk.StringVar()
        self.linear_var = tk.StringVar()
        self.angular_var = tk.StringVar()
        self.r2_var = tk.StringVar()
        self.joy_var = tk.StringVar()
        self.l1_var = tk.StringVar()
        self.l2_var = tk.StringVar()

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

        ttk.Label(top_frame, text='Dirección:', font=('Arial', 12, 'bold')).grid(row=1, column=0, sticky='w')
        ttk.Label(top_frame, textvariable=self.direction_var, font=('Arial', 12)).grid(row=1, column=1, sticky='w', padx=8)

        ttk.Label(top_frame, text='Estado:', font=('Arial', 12, 'bold')).grid(row=2, column=0, sticky='w')
        ttk.Label(top_frame, textvariable=self.status_var, font=('Arial', 12)).grid(row=2, column=1, sticky='w', padx=8)

        ttk.Separator(main_frame).pack(fill='x', pady=12)

        ttk.Label(main_frame, text='Velocidad deseada', font=('Arial', 12, 'bold')).pack(anchor='w')
        self.target_bar = ttk.Progressbar(
            main_frame,
            orient='horizontal',
            length=440,
            mode='determinate',
            maximum=100
        )
        self.target_bar.pack(anchor='w', pady=4)

        ttk.Label(main_frame, textvariable=self.target_speed_var).pack(anchor='w')

        ttk.Label(main_frame, text='Velocidad real motores', font=('Arial', 12, 'bold')).pack(anchor='w', pady=(14, 0))
        self.real_bar = ttk.Progressbar(
            main_frame,
            orient='horizontal',
            length=440,
            mode='determinate',
            maximum=100
        )
        self.real_bar.pack(anchor='w', pady=4)

        ttk.Label(main_frame, textvariable=self.real_speed_var).pack(anchor='w')

        ttk.Separator(main_frame).pack(fill='x', pady=12)

        data_frame = ttk.Frame(main_frame)
        data_frame.pack(fill='x')

        ttk.Label(data_frame, textvariable=self.linear_var).grid(row=0, column=0, sticky='w', padx=4, pady=2)
        ttk.Label(data_frame, textvariable=self.angular_var).grid(row=1, column=0, sticky='w', padx=4, pady=2)
        ttk.Label(data_frame, textvariable=self.r2_var).grid(row=2, column=0, sticky='w', padx=4, pady=2)
        ttk.Label(data_frame, textvariable=self.joy_var).grid(row=3, column=0, sticky='w', padx=4, pady=2)
        ttk.Label(data_frame, textvariable=self.l1_var).grid(row=4, column=0, sticky='w', padx=4, pady=2)
        ttk.Label(data_frame, textvariable=self.l2_var).grid(row=5, column=0, sticky='w', padx=4, pady=2)

        ttk.Separator(main_frame).pack(fill='x', pady=12)

        help_text = (
            'Controles: L1 habilita | R2 velocidad | L2+Triángulo sube caja | '
            'L2+Círculo baja caja | L2+X cambia avance/reversa'
        )

        ttk.Label(
            main_frame,
            text=help_text,
            wraplength=470,
            font=('Arial', 9)
        ).pack(anchor='w')

    def refresh_ui(self):
        status = self.ros_node.status

        gear = status.get('gear', 1)
        direction = status.get('direction', 'FORWARD')
        status_text = status.get('status_text', 'SIN DATOS')

        target_speed = float(status.get('target_speed', 0.0))
        real_speed = float(status.get('real_speed_abs', 0.0))

        linear_x = float(status.get('linear_x', 0.0))
        angular_z = float(status.get('angular_z', 0.0))

        r2 = float(status.get('r2', 0.0))
        joy_x = float(status.get('joy_x', 0.0))
        joy_y = float(status.get('joy_y', 0.0))

        l1 = int(status.get('l1_pressed', 0))
        l2 = int(status.get('l2_pressed', 0))

        self.gear_var.set(str(gear))
        self.direction_var.set(direction)
        self.status_var.set(status_text)

        self.target_bar['value'] = target_speed * 100.0
        self.real_bar['value'] = real_speed * 100.0

        self.target_speed_var.set(f'Deseada: {target_speed * 100.0:.1f}%')
        self.real_speed_var.set(f'Real: {real_speed * 100.0:.1f}%')

        self.linear_var.set(f'linear.x: {linear_x:.3f}')
        self.angular_var.set(f'angular.z: {angular_z:.3f}')
        self.r2_var.set(f'R2: {r2:.3f}')
        self.joy_var.set(f'Joystick X: {joy_x:.3f} | Joystick Y: {joy_y:.3f}')
        self.l1_var.set(f'L1 habilitación: {l1}')
        self.l2_var.set(f'L2 modificador: {l2}')

        self.root.after(100, self.refresh_ui)


def main(args=None):
    rclpy.init(args=args)

    ros_node = DashboardRosNode()

    root = tk.Tk()
    app = DriveDashboardApp(root, ros_node)

    def spin_ros():
        rclpy.spin_once(ros_node, timeout_sec=0.0)
        root.after(20, spin_ros)

    root.after(20, spin_ros)

    try:
        root.mainloop()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()