import tkinter as tk
from tkinter import ttk

import message_filters
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from rescue_command_station.vision.tk_image import (
    bgr_frame_to_png_data,
    depth_frame_to_color,
)


class RgbdViewerRosNode(Node):
    def __init__(self):
        super().__init__('rgbd_viewer_node')

        self.declare_parameter('depth_topic', '/robot/camera/astra/depth/image_raw')
        self.declare_parameter('color_topic', '/robot/camera/astra/color/image_raw')

        self.depth_topic = self.get_parameter('depth_topic').value
        self.color_topic = self.get_parameter('color_topic').value

        self.bridge = CvBridge()
        self.latest_color_frame = None
        self.latest_depth_frame = None
        self.frame_pairs = 0

        self.depth_subscriber = message_filters.Subscriber(self, Image, self.depth_topic)
        self.color_subscriber = message_filters.Subscriber(self, Image, self.color_topic)

        self.synchronizer = message_filters.ApproximateTimeSynchronizer(
            [self.depth_subscriber, self.color_subscriber],
            queue_size=10,
            slop=0.1
        )
        self.synchronizer.registerCallback(self.synchronized_callback)

        self.get_logger().info(f'Sincronizando RGB-D: {self.color_topic} + {self.depth_topic}')

    def synchronized_callback(self, depth_msg, color_msg):
        try:
            color_frame = self.bridge.imgmsg_to_cv2(color_msg, desired_encoding='bgr8')
            depth_frame = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='16UC1')

            self.latest_color_frame = color_frame
            self.latest_depth_frame = depth_frame_to_color(depth_frame)
            self.frame_pairs += 1
        except Exception as exc:
            self.get_logger().warn(f'No se pudo procesar par RGB-D: {exc}')


class RgbdViewerApp:
    def __init__(self, root, ros_node):
        self.root = root
        self.ros_node = ros_node
        self.color_photo = None
        self.depth_photo = None

        self.root.title('Pedro Rescue - Visor RGB-D')
        self.root.geometry('980x520')

        self.status_var = tk.StringVar()
        self.build_ui()
        self.refresh_ui()

    def build_ui(self):
        main_frame = ttk.Frame(self.root, padding=16)
        main_frame.pack(fill='both', expand=True)

        ttk.Label(
            main_frame,
            text='Visor RGB-D Astra',
            font=('Arial', 18, 'bold')
        ).pack(anchor='w', pady=(0, 8))

        ttk.Label(main_frame, textvariable=self.status_var).pack(anchor='w', pady=(0, 12))

        video_frame = ttk.Frame(main_frame)
        video_frame.pack(fill='both', expand=True)

        left = ttk.Frame(video_frame)
        left.grid(row=0, column=0, sticky='nsew', padx=(0, 12))

        right = ttk.Frame(video_frame)
        right.grid(row=0, column=1, sticky='nsew')

        video_frame.columnconfigure(0, weight=1)
        video_frame.columnconfigure(1, weight=1)
        video_frame.rowconfigure(0, weight=1)

        ttk.Label(left, text='Color', font=('Arial', 12, 'bold')).pack(anchor='w')
        self.color_label = ttk.Label(left)
        self.color_label.pack(fill='both', expand=True)

        ttk.Label(right, text='Profundidad', font=('Arial', 12, 'bold')).pack(anchor='w')
        self.depth_label = ttk.Label(right)
        self.depth_label.pack(fill='both', expand=True)

    def refresh_ui(self):
        self.status_var.set(
            f'Color: {self.ros_node.color_topic} | '
            f'Profundidad: {self.ros_node.depth_topic} | '
            f'Pares: {self.ros_node.frame_pairs}'
        )

        self.refresh_image(self.ros_node.latest_color_frame, self.color_label, 'color_photo')
        self.refresh_image(self.ros_node.latest_depth_frame, self.depth_label, 'depth_photo')

        self.root.after(100, self.refresh_ui)

    def refresh_image(self, frame, label, photo_attribute):
        if frame is None:
            return

        png_data = bgr_frame_to_png_data(frame, max_width=460, max_height=360)
        if png_data is None:
            return

        photo = tk.PhotoImage(data=png_data, format='png')
        setattr(self, photo_attribute, photo)
        label.configure(image=photo)


def main(args=None):
    rclpy.init(args=args)
    ros_node = RgbdViewerRosNode()

    root = tk.Tk()
    RgbdViewerApp(root, ros_node)

    def spin_ros():
        rclpy.spin_once(ros_node, timeout_sec=0.0)
        root.after(20, spin_ros)

    root.after(20, spin_ros)

    try:
        root.mainloop()
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()
