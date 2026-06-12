import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, PointCloud2
from std_msgs.msg import Header

import cv2
import numpy as np

from rescue_robot_core.camera_drivers.logitech_camera_node import disable_dynamic_framerate
from rescue_robot_core.camera_drivers.point_cloud import depth_image_to_point_cloud2
from rescue_robot_core.camera_drivers.ros_image import (
    numpy_frame_to_compressed_image_msg,
    raw_jpeg_to_compressed_image_msg,
)


class AstraRgbdCameraNode(Node):
    def __init__(self):
        super().__init__('astra_rgbd_camera_node')

        # La Astra Pro solo expone su camara RGB por V4L2; la profundidad
        # requiere OpenNI2, por eso queda deshabilitada por defecto.
        self.declare_parameter('depth_index', -1)
        self.declare_parameter('color_index', 2)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('depth_fourcc', 'YUYV')
        self.declare_parameter('color_fourcc', 'MJPG')
        self.declare_parameter('buffer_size', 1)
        self.declare_parameter('read_failure_reopen_threshold', 10)
        self.declare_parameter('frame_timeout_seconds', 2.0)
        self.declare_parameter('stats_log_period_seconds', 5.0)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('mjpeg_passthrough', True)
        self.declare_parameter('depth_topic', '/robot/camera/astra/depth/image_raw/compressed')
        self.declare_parameter('color_topic', '/robot/camera/astra/color/image_raw/compressed')
        self.declare_parameter('point_cloud_topic', '/robot/camera/astra/points')
        self.declare_parameter('depth_frame_id', 'astra_depth_optical_frame')
        self.declare_parameter('color_frame_id', 'astra_color_optical_frame')
        self.declare_parameter('publish_point_cloud', True)
        self.declare_parameter('point_cloud_every_n', 3)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('point_cloud_stride', 8)
        self.declare_parameter('point_cloud_max_depth_m', 5.0)
        self.declare_parameter('fx', 525.0)
        self.declare_parameter('fy', 525.0)
        self.declare_parameter('cx', 319.5)
        self.declare_parameter('cy', 239.5)

        self.depth_index = self.get_parameter('depth_index').value
        self.color_index = self.get_parameter('color_index').value
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = self.get_parameter('fps').value
        self.depth_fourcc = self.get_parameter('depth_fourcc').value
        self.color_fourcc = self.get_parameter('color_fourcc').value
        self.buffer_size = int(self.get_parameter('buffer_size').value)
        self.read_failure_reopen_threshold = int(
            self.get_parameter('read_failure_reopen_threshold').value
        )
        self.frame_timeout_seconds = float(self.get_parameter('frame_timeout_seconds').value)
        self.stats_log_period_seconds = float(
            self.get_parameter('stats_log_period_seconds').value
        )
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.color_mjpeg_passthrough = (
            bool(self.get_parameter('mjpeg_passthrough').value) and self.color_fourcc == 'MJPG'
        )
        self.depth_topic = self.get_parameter('depth_topic').value
        self.color_topic = self.get_parameter('color_topic').value
        self.point_cloud_topic = self.get_parameter('point_cloud_topic').value
        self.depth_frame_id = self.get_parameter('depth_frame_id').value
        self.color_frame_id = self.get_parameter('color_frame_id').value
        self.publish_point_cloud = self.get_parameter('publish_point_cloud').value
        self.point_cloud_every_n = max(int(self.get_parameter('point_cloud_every_n').value), 1)
        self.depth_scale = self.get_parameter('depth_scale').value
        self.point_cloud_stride = self.get_parameter('point_cloud_stride').value
        self.point_cloud_max_depth_m = self.get_parameter('point_cloud_max_depth_m').value
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        self.depth_frame_count = 0
        now = self.now_seconds()
        self.depth_read_failures = 0
        self.color_read_failures = 0
        self.depth_published_frames = 0
        self.color_published_frames = 0
        self.last_depth_frame_time = now
        self.last_color_frame_time = now
        self.last_stats_time = now
        self.last_depth_stats_frames = 0
        self.last_color_stats_frames = 0

        self.depth_capture = None
        self.color_capture = None
        self.open_depth_capture()
        self.open_color_capture()

        if not self.is_capture_open(self.depth_capture) and not self.is_capture_open(self.color_capture):
            self.get_logger().warn(
                'No se pudo abrir ningun canal de la camara Astra; '
                'el nodo queda vivo sin publicar RGB-D.'
            )

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.depth_publisher = self.create_publisher(CompressedImage, self.depth_topic, sensor_qos)
        self.color_publisher = self.create_publisher(CompressedImage, self.color_topic, sensor_qos)
        self.point_cloud_publisher = self.create_publisher(
            PointCloud2,
            self.point_cloud_topic,
            sensor_qos
        )

        # Hilos dedicados: leer con timers de ROS desperdicia frames de la camara
        self.stop_event = threading.Event()
        self.depth_thread = threading.Thread(target=self.depth_loop, daemon=True)
        self.color_thread = threading.Thread(target=self.color_loop, daemon=True)
        self.depth_thread.start()
        self.color_thread.start()

        self.get_logger().info(f'Publicando profundidad en {self.depth_topic}')
        self.get_logger().info(f'Publicando color en {self.color_topic}')
        if self.publish_point_cloud:
            self.get_logger().info(f'Publicando nube de puntos en {self.point_cloud_topic}')

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def is_capture_open(self, capture):
        return capture is not None and capture.isOpened()

    def open_capture(self, index, fourcc, label):
        if index is None or int(index) < 0:
            return None

        capture = cv2.VideoCapture(int(index), cv2.CAP_V4L2)
        if not capture.isOpened():
            self.get_logger().warn(f'No se pudo abrir {label} en /dev/video{index}')
            return capture

        if fourcc:
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc[:4]))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, max(self.buffer_size, 1))
        self.get_logger().info(
            f'Canal {label} abierto en /dev/video{index}, fourcc={fourcc or "default"}'
        )
        return capture

    def open_depth_capture(self):
        if self.is_capture_open(self.depth_capture):
            self.depth_capture.release()
        self.depth_capture = self.open_capture(self.depth_index, self.depth_fourcc, 'profundidad')
        self.depth_read_failures = 0
        self.last_depth_frame_time = self.now_seconds()

    def open_color_capture(self):
        if self.color_index == self.depth_index:
            self.get_logger().info(
                'Astra color deshabilitado porque color_index coincide con depth_index'
            )
            self.color_capture = None
            return

        if self.is_capture_open(self.color_capture):
            self.color_capture.release()
        self.color_capture = self.open_capture(self.color_index, self.color_fourcc, 'color')
        if self.color_mjpeg_passthrough and self.is_capture_open(self.color_capture):
            # Entrega el JPEG de la camara sin decodificar (ahorra CPU)
            self.color_capture.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        if self.is_capture_open(self.color_capture):
            disable_dynamic_framerate(self.color_index, self.get_logger())
        self.color_read_failures = 0
        self.last_color_frame_time = self.now_seconds()

    def should_reopen(self, failures, last_frame_time):
        return (
            failures >= self.read_failure_reopen_threshold or
            self.now_seconds() - last_frame_time >= self.frame_timeout_seconds
        )

    def depth_loop(self):
        while not self.stop_event.is_set() and rclpy.ok():
            self.publish_depth_frame()

    def color_loop(self):
        while not self.stop_event.is_set() and rclpy.ok():
            self.publish_color_frame()

    def publish_depth_frame(self):
        if not self.is_capture_open(self.depth_capture):
            time.sleep(1.0)
            self.open_depth_capture()
            return

        ok, frame = self.depth_capture.read()
        if not ok:
            time.sleep(0.05)
            self.depth_read_failures += 1
            if self.should_reopen(self.depth_read_failures, self.last_depth_frame_time):
                self.get_logger().warn(
                    'Reabriendo Astra profundidad: '
                    f'fallos={self.depth_read_failures}'
                )
                self.open_depth_capture()
            return

        self.depth_read_failures = 0
        self.last_depth_frame_time = self.now_seconds()
        self.depth_published_frames += 1
        depth_frame = self.normalize_depth_frame(frame)
        msg = numpy_frame_to_compressed_image_msg(
            depth_frame,
            stamp=self.get_clock().now().to_msg(),
            frame_id=self.depth_frame_id,
            fmt='png'
        )
        self.depth_publisher.publish(msg)
        self.depth_frame_count += 1

        should_publish_cloud = self.depth_frame_count % self.point_cloud_every_n == 0
        if self.publish_point_cloud and should_publish_cloud:
            cloud_header = Header()
            cloud_header.stamp = msg.header.stamp
            cloud_header.frame_id = self.depth_frame_id
            cloud_msg = depth_image_to_point_cloud2(
                depth_frame=depth_frame,
                header=cloud_header,
                fx=self.fx,
                fy=self.fy,
                cx=self.cx,
                cy=self.cy,
                depth_scale=self.depth_scale,
                stride=self.point_cloud_stride,
                max_depth_m=self.point_cloud_max_depth_m
            )
            self.point_cloud_publisher.publish(cloud_msg)
        self.log_stats()

    def publish_color_frame(self):
        if not self.is_capture_open(self.color_capture):
            time.sleep(1.0)
            if self.color_index is not None and int(self.color_index) >= 0:
                self.open_color_capture()
            return

        ok, frame = self.color_capture.read()
        if not ok:
            time.sleep(0.05)
            self.color_read_failures += 1
            if self.should_reopen(self.color_read_failures, self.last_color_frame_time):
                self.get_logger().warn(
                    'Reabriendo Astra color: '
                    f'fallos={self.color_read_failures}'
                )
                self.open_color_capture()
            return

        self.color_read_failures = 0
        self.last_color_frame_time = self.now_seconds()
        self.color_published_frames += 1
        stamp = self.get_clock().now().to_msg()
        if self.color_mjpeg_passthrough:
            msg = raw_jpeg_to_compressed_image_msg(
                frame,
                stamp=stamp,
                frame_id=self.color_frame_id
            )
        else:
            msg = numpy_frame_to_compressed_image_msg(
                frame,
                stamp=stamp,
                frame_id=self.color_frame_id,
                fmt='jpeg',
                jpeg_quality=self.jpeg_quality
            )
        self.color_publisher.publish(msg)
        self.log_stats()

    def normalize_depth_frame(self, frame):
        if frame.ndim == 3 and frame.shape[2] >= 2:
            low_byte = frame[:, :, 0].astype(np.uint16)
            high_byte = frame[:, :, 1].astype(np.uint16)
            return low_byte | (high_byte << 8)

        if frame.ndim == 3:
            return frame[:, :, 0].astype(np.uint16)

        return frame.astype(np.uint16)

    def shutdown_camera(self):
        self.stop_event.set()
        for thread in (self.depth_thread, self.color_thread):
            if thread.is_alive():
                thread.join(timeout=2.0)
        if self.is_capture_open(self.depth_capture):
            self.depth_capture.release()
        if self.is_capture_open(self.color_capture):
            self.color_capture.release()

    def log_stats(self):
        now = self.now_seconds()
        elapsed = now - self.last_stats_time
        if elapsed < self.stats_log_period_seconds:
            return

        depth_frames = self.depth_published_frames - self.last_depth_stats_frames
        color_frames = self.color_published_frames - self.last_color_stats_frames
        depth_fps = depth_frames / elapsed if elapsed > 0.0 else 0.0
        color_fps = color_frames / elapsed if elapsed > 0.0 else 0.0

        self.last_stats_time = now
        self.last_depth_stats_frames = self.depth_published_frames
        self.last_color_stats_frames = self.color_published_frames
        self.get_logger().info(
            f'Astra publicando profundidad={depth_fps:.1f} fps, color={color_fps:.1f} fps'
        )


def main(args=None):
    rclpy.init(args=args)
    node = AstraRgbdCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_camera()
        node.destroy_node()
        rclpy.shutdown()
