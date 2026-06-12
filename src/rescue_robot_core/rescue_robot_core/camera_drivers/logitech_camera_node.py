import subprocess
import threading
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage

import cv2

from rescue_robot_core.camera_drivers.ros_image import (
    numpy_frame_to_compressed_image_msg,
    raw_jpeg_to_compressed_image_msg,
)


def disable_dynamic_framerate(device_index, logger):
    # Sin esto la camara baja sola a ~7 fps cuando hay poca luz
    try:
        subprocess.run(
            ['v4l2-ctl', '-d', f'/dev/video{device_index}',
             '-c', 'exposure_dynamic_framerate=0'],
            check=False,
            capture_output=True,
            timeout=5,
        )
    except (FileNotFoundError, subprocess.TimeoutExpired) as exc:
        logger.warn(f'No se pudo fijar exposure_dynamic_framerate: {exc}')


class LogitechCameraNode(Node):
    def __init__(self):
        super().__init__('logitech_camera_node')

        self.declare_parameter('index', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('fourcc', 'MJPG')
        self.declare_parameter('buffer_size', 1)
        self.declare_parameter('stale_grabs', 0)
        self.declare_parameter('read_failure_reopen_threshold', 10)
        self.declare_parameter('frame_timeout_seconds', 2.0)
        self.declare_parameter('stats_log_period_seconds', 5.0)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('mjpeg_passthrough', True)
        self.declare_parameter('image_topic', '/robot/camera/front/image_raw/compressed')
        self.declare_parameter('camera_frame_id', 'front_camera_frame')

        self.camera_index = self.get_parameter('index').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.fourcc = self.get_parameter('fourcc').value
        self.buffer_size = int(self.get_parameter('buffer_size').value)
        self.stale_grabs = int(self.get_parameter('stale_grabs').value)
        self.read_failure_reopen_threshold = int(
            self.get_parameter('read_failure_reopen_threshold').value
        )
        self.frame_timeout_seconds = float(self.get_parameter('frame_timeout_seconds').value)
        self.stats_log_period_seconds = float(
            self.get_parameter('stats_log_period_seconds').value
        )
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.mjpeg_passthrough = (
            bool(self.get_parameter('mjpeg_passthrough').value) and self.fourcc == 'MJPG'
        )
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.read_failures = 0
        self.published_frames = 0
        self.last_frame_time = self.now_seconds()
        self.last_stats_time = self.last_frame_time
        self.last_stats_frames = 0

        self.capture = None
        self.open_capture()

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(CompressedImage, self.image_topic, sensor_qos)

        # Hilo dedicado: leer con timer de ROS desperdicia frames de la camara
        self.stop_event = threading.Event()
        self.capture_thread = threading.Thread(target=self.capture_loop, daemon=True)
        self.capture_thread.start()

        self.get_logger().info(f'Publicando camara frontal en {self.image_topic}')

    def now_seconds(self):
        return self.get_clock().now().nanoseconds / 1e9

    def open_capture(self):
        if self.capture is not None and self.capture.isOpened():
            self.capture.release()

        self.capture = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            self.get_logger().error(f'No se pudo abrir la camara en /dev/video{self.camera_index}')
            return

        if self.fourcc:
            self.capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc[:4]))
        self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.capture.set(cv2.CAP_PROP_FPS, self.fps)
        self.capture.set(cv2.CAP_PROP_BUFFERSIZE, max(self.buffer_size, 1))
        if self.mjpeg_passthrough:
            # Entrega el JPEG de la camara sin decodificar (ahorra CPU)
            self.capture.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        disable_dynamic_framerate(self.camera_index, self.get_logger())
        self.read_failures = 0
        self.last_frame_time = self.now_seconds()

        self.get_logger().info(
            f'Camara frontal abierta: /dev/video{self.camera_index} '
            f'{self.width}x{self.height}@{self.fps}, fourcc={self.fourcc or "default"}'
        )

    def capture_loop(self):
        while not self.stop_event.is_set() and rclpy.ok():
            self.publish_frame()

    def publish_frame(self):
        if self.capture is None or not self.capture.isOpened():
            time.sleep(1.0)
            self.open_capture()
            return

        for _ in range(max(self.stale_grabs, 0)):
            self.capture.read()

        ok, frame = self.capture.read()
        if not ok:
            time.sleep(0.05)
            self.read_failures += 1
            elapsed_without_frames = self.now_seconds() - self.last_frame_time
            should_reopen = (
                self.read_failures >= self.read_failure_reopen_threshold or
                elapsed_without_frames >= self.frame_timeout_seconds
            )
            if should_reopen:
                self.get_logger().warn(
                    'Reabriendo camara frontal: '
                    f'fallos={self.read_failures}, '
                    f'sin_frames={elapsed_without_frames:.2f}s'
                )
                self.open_capture()
            return

        self.read_failures = 0
        self.last_frame_time = self.now_seconds()
        self.published_frames += 1
        stamp = self.get_clock().now().to_msg()
        if self.mjpeg_passthrough:
            msg = raw_jpeg_to_compressed_image_msg(
                frame,
                stamp=stamp,
                frame_id=self.camera_frame_id
            )
        else:
            msg = numpy_frame_to_compressed_image_msg(
                frame,
                stamp=stamp,
                frame_id=self.camera_frame_id,
                fmt='jpeg',
                jpeg_quality=self.jpeg_quality
            )
        self.publisher.publish(msg)
        self.log_stats()

    def log_stats(self):
        now = self.now_seconds()
        elapsed = now - self.last_stats_time
        if elapsed < self.stats_log_period_seconds:
            return

        frames = self.published_frames - self.last_stats_frames
        fps = frames / elapsed if elapsed > 0.0 else 0.0
        self.last_stats_time = now
        self.last_stats_frames = self.published_frames
        self.get_logger().info(f'Camara frontal publicando {fps:.1f} fps')

    def shutdown_camera(self):
        self.stop_event.set()
        if self.capture_thread.is_alive():
            self.capture_thread.join(timeout=2.0)
        if self.capture is not None and self.capture.isOpened():
            self.capture.release()


def main(args=None):
    rclpy.init(args=args)
    node = LogitechCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_camera()
        node.destroy_node()
        rclpy.shutdown()
