import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image

import cv2

from rescue_robot_core.camera_drivers.ros_image import numpy_frame_to_image_msg


class LogitechCameraNode(Node):
    def __init__(self):
        super().__init__('logitech_camera_node')

        self.declare_parameter('index', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('fourcc', 'MJPG')
        self.declare_parameter('buffer_size', 1)
        self.declare_parameter('stale_grabs', 1)
        self.declare_parameter('read_failure_reopen_threshold', 10)
        self.declare_parameter('image_topic', '/robot/camera/front/image_raw')
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
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value
        self.read_failures = 0

        self.capture = None
        self.open_capture()

        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self.publisher = self.create_publisher(Image, self.image_topic, sensor_qos)
        self.timer = self.create_timer(1.0 / max(float(self.fps), 1.0), self.publish_frame)

        self.get_logger().info(f'Publicando camara frontal en {self.image_topic}')

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
        self.read_failures = 0

        self.get_logger().info(
            f'Camara frontal abierta: /dev/video{self.camera_index} '
            f'{self.width}x{self.height}@{self.fps}, fourcc={self.fourcc or "default"}'
        )

    def publish_frame(self):
        if not self.capture.isOpened():
            return

        for _ in range(max(self.stale_grabs, 0)):
            self.capture.grab()

        ok, frame = self.capture.retrieve()
        if not ok:
            ok, frame = self.capture.read()

        if not ok:
            self.read_failures += 1
            if self.read_failures >= self.read_failure_reopen_threshold:
                self.get_logger().warn('Reabriendo camara frontal por fallos consecutivos de lectura')
                self.open_capture()
            return

        self.read_failures = 0
        msg = numpy_frame_to_image_msg(
            frame,
            encoding='bgr8',
            stamp=self.get_clock().now().to_msg(),
            frame_id=self.camera_frame_id
        )
        self.publisher.publish(msg)

    def shutdown_camera(self):
        if self.capture.isOpened():
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
