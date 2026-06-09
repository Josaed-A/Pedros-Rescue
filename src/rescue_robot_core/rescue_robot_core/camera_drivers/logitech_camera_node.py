import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2


class LogitechCameraNode(Node):
    def __init__(self):
        super().__init__('logitech_camera_node')

        self.bridge = CvBridge()

        self.declare_parameter('index', 0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('image_topic', '/robot/camera/front/image_raw')
        self.declare_parameter('camera_frame_id', 'front_camera_frame')

        self.camera_index = self.get_parameter('index').value
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.fps = self.get_parameter('fps').value
        self.image_topic = self.get_parameter('image_topic').value
        self.camera_frame_id = self.get_parameter('camera_frame_id').value

        self.capture = cv2.VideoCapture(self.camera_index, cv2.CAP_V4L2)
        if not self.capture.isOpened():
            self.get_logger().error(f'No se pudo abrir la camara en /dev/video{self.camera_index}')
        else:
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.capture.set(cv2.CAP_PROP_FPS, self.fps)
            self.get_logger().info(
                f'Camara frontal abierta: /dev/video{self.camera_index} '
                f'{self.width}x{self.height}@{self.fps}'
            )

        self.publisher = self.create_publisher(Image, self.image_topic, 10)
        self.timer = self.create_timer(1.0 / max(float(self.fps), 1.0), self.publish_frame)

        self.get_logger().info(f'Publicando camara frontal en {self.image_topic}')

    def publish_frame(self):
        if not self.capture.isOpened():
            return

        ok, frame = self.capture.read()
        if not ok:
            self.get_logger().warn('No se pudo leer frame de la camara frontal')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.camera_frame_id
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
