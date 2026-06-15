import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import CompressedImage


class LogitechPublisher(Node):
    def __init__(self):
        super().__init__('logitech_pub')
        self.declare_parameter('device', 0)
        self.declare_parameter('topic', '/robot/camera/front/image_raw/compressed')
        self.declare_parameter('fps', 15)
        self.declare_parameter('jpeg_quality', 80)

        dev   = self.get_parameter('device').value
        topic = self.get_parameter('topic').value
        self._fps  = self.get_parameter('fps').value
        self._qual = self.get_parameter('jpeg_quality').value

        dev_index = dev if isinstance(dev, int) else int(dev)
        self._cap = cv2.VideoCapture(dev_index, cv2.CAP_V4L2)
        if not self._cap.isOpened():
            self.get_logger().error(f'No se pudo abrir /dev/video{dev_index}')
            return

        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self._cap.set(cv2.CAP_PROP_FPS, self._fps)

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self._pub = self.create_publisher(CompressedImage, topic, qos)
        self.create_timer(1.0 / self._fps, self._publish)
        self.get_logger().info(f'Logitech publicando en {topic} a {self._fps} fps')

    def _publish(self):
        ok, frame = self._cap.read()
        if not ok:
            return
        _, buf = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self._qual])
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        self._pub.publish(msg)

    def destroy_node(self):
        if self._cap.isOpened():
            self._cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = LogitechPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
