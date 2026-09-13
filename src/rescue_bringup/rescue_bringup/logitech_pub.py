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

        self._dev_index = dev if isinstance(dev, int) else int(dev)
        self._topic = topic
        self._cap = None
        self._publish_timer = None

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self._pub = self.create_publisher(CompressedImage, topic, qos)

        # Si la camara esta ocupada (otro launch sin cerrar, otro proceso) el
        # nodo NO se queda mudo para siempre: reintenta hasta abrirla. Antes
        # se rendia con un solo error, el nodo seguia vivo sin publicar y
        # parecia que "la deteccion no arranco".
        if not self._open_camera():
            self._retry_timer = self.create_timer(2.0, self._retry_open)

    def _open_camera(self) -> bool:
        cap = cv2.VideoCapture(self._dev_index, cv2.CAP_V4L2)
        if not cap.isOpened():
            cap.release()
            return False
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, self._fps)
        self._cap = cap
        self._publish_timer = self.create_timer(1.0 / self._fps, self._publish)
        self.get_logger().info(
            f'Publicando /dev/video{self._dev_index} en {self._topic} a {self._fps} fps')
        return True

    def _retry_open(self) -> None:
        if self._open_camera():
            self._retry_timer.cancel()
            return
        self.get_logger().warn(
            f'/dev/video{self._dev_index} ocupada o ausente — sin imagen, por lo tanto '
            f'SIN DETECCION. Reintentando... (¿hay otro launch o script usando la camara?)')

    def _publish(self):
        if self._cap is None:
            return
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
        if self._cap is not None and self._cap.isOpened():
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
        if rclpy.ok():
            rclpy.shutdown()
