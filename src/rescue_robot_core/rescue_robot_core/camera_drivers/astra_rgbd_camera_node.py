import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2

import cv2
import numpy as np

from rescue_robot_core.camera_drivers.point_cloud import depth_image_to_point_cloud2


class AstraRgbdCameraNode(Node):
    def __init__(self):
        super().__init__('astra_rgbd_camera_node')

        self.bridge = CvBridge()

        self.declare_parameter('depth_index', 2)
        self.declare_parameter('color_index', 3)
        self.declare_parameter('fps', 30)
        self.declare_parameter('depth_topic', '/robot/camera/astra/depth/image_raw')
        self.declare_parameter('color_topic', '/robot/camera/astra/color/image_raw')
        self.declare_parameter('point_cloud_topic', '/robot/camera/astra/points')
        self.declare_parameter('depth_frame_id', 'astra_depth_optical_frame')
        self.declare_parameter('color_frame_id', 'astra_color_optical_frame')
        self.declare_parameter('publish_point_cloud', True)
        self.declare_parameter('depth_scale', 0.001)
        self.declare_parameter('point_cloud_stride', 4)
        self.declare_parameter('point_cloud_max_depth_m', 5.0)
        self.declare_parameter('fx', 525.0)
        self.declare_parameter('fy', 525.0)
        self.declare_parameter('cx', 319.5)
        self.declare_parameter('cy', 239.5)

        self.depth_index = self.get_parameter('depth_index').value
        self.color_index = self.get_parameter('color_index').value
        self.fps = self.get_parameter('fps').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.color_topic = self.get_parameter('color_topic').value
        self.point_cloud_topic = self.get_parameter('point_cloud_topic').value
        self.depth_frame_id = self.get_parameter('depth_frame_id').value
        self.color_frame_id = self.get_parameter('color_frame_id').value
        self.publish_point_cloud = self.get_parameter('publish_point_cloud').value
        self.depth_scale = self.get_parameter('depth_scale').value
        self.point_cloud_stride = self.get_parameter('point_cloud_stride').value
        self.point_cloud_max_depth_m = self.get_parameter('point_cloud_max_depth_m').value
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value

        self.depth_capture = cv2.VideoCapture(self.depth_index, cv2.CAP_V4L2)
        if self.depth_capture.isOpened():
            self.depth_capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'Y16 '))
            self.get_logger().info(f'Canal de profundidad abierto en /dev/video{self.depth_index}')
        else:
            self.get_logger().error(f'No se pudo abrir profundidad en /dev/video{self.depth_index}')

        self.color_capture = cv2.VideoCapture(self.color_index, cv2.CAP_V4L2)
        if self.color_capture.isOpened():
            self.get_logger().info(f'Canal de color abierto en /dev/video{self.color_index}')
        else:
            self.get_logger().error(f'No se pudo abrir color en /dev/video{self.color_index}')

        if not self.depth_capture.isOpened() and not self.color_capture.isOpened():
            raise RuntimeError('No se pudo abrir ningun canal de la camara Astra.')

        self.depth_publisher = self.create_publisher(Image, self.depth_topic, 10)
        self.color_publisher = self.create_publisher(Image, self.color_topic, 10)
        self.point_cloud_publisher = self.create_publisher(
            PointCloud2,
            self.point_cloud_topic,
            10
        )

        period = 1.0 / max(float(self.fps), 1.0)
        self.depth_timer = self.create_timer(period, self.publish_depth_frame)
        self.color_timer = self.create_timer(period, self.publish_color_frame)

        self.get_logger().info(f'Publicando profundidad en {self.depth_topic}')
        self.get_logger().info(f'Publicando color en {self.color_topic}')
        if self.publish_point_cloud:
            self.get_logger().info(f'Publicando nube de puntos en {self.point_cloud_topic}')

    def publish_depth_frame(self):
        if not self.depth_capture.isOpened():
            return

        ok, frame = self.depth_capture.read()
        if not ok:
            return

        depth_frame = self.normalize_depth_frame(frame)
        msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding='16UC1')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.depth_frame_id
        self.depth_publisher.publish(msg)

        if self.publish_point_cloud:
            cloud_msg = depth_image_to_point_cloud2(
                depth_frame=depth_frame,
                header=msg.header,
                fx=self.fx,
                fy=self.fy,
                cx=self.cx,
                cy=self.cy,
                depth_scale=self.depth_scale,
                stride=self.point_cloud_stride,
                max_depth_m=self.point_cloud_max_depth_m
            )
            self.point_cloud_publisher.publish(cloud_msg)

    def publish_color_frame(self):
        if not self.color_capture.isOpened():
            return

        ok, frame = self.color_capture.read()
        if not ok:
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.color_frame_id
        self.color_publisher.publish(msg)

    def normalize_depth_frame(self, frame):
        if frame.ndim == 3 and frame.shape[2] >= 2:
            low_byte = frame[:, :, 0].astype(np.uint16)
            high_byte = frame[:, :, 1].astype(np.uint16)
            return low_byte | (high_byte << 8)

        if frame.ndim == 3:
            return frame[:, :, 0].astype(np.uint16)

        return frame.astype(np.uint16)

    def shutdown_camera(self):
        if self.depth_capture.isOpened():
            self.depth_capture.release()
        if self.color_capture.isOpened():
            self.color_capture.release()


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
