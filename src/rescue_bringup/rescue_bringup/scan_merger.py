#!/usr/bin/env python3
"""
scan_merger.py
Fusiona el scan del LiDAR trasero (/ldlidar_node/scan) con el scan virtual
generado desde la cámara 3D frontal (/camera/scan) en un único topic /scan_merged.

Ambos scans se transforman a base_footprint antes de fusionar, así el mapa
slam_toolbox recibe una visión 360° cooperativa: LiDAR para atrás/lados,
cámara para el frente donde el LiDAR tiene punto ciego.
"""

import math
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf2_ros import TransformException


class ScanMerger(Node):

    def __init__(self):
        super().__init__('scan_merger')

        self.declare_parameter('lidar_topic',  '/ldlidar_node/scan')
        self.declare_parameter('camera_topic', '/camera/scan')
        self.declare_parameter('output_topic', '/scan_merged')
        self.declare_parameter('target_frame', 'base_footprint')
        self.declare_parameter('angle_min',    -math.pi)
        self.declare_parameter('angle_max',     math.pi)
        self.declare_parameter('angle_increment', math.radians(0.5))
        self.declare_parameter('range_min',    0.10)
        self.declare_parameter('range_max',    12.0)

        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        self._lidar_scan  = None
        self._camera_scan = None

        lidar_topic  = self.get_parameter('lidar_topic').value
        camera_topic = self.get_parameter('camera_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.create_subscription(LaserScan, lidar_topic,
                                 self._on_lidar, qos_profile_sensor_data)
        self.create_subscription(LaserScan, camera_topic,
                                 self._on_camera, qos_profile_sensor_data)

        self._pub = self.create_publisher(LaserScan, output_topic, 10)
        self.create_timer(0.05, self._publish_merged)   # 20 Hz

        self.get_logger().info(
            f'ScanMerger: {lidar_topic} + {camera_topic} → {output_topic}')

    def _on_lidar(self, msg):
        self._lidar_scan = msg

    def _on_camera(self, msg):
        self._camera_scan = msg

    def _scan_to_points(self, scan: LaserScan, target_frame: str):
        """Convierte un LaserScan a lista de (x, y) en target_frame."""
        try:
            t = self._tf_buf.lookup_transform(
                target_frame, scan.header.frame_id, scan.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.05))
        except TransformException:
            try:
                t = self._tf_buf.lookup_transform(
                    target_frame, scan.header.frame_id, rclpy.time.Time())
            except TransformException:
                return []

        tx = t.transform.translation.x
        ty = t.transform.translation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w
        yaw = 2.0 * math.atan2(qz, qw)

        points = []
        angle = scan.angle_min
        for r in scan.ranges:
            if scan.range_min <= r <= scan.range_max and math.isfinite(r):
                lx = r * math.cos(angle)
                ly = r * math.sin(angle)
                bx = tx + lx * math.cos(yaw) - ly * math.sin(yaw)
                by = ty + lx * math.sin(yaw) + ly * math.cos(yaw)
                points.append((bx, by))
            angle += scan.angle_increment

        return points

    def _publish_merged(self):
        if self._lidar_scan is None:
            return

        target = self.get_parameter('target_frame').value
        a_min  = self.get_parameter('angle_min').value
        a_max  = self.get_parameter('angle_max').value
        a_inc  = self.get_parameter('angle_increment').value
        r_min  = self.get_parameter('range_min').value
        r_max  = self.get_parameter('range_max').value

        n_bins = int((a_max - a_min) / a_inc) + 1
        ranges = [float('inf')] * n_bins

        for pts_src in [self._lidar_scan, self._camera_scan]:
            if pts_src is None:
                continue
            for (bx, by) in self._scan_to_points(pts_src, target):
                angle = math.atan2(by, bx)
                dist  = math.hypot(bx, by)
                if not (r_min <= dist <= r_max):
                    continue
                idx = int((angle - a_min) / a_inc)
                if 0 <= idx < n_bins:
                    if dist < ranges[idx]:
                        ranges[idx] = dist

        out = LaserScan()
        out.header.stamp    = self.get_clock().now().to_msg()
        out.header.frame_id = target
        out.angle_min       = a_min
        out.angle_max       = a_max
        out.angle_increment = a_inc
        out.time_increment  = 0.0
        out.scan_time       = 0.05
        out.range_min       = r_min
        out.range_max       = r_max
        out.ranges          = [r if r < float('inf') else 0.0 for r in ranges]

        self._pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ScanMerger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
