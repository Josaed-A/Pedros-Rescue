#!/usr/bin/env python3
"""
scan_merger.py
Fusiona el scan del LiDAR trasero (/ldlidar_node/scan) con el scan virtual
generado desde la cámara 3D frontal (/camera/scan) en un único topic /scan_merged.

Filtro angular del LiDAR:
  Solo se usan los rayos entre lidar_valid_min (30°) y lidar_valid_max (310°).
  El cono 310°→0°→30° queda descartado porque ahí está el brazo robótico,
  que al moverse generaría obstáculos falsos en el mapa SLAM.
  La cámara cubre ese sector frontal en su lugar.

Ambos scans se transforman a base_footprint antes de fusionar.
"""

import math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf2_ros import TransformException

DEG = math.pi / 180.0


def _angle_in_arc(angle: float, arc_min: float, arc_max: float) -> bool:
    """True si angle (rad, cualquier rango) cae en el arco [arc_min, arc_max].
    Maneja el caso arc_min < arc_max (arco normal) y arc_min > arc_max (cruza 0°).
    """
    a = math.atan2(math.sin(angle), math.cos(angle))   # normaliza a [-π, π]
    lo = math.atan2(math.sin(arc_min), math.cos(arc_min))
    hi = math.atan2(math.sin(arc_max), math.cos(arc_max))
    if lo <= hi:
        return lo <= a <= hi
    else:                           # arco cruza ±π
        return a >= lo or a <= hi


class ScanMerger(Node):

    def __init__(self):
        super().__init__('scan_merger')

        # Topics
        self.declare_parameter('lidar_topic',  '/ldlidar_node/scan')
        self.declare_parameter('camera_topic', '/camera/scan')
        self.declare_parameter('output_topic', '/scan_merged')
        self.declare_parameter('target_frame', 'base_footprint')

        # Salida fusionada
        self.declare_parameter('angle_min',       -math.pi)
        self.declare_parameter('angle_max',        math.pi)
        self.declare_parameter('angle_increment',  math.radians(0.5))
        self.declare_parameter('range_min',        0.10)
        self.declare_parameter('range_max',       12.0)

        # Filtro angular LiDAR — excluye el cono del brazo (alrededor de 0°)
        # Solo se procesan rayos cuyo ángulo en el frame del LiDAR esté
        # entre lidar_valid_min_deg y lidar_valid_max_deg (en grados).
        self.declare_parameter('lidar_valid_min_deg',  30.0)   # 30°
        self.declare_parameter('lidar_valid_max_deg', 310.0)   # 310°

        self._tf_buf      = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        self._lidar_scan  = None
        self._camera_scan = None

        lidar_topic  = self.get_parameter('lidar_topic').value
        camera_topic = self.get_parameter('camera_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.create_subscription(LaserScan, lidar_topic,
                                 self._on_lidar,  qos_profile_sensor_data)
        self.create_subscription(LaserScan, camera_topic,
                                 self._on_camera, qos_profile_sensor_data)

        self._pub = self.create_publisher(LaserScan, output_topic, 10)
        self.create_timer(0.05, self._publish_merged)   # 20 Hz

        self.get_logger().info(
            f'ScanMerger listo\n'
            f'  LiDAR : {lidar_topic}  '
            f'[{self.get_parameter("lidar_valid_min_deg").value}° – '
            f'{self.get_parameter("lidar_valid_max_deg").value}°]\n'
            f'  Cámara: {camera_topic}  (frontal, cubre zona del brazo)\n'
            f'  Salida: {output_topic}')

    # ── Callbacks ────────────────────────────────────────────────────

    def _on_lidar(self, msg):
        self._lidar_scan = msg

    def _on_camera(self, msg):
        self._camera_scan = msg

    # ── Conversión scan → puntos en target_frame ──────────────────

    def _scan_to_points(self, scan: LaserScan, target_frame: str,
                        angle_filter_min=None, angle_filter_max=None):
        """Convierte LaserScan a lista de (x, y) en target_frame.

        Si angle_filter_min/max se pasan (en rad, en el frame del sensor),
        solo se incluyen los rayos cuyo ángulo local cae en ese arco.
        """
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

        tx  = t.transform.translation.x
        ty  = t.transform.translation.y
        qz  = t.transform.rotation.z
        qw  = t.transform.rotation.w
        yaw = 2.0 * math.atan2(qz, qw)

        use_filter = (angle_filter_min is not None and
                      angle_filter_max is not None)

        points = []
        angle  = scan.angle_min
        for r in scan.ranges:
            local_angle = angle
            angle += scan.angle_increment

            if not (scan.range_min <= r <= scan.range_max and math.isfinite(r)):
                continue

            # Aplicar filtro angular en el frame del sensor
            if use_filter and not _angle_in_arc(
                    local_angle, angle_filter_min, angle_filter_max):
                continue

            lx = r * math.cos(local_angle)
            ly = r * math.sin(local_angle)
            bx = tx + lx * math.cos(yaw) - ly * math.sin(yaw)
            by = ty + lx * math.sin(yaw) + ly * math.cos(yaw)
            points.append((bx, by))

        return points

    # ── Publicación del scan fusionado ────────────────────────────

    def _publish_merged(self):
        if self._lidar_scan is None:
            return

        target = self.get_parameter('target_frame').value
        a_min  = self.get_parameter('angle_min').value
        a_max  = self.get_parameter('angle_max').value
        a_inc  = self.get_parameter('angle_increment').value
        r_min  = self.get_parameter('range_min').value
        r_max  = self.get_parameter('range_max').value

        # Límites del filtro del LiDAR (convertidos a rad)
        lf_min = self.get_parameter('lidar_valid_min_deg').value * DEG
        lf_max = self.get_parameter('lidar_valid_max_deg').value * DEG

        n_bins = int((a_max - a_min) / a_inc) + 1
        ranges = [float('inf')] * n_bins

        # 1. LiDAR con filtro angular (excluye cono del brazo)
        for (bx, by) in self._scan_to_points(
                self._lidar_scan, target,
                angle_filter_min=lf_min, angle_filter_max=lf_max):
            self._insert_point(bx, by, ranges, a_min, a_inc, n_bins, r_min, r_max)

        # 2. Cámara sin filtro (cubre el frente donde el LiDAR está ciego)
        if self._camera_scan is not None:
            for (bx, by) in self._scan_to_points(self._camera_scan, target):
                self._insert_point(bx, by, ranges, a_min, a_inc, n_bins, r_min, r_max)

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
        out.ranges          = [r if math.isfinite(r) else 0.0 for r in ranges]

        self._pub.publish(out)

    @staticmethod
    def _insert_point(bx, by, ranges, a_min, a_inc, n_bins, r_min, r_max):
        dist  = math.hypot(bx, by)
        if not (r_min <= dist <= r_max):
            return
        angle = math.atan2(by, bx)
        idx   = int((angle - a_min) / a_inc)
        if 0 <= idx < n_bins and dist < ranges[idx]:
            ranges[idx] = dist


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
