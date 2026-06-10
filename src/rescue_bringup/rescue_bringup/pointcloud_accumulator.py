#!/usr/bin/env python3
"""
pointcloud_accumulator.py
Acumula la nube de puntos 3D en el frame 'map' y la exporta como PLY ASCII.

Fuentes (en orden de prioridad):
  1. /camera/depth_registered/points  (XYZRGB — cámara depth + color alineado)
  2. /camera/depth/points              (XYZ    — cámara depth sin color)
  3. /ldlidar_node/scan                (LaserScan → puntos 2D a la altura del lidar)

Formato PLY RoboCup 2026 (páginas 20-22):
  • Campos mínimos:  x y z
  • Campos bonus (+25% score): r g b  (nube coloreada con RGB de la cámara)
  • Nombre:  RoboCup2026-{team}-{mission}-{HH-MM-SS}-map.ply
  • Origen:  centro del frente del robot, nivel del suelo, posición de inicio
  • Rotación: +Y = dirección inicial del robot, +Z = arriba

Servicio:
  /save_pointcloud_ply  (std_srvs/srv/Trigger)
"""

import datetime
import math
import os
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, PointCloud2, PointField
from std_srvs.srv import Trigger
import tf2_ros

# Robot base half-length (from URDF: base_length=0.35 m)
ROBOT_HALF_LENGTH = 0.175   # m — distancia del centro al frente del robot

# ── Constantes ─────────────────────────────────────────────────────
VOXEL_SIZE   = 0.02   # m — resolución de la nube final (2 cm)
MAX_RANGE    = 4.0    # m — descartar puntos más lejanos (rango útil Astra Pro)
MIN_RANGE    = 0.3    # m — descartar puntos muy cercanos (ruido)
MAX_POINTS   = 5_000_000   # límite de memoria


def _pc2_to_xyz_rgb(msg: PointCloud2) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """
    Decodifica un PointCloud2 en arrays numpy.
    Devuelve (xyz, rgb) donde rgb puede ser None si no hay campo de color.
    xyz: (N, 3) float32
    rgb: (N, 3) uint8  o  None
    """
    fields = {f.name: f for f in msg.fields}
    has_rgb = 'rgb' in fields or 'rgba' in fields

    point_step = msg.point_step
    n_points = msg.width * msg.height

    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_points, point_step)

    fx = fields['x'].offset
    fy = fields['y'].offset
    fz = fields['z'].offset
    x = raw[:, fx:fx+4].view(np.float32).reshape(-1)
    y = raw[:, fy:fy+4].view(np.float32).reshape(-1)
    z = raw[:, fz:fz+4].view(np.float32).reshape(-1)
    xyz = np.stack([x, y, z], axis=-1)

    valid = np.isfinite(xyz).all(axis=1)
    xyz = xyz[valid]

    rgb = None
    if has_rgb:
        fc = fields.get('rgb') or fields.get('rgba')
        c_raw = raw[valid, fc.offset:fc.offset+4]
        r = c_raw[:, 2].astype(np.uint8)
        g = c_raw[:, 1].astype(np.uint8)
        b = c_raw[:, 0].astype(np.uint8)
        rgb = np.stack([r, g, b], axis=-1)

    return xyz, rgb


def _voxel_filter(xyz: np.ndarray, rgb: Optional[np.ndarray],
                  voxel_size: float) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    """Reduce la densidad con un filtro voxel grid simple (centroide por celda)."""
    if len(xyz) == 0:
        return xyz, rgb

    origin = xyz.min(axis=0)
    idx = ((xyz - origin) / voxel_size).astype(np.int32)
    keys = idx[:, 0] * 100_000_000 + idx[:, 1] * 100_000 + idx[:, 2]
    _, inv = np.unique(keys, return_index=True)
    filtered_xyz = xyz[inv]
    filtered_rgb = rgb[inv] if rgb is not None else None
    return filtered_xyz, filtered_rgb


def _apply_tf(xyz: np.ndarray, t) -> np.ndarray:
    """Aplica una transformada TF (rotación + traslación) a un array (N,3)."""
    tr = t.transform.rotation
    qx, qy, qz, qw = tr.x, tr.y, tr.z, tr.w
    R = np.array([
        [1-2*(qy*qy+qz*qz), 2*(qx*qy-qz*qw), 2*(qx*qz+qy*qw)],
        [2*(qx*qy+qz*qw), 1-2*(qx*qx+qz*qz), 2*(qy*qz-qx*qw)],
        [2*(qx*qz-qy*qw), 2*(qy*qz+qx*qw), 1-2*(qx*qx+qy*qy)],
    ], dtype=np.float32)
    T = np.array([t.transform.translation.x,
                  t.transform.translation.y,
                  t.transform.translation.z], dtype=np.float32)
    return (R @ xyz.T).T + T


def _map_to_ply(xyz_map: np.ndarray,
                start_x: float, start_y: float, start_yaw: float) -> np.ndarray:
    """
    Convierte puntos del frame 'map' al frame PLY de RoboCup 2026.

    Spec: origen = centro del frente del robot en posición de inicio (nivel suelo).
          +Y = dirección inicial del robot, +Z = arriba.

    Derivación:
      robot forward en map = (cos θ, sin θ)
      PLY +Y = robot forward → PLY +X = robot right = (sin θ, -cos θ)
      Rotation matrix (map → PLY):
        row PLY_X: ( sin θ, -cos θ, 0)
        row PLY_Y: ( cos θ,  sin θ, 0)
        row PLY_Z: (   0,      0,   1)
    """
    # Origen = centro del frente del robot al inicio
    origin = np.array([
        start_x + math.cos(start_yaw) * ROBOT_HALF_LENGTH,
        start_y + math.sin(start_yaw) * ROBOT_HALF_LENGTH,
        0.0,
    ], dtype=np.float32)

    cs = math.cos(start_yaw)
    sn = math.sin(start_yaw)
    R_map_to_ply = np.array([
        [ sn, -cs, 0.0],   # PLY +X = robot right
        [ cs,  sn, 0.0],   # PLY +Y = robot forward
        [0.0, 0.0, 1.0],   # PLY +Z = up
    ], dtype=np.float32)

    translated = xyz_map - origin
    return (R_map_to_ply @ translated.T).T


def _yaw_from_quaternion(qx: float, qy: float, qz: float, qw: float) -> float:
    """Extrae el ángulo yaw de un cuaternión (rotación alrededor de Z)."""
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


class PointCloudAccumulator(Node):
    """Acumula nube de puntos 3D en el frame 'map' y exporta PLY."""

    def __init__(self):
        super().__init__('pointcloud_accumulator')

        self.declare_parameter('output_dir',  '/root/maps')
        self.declare_parameter('team_name',   'PedrosRescue')
        self.declare_parameter('mission',     'M1')
        self.declare_parameter('voxel_size',  VOXEL_SIZE)
        self.declare_parameter('max_range',   MAX_RANGE)
        self.declare_parameter('min_range',   MIN_RANGE)
        self.declare_parameter('sample_rate', 3)

        self._xyz_acc: List[np.ndarray] = []
        self._rgb_acc: List[np.ndarray] = []
        self._has_color = False
        self._msg_count = 0
        self._scan_count = 0
        self._total_saved = 0

        # Start time y pose inicial — se registran una sola vez
        self._start_time: Optional[datetime.datetime] = None
        self._start_x: Optional[float] = None
        self._start_y: Optional[float] = None
        self._start_yaw: Optional[float] = None

        # TF
        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        self.create_subscription(
            PointCloud2, '/camera/depth_registered/points', self._on_cloud, 5)
        self.create_subscription(
            PointCloud2, '/camera/depth/points', self._on_cloud, 5)
        self.create_subscription(
            LaserScan, '/ldlidar_node/scan', self._on_scan, 5)

        self.create_service(Trigger, '/save_pointcloud_ply', self._on_save)

        # Publisher nube acumulada en vivo para RViz
        self._accum_pub = self.create_publisher(
            PointCloud2, '/accumulated_pointcloud', 2)
        self.create_timer(2.0, self._publish_accumulated)

        # Timer que intenta capturar la pose de inicio (1 Hz)
        self._start_timer = self.create_timer(1.0, self._try_record_start)

        self.get_logger().info(
            'PointCloudAccumulator activo — acumulando lidar + cámara depth\n'
            '  Guardar PLY: ./scripts/run_slam_container.sh save-ply'
        )

    # ─── Captura de pose inicial ──────────────────────────────────

    def _try_record_start(self) -> None:
        """Registra posición y orientación inicial del robot (una sola vez)."""
        if self._start_x is not None:
            self._start_timer.cancel()
            return
        try:
            t = self._tf_buf.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            self._start_x   = t.transform.translation.x
            self._start_y   = t.transform.translation.y
            self._start_yaw = _yaw_from_quaternion(
                t.transform.rotation.x, t.transform.rotation.y,
                t.transform.rotation.z, t.transform.rotation.w)
            self._start_time = datetime.datetime.now()
            self._start_timer.cancel()
            self.get_logger().info(
                f'Pose inicial registrada: ({self._start_x:.3f}, '
                f'{self._start_y:.3f}), yaw={math.degrees(self._start_yaw):.1f}°  '
                f'Hora misión: {self._start_time.strftime("%H:%M:%S")}')
        except Exception:
            pass

    # ─── Callbacks ────────────────────────────────────────────────

    def _on_scan(self, msg: LaserScan) -> None:
        """Convierte un LaserScan 2D en puntos 3D a la altura del lidar."""
        self._scan_count += 1
        rate = self.get_parameter('sample_rate').value
        if self._scan_count % rate != 0:
            return
        if self._total_saved >= MAX_POINTS:
            return
        try:
            t = self._tf_buf.lookup_transform('map', msg.header.frame_id, rclpy.time.Time())
        except Exception:
            return

        angles = msg.angle_min + np.arange(len(msg.ranges)) * msg.angle_increment
        ranges = np.array(msg.ranges, dtype=np.float32)
        r_min = self.get_parameter('min_range').value
        r_max = self.get_parameter('max_range').value
        valid = np.isfinite(ranges) & (ranges >= r_min) & (ranges <= r_max)
        angles = angles[valid]
        ranges = ranges[valid]
        if len(ranges) == 0:
            return

        lx = ranges * np.cos(angles)
        ly = ranges * np.sin(angles)
        lz = np.zeros_like(lx)
        xyz = np.stack([lx, ly, lz], axis=-1)

        xyz_map = _apply_tf(xyz, t)

        v = self.get_parameter('voxel_size').value
        xyz_map, _ = _voxel_filter(xyz_map, None, v)
        self._xyz_acc.append(xyz_map)
        self._total_saved += len(xyz_map)

    def _on_cloud(self, msg: PointCloud2) -> None:
        self._msg_count += 1
        rate = self.get_parameter('sample_rate').value
        if self._msg_count % rate != 0:
            return
        if self._total_saved >= MAX_POINTS:
            return

        try:
            t = self._tf_buf.lookup_transform(
                'map', msg.header.frame_id, msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.1))
        except Exception:
            try:
                t = self._tf_buf.lookup_transform(
                    'map', msg.header.frame_id, rclpy.time.Time())
            except Exception:
                return

        xyz, rgb = _pc2_to_xyz_rgb(msg)
        if len(xyz) == 0:
            return

        r_min = self.get_parameter('min_range').value
        r_max = self.get_parameter('max_range').value
        dist = np.linalg.norm(xyz, axis=1)
        mask = (dist >= r_min) & (dist <= r_max)
        xyz = xyz[mask]
        if rgb is not None:
            rgb = rgb[mask]
            self._has_color = True

        if len(xyz) == 0:
            return

        xyz_map = _apply_tf(xyz, t)

        v = self.get_parameter('voxel_size').value
        xyz_map, rgb = _voxel_filter(xyz_map, rgb, v)

        self._xyz_acc.append(xyz_map)
        if rgb is not None:
            self._rgb_acc.append(rgb)
        self._total_saved += len(xyz_map)

    def _on_save(self, _req, resp: Trigger.Response) -> Trigger.Response:
        if not self._xyz_acc:
            resp.success = False
            resp.message = 'No hay puntos acumulados todavía'
            return resp
        try:
            path = self._export_ply()
            resp.success = True
            resp.message = f'PLY guardado → {path}  ({self._total_saved:,} puntos)'
            self.get_logger().info(resp.message)
        except Exception as exc:
            resp.success = False
            resp.message = f'Error al exportar PLY: {exc}'
            self.get_logger().error(resp.message)
        return resp

    # ─── Publisher RViz ───────────────────────────────────────────

    def _publish_accumulated(self) -> None:
        """Publica la nube acumulada en /accumulated_pointcloud cada 2 s."""
        if not self._xyz_acc:
            return
        all_xyz = np.concatenate(self._xyz_acc, axis=0).astype(np.float32)

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.height = 1
        msg.width = len(all_xyz)
        msg.fields = [
            PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 12
        msg.row_step = 12 * len(all_xyz)
        msg.is_dense = True
        msg.data = all_xyz.tobytes()
        self._accum_pub.publish(msg)

    # ─── Exportación PLY ──────────────────────────────────────────

    def _export_ply(self) -> str:
        all_xyz = np.concatenate(self._xyz_acc, axis=0).astype(np.float32)
        has_rgb = self._has_color and len(self._rgb_acc) == len(self._xyz_acc)
        all_rgb = np.concatenate(self._rgb_acc, axis=0) if has_rgb else None

        v = self.get_parameter('voxel_size').value
        all_xyz, all_rgb = _voxel_filter(all_xyz, all_rgb, v)

        # Transformar al sistema de coordenadas PLY de RoboCup 2026
        if self._start_x is not None:
            all_xyz = _map_to_ply(all_xyz,
                                  self._start_x, self._start_y, self._start_yaw)
        else:
            self.get_logger().warn(
                'Pose inicial no registrada — PLY guardado en frame map (sin transform)')

        n = len(all_xyz)

        # Nombre con hora de INICIO de misión (no la hora de guardado)
        if self._start_time is not None:
            ts = self._start_time.strftime('%H-%M-%S')
        else:
            ts = datetime.datetime.now().strftime('%H-%M-%S')

        team  = self.get_parameter('team_name').value
        miss  = self.get_parameter('mission').value
        fname = f'RoboCup2026-{team}-{miss}-{ts}-map.ply'
        out_dir = self.get_parameter('output_dir').value
        os.makedirs(out_dir, exist_ok=True)
        filepath = os.path.join(out_dir, fname)

        with open(filepath, 'w') as f:
            # Cabecera PLY ASCII — formato RoboCup 2026
            f.write('ply\n')
            f.write('format ascii 1.0\n')
            f.write(f'comment Team: {team}\n')
            f.write(f'comment Mission: {miss}\n')
            f.write(f'comment StartTime: {ts.replace("-", ":")}\n')
            f.write(f'comment Origin: center-front-robot floor height start position\n')
            f.write(f'comment Axes: +Y=forward +Z=up scale=meters\n')
            f.write(f'element vertex {n}\n')
            f.write('property float x\n')
            f.write('property float y\n')
            f.write('property float z\n')
            if all_rgb is not None:
                f.write('property uchar red\n')
                f.write('property uchar green\n')
                f.write('property uchar blue\n')
            f.write('end_header\n')

            if all_rgb is not None:
                for i in range(n):
                    x, y, z = all_xyz[i]
                    r, g, b  = all_rgb[i]
                    f.write(f'{x:.4f} {y:.4f} {z:.4f} {int(r)} {int(g)} {int(b)}\n')
            else:
                for i in range(n):
                    x, y, z = all_xyz[i]
                    f.write(f'{x:.4f} {y:.4f} {z:.4f}\n')

        return filepath


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudAccumulator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
