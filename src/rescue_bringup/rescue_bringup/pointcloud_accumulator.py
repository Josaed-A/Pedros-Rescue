#!/usr/bin/env python3
"""
pointcloud_accumulator.py
Acumula la nube de puntos 3D en el frame 'map' y la exporta como PLY ASCII.

Fuente principal:
  /camera/depth_registered/points  (XYZRGB — cámara depth + color alineado)
  Fallback: /camera/depth/points   (XYZ  — solo profundidad)

La nube acumulada se publica en /accumulated_pointcloud (frame: map) para
que RViz la muestre creciendo con el tiempo, igual que el mapa SLAM.

Servicio:
  /save_pointcloud_ply  (std_srvs/srv/Trigger)

Formato PLY RoboCup 2026 (páginas 20-22):
  • Campos mínimos:  x y z
  • Campos bonus (+25% score): r g b  (nube coloreada con RGB de la cámara)
  • Nombre:  RoboCup2026-{team}-{mission}-{HH-MM-SS}-map.ply
  • Origen:  centro del frente del robot, nivel del suelo, posición de inicio
  • Rotación: +Y = dirección inicial del robot, +Z = arriba
"""

import datetime
import math
import os
from typing import Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_srvs.srv import Trigger
import tf2_ros

ROBOT_HALF_LENGTH = 0.175   # m — distancia del centro al frente del robot
MAX_POINTS   = 2_000_000    # límite para el buffer acumulado
DISPLAY_MAX  = 300_000      # max puntos publicados a RViz por mensaje


# ── Decodificación PointCloud2 ─────────────────────────────────────

def _pc2_to_xyz_rgb(msg: PointCloud2) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    fields = {f.name: f for f in msg.fields}
    n = msg.width * msg.height
    raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, msg.point_step)

    def _col(name):
        off = fields[name].offset
        return raw[:, off:off+4].view(np.float32).reshape(-1)

    x, y, z = _col('x'), _col('y'), _col('z')
    xyz = np.stack([x, y, z], axis=-1)
    valid = np.isfinite(xyz).all(axis=1)
    xyz = xyz[valid]

    rgb = None
    fc = fields.get('rgb') or fields.get('rgba')
    if fc is not None:
        c_raw = raw[valid, fc.offset:fc.offset+4]
        rgb = np.stack([c_raw[:, 2], c_raw[:, 1], c_raw[:, 0]], axis=-1).astype(np.uint8)

    return xyz, rgb


# ── Filtros ────────────────────────────────────────────────────────

def _voxel_filter(xyz: np.ndarray, rgb: Optional[np.ndarray],
                  voxel_size: float) -> Tuple[np.ndarray, Optional[np.ndarray]]:
    if len(xyz) == 0:
        return xyz, rgb
    origin = xyz.min(axis=0)
    idx = ((xyz - origin) / voxel_size).astype(np.int32)
    keys = idx[:, 0] * 100_000_000 + idx[:, 1] * 100_000 + idx[:, 2]
    _, inv = np.unique(keys, return_index=True)
    return xyz[inv], (rgb[inv] if rgb is not None else None)


# ── Transformada TF ─────────────────────────────────────────────────

def _apply_tf(xyz: np.ndarray, t) -> np.ndarray:
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


# ── Conversión frame map → frame PLY RoboCup ───────────────────────

def _map_to_ply(xyz_map: np.ndarray,
                start_x: float, start_y: float, start_yaw: float) -> np.ndarray:
    origin = np.array([
        start_x + math.cos(start_yaw) * ROBOT_HALF_LENGTH,
        start_y + math.sin(start_yaw) * ROBOT_HALF_LENGTH,
        0.0,
    ], dtype=np.float32)
    cs, sn = math.cos(start_yaw), math.sin(start_yaw)
    R = np.array([
        [ sn, -cs, 0.0],
        [ cs,  sn, 0.0],
        [0.0, 0.0, 1.0],
    ], dtype=np.float32)
    return (R @ (xyz_map - origin).T).T


def _yaw_from_quat(qx, qy, qz, qw):
    return math.atan2(2.0 * (qw*qz + qx*qy), 1.0 - 2.0 * (qy*qy + qz*qz))


# ── Nodo principal ─────────────────────────────────────────────────

class PointCloudAccumulator(Node):

    def __init__(self):
        super().__init__('pointcloud_accumulator')

        self.declare_parameter('output_dir',  '/root/maps')
        self.declare_parameter('team_name',   'PedrosRescue')
        self.declare_parameter('mission',     'M1')
        self.declare_parameter('voxel_size',  0.03)   # 3 cm — balance precisión/memoria
        self.declare_parameter('max_range',   4.0)
        self.declare_parameter('min_range',   0.3)
        self.declare_parameter('sample_rate', 6)      # 1 de cada N frames de cámara

        # Buffer acumulado único (evita concatenar lista entera cada publicación)
        self._xyz: np.ndarray = np.empty((0, 3), dtype=np.float32)
        self._rgb: Optional[np.ndarray] = None
        self._has_color = False
        self._dirty = False
        self._pts_since_filter = 0   # puntos añadidos desde el último voxel global

        # Contadores de muestra independientes
        self._cloud_count = 0

        # Pose inicial para PLY RoboCup
        self._start_time: Optional[datetime.datetime] = None
        self._start_x: Optional[float] = None
        self._start_y: Optional[float] = None
        self._start_yaw: Optional[float] = None

        # TF
        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        # Suscripciones — solo UNA nube de cámara (registrada con color preferida)
        # Con sensor_data QoS para tolerar publishers BEST_EFFORT
        self._sub_colored = self.create_subscription(
            PointCloud2, '/camera/depth_registered/points',
            self._on_cloud, qos_profile_sensor_data)
        self._sub_depth = self.create_subscription(
            PointCloud2, '/camera/depth/points',
            self._on_depth_only, qos_profile_sensor_data)

        # Servicio guardar PLY
        self.create_service(Trigger, '/save_pointcloud_ply', self._on_save)

        # Publisher nube acumulada → RViz
        self._pub = self.create_publisher(PointCloud2, '/accumulated_pointcloud', 2)
        self.create_timer(3.0, self._publish_accumulated)

        # Captura pose inicial (1 Hz, se cancela cuando la consigue)
        self._start_timer = self.create_timer(1.0, self._try_record_start)

        self.get_logger().info(
            'PointCloudAccumulator listo — nube 3D crecerá como el mapa SLAM\n'
            '  RViz: /accumulated_pointcloud  |  Guardar PLY: save-ply')

    # ─── Pose inicial ─────────────────────────────────────────────

    def _try_record_start(self):
        if self._start_x is not None:
            self._start_timer.cancel()
            return
        try:
            t = self._tf_buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            self._start_x   = t.transform.translation.x
            self._start_y   = t.transform.translation.y
            self._start_yaw = _yaw_from_quat(t.transform.rotation.x, t.transform.rotation.y,
                                              t.transform.rotation.z, t.transform.rotation.w)
            self._start_time = datetime.datetime.now()
            self._start_timer.cancel()
            self.get_logger().info(
                f'Pose inicial: ({self._start_x:.2f}, {self._start_y:.2f}) '
                f'yaw={math.degrees(self._start_yaw):.1f}°')
        except Exception:
            pass

    # ─── Callback nube coloreada (depth_registered) ───────────────

    def _on_cloud(self, msg: PointCloud2):
        """Nube XYZRGB — tiene color: la usamos preferentemente."""
        self._cloud_count += 1
        rate = self.get_parameter('sample_rate').value
        if self._cloud_count % rate != 0:
            return
        if len(self._xyz) >= MAX_POINTS:
            return
        self._accumulate(msg, expect_color=True)

    # ─── Callback solo profundidad ─────────────────────────────────

    def _on_depth_only(self, msg: PointCloud2):
        """Nube XYZ — solo si aún no hay nube coloreada disponible."""
        if self._has_color:
            return   # ignorar: ya tenemos color desde depth_registered
        self._cloud_count += 1
        rate = self.get_parameter('sample_rate').value
        if self._cloud_count % rate != 0:
            return
        if len(self._xyz) >= MAX_POINTS:
            return
        self._accumulate(msg, expect_color=False)

    # ─── Acumulación ──────────────────────────────────────────────

    def _accumulate(self, msg: PointCloud2, expect_color: bool):
        # TF lookup
        try:
            t = self._tf_buf.lookup_transform(
                'map', msg.header.frame_id, msg.header.stamp,
                timeout=rclpy.duration.Duration(seconds=0.05))
        except Exception:
            try:
                t = self._tf_buf.lookup_transform(
                    'map', msg.header.frame_id, rclpy.time.Time())
            except Exception:
                return

        xyz, rgb = _pc2_to_xyz_rgb(msg)
        if len(xyz) == 0:
            return

        # Filtro de rango
        r_min = self.get_parameter('min_range').value
        r_max = self.get_parameter('max_range').value
        dist  = np.linalg.norm(xyz, axis=1)
        mask  = (dist >= r_min) & (dist <= r_max)
        xyz   = xyz[mask]
        if len(xyz) == 0:
            return
        if rgb is not None:
            rgb = rgb[mask]

        # Transformar a frame map
        xyz_map = _apply_tf(xyz, t)

        # Voxel filter por frame (para no añadir ruido denso)
        v = self.get_parameter('voxel_size').value
        xyz_map, rgb = _voxel_filter(xyz_map, rgb, v)

        # Añadir al buffer acumulado
        if len(self._xyz) == 0:
            self._xyz = xyz_map
            if rgb is not None:
                self._rgb = rgb
                self._has_color = True
        else:
            self._xyz = np.concatenate([self._xyz, xyz_map], axis=0)
            if rgb is not None:
                self._has_color = True
                if self._rgb is None:
                    # Primera nube coloreada — rellena pasado con gris
                    n_prev = len(self._xyz) - len(xyz_map)
                    pad = np.full((n_prev, 3), 128, dtype=np.uint8)
                    self._rgb = np.concatenate([pad, rgb], axis=0)
                else:
                    self._rgb = np.concatenate([self._rgb, rgb], axis=0)
            elif self._rgb is not None:
                # Frame sin color pero buffer tiene color → añadir gris
                pad = np.full((len(xyz_map), 3), 128, dtype=np.uint8)
                self._rgb = np.concatenate([self._rgb, pad], axis=0)

        self._dirty = True
        self._pts_since_filter += len(xyz_map)

        # Voxel filter global cada 100k puntos nuevos (mantiene tamaño manejable)
        if self._pts_since_filter >= 100_000:
            self._xyz, self._rgb = _voxel_filter(self._xyz, self._rgb, v)
            self._pts_since_filter = 0
            self.get_logger().info(
                f'Nube acumulada: {len(self._xyz):,} puntos'
                + (' (con color)' if self._has_color else ''))

    # ─── Publicar para RViz ────────────────────────────────────────

    def _publish_accumulated(self):
        if not self._dirty or len(self._xyz) == 0:
            return
        self._dirty = False

        xyz = self._xyz
        rgb = self._rgb

        # Submuestreo para no saturar DDS (max DISPLAY_MAX puntos)
        if len(xyz) > DISPLAY_MAX:
            idx = np.random.choice(len(xyz), DISPLAY_MAX, replace=False)
            xyz = xyz[idx]
            if rgb is not None:
                rgb = rgb[idx]

        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.height = 1
        msg.width  = len(xyz)
        msg.is_bigendian = False
        msg.is_dense     = True

        if rgb is not None and len(rgb) == len(xyz):
            # XYZRGB — empaquetado: 16 bytes por punto (x,y,z,_,rgb empacado)
            r = rgb[:, 0].astype(np.uint32)
            g = rgb[:, 1].astype(np.uint32)
            b = rgb[:, 2].astype(np.uint32)
            packed = ((r << 16) | (g << 8) | b).astype(np.float32)
            data = np.column_stack([xyz, packed.view(np.float32)]).astype(np.float32)
            msg.fields = [
                PointField(name='x',   offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name='y',   offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name='z',   offset=8,  datatype=PointField.FLOAT32, count=1),
                PointField(name='rgb', offset=12, datatype=PointField.FLOAT32, count=1),
            ]
            msg.point_step = 16
            msg.row_step   = 16 * len(xyz)
            msg.data       = data.tobytes()
        else:
            # XYZ
            msg.fields = [
                PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
                PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
                PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
            ]
            msg.point_step = 12
            msg.row_step   = 12 * len(xyz)
            msg.data       = xyz.tobytes()

        self._pub.publish(msg)

    # ─── Guardar PLY ──────────────────────────────────────────────

    def _on_save(self, _req, resp: Trigger.Response) -> Trigger.Response:
        if len(self._xyz) == 0:
            resp.success = False
            resp.message = 'No hay puntos acumulados todavía'
            return resp
        try:
            path = self._export_ply()
            resp.success = True
            resp.message = f'PLY guardado → {path}  ({len(self._xyz):,} puntos)'
            self.get_logger().info(resp.message)
        except Exception as exc:
            resp.success = False
            resp.message = f'Error al exportar PLY: {exc}'
            self.get_logger().error(resp.message)
        return resp

    def _export_ply(self) -> str:
        v     = self.get_parameter('voxel_size').value
        xyz, rgb = _voxel_filter(self._xyz.copy(), self._rgb, v)
        n     = len(xyz)

        # Transformar al sistema de coordenadas PLY RoboCup 2026
        if self._start_x is not None:
            xyz = _map_to_ply(xyz, self._start_x, self._start_y, self._start_yaw)
        else:
            self.get_logger().warn('Pose inicial no registrada — PLY en frame map')

        ts    = (self._start_time or datetime.datetime.now()).strftime('%H-%M-%S')
        team  = self.get_parameter('team_name').value
        miss  = self.get_parameter('mission').value
        fname = f'RoboCup2026-{team}-{miss}-{ts}-map.ply'
        out   = self.get_parameter('output_dir').value
        os.makedirs(out, exist_ok=True)
        path  = os.path.join(out, fname)

        with open(path, 'w') as f:
            f.write('ply\nformat ascii 1.0\n')
            f.write(f'comment Team: {team}\ncomment Mission: {miss}\n')
            f.write(f'comment StartTime: {ts.replace("-",":")}\n')
            f.write('comment Origin: center-front-robot floor height start position\n')
            f.write('comment Axes: +Y=forward +Z=up scale=meters\n')
            f.write(f'element vertex {n}\n')
            f.write('property float x\nproperty float y\nproperty float z\n')
            if rgb is not None and len(rgb) == n:
                f.write('property uchar red\nproperty uchar green\nproperty uchar blue\n')
            f.write('end_header\n')
            for i in range(n):
                x, y, z = xyz[i]
                if rgb is not None and len(rgb) == n:
                    r, g, b = rgb[i]
                    f.write(f'{x:.4f} {y:.4f} {z:.4f} {int(r)} {int(g)} {int(b)}\n')
                else:
                    f.write(f'{x:.4f} {y:.4f} {z:.4f}\n')
        return path


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
