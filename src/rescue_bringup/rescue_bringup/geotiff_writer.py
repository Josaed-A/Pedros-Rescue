#!/usr/bin/env python3
"""
geotiff_writer.py
Genera mapas GeoTIFF conformes con RoboCup Rescue 2026 (páginas 22-23).

Suscribe: /map (OccupancyGrid de slam_toolbox)
Trackea:  pose del robot vía TF  map → base_footprint
Servicio: /save_geotiff  (std_srvs/srv/Trigger) → genera y guarda el TIFF

Elementos visuales implementados (spec páginas 22-23):
  ● Fondo checkerboard gris claro/oscuro  — zonas inexploradas (100 cm cuadros)
  ● Gradiente blanco-gris por confianza   — zonas libres exploradas
  ● Grid negro 50 cm                      — sobre zonas exploradas
  ● Paredes azul oscuro  RGB(0,40,120)
  ● Ruta robot magenta   RGB(120,0,140)   (~2 cm de ancho)
  ● Flecha verde inicial  RGB(0,240,0)    — posición de inicio
  ● Escala exacta 1 m    RGB(0,50,140)    — esquina superior derecha
  ● Flechas orientación X↑ Y←            — esquina superior izquierda
  ● Texto filename        RGB(0,44,207)   — margen inferior
  ● Marcadores objetos (AprilTag/Hazmat/Objeto) con color y texto
"""

import datetime
import math
import os
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger
import tf2_ros

try:
    from PIL import Image, ImageDraw, ImageFont
    _PIL_OK = True
except ImportError:
    _PIL_OK = False


# ── Configuración de renderizado ───────────────────────────────────
RENDER_SCALE = 4    # píxeles por celda del mapa  (0.05 m/celda → 80 px/m)
MARGIN_TOP   = 95   # px encima del mapa (escala + orientación)
MARGIN_SIDE  = 12   # px a cada lado
MARGIN_BOT   = 38   # px debajo del mapa (filename)

# ── Colores RoboCup 2026 (páginas 22-23) ──────────────────────────
C_FILENAME   = (  0,  44, 207)   # texto filename       — azul oscuro
C_ANNOT      = (  0,  50, 140)   # escala / orientación — azul oscuro
C_WALL       = (  0,  40, 120)   # paredes / obstáculos — azul oscuro
C_ROBOT_INIT = (  0, 240,   0)   # posición inicial     — verde
C_PATH       = (120,   0, 140)   # ruta robot           — magenta
C_APRILTAG   = (255, 200,   0)   # AprilTag             — amarillo
C_HAZMAT     = (255, 100,  30)   # hazmat               — naranja
C_REAL_OBJ   = (240,  10,  10)   # objeto real          — rojo
C_UNEXPL_A   = (226, 226, 227)   # checkerboard A       — gris claro
C_UNEXPL_B   = (237, 237, 238)   # checkerboard B       — gris más claro
C_GRID       = (190, 190, 191)   # grid explorado       — gris
C_BG         = (200, 200, 200)   # fondo del canvas


class GeotiffWriter(Node):
    """Nodo ROS 2 que genera mapas GeoTIFF conformes con RoboCup Rescue 2026."""

    def __init__(self):
        super().__init__('geotiff_writer')

        if not _PIL_OK:
            self.get_logger().fatal(
                'Pillow no está instalado. Ejecuta: pip3 install Pillow')
            raise RuntimeError('Pillow requerido')

        # Parámetros
        self.declare_parameter('output_dir',  '/root/maps')
        self.declare_parameter('team_name',   'PedrosRescue')
        self.declare_parameter('mission',     'M1')
        self.declare_parameter('path_step_m', 0.08)   # m mínimos entre muestras de ruta

        # Estado interno
        self._map: Optional[OccupancyGrid] = None
        self._path: List[Tuple[float, float]] = []
        self._init_pose: Optional[Tuple[float, float]] = None
        self._objects: List[dict] = []   # {type, name, wx, wy}

        # TF para rastrear posición del robot
        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        # Suscripciones
        self.create_subscription(OccupancyGrid, '/map', self._on_map, 10)

        # Servicio para guardar el mapa
        self.create_service(Trigger, '/save_geotiff', self._on_save)

        # Timer de muestreo de ruta (5 Hz)
        self.create_timer(0.2, self._sample_path)

        self.get_logger().info(
            'GeotiffWriter activo — para guardar el mapa GeoTIFF ejecuta:\n'
            '  ./scripts/run_slam_container.sh save-geotiff\n'
            '  (o: ros2 service call /save_geotiff std_srvs/srv/Trigger "{}")'
        )

    # ─── Callbacks ────────────────────────────────────────────────

    def _on_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _sample_path(self) -> None:
        """Muestrea la posición del robot desde TF y la acumula en la ruta."""
        if self._map is None:
            return
        try:
            t = self._tf_buf.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time())
            wx = t.transform.translation.x
            wy = t.transform.translation.y

            if self._init_pose is None:
                self._init_pose = (wx, wy)

            step = self.get_parameter('path_step_m').value
            if (not self._path or
                    math.hypot(wx - self._path[-1][0],
                               wy - self._path[-1][1]) >= step):
                self._path.append((wx, wy))
        except Exception:
            pass   # TF aún no disponible

    def _on_save(self, _req, resp: Trigger.Response) -> Trigger.Response:
        if self._map is None:
            resp.success = False
            resp.message = 'Sin datos de mapa todavía'
            return resp
        try:
            filepath = self._build_geotiff()
            resp.success = True
            resp.message = f'GeoTIFF guardado → {filepath}'
            self.get_logger().info(resp.message)
        except Exception as exc:
            resp.success = False
            resp.message = f'Error al generar GeoTIFF: {exc}'
            self.get_logger().error(resp.message)
        return resp

    # ─── Utilidades de coordenadas ─────────────────────────────────

    def _world_to_px(self, wx: float, wy: float, info,
                     off_x: int, off_y: int) -> Tuple[int, int]:
        """Coordenadas mundo (m) → píxel en imagen (con flip de eje Y)."""
        cx = (wx - info.origin.position.x) / info.resolution
        cy = (wy - info.origin.position.y) / info.resolution
        px = int(cx * RENDER_SCALE) + off_x
        py = int((info.height - 1 - cy) * RENDER_SCALE) + off_y
        return px, py

    # ─── Generación del GeoTIFF ────────────────────────────────────

    def _build_geotiff(self) -> str:
        info = self._map.info
        raw  = np.array(self._map.data, dtype=np.int16).reshape(
            info.height, info.width)

        S    = RENDER_SCALE
        mw   = info.width  * S           # ancho del mapa en px
        mh   = info.height * S           # alto  del mapa en px
        tw   = mw + MARGIN_SIDE * 2      # ancho total del canvas
        th   = mh + MARGIN_TOP + MARGIN_BOT
        ox   = MARGIN_SIDE               # esquina izquierda del mapa en canvas
        oy   = MARGIN_TOP                # esquina superior del mapa en canvas
        ppm  = S / info.resolution       # píxeles por metro  (ej. 80 px/m @ 4×)

        # ── 1. Capa raster (operaciones numpy, sin bucles Python) ──

        big = np.repeat(np.repeat(raw, S, axis=0), S, axis=1)
        rgb = np.full((mh, mw, 3), C_BG, dtype=np.uint8)

        # Checkerboard para zonas inexploradas (cuadros de 100 cm = ppm px)
        cs  = max(1, int(ppm))
        yi  = np.arange(mh)[:, None]
        xi  = np.arange(mw)[None, :]
        ck  = ((xi // cs) + (yi // cs)) % 2
        unk = (big == -1)
        rgb[unk & (ck == 0)] = C_UNEXPL_A
        rgb[unk & (ck == 1)] = C_UNEXPL_B

        # Zonas libres exploradas → gradiente blanco-gris
        free = (big >= 0) & (big < 90)
        vals = np.where(free, big, 0).astype(np.float32)
        # occupancy 0 → 255 (blanco); 89 → ~134 (gris medio)
        lum = np.clip(255 - vals * (255 - 128) / 100, 128, 255).astype(np.uint8)
        rgb[free, 0] = lum[free]
        rgb[free, 1] = lum[free]
        rgb[free, 2] = lum[free]

        # Paredes → azul oscuro
        rgb[big >= 90] = C_WALL

        # Grid 50 cm sobre zonas exploradas
        gs = max(1, int(ppm * 0.5))
        for row in range(0, mh, gs):
            mask = free[row]
            rgb[row][mask] = C_GRID
        for col in range(0, mw, gs):
            mask = free[:, col]
            rgb[mask, col] = C_GRID

        # ── 2. Canvas PIL ──────────────────────────────────────────

        canvas = Image.new('RGB', (tw, th), C_BG)
        canvas.paste(Image.fromarray(rgb, 'RGB'), (ox, oy))
        draw = ImageDraw.Draw(canvas)

        # Borde del mapa
        draw.rectangle(
            [(ox, oy), (ox + mw - 1, oy + mh - 1)],
            outline=(60, 60, 60), width=2)

        # Fuentes (DejaVu preinstalado en imágenes ROS Docker)
        dj = '/usr/share/fonts/truetype/dejavu/'
        try:
            f_big = ImageFont.truetype(dj + 'DejaVuSans-Bold.ttf', 20)
            f_med = ImageFont.truetype(dj + 'DejaVuSans-Bold.ttf', 14)
            f_sml = ImageFont.truetype(dj + 'DejaVuSans.ttf', 11)
        except Exception:
            f_big = f_med = f_sml = ImageFont.load_default()

        # ── 3. RUTA DEL ROBOT (magenta, ~2 cm de ancho) ───────────

        path_w = max(2, int(ppm * 0.02))
        if len(self._path) >= 2:
            pts = [self._world_to_px(wx, wy, info, ox, oy)
                   for wx, wy in self._path]
            draw.line(pts, fill=C_PATH, width=path_w)

        # ── 4. POSICIÓN INICIAL DEL ROBOT (flecha verde hacia arriba)

        if self._init_pose is not None:
            ipx, ipy = self._world_to_px(*self._init_pose, info, ox, oy)
        else:
            ipx = ox + mw // 2
            ipy = oy + mh // 2
        ar = max(10, int(ppm * 0.175))   # ~35 cm de tamaño
        draw.polygon([
            (ipx,          ipy - ar),          # punta (arriba)
            (ipx - ar // 2, ipy + ar // 2),    # base izquierda
            (ipx + ar // 2, ipy + ar // 2),    # base derecha
        ], fill=C_ROBOT_INIT, outline=(0, 180, 0))

        # ── 5. MARCADORES DE OBJETOS ───────────────────────────────

        for obj in self._objects:
            px, py = self._world_to_px(obj['wx'], obj['wy'], info, ox, oy)
            self._draw_marker(draw, obj['type'], obj['name'], px, py, ppm, f_sml)

        # ── 6. BARRA DE ESCALA — esquina superior derecha ──────────
        # Línea exacta de 1 m con marcas en los extremos

        sb_r  = ox + mw - 14          # extremo derecho
        sb_t  = oy - 58               # posición vertical
        sb_l  = sb_r - int(ppm)       # extremo izquierdo (exactamente 1 m)
        sb_my = sb_t + 20             # y central de la línea

        draw.text((sb_l, sb_t), '1 m', fill=C_ANNOT, font=f_med)
        draw.line([(sb_l, sb_my), (sb_r, sb_my)], fill=C_ANNOT, width=3)
        draw.line([(sb_l, sb_my - 7), (sb_l, sb_my + 7)], fill=C_ANNOT, width=2)
        draw.line([(sb_r, sb_my - 7), (sb_r, sb_my + 7)], fill=C_ANNOT, width=2)

        # ── 7. FLECHAS DE ORIENTACIÓN — esquina superior izquierda ─
        # X apunta hacia arriba (REP-103: X forward = arriba en el mapa)
        # Y apunta hacia la izquierda (sistema dextrógiro)

        ab_x = ox + 14               # ancla horizontal
        ab_y = oy - 82               # ancla vertical
        al   = max(30, int(ppm * 0.45))   # longitud flecha (~45 cm)
        acx  = ab_x + al // 2        # centro horizontal del conjunto

        # Flecha X (arriba)
        x_tip = ab_y + 6
        x_bas = ab_y + al
        draw.line([(acx, x_bas), (acx, x_tip + 10)], fill=C_ANNOT, width=3)
        draw.polygon([(acx, x_tip),
                      (acx - 5, x_tip + 11),
                      (acx + 5, x_tip + 11)], fill=C_ANNOT)
        draw.text((acx + 7, x_tip - 2), 'X', fill=C_ANNOT, font=f_med)

        # Flecha Y (izquierda)
        y_tip_x = ab_x + 6
        y_bas_x = ab_x + al
        draw.line([(y_bas_x, x_bas), (y_tip_x + 10, x_bas)], fill=C_ANNOT, width=3)
        draw.polygon([(y_tip_x, x_bas),
                      (y_tip_x + 11, x_bas - 5),
                      (y_tip_x + 11, x_bas + 5)], fill=C_ANNOT)
        draw.text((y_tip_x - 4, x_bas + 7), 'Y', fill=C_ANNOT, font=f_med)

        # ── 8. TEXTO FILENAME — margen inferior ────────────────────

        ts    = datetime.datetime.now().strftime('%H-%M-%S')
        team  = self.get_parameter('team_name').value
        miss  = self.get_parameter('mission').value
        fname = f'RoboCup2026-{team}-{miss}-{ts}.tiff'
        draw.text((ox, oy + mh + 9), fname, fill=C_FILENAME, font=f_big)

        # ── 9. Guardar ─────────────────────────────────────────────

        out_dir = self.get_parameter('output_dir').value
        os.makedirs(out_dir, exist_ok=True)
        filepath = os.path.join(out_dir, fname)
        canvas.save(filepath, format='TIFF', compression='raw')
        return filepath

    # ─── Renderizado de marcadores de objetos ─────────────────────

    def _draw_marker(self, draw, mtype: str, name: str,
                     px: int, py: int, ppm: float, font) -> None:
        """Dibuja un marcador de objeto detectado sobre el mapa."""
        if mtype == 'ar_code':
            # Círculo amarillo ~35 cm de diámetro con texto blanco "#N"
            r = max(8, int(ppm * 0.175))
            draw.ellipse(
                [(px - r, py - r), (px + r, py + r)], fill=C_APRILTAG)
            draw.text(
                (px - r // 2, py - 7), f'#{name}',
                fill=(255, 255, 255), font=font)

        elif mtype == 'hazmat_sign':
            # Diamante naranja ~30 cm con primeras 2 letras
            h = max(8, int(ppm * 0.30))
            draw.polygon(
                [(px, py - h), (px + h, py), (px, py + h), (px - h, py)],
                fill=C_HAZMAT)
            draw.text(
                (px - h // 2, py - 7), name[:2].upper(),
                fill=(255, 255, 255), font=font)

        elif mtype == 'real_object':
            # Diamante rojo ~30 cm con primeras 2 letras
            h = max(8, int(ppm * 0.30))
            draw.polygon(
                [(px, py - h), (px + h, py), (px, py + h), (px - h, py)],
                fill=C_REAL_OBJ)
            draw.text(
                (px - h // 2, py - 7), name[:2].upper(),
                fill=(255, 255, 255), font=font)


def main(args=None):
    rclpy.init(args=args)
    node = GeotiffWriter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
