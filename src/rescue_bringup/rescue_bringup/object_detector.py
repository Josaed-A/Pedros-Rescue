#!/usr/bin/env python3
"""
object_detector.py
Detecta objetos de interés RoboCup Rescue 2026 y genera el CSV de detecciones.

Detectores implementados:
  1. AprilTag Standard41h12 → tipo 'ar_code'        (1 pt)
  2. Hazmat signs via YOLO custom (49 clases) → tipo 'hazmat_sign'  (2 pts)
     Fallback: detector HSV (naranja) si no hay modelo entrenado.
  3. Objetos físicos via YOLO (ultralytics): → tipo 'real_object'  (10 pts)
        backpack, hard hat, fire extinguisher, person (víctima), bottle

Entradas:
  /camera/color/image_raw        (sensor_msgs/Image)
  /camera/color/camera_info      (sensor_msgs/CameraInfo)
  /camera/depth/image_raw        (sensor_msgs/Image — depth float32 en mm o m)

Salidas:
  /object_detections             (std_msgs/String — JSON por detección, para geotiff_writer)
  /object_detection_markers      (visualization_msgs/MarkerArray — visualización RViz)
  /save_detection_csv            (std_srvs/srv/Trigger)
  alerts_dir/<camera_id>/*.jpg+.json  (captura + descripción cuando una señal hazmat
                                        se confirma durante `alert_confirm_frames`
                                        frames seguidos — ver hazmat/alertas_detectadas/)

CSV formato RoboCup 2026:
  detection,time,type,name,x,y,z,robot,mode
  (x, y, z en metros en el frame 'map')
"""

import csv
import datetime
import json
import math
import os
import time
from collections import OrderedDict
from typing import Dict, List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image, CameraInfo, CompressedImage
from std_msgs.msg import String as StringMsg
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

try:
    import cv2
    _CV2_OK = True
except ImportError:
    _CV2_OK = False

try:
    from cv_bridge import CvBridge
    _BRIDGE_OK = True
except ImportError:
    _BRIDGE_OK = False

try:
    from ultralytics import YOLO as _YOLO
    _YOLO_OK = True
except ImportError:
    _YOLO_OK = False

from rescue_bringup.hazmat_common import run_hazmat_yolo


# ── Objetos YOLO que consideramos "objetos de misión" ─────────────────────────
# Claves: nombre COCO → nombre display para CSV
YOLO_TARGET_CLASSES: Dict[str, str] = {
    'backpack':         'Backpack',
    'handbag':          'Bag',
    'suitcase':         'Suitcase',
    'fire hydrant':     'FireHydrant',
    'bottle':           'Bottle',
    'person':           'Victim',
    'teddy bear':       'Doll',
    'sports ball':      'Ball',
    'chair':            'Chair',
    'cell phone':       'Phone',
}

# Umbral de confianza YOLO
YOLO_CONF = 0.50

# Rango válido del sensor de profundidad (m)
DEPTH_MIN = 0.3
DEPTH_MAX = 4.0

# Deduplicación: si una detección del mismo tipo está a < DEDUP_DIST m → misma
DEDUP_DIST = 0.5   # m

# Intervalo de detección (segundos) — no procesar cada frame
DETECT_INTERVAL = 0.5   # s

# Colores HSV para hazmat (naranja — valor ajustable según condiciones de luz)
HAZMAT_H_LO, HAZMAT_H_HI = 8, 22    # matiz (0-180 en OpenCV)
HAZMAT_S_LO = 120                    # saturación mínima
HAZMAT_V_LO = 100                    # brillo mínimo
HAZMAT_AREA_MIN = 500                # área mínima en píxeles²


class ObjectDetector(Node):
    """Detecta y localiza objetos RoboCup Rescue 2026 en 3D."""

    def __init__(self):
        super().__init__('object_detector')

        self.declare_parameter('output_dir',     '/workspace/maps')
        self.declare_parameter('team_name',      'SabanaHerons')
        self.declare_parameter('mission',        'M1')
        self.declare_parameter('country',        'Colombia')
        self.declare_parameter('robot_name',     'Pedro')
        self.declare_parameter('mode',           'T')
        self.declare_parameter('yolo_model',     'yolov8n.pt')
        self.declare_parameter('hazmat_model',   '')
        self.declare_parameter('hazmat_conf',    0.40)
        self.declare_parameter('enable_yolo',    True)
        self.declare_parameter('enable_apriltag', True)
        self.declare_parameter('enable_hazmat',  True)
        # hazmat_mode='local' (default): esta instancia carga su propio modelo
        # hazmat y lo corre ella misma — comportamiento de siempre, sin cambios.
        # hazmat_mode='worker': no carga ningun modelo; reenvia sus frames a un
        # hazmat_worker compartido (un solo modelo cargado, sirve a N camaras)
        # y mezcla los resultados que le llegan de vuelta en el mismo pipeline
        # (dibujo, alertas, CSV, RViz siguen identicos). Requiere `camera_id`.
        self.declare_parameter('hazmat_mode',        'local')
        # Buffer pequeño y acotado de frames recientes (por seq) esperando su
        # resultado hazmat — evita dibujar un resultado sobre un frame que no
        # lo produjo. Ver _submit_frame_to_worker / _on_hazmat_result.
        self.declare_parameter('frame_buffer_maxlen', 12)

        # ── Configuración de topics (compatible con ambos drivers) ──
        # use_compressed=false → driver oficial astra_camera / orbbec_camera (raw Image)
        # use_compressed=true  → driver del compañero astra_rgbd_camera_node (CompressedImage)
        self.declare_parameter('use_compressed',     False)
        self.declare_parameter('color_topic',        '/camera/color/image_raw')
        self.declare_parameter('depth_topic',        '/camera/depth/image_raw')
        self.declare_parameter('camera_info_topic',  '/camera/color/camera_info')
        self.declare_parameter('camera_frame_id',    'camera_optical_link')
        # require_depth=false → detecta sin profundidad (x=y=z=0), útil con Logitech
        self.declare_parameter('require_depth',      True)
        # Intrínsecos de la Astra Pro (fallback cuando no hay CameraInfo)
        self.declare_parameter('fx',           525.0)
        self.declare_parameter('fy',           525.0)
        self.declare_parameter('cx',           319.5)
        self.declare_parameter('cy',           239.5)
        # depth_scale: factor mm→m para la profundidad del compañero (uint16 → metros)
        self.declare_parameter('depth_scale',  0.001)

        # ── Capturas automaticas de alertas hazmat (evidencia para revision humana) ──
        # camera_id distingue instancias cuando corren dos object_detector en paralelo
        # (una por camara) — se usa como subcarpeta dentro de alerts_dir.
        self.declare_parameter('camera_id',            '')
        self.declare_parameter('alerts_dir',           '')
        # Una deteccion hazmat solo se considera "confirmada" (y se captura) tras
        # verse consistentemente durante N frames seguidos — evita guardar ruido
        # de un solo frame cuando el modelo parpadea entre clases.
        self.declare_parameter('alert_confirm_frames', 3)
        self.declare_parameter('alert_cooldown_sec',   20.0)
        self.declare_parameter('alert_pixel_tolerance', 80)

        if not _CV2_OK:
            self.get_logger().fatal('OpenCV (cv2) no encontrado — instala python3-opencv')
            raise RuntimeError('cv2 requerido')
        if not _BRIDGE_OK:
            self.get_logger().fatal('cv_bridge no encontrado')
            raise RuntimeError('cv_bridge requerido')

        self._bridge = CvBridge()
        self._cam_info: Optional[CameraInfo] = None
        self._depth_img: Optional[np.ndarray] = None
        self._color_img: Optional[np.ndarray] = None
        self._last_detect_time = 0.0

        # Detecciones acumuladas: lista de dicts
        self._detections: List[dict] = []
        self._det_counter = 0

        # ── Modo de deteccion hazmat: local (propio modelo) o worker (compartido) ──
        self._camera_id = self.get_parameter('camera_id').value
        self._hazmat_mode = self.get_parameter('hazmat_mode').value
        if self._hazmat_mode == 'worker' and not self._camera_id:
            self.get_logger().warn(
                "hazmat_mode='worker' requiere 'camera_id' — cayendo a modo 'local'")
            self._hazmat_mode = 'local'
        self._hazmat_submit_pub = None

        # Buffer de frames en vuelo hacia el worker: seq -> {'bgr', 'detections'}.
        # Pequeño y acotado (FIFO) — nunca se "reutiliza" una entrada vieja para
        # un resultado nuevo; si el resultado llega despues de que su entrada
        # fue expulsada, se descarta (ver _on_hazmat_result).
        self._frame_seq = 0
        self._frame_buffer: "OrderedDict[int, dict]" = OrderedDict()
        self._frame_buffer_maxlen = max(1, int(self.get_parameter('frame_buffer_maxlen').value))

        # ── Tracking + captura automatica de alertas hazmat ─────────────
        self._alert_confirm_frames = int(self.get_parameter('alert_confirm_frames').value)
        self._alert_cooldown_sec   = float(self.get_parameter('alert_cooldown_sec').value)
        self._alert_pixel_tol      = float(self.get_parameter('alert_pixel_tolerance').value)
        self._hazmat_track: Dict[str, dict] = {}   # nombre clase -> {count, cx, cy, last_seen}
        self._alert_last_saved: Dict[str, float] = {}  # nombre clase -> ultimo timestamp guardado

        alerts_dir_param = self.get_parameter('alerts_dir').value
        if alerts_dir_param:
            self._alerts_dir = os.path.join(alerts_dir_param, self._camera_id) \
                if self._camera_id else alerts_dir_param
            try:
                os.makedirs(self._alerts_dir, exist_ok=True)
                self.get_logger().info(f'Alertas hazmat -> {self._alerts_dir}')
            except OSError as exc:
                self.get_logger().warn(f'No se pudo crear alerts_dir: {exc}')
                self._alerts_dir = None
        else:
            self._alerts_dir = None

        # Hora de inicio de misión
        self._start_time: Optional[datetime.datetime] = None
        self._start_timer_obj = self.create_timer(1.0, self._try_record_start)

        # TF
        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        # AprilTag detector (OpenCV aruco)
        # DICT_APRILTAG_41h12 = 21 (some OpenCV ARM builds omit the named constant)
        _APRILTAG_DICT = getattr(cv2.aruco, 'DICT_APRILTAG_41h12', 21)
        self._aruco_detector = None
        if _CV2_OK and self.get_parameter('enable_apriltag').value:
            try:
                aruco_dict = cv2.aruco.getPredefinedDictionary(_APRILTAG_DICT)
                params = cv2.aruco.DetectorParameters()
                self._aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, params)
                self.get_logger().info('AprilTag detector (41h12) activo')
            except AttributeError:
                # OpenCV < 4.7: API antigua
                self._aruco_dict = cv2.aruco.Dictionary_get(_APRILTAG_DICT)
                self._aruco_params = cv2.aruco.DetectorParameters_create()
                self._aruco_detector = 'legacy'
                self.get_logger().info('AprilTag detector (41h12 legacy API) activo')

        # YOLO model (objetos COCO: personas, mochilas, etc.)
        self._yolo = None
        if _YOLO_OK and self.get_parameter('enable_yolo').value:
            model_path = self.get_parameter('yolo_model').value
            try:
                self._yolo = _YOLO(model_path)
                self.get_logger().info(f'YOLO (objetos) cargado: {model_path}')
            except Exception as exc:
                self.get_logger().warn(f'No se pudo cargar YOLO ({model_path}): {exc}')
        elif not _YOLO_OK and self.get_parameter('enable_yolo').value:
            self.get_logger().warn(
                'ultralytics no instalado — detección YOLO desactivada. '
                'Instala con: pip3 install ultralytics')

        # Modelo hazmat entrenado (reemplaza detector HSV cuando está disponible).
        # En hazmat_mode='worker' NO se carga aqui — lo carga una sola vez el
        # hazmat_worker compartido; esta instancia solo reenvia frames y recibe
        # resultados (ver _submit_frame_to_worker / _on_hazmat_result).
        self._hazmat_yolo = None
        hazmat_model_path = self.get_parameter('hazmat_model').value
        if self._hazmat_mode == 'local' and _YOLO_OK and self.get_parameter('enable_hazmat').value \
                and hazmat_model_path:
            if os.path.exists(hazmat_model_path):
                try:
                    self._hazmat_yolo = _YOLO(hazmat_model_path)
                    self.get_logger().info(f'YOLO (hazmat) cargado: {hazmat_model_path}')
                except Exception as exc:
                    self.get_logger().warn(f'No se pudo cargar hazmat YOLO: {exc}')
            else:
                self.get_logger().warn(
                    f'hazmat_model no encontrado: {hazmat_model_path} — usando detector HSV')

        # Publicadores
        self._det_pub = self.create_publisher(StringMsg, '/object_detections', 10)
        self._marker_pub = self.create_publisher(
            MarkerArray, '/object_detection_markers', 10)
        self._annotated_pub = self.create_publisher(
            CompressedImage, '/camera/color/image_annotated/compressed', 1)

        # ── Suscripciones — raw o compressed según el driver ─────────
        use_compressed   = self.get_parameter('use_compressed').value
        color_topic      = self.get_parameter('color_topic').value
        depth_topic      = self.get_parameter('depth_topic').value
        cam_info_topic   = self.get_parameter('camera_info_topic').value

        # logitech_pub / astra_rgbd_camera_node publican con QoS BEST_EFFORT
        # (sensor data) — un subscriber RELIABLE (el default de create_subscription)
        # es incompatible con eso y no recibe ningun mensaje.
        image_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(CameraInfo, cam_info_topic, self._on_cam_info, 5)

        if use_compressed:
            self.create_subscription(
                CompressedImage, color_topic, self._on_color_compressed, image_qos)
            self.create_subscription(
                CompressedImage, depth_topic, self._on_depth_compressed, image_qos)
            self.get_logger().info(
                f'Modo COMPRESSED — color: {color_topic}  depth: {depth_topic}')
        else:
            self.create_subscription(Image, color_topic, self._on_color, image_qos)
            self.create_subscription(Image, depth_topic, self._on_depth, image_qos)
            self.get_logger().info(
                f'Modo RAW — color: {color_topic}  depth: {depth_topic}')

        # Servicio de guardado
        self.create_service(Trigger, '/save_detection_csv', self._on_save_csv)

        # ── hazmat_mode='worker': reenviar frames al worker compartido y
        # escuchar sus resultados. Misma image_qos (BEST_EFFORT, depth=1) que
        # las cámaras — el worker solo necesita el frame más reciente.
        if self._hazmat_mode == 'worker' and self.get_parameter('enable_hazmat').value:
            self._hazmat_submit_pub = self.create_publisher(
                CompressedImage, f'/hazmat/submit/{self._camera_id}', image_qos)
            self.create_subscription(
                StringMsg, f'/hazmat/result/{self._camera_id}',
                self._on_hazmat_result, 5)
            self.get_logger().info(
                f'Hazmat en modo WORKER — camera_id={self._camera_id} '
                f'(submit: /hazmat/submit/{self._camera_id}, '
                f'result: /hazmat/result/{self._camera_id})')

        if self._hazmat_mode == 'worker':
            hazmat_status = 'WORKER'
        elif self._hazmat_yolo:
            hazmat_status = 'YOLO'
        elif self.get_parameter('enable_hazmat').value:
            hazmat_status = 'HSV'
        else:
            hazmat_status = '✗'
        self.get_logger().info(
            'ObjectDetector activo\n'
            '  AprilTag : ' + ('✓' if self._aruco_detector else '✗') + '\n'
            '  Hazmat   : ' + hazmat_status + '\n'
            '  YOLO obj : ' + ('✓' if self._yolo else '✗')
        )

    # ─── Start time ───────────────────────────────────────────────

    def _try_record_start(self) -> None:
        if self._start_time is not None:
            self._start_timer_obj.cancel()
            return
        try:
            self._tf_buf.lookup_transform('map', 'base_footprint', rclpy.time.Time())
            self._start_time = datetime.datetime.now()
            self._start_timer_obj.cancel()
        except Exception:
            pass

    # ─── Callbacks de sensores ────────────────────────────────────

    def _on_cam_info(self, msg: CameraInfo) -> None:
        self._cam_info = msg

    def _on_depth(self, msg: Image) -> None:
        try:
            if msg.encoding in ('16UC1', 'mono16'):
                arr = self._bridge.imgmsg_to_cv2(msg, '16UC1').astype(np.float32)
                arr *= 0.001   # mm → m
            else:
                arr = self._bridge.imgmsg_to_cv2(msg, '32FC1')
            self._depth_img = arr
        except Exception:
            pass

    def _on_depth_compressed(self, msg: CompressedImage) -> None:
        """Decodifica el depth PNG uint16 del astra_rgbd_camera_node del compañero."""
        try:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            arr = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)
            if arr is None:
                return
            scale = float(self.get_parameter('depth_scale').value)
            self._depth_img = arr.astype(np.float32) * scale  # uint16 mm → float32 m
        except Exception:
            pass

    def _on_color_compressed(self, msg: CompressedImage) -> None:
        """Decodifica el color JPEG del astra_rgbd_camera_node del compañero y lo procesa."""
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_detect_time < DETECT_INTERVAL:
            return
        self._last_detect_time = now

        try:
            buf = np.frombuffer(msg.data, dtype=np.uint8)
            bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
        except Exception:
            return
        if bgr is None:
            return

        self._process_frame(bgr)

    def _on_color(self, msg: Image) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_detect_time < DETECT_INTERVAL:
            return
        self._last_detect_time = now

        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        self._process_frame(bgr)

    def _process_frame(self, bgr: np.ndarray) -> None:
        """Corre los tres detectores sobre un frame BGR y despacha resultados.

        AprilTag y YOLO-objetos se calculan aqui mismo, de forma sincrona,
        sobre `bgr` — sin cambios. En hazmat_mode='worker', el hazmat NO se
        agrega a `detections` en este metodo: se reenvia el frame (ya con las
        detecciones locales encontradas hasta el momento, para poder fusionarlas
        despues) al worker compartido, y el resultado se dibuja/publica mas
        tarde, de forma asincrona, sobre ESE MISMO frame exacto cuando llegue
        (ver _submit_frame_to_worker / _on_hazmat_result). Asi nunca se pinta
        un resultado hazmat viejo sobre el frame actual.
        """
        self._color_img = bgr
        detections = []

        if self._aruco_detector and self.get_parameter('enable_apriltag').value:
            detections += self._detect_apriltags(bgr)

        if self._yolo and self.get_parameter('enable_yolo').value:
            detections += self._detect_yolo(bgr)

        if self.get_parameter('enable_hazmat').value:
            if self._hazmat_mode == 'worker':
                self._submit_frame_to_worker(bgr, detections)
            elif self._hazmat_yolo:
                detections += self._detect_hazmat_yolo(bgr)
            else:
                detections += self._detect_hazmat_hsv(bgr)

        annotated = self._draw_annotated(bgr, detections)
        self._publish_annotated_frame(annotated)
        self._update_hazmat_alerts(annotated, detections)

        for det in detections:
            self._process_detection(det)

    # ─── Detectores ───────────────────────────────────────────────

    def _detect_apriltags(self, bgr: np.ndarray) -> List[dict]:
        """Detecta AprilTags Standard41h12 y devuelve lista de dicts."""
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        results = []

        try:
            if self._aruco_detector == 'legacy':
                corners, ids, _ = cv2.aruco.detectMarkers(
                    gray, self._aruco_dict, parameters=self._aruco_params)
            else:
                corners, ids, _ = self._aruco_detector.detectMarkers(gray)

            if ids is None:
                return []

            for i, tag_id in enumerate(ids.flatten()):
                c = corners[i][0]
                cx = int(c[:, 0].mean())
                cy = int(c[:, 1].mean())
                results.append({
                    'type': 'ar_code',
                    'name': str(int(tag_id)),
                    'u': cx, 'v': cy,
                    'x1': int(c[:, 0].min()), 'y1': int(c[:, 1].min()),
                    'x2': int(c[:, 0].max()), 'y2': int(c[:, 1].max()),
                })
        except Exception as exc:
            self.get_logger().debug(f'AprilTag error: {exc}')

        return results

    def _detect_hazmat_yolo(self, bgr: np.ndarray) -> List[dict]:
        """Detecta señales hazmat con el modelo YOLO entrenado (49 clases)."""
        conf = float(self.get_parameter('hazmat_conf').value)
        try:
            return run_hazmat_yolo(self._hazmat_yolo, bgr, conf)
        except Exception as exc:
            self.get_logger().debug(f'Hazmat YOLO error: {exc}')
            return []

    # ─── hazmat_mode='worker': reenvio de frames + consumo de resultados ──
    #
    # Cada frame reenviado lleva un token de identidad — "camera_id#seq" en
    # header.frame_id, timestamp de captura en header.stamp — para que el
    # resultado que vuelva del worker se pueda emparejar EXACTAMENTE con el
    # frame que lo produjo. No se depende solo del topic (que ya aisla por
    # camara) como unica fuente de verdad: camera_id tambien viaja dentro del
    # mensaje y se valida al recibir el resultado.

    def _submit_frame_to_worker(self, bgr: np.ndarray, local_detections: list) -> None:
        """Reenvia el frame actual al hazmat_worker compartido y lo guarda en
        un buffer chico y acotado (FIFO) indexado por su numero de secuencia.
        `local_detections` (AprilTag/YOLO-objetos ya calculados para ESTE
        frame) se guarda junto al frame para poder fusionarlos con el
        resultado hazmat cuando llegue, sin recalcular nada.

        El worker decide, segun su propio scheduler de prioridad, si/cuando
        infiere realmente este frame — aqui solo se publica (QoS depth=1
        descarta el anterior si el worker todavia no lo habia consumido,
        nunca se acumulan frames viejos en el transporte)."""
        if self._hazmat_submit_pub is None:
            return

        self._frame_seq += 1
        seq = self._frame_seq
        self._frame_buffer[seq] = {'bgr': bgr, 'detections': list(local_detections)}
        while len(self._frame_buffer) > self._frame_buffer_maxlen:
            self._frame_buffer.popitem(last=False)  # descarta la entrada mas vieja

        try:
            _, buf = cv2.imencode('.jpg', bgr, [cv2.IMWRITE_JPEG_QUALITY, 80])
        except Exception:
            return
        msg = CompressedImage()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self._camera_id}#{seq}'
        msg.format = 'jpeg'
        msg.data = buf.tobytes()
        self._hazmat_submit_pub.publish(msg)

    def _on_hazmat_result(self, msg: StringMsg) -> None:
        """Empareja el resultado con el frame EXACTO que lo produjo (por
        camera_id + frame_seq) y dibuja/publica solo sobre ese frame — nunca
        sobre el frame "actual" de este ciclo, que puede ser otro distinto.
        Si el frame ya no esta en el buffer (demasiado viejo, ya expulsado),
        el resultado se descarta. Un resultado vacio (sin detecciones) igual
        se procesa: invalida cualquier racha de confirmacion pendiente para
        esta camara en vez de dejarla "colgada"."""
        try:
            payload = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            return

        result_camera_id = payload.get('camera_id')
        if result_camera_id != self._camera_id:
            # No depender solo del topic: si por algun motivo llega un
            # resultado con otro camera_id, se descarta explicitamente.
            self.get_logger().warn(
                f'Resultado hazmat con camera_id "{result_camera_id}" '
                f'!= esperado "{self._camera_id}" — descartado')
            return

        seq = payload.get('frame_seq')
        entry = self._frame_buffer.pop(seq, None)
        if entry is None:
            self.get_logger().debug(
                f'Resultado hazmat para frame_seq={seq} ya no disponible '
                f'en el buffer (expirado/expulsado) — descartado')
            return

        hazmat_detections = payload.get('detections') or []
        bgr = entry['bgr']
        merged = entry['detections'] + hazmat_detections

        annotated = self._draw_annotated(bgr, merged)
        self._publish_annotated_frame(annotated)
        self._update_hazmat_alerts(annotated, merged)

        for det in hazmat_detections:
            self._process_detection(det)

    def _detect_hazmat_hsv(self, bgr: np.ndarray) -> List[dict]:
        """Fallback: detecta señales hazmat (diamante naranja) por color HSV + forma."""
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)

        lo = np.array([HAZMAT_H_LO, HAZMAT_S_LO, HAZMAT_V_LO], dtype=np.uint8)
        hi = np.array([HAZMAT_H_HI, 255, 255], dtype=np.uint8)
        mask = cv2.inRange(hsv, lo, hi)

        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        results = []
        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area < HAZMAT_AREA_MIN:
                continue
            hull = cv2.convexHull(cnt)
            approx = cv2.approxPolyDP(hull, 0.1 * cv2.arcLength(hull, True), True)
            if len(approx) not in (3, 4, 5):
                continue

            M = cv2.moments(cnt)
            if M['m00'] == 0:
                continue
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
            bx, by, bw, bh = cv2.boundingRect(cnt)
            results.append({
                'type': 'hazmat_sign',
                'name': 'HZ',
                'u': cx, 'v': cy,
                'x1': bx, 'y1': by, 'x2': bx + bw, 'y2': by + bh,
            })

        return results

    def _detect_yolo(self, bgr: np.ndarray) -> List[dict]:
        """Detecta objetos físicos con YOLO (ultralytics)."""
        results_out = []
        try:
            results = self._yolo(bgr, conf=YOLO_CONF, verbose=False)
            for r in results:
                for box in r.boxes:
                    cls_name = self._yolo.names[int(box.cls[0])].lower()
                    if cls_name not in YOLO_TARGET_CLASSES:
                        continue
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    results_out.append({
                        'type': 'real_object',
                        'name': YOLO_TARGET_CLASSES[cls_name],
                        'u': cx, 'v': cy,
                        'x1': int(x1), 'y1': int(y1), 'x2': int(x2), 'y2': int(y2),
                    })
        except Exception as exc:
            self.get_logger().debug(f'YOLO error: {exc}')
        return results_out

    # ─── Imagen anotada ──────────────────────────────────────────

    _DET_COLORS = {
        'ar_code':     (0, 220, 255),
        'hazmat_sign': (0, 120, 255),
        'real_object': (60, 60, 255),
    }

    def _draw_annotated(self, bgr: np.ndarray, detections: list) -> np.ndarray:
        annotated = bgr.copy()
        for det in detections:
            color = self._DET_COLORS.get(det['type'], (200, 200, 200))
            x1 = det.get('x1', det['u'] - 20)
            y1 = det.get('y1', det['v'] - 20)
            x2 = det.get('x2', det['u'] + 20)
            y2 = det.get('y2', det['v'] + 20)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            label = f"{det['type']} {det['name']}"
            cv2.putText(annotated, label, (x1, max(y1 - 5, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)
        return annotated

    def _publish_annotated_frame(self, annotated: np.ndarray) -> None:
        if self._annotated_pub.get_subscription_count() == 0:
            return
        _, buf = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 75])
        out = CompressedImage()
        out.header.stamp = self.get_clock().now().to_msg()
        out.format = 'jpeg'
        out.data = buf.tobytes()
        self._annotated_pub.publish(out)

    # ─── Alertas hazmat: confirmacion temporal + captura automatica ──

    def _update_hazmat_alerts(self, annotated: np.ndarray, detections: list) -> None:
        """
        Una deteccion hazmat parpadea entre frames por ruido del modelo — para no
        quedarnos con nada aprovechable, solo la contamos como "confirmada" (y
        disparamos una captura para revision humana) cuando la misma clase
        aparece en aprox. la misma zona durante `alert_confirm_frames` ciclos
        seguidos. Sin eso, `require_depth=False` colapsa todas las señales a la
        misma pose (0,0,0) y el dedup por distancia en `_process_detection`
        oculta casi todo tras la primera.
        """
        if self._alerts_dir is None:
            return

        seen_now = set()
        now = time.time()

        for det in detections:
            if det['type'] != 'hazmat_sign':
                continue
            name = det['name']
            seen_now.add(name)
            cx, cy = det['u'], det['v']

            track = self._hazmat_track.get(name)
            if track is not None:
                dist = math.hypot(cx - track['cx'], cy - track['cy'])
                if dist <= self._alert_pixel_tol:
                    track['count'] += 1
                else:
                    track['count'] = 1
            else:
                track = {'count': 1}
                self._hazmat_track[name] = track
            track['cx'], track['cy'] = cx, cy
            track['last_seen'] = now
            track['det'] = det

            if track['count'] < self._alert_confirm_frames:
                continue
            last_saved = self._alert_last_saved.get(name, 0.0)
            if now - last_saved < self._alert_cooldown_sec:
                continue

            self._alert_last_saved[name] = now
            self._save_alert_capture(annotated, det)

        # Clases que no aparecieron este ciclo: reiniciar su racha de confirmacion.
        for name in list(self._hazmat_track.keys()):
            if name not in seen_now:
                del self._hazmat_track[name]

    def _save_alert_capture(self, annotated: np.ndarray, det: dict) -> None:
        ts = datetime.datetime.now()
        stamp = ts.strftime('%Y%m%d-%H%M%S-%f')[:-3]
        safe_name = det['name'].replace('/', '_')
        base = f'{stamp}_{safe_name}'

        img_path = os.path.join(self._alerts_dir, base + '.jpg')
        json_path = os.path.join(self._alerts_dir, base + '.json')
        try:
            cv2.imwrite(img_path, annotated)
            with open(json_path, 'w') as f:
                json.dump({
                    'timestamp':  ts.isoformat(timespec='milliseconds'),
                    'camera_id':  self._camera_id or 'default',
                    'type':       det['type'],
                    'name':       det['name'],
                    'confidence': round(det.get('conf', 0.0), 4),
                    'bbox_px':    [det.get('x1'), det.get('y1'), det.get('x2'), det.get('y2')],
                    'image_file': os.path.basename(img_path),
                }, f, indent=2)
            self.get_logger().info(
                f'[ALERTA HAZMAT] "{det["name"]}" confirmada '
                f'(conf={det.get("conf", 0.0):.2f}) -> {img_path}')
        except OSError as exc:
            self.get_logger().warn(f'No se pudo guardar la alerta hazmat: {exc}')

    # ─── Localización 3D y registro ──────────────────────────────

    def _pixel_to_3d_map(self, u: int, v: int) -> Optional[Tuple[float, float, float]]:
        """
        Convierte un pixel (u, v) a coordenadas 3D en el frame 'map'.
        Usa la imagen de profundidad + intrínsecos de la cámara + TF.
        """
        if self._depth_img is None:
            return None

        h, w = self._depth_img.shape[:2]
        # Buscar en ventana 5×5 alrededor del píxel el depth mediano válido
        u0, u1 = max(0, u - 2), min(w, u + 3)
        v0, v1 = max(0, v - 2), min(h, v + 3)
        patch = self._depth_img[v0:v1, u0:u1].flatten()
        valid = patch[(patch > DEPTH_MIN) & (patch < DEPTH_MAX)]
        if len(valid) == 0:
            return None
        depth = float(np.median(valid))

        # Proyección inversa — usa CameraInfo si está disponible, si no los parámetros
        if self._cam_info is not None:
            fx = self._cam_info.k[0]
            fy = self._cam_info.k[4]
            cx = self._cam_info.k[2]
            cy = self._cam_info.k[5]
        else:
            fx = float(self.get_parameter('fx').value)
            fy = float(self.get_parameter('fy').value)
            cx = float(self.get_parameter('cx').value)
            cy = float(self.get_parameter('cy').value)
        if fx == 0 or fy == 0:
            return None

        # Punto en frame óptico de la cámara (X derecha, Y abajo, Z adelante)
        x_cam = (u - cx) * depth / fx
        y_cam = (v - cy) * depth / fy
        z_cam = depth

        # Transformar al frame 'map'
        frame_id = (
            self._cam_info.header.frame_id
            if self._cam_info is not None and self._cam_info.header.frame_id
            else self.get_parameter('camera_frame_id').value
        )
        try:
            t = self._tf_buf.lookup_transform('map', frame_id, rclpy.time.Time())
        except Exception:
            return None

        p = np.array([[x_cam, y_cam, z_cam]], dtype=np.float32)
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
        p_map = (R @ p.T).T[0] + T
        return float(p_map[0]), float(p_map[1]), float(p_map[2])

    def _process_detection(self, det: dict) -> None:
        """Valida, deduplica y registra una detección."""
        pos = self._pixel_to_3d_map(det['u'], det['v'])
        if pos is None:
            if self.get_parameter('require_depth').value:
                return
            x, y, z = 0.0, 0.0, 0.0
        else:
            x, y, z = pos

        # Deduplicación: misma clase a < DEDUP_DIST m
        for existing in self._detections:
            if (existing['type'] == det['type'] and
                    math.sqrt((x - existing['x'])**2 +
                              (y - existing['y'])**2) < DEDUP_DIST):
                return

        self._det_counter += 1
        now = datetime.datetime.now()
        time_str = now.strftime('%H:%M:%S')

        raw_mode = self.get_parameter('mode').value.lower()
        mode_char = 'A' if raw_mode in ('a', 'autonomous') else 'T'

        record = {
            'detection': self._det_counter,
            'time':      time_str,
            'type':      det['type'],
            'name':      det['name'],
            'x': round(x, 4),
            'y': round(y, 4),
            'z': round(z, 4),
            'robot':     self.get_parameter('robot_name').value,
            'mode':      mode_char,
        }
        self._detections.append(record)

        # Publicar para geotiff_writer + dashboard (formato JSON con wx/wy para
        # el mapa 2D). camera_id explicito: el dashboard consume detecciones de
        # front y astra por este MISMO topic y necesita saber cual la genero
        # para no asumir siempre "front" al elegir el snapshot del popup.
        geo_msg = StringMsg()
        geo_msg.data = json.dumps({
            'type': det['type'],
            'name': det['name'],
            'wx': x, 'wy': y,
            'camera_id': self._camera_id or 'front',
        })
        self._det_pub.publish(geo_msg)

        # Publicar marcador 3D para RViz
        self._publish_marker(record)

        self.get_logger().info(
            f'[DETECCIÓN #{self._det_counter}] {det["type"]} "{det["name"]}" '
            f'@ ({x:.2f}, {y:.2f}, {z:.2f}) m')

    # ─── Marcadores RViz ──────────────────────────────────────────

    def _publish_marker(self, det: dict) -> None:
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = det['type']
        marker.id = det['detection']
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD
        marker.pose.position.x = det['x']
        marker.pose.position.y = det['y']
        marker.pose.position.z = det['z'] + 0.15
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.3
        marker.lifetime.sec = 0   # permanente

        dtype = det['type']
        if dtype == 'ar_code':
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.78, 0.0
        elif dtype == 'hazmat_sign':
            marker.color.r, marker.color.g, marker.color.b = 1.0, 0.39, 0.12
        else:
            marker.color.r, marker.color.g, marker.color.b = 0.94, 0.04, 0.04
        marker.color.a = 0.9

        # Texto encima
        label = Marker()
        label.header = marker.header
        label.ns = det['type'] + '_label'
        label.id = det['detection']
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position.x = det['x']
        label.pose.position.y = det['y']
        label.pose.position.z = det['z'] + 0.35
        label.pose.orientation.w = 1.0
        label.scale.z = 0.12
        label.color.r = label.color.g = label.color.b = label.color.a = 1.0
        label.text = f'{det["type"]}\n{det["name"]}'
        label.lifetime.sec = 0

        arr = MarkerArray()
        arr.markers = [marker, label]
        self._marker_pub.publish(arr)

    # ─── Exportación CSV ──────────────────────────────────────────

    def _on_save_csv(self, _req, resp: Trigger.Response) -> Trigger.Response:
        try:
            path = self._export_csv()
            n = len(self._detections)
            resp.success = True
            resp.message = f'CSV guardado → {path}  ({n} detecciones)'
            self.get_logger().info(resp.message)
        except Exception as exc:
            resp.success = False
            resp.message = f'Error al exportar CSV: {exc}'
            self.get_logger().error(resp.message)
        return resp

    def _export_csv(self) -> str:
        ref = self._start_time or datetime.datetime.now()
        ts         = ref.strftime('%H-%M-%S')
        start_date = ref.strftime('%Y-%m-%d')
        start_hms  = ref.strftime('%H:%M:%S')

        team    = self.get_parameter('team_name').value
        country = self.get_parameter('country').value
        miss    = self.get_parameter('mission').value
        fname   = f'RoboCup2026-{team}-{miss}-{ts}-pois.csv'
        out_dir = self.get_parameter('output_dir').value
        os.makedirs(out_dir, exist_ok=True)
        filepath = os.path.join(out_dir, fname)

        with open(filepath, 'w', newline='') as f:
            # Preamble obligatorio RoboCup 2026 (spec pág. 20)
            f.write(f'"pois"\n')
            f.write(f'"1.3"\n')
            f.write(f'"{team}"\n')
            f.write(f'"{country}"\n')
            f.write(f'"{start_date}"\n')
            f.write(f'"{start_hms}"\n')
            f.write(f'"{miss}"\n')
            f.write('\n')
            writer = csv.writer(f, quoting=csv.QUOTE_NONNUMERIC)
            writer.writerow(['detection', 'time', 'type', 'name',
                             'x', 'y', 'z', 'robot', 'mode'])
            for det in self._detections:
                writer.writerow([
                    det['detection'], det['time'],
                    det['type'],      det['name'],
                    det['x'],         det['y'],    det['z'],
                    det['robot'],     det['mode'],
                ])

        return filepath


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
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
