#!/usr/bin/env python3
"""
object_detector.py
Detecta objetos de interés RoboCup Rescue 2026 y genera el CSV de detecciones.

Detectores implementados:
  1. AprilTag tagStandard41h12 → tipo 'ar_code'        (1 pt)
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

CSV formato RoboCup 2026:
  detection,time,type,name,x,y,z,robot,mode
  (x, y, z en metros en el frame 'map')
"""

import csv
import datetime
import json
import math
import os
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

try:
    # OpenCV (cv2.aruco) NO soporta la familia tagStandard41h12 — solo
    # 16h5/25h9/36h10/36h11 y variantes ArUco. pupil-apriltags usa la
    # librería oficial AprilTag en C, que sí la reconoce.
    from pupil_apriltags import Detector as _AprilTagDetector
    _APRILTAG_OK = True
except ImportError:
    _APRILTAG_OK = False


# ── Objetos de misión RoboCup Rescue 2026 ──────────────────────────────────
# No existen en el vocabulario COCO (80 clases), así que se entrenó un
# yolov8s custom (mission_objects_yolo.pt, ver training/train_mission.py o
# el kernel de Kaggle) combinando datasets públicos de Roboflow. Sus 6
# clases ya vienen exactamente con estos nombres — no hace falta mapeo.
#
# OJO — confiabilidad real por clase según el entrenamiento (ver
# runs/mission_objects/confusion_matrix.png del run de Kaggle):
#   Gloves, FireExtinguisher     → miles de ejemplos, funcionan bien.
#   HardHat                     → 10k+ ejemplos de train pero el split de
#                                  validación quedó vacío — sin confirmar.
#   PowerCable, FuelCan          → decenas de ejemplos, poco confiables.
#   Rope                        → CERO datos (el dataset fuente solo tenía
#                                  una clase "hang" que no matcheó "rope",
#                                  y el dataset de respaldo falló al bajar).
#                                  Este modelo NUNCA va a detectar cuerdas.
MISSION_OBJECT_CLASSES = {'Rope', 'Gloves', 'HardHat', 'PowerCable', 'FuelCan', 'FireExtinguisher'}

# Umbral de confianza para el modelo de objetos de misión (custom, entrenado
# con datos reales — no zero-shot, así que no necesita el umbral tan bajo
# que hacía falta con YOLO-World).
YOLO_CONF = 0.35

# Rango válido del sensor de profundidad (m)
DEPTH_MIN = 0.3
DEPTH_MAX = 4.0

# Deduplicación: si una detección del mismo tipo/nombre está a < DEDUP_DIST m
# Y fue vista hace menos de DEDUP_COOLDOWN s → se considera la misma.
DEDUP_DIST = 0.5      # m
DEDUP_COOLDOWN = 15.0  # s

# Intervalo de detección (segundos) — no procesar cada frame
DETECT_INTERVAL = 0.5   # s

# Colores HSV para hazmat (naranja — valor ajustable según condiciones de luz)
HAZMAT_H_LO, HAZMAT_H_HI = 8, 22    # matiz (0-180 en OpenCV)
HAZMAT_S_LO = 120                    # saturación mínima
HAZMAT_V_LO = 100                    # brillo mínimo
HAZMAT_AREA_MIN = 500                # área mínima en píxeles²

# ── Consolidación de clases hazmat ─────────────────────────────────────────
# hazmat_yolo.pt ahora es el modelo curado de 13 clases (hazmat13, entrenado
# directo sobre las señales RoboCup Rescue 2026 — nombres ya casi exactos,
# ej. 'Flammable Gas', 'Dangerous', 'Non-flammable Gas'). Se mantienen
# también las variantes del modelo público viejo de 49 clases (plurales,
# sinónimos) por si se vuelve a usar hazmat_yolo_49class_backup.pt.
# Claves en minúscula, sin guiones (normalizados antes del lookup).
HAZMAT_CLASS_MAP: Dict[str, str] = {
    'poison':                                                                     'Poison',
    'poisons':                                                                    'Poison',
    'toxins':                                                                     'Poison',
    # Mismo pictograma (llama sobre círculo) que "Oxidizer"
    'oxygen':                                                                     'Oxygen',
    '011_oxidizer':                                                               'Oxygen',
    'oxidizer':                                                                   'Oxygen',
    'oxidizing substances':                                                       'Oxygen',
    'oxidising agents':                                                           'Oxygen',
    'flammable gas':                                                              'FlammableGas',
    'flammable gases':                                                            'FlammableGas',
    'flammable solid':                                                            'FlammableSolid',
    'flammable solids':                                                           'FlammableSolid',
    'corrosive':                                                                  'Corrosive',
    'dangerous':                                                                  'Dangerous',
    'dangerous when wet':                                                         'Dangerous',
    'non flammable gas':                                                          'NonFlammableGas',
    'nonflammable gas':                                                           'NonFlammableGas',
    'nonflammable gases':                                                         'NonFlammableGas',
    'organic peroxide':                                                           'OrganicPeroxide',
    'organic peroxides':                                                          'OrganicPeroxide',
    'organic peroxids':                                                           'OrganicPeroxide',
    'explosive':                                                                  'Explosive',
    'explosives':                                                                 'Explosive',
    'explosive substances':                                                       'Explosive',
    'explosives products considered extremely insensitive with no risk to create a mass explosion': 'Explosive',
    'explosives products considered very insensitive that are used as blasting agents':              'Explosive',
    'explosives products with no significant risk of creating a blast':           'Explosive',
    'explosives products with the potential to create a fire or minor blast':     'Explosive',
    'explosives products with the potential to create a mass explosion':          'Explosive',
    'explosives products with the potential to create a projectile hazard':       'Explosive',
    'radioactive':                                                                'Radioactive',
    'inhalation hazard':                                                          'InhalationHazard',
    'spontaneously combustible':                                                  'SpontaneouslyCombustible',
    'spontaneously combustible material':                                         'SpontaneouslyCombustible',
    'infectious substance':                                                       'InfectiousSubstance',
    'infectous substance':                                                        'InfectiousSubstance',  # typo del dataset
}


# pupil-apriltags expone decision_margin (a mayor valor, patrón más nítido/
# confiable) — filtra ruido de fondo sin necesidad de ajustar a ciegas
# parámetros de binarización como con cv2.aruco.
APRILTAG_MIN_DECISION_MARGIN = 30.0

# hazmat_yolo.pt es un modelo chico entrenado con dataset limitado y confunde
# fácil: una cara con "Explosive", el azul sólido de un guante con "Dangerous
# when wet" (diamante azul), etc. Un rótulo hazmat nunca puede ser una
# persona ni otro objeto de misión ya identificado, así que se descarta si
# se superpone mucho con cualquiera de los dos.
HAZMAT_OVERLAP_MAX = 0.3


def _box_overlap_ratio(a: dict, b: dict) -> float:
    """Fracción del área de 'a' que cae dentro de 'b' (no IoU simétrico:
    nos importa si el cuadro chico del hazmat cae DENTRO del otro)."""
    ax1, ay1, ax2, ay2 = a['x1'], a['y1'], a['x2'], a['y2']
    bx1, by1, bx2, by2 = b['x1'], b['y1'], b['x2'], b['y2']
    ix1, iy1 = max(ax1, bx1), max(ay1, by1)
    ix2, iy2 = min(ax2, bx2), min(ay2, by2)
    inter = max(0, ix2 - ix1) * max(0, iy2 - iy1)
    area_a = max(1, (ax2 - ax1) * (ay2 - ay1))
    return inter / area_a


def _filter_hazmat_near_people(detections: List[dict], person_boxes: List[dict]) -> List[dict]:
    other_boxes = person_boxes + [d for d in detections if d['type'] == 'real_object']
    if not other_boxes:
        return detections
    return [
        d for d in detections
        if not (d['type'] == 'hazmat_sign' and
                any(_box_overlap_ratio(d, o) > HAZMAT_OVERLAP_MAX for o in other_boxes))
    ]


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
        self.declare_parameter('yolo_model',
                                '/workspace/src/rescue_bringup/models/mission_objects_yolo.pt')
        self.declare_parameter('hazmat_model',   '')
        self.declare_parameter('hazmat_conf',    0.65)
        self.declare_parameter('enable_yolo',    True)
        self.declare_parameter('enable_apriltag', True)
        self.declare_parameter('enable_hazmat',  True)

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
        self._last_logged_at: Dict[Tuple[str, str], float] = {}

        # Hora de inicio de misión
        self._start_time: Optional[datetime.datetime] = None
        self._start_timer_obj = self.create_timer(1.0, self._try_record_start)

        # TF
        self._tf_buf = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buf, self)

        # AprilTag detector — familia real: tagStandard41h12.
        # OJO: OpenCV (cv2.aruco) NO soporta esta familia bajo ningún nombre
        # (solo 16h5/25h9/36h10/36h11 + variantes ArUco); por eso se usa
        # pupil-apriltags, que envuelve la librería oficial AprilTag en C.
        self._apriltag_detector = None
        if _APRILTAG_OK and self.get_parameter('enable_apriltag').value:
            self._apriltag_detector = _AprilTagDetector(
                families='tagStandard41h12', nthreads=2)
            self.get_logger().info('AprilTag detector (tagStandard41h12) activo')
        elif not _APRILTAG_OK and self.get_parameter('enable_apriltag').value:
            self.get_logger().warn(
                'pupil_apriltags no instalado — detección AprilTag desactivada. '
                'Instala con: pip3 install pupil-apriltags')

        # YOLO model — objetos de misión (rope, gloves, hard hat, etc.),
        # modelo custom entrenado (ver MISSION_OBJECT_CLASSES arriba).
        self._yolo = None
        # Modelo liviano SOLO para detectar personas — no es una detección de
        # misión, se usa exclusivamente para _filter_hazmat_near_people (el
        # modelo custom de 6 clases no tiene clase 'person' en absoluto).
        self._person_yolo = None
        if _YOLO_OK and self.get_parameter('enable_yolo').value:
            model_path = self.get_parameter('yolo_model').value
            try:
                self._yolo = _YOLO(model_path)
                self.get_logger().info(f'YOLO (objetos de misión) cargado: {model_path}')
            except Exception as exc:
                self.get_logger().warn(f'No se pudo cargar YOLO ({model_path}): {exc}')
            try:
                self._person_yolo = _YOLO('yolov8n.pt')
            except Exception as exc:
                self.get_logger().warn(f'No se pudo cargar YOLO (personas): {exc}')
        elif not _YOLO_OK and self.get_parameter('enable_yolo').value:
            self.get_logger().warn(
                'ultralytics no instalado — detección YOLO desactivada. '
                'Instala con: pip3 install ultralytics')

        # Modelo hazmat entrenado (reemplaza detector HSV cuando está disponible)
        self._hazmat_yolo = None
        hazmat_model_path = self.get_parameter('hazmat_model').value
        if _YOLO_OK and self.get_parameter('enable_hazmat').value and hazmat_model_path:
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

        # Los drivers de cámara (logitech_pub, astra_rgbd_camera_node) publican
        # con QoS BEST_EFFORT; la QoS por defecto (RELIABLE) es incompatible y
        # descarta silenciosamente todos los mensajes.
        sensor_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self.create_subscription(CameraInfo, cam_info_topic, self._on_cam_info, 5)

        if use_compressed:
            self.create_subscription(
                CompressedImage, color_topic, self._on_color_compressed, sensor_qos)
            self.create_subscription(
                CompressedImage, depth_topic, self._on_depth_compressed, sensor_qos)
            self.get_logger().info(
                f'Modo COMPRESSED — color: {color_topic}  depth: {depth_topic}')
        else:
            self.create_subscription(Image, color_topic, self._on_color, 5)
            self.create_subscription(Image, depth_topic, self._on_depth, 5)
            self.get_logger().info(
                f'Modo RAW — color: {color_topic}  depth: {depth_topic}')

        # Servicio de guardado
        self.create_service(Trigger, '/save_detection_csv', self._on_save_csv)

        hazmat_mode = 'YOLO' if self._hazmat_yolo else ('HSV' if self.get_parameter('enable_hazmat').value else '✗')
        self.get_logger().info(
            'ObjectDetector activo\n'
            '  AprilTag : ' + ('✓' if self._apriltag_detector else '✗') + '\n'
            '  Hazmat   : ' + hazmat_mode + '\n'
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

        self._color_img = bgr
        detections = self._run_detectors(bgr)

        self._publish_annotated_frame(bgr, detections)

        for det in detections:
            self._process_detection(det)

    def _on_color(self, msg: Image) -> None:
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_detect_time < DETECT_INTERVAL:
            return
        self._last_detect_time = now

        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        self._color_img = bgr
        detections = self._run_detectors(bgr)

        self._publish_annotated_frame(bgr, detections)

        for det in detections:
            self._process_detection(det)

    def _run_detectors(self, bgr: np.ndarray) -> List[dict]:
        detections = []

        if self._apriltag_detector and self.get_parameter('enable_apriltag').value:
            detections += self._detect_apriltags(bgr)

        if self.get_parameter('enable_hazmat').value:
            if self._hazmat_yolo:
                detections += self._detect_hazmat_yolo(bgr)
            else:
                detections += self._detect_hazmat_hsv(bgr)

        person_boxes = []
        if self.get_parameter('enable_yolo').value:
            if self._yolo:
                detections += self._detect_yolo(bgr)
            if self._person_yolo:
                person_boxes = self._detect_people(bgr)

        return _filter_hazmat_near_people(detections, person_boxes)

    # ─── Detectores ───────────────────────────────────────────────

    def _detect_apriltags(self, bgr: np.ndarray) -> List[dict]:
        """Detecta AprilTags tagStandard41h12 y devuelve lista de dicts."""
        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        results = []

        try:
            for det in self._apriltag_detector.detect(gray):
                # hamming>0 = se corrigieron bits de error (lectura dudosa);
                # decision_margin bajo = patrón débil, probable ruido de fondo.
                if det.hamming > 0 or det.decision_margin < APRILTAG_MIN_DECISION_MARGIN:
                    continue
                c = det.corners
                cx, cy = int(det.center[0]), int(det.center[1])
                results.append({
                    'type': 'ar_code',
                    'name': str(int(det.tag_id)),
                    'u': cx, 'v': cy,
                    'x1': int(c[:, 0].min()), 'y1': int(c[:, 1].min()),
                    'x2': int(c[:, 0].max()), 'y2': int(c[:, 1].max()),
                })
        except Exception as exc:
            self.get_logger().debug(f'AprilTag error: {exc}')

        return results

    def _detect_hazmat_yolo(self, bgr: np.ndarray) -> List[dict]:
        """Detecta señales hazmat con el modelo YOLO entrenado (49 clases crudas,
        consolidadas a las 13 señales RoboCup Rescue 2026 vía HAZMAT_CLASS_MAP)."""
        results_out = []
        conf = float(self.get_parameter('hazmat_conf').value)
        try:
            res = self._hazmat_yolo(bgr, conf=conf, verbose=False)
            for r in res:
                for box in r.boxes:
                    raw_name = self._hazmat_yolo.names[int(box.cls[0])]
                    cls_name = HAZMAT_CLASS_MAP.get(raw_name.lower().replace('-', ' '))
                    if cls_name is None:
                        continue  # no es una de las 13 señales de misión
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    results_out.append({
                        'type': 'hazmat_sign',
                        'name': cls_name,
                        'u': cx, 'v': cy,
                        'x1': int(x1), 'y1': int(y1), 'x2': int(x2), 'y2': int(y2),
                    })
        except Exception as exc:
            self.get_logger().debug(f'Hazmat YOLO error: {exc}')
        return results_out

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
        """Detecta objetos de misión con el modelo custom (6 clases, ver
        MISSION_OBJECT_CLASSES). Sus nombres de clase ya son los nombres
        display finales, no hace falta remapeo."""
        results_out = []
        try:
            results = self._yolo(bgr, conf=YOLO_CONF, verbose=False)
            for r in results:
                for box in r.boxes:
                    cls_name = self._yolo.names[int(box.cls[0])]
                    if cls_name not in MISSION_OBJECT_CLASSES:
                        continue
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    results_out.append({
                        'type': 'real_object',
                        'name': cls_name,
                        'u': cx, 'v': cy,
                        'x1': int(x1), 'y1': int(y1), 'x2': int(x2), 'y2': int(y2),
                    })
        except Exception as exc:
            self.get_logger().debug(f'YOLO error: {exc}')
        return results_out

    def _detect_people(self, bgr: np.ndarray) -> List[dict]:
        """Cajas de persona vía yolov8n COCO — solo para
        _filter_hazmat_near_people, no se reportan como detección."""
        boxes = []
        try:
            results = self._person_yolo(bgr, conf=YOLO_CONF, verbose=False)
            for r in results:
                for box in r.boxes:
                    if self._person_yolo.names[int(box.cls[0])] != 'person':
                        continue
                    x1, y1, x2, y2 = box.xyxy[0].tolist()
                    boxes.append({'x1': int(x1), 'y1': int(y1), 'x2': int(x2), 'y2': int(y2)})
        except Exception as exc:
            self.get_logger().debug(f'YOLO (personas) error: {exc}')
        return boxes

    # ─── Imagen anotada ──────────────────────────────────────────

    def _publish_annotated_frame(self, bgr: np.ndarray, detections: list) -> None:
        if self._annotated_pub.get_subscription_count() == 0:
            return
        _COLORS = {
            'ar_code':     (0, 220, 255),
            'hazmat_sign': (0, 120, 255),
            'real_object': (60, 60, 255),
        }
        annotated = bgr.copy()
        for det in detections:
            color = _COLORS.get(det['type'], (200, 200, 200))
            x1 = det.get('x1', det['u'] - 20)
            y1 = det.get('y1', det['v'] - 20)
            x2 = det.get('x2', det['u'] + 20)
            y2 = det.get('y2', det['v'] + 20)
            cv2.rectangle(annotated, (x1, y1), (x2, y2), color, 2)
            label = f"{det['type']} {det['name']}"
            cv2.putText(annotated, label, (x1, max(y1 - 5, 12)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.48, color, 1, cv2.LINE_AA)
        _, buf = cv2.imencode('.jpg', annotated, [cv2.IMWRITE_JPEG_QUALITY, 75])
        out = CompressedImage()
        out.header.stamp = self.get_clock().now().to_msg()
        out.format = 'jpeg'
        out.data = buf.tobytes()
        self._annotated_pub.publish(out)

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

        # Deduplicación: misma clase Y mismo nombre a < DEDUP_DIST m, Y visto
        # hace menos de DEDUP_COOLDOWN s. Sin el cooldown, un objeto que sigue
        # en cuadro (o sin profundidad real, donde todo cae en (0,0,0)) se
        # marcaría UNA vez y nunca más — se ve el cuadro en vivo pero no
        # vuelve a aparecer en la lista/CSV (bug ya visto con QR y hazmat).
        now_s = self.get_clock().now().nanoseconds * 1e-9
        key = (det['type'], det['name'])
        last_seen = self._last_logged_at.get(key, -1e9)
        for existing in self._detections:
            if (existing['type'] == det['type'] and
                    existing['name'] == det['name'] and
                    math.sqrt((x - existing['x'])**2 +
                              (y - existing['y'])**2) < DEDUP_DIST and
                    now_s - last_seen < DEDUP_COOLDOWN):
                return
        self._last_logged_at[key] = now_s

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

        # Publicar para geotiff_writer (formato JSON con wx/wy para el mapa 2D)
        geo_msg = StringMsg()
        geo_msg.data = json.dumps({
            'type': det['type'],
            'name': det['name'],
            'wx': x, 'wy': y,
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
