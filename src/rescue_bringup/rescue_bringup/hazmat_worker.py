#!/usr/bin/env python3
"""
hazmat_worker.py
Worker HAZMAT compartido por N cámaras — carga el modelo YOLO hazmat UNA sola
vez (a diferencia de object_detector en modo 'local', que carga su propia
copia por instancia) y sirve inferencia a cada cámara vía topics:

  /hazmat/submit/<camera_id>  (CompressedImage, publicado por cada
                                object_detector con hazmat_mode='worker').
                                header.frame_id lleva el token de identidad
                                "camera_id#frame_seq" y header.stamp el
                                timestamp de captura/origen del frame.
       -> hazmat_worker
  /hazmat/result/<camera_id>  (std_msgs/String, JSON: {camera_id, frame_seq,
                                stamp_sec, stamp_nanosec, detections}) — SIEMPRE
                                se publica, incluso con detections=[] vacio,
                                para que el detector pueda invalidar cualquier
                                confirmacion pendiente cuando el worker no
                                encuentra nada.

No toca ninguna captura de cámara ni el streaming: cada cámara sigue
publicando con su propio OpenCV (logitech_pub) exactamente igual que antes;
este nodo solo consume los frames que cada object_detector ya decodifica y
le reenvía. Tampoco reemplaza el dibujo/alertas/CSV/RViz — todo eso lo sigue
haciendo cada object_detector con los resultados que recibe de aquí, empatando
cada resultado con el frame EXACTO que lo produjo por (camera_id, frame_seq)
(ver _submit_frame_to_worker / _on_hazmat_result en object_detector.py). El
worker no depende solo del nombre del topic para saber de que cámara es un
frame: valida que el camera_id embebido en el token coincida con el topic.

Scheduler de prioridad (todo configurable por parámetro):
  - Estado normal: round-robin — se infiere una cámara distinta cada tick.
  - Si una cámara acumula `priority_confirm_frames` inferencias seguidas con
    al menos una detección por encima de `hazmat_conf`, esa cámara pasa a
    "boosted": se infiere en (casi) todos los ticks siguientes.
  - La cámara sin boost no se detiene nunca — sigue recibiendo un tick de
    inferencia cada `demoted_period_ticks` (piso de vigilancia).
  - Si la cámara boosted deja de detectar durante `priority_release_sec`,
    la prioridad vuelve a normal (round-robin) para todas.

Buffer: cada cámara solo guarda su ÚLTIMO frame recibido (se sobreescribe),
y la suscripción usa QoS BEST_EFFORT/KEEP_LAST(depth=1) — un frame viejo que
el worker no llegó a inferir nunca se acumula, siempre se descarta por el más
reciente.
"""

import json
import time
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String as StringMsg

try:
    import cv2
    _CV2_OK = True
except ImportError:
    _CV2_OK = False

try:
    from ultralytics import YOLO as _YOLO
    _YOLO_OK = True
except ImportError:
    _YOLO_OK = False

from rescue_bringup.hazmat_common import run_hazmat_yolo


class _CameraSlot:
    """Estado de una cámara dentro del scheduler. Sin cola: solo el frame
    pendiente más reciente (se sobreescribe, nunca se acumulan viejos).
    `pending_frame` es un dict {'bgr', 'seq', 'stamp'} — el token de
    identidad viaja junto con los pixeles, no se separa en ningún momento."""

    def __init__(self, camera_id: str):
        self.camera_id = camera_id
        self.pending_frame: Optional[dict] = None
        self.consecutive_hits = 0
        self.last_hit_time = 0.0


class HazmatWorker(Node):
    def __init__(self):
        super().__init__('hazmat_worker')

        self.declare_parameter('camera_ids', ['front', 'astra'])
        self.declare_parameter('hazmat_model', '')
        self.declare_parameter('hazmat_conf', 0.40)
        # Resolucion de inferencia: 416 = lo que usa el script standalone
        # (mas rapido, mas detecciones/seg); 640 = default de ultralytics.
        self.declare_parameter('hazmat_imgsz', 640)
        self.declare_parameter('tick_period_sec', 0.15)
        self.declare_parameter('demoted_period_ticks', 4)
        self.declare_parameter('priority_confirm_frames', 2)
        self.declare_parameter('priority_release_sec', 8.0)

        if not _CV2_OK:
            self.get_logger().fatal('OpenCV (cv2) no encontrado — instala python3-opencv')
            raise RuntimeError('cv2 requerido')
        if not _YOLO_OK:
            self.get_logger().fatal('ultralytics no encontrado — pip3 install ultralytics')
            raise RuntimeError('ultralytics requerido')

        camera_ids = [str(c) for c in self.get_parameter('camera_ids').value]
        model_path = self.get_parameter('hazmat_model').value
        self._conf = float(self.get_parameter('hazmat_conf').value)
        self._imgsz = int(self.get_parameter('hazmat_imgsz').value)
        self._demoted_period = max(1, int(self.get_parameter('demoted_period_ticks').value))
        self._confirm_frames = max(1, int(self.get_parameter('priority_confirm_frames').value))
        self._release_sec = float(self.get_parameter('priority_release_sec').value)

        if not model_path:
            self.get_logger().fatal("hazmat_worker requiere el parametro 'hazmat_model' (.pt)")
            raise RuntimeError('hazmat_model requerido')

        # Modelo cargado UNA sola vez para todas las camaras.
        self._model = _YOLO(model_path)
        self.get_logger().info(
            f'HAZMAT worker activo — modelo cargado UNA vez: {model_path} '
            f'({len(self._model.names)} clases) para cámaras: {camera_ids}')

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST, depth=1,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )

        self._slots: Dict[str, _CameraSlot] = {}
        self._result_pubs = {}
        for cam_id in camera_ids:
            self._slots[cam_id] = _CameraSlot(cam_id)
            self.create_subscription(
                CompressedImage, f'/hazmat/submit/{cam_id}',
                self._make_submit_callback(cam_id), qos)
            self._result_pubs[cam_id] = self.create_publisher(
                StringMsg, f'/hazmat/result/{cam_id}', 5)
            self.get_logger().info(
                f'  cámara "{cam_id}": /hazmat/submit/{cam_id} -> /hazmat/result/{cam_id}')

        self._boosted_cam: Optional[str] = None
        self._tick_count = 0
        self._round_robin_idx = 0

        tick_period = float(self.get_parameter('tick_period_sec').value)
        self.create_timer(tick_period, self._on_tick)

    # ─── Recepción de frames (uno por cámara, siempre el más reciente) ────

    def _make_submit_callback(self, cam_id: str):
        def _callback(msg: CompressedImage) -> None:
            # Token de identidad: "camera_id#seq" en frame_id. No se usa solo
            # el topic (que ya aisla por camara) para saber de quien es este
            # frame — se valida tambien el camera_id embebido en el mensaje.
            token_cam_id, _, seq_str = (msg.header.frame_id or '').partition('#')
            if token_cam_id != cam_id or not seq_str.isdigit():
                self.get_logger().warn(
                    f'Frame con token invalido en /hazmat/submit/{cam_id}: '
                    f'frame_id="{msg.header.frame_id}" — descartado')
                return

            try:
                buf = np.frombuffer(msg.data, dtype=np.uint8)
                bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            except Exception:
                return
            if bgr is None:
                return

            self._slots[cam_id].pending_frame = {
                'bgr': bgr, 'seq': int(seq_str), 'stamp': msg.header.stamp,
            }
        return _callback

    # ─── Scheduler de prioridad ────────────────────────────────────────

    def _pick_camera_for_tick(self) -> Optional[str]:
        ids = list(self._slots.keys())
        if not ids:
            return None
        self._tick_count += 1

        if self._boosted_cam is not None and self._boosted_cam in ids:
            demoted = [c for c in ids if c != self._boosted_cam]
            if not demoted or self._tick_count % self._demoted_period != 0:
                return self._boosted_cam
            # Piso de vigilancia: cada demoted_period ticks, un turno para
            # las camaras sin boost (rotando entre ellas si hay mas de una).
            return demoted[(self._tick_count // self._demoted_period) % len(demoted)]

        # Estado normal: round-robin simple entre todas las cámaras.
        cam_id = ids[self._round_robin_idx % len(ids)]
        self._round_robin_idx += 1
        return cam_id

    def _on_tick(self) -> None:
        cam_id = self._pick_camera_for_tick()
        if cam_id is None:
            return
        slot = self._slots[cam_id]
        if slot.pending_frame is None:
            return

        frame_entry = slot.pending_frame
        slot.pending_frame = None  # consumido — no se re-infiere el mismo frame
        bgr = frame_entry['bgr']
        seq = frame_entry['seq']
        stamp = frame_entry['stamp']

        try:
            detections = run_hazmat_yolo(self._model, bgr, self._conf, self._imgsz)
        except Exception as exc:
            # No perder este frame en silencio: si no publicamos nada, la
            # entrada de este seq queda "colgada" en el buffer del detector
            # hasta que el FIFO la expulse por antiguedad. Se publica un
            # resultado vacio para que el detector la libere de inmediato
            # (mismo tratamiento que "sin detecciones", sin logica especial
            # de reintento/recuperacion).
            self.get_logger().debug(f'HAZMAT worker inferencia error ({cam_id}): {exc}')
            detections = []

        # Scheduler de prioridad: SIN CAMBIOS respecto a la version anterior —
        # solo boostea/decae con base en si hubo deteccion, igual que antes.
        now = time.time()
        if detections:
            slot.consecutive_hits += 1
            slot.last_hit_time = now
            if slot.consecutive_hits >= self._confirm_frames and self._boosted_cam != cam_id:
                self._boosted_cam = cam_id
                self.get_logger().info(
                    f'[HAZMAT worker] prioridad -> "{cam_id}" '
                    f'(confirmado {slot.consecutive_hits} inferencias seguidas)')
        else:
            slot.consecutive_hits = 0

        # Publicar SIEMPRE (incluso detections=[] vacio) — el detector necesita
        # el resultado vacio para invalidar correctamente una confirmacion
        # pendiente; antes de esta correccion, un resultado vacio nunca se
        # publicaba y una deteccion podia quedar "colgada" indefinidamente.
        self._result_pubs[cam_id].publish(StringMsg(data=json.dumps({
            'camera_id': cam_id,
            'frame_seq': seq,
            'stamp_sec': stamp.sec,
            'stamp_nanosec': stamp.nanosec,
            'detections': detections,
        })))

        self._maybe_release_priority(now)

    def _maybe_release_priority(self, now: float) -> None:
        if self._boosted_cam is None:
            return
        boosted_slot = self._slots[self._boosted_cam]
        if now - boosted_slot.last_hit_time > self._release_sec:
            self.get_logger().info(
                f'[HAZMAT worker] prioridad -> normal '
                f'(sin señal en "{self._boosted_cam}" hace {self._release_sec:.0f}s)')
            self._boosted_cam = None
            for slot in self._slots.values():
                slot.consecutive_hits = 0


def main(args=None):
    rclpy.init(args=args)
    node = HazmatWorker()
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
