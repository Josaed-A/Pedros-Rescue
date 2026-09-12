#!/usr/bin/env python3
"""
hazmat_worker.py
Worker HAZMAT compartido por N cámaras — carga el modelo YOLO hazmat UNA sola
vez (a diferencia de object_detector en modo 'local', que carga su propia
copia por instancia) y sirve inferencia a cada cámara vía topics:

  /hazmat/submit/<camera_id>  (CompressedImage, publicado por cada
                                object_detector con hazmat_mode='worker')
       -> hazmat_worker
  /hazmat/result/<camera_id>  (std_msgs/String, JSON: lista de detecciones
                                en el mismo formato que produce
                                object_detector._detect_hazmat_yolo)

No toca ninguna captura de cámara ni el streaming: cada cámara sigue
publicando con su propio OpenCV (logitech_pub) exactamente igual que antes;
este nodo solo consume los frames que cada object_detector ya decodifica y
le reenvía. Tampoco reemplaza el dibujo/alertas/CSV/RViz — todo eso lo sigue
haciendo cada object_detector con los resultados que recibe de aquí (ver
_get_worker_hazmat_detections / _on_hazmat_result en object_detector.py).

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
    pendiente más reciente (se sobreescribe, nunca se acumulan viejos)."""

    def __init__(self, camera_id: str):
        self.camera_id = camera_id
        self.pending_frame: Optional[np.ndarray] = None
        self.consecutive_hits = 0
        self.last_hit_time = 0.0


class HazmatWorker(Node):
    def __init__(self):
        super().__init__('hazmat_worker')

        self.declare_parameter('camera_ids', ['front', 'astra'])
        self.declare_parameter('hazmat_model', '')
        self.declare_parameter('hazmat_conf', 0.40)
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
            try:
                buf = np.frombuffer(msg.data, dtype=np.uint8)
                bgr = cv2.imdecode(buf, cv2.IMREAD_COLOR)
            except Exception:
                return
            if bgr is None:
                return
            self._slots[cam_id].pending_frame = bgr
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

        bgr = slot.pending_frame
        slot.pending_frame = None  # consumido — no se re-infiere el mismo frame

        try:
            detections = run_hazmat_yolo(self._model, bgr, self._conf)
        except Exception as exc:
            self.get_logger().debug(f'HAZMAT worker inferencia error ({cam_id}): {exc}')
            return

        now = time.time()
        if detections:
            slot.consecutive_hits += 1
            slot.last_hit_time = now
            if slot.consecutive_hits >= self._confirm_frames and self._boosted_cam != cam_id:
                self._boosted_cam = cam_id
                self.get_logger().info(
                    f'[HAZMAT worker] prioridad -> "{cam_id}" '
                    f'(confirmado {slot.consecutive_hits} inferencias seguidas)')
            self._result_pubs[cam_id].publish(StringMsg(data=json.dumps(detections)))
        else:
            slot.consecutive_hits = 0

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
