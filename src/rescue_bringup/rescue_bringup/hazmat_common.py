"""
hazmat_common.py
Inferencia hazmat compartida entre object_detector (modo local, un modelo por
instancia) y hazmat_worker (modo compartido, un modelo para N camaras). Un
solo lugar para cargar el modelo y para la forma del dict de deteccion — evita
mantener dos copias sincronizadas.

Backends segun la extension de `hazmat_model`:
  - .pt / .onnx → ultralytics YOLO (best.pt, 49 clases)
  - .eim        → runner Edge Impulse (ver hazmat_eim.py)
"""

import os
from typing import List

from rescue_bringup.hazmat_eim import EimHazmatModel

# Paleta BGR por clase (class_id % len) — la misma en la interfaz
# (object_detector._draw_annotated) y en hazmat/training/test_hazmat_camera.py.
HAZMAT_COLORS = [
    (0, 165, 255), (0, 255, 0), (255, 80, 0), (0, 200, 255),
    (255, 0, 200), (200, 255, 0), (130, 0, 255), (0, 130, 255),
]


def hazmat_color(class_id: int):
    return HAZMAT_COLORS[int(class_id) % len(HAZMAT_COLORS)]


def load_hazmat_model(path: str):
    """Carga el modelo hazmat con el backend que corresponde a su extension."""
    if os.path.splitext(path)[1].lower() == '.eim':
        return EimHazmatModel(path)
    from ultralytics import YOLO
    return YOLO(path)


def run_hazmat(model, bgr, conf: float, imgsz: int = 640) -> List[dict]:
    """Corre cualquier modelo devuelto por load_hazmat_model. `imgsz` solo
    aplica a YOLO: un .eim tiene su resolucion de entrada fija."""
    if isinstance(model, EimHazmatModel):
        return model.detect(bgr, conf)
    return run_hazmat_yolo(model, bgr, conf, imgsz)


def run_hazmat_yolo(model, bgr, conf: float, imgsz: int = 640) -> List[dict]:
    """Corre el modelo YOLO hazmat sobre un frame BGR y arma la lista de
    detecciones en el formato que consume el resto del pipeline
    (_draw_annotated, _process_detection, _update_hazmat_alerts).

    `imgsz` es la resolucion de inferencia: 640 es el default de ultralytics
    (comportamiento historico); 416 es lo que usa el script standalone
    hazmat/training/test_hazmat_camera.py — mas rapido, mas detecciones por
    segundo a igual CPU."""
    results_out = []
    res = model(bgr, conf=conf, imgsz=imgsz, verbose=False)
    for r in res:
        for box in r.boxes:
            class_id = int(box.cls[0])
            cls_name = model.names[class_id]
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            results_out.append({
                'type': 'hazmat_sign',
                'name': cls_name.replace(' ', '_')[:20],
                'class_id': class_id,
                'conf': float(box.conf[0]),
                'u': cx, 'v': cy,
                'x1': int(x1), 'y1': int(y1), 'x2': int(x2), 'y2': int(y2),
            })
    return results_out
