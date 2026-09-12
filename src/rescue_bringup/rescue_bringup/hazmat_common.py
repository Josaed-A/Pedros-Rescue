"""
hazmat_common.py
Inferencia YOLO-hazmat compartida entre object_detector (modo local, un
modelo por instancia) y hazmat_worker (modo compartido, un modelo para N
camaras). Un solo lugar para la forma del dict de deteccion — evita mantener
dos copias de este bucle sincronizadas.
"""

from typing import List


def run_hazmat_yolo(model, bgr, conf: float) -> List[dict]:
    """Corre el modelo YOLO hazmat sobre un frame BGR y arma la lista de
    detecciones en el formato que consume el resto del pipeline
    (_draw_annotated, _process_detection, _update_hazmat_alerts)."""
    results_out = []
    res = model(bgr, conf=conf, verbose=False)
    for r in res:
        for box in r.boxes:
            cls_name = model.names[int(box.cls[0])]
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)
            results_out.append({
                'type': 'hazmat_sign',
                'name': cls_name.replace(' ', '_')[:20],
                'conf': float(box.conf[0]),
                'u': cx, 'v': cy,
                'x1': int(x1), 'y1': int(y1), 'x2': int(x2), 'y2': int(y2),
            })
    return results_out
