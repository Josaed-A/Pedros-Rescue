#!/usr/bin/env python3
"""
test_hazmat_camera.py — detección hazmat en tiempo real (OpenCV puro, sin ROS).
Uso: python3 hazmat/training/test_hazmat_camera.py [--model best|ei|RUTA] [--device 2] [--conf 0.4]
     Q / ESC → salir   |   +/- → confianza   |   S → screenshot

--model best → src/rescue_bringup/models/best.pt           (YOLO, 49 clases)
--model ei   → src/rescue_bringup/models/hazmat_ei_v3.eim  (Edge Impulse, 15 clases)
Ambos pasan por el mismo loader que usa el robot (rescue_bringup.hazmat_common).
"""
import os, sys

# Debe ejecutarse ANTES de importar cv2/Qt
if os.environ.get('QT_QPA_PLATFORM') != 'xcb':
    os.environ['QT_QPA_PLATFORM'] = 'xcb'
    os.execv(sys.executable, [sys.executable] + sys.argv)

import argparse, time
from pathlib import Path

import cv2

ROOT   = Path(__file__).parent.parent.parent
MODELS = ROOT / 'src' / 'rescue_bringup' / 'models'
sys.path.insert(0, str(ROOT / 'src' / 'rescue_bringup'))
from rescue_bringup.hazmat_common import hazmat_color, load_hazmat_model, run_hazmat  # noqa: E402

SHORTCUTS = {'best': MODELS / 'best.pt', 'ei': MODELS / 'hazmat_ei_v3.eim'}
WINDOW = 'Pedro Rescue — Hazmat'

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--model',  default='best', help='best | ei | ruta a .pt/.eim')
    p.add_argument('--device', type=int, default=2)
    p.add_argument('--conf',   type=float, default=0.40)
    p.add_argument('--imgsz',  type=int, default=416, help='solo aplica a .pt')
    return p.parse_args()

def model_view_box(model, w, h):
    """Zona del frame que realmente ve un .eim fit-shortest (recorte central)."""
    if getattr(model, 'resize_mode', None) != 'fit-shortest':
        return None
    s = max(model.input_width / w, model.input_height / h)
    cw, ch = model.input_width / s, model.input_height / s
    x1, y1 = int((w - cw) / 2), int((h - ch) / 2)
    return x1, y1, int(x1 + cw), int(y1 + ch)

def main():
    args = parse_args()
    model_path = SHORTCUTS.get(args.model, Path(args.model))

    model = load_hazmat_model(str(model_path))
    backend = getattr(model, 'description', 'YOLO')
    print(f'Modelo cargado — {model_path.name} [{backend}] — {len(model.names)} clases')

    cap = cv2.VideoCapture(args.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 15)
    for _ in range(5): cap.read()   # warmup

    # Ventana sin toolbar Qt
    cv2.namedWindow(WINDOW, cv2.WINDOW_GUI_NORMAL)
    cv2.resizeWindow(WINDOW, 640, 480)

    conf = args.conf
    shot = 0
    t_prev = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        detections = run_hazmat(model, frame, conf, args.imgsz)

        view = model_view_box(model, frame.shape[1], frame.shape[0])
        if view:
            # Oscurece lo que el modelo no ve para que la señal se centre.
            vx1, vy1, vx2, vy2 = view
            shade = frame.copy()
            shade[:, :vx1] = 0; shade[:, vx2:] = 0
            shade[:vy1, :] = 0; shade[vy2:, :] = 0
            frame = cv2.addWeighted(shade, 0.55, frame, 0.45, 0)
            cv2.rectangle(frame, (vx1, vy1), (vx2 - 1, vy2 - 1), (180, 180, 180), 1)

        for det in detections:
            col = hazmat_color(det['class_id'])
            x1, y1, x2, y2 = det['x1'], det['y1'], det['x2'], det['y2']
            cv2.rectangle(frame, (x1,y1), (x2,y2), col, 2)
            lbl = f"{det['name'][:16]} {det['conf']:.2f}"
            (tw,th),_ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            cv2.rectangle(frame, (x1, y1-th-6), (x1+tw+4, y1), col, -1)
            cv2.putText(frame, lbl, (x1+2, y1-4),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

        fps = 1.0 / max(time.time() - t_prev, 1e-6)
        t_prev = time.time()
        cv2.putText(frame, f'FPS:{fps:.1f}  conf:{conf:.2f}  det:{len(detections)}',
                    (8,22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)
        cv2.putText(frame, model_path.name, (8, frame.shape[0] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1)

        cv2.imshow(WINDOW, frame)

        k = cv2.waitKey(1) & 0xFF
        if k in (ord('q'), 27):
            break
        elif k == ord('+'):
            conf = min(0.95, conf + 0.05); print(f'conf={conf:.2f}')
        elif k == ord('-'):
            conf = max(0.05, conf - 0.05); print(f'conf={conf:.2f}')
        elif k == ord('s'):
            f = f'screenshot_{shot:03d}.jpg'; cv2.imwrite(f, frame)
            print(f'Guardado: {f}'); shot += 1

    cap.release()
    cv2.destroyAllWindows()
    close = getattr(model, 'close', None)
    if close:
        close()

if __name__ == '__main__':
    main()
