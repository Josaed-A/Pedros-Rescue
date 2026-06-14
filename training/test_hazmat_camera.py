#!/usr/bin/env python3
"""
test_hazmat_camera.py — detección hazmat en tiempo real.
Uso: python3 training/test_hazmat_camera.py [--device 2] [--conf 0.4]
     Q / ESC → salir   |   +/- → confianza   |   S → screenshot
"""
import os, sys

# Debe ejecutarse ANTES de importar cv2/Qt
if os.environ.get('QT_QPA_PLATFORM') != 'xcb':
    os.environ['QT_QPA_PLATFORM'] = 'xcb'
    os.execv(sys.executable, [sys.executable] + sys.argv)

import argparse, time
from pathlib import Path

import cv2
from ultralytics import YOLO

ROOT  = Path(__file__).parent.parent
MODEL = ROOT / 'src' / 'rescue_bringup' / 'models' / 'hazmat_yolo.pt'

COLORS = [
    (0,165,255),(0,255,0),(255,80,0),(0,200,255),
    (255,0,200),(200,255,0),(130,0,255),(0,130,255),
]

def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--device', type=int, default=2)
    p.add_argument('--conf',   type=float, default=0.40)
    p.add_argument('--imgsz',  type=int, default=416)
    return p.parse_args()

def main():
    args = parse_args()

    model = YOLO(str(MODEL))
    print(f'Modelo cargado — {len(model.names)} clases')

    cap = cv2.VideoCapture(args.device)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    cap.set(cv2.CAP_PROP_FPS, 15)
    for _ in range(5): cap.read()   # warmup

    # Ventana sin toolbar Qt
    cv2.namedWindow('Pedro Rescue — Hazmat', cv2.WINDOW_GUI_NORMAL)
    cv2.resizeWindow('Pedro Rescue — Hazmat', 640, 480)

    conf = args.conf
    shot = 0
    t_prev = time.time()

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        results = model(frame, conf=conf, imgsz=args.imgsz, verbose=False)

        for r in results:
            for box in r.boxes:
                cid  = int(box.cls[0])
                name = model.names[cid]
                score = float(box.conf[0])
                x1,y1,x2,y2 = map(int, box.xyxy[0].tolist())
                col = COLORS[cid % len(COLORS)]
                cv2.rectangle(frame, (x1,y1), (x2,y2), col, 2)
                lbl = f'{name[:16]} {score:.2f}'
                (tw,th),_ = cv2.getTextSize(lbl, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
                cv2.rectangle(frame, (x1, y1-th-6), (x1+tw+4, y1), col, -1)
                cv2.putText(frame, lbl, (x1+2, y1-4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 1)

        fps = 1.0 / max(time.time() - t_prev, 1e-6)
        t_prev = time.time()
        n = sum(len(r.boxes) for r in results)
        cv2.putText(frame, f'FPS:{fps:.1f}  conf:{conf:.2f}  det:{n}',
                    (8,22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 2)

        cv2.imshow('Pedro Rescue — Hazmat', frame)

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

if __name__ == '__main__':
    main()
