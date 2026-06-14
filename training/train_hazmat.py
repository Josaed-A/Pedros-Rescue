#!/usr/bin/env python3
"""
Entrena YOLOv8 sobre el dataset de señales hazmat RoboCup Rescue 2026.

Uso:
    python3 training/train_hazmat.py [--epochs 50] [--model yolov8n.pt] [--imgsz 640]

El modelo entrenado se guarda en:
    src/rescue_bringup/models/hazmat_yolo.pt
"""

import argparse
import os
import shutil
from pathlib import Path

ROOT = Path(__file__).parent.parent
DATA_YAML = ROOT / "datasets" / "hazmat" / "data.yaml"
OUTPUT_MODEL = ROOT / "src" / "rescue_bringup" / "models" / "hazmat_yolo.pt"


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--model",  default="yolov8n.pt",
                   help="Modelo base YOLO (yolov8n/s/m.pt) — n=más rápido, m=más preciso")
    p.add_argument("--epochs", type=int, default=50,
                   help="Épocas de entrenamiento (default: 50)")
    p.add_argument("--imgsz",  type=int, default=640,
                   help="Tamaño de imagen de entrenamiento (default: 640)")
    p.add_argument("--batch",  type=int, default=16,
                   help="Batch size (default: 16; reducir a 8 si hay OOM)")
    p.add_argument("--device", default="",
                   help="Dispositivo: '' = auto, '0' = GPU 0, 'cpu' = CPU")
    p.add_argument("--workers", type=int, default=4,
                   help="Workers de DataLoader (default: 4)")
    return p.parse_args()


def main():
    args = parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        print("ERROR: ultralytics no instalado. Ejecuta:")
        print("  pip3 install ultralytics")
        raise SystemExit(1)

    if not DATA_YAML.exists():
        print(f"ERROR: No se encontró {DATA_YAML}")
        raise SystemExit(1)

    print(f"[train_hazmat] Modelo base : {args.model}")
    print(f"[train_hazmat] Dataset     : {DATA_YAML}")
    print(f"[train_hazmat] Épocas      : {args.epochs}")
    print(f"[train_hazmat] Imagen size : {args.imgsz}")
    print(f"[train_hazmat] Batch       : {args.batch}")
    print()

    model = YOLO(args.model)

    results = model.train(
        data=str(DATA_YAML),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        device=args.device if args.device else None,
        workers=args.workers,
        project=str(ROOT / "training" / "runs"),
        name="hazmat",
        exist_ok=True,
        # Augmentaciones útiles para señales en escenarios de rescate
        hsv_h=0.015,
        hsv_s=0.7,
        hsv_v=0.4,
        fliplr=0.5,
        mosaic=1.0,
        degrees=10.0,
        translate=0.1,
        scale=0.5,
        # Parar temprano si no mejora
        patience=15,
    )

    best_pt = Path(results.save_dir) / "weights" / "best.pt"
    if best_pt.exists():
        OUTPUT_MODEL.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy(best_pt, OUTPUT_MODEL)
        print(f"\n[train_hazmat] Modelo guardado en: {OUTPUT_MODEL}")
        print(f"[train_hazmat] mAP50: {results.results_dict.get('metrics/mAP50(B)', 'N/A')}")
    else:
        print(f"[train_hazmat] ADVERTENCIA: no se encontró best.pt en {best_pt}")
        print(f"[train_hazmat] Busca los pesos en: {results.save_dir}/weights/")


if __name__ == "__main__":
    main()
