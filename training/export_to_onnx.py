#!/usr/bin/env python3
"""
export_to_onnx.py
Exporta el modelo hazmat entrenado a ONNX para la Raspberry Pi.

En la Pi no hay GPU — ONNX Runtime corre mucho más rápido que PyTorch en ARM.
El modelo ONNX también funciona con OpenCV DNN (sin instalar ultralytics en la Pi).

Uso:
    python3 training/export_to_onnx.py
    python3 training/export_to_onnx.py --model src/rescue_bringup/models/hazmat_yolo.pt

Salida:
    src/rescue_bringup/models/hazmat_yolo.onnx
"""

import argparse
from pathlib import Path

ROOT = Path(__file__).parent.parent
DEFAULT_PT   = ROOT / 'src' / 'rescue_bringup' / 'models' / 'hazmat_yolo.pt'
DEFAULT_ONNX = ROOT / 'src' / 'rescue_bringup' / 'models' / 'hazmat_yolo.onnx'


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument('--model',  default=str(DEFAULT_PT),  help='Ruta al .pt entrenado')
    p.add_argument('--output', default=str(DEFAULT_ONNX), help='Ruta de salida .onnx')
    p.add_argument('--imgsz',  type=int, default=416,    help='Tamaño de imagen (debe coincidir con el entrenamiento)')
    return p.parse_args()


def main():
    args = parse_args()

    try:
        from ultralytics import YOLO
    except ImportError:
        print('ERROR: pip3 install ultralytics')
        raise SystemExit(1)

    pt_path = Path(args.model)
    if not pt_path.exists():
        print(f'ERROR: No se encontró {pt_path}')
        print('Entrena primero con: python3 training/train_hazmat.py')
        raise SystemExit(1)

    print(f'Exportando {pt_path} → ONNX ...')
    model = YOLO(str(pt_path))

    # simplify=True reduce el grafo para ORT, opset 11 compatible con Pi
    exported = model.export(
        format='onnx',
        imgsz=args.imgsz,
        simplify=True,
        opset=11,
    )

    out_path = Path(args.output)
    if Path(exported) != out_path:
        import shutil
        shutil.copy(exported, out_path)

    size_mb = out_path.stat().st_size / 1e6
    print(f'\nExportado: {out_path}  ({size_mb:.1f} MB)')
    print()
    print('Para usarlo en la Pi (sin ultralytics):')
    print('  pip3 install onnxruntime  # ~50MB, funciona en ARM')
    print('  # O con OpenCV DNN (ya instalado con cv2):')
    print('  #   net = cv2.dnn.readNetFromONNX("hazmat_yolo.onnx")')


if __name__ == '__main__':
    main()
