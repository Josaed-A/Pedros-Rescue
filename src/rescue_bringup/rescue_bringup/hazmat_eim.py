"""
hazmat_eim.py
Backend Edge Impulse (.eim) para la deteccion HAZMAT.

Un .eim es un ejecutable Linux autocontenido exportado desde Edge Impulse
(deployment "Linux (x86_64)" / "Linux (AARCH64)"). No se importa como
libreria: se lanza como subproceso y se le habla por un socket UNIX con
mensajes JSON (protocolo del runner: `hello` -> `classify`). Asi no hace falta
ni el SDK `edge_impulse_linux` ni `ultralytics`.

La salida se adapta al mismo dict de deteccion que produce `run_hazmat_yolo`,
de modo que object_detector y hazmat_worker no distinguen el backend.

El binario es especifico de arquitectura: el export x86_64 corre en el PC;
para la Raspberry Pi hay que exportar "Linux (AARCH64)".
"""

import atexit
import json
import os
import shutil
import socket
import subprocess
import tempfile
import time
from typing import List, Tuple

import cv2
import numpy as np


class EimHazmatModel:
    """Un runner .eim vivo mientras exista la instancia (un subproceso)."""

    def __init__(self, path: str, start_timeout_sec: float = 15.0):
        self.path = os.path.abspath(path)
        if not os.path.isfile(self.path):
            raise FileNotFoundError(self.path)
        if not os.access(self.path, os.X_OK):
            os.chmod(self.path, os.stat(self.path).st_mode | 0o111)

        self._timeout = start_timeout_sec
        self._proc = None
        self._sock = None
        self._tmpdir = None
        self._msg_id = 0
        self._threshold = None
        self._start()
        atexit.register(self.close)

    # ─── Ciclo de vida del runner ─────────────────────────────────────────

    def _start(self) -> None:
        self._tmpdir = tempfile.mkdtemp(prefix='hazmat_eim_')
        sock_path = os.path.join(self._tmpdir, 'runner.sock')
        # stderr a archivo (no PIPE): un PIPE sin leer puede llenarse y
        # bloquear al runner; el archivo solo se lee si falla el arranque.
        err_path = os.path.join(self._tmpdir, 'runner.err')
        with open(err_path, 'wb') as err_file:
            self._proc = subprocess.Popen(
                [self.path, sock_path], stdout=subprocess.DEVNULL, stderr=err_file)

        deadline = time.time() + self._timeout
        while True:
            if self._proc.poll() is not None:
                with open(err_path, 'rb') as f:
                    err = f.read().decode(errors='replace').strip()
                raise RuntimeError(
                    f'el runner .eim termino al arrancar (codigo {self._proc.returncode}): '
                    f'{err[-300:]}')
            if os.path.exists(sock_path):
                try:
                    sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
                    sock.settimeout(self._timeout)
                    sock.connect(sock_path)
                    self._sock = sock
                    break
                except OSError:
                    sock.close()  # existe pero aun no escucha
            if time.time() > deadline:
                self.close()
                raise TimeoutError(f'el runner .eim no abrio su socket en {self._timeout:.0f}s')
            time.sleep(0.05)

        info = self._rpc({'hello': 1})
        params = info['model_parameters']
        if params.get('model_type') != 'object_detection':
            raise ValueError(
                f"el .eim es de tipo '{params.get('model_type')}', se necesita object_detection")

        self.input_width = int(params['image_input_width'])
        self.input_height = int(params['image_input_height'])
        self.channels = int(params.get('image_channel_count', 3))
        self.resize_mode = params.get('image_resize_mode', 'squash')
        self.names = dict(enumerate(params['labels']))
        self._label_ids = {label: i for i, label in self.names.items()}
        od_thresholds = [t for t in params.get('thresholds', [])
                         if t.get('type') == 'object_detection']
        self._threshold_id = od_thresholds[0]['id'] if od_thresholds else None
        self._threshold = None  # runner nuevo: reaplicar umbral en el proximo detect
        project = info.get('project', {})
        self.description = (
            f"Edge Impulse '{project.get('name', '?')}' v{project.get('deploy_version', '?')} "
            f'({self.input_width}x{self.input_height}, {self.resize_mode})')

    def close(self) -> None:
        if self._sock is not None:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        if self._proc is not None and self._proc.poll() is None:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                self._proc.kill()
        self._proc = None
        if self._tmpdir:
            shutil.rmtree(self._tmpdir, ignore_errors=True)
            self._tmpdir = None

    def _restart(self) -> None:
        self.close()
        self._start()

    def _rpc(self, payload: dict) -> dict:
        self._msg_id += 1
        self._sock.sendall(json.dumps(dict(payload, id=self._msg_id)).encode())
        buf = b''
        while True:
            chunk = self._sock.recv(1 << 16)
            if not chunk:
                raise ConnectionError('el runner .eim cerro el socket')
            buf += chunk
            try:
                resp = json.loads(buf.decode().rstrip('\x00'))
                break
            except (UnicodeDecodeError, json.JSONDecodeError):
                continue  # respuesta incompleta, seguir leyendo
        if not resp.get('success', False):
            raise RuntimeError(f"runner .eim: {resp.get('error', resp)}")
        return resp

    # ─── Inferencia ───────────────────────────────────────────────────────

    def _prepare(self, bgr: np.ndarray) -> Tuple[list, Tuple[float, float, float, float]]:
        """Redimensiona como lo hizo Edge Impulse al entrenar y empaqueta los
        pixeles como enteros 0xRRGGBB. Devuelve tambien (sx, sy, dx, dy) para
        llevar una coordenada del modelo al frame original:
        x_orig = (x + dx) / sx."""
        h, w = bgr.shape[:2]
        W, H = self.input_width, self.input_height

        if self.resize_mode == 'fit-shortest':
            # Escala hasta cubrir y recorta el centro (los bordes largos no se ven).
            s = max(W / w, H / h)
            rw, rh = max(W, round(w * s)), max(H, round(h * s))
            resized = cv2.resize(bgr, (rw, rh), interpolation=cv2.INTER_AREA)
            ox, oy = (rw - W) // 2, (rh - H) // 2
            img = resized[oy:oy + H, ox:ox + W]
            mapping = (s, s, float(ox), float(oy))
        elif self.resize_mode == 'fit-longest':
            # Escala hasta caber y rellena con negro (letterbox).
            s = min(W / w, H / h)
            rw, rh = max(1, round(w * s)), max(1, round(h * s))
            resized = cv2.resize(bgr, (rw, rh), interpolation=cv2.INTER_AREA)
            px, py = (W - rw) // 2, (H - rh) // 2
            img = np.zeros((H, W, 3), dtype=np.uint8)
            img[py:py + rh, px:px + rw] = resized
            mapping = (s, s, float(-px), float(-py))
        else:  # squash
            img = cv2.resize(bgr, (W, H), interpolation=cv2.INTER_AREA)
            mapping = (W / w, H / h, 0.0, 0.0)

        if self.channels == 1:
            g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.uint32)
            packed = (g << 16) | (g << 8) | g
        else:
            rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB).astype(np.uint32)
            packed = (rgb[..., 0] << 16) | (rgb[..., 1] << 8) | rgb[..., 2]
        return packed.ravel().tolist(), mapping

    def _ensure_threshold(self, conf: float) -> None:
        # El runner filtra por su propio umbral (0.5 en el export); se alinea
        # con hazmat_conf para no perder cajas entre conf y ese valor.
        if self._threshold_id is None or self._threshold == conf:
            return
        try:
            self._rpc({'set_threshold': {'id': self._threshold_id, 'min_score': conf}})
        except RuntimeError:
            pass  # runner antiguo sin set_threshold: queda su umbral fijo
        self._threshold = conf

    def detect(self, bgr: np.ndarray, conf: float) -> List[dict]:
        if self._proc is None or self._proc.poll() is not None:
            self._restart()
        try:
            self._ensure_threshold(conf)
            features, (sx, sy, dx, dy) = self._prepare(bgr)
            resp = self._rpc({'classify': features})
        except (OSError, ConnectionError):
            self._restart()  # el llamador trata esta excepcion como "sin detecciones"
            raise

        h, w = bgr.shape[:2]
        out = []
        for bb in resp.get('result', {}).get('bounding_boxes', []):
            score = float(bb.get('value', 0.0))
            if score < conf:
                continue
            x1 = int(min(max((bb['x'] + dx) / sx, 0), w - 1))
            y1 = int(min(max((bb['y'] + dy) / sy, 0), h - 1))
            x2 = int(min(max((bb['x'] + bb['width'] + dx) / sx, 0), w - 1))
            y2 = int(min(max((bb['y'] + bb['height'] + dy) / sy, 0), h - 1))
            label = str(bb.get('label', '?'))
            out.append({
                'type': 'hazmat_sign',
                'name': label.replace(' ', '_')[:20],
                'class_id': self._label_ids.get(label, 0),
                'conf': score,
                'u': (x1 + x2) // 2, 'v': (y1 + y2) // 2,
                'x1': x1, 'y1': y1, 'x2': x2, 'y2': y2,
            })
        return out
