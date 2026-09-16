#!/usr/bin/env python3
"""
hazmat_viewer.py
Visor OpenCV minimo para la prueba simple (test_hazmat_simple.launch.py):
muestra el feed anotado de object_detector y superpone el ritmo del stream,
el del hazmat_worker y las señales de la ultima inferencia. No es parte del
robot — reemplaza al dashboard cuando solo se quiere evaluar el modelo.

Q / ESC cierra la ventana.
"""

import json
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String as StringMsg


def _rate(times: deque) -> float:
    if len(times) < 2 or time.time() - times[-1] > 2.0:
        return 0.0
    span = times[-1] - times[0]
    return (len(times) - 1) / span if span > 0 else 0.0


class HazmatViewer(Node):
    def __init__(self):
        super().__init__('hazmat_viewer')
        self.declare_parameter('image_topic', '/camera/color/image_annotated/compressed')
        self.declare_parameter('result_topic', '/hazmat/result/front')
        self.declare_parameter('window_name', 'HAZMAT - prueba simple')

        self._window = self.get_parameter('window_name').value
        self._frame = None
        self._frame_times = deque(maxlen=30)
        self._result_times = deque(maxlen=30)
        self._labels = []

        qos = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=1,
                         reliability=QoSReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            CompressedImage, self.get_parameter('image_topic').value, self._on_image, qos)
        self.create_subscription(
            StringMsg, self.get_parameter('result_topic').value, self._on_result, 5)

        cv2.namedWindow(self._window, cv2.WINDOW_NORMAL)
        self.create_timer(1.0 / 30.0, self._on_gui)

    def _on_image(self, msg: CompressedImage) -> None:
        frame = cv2.imdecode(np.frombuffer(msg.data, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is not None:
            self._frame = frame
            self._frame_times.append(time.time())

    def _on_result(self, msg: StringMsg) -> None:
        try:
            detections = json.loads(msg.data).get('detections', [])
        except json.JSONDecodeError:
            return
        self._result_times.append(time.time())
        self._labels = [f"{d['name']} {d['conf']:.2f}" for d in detections]

    def _on_gui(self) -> None:
        if self._frame is None:
            img = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(img, 'esperando imagen...', (180, 240),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 2)
        else:
            img = self._frame.copy()
        lines = [
            f'stream {_rate(self._frame_times):4.1f} Hz',
            f'worker {_rate(self._result_times):4.1f} inf/s',
            'senales: ' + (', '.join(self._labels) if self._labels else '-'),
        ]
        cv2.rectangle(img, (0, 0), (img.shape[1], 22 * len(lines) + 8), (0, 0, 0), -1)
        for i, text in enumerate(lines):
            cv2.putText(img, text, (8, 22 + 22 * i), cv2.FONT_HERSHEY_SIMPLEX,
                        0.55, (0, 255, 0), 1, cv2.LINE_AA)
        cv2.imshow(self._window, img)
        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            raise SystemExit


def main(args=None):
    rclpy.init(args=args)
    node = HazmatViewer()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
