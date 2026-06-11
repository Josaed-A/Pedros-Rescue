import sys

import cv2
import numpy as np


def image_msg_to_numpy(msg, desired_encoding=None):
    encoding = msg.encoding.lower()
    dtype, channels = get_encoding_layout(encoding)

    frame = np.frombuffer(msg.data, dtype=dtype)
    if msg.is_bigendian != (sys.byteorder == 'big'):
        frame = frame.byteswap()

    row_items = msg.step // dtype().nbytes
    if channels == 1:
        frame = frame.reshape((msg.height, row_items))[:, :msg.width]
    else:
        row_pixels = row_items // channels
        frame = frame.reshape((msg.height, row_pixels, channels))[:, :msg.width, :]

    frame = np.ascontiguousarray(frame)
    if desired_encoding is None or desired_encoding.lower() == encoding:
        return frame

    desired = desired_encoding.lower()
    if encoding == 'rgb8' and desired == 'bgr8':
        return cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

    if encoding == 'bgr8' and desired == 'rgb8':
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    if encoding in ('16uc1', 'mono16') and desired == '16uc1':
        return frame.astype(np.uint16, copy=False)

    raise ValueError(f'No se puede convertir imagen de {msg.encoding} a {desired_encoding}')


def get_encoding_layout(encoding):
    if encoding in ('bgr8', 'rgb8'):
        return np.uint8, 3

    if encoding in ('mono8', '8uc1'):
        return np.uint8, 1

    if encoding in ('16uc1', 'mono16'):
        return np.uint16, 1

    raise ValueError(f'Encoding de imagen no soportado: {encoding}')
