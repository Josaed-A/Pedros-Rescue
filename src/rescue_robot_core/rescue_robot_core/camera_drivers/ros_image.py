import sys

import cv2
import numpy as np
from sensor_msgs.msg import CompressedImage, Image


def raw_jpeg_to_compressed_image_msg(jpeg_buffer, stamp, frame_id):
    msg = CompressedImage()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.format = 'jpeg'
    msg.data = np.asarray(jpeg_buffer, dtype=np.uint8).tobytes()
    return msg


def numpy_frame_to_compressed_image_msg(frame, stamp, frame_id, fmt='jpeg', jpeg_quality=80):
    if fmt == 'jpeg':
        ok, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, int(jpeg_quality)])
    elif fmt == 'png':
        # Nivel 1: compresion rapida para no saturar la CPU de la Raspberry
        ok, buffer = cv2.imencode('.png', frame, [cv2.IMWRITE_PNG_COMPRESSION, 1])
    else:
        raise ValueError(f'Formato de compresion no soportado: {fmt}')

    if not ok:
        raise RuntimeError(f'No se pudo codificar el frame como {fmt}')

    msg = CompressedImage()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.format = fmt
    msg.data = buffer.tobytes()
    return msg


def numpy_frame_to_image_msg(frame, encoding, stamp, frame_id):
    frame = np.ascontiguousarray(frame)

    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = int(frame.shape[0])
    msg.width = int(frame.shape[1])
    msg.encoding = encoding
    msg.is_bigendian = 1 if sys.byteorder == 'big' else 0
    msg.step = int(frame.strides[0])
    msg.data = frame.tobytes()

    return msg
