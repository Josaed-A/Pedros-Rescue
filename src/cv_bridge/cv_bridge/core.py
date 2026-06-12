import sys

import cv2
import numpy as np
from sensor_msgs.msg import Image


class CvBridgeError(Exception):
    pass


class CvBridge:
    _ENCODINGS = {
        'bgr8': (np.uint8, 3),
        'rgb8': (np.uint8, 3),
        'mono8': (np.uint8, 1),
        '8UC1': (np.uint8, 1),
        '16UC1': (np.uint16, 1),
        'mono16': (np.uint16, 1),
    }

    def cv2_to_imgmsg(self, cv_image, encoding='passthrough'):
        if cv_image is None:
            raise CvBridgeError('cv_image is None')

        array = np.ascontiguousarray(cv_image)
        resolved_encoding = self._encoding_from_array(array) if encoding == 'passthrough' else encoding
        dtype, channels = self._encoding_info(resolved_encoding)

        if array.dtype != dtype:
            raise CvBridgeError(
                f'Image dtype {array.dtype} is incompatible with encoding {resolved_encoding}'
            )

        if channels == 1 and array.ndim != 2:
            raise CvBridgeError(f'Encoding {resolved_encoding} expects a single-channel image')
        if channels > 1 and (array.ndim != 3 or array.shape[2] != channels):
            raise CvBridgeError(f'Encoding {resolved_encoding} expects {channels} channels')

        msg = Image()
        msg.height = int(array.shape[0])
        msg.width = int(array.shape[1])
        msg.encoding = resolved_encoding
        msg.is_bigendian = 1 if sys.byteorder == 'big' else 0
        msg.step = int(array.strides[0])
        msg.data = array.tobytes()
        return msg

    def imgmsg_to_cv2(self, img_msg, desired_encoding='passthrough'):
        source_encoding = img_msg.encoding or 'passthrough'
        resolved_source = self._encoding_from_msg(img_msg) if source_encoding == 'passthrough' else source_encoding
        dtype, channels = self._encoding_info(resolved_source)

        itemsize = np.dtype(dtype).itemsize
        row_width = int(img_msg.width) * channels * itemsize
        if int(img_msg.step) < row_width:
            raise CvBridgeError('Image step is smaller than the encoded row width')

        flat = np.frombuffer(bytes(img_msg.data), dtype=dtype)
        step_items = int(img_msg.step) // itemsize
        try:
            rows = flat.reshape((int(img_msg.height), step_items))
        except ValueError as exc:
            raise CvBridgeError(f'Image data size does not match height/step: {exc}') from exc

        rows = rows[:, : int(img_msg.width) * channels]
        if channels == 1:
            image = rows.reshape((int(img_msg.height), int(img_msg.width))).copy()
        else:
            image = rows.reshape((int(img_msg.height), int(img_msg.width), channels)).copy()

        if img_msg.is_bigendian and sys.byteorder == 'little':
            image = image.byteswap()
        elif not img_msg.is_bigendian and sys.byteorder == 'big':
            image = image.byteswap()

        if desired_encoding in ('passthrough', resolved_source):
            return image

        return self._convert(image, resolved_source, desired_encoding)

    def _convert(self, image, source_encoding, desired_encoding):
        if source_encoding == 'rgb8' and desired_encoding == 'bgr8':
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if source_encoding == 'bgr8' and desired_encoding == 'rgb8':
            return cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        if source_encoding in ('mono8', '8UC1') and desired_encoding in ('mono8', '8UC1'):
            return image
        if source_encoding in ('16UC1', 'mono16') and desired_encoding in ('16UC1', 'mono16'):
            return image
        raise CvBridgeError(f'Unsupported conversion: {source_encoding} -> {desired_encoding}')

    def _encoding_info(self, encoding):
        try:
            return self._ENCODINGS[encoding]
        except KeyError as exc:
            raise CvBridgeError(f'Unsupported image encoding: {encoding}') from exc

    def _encoding_from_array(self, array):
        if array.dtype == np.uint8 and array.ndim == 2:
            return 'mono8'
        if array.dtype == np.uint16 and array.ndim == 2:
            return '16UC1'
        if array.dtype == np.uint8 and array.ndim == 3 and array.shape[2] == 3:
            return 'bgr8'
        raise CvBridgeError(f'Cannot infer encoding for shape={array.shape}, dtype={array.dtype}')

    def _encoding_from_msg(self, img_msg):
        if img_msg.step == img_msg.width:
            return 'mono8'
        if img_msg.step == img_msg.width * 2:
            return '16UC1'
        if img_msg.step == img_msg.width * 3:
            return 'bgr8'
        raise CvBridgeError('Cannot infer encoding from Image message')
