import sys

import numpy as np
from sensor_msgs.msg import Image


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
