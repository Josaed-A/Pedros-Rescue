import base64

import cv2


def bgr_frame_to_png_data(frame, max_width, max_height):
    if frame is None:
        return None

    display_frame = resize_to_fit(frame, max_width, max_height)
    ok, encoded = cv2.imencode('.png', display_frame)

    if not ok:
        return None

    return base64.b64encode(encoded).decode('ascii')


def resize_to_fit(frame, max_width, max_height):
    height, width = frame.shape[:2]

    if width <= 0 or height <= 0:
        return frame

    scale = min(max_width / width, max_height / height)

    if abs(scale - 1.0) < 0.01:
        return frame

    target_size = (max(1, int(width * scale)), max(1, int(height * scale)))
    interp = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    return cv2.resize(frame, target_size, interpolation=interp)


def depth_frame_to_color(depth_frame):
    depth_visual = cv2.normalize(
        depth_frame,
        None,
        0,
        255,
        cv2.NORM_MINMAX,
        dtype=cv2.CV_8U
    )
    return cv2.applyColorMap(depth_visual, cv2.COLORMAP_JET)
