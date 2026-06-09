import cv2


class QrDetector:
    def __init__(self):
        self.detector = cv2.QRCodeDetector()

    def detect_and_annotate(self, frame):
        data, bbox, _ = self.detector.detectAndDecode(frame)

        if not data or bbox is None or len(bbox) == 0:
            return frame, ''

        points = bbox[0].astype(int)
        point_count = len(points)

        for index in range(point_count):
            start = tuple(points[index])
            end = tuple(points[(index + 1) % point_count])
            cv2.line(frame, start, end, (0, 255, 0), 3)

        cv2.putText(
            frame,
            data,
            (points[0][0], points[0][1] - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 0, 255),
            2
        )

        return frame, data
