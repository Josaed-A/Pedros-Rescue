import cv2
import zxingcpp

_QR_FORMAT = zxingcpp.BarcodeFormat.QRCode


class QrDetector:
    """QR detector basado en zxing-cpp — mucho más resiliente que
    cv2.QRCodeDetector ante ángulo/perspectiva, baja resolución y códigos
    parcialmente cortados. try_rotate/try_downscale/try_invert (todos
    activos por defecto) cubren rotación arbitraria, distancia y contraste
    invertido; el reintento con upscale cubre códigos pequeños/lejanos.
    """

    def _decode(self, gray):
        return [
            r for r in zxingcpp.read_barcodes(gray, formats=_QR_FORMAT)
            if r.valid and r.text
        ]

    def detect_and_annotate(self, frame):
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        results = self._decode(gray)
        scale = 1.0
        if not results:
            # Reintento con upscale: ayuda con códigos pequeños/lejanos que
            # zxing-cpp no reconstruye a resolución nativa (solo hace
            # downscale automático, nunca upscale).
            scale = 2.0
            big = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
            results = self._decode(big)

        if not results:
            return frame, ''

        result = results[0]
        data = result.text
        pos = result.position
        points = [
            (int(pos.top_left.x / scale), int(pos.top_left.y / scale)),
            (int(pos.top_right.x / scale), int(pos.top_right.y / scale)),
            (int(pos.bottom_right.x / scale), int(pos.bottom_right.y / scale)),
            (int(pos.bottom_left.x / scale), int(pos.bottom_left.y / scale)),
        ]

        for index in range(4):
            cv2.line(frame, points[index], points[(index + 1) % 4], (0, 255, 0), 3)

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
