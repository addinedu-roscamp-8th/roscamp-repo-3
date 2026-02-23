import threading
from pathlib import Path
from typing import Iterable, Set, Tuple

import cv2
import numpy as np

try:
    from PIL import Image, ImageDraw, ImageFont
except Exception:  # pragma: no cover
    Image = None
    ImageDraw = None
    ImageFont = None


_KOREAN_FONT_CANDIDATES = [
    Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"),
    Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc"),
]


def _load_korean_font(size: int):
    if ImageFont is None:
        return None
    for font_path in _KOREAN_FONT_CANDIDATES:
        if font_path.exists():
            try:
                return ImageFont.truetype(str(font_path), size=size)
            except Exception:
                continue
    return None


class GuiAlertOverlay:
    def __init__(
        self,
        fire_labels: Iterable[str],
        ashes_labels: Iterable[str],
        blink_hz: float = 2.0,
        extinguish_text_duration_sec: float = 5.0,
    ):
        self.fire_labels: Set[str] = {str(x).lower() for x in fire_labels}
        self.ashes_labels: Set[str] = {str(x).lower() for x in ashes_labels}
        self.blink_hz = blink_hz
        self.extinguish_text_duration_sec = extinguish_text_duration_sec
        self._lock = threading.Lock()
        self._fire_active = False
        self._extinguish_text_until = 0.0

    def update_from_labels(self, labels: Iterable[str], now: float):
        normalized = {str(x).lower() for x in labels}
        fire_detected = any(label in self.fire_labels for label in normalized)
        ashes_detected = any(label in self.ashes_labels for label in normalized)

        with self._lock:
            if fire_detected:
                self._fire_active = True
                self._extinguish_text_until = 0.0
            if ashes_detected:
                self._fire_active = False
                self._extinguish_text_until = now + self.extinguish_text_duration_sec

    def apply_overlay(self, frame: np.ndarray, now: float) -> np.ndarray:
        with self._lock:
            fire_active = self._fire_active
            extinguish_until = self._extinguish_text_until
            if not fire_active and extinguish_until > 0.0 and now >= extinguish_until:
                self._extinguish_text_until = 0.0
                extinguish_until = 0.0

        if fire_active:
            blink_on = int(now * self.blink_hz * 2) % 2 == 0
            if blink_on:
                overlay = frame.copy()
                overlay[:] = (0, 0, 255)
                frame = cv2.addWeighted(overlay, 0.35, frame, 0.65, 0.0)
            self._draw_center_text(frame, "화재 감지", (255, 255, 255))
            return frame

        if extinguish_until > now:
            self._draw_center_text(frame, "진압 완료", (0, 255, 0))
            return frame

        return frame

    @staticmethod
    def _draw_center_text(frame: np.ndarray, text: str, color: Tuple[int, int, int]):
        h, w = frame.shape[:2]
        font_px = max(30, min(110, int(w / 10)))
        pil_font = _load_korean_font(font_px)

        if pil_font is not None and Image is not None and ImageDraw is not None:
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            pil_img = Image.fromarray(rgb)
            draw = ImageDraw.Draw(pil_img)
            bbox = draw.textbbox((0, 0), text, font=pil_font)
            text_w = bbox[2] - bbox[0]
            text_h = bbox[3] - bbox[1]
            x = max(0, (w - text_w) // 2)
            y = max(0, (h - text_h) // 2)

            draw.text((x, y), text, font=pil_font, fill=(0, 0, 0), stroke_width=6, stroke_fill=(0, 0, 0))
            draw.text((x, y), text, font=pil_font, fill=(color[2], color[1], color[0]))
            frame[:] = cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)
            return

        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = max(1.2, min(3.2, w / 520.0))
        thickness = max(2, int(font_scale * 2))
        text_size, _ = cv2.getTextSize(text, font, font_scale, thickness)
        x = max(0, (w - text_size[0]) // 2)
        y = max(text_size[1], (h + text_size[1]) // 2)
        cv2.putText(frame, text, (x, y), font, font_scale, (0, 0, 0), thickness + 3, cv2.LINE_AA)
        cv2.putText(frame, text, (x, y), font, font_scale, color, thickness, cv2.LINE_AA)
