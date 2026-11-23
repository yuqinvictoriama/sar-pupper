"""Find the ball in a frame by colour.

    BallDetector(color_cfg, detector_cfg).detect(bgr) -> [Blob, ...]

A Blob is a connected patch of ball-coloured pixels: its centre, its area and
its bounding box. Everything about lenses and geometry happens later, in
measure.py.
"""
from __future__ import annotations

from dataclasses import dataclass

import cv2
import numpy as np


@dataclass
class Blob:
    u: float            # centroid, pixels (full-frame coordinates)
    v: float
    area: float         # pixels
    x: int              # bounding box
    y: int
    w: int
    h: int
    touches_edge: bool  # cut off by the frame edge: its size cannot be trusted

    @property
    def diameter_px(self) -> float:
        return 2.0 * (self.area / np.pi) ** 0.5


def color_bounds(color: dict):
    lo = np.array([color["h_min"], color["s_min"], color["v_min"]], dtype=np.uint8)
    hi = np.array([color["h_max"], color.get("s_max", 255), color.get("v_max", 255)], dtype=np.uint8)
    return lo, hi


def color_mask(bgr: np.ndarray, color: dict) -> np.ndarray:
    """0/255 mask of pixels inside the colour profile."""
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lo, hi = color_bounds(color)
    return cv2.inRange(hsv, lo, hi)


class BallDetector:
    def __init__(self, color: dict, det: dict, ignore_mask: np.ndarray | None = None):
        self.color = dict(color)
        self.min_area = float(det.get("min_area_px", 10))
        self.max_area_frac = float(det.get("max_area_frac", 0.25))
        self.open_px = int(det.get("open_px", 3))
        self.max_aspect = float(det.get("max_aspect", 2.8))
        self.min_fill = float(det.get("min_fill", 0.5))
        self.max_fill = float(det.get("max_fill", 1.15))
        self.min_circularity = float(det.get("min_circularity", 0.6))
        self.max_blobs = int(det.get("max_blobs", 6))
        self.ignore_mask = ignore_mask  # 0/255, same size as the frame; 255 = never a ball
        self._kernel = (cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.open_px, self.open_px))
                        if self.open_px >= 2 else None)

    def mask(self, bgr: np.ndarray) -> np.ndarray:
        m = color_mask(bgr, self.color)
        if self.ignore_mask is not None and self.ignore_mask.shape == m.shape:
            m[self.ignore_mask > 0] = 0
        return m

    def detect(self, bgr: np.ndarray, roi=None) -> list:
        """Blobs in the frame, largest first. roi = (x0, y0, x1, y1) limits the search."""
        full_h, full_w = bgr.shape[:2]
        x0, y0, x1, y1 = 0, 0, full_w, full_h
        if roi is not None:
            x0, y0 = max(0, int(roi[0])), max(0, int(roi[1]))
            x1, y1 = min(full_w, int(roi[2])), min(full_h, int(roi[3]))
            if x1 - x0 < 4 or y1 - y0 < 4:
                return []
        view = bgr[y0:y1, x0:x1]
        raw = color_mask(view, self.color)
        if self.ignore_mask is not None and self.ignore_mask.shape[:2] == (full_h, full_w):
            raw[self.ignore_mask[y0:y1, x0:x1] > 0] = 0
        # Components are found on the opened mask (drops speckle); size and
        # centre are then measured on the raw mask so opening does not bias them.
        work = cv2.morphologyEx(raw, cv2.MORPH_OPEN, self._kernel) if self._kernel is not None else raw
        n, _labels, stats, _cent = cv2.connectedComponentsWithStats(work, connectivity=8)
        if n <= 1:
            return []
        max_area = self.max_area_frac * full_w * full_h
        order = np.argsort(-stats[1:, cv2.CC_STAT_AREA])[: self.max_blobs * 3] + 1
        blobs = []
        vh, vw = raw.shape[:2]
        for i in order:
            bx, by, bw, bh, area = (int(v) for v in stats[i])
            if area < max(1.0, self.min_area * 0.5) or area > max_area:
                continue
            px0, py0 = max(0, bx - 1), max(0, by - 1)
            px1, py1 = min(vw, bx + bw + 1), min(vh, by + bh + 1)
            m = cv2.moments(raw[py0:py1, px0:px1], binaryImage=True)
            a = m["m00"]
            if a < self.min_area or a > max_area:
                continue
            aspect = max(bw, bh) / float(max(1, min(bw, bh)))
            if aspect > self.max_aspect:
                continue
            fill = a / (np.pi / 4.0 * bw * bh)        # 1.0 for a disc or an ellipse, 1.27 for a rectangle
            if a >= 40.0 and fill < self.min_fill:
                continue
            if a >= 80.0 and fill > self.max_fill:
                continue
            if a >= 80.0 and self.min_circularity > 0.0:
                # big enough to judge its outline: a ball is round, a shirt or a box is not
                contours, _ = cv2.findContours(raw[py0:py1, px0:px1], cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                if contours:
                    c = max(contours, key=cv2.contourArea)
                    per = cv2.arcLength(c, True)
                    if per > 0 and 4.0 * np.pi * cv2.contourArea(c) / (per * per) < self.min_circularity:
                        continue
            u = px0 + m["m10"] / a + x0
            v = py0 + m["m01"] / a + y0
            gx, gy = bx + x0, by + y0
            edge = gx <= 0 or gy <= 0 or gx + bw >= full_w or gy + bh >= full_h
            blobs.append(Blob(float(u), float(v), float(a), gx, gy, bw, bh, bool(edge)))
            if len(blobs) >= self.max_blobs:
                break
        return blobs


def draw_blobs(bgr: np.ndarray, blobs, labels=None) -> np.ndarray:
    out = bgr.copy()
    for i, b in enumerate(blobs):
        cv2.rectangle(out, (b.x - 2, b.y - 2), (b.x + b.w + 2, b.y + b.h + 2), (0, 0, 255), 1)
        text = labels[i] if labels and i < len(labels) else "%.0fpx" % b.diameter_px
        cv2.putText(out, text, (b.x, max(12, b.y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1, cv2.LINE_AA)
    return out
