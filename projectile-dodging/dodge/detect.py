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


def chroma_of(hsv: np.ndarray) -> np.ndarray:
    """Colourfulness of each pixel, 0-255: the gap between its strongest and weakest colour channel.

    (For 8-bit HSV that gap is exactly saturation x brightness / 255.)
    """
    # extractChannel hands OpenCV contiguous planes; slicing the array would make it copy them first
    return cv2.multiply(cv2.extractChannel(hsv, 1), cv2.extractChannel(hsv, 2), scale=1.0 / 255.0)


def color_masks(bgr: np.ndarray, color: dict):
    """(mask, seed): 0/255 masks of ball-coloured pixels and of unmistakably ball-coloured ones.

    A pixel is ball-coloured when its hue is in range, it is saturated enough
    (s_min), bright enough (v_min) and colourful enough in absolute terms
    (c_min). Saturation alone is a ratio, and in the dark parts of a noisy
    picture that ratio is large by accident: a dim floor passes it. Absolute
    colourfulness is what a neon ball has and a dim floor has not.

    seed marks the pixels that clear a higher bar (c_seed). A patch only
    counts as a ball if it contains some: the low bar gives the ball its full
    outline, shaded side included; the high bar keeps out patches of noise
    that only just scrape past the low one. seed is None when c_seed is unset.
    """
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    lo, hi = color_bounds(color)
    mask = cv2.inRange(hsv, lo, hi)
    c_min = int(color.get("c_min", 0) or 0)
    c_seed = int(color.get("c_seed", 0) or 0)
    if c_min <= 0 and c_seed <= 0:
        return mask, None
    chroma = chroma_of(hsv)
    if c_min > 0:
        mask = cv2.bitwise_and(mask, cv2.compare(chroma, c_min, cv2.CMP_GE))
    seed = cv2.bitwise_and(mask, cv2.compare(chroma, c_seed, cv2.CMP_GE)) if c_seed > c_min else None
    return mask, seed


def color_mask(bgr: np.ndarray, color: dict) -> np.ndarray:
    """0/255 mask of pixels inside the colour profile."""
    return color_masks(bgr, color)[0]


def circle_params(image_circle: dict | None, ref_size, width: int, height: int):
    """(cx, cy, usable radius) of a lens's lit disc in a width x height frame, or None.

    image_circle = {cx, cy, r, margin_frac} is given for frames of ref_size
    (the size the camera's intrinsics are for). The usable radius stops short
    of the rim by margin_frac of r: the last few percent of a fisheye picture
    are colour fringes, some of them ball-green.
    """
    if not image_circle or not image_circle.get("r"):
        return None
    ref_w, ref_h = ref_size if ref_size and ref_size[0] else (width, height)
    sx, sy = width / float(ref_w), height / float(ref_h)
    r = float(image_circle["r"]) * (1.0 - float(image_circle.get("margin_frac", 0.08)))
    return float(image_circle["cx"]) * sx, float(image_circle["cy"]) * sy, r * 0.5 * (sx + sy)


def circle_ignore_mask(image_circle: dict | None, ref_size, width: int, height: int):
    """0/255 mask (255 = ignore) of everything outside the usable disc, or None."""
    c = circle_params(image_circle, ref_size, width, height)
    if c is None:
        return None
    mask = np.full((height, width), 255, dtype=np.uint8)
    cv2.circle(mask, (int(round(c[0])), int(round(c[1]))), max(1, int(c[2])), 0, -1)
    return mask


_GROW = np.ones((3, 3), np.uint8)


class BallDetector:
    def __init__(self, color: dict, det: dict, ignore_mask: np.ndarray | None = None,
                 image_circle: dict | None = None, ref_size=None, far_ball_px: float = 0.0):
        self.color = dict(color)
        self.image_circle = dict(image_circle) if image_circle else None
        self.ref_size = tuple(ref_size) if ref_size else None
        self._ignore_cache = {}
        self.min_area = float(det.get("min_area_px", 10))
        # far_ball_px: how wide the ball is (in a ref_size frame) at the greatest
        # range it can be thrown from. Patches clearly smaller than that cannot
        # be the ball, and most stray patches of noise are that small.
        self.far_area = np.pi / 4.0 * (float(det.get("far_ball_margin", 0.6)) * float(far_ball_px)) ** 2
        self.max_area_frac = float(det.get("max_area_frac", 0.25))
        self.open_px = int(det.get("open_px", 3))
        self.max_aspect = float(det.get("max_aspect", 2.8))
        self.min_fill = float(det.get("min_fill", 0.5))
        self.max_fill = float(det.get("max_fill", 1.15))
        self.min_solidity = float(det.get("min_solidity", 0.8))
        self.min_ellipse_fill = float(det.get("min_ellipse_fill", 0.78))
        self.shape_min_area = float(det.get("shape_min_area_px", 80))
        self.max_candidates = int(det.get("max_candidates", 150))
        self.seed_frac = float(det.get("seed_frac", 0.15))
        self.max_blobs = int(det.get("max_blobs", 10))
        self.ignore_mask = ignore_mask  # 0/255, same size as the frame; 255 = never a ball
        self._kernel = (cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (self.open_px, self.open_px))
                        if self.open_px >= 2 else None)

    @classmethod
    def for_camera(cls, cam_cfg: dict, det: dict, ignore_mask: np.ndarray | None = None, color: dict | None = None):
        """The detector for one camera of the config (its colour profile and, for a fisheye, its lit disc)."""
        intr = cam_cfg.get("intrinsics", {})
        far_px = 0.0
        max_range = float(det.get("max_range_m", 0) or 0)
        if max_range > 0 and intr.get("width"):
            if "fx" in intr:
                focal = float(intr["fx"])
            else:
                focal = (float(intr["width"]) / 2.0) / np.tan(np.radians(float(intr.get("hfov_deg", 120.0)) / 2.0))
            far_px = float(det.get("ball_diameter_m", 0.074)) * focal / max_range
        return cls(color or cam_cfg["color"], det, ignore_mask, cam_cfg.get("image_circle"),
                   (intr.get("width", 0), intr.get("height", 0)), far_px)

    def ignore_for(self, width: int, height: int):
        """Pixels that can never be a ball in a frame of this size (0/255), or None."""
        key = (width, height)
        if key not in self._ignore_cache:
            m = circle_ignore_mask(self.image_circle, self.ref_size, width, height)
            if self.ignore_mask is not None and self.ignore_mask.shape[:2] == (height, width):
                m = self.ignore_mask.copy() if m is None else cv2.bitwise_or(m, self.ignore_mask)
            self._ignore_cache[key] = m
        return self._ignore_cache[key]

    def cover(self, bgr: np.ndarray) -> float:
        """Share of the picture that counts as ball-coloured once speckle is removed."""
        m = self.mask(bgr)
        if self._kernel is not None:
            m = cv2.morphologyEx(m, cv2.MORPH_OPEN, self._kernel)
        return float(cv2.countNonZero(m)) / float(m.size)

    def mask(self, bgr: np.ndarray) -> np.ndarray:
        m = color_mask(bgr, self.color)
        ignore = self.ignore_for(bgr.shape[1], bgr.shape[0])
        if ignore is not None:
            m[ignore > 0] = 0
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
        raw, seed = color_masks(view, self.color)
        ignore = self.ignore_for(full_w, full_h)
        if ignore is not None:
            raw[ignore[y0:y1, x0:x1] > 0] = 0
        # Components are found on the opened mask (drops speckle); each is then
        # measured on the raw mask, so opening does not shave its edge.
        work = cv2.morphologyEx(raw, cv2.MORPH_OPEN, self._kernel) if self._kernel is not None else raw
        n, labels, stats, _cent = cv2.connectedComponentsWithStats(work, connectivity=8)
        if n <= 1:
            return []
        max_area = self.max_area_frac * full_w * full_h
        scale = full_w / float(self.ref_size[0]) if self.ref_size and self.ref_size[0] else 1.0
        min_area = max(self.min_area, self.far_area * scale * scale)
        areas = stats[1:, cv2.CC_STAT_AREA]
        keep = np.nonzero((areas >= max(1.0, min_area * 0.5)) & (areas <= max_area))[0]
        # Every patch gets judged, largest first: a far ball is small, and must
        # not be crowded out by bigger patches that are not balls.
        order = keep[np.argsort(-areas[keep])][: self.max_candidates] + 1
        blobs = []
        vh, vw = raw.shape[:2]
        for i in order:
            bx, by, bw, bh, _area = (int(v) for v in stats[i])
            px0, py0 = max(0, bx - 1), max(0, by - 1)
            px1, py1 = min(vw, bx + bw + 1), min(vh, by + bh + 1)
            # this patch only: its opened core, grown back by one pixel into the raw mask
            core = (labels[py0:py1, px0:px1] == i).astype(np.uint8)
            patch = cv2.bitwise_and(cv2.dilate(core, _GROW), raw[py0:py1, px0:px1])
            a_raw = float(cv2.countNonZero(patch))
            if a_raw < min_area or a_raw > max_area:
                continue
            if seed is not None:
                strong = cv2.countNonZero(cv2.bitwise_and(patch, seed[py0:py1, px0:px1]))
                if strong < max(2.0, self.seed_frac * a_raw):
                    continue                          # nothing in it is strongly ball-coloured: noise
            if max(bw, bh) > self.max_aspect * max(1, min(bw, bh)) * 1.5:
                continue                              # far too long to be a ball, whatever its angle
            if a_raw >= 40.0 and a_raw < self.min_fill * (np.pi / 4.0 * bw * bh) * 0.6:
                continue                              # mostly empty bounding box: a streak or a web
            contours, _ = cv2.findContours(patch, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if not contours:
                continue
            hull = cv2.convexHull(max(contours, key=cv2.contourArea))
            filled = np.zeros_like(patch)
            cv2.fillConvexPoly(filled, hull, 255)
            a_hull = float(cv2.countNonZero(filled))
            if a_raw >= self.shape_min_area:
                # Big enough to judge its shape. A ball is a filled, convex,
                # round-or-oval patch. These three tests do not care how ragged
                # its edge is, which noise, dimples and a seam all make it.
                if a_raw < self.min_solidity * a_hull:
                    continue                          # dents and holes: clothing, a patch of floor
                (_rc, (rw, rh), _ang) = cv2.minAreaRect(hull)
                rw, rh = rw + 1.0, rh + 1.0           # the hull runs through pixel centres
                if max(rw, rh) > self.max_aspect * min(rw, rh):
                    continue
                oval = a_hull / (np.pi / 4.0 * rw * rh)   # 1.0 for a disc or oval, 1.27 for a box, 0.64 for a triangle
                if oval < self.min_ellipse_fill or oval > self.max_fill:
                    continue
            else:
                aspect = max(bw, bh) / float(max(1, min(bw, bh)))
                if aspect > self.max_aspect:
                    continue
                if a_raw >= 40.0 and a_raw < self.min_fill * (np.pi / 4.0 * bw * bh):
                    continue
            # Size and centre come from the filled outline, so dimples, the seam
            # and a glint on the ball do not make it look smaller.
            m = cv2.moments(filled, binaryImage=True)
            a = m["m00"]
            if a <= 0:
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
