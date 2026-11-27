"""What is floor and what is not, by colour (after Ulrich & Nourbakhsh, "Appearance-based
obstacle detection with monocular color vision", AAAI 2000), plus one fixed colour class for
the thing that must never count as an obstacle: the ball. (Obstacles are told apart by their
shape, not by taught colours: terrain/scan.py and terrain/gauge.py. The two box labels are
kept for pictures and for anyone who wants to mark a colour.)

Everything works on HSV images as OpenCV makes them from 8-bit RGB (H 0..179, S and V 0..255).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

# label values in a classified image
FLOOR, OBSTACLE, BOX_SIDE, BOX_TOP, BALL, INVALID = 0, 1, 2, 3, 4, 255


@dataclass
class ColorRange:
    """A colour class: hue interval (degrees/2, OpenCV style 0..179, may wrap), and floors on
    saturation and value."""

    h_lo: int
    h_hi: int
    s_min: int = 80
    v_min: int = 50
    s_max: int = 255
    v_max: int = 255

    def mask(self, hsv):
        h, s, v = hsv[..., 0], hsv[..., 1], hsv[..., 2]
        in_h = (h >= self.h_lo) & (h <= self.h_hi) if self.h_lo <= self.h_hi else (h >= self.h_lo) | (h <= self.h_hi)
        return in_h & (s >= self.s_min) & (s <= self.s_max) & (v >= self.v_min) & (v <= self.v_max)

    @classmethod
    def from_dict(cls, d):
        return cls(int(d["h_lo"]), int(d["h_hi"]), int(d.get("s_min", 80)), int(d.get("v_min", 50)),
                   int(d.get("s_max", 255)), int(d.get("v_max", 255)))

    @classmethod
    def from_samples(cls, hsv_pixels, h_pad=6, s_pad=40, v_pad=50):
        """A range that covers the central 96 % of sample pixels (N, 3), padded."""
        hsv_pixels = np.asarray(hsv_pixels).reshape(-1, 3).astype(int)
        s_lo, s_hi = np.percentile(hsv_pixels[:, 1], [2, 98])
        v_lo, v_hi = np.percentile(hsv_pixels[:, 2], [2, 98])
        if s_hi < 40:      # a white or grey class: any hue, low saturation
            return cls(0, 179, 0, max(0, int(v_lo) - v_pad), min(255, int(s_hi) + s_pad), 255)
        # hue is circular: centre on the circular mean
        ang = hsv_pixels[:, 0] * (2 * np.pi / 180.0)
        mean = np.arctan2(np.sin(ang).mean(), np.cos(ang).mean()) * 180.0 / (2 * np.pi) % 180
        diff = (hsv_pixels[:, 0] - mean + 90) % 180 - 90
        lo, hi = np.percentile(diff, [2, 98])
        return cls(int(round(mean + lo - h_pad)) % 180, int(round(mean + hi + h_pad)) % 180,
                   max(0, int(s_lo) - s_pad), max(0, int(v_lo) - v_pad))

    def to_dict(self):
        return {"h_lo": self.h_lo, "h_hi": self.h_hi, "s_min": self.s_min, "v_min": self.v_min,
                "s_max": self.s_max, "v_max": self.v_max}


class FloorModel:
    """Colour histograms of the floor. Chromatic pixels are judged by hue and saturation;
    grey ones (low saturation, where hue is noise) by brightness."""

    H_BINS, S_BINS, V_BINS = 30, 8, 16
    S_GREY = 40

    def __init__(self, threshold: float = 0.004):
        self.threshold = threshold        # a bin must hold this fraction of the floor to count
        self.hs = np.zeros((self.H_BINS, self.S_BINS))
        self.v_grey = np.zeros(self.V_BINS)
        self.v_range = (0.0, 255.0)
        self.grey_fraction = 0.0
        self.trained = False

    def _bins(self, hsv):
        h = (hsv[..., 0].astype(int) * self.H_BINS) // 180
        s = (hsv[..., 1].astype(int) * self.S_BINS) // 256
        v = (hsv[..., 2].astype(int) * self.V_BINS) // 256
        return h, s, v

    def learn(self, hsv_pixels, rate: float = 1.0) -> None:
        """Take floor samples (N, 3). rate = 1 replaces the model, < 1 blends it in."""
        px = np.asarray(hsv_pixels).reshape(-1, 3)
        if len(px) < 50:
            return
        h, s, v = self._bins(px)
        grey = px[:, 1] < self.S_GREY
        hs = np.zeros_like(self.hs)
        np.add.at(hs, (h[~grey], s[~grey]), 1.0)
        vg = np.zeros_like(self.v_grey)
        np.add.at(vg, v[grey], 1.0)
        hs /= len(px)
        vg /= len(px)
        lo, hi = np.percentile(px[:, 2], [1, 99])
        if not self.trained or rate >= 1.0:
            self.hs, self.v_grey, self.v_range, self.grey_fraction = hs, vg, (float(lo), float(hi)), float(grey.mean())
        else:
            self.hs = (1 - rate) * self.hs + rate * hs
            self.v_grey = (1 - rate) * self.v_grey + rate * vg
            self.v_range = ((1 - rate) * self.v_range[0] + rate * lo, (1 - rate) * self.v_range[1] + rate * hi)
            self.grey_fraction = (1 - rate) * self.grey_fraction + rate * float(grey.mean())
        self.trained = True

    def is_floor(self, hsv):
        """Boolean mask: pixels whose colour the floor is known to have."""
        if not self.trained:
            return np.zeros(hsv.shape[:-1], dtype=bool)
        h, s, v = self._bins(hsv)
        # tolerate neighbours: one bin either way in hue (circular) and saturation
        hs = self.hs
        wide = hs.copy()
        wide = np.maximum(wide, np.roll(hs, 1, axis=0))
        wide = np.maximum(wide, np.roll(hs, -1, axis=0))
        wide[:, 1:] = np.maximum(wide[:, 1:], hs[:, :-1])
        wide[:, :-1] = np.maximum(wide[:, :-1], hs[:, 1:])
        vg = self.v_grey
        vwide = np.maximum(vg, np.maximum(np.roll(vg, 1), np.roll(vg, -1)))
        vwide[0], vwide[-1] = max(vg[0], vg[1]), max(vg[-1], vg[-2])
        grey = hsv[..., 1] < self.S_GREY
        chroma_ok = wide[h, s] >= self.threshold
        # a shadow darkens the floor without changing its colour much: allow darker, not much brighter
        v_ok = (hsv[..., 2] >= 0.45 * self.v_range[0]) & (hsv[..., 2] <= self.v_range[1] + 40)
        grey_ok = vwide[v] >= self.threshold
        return np.where(grey, grey_ok, chroma_ok & v_ok)

    def to_dict(self):
        return {"hs": self.hs.tolist(), "v_grey": self.v_grey.tolist(), "v_range": list(self.v_range),
                "grey_fraction": self.grey_fraction}

    @classmethod
    def from_dict(cls, d, threshold: float = 0.004):
        m = cls(threshold)
        m.hs = np.array(d["hs"], dtype=float)
        m.v_grey = np.array(d["v_grey"], dtype=float)
        m.v_range = tuple(d["v_range"])
        m.grey_fraction = float(d.get("grey_fraction", 0.0))
        m.trained = True
        return m


def classify(hsv, valid, floor: FloorModel, box_side: ColorRange | None = None, box_top: ColorRange | None = None,
             ball: ColorRange | None = None):
    """Label image (uint8): FLOOR, OBSTACLE, BOX_SIDE, BOX_TOP, BALL, INVALID."""
    labels = np.full(hsv.shape[:-1], OBSTACLE, dtype=np.uint8)
    labels[floor.is_floor(hsv)] = FLOOR
    if ball is not None:
        labels[ball.mask(hsv)] = BALL
    if box_top is not None:
        labels[box_top.mask(hsv) & (labels != FLOOR)] = BOX_TOP
    if box_side is not None:
        labels[box_side.mask(hsv)] = BOX_SIDE
    labels[~valid] = INVALID
    return labels
