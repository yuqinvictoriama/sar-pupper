"""Telling people apart by their clothes (plan, Pipeline step 9).

The pose gives the torso (between shoulders and hips) and the thighs (hips to knees)
directly, so no segmentation mask is needed: a hue x saturation histogram of each region is
the descriptor. Descriptors are kept per camera, because the fisheye is red-tinted and the
rear cameras are not.
"""
from __future__ import annotations

import cv2
import numpy as np

from .pose_decode import K

REGIONS = (("l_shoulder", "r_shoulder", "r_hip", "l_hip"), ("l_hip", "r_hip", "r_knee", "l_knee"))


def describe(image_bgr: np.ndarray, kpts: np.ndarray, conf_thr: float, bins=(12, 6)) -> np.ndarray | None:
    """Histogram descriptor of the person in this image, or None when the torso is not visible."""
    h, w = image_bgr.shape[:2]
    parts = []
    for i, names in enumerate(REGIONS):
        idx = [K[n] for n in names]
        if not all(kpts[j, 2] >= conf_thr for j in idx):
            if i == 0:
                return None
            parts.append(np.zeros(bins[0] * bins[1], np.float32))
            continue
        poly = np.round(kpts[idx, :2]).astype(np.int32)
        x1, y1 = np.clip(poly.min(axis=0), 0, [w - 1, h - 1])
        x2, y2 = np.clip(poly.max(axis=0) + 1, 1, [w, h])
        if x2 - x1 < 2 or y2 - y1 < 2:
            if i == 0:
                return None
            parts.append(np.zeros(bins[0] * bins[1], np.float32))
            continue
        mask = np.zeros((y2 - y1, x2 - x1), np.uint8)
        cv2.fillConvexPoly(mask, cv2.convexHull(poly - [x1, y1]), 255)
        hsv = cv2.cvtColor(np.ascontiguousarray(image_bgr[y1:y2, x1:x2]), cv2.COLOR_BGR2HSV)
        hist = cv2.calcHist([hsv], [0, 1], mask, list(bins), [0, 180, 0, 256]).astype(np.float32)
        # Spread each bin into its neighbours: a colour on a bin edge (or red, which wraps round
        # the hue circle) must not look like two different colours from one frame to the next.
        hist = 0.5 * hist + 0.25 * (np.roll(hist, 1, axis=0) + np.roll(hist, -1, axis=0))
        pad = np.pad(hist, ((0, 0), (1, 1)), mode="edge")
        hist = (0.5 * hist + 0.25 * (pad[:, :-2] + pad[:, 2:])).ravel()
        total = float(hist.sum())
        parts.append(hist / total if total > 0 else hist)
    return np.concatenate(parts)


def distance(a: np.ndarray | None, b: np.ndarray | None) -> float | None:
    """0 = same colours, 1 = nothing in common. None when either is missing (no evidence)."""
    if a is None or b is None:
        return None
    n = len(a) // 2
    out, used = 0.0, 0
    for sl in (slice(0, n), slice(n, 2 * n)):
        if a[sl].sum() <= 0 or b[sl].sum() <= 0:
            continue
        out += 1.0 - float(np.sqrt(a[sl] * b[sl]).sum())        # 1 - Bhattacharyya coefficient
        used += 1
    return None if not used else out / used


def blend(old: np.ndarray | None, new: np.ndarray | None, rate: float) -> np.ndarray | None:
    if new is None:
        return old
    if old is None:
        return new.copy()
    return (1.0 - rate) * old + rate * new
