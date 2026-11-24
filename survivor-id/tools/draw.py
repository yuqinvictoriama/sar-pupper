"""Drawing people, skeletons and the gesture verdict on a picture (tools and the node's debug image)."""
from __future__ import annotations

import cv2
import numpy as np

import _common  # noqa: F401
from survivor.pose_decode import SKELETON

GREEN, AMBER, GREY, RED, WHITE = (60, 220, 60), (0, 190, 255), (170, 170, 170), (60, 60, 240), (255, 255, 255)


def person(img: np.ndarray, obs, res=None, label: str = "", survivor: bool = False, conf_thr: float = 0.4,
           show_range: bool = True) -> None:
    """obs: PersonObs (its box and keypoints are in this image's pixels). res: GestureResult or None."""
    colour = RED if survivor else (GREEN if (res is not None and res.up) else (AMBER if (res is not None and res.maybe) else GREY))
    x1, y1, x2, y2 = (int(round(v)) for v in obs.box)
    cv2.rectangle(img, (x1, y1), (x2, y2), colour, 3 if survivor else 2)
    k = obs.kpts
    for a, b in SKELETON:
        if k[a, 2] >= conf_thr and k[b, 2] >= conf_thr:
            cv2.line(img, (int(k[a, 0]), int(k[a, 1])), (int(k[b, 0]), int(k[b, 1])), colour, 2, cv2.LINE_AA)
    for x, y, c in k:
        if c >= conf_thr:
            cv2.circle(img, (int(x), int(y)), 3, WHITE, -1, cv2.LINE_AA)
    lines = [label] if label else []
    if res is not None:
        lines.append("ARMS UP" if res.up else (res.why or "not the signal"))
    if show_range and obs.range_m is not None:
        lines.append("%.1f m" % obs.ground_range)
    for i, text in enumerate(lines):
        org = (x1 + 3, max(14, y1 - 6) + 16 * i)
        cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.45, colour, 1, cv2.LINE_AA)


def banner(img: np.ndarray, text: str) -> None:
    cv2.rectangle(img, (0, 0), (img.shape[1], 22), (0, 0, 0), -1)
    cv2.putText(img, text, (6, 16), cv2.FONT_HERSHEY_SIMPLEX, 0.5, WHITE, 1, cv2.LINE_AA)
