"""Shared by the tools: find the survivor code, pick a pose backend."""
from __future__ import annotations

import os
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
for p in (ROOT, os.path.join(ROOT, "robot")):
    if p not in sys.path:
        sys.path.insert(0, p)


def make_pose(backend: str, cfg: dict, weights: str | None = None):
    """A pose model: pose(image_bgr) -> [PoseDet]. backend = "hailo" (on the robot) or "torch" (a laptop)."""
    from survivor.config import resolve
    d = cfg["detector"]
    kw = dict(score_threshold=float(d["score_threshold"]), iou_threshold=float(d["iou_threshold"]),
              max_people=int(d["max_people"]))
    if backend == "hailo":
        from hailo_pose import HailoYoloPose
        return HailoYoloPose(resolve(weights or d["model"]), **kw)
    if backend == "torch":
        from cpu_pose import TorchYoloPose
        return TorchYoloPose(weights or "yolov8s-pose.pt", **kw)
    raise SystemExit("unknown backend %r (hailo or torch)" % backend)


def default_backend() -> str:
    try:
        import hailo_platform  # noqa: F401
        return "hailo"
    except ImportError:
        return "torch"
