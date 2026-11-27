"""One camera's share of the work that needs the picture itself: find the ball in a frame.

    pipe = FramePipeline("elp_left", cfg)
    pipe.set_hint(core.track_hint())           # where a tracked ball is, whenever that changes
    result = pipe.process(t_capture, bgr)      # -> FrameResult with the blobs

Everything after this (ranging, tracking, the decision) needs only the blobs, a
few numbers per frame. That is what lets each camera do this part in a process
of its own on the robot (robot/camera_worker.py) while one small process keeps
the track and decides.
"""
from __future__ import annotations

import os
import time
from dataclasses import dataclass, field

from .camera_models import build_cameras
from .config import ROOT
from .core import predict_pixel
from .detect import BallDetector


@dataclass
class FrameResult:
    cam: str
    t: float                 # time of the picture, corrected by the camera's time_offset_s
    raw: float               # the camera's own stamp
    size: tuple              # (width, height) of the picture
    blobs: list = field(default_factory=list)
    roi: bool = False        # found by looking only near the tracked ball
    detect_ms: float = 0.0   # time spent looking for the ball
    decode_ms: float = 0.0   # time spent turning the camera's JPEG into a picture
    start: float = 0.0       # wall clock when the search began (start - raw = how old the picture already was)
    done: float = 0.0        # wall clock when the search ended


def load_mask(cam_cfg: dict, warn=None):
    """The camera's static ignore mask (robot parts in view), or None."""
    mask_file = cam_cfg.get("mask_file")
    if not mask_file:
        return None
    import cv2
    path = mask_file if os.path.isabs(mask_file) else os.path.join(ROOT, mask_file)
    mask = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if mask is None and warn is not None:
        warn("mask file %s could not be read" % path)
    return mask


class FramePipeline:
    def __init__(self, name: str, cfg: dict, mask=None):
        self.name = name
        cam = cfg["cameras"][name]
        self.det_cfg = cfg["detector"]
        self.detector = BallDetector.for_camera(cam, self.det_cfg, mask)
        self.camera = build_cameras({"cameras": {name: dict(cam, enabled=True)}})[name]
        self._scaled: dict = {}
        self.time_offset = float(cam.get("time_offset_s", 0.0))
        self.roi_tracking = bool(self.det_cfg.get("roi_tracking", True))
        self.full_every = max(1, int(self.det_cfg.get("full_frame_every", 3)))
        self.frame_no = 0
        self.hint = None

    def set_hint(self, hint) -> None:
        """hint: DodgeCore.track_hint(), or None when nothing is tracked."""
        self.hint = hint

    def camera_for(self, size):
        if (int(size[0]), int(size[1])) == (self.camera.model.width, self.camera.model.height):
            return self.camera
        key = (int(size[0]), int(size[1]))
        if key not in self._scaled:
            self._scaled[key] = self.camera.with_resolution(*key)
        return self._scaled[key]

    def process(self, t_capture: float, bgr, decode_ms: float = 0.0) -> FrameResult:
        t0 = time.time()
        t = t_capture + self.time_offset
        size = (bgr.shape[1], bgr.shape[0])
        self.frame_no += 1
        blobs, roi = None, False
        hint = self.hint
        if self.roi_tracking and hint is not None and self.frame_no % self.full_every != 0:
            pred = predict_pixel(self.camera_for(size), hint, t, self.det_cfg)
            if pred is not None:
                u, v, half = pred
                blobs = self.detector.detect(bgr, roi=(u - half, v - half, u + half, v + half))
                roi = True
                if not blobs:
                    blobs, roi = None, False      # not where expected: look everywhere
        if blobs is None:
            blobs = self.detector.detect(bgr)
        done = time.time()
        return FrameResult(self.name, t, t_capture, size, blobs, roi, (done - t0) * 1000.0, decode_ms, t0, done)
