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

import math
import os
import time
from dataclasses import dataclass, field

import cv2
import numpy as np

from .camera_models import build_cameras
from .config import ROOT
from .core import predict_pixel
from .detect import BallDetector


_REDUCED = {2: cv2.IMREAD_REDUCED_COLOR_2, 4: cv2.IMREAD_REDUCED_COLOR_4, 8: cv2.IMREAD_REDUCED_COLOR_8}


@dataclass
class FrameResult:
    cam: str
    t: float                 # time of the picture, corrected by the camera's time_offset_s
    raw: float               # the camera's own stamp
    size: tuple              # (width, height) of the picture
    blobs: list = field(default_factory=list)
    roi: bool = False        # found by looking only near the tracked ball
    still: int = 0           # blobs left out because they have not moved for a while (see StillFilter)
    detect_ms: float = 0.0   # time spent looking for the ball
    decode_ms: float = 0.0   # time spent turning the camera's JPEG into a picture
    start: float = 0.0       # wall clock when the search began (start - raw = how old the picture already was)
    done: float = 0.0        # wall clock when the search ended


def load_mask(cam_cfg: dict, warn=None):
    """The camera's static ignore mask (robot parts in view), or None."""
    mask_file = cam_cfg.get("mask_file")
    if not mask_file:
        return None
    path = mask_file if os.path.isabs(mask_file) else os.path.join(ROOT, mask_file)
    mask = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
    if mask is None and warn is not None:
        warn("mask file %s could not be read" % path)
    return mask


class StillFilter:
    """Leaves out blobs that have sat in one place in the picture for a while: they are not thrown balls.

    A room has still things of the ball's colour in it, above all the balls already thrown. Each
    flickers a little from picture to picture, and to the tracker a flickering blob among others is
    easily a "moving" one: on the Pupper (2026-11-26) a bowl of balls and a rolled-up mat produced a
    false "throw seen" every few seconds and a few predicted hits, with nobody throwing.

    Two rules. The first is for a thing that sits in one spot (a ball on the floor); the second for a
    thing whose colour shows in bits that come and go (a drying rack, a patterned mat): the picture is
    divided into squares of cell_px, and a square that has had a blob in it for most of the last few
    seconds (busy_level of the time, averaged over busy_s) reports no more blobs, unless one is clearly
    bigger than what that square usually shows. A ball flying through such a square is not seen there;
    a ball coming straight at the camera is in its square for under a second, too short to count.

    A blob is still when something has been seen at its place for after_s seconds and, over the last
    window_s, it has neither moved (more than move_px or move_diameters of its own width) nor grown
    (by more than grow: a ball coming straight at the camera stays in one place but swells; it must
    swell beyond the biggest the place has shown before, because a still ball in poor light is found
    whole in one picture and half in the next). A thrown
    ball is never still by this measure, and neither is anything while the robot turns or walks past
    it; those are passed on as before and the tracker judges them.
    """

    def __init__(self, cfg: dict | None = None, scale: float = 1.0):
        """scale: how many times smaller than the camera's own pictures the searched ones are
        (cameras.<name>.search_scale); the pixel distances below are for the camera's own."""
        cfg = cfg or {}
        self.enabled = bool(cfg.get("enabled", True))
        self.after = float(cfg.get("after_s", 1.0))
        self.window = float(cfg.get("window_s", 0.5))
        self.move_px = float(cfg.get("move_px", 6.0)) / scale
        self.move_d = float(cfg.get("move_diameters", 0.5))
        self.grow = float(cfg.get("grow", 1.2))
        self.forget = float(cfg.get("forget_s", 1.5))
        self.max_places = int(cfg.get("max_places", 60))
        self.places: list = []       # each: {"t0", "seen", "u", "v", "d", "hist": [(t, u, v, d), ...]}
        self.cell = float(cfg.get("cell_px", 48.0)) / scale
        self.busy_s = float(cfg.get("busy_s", 2.0))
        self.busy_level = float(cfg.get("busy_level", 0.3))        # 0 = this rule off
        self.busy_bigger = float(cfg.get("busy_bigger", 1.6))
        self.cells: dict = {}        # (column, row) -> [share of recent time with a blob in it, usual blob width]
        self.t_prev = None

    def filter(self, t: float, blobs: list):
        """(blobs worth passing on, number left out)."""
        if not self.enabled:
            return blobs, 0
        self.places = [p for p in self.places if 0.0 <= t - p["seen"] <= self.forget]
        # ---- squares of the picture that keep showing colour
        busy = set()
        if self.busy_level > 0:
            dt = 0.0 if self.t_prev is None else min(0.5, max(0.0, t - self.t_prev))
            self.t_prev = t
            fade = math.exp(-dt / self.busy_s)
            hit = {}
            for b in blobs:
                hit.setdefault((int(b.u // self.cell), int(b.v // self.cell)), []).append(float(b.diameter_px))
            busy = {key for key, c in self.cells.items() if c[0] >= self.busy_level}
            for key in list(self.cells):
                c = self.cells[key]
                c[0] *= fade
                if c[0] < 0.02 and key not in hit:
                    del self.cells[key]
            for key, widths in hit.items():
                c = self.cells.setdefault(key, [0.0, min(widths)])
                c[0] += 1.0 - fade
                c[1] += 0.2 * (min(widths) - c[1])
        kept, used = [], set()
        for b in blobs:
            d = float(b.diameter_px)
            key = (int(b.u // self.cell), int(b.v // self.cell))
            if key in busy and d < self.busy_bigger * self.cells[key][1] if key in self.cells else False:
                continue                           # one more bit of colour where there always is some
            best, best_dist = None, None
            for i, p in enumerate(self.places):
                if i in used or not (0.6 * p["d"] <= d <= 1.7 * p["d"]):
                    continue                       # a different size is a different thing (a ball passing in front)
                dist = math.hypot(b.u - p["u"], b.v - p["v"])
                if dist <= max(self.move_px, 0.8 * max(d, p["d"])) and (best is None or dist < best_dist):
                    best, best_dist = i, dist
            if best is None:
                if len(self.places) < self.max_places:
                    self.places.append({"t0": t, "seen": t, "u": b.u, "v": b.v, "d": d, "big": 0.0, "hist": [(t, b.u, b.v, d)]})
                    used.add(len(self.places) - 1)
                kept.append(b)
                continue
            used.add(best)
            p = self.places[best]
            gone = [h[3] for h in p["hist"] if t - h[0] > self.window + 0.25]
            if gone:
                p["big"] = max(p["big"], max(gone))      # the biggest this place has shown, up to a window ago
            hist = [h for h in p["hist"] if t - h[0] <= self.window + 0.25]
            hist.append((t, b.u, b.v, d))
            p.update(seen=t, u=b.u, v=b.v, d=d, hist=hist)
            old = hist[0]
            for h in hist:                         # the entry nearest to one window ago
                if t - h[0] >= self.window:
                    old = h
            waited = (t - p["t0"]) >= self.after and (t - old[0]) >= 0.8 * self.window
            moved = math.hypot(b.u - old[1], b.v - old[2]) > max(self.move_px, self.move_d * d)
            grown = d > self.grow * max(old[3], p["big"])
            if waited and not moved and not grown:
                continue                           # still: leave it out
            kept.append(b)
        return kept, len(blobs) - len(kept)


class FramePipeline:
    def __init__(self, name: str, cfg: dict, mask=None):
        self.name = name
        cam = cfg["cameras"][name]
        self.det_cfg = cfg["detector"]
        # search_scale 2: every JPEG is decoded at half size and searched at that size, never in full. A
        # quarter of the pixels to decode and search, for a ball half as many pixels across. The blobs then
        # come in that picture's pixels, and FrameResult.size says which size that is. The detector's sizes
        # in pixels stay as they are: scaled down with the picture (shape tests from 20 px up), the shape
        # tests turned the ball itself away, and the replay of 2026-11-27 was no better for it.
        self.search_scale = int(cam.get("search_scale", 1) or 1)
        if self.search_scale not in (1, 2, 4, 8):
            raise ValueError("%s: search_scale must be 1, 2, 4 or 8, not %r" % (name, cam.get("search_scale")))
        s = float(self.search_scale)
        self.detector = BallDetector.for_camera(cam, self.det_cfg, mask)
        self.camera = build_cameras({"cameras": {name: dict(cam, enabled=True)}})[name]
        self._scaled: dict = {}
        self._roi_cfg = dict(self.det_cfg, roi_min_px=float(self.det_cfg.get("roi_min_px", 96)) / s)
        self.time_offset = float(cam.get("time_offset_s", 0.0))
        self.roi_tracking = bool(self.det_cfg.get("roi_tracking", True))
        self.full_every = max(1, int(self.det_cfg.get("full_frame_every", 3)))
        self.frame_no = 0
        self.hint = None
        self.still = StillFilter(self.det_cfg.get("still"), s)
        self.two_stage = bool(self.det_cfg.get("decode_small_first", True))
        self.brightness = -1.0       # mean grey level of a recent picture (0-255), -1 before the first
        self.hot_frames = int(self.det_cfg.get("full_decode_frames", 6))
        self.max_measured = int(self.det_cfg.get("max_measured", 12))
        self._hot = 0                # pictures still to be decoded in full straight away

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

    def process_jpeg(self, t_capture: float, data) -> FrameResult:
        """One picture, still as the camera's JPEG (bytes or a uint8 array).

        Decoding and searching a full picture is most of the work there is (20-30 ms of a 1280 x 720
        one on the Pupper), and most pictures show no ball. So the JPEG is first decoded at
        1/coarse_step size, which costs about 60 % of a full decode (measured on the Pi: 6.7 against
        10.6 ms), and that small picture is searched for ball colour, which costs little. Only if it
        shows something that has not been sitting there all along is the full picture decoded and
        the thing measured. Once something is in view, and while a ball that is closing in is tracked,
        the following pictures are decoded in full straight away, so only the first costs both.
        """
        det = self.detector
        data = np.frombuffer(data, dtype=np.uint8) if isinstance(data, (bytes, bytearray, memoryview)) else data
        full_w, full_h = self.camera.model.width, self.camera.model.height
        t0 = time.time()
        if self.search_scale > 1:
            s = self.search_scale
            bgr = cv2.imdecode(data, _REDUCED[s])
            if bgr is None:
                raise ValueError("%s: a picture that will not decode" % self.name)
            if bgr.shape[1] != (full_w + s - 1) // s or bgr.shape[0] != (full_h + s - 1) // s:
                bgr = cv2.imdecode(data, cv2.IMREAD_COLOR)   # not the size this camera is set up for: as it comes
            return self.process(t_capture, bgr, (time.time() - t0) * 1000.0)
        flag = _REDUCED.get(det.coarse_step)
        step = det.coarse_step
        small = None
        if self._hot > 0:
            self._hot -= 1
        elif flag is not None and self.two_stage and not (self.roi_tracking and self.hint is not None):
            small = cv2.imdecode(data, flag)
            if small is None or small.shape[1] != (full_w + step - 1) // step or small.shape[0] != (full_h + step - 1) // step:
                small = None                        # not the size this camera is set up for: the plain way
        if small is None:
            bgr = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if bgr is None:
                raise ValueError("%s: a picture that will not decode" % self.name)
            return self.process(t_capture, bgr, (time.time() - t0) * 1000.0)
        decode = [time.time() - t0]

        def full():
            t2 = time.time()
            bgr = cv2.imdecode(data, cv2.IMREAD_COLOR)
            decode[0] += time.time() - t2
            return bgr
        self.frame_no += 1
        self._look(small)
        t = t_capture + self.time_offset
        blobs, n_still = self._search(t, small, full)
        done = time.time()
        return FrameResult(self.name, t, t_capture, (full_w, full_h), blobs, False, n_still,
                           (done - t0 - decode[0]) * 1000.0, decode[0] * 1000.0, t0, done)

    def _search(self, t: float, small, full):
        """The whole-picture search: (blobs to pass on, number left out as still).

        small is the picture at 1/coarse_step size; full() returns the full picture, and is only called if
        the small one shows something worth measuring.
        """
        det = self.detector
        full_w, full_h = self.camera.model.width, self.camera.model.height
        candidates = det.coarse_candidates(small, full_w, full_h)
        if candidates is None:                      # colour all over the picture: search all of it directly
            blobs, n_still = self.still.filter(t, det.detect(full()))
        else:
            moving, n_still = self.still.filter(t, candidates)
            if moving:
                self._hot = self.hot_frames         # something is there: the next pictures in full straight away
            # Measuring a place at full size costs about a millisecond on the Pi. In a speckled picture
            # only the places with most colour are measured: a ball near enough to matter is among them.
            blobs = det.measure_candidates(full(), moving[: self.max_measured]) if moving else []
        if blobs:
            self._hot = self.hot_frames
        return blobs, n_still

    def _look(self, picture) -> None:
        """Remember how bright the pictures are (a camera that is too dark finds nothing)."""
        if self.frame_no % 30 == 0:
            self.brightness = float(picture[::8, ::8].mean())

    def process(self, t_capture: float, bgr, decode_ms: float = 0.0) -> FrameResult:
        """One picture, already decoded."""
        t0 = time.time()
        t = t_capture + self.time_offset
        size = (bgr.shape[1], bgr.shape[0])
        det = self.detector
        self._look(bgr)
        self.frame_no += 1
        blobs, roi, n_still = None, False, 0
        hint = self.hint
        if self.roi_tracking and hint is not None and self.frame_no % self.full_every != 0:
            pred = predict_pixel(self.camera_for(size), hint, t, self._roi_cfg if self.search_scale > 1 else self.det_cfg)
            if pred is not None:
                u, v, half = pred
                blobs = det.detect(bgr, roi=(u - half, v - half, u + half, v + half))
                roi = True
                if not blobs:
                    blobs, roi = None, False      # not where expected: look everywhere
        if blobs is None:
            step = det.coarse_step
            if step > 1 and size == (self.camera.model.width, self.camera.model.height):
                small = cv2.resize(bgr, ((size[0] + step - 1) // step, (size[1] + step - 1) // step),
                                   interpolation=cv2.INTER_NEAREST)
                blobs, n_still = self._search(t, small, lambda: bgr)
            else:                                 # a picture of another size than the camera is set up for
                blobs, n_still = self.still.filter(t, det.detect(bgr))
        done = time.time()
        return FrameResult(self.name, t, t_capture, size, blobs, roi, n_still, (done - t0) * 1000.0, decode_ms, t0, done)
