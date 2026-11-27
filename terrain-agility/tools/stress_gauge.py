#!/usr/bin/env python3
"""How much worse can the picture get before the gauge stops reading the slab? Real camera
frames from the Pupper (glossy wooden floor, 26 Nov 2026) are degraded step by step - glare,
blur, exposure, contrast, colour cast, noise, JPEG - and measured again.

    python3 tools/stress_gauge.py                         # the frames in tests/real_wood_floor
    python3 tools/stress_gauge.py --frames data/slab --clear data/floor --floor config/floor_model.json
    python3 tools/stress_gauge.py --pose yaml             # read with the pose in config/terrain.yaml instead

Laptop or robot; nothing moves. For each kind of degradation it prints, level by level, what
the gauge reads from the slab frames (height, depth, distance, share of bearings with a side
face) and what that reading would be taken for, and on the clear-floor frames whether the
walking scan now sees an obstacle in the lane that is not there. The first level at which
anything changes is the breaking point.

"Taken for" is the decision on the slab's shape alone: the distance and frame-count checks are
set aside (the frames were taken 0.55 m from it, farther than a crossing starts from, and
there are six of them), so it answers "would it still be scaled across from the right place".

The floor model is kept as it was learned (the node also adapts it slowly to the floor just
ahead, which this does not do): a change of exposure is judged against the old floor colours.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tools"))

import cv2  # noqa: E402

import _frames  # noqa: E402
from terrain import decision as D  # noqa: E402
from terrain import floor_model as FM  # noqa: E402
from terrain.camera_model import CameraPose, PolarView  # noqa: E402
from terrain.config import TerrainConfig  # noqa: E402
from terrain.gauge import BoxGauge, fine_view, measure_obstacle  # noqa: E402
from terrain.scan import build_scan, find_obstacles  # noqa: E402

DATA = ROOT / "tests" / "real_wood_floor"
NOMINAL = CameraPose(height=0.165, pitch_deg=-11.0, x_forward=0.150)   # what these frames showed (not a calibration)


# ---------------------------------------------------------------------------- degradations (BGR uint8 in and out)
def _f(img):
    return img.astype(np.float32)


def _u8(img):
    return np.clip(img, 0, 255).astype(np.uint8)


def glare(img, level, where, rng):
    """A specular highlight: a bright, slightly warm blob ``level`` (0..1) strong, stretched
    along the image's vertical as the reflection of a ceiling light on a glossy floor is."""
    h, w = img.shape[:2]
    u0, v0, size = where
    v, u = np.mgrid[0:h, 0:w]
    g = np.exp(-(((u - u0) / size) ** 2 + ((v - v0) / (2.5 * size)) ** 2))
    tint = np.array([0.85, 0.95, 1.0], np.float32)            # BGR: warm white
    return _u8(_f(img) + level * 255.0 * g[..., None] * tint)


def blur(img, sigma, *_):
    return img if sigma <= 0 else cv2.GaussianBlur(img, (0, 0), sigma)


def motion_blur(img, length, *_):
    """Vertical smear, as the body bobbing while it settles would give."""
    n = int(length)
    if n <= 1:
        return img
    k = np.zeros((n, n), np.float32)
    k[:, n // 2] = 1.0 / n
    return cv2.filter2D(img, -1, k)


def exposure(img, gain, *_):
    return _u8(_f(img) * gain)


def offset(img, add, *_):
    return _u8(_f(img) + add)


def contrast(img, c, *_):
    m = _f(img).mean(axis=(0, 1), keepdims=True)
    return _u8(m + (_f(img) - m) * c)


def gamma(img, g, *_):
    return _u8(255.0 * (_f(img) / 255.0) ** g)


def red_cast(img, gain, *_):
    """White balance drift: the red channel's gain changed (the course fixes it at [1.4, 3.3])."""
    out = _f(img)
    out[..., 2] *= gain
    return _u8(out)


def blue_cast(img, gain, *_):
    out = _f(img)
    out[..., 0] *= gain
    return _u8(out)


def noise(img, sigma, _where, rng):
    return _u8(_f(img) + rng.normal(0, sigma, img.shape))


def jpeg(img, quality, *_):
    ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, int(quality)])
    return cv2.imdecode(buf, cv2.IMREAD_COLOR)


# name -> (function, levels from mild to severe, the undisturbed level, where it acts)
def degradations():
    return {
        "glare at the slab's foot": (glare, [0.1, 0.2, 0.3, 0.35, 0.4, 0.45, 0.6, 0.8, 1.0], 0.0, "foot"),
        "glare on the slab's top": (glare, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0], 0.0, "top"),
        "glare on the floor beyond": (glare, [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0], 0.0, "beyond"),
        "glare on the floor in front": (glare, [0.1, 0.2, 0.3, 0.35, 0.4, 0.45, 0.6, 0.8, 1.0], 0.0, "front"),
        "gaussian blur (px)": (blur, [0.5, 1.0, 1.5, 2.0, 2.5, 3.0, 4.0, 6.0], 0.0, None),
        "vertical motion blur (px)": (motion_blur, [3, 5, 7, 9, 11, 13, 17, 25], 1, None),
        "exposure gain": (exposure, [0.3, 0.4, 0.5, 0.55, 0.6, 0.7, 0.8, 0.9, 1.1, 1.15, 1.2, 1.25, 1.3, 1.5, 1.8], 1.0, None),
        "brightness offset": (offset, [-60, -50, -40, -25, -15, 15, 25, 30, 35, 40, 60], 0.0, None),
        "contrast": (contrast, [0.4, 0.55, 0.6, 0.65, 0.7, 0.85, 1.1, 1.2, 1.3, 1.4, 1.6], 1.0, None),
        "gamma": (gamma, [0.5, 0.65, 0.7, 0.75, 0.8, 0.9, 1.1, 1.25, 1.4, 1.5, 2.0], 1.0, None),
        "red gain (white balance)": (red_cast, [0.7, 0.8, 0.9, 0.95, 1.05, 1.1, 1.2, 1.3, 1.35], 1.0, None),
        "blue gain (white balance)": (blue_cast, [0.7, 0.8, 0.85, 0.9, 0.95, 1.05, 1.1, 1.2, 1.35], 1.0, None),
        "sensor noise (std)": (noise, [3, 5, 6, 8, 10, 15, 20, 30, 40], 0.0, None),
        "JPEG quality": (jpeg, [70, 50, 30, 20, 15, 10, 5], 95, None),
    }


# ---------------------------------------------------------------------------- reading
class Reader:
    def __init__(self, cfg, pose, floor):
        self.cfg, self.pose, self.floor = cfg, pose, floor
        self.view = None

    def gauge(self, frames):
        """-> (BoxEstimate or None, per-frame measurements)"""
        cfg = self.cfg
        g = BoxGauge(len(frames))
        ms = []
        for bgr, R in frames:
            if self.view is None:
                self.view = fine_view(cfg.model.scaled(bgr.shape[1], bgr.shape[0]), self.pose)
            m = measure_obstacle(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), self.view, self.floor, R_level_body=R, cfg=cfg.gauge,
                                 ball=cfg.ball, near_depression_deg=cfg.near_depression_deg, landing=cfg.decision.landing_clear)
            ms.append(m)
            g.add(m)
        return g.estimate(), ms

    def taken_for(self, est):
        """The decision on its shape alone (distance and frame count set aside)."""
        if est is None:
            return D.decide(None, self.cfg.decision)
        return D.decide(replace(est, near_x=0.36, frames=max(est.frames, self.cfg.decision.min_frames)), self.cfg.decision)

    def nearest_ahead(self, bgr, R, half_width=0.15, max_bearing=60.0):
        """m to the nearest obstacle point the walking scan puts in the lane straight ahead
        (|y| < half_width, bearing within max_bearing, nearer than the engage range), or inf."""
        best = float("inf")
        for o in self.lane_obstacles(bgr, R):
            p = o.points
            b = np.degrees(np.arctan2(p[:, 1], p[:, 0] - self.pose.x_forward))
            ok = (np.abs(p[:, 1]) < half_width) & (p[:, 0] > 0) & (np.abs(b) < max_bearing)
            if ok.any():
                best = min(best, float(p[ok, 0].min()))
        return best

    def lane_obstacles(self, bgr, R):
        """Obstacles the walking scan puts in the lane within the engage range (body frame)."""
        cfg = self.cfg
        view = PolarView(cfg.model.scaled(bgr.shape[1], bgr.shape[0]), self.pose)
        pol, valid = view.sample(bgr, R)
        labels = FM.classify(cv2.cvtColor(pol, cv2.COLOR_BGR2HSV), valid, self.floor, ball=cfg.ball)
        obs = find_obstacles(build_scan(labels, view, cfg.scan), cfg.scan)
        out = []
        for o in obs:
            if o.points is None or not len(o.points):
                continue
            near_lane = o.points[np.abs(o.points[:, 1]) < cfg.fsm.corridor_half]
            if len(near_lane) and float(near_lane[:, 0].min()) < cfg.fsm.engage_range:
                out.append(o)
        return out


def learn_floor(cfg, pose, frames, min_dep=20.0):
    """The floor's colour from these frames, as tools/learn_colors.py floor learns it (rows from
    ``min_dep`` down to the chin, bearings within 35 deg)."""
    pixels = []
    for bgr, R in frames:
        view = PolarView(cfg.model.scaled(bgr.shape[1], bgr.shape[0]), pose)
        pol, _ = view.sample(bgr, R)
        hsv = cv2.cvtColor(pol, cv2.COLOR_BGR2HSV)
        rows = (view.depressions > min_dep) & (view.depressions < cfg.near_depression_deg - 2.0)
        pixels.append(hsv[np.ix_(rows, np.abs(view.bearings) < 35.0)].reshape(-1, 3))
    fm = FM.FloorModel()
    fm.learn(np.concatenate(pixels))
    return fm


def _where(reader, est, shape, kind):
    """(u, v, size px) in the raw image of a point on the floor or the slab, straight ahead."""
    near = est.near_x if est is not None and math.isfinite(est.near_x) else 0.55
    depth = est.depth if est is not None and math.isfinite(est.depth) else 0.38
    h = est.height if est is not None and math.isfinite(est.height) else 0.015
    x, z = {"foot": (near - 0.012, 0.0), "top": (near + 0.5 * depth, h), "beyond": (near + depth + 0.12, 0.0),
            "front": (near - 0.15, 0.0)}[kind]
    pose = reader.pose
    p_body = np.array([x - pose.x_forward, -pose.y_left, z - pose.height])
    ray = pose.rotation().T @ p_body
    model = reader.cfg.model.scaled(shape[1], shape[0])
    u, v, ok = model.project(ray[None, :])
    return float(u[0]), float(v[0]), 18.0


def fmt(est, verdict):
    if est is None:
        return "nothing measured".ljust(58) + f" -> {verdict.action}"
    mm = lambda v: "  ?" if not math.isfinite(v) else f"{v * 1000:4.0f}"      # noqa: E731
    return (f"h {mm(est.height)} mm  d {mm(est.depth)} mm  at {est.near_x * 1000:4.0f} mm  side {est.side_seen:4.2f}  "
            f"skew {math.degrees(est.skew):+5.1f}  -> {verdict.action}")


def changed(base, est, verdict, base_verdict, tol_h, tol_d, tol_x):
    """Why this reading counts as broken, or ''."""
    if verdict.action != base_verdict.action:
        return f"now {verdict.action}" + (f" ({verdict.reasons[0][:70]})" if verdict.reasons else "")
    if est is None:
        return "nothing measured"
    if abs(est.height - base.height) > tol_h:
        return f"height off by {(est.height - base.height) * 1000:+.1f} mm"
    if abs(est.depth - base.depth) > tol_d:
        return f"depth off by {(est.depth - base.depth) * 1000:+.0f} mm"
    if abs(est.near_x - base.near_x) > tol_x:
        return f"distance off by {(est.near_x - base.near_x) * 1000:+.0f} mm"
    return ""


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--frames", default=str(DATA / "slab"), help="frames with the slab ahead (tools/record_frames.py)")
    ap.add_argument("--clear", default=str(DATA / "clear"), help="frames of clear floor ('' to skip)")
    ap.add_argument("--floor", default=str(DATA / "floor_model.json"), help="the floor model those frames go with")
    ap.add_argument("--pose", choices=("nominal", "yaml"), default="nominal",
                    help="camera pose to read with: 165 mm, 11 deg up (what these frames showed) or config/terrain.yaml's")
    ap.add_argument("--tol-height", type=float, default=0.003, help="m of height change that counts as broken")
    ap.add_argument("--tol-depth", type=float, default=0.030)
    ap.add_argument("--tol-near", type=float, default=0.015)
    ap.add_argument("--relearn", action="store_true",
                    help="teach the floor again at every level from the degraded frames themselves (the clear-floor ones "
                         "and the floor in front of the slab, from 25 deg down), as re-running learn_colors.py floor "
                         "there under that light would, instead of keeping the saved floor model")
    ap.add_argument("--only", help="run only degradations whose name contains this")
    ap.add_argument("--json", help="write every reading here")
    ap.add_argument("--seed", type=int, default=0)
    args = ap.parse_args()

    cfg = TerrainConfig()
    pose = NOMINAL if args.pose == "nominal" else cfg.pose
    floor = FM.FloorModel.from_dict(json.loads(Path(args.floor).read_text()))
    reader = Reader(cfg, pose, floor)
    slab = [(bgr, R) for bgr, R, _ in _frames.load(args.frames)]
    clear = [(bgr, R) for bgr, R, _ in _frames.load(args.clear)] if args.clear else []
    rng = np.random.default_rng(args.seed)
    if args.relearn:
        if not clear:
            raise SystemExit("--relearn needs clear-floor frames (--clear)")
        reader.floor = learn_floor(cfg, pose, clear + slab, 25.0)
        print("floor taught from the frames themselves (and again at every level)")

    base, _ = reader.gauge(slab)
    base_verdict = reader.taken_for(base)
    print(f"camera pose read with: {pose}")
    print(f"undisturbed ({len(slab)} slab frames): {fmt(base, base_verdict)}")
    base_lane = [reader.nearest_ahead(b, R) for b, R in clear]
    if clear:
        print(f"undisturbed ({len(clear)} clear-floor frames): nearest thing in the lane ahead (m) per frame: "
              f"{[round(v, 2) for v in base_lane]}")
    if base is None or not math.isfinite(base.height):
        print("the undisturbed frames give no height: nothing to degrade")
    rows, breaks = [], {}
    for name, (fn, levels, _neutral, where) in degradations().items():
        if args.only and args.only not in name:
            continue
        print(f"\n{name}")
        pos = _where(reader, base, slab[0][0].shape, where) if where else None
        for lvl in levels:
            if args.relearn:
                reader.floor = learn_floor(cfg, pose, [(fn(b, lvl, pos, rng), R) for b, R in clear + slab], 25.0)
            frames = [(fn(b, lvl, pos, rng), R) for b, R in slab]
            est, ms = reader.gauge(frames)
            verdict = reader.taken_for(est)
            why = changed(base, est, verdict, base_verdict, args.tol_height, args.tol_depth, args.tol_near)
            ghosts = []
            if clear:
                ghosts = [reader.nearest_ahead(fn(b, lvl, pos, rng), R) < n0 - 0.10 for (b, R), n0 in zip(clear, base_lane)]
            ghost = sum(ghosts)
            flag = (" BROKEN: " + why if why else "") + (f"  CLEAR FLOOR: obstacle in the lane on {ghost}/{len(clear)} frames" if ghost else "")
            print(f"  {lvl:>6}  {fmt(est, verdict)}{flag}")
            side = "below" if lvl < _neutral else "above"
            if why or ghost:
                hit = (lvl, why or f"phantom obstacle in the lane on clear floor ({ghost}/{len(clear)} frames)")
                old = breaks.get((name, side))
                if old is None or abs(lvl - _neutral) < abs(old[0] - _neutral):
                    breaks[(name, side)] = hit
            rows.append(dict(degradation=name, level=lvl, action=verdict.action, reasons=verdict.reasons, broken=why, phantom_frames=ghost,
                             **({} if est is None else dict(height=est.height, depth=est.depth, near_x=est.near_x,
                                                            side_seen=est.side_seen, skew=est.skew, width=est.width))))
    print("\nmildest level at which something changed (each side of the undisturbed level):")
    for name, (fn, levels, neutral, where) in degradations().items():
        if args.only and args.only not in name:
            continue
        for side in ("below", "above"):
            if not any((lv < neutral) == (side == "below") for lv in levels):
                continue
            lvl, why = breaks.get((name, side), (None, "held at every level tried"))
            print(f"  {name:30s} {side:5s} {'' if lvl is None else lvl!s:>6}  {why}")
    if args.json:
        Path(args.json).write_text(json.dumps(rows, indent=1, default=float))


if __name__ == "__main__":
    main()
