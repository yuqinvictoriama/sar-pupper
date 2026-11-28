#!/usr/bin/env python3
"""False dodges against missed throws, for a range of threshold settings. No robot, no ROS.

    python3 tools/sweep_thresholds.py data/throws --cache /tmp/sweep-cache
    python3 tools/sweep_thresholds.py data/throws --labels data/labels.csv --n-synth 30 --out sweep.md
    python3 tools/sweep_thresholds.py data/throws --set threat.min_climb=0.03 --only    # one setting, no sweep
    python3 tools/sweep_thresholds.py --no-real                                          # simulation only

Two kinds of test case:

* Real recordings (the dodge node's data/throws folders, tools/record.py recordings). Each one is
  decoded and searched ONCE with the configuration as it stands, exactly as the camera processes
  do on the robot (FramePipeline.process_jpeg, still filter, search near the tracked ball), and the
  blobs are kept. Every setting is then tried by running the dodge core on those blobs. Settings
  that change what the cameras find (detector.*, cameras.*) need the pictures searched again: they
  are refused here; sweep them with tools/replay.py. One approximation: the search near a tracked
  ball follows the tracks of the configuration as it stands, not of the setting tried.

  A recording's label comes from --labels (<folder>,<hit|miss|none>), else from the label.json
  written by Pupper Dodging View ("dodged"/"hit" -> hit, "off target" -> miss, "not a throw" ->
  none). Unlabelled ones are only counted ("unmarked"): what they show is not known.

* Simulated cases (dodge/sim.py's cameras: blobs, not pictures, at the rates measured on the
  Pupper): lobs aimed at the robot from every side (hit), lobs aimed 0.7-1.5 m wide (miss), and
  things that are not throws (none): a ball on the floor, a still ball whose range two cameras
  read differently (the 4.5 m / 1.5 m case of 2026-11-26), a still ball whose blob flickers, a
  ball lifted out of a basket, one tossed straight up and caught, one carried past, one dropped.

Counted: a false dodge is any dodge in a "miss" or "none" case; a missed throw is a "hit" case
with no dodge (real), or one the simulated robot did not get clear of (simulated). The
recommended setting is the one with the lowest score, 3 x false dodges + missed throws
(--false-weight), nearest the current configuration, found one key at a time; its margin is the
range of each value that scores the same.
"""
from __future__ import annotations

import argparse
import copy
import csv
import hashlib
import json
import math
import os
import pickle
import sys
from concurrent.futures import ProcessPoolExecutor

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from dodge.config import load_config  # noqa: E402

# What is swept by default: (key, values). The current value is always added.
DEFAULT_SWEEP = [
    ("threat.min_climb", [0.0, 0.01, 0.015, 0.02, 0.03, 0.04, 0.06]),
    ("threat.bearing_max_closing_speed", [5.0, 6.0, 7.0, 8.0, 10.0, 12.0]),
    ("threat.loom_min", [0.15, 0.2, 0.3, 0.45, 0.6, 10.0]),
    ("threat.loom_support", [0.0, 0.15, 0.3, 0.45, 0.6]),
    ("threat.min_closing_speed", [0.5, 1.0, 2.0, 2.5, 3.0, 3.5]),
    ("threat.trigger_margin_s", [0.3, 0.45, 0.6, 0.75]),
    ("threat.confirm_cycles", [1, 2, 3, 4]),
    ("threat.sigma_lat_max_m", [0.1, 0.15, 0.2, 0.3]),
    ("threat.k_sigma", [0.5, 1.0, 1.5, 2.0]),
    ("threat.margin_m", [0.0, 0.05, 0.1]),
    ("threat.min_hits", [3, 4, 5]),
    ("tracker.gate_angle_deg", [20.0, 30.0, 45.0, 60.0, 90.0]),
    ("tracker.confirm_hits", [2, 3, 4]),
]

LABEL_WORDS = {"dodged": "hit", "hit": "hit", "off target": "miss", "not a throw": "none", "miss": "miss", "none": "none"}

# Rates measured on the Pupper in dry run, 2026-11-26 evening (plan.md, Implementation status)
MEASURED_TIMING = {"fisheye": (20.0, 0.07), "elp_left": (21.0, 0.04), "elp_right": (21.0, 0.04)}
MEASURED_LAG, MEASURED_DELAY = 0.25, 0.07


# ---------------------------------------------------------------------------------------------- settings
def get_key(cfg: dict, key: str):
    node = cfg
    for part in key.split("."):
        node = node[part]
    return node


def with_settings(cfg: dict, settings: dict) -> dict:
    out = copy.deepcopy(cfg)
    for key, value in settings.items():
        parts = key.split(".")
        node = out
        for part in parts[:-1]:
            node = node.setdefault(part, {})
        node[parts[-1]] = value
    return out


def parse_value(text: str):
    for kind in (int, float):
        try:
            return kind(text)
        except ValueError:
            pass
    return {"true": True, "false": False}.get(text.lower(), text)


def detection_fingerprint(cfg: dict) -> str:
    """What the cached blobs depend on: the cameras and the detector (and the code version is the caller's)."""
    cams = {}
    for name, cam in cfg["cameras"].items():
        cams[name] = dict(cam)
        if int(cams[name].get("search_scale", 1) or 1) == 1:
            cams[name].pop("search_scale", None)      # 1 is how pictures were always searched: caches made before stay valid
    part = {"cameras": cams, "detector": cfg["detector"], "ball": cfg["ball"]}
    return hashlib.sha1(json.dumps(part, sort_keys=True, default=str).encode()).hexdigest()[:12]


# ---------------------------------------------------------------------------------------------- real recordings
def find_recordings(paths) -> list:
    out = []
    for p in paths:
        p = os.path.abspath(p)
        if any(n.endswith(".idx") for n in os.listdir(p)):
            out.append(p)
            continue
        for name in sorted(os.listdir(p)):
            q = os.path.join(p, name)
            if os.path.isdir(q) and any(n.endswith(".idx") for n in os.listdir(q)):
                out.append(q)
    return out


def read_labels(recordings, labels_csv: str | None) -> dict:
    labels = {}
    for path in recordings:
        lp = os.path.join(path, "label.json")
        if os.path.exists(lp):
            try:
                with open(lp) as f:
                    word = str(json.load(f).get("result", "")).strip().lower()
                if word in LABEL_WORDS:
                    labels[os.path.basename(path)] = LABEL_WORDS[word]
            except (OSError, ValueError):
                pass
    if labels_csv:
        with open(labels_csv) as f:
            for row in csv.reader(f):
                if len(row) >= 2 and not row[0].startswith("#") and row[1].strip().lower() in LABEL_WORDS:
                    labels[os.path.basename(row[0].strip().rstrip("/"))] = LABEL_WORDS[row[1].strip().lower()]
    return labels


def detect_recording(path: str, cfg: dict, latency: float = 0.03) -> dict:
    """Search every picture as the robot's camera processes do, with the dodge core steering the search
    (as tools/replay.py does). Returns the blobs per frame plus the IMU, and the dodges of this config."""
    import cv2
    from dodge.core import DodgeCore
    from dodge.pipeline import FramePipeline
    from dodge.recording import RecordingReader

    cv2.setNumThreads(1)
    rec = RecordingReader(path)
    core = DodgeCore(cfg, armed=False)
    pipes = {name: FramePipeline(name, cfg) for name, cam in cfg["cameras"].items() if cam.get("enabled", True)}
    imu = rec.imu()
    imu_i = 0
    frames, events = [], []
    for t_raw, cam, jpeg in rec.frames():
        if cam not in pipes:
            continue
        t = t_raw + float(cfg["cameras"][cam].get("time_offset_s", 0.0))
        t_now = t + latency
        while imu_i < len(imu) and imu[imu_i][0] <= t_now:
            core.set_imu(imu[imu_i][0], imu[imu_i][1], imu[imu_i][2])
            imu_i += 1
        try:
            res = pipes[cam].process_jpeg(t_raw, np.frombuffer(jpeg, np.uint8))
        except ValueError:
            continue
        blobs = [tuple(b.__dict__.values()) if hasattr(b, "__dict__") else tuple(b) for b in res.blobs]
        frames.append((t, t_now, cam, tuple(res.size), blobs))
        core.process_blobs(cam, t, res.blobs, t_now, res.size)
        for pipe in pipes.values():
            pipe.set_hint(core.track_hint())
        core.command(t_now)
        events += core.pop_events()
    rec.close()
    return {"path": path, "frames": frames, "imu": imu, "events": events}


def cached_detection(args) -> dict:
    path, cfg, cache_dir = args
    key = os.path.basename(path) + "-" + detection_fingerprint(cfg) + ".pkl"
    cp = os.path.join(cache_dir, key) if cache_dir else None
    if cp and os.path.exists(cp):
        with open(cp, "rb") as f:
            return pickle.load(f)
    if not all(os.path.exists(os.path.join(path, n[:-4] + ".bin")) for n in os.listdir(path) if n.endswith(".idx")):
        return None                         # pictures not here (only the small files were copied) and nothing cached
    out = detect_recording(path, cfg)
    if cp:
        tmp = cp + ".tmp%d" % os.getpid()
        with open(tmp, "wb") as f:
            pickle.dump(out, f)
        os.replace(tmp, cp)
    return out


def run_core_on_blobs(det: dict, cfg: dict) -> list:
    """The dodge core on cached blobs. Returns its dodge events."""
    from dodge.core import DodgeCore
    from dodge.detect import Blob

    core = DodgeCore(cfg, armed=False)
    imu, imu_i = det["imu"], 0
    dodges = []
    for t, t_now, cam, size, blobs in det["frames"]:
        while imu_i < len(imu) and imu[imu_i][0] <= t_now:
            core.set_imu(imu[imu_i][0], imu[imu_i][1], imu[imu_i][2])
            imu_i += 1
        core.process_blobs(cam, t, [Blob(*b) for b in blobs], t_now, size)
        core.command(t_now)
        dodges += [e for e in core.pop_events() if e["event"] == "dodge"]
    return dodges


# ---------------------------------------------------------------------------------------------- simulation
class PathBall:
    """A ball that follows a given path (hand-held, carried, dropped), for the simulated cameras."""

    def __init__(self, fn, t_end: float):
        self.fn, self.t_land = fn, t_end

    def position(self, t: float):
        return None if t > self.t_land else np.asarray(self.fn(t), dtype=float)


def sim_negative(cfg: dict, ball, seed: int, cam_bias=None, flicker: float = 1.0, p_detect: float = 0.97,
                 timing=None, reliable_px: float = 6.0) -> bool:
    """Show the cameras something that is not a throw at the robot. True if the core decided to dodge.

    cam_bias: {camera: factor} on the ball's apparent size (each camera's own range error).
    flicker: factor on the blob-size noise.
    """
    from dodge.core import DodgeCore
    from dodge.sim import SimCamera

    rng = np.random.default_rng(seed)
    core = DodgeCore(cfg, armed=False)
    timing = timing or MEASURED_TIMING
    cams = []
    for name, cam in core.cameras.items():
        fps, lat = timing[name]
        sc = SimCamera(cam, fps, lat, rng, diameter_sigma=float(cam.cfg.get("size_sigma_px", 0.4)) * 0.8 * flicker,
                       min_area=math.pi * reliable_px ** 2 / 4.0, p_detect=p_detect)
        sc.ball_radius = cfg["ball"]["diameter_m"] / 2.0
        sc.size_bias = (cam_bias or {}).get(name, 1.0)
        cams.append((name, sc))
    t0, t, dt = 100.0, 0.0, 0.004
    fired = False
    while t <= ball.t_land:
        p = ball.position(t)
        for name, sc in cams:
            sc.step(t, [] if p is None else [p])
            for _avail, t_exp, blobs in sc.ready(t):
                fired |= bool(core.process_blobs(name, t0 + t_exp, blobs, t0 + t))
        t += dt
    return fired


def synthetic_cases(cfg: dict, n: int, seed: int) -> list:
    """(kind, label, spec) for every simulated case. spec is all a worker needs to rebuild it."""
    rng = np.random.default_rng(seed)
    cases = []
    sectors = [(-15, 15), (15, 60), (-60, -15), (60, 120), (-120, -60), (120, 165), (-165, -120), (165, 195)]
    for i in range(n * 2):
        lo, hi = sectors[i % len(sectors)]
        cases.append(("lob at the robot", "hit", {"kind": "aimed", "bearing": float(rng.uniform(lo, hi)),
                      "dist": float(rng.uniform(2.0, 3.0)), "release_h": float(rng.uniform(0.8, 1.2)),
                      "rise": float(rng.uniform(0.4, 0.9)),
                      "offset": (float(rng.uniform(-0.08, 0.08)), float(rng.uniform(-0.08, 0.08))),
                      "seed": int(rng.integers(1 << 30))}))
    for _ in range(n):
        cases.append(("lob 0.7-1.5 m wide", "miss", {"kind": "aimed", "bearing": float(rng.uniform(-180, 180)),
                      "dist": float(rng.uniform(2.0, 3.0)), "release_h": 1.0, "rise": float(rng.uniform(0.4, 0.9)),
                      "offset": (float(rng.choice([-1, 1]) * rng.uniform(0.7, 1.5)), 0.0),
                      "seed": int(rng.integers(1 << 30))}))
    hip = float(cfg["robot"]["hip_height_m"])
    for _ in range(max(4, n // 2)):
        b, d = rng.uniform(-math.pi, math.pi), rng.uniform(0.8, 3.0)
        cases.append(("ball on the floor", "none", {"kind": "still", "p": [d * math.cos(b), d * math.sin(b), -hip + 0.037],
                      "flicker": 1.0, "seed": int(rng.integers(1 << 30))}))
    for _ in range(max(4, n // 2)):
        # a still ball where a rear camera and the fisheye overlap, each reading its own range (4.5 m vs 1.5 m seen)
        b = math.radians(rng.choice([-1, 1]) * rng.uniform(60, 100))
        d = rng.uniform(1.0, 2.0)
        cases.append(("still ball, cameras disagree on range", "none", {"kind": "still",
                      "p": [d * math.cos(b), d * math.sin(b), rng.uniform(-0.1, 0.5)], "flicker": 1.5,
                      "bias": {"fisheye": float(rng.uniform(1.2, 1.8)), "elp_left": float(rng.uniform(0.4, 0.8)),
                               "elp_right": float(rng.uniform(0.4, 0.8))},
                      "seed": int(rng.integers(1 << 30))}))
    for _ in range(max(4, n // 2)):
        b, d = rng.uniform(-math.pi, math.pi), rng.uniform(0.8, 2.5)
        cases.append(("still ball, blob flickering", "none", {"kind": "still", "p": [d * math.cos(b), d * math.sin(b),
                      rng.uniform(-0.1, 0.6)], "flicker": 4.0, "p_detect": 0.75, "seed": int(rng.integers(1 << 30))}))
    for _ in range(max(4, n // 2)):
        cases.append(("ball lifted out of a basket", "none", {"kind": "lift", "bearing": float(rng.uniform(-180, 180)),
                      "dist": float(rng.uniform(1.0, 1.6)), "speed": float(rng.uniform(0.5, 1.5)),
                      "top": float(rng.uniform(0.6, 1.1)), "toward": float(rng.uniform(0.0, 0.4)),
                      "seed": int(rng.integers(1 << 30))}))
    for _ in range(max(4, n // 2)):
        cases.append(("ball tossed up and caught", "none", {"kind": "toss", "bearing": float(rng.uniform(-180, 180)),
                      "dist": float(rng.uniform(1.0, 2.0)), "rise": float(rng.uniform(0.2, 0.6)),
                      "seed": int(rng.integers(1 << 30))}))
    for _ in range(max(4, n // 2)):
        cases.append(("ball carried past", "none", {"kind": "carry", "bearing": float(rng.uniform(-180, 180)),
                      "dist": float(rng.uniform(0.8, 1.6)), "speed": float(rng.uniform(0.7, 1.5)),
                      "seed": int(rng.integers(1 << 30))}))
    for _ in range(max(4, n // 2)):
        cases.append(("ball dropped from the hand", "none", {"kind": "drop", "bearing": float(rng.uniform(-180, 180)),
                      "dist": float(rng.uniform(0.6, 1.6)), "seed": int(rng.integers(1 << 30))}))
    return cases


def run_synthetic(args):
    """One simulated case under one configuration: (triggered, failed). failed = false dodge or missed throw."""
    cfg, (kind, label, spec) = args
    from dodge.sim import aimed_throw, run_closed_loop

    hip = float(cfg["robot"]["hip_height_m"])
    seed = spec["seed"]
    if spec["kind"] == "aimed":
        th = aimed_throw(cfg, spec["bearing"], spec["dist"], release_h=spec["release_h"], rise=spec["rise"],
                         offset=spec["offset"])
        r = run_closed_loop(cfg, th, seed=seed, timing=MEASURED_TIMING, robot_lag=MEASURED_LAG,
                            actuation_delay=MEASURED_DELAY)
        if label == "hit":
            if not r["would_hit"]:
                return None                                   # aimed, but grazes past: not a test of anything
            return {"triggered": r["triggered"], "failed": bool(r["hit"]), "time_left": r["time_left"]}
        return {"triggered": r["triggered"], "failed": bool(r["triggered"]), "time_left": None}
    b = math.radians(spec.get("bearing", 0.0))
    d = spec.get("dist", 1.0)
    out = np.array([math.cos(b), math.sin(b), 0.0])
    side = np.array([-math.sin(b), math.cos(b), 0.0])
    if spec["kind"] == "still":
        p = np.array(spec["p"])
        ball = PathBall(lambda t: p, 2.5)
    elif spec["kind"] == "lift":
        base = out * d + np.array([0.0, 0.0, -hip + 0.15])
        sp, top, toward = spec["speed"], spec["top"], spec["toward"]
        t_up = top / sp

        def path(t, base=base, sp=sp, top=top, toward=toward, t_up=t_up):
            s = min(t, t_up)
            return base + np.array([0.0, 0.0, sp * s]) - out * toward * (s / t_up)
        ball = PathBall(path, t_up + 1.0)
    elif spec["kind"] == "toss":
        base = out * d + np.array([0.0, 0.0, 1.0 - hip])
        vz = math.sqrt(2 * 9.81 * spec["rise"])
        t_fl = 2 * vz / 9.81

        def path(t, base=base, vz=vz, t_fl=t_fl):
            s = min(max(t - 0.3, 0.0), t_fl)
            return base + np.array([0.0, 0.0, vz * s - 4.905 * s * s])
        ball = PathBall(path, t_fl + 0.8)
    elif spec["kind"] == "carry":
        base = out * d + np.array([0.0, 0.0, 0.9 - hip]) - side * 1.5
        sp = spec["speed"]
        ball = PathBall(lambda t, base=base, sp=sp: base + side * sp * t, 3.0 / sp)
    else:                                                     # dropped from about 1 m, lands and rolls a little
        base = out * d + np.array([0.0, 0.0, 1.0 - hip])
        t_fall = math.sqrt(2 * (1.0 - 0.037) / 9.81)

        def path(t, base=base, t_fall=t_fall):
            s = max(t - 0.3, 0.0)
            if s < t_fall:
                return base - np.array([0.0, 0.0, 4.905 * s * s])
            return base - np.array([0.0, 0.0, 1.0 - 0.037]) - out * 0.3 * (s - t_fall)
        ball = PathBall(path, 0.3 + t_fall + 1.0)
    fired = sim_negative(cfg, ball, seed, cam_bias=spec.get("bias"), flicker=spec.get("flicker", 1.0),
                         p_detect=spec.get("p_detect", 0.97))
    return {"triggered": fired, "failed": fired, "time_left": None}


# ---------------------------------------------------------------------------------------------- scoring
def score_real(args):
    det, cfg = args
    d = run_core_on_blobs(det, cfg)
    return os.path.basename(det["path"]), len(d), (d[0]["threat"]["tau"] if d else None)


def evaluate(cfg: dict, dets: list, labels: dict, cases: list, pool) -> dict:
    real = list(pool.map(score_real, [(d, cfg) for d in dets], chunksize=4)) if dets else []
    res = {"real_false": 0, "real_false_of": 0, "real_miss": 0, "real_hit_of": 0, "unmarked_dodged": 0,
           "unmarked_of": 0, "tau_hits": [], "real_detail": {}}
    for name, n, tau in real:
        lab = labels.get(name)
        res["real_detail"][name] = (lab, n, tau)
        if lab in ("none", "miss"):
            res["real_false_of"] += 1
            res["real_false"] += 1 if n else 0
        elif lab == "hit":
            res["real_hit_of"] += 1
            res["real_miss"] += 0 if n else 1
            if n and tau is not None:
                res["tau_hits"].append(tau)
        else:
            res["unmarked_of"] += 1
            res["unmarked_dodged"] += 1 if n else 0
    syn = list(pool.map(run_synthetic, [(cfg, c) for c in cases], chunksize=2)) if cases else []
    by_kind = {}
    res.update({"syn_false": 0, "syn_false_of": 0, "syn_miss": 0, "syn_hit_of": 0, "syn_time_left": []})
    for (kind, label, _spec), r in zip(cases, syn):
        if r is None:
            continue
        k = by_kind.setdefault(kind, [0, 0])
        k[0] += 1
        k[1] += 1 if r["failed"] else 0
        if label == "hit":
            res["syn_hit_of"] += 1
            res["syn_miss"] += 1 if r["failed"] else 0
            if r["time_left"] is not None:
                res["syn_time_left"].append(r["time_left"])
        else:
            res["syn_false_of"] += 1
            res["syn_false"] += 1 if r["failed"] else 0
    res["by_kind"] = by_kind
    return res


WEIGHT = {"false": 3.0}   # one false dodge counts as this many missed throws (one ended in a fall, 2026-11-26)


def cost(r: dict) -> tuple:
    """A false dodge weighs WEIGHT["false"] missed throws; ties go to fewer false dodges."""
    false, miss = r["real_false"] + r["syn_false"], r["real_miss"] + r["syn_miss"]
    return (WEIGHT["false"] * false + miss, false)


def row(label: str, r: dict) -> str:
    tl = np.median(r["syn_time_left"]) if r["syn_time_left"] else float("nan")
    tau = np.median(r["tau_hits"]) if r["tau_hits"] else float("nan")
    return "| %s | %d/%d | %d/%d | %d/%d | %d/%d | %d/%d | %.2f | %.2f |" % (
        label, r["real_false"], r["real_false_of"], r["real_miss"], r["real_hit_of"],
        r["syn_false"], r["syn_false_of"], r["syn_miss"], r["syn_hit_of"],
        r["unmarked_dodged"], r["unmarked_of"], tau, tl)


HEADER = ("| setting | real false dodges | real throws missed | sim false dodges | sim throws not cleared | "
          "unmarked recordings dodged | real: s left at decision (median) | sim: s left at decision (median) |\n"
          "|---|---|---|---|---|---|---|---|")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recordings", nargs="*", help="recording folders, or folders of them (data/throws)")
    ap.add_argument("--config", default=None)
    ap.add_argument("--calib-dir", default=None, help="calibration folder to use instead of config/calib")
    ap.add_argument("--labels", default=None, help="CSV <folder>,<hit|miss|none>; overrides label.json")
    ap.add_argument("--cache", default=None, help="keep the searched blobs here between runs")
    ap.add_argument("--n-synth", type=int, default=20, help="simulated cases per kind (lobs at the robot: twice this)")
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--no-real", action="store_true")
    ap.add_argument("--set", action="append", default=[], metavar="KEY=VALUE", help="start from this setting")
    ap.add_argument("--only", action="store_true", help="evaluate the starting settings only, no sweep")
    ap.add_argument("--sweep", action="append", default=[], metavar="KEY=V1,V2,...", help="sweep these instead")
    ap.add_argument("--rounds", type=int, default=2, help="rounds of one-key-at-a-time improvement")
    ap.add_argument("--jobs", type=int, default=max(1, (os.cpu_count() or 2) - 1))
    ap.add_argument("--false-weight", type=float, default=WEIGHT["false"],
                    help="how many missed throws one false dodge is worth when choosing (default %(default)s)")
    ap.add_argument("--out", default=None, help="also write the tables here (Markdown)")
    ap.add_argument("--detail", default=None, help="write each real recording's result per setting here (JSON)")
    args = ap.parse_args()

    WEIGHT["false"] = args.false_weight
    base = load_config(args.config, calib_dir=args.calib_dir)
    start = {kv.split("=", 1)[0]: parse_value(kv.split("=", 1)[1]) for kv in args.set}
    sweep = DEFAULT_SWEEP
    if args.sweep:
        sweep = [(kv.split("=", 1)[0], [parse_value(v) for v in kv.split("=", 1)[1].split(",")]) for kv in args.sweep]
    for key, _ in sweep + [(k, None) for k in start]:
        if key.startswith(("detector.", "cameras.")):
            ap.error("%s changes what the cameras find: sweep it with tools/replay.py, not on cached blobs" % key)
    if args.cache:
        os.makedirs(args.cache, exist_ok=True)
    lines = []

    def say(text: str = "") -> None:
        print(text, flush=True)
        lines.append(text)

    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        dets, labels = [], {}
        if not args.no_real and args.recordings:
            recs = find_recordings(args.recordings)
            labels = read_labels(recs, args.labels)
            print("searching %d recordings (cached after the first time)..." % len(recs), file=sys.stderr, flush=True)
            dets = list(pool.map(cached_detection, [(p, base, args.cache) for p in recs]))
            missing = [os.path.basename(p) for p, d in zip(recs, dets) if d is None]
            if missing:
                print("left out, no pictures and nothing cached: %d (%s ...)" % (len(missing), ", ".join(missing[:3])),
                      file=sys.stderr)
            dets = [d for d in dets if d is not None]
        cases = synthetic_cases(base, args.n_synth, args.seed)
        cfg0 = with_settings(base, start)
        current = {key: get_key(cfg0, key) for key, _ in sweep}
        say("Real recordings: %d (%d labelled hit, %d miss or none, %d unmarked). Simulated cases: %d." % (
            len(dets), sum(1 for d in dets if labels.get(os.path.basename(d["path"])) == "hit"),
            sum(1 for d in dets if labels.get(os.path.basename(d["path"])) in ("miss", "none")),
            sum(1 for d in dets if os.path.basename(d["path"]) not in labels), len(cases)))
        say()
        r0 = evaluate(cfg0, dets, labels, cases, pool)
        detail = {"current": r0["real_detail"]}
        say(HEADER)
        say(row("current" + (" (%s)" % ", ".join("%s=%s" % kv for kv in start.items()) if start else ""), r0))
        say()
        say("Simulated, by kind (failed / cases): " + "; ".join("%s %d/%d" % (k, v[1], v[0]) for k, v in r0["by_kind"].items()))
        if args.only:
            _finish(args, lines, detail)
            return
        best_settings, best = dict(start), r0
        margins = {}
        for rnd in range(args.rounds):
            changed = False
            for key, values in sweep:
                cur = get_key(with_settings(base, best_settings), key)
                vals = sorted(set(values) | {cur}, key=float)
                results = {}
                say()
                say("Round %d: %s (now %s)" % (rnd + 1, key, cur))
                say()
                say(HEADER)
                for v in vals:
                    s = dict(best_settings, **{key: v})
                    r = best if v == cur else evaluate(with_settings(base, s), dets, labels, cases, pool)
                    results[v] = r
                    detail["%s=%s (round %d)" % (key, v, rnd + 1)] = r["real_detail"]
                    say(row("%s = %s%s" % (key, v, "  (now)" if v == cur else ""), r))
                pick = min(vals, key=lambda v: (cost(results[v]), abs(vals.index(v) - vals.index(cur))))
                if cost(results[pick]) < cost(best):
                    best_settings[key] = pick
                    best = results[pick]
                    changed = True
                else:
                    pick = cur
                # margin: neighbouring values that score the same as the pick
                i = vals.index(pick)
                lo = hi = i
                while lo > 0 and cost(results[vals[lo - 1]]) == cost(results[pick]):
                    lo -= 1
                while hi < len(vals) - 1 and cost(results[vals[hi + 1]]) == cost(results[pick]):
                    hi += 1
                margins[key] = (pick, vals[lo], vals[hi], lo == 0, hi == len(vals) - 1)
            if not changed:
                break
        say()
        say("## Recommended")
        say()
        if best_settings == start:
            say("No change: no setting tried scores better than the current one.")
        else:
            say("Changed from the current configuration: " + ", ".join("%s %s -> %s" % (k, current.get(k, "?"), v)
                                                                     for k, v in best_settings.items() if k not in start))
        say()
        say(HEADER)
        say(row("current", r0))
        say(row("recommended", best))
        say()
        say("Margin (values of each key, all else as recommended, that score the same; '<' or '>' = the edge of the range tried):")
        say()
        for key, (pick, lo, hi, at_lo, at_hi) in margins.items():
            say("- %s = %s: same score from %s%s to %s%s" % (key, pick, "<" if at_lo else "", lo, hi, ">" if at_hi else ""))
    _finish(args, lines, detail)


def _finish(args, lines, detail) -> None:
    if args.out:
        with open(args.out, "w") as f:
            f.write("\n".join(lines) + "\n")
    if args.detail:
        with open(args.detail, "w") as f:
            json.dump(detail, f, indent=0, default=str)


if __name__ == "__main__":
    main()
