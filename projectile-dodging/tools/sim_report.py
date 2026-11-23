#!/usr/bin/env python3
"""Monte-Carlo check of the dodge logic in simulation. No robot, no ROS.

    python3 tools/sim_report.py                 # Tier A lobs, every sector
    python3 tools/sim_report.py --n 60 --lag 0.3 --fisheye-fps 30
    python3 tools/sim_report.py --flat          # Tier B: flat throws

Throws are aimed at the robot from random bearings inside each sector. For each
sector it reports how many would-be hits were dodged, plus two false-alarm
checks: throws aimed well to the side, and a ball lying on the floor.

This tests the tracking, decision and timing logic with the numbers in
config/dodge.yaml. It says nothing about colour detection or how the real
robot accelerates: re-run it with --lag and --delay from tools/step_response.py
once those are measured.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from dodge.config import load_config  # noqa: E402
from dodge.sim import DEFAULT_CAMERA_TIMING, aimed_throw, run_closed_loop  # noqa: E402

SECTORS = [
    ("front cone", [(-15, 15)]),
    ("front diagonals", [(15, 60), (-60, -15)]),
    ("sides", [(60, 120), (-120, -60)]),
    ("rear diagonals", [(120, 165), (-165, -120)]),
    ("rear cone", [(165, 195)]),
]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--n", type=int, default=40, help="throws per sector")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--dist", type=float, nargs=2, default=[3.0, 4.0], help="thrower distance range (m)")
    ap.add_argument("--rise", type=float, nargs=2, default=[0.5, 1.0], help="apex above the release point (m)")
    ap.add_argument("--flat", action="store_true", help="flat throws instead of lobs (rise 0-0.15 m)")
    ap.add_argument("--lag", type=float, default=0.2, help="robot's velocity response lag (s)")
    ap.add_argument("--delay", type=float, default=0.06, help="delay from command to the robot starting to respond (s)")
    ap.add_argument("--fisheye-fps", type=float, default=DEFAULT_CAMERA_TIMING["fisheye"][0])
    ap.add_argument("--fisheye-latency", type=float, default=DEFAULT_CAMERA_TIMING["fisheye"][1])
    ap.add_argument("--elp-fps", type=float, default=DEFAULT_CAMERA_TIMING["elp_left"][0])
    ap.add_argument("--elp-latency", type=float, default=DEFAULT_CAMERA_TIMING["elp_left"][1])
    ap.add_argument("--reliable-px", type=float, default=6.0, help="smallest ball (px across) the detector is assumed to see")
    ap.add_argument("--size-bias", type=float, default=1.0, help="apparent-size error factor (1.1 = blobs 10%% too big)")
    args = ap.parse_args()

    cfg = load_config(args.config)
    timing = {"fisheye": (args.fisheye_fps, args.fisheye_latency),
              "elp_left": (args.elp_fps, args.elp_latency), "elp_right": (args.elp_fps, args.elp_latency)}
    rng = np.random.default_rng(args.seed)
    rise = (0.0, 0.15) if args.flat else tuple(args.rise)
    common = dict(timing=timing, robot_lag=args.lag, actuation_delay=args.delay,
                  reliable_px=args.reliable_px, size_bias=args.size_bias)

    print("%-16s %9s %9s %9s %12s %12s" % ("sector", "would hit", "dodged", "rate", "time left", "clearance"))
    for name, spans in SECTORS:
        would = dodged = 0
        left, clear = [], []
        for i in range(args.n):
            lo, hi = spans[i % len(spans)]
            th = aimed_throw(cfg, rng.uniform(lo, hi), rng.uniform(*args.dist), release_h=rng.uniform(0.8, 1.2),
                             rise=rng.uniform(*rise), offset=(rng.uniform(-0.08, 0.08), rng.uniform(-0.08, 0.08)))
            r = run_closed_loop(cfg, th, seed=int(rng.integers(1 << 30)), **common)
            if not r["would_hit"]:
                continue
            would += 1
            dodged += 0 if r["hit"] else 1
            if r["time_left"] is not None:
                left.append(r["time_left"])
            clear.append(r["min_clearance"])
        rate = "%.0f%%" % (100.0 * dodged / would) if would else "-"
        print("%-16s %9d %9d %9s %10.2f s %10.2f m" % (name, would, dodged, rate,
              float(np.median(left)) if left else float("nan"), float(np.median(clear)) if clear else float("nan")))

    false = total = 0
    for i in range(args.n):
        side = rng.choice([-1.0, 1.0]) * rng.uniform(0.7, 1.5)
        th = aimed_throw(cfg, rng.uniform(-180, 180), rng.uniform(*args.dist), rise=rng.uniform(*rise), offset=(side, 0.0))
        r = run_closed_loop(cfg, th, seed=int(rng.integers(1 << 30)), **common)
        total += 1
        false += 1 if r["triggered"] else 0
    print("\nthrows aimed 0.7-1.5 m to the side:  dodged anyway %d of %d" % (false, total))

    false = total = 0
    hip = float(cfg["robot"]["hip_height_m"])
    for i in range(max(8, args.n // 4)):
        b = rng.uniform(-np.pi, np.pi)
        d = rng.uniform(1.0, 3.0)
        floor_ball = [d * np.cos(b), d * np.sin(b), -hip + 0.037]
        th = aimed_throw(cfg, rng.uniform(-180, 180), 3.5, rise=0.7, offset=(3.0, 0.0))   # a throw far off, for timing
        r = run_closed_loop(cfg, th, seed=int(rng.integers(1 << 30)), floor_balls=[floor_ball], **common)
        total += 1
        false += 1 if r["triggered"] else 0
    print("ball lying on the floor in view:       dodged %d of %d" % (false, total))


if __name__ == "__main__":
    main()
