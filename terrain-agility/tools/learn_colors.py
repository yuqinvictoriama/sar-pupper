#!/usr/bin/env python3
"""Teach terrain agility what the floor looks like to this camera, and check what it makes
of an obstacle. Nothing else is taught: obstacles are told apart by their shape.

Run on the robot with the robot stack up and the robot STANDING (so the camera is at its
walking height), under the lighting you will run in. The camera's white balance must be
the fixed one (the Gemini launch and robot/standalone.launch.py both set it).

  1. Floor - stand the robot on clear floor: nothing but floor for 1 m ahead and to the sides.
         python3 tools/learn_colors.py floor
     Writes config/floor_model.json. Repeat (--add) on other patches if the floor varies.

  2. Check - with an obstacle square across the robot's path, near face about 27 cm ahead of
     the front feet (36 cm from the body centre):
         python3 tools/learn_colors.py check
     Writes look.jpg (camera view above, classes below: grey floor, red obstacle, green ball)
     and prints what the scan and the gauge make of it, and what would be done about it:
     step over, scale across, or walk around - with the reason.

Both take --frames <folder> to work from frames saved by tools/record_frames.py.
"""
import argparse
import json
import math

import cv2
import numpy as np

import _frames
from terrain import floor_model as FM
from terrain.camera_model import PolarView
from terrain.config import TerrainConfig
from terrain import decision as D
from terrain.gauge import BoxGauge, fine_view, measure_obstacle


def polar(cfg, bgr, R):
    view = PolarView(cfg.model.scaled(bgr.shape[1], bgr.shape[0]), cfg.pose)
    pol, valid = view.sample(bgr, R)
    return view, pol, cv2.cvtColor(pol, cv2.COLOR_BGR2HSV), valid


def cmd_floor(cfg, args):
    frames = _frames.frames_from_args(args, 8)
    floor = cfg.load_floor() if args.add else FM.FloorModel()
    pixels = []
    for bgr, R, _ in frames:
        view, _, hsv, valid = polar(cfg, bgr, R)
        rows = (view.depressions > 20.0) & (view.depressions < cfg.near_depression_deg - 2.0)
        cols = np.abs(view.bearings) < 35.0
        patch = hsv[np.ix_(rows, cols)]
        pixels.append(patch[valid[np.ix_(rows, cols)]])
    pixels = np.concatenate(pixels)
    floor.learn(pixels, rate=0.5 if args.add and floor.trained else 1.0)
    cfg.save_floor(floor)
    # how much of the wider view now counts as floor
    bgr, R, _ = frames[-1]
    view, pol, hsv, valid = polar(cfg, bgr, R)
    rows = (view.depressions > 8.0) & (view.depressions < cfg.near_depression_deg)
    frac = float(np.mean(floor.is_floor(hsv)[rows][:, np.abs(view.bearings) < 60]))
    print(f"floor colour learned from {len(pixels)} pixels in {len(frames)} frames -> {cfg.floor_file}")
    print(f"  {'mostly grey/white' if floor.grey_fraction > 0.5 else 'coloured'} floor "
          f"(grey fraction {floor.grey_fraction:.2f}, brightness {floor.v_range[0]:.0f}..{floor.v_range[1]:.0f})")
    print(f"  {100 * frac:.0f} % of the view below the horizon (within 60 deg of straight ahead) now reads as floor")
    if frac < 0.85:
        print("  NOTE: under 85 %. Either something that is not floor is in view, or the floor varies: "
              "move to another patch and run again with --add.")


def cmd_check(cfg, args):
    frames = _frames.frames_from_args(args, 3)
    floor = cfg.load_floor()
    if not floor.trained:
        print("no floor model yet (run `floor`): everything will read as obstacle")
    bgr, R, quat = frames[-1]
    view, pol, hsv, labels, scan, obstacles = _frames.classify_frame(bgr, R, cfg, floor)
    img = _frames.overlay(view, pol, labels, obstacles, cfg.near_depression_deg)
    cv2.imwrite(args.out, img)
    h, w = bgr.shape[:2]
    print(f"frame {w} x {h}" + ("" if (w, h) == (cfg.model.width, cfg.model.height) else
                              f"  (lens model is for {cfg.model.width} x {cfg.model.height}: scaled)"))
    if quat is not None:
        from terrain.camera_model import tilt_of
        print(f"body tilt {math.degrees(tilt_of(R)):.1f} deg")
    if not obstacles:
        print("scan: no obstacles")
    for ob in obstacles:
        kind = "certainly tall: walked around from a distance" if ob.tall else "may be low: walked up to and measured"
        height = "to the horizon" if not math.isfinite(ob.height) else f"at most {ob.height * 1000:.0f} mm tall"
        print(f"scan: {ob.nearest:.2f} m from the lens at {ob.nearest_bearing_deg:+.0f} deg, "
              f"{ob.width:.2f} m wide, {height} ({kind})")
    if not floor.trained:
        return
    gview = fine_view(cfg.model.scaled(w, h), cfg.pose)
    gauge = BoxGauge(frames_needed=1)
    for f in frames:
        gauge.add(measure_obstacle(cv2.cvtColor(f[0], cv2.COLOR_BGR2RGB), gview, floor, R_level_body=f[1], cfg=cfg.gauge,
                                   landing=cfg.decision.landing_clear, ball=cfg.ball,
                                   near_depression_deg=cfg.near_depression_deg))
    est = gauge.estimate()
    if est is not None:
        mm = lambda v: "not read" if not math.isfinite(v) else f"{v * 1000:.0f} mm"      # noqa: E731
        print(f"gauge ({est.frames}/{len(frames)} frames): near face {est.near_x * 1000:.0f} mm ahead of the body centre, "
              f"height {mm(est.height)} (at most {mm(est.height_upper)}), depth {mm(est.depth)} (at most {mm(est.extent)}), "
              f"width {mm(est.width)}, skew {math.degrees(est.skew):+.1f} deg, "
              f"side face seen on {100 * est.side_seen:.0f} % of its width")
        est.frames = max(est.frames, cfg.decision.min_frames)      # (a check from a few frames: do not fail it for that)
        print(f"would do: {D.decide(est, cfg.decision)}")
        ms = [s for s in gauge.samples if math.isfinite(s.foot_dep)]
        if ms:
            print(json.dumps({"foot_dep": round(float(np.median([m.foot_dep for m in ms])), 2),
                              "top_dep": round(float(np.nanmedian([m.top_dep for m in ms])), 2)}))
    else:
        print("gauge: no obstacle within 0.9 m straight ahead with its foot in view")
    print(f"wrote {args.out}: camera view on top, classes below; yellow line = horizon, magenta = near_depression_deg "
          f"({cfg.near_depression_deg:.0f}). The robot's own body must be entirely BELOW the magenta line.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("floor", "check"):
        p = sub.add_parser(name)
        _frames.add_frame_args(p)
        if name == "floor":
            p.add_argument("--add", action="store_true", help="blend into the existing floor model instead of replacing it")
        if name == "check":
            p.add_argument("--out", default="look.jpg")
    args = ap.parse_args()
    cfg = TerrainConfig(args.config)
    {"floor": cmd_floor, "check": cmd_check}[args.cmd](cfg, args)


if __name__ == "__main__":
    main()
