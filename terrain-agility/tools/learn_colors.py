#!/usr/bin/env python3
"""Teach terrain agility what the floor and the step-over box look like to this camera.

Run on the robot with the robot stack up and the robot STANDING (so the camera is at its
walking height), under the lighting you will run in. The camera's white balance must be
the fixed one (the Gemini launch and robot/standalone.launch.py both set it).

  1. Floor - stand the robot on clear floor: nothing but floor for 1 m ahead and to the sides.
         python3 tools/learn_colors.py floor
     Writes config/floor_model.json. Repeat (--add) on other patches if the floor varies.

  2. Box - put the box square across the robot's path, near face about 40 cm ahead of the
     front feet, nothing else within 1 m ahead.
         python3 tools/learn_colors.py box
     Finds the pixels that are not floor, splits them into the box's side (lower in the
     picture) and top (above it), and writes both colour ranges into config/terrain.yaml.

  3. Check - with anything in view:
         python3 tools/learn_colors.py check
     Writes look.jpg (camera view above, classes below: grey floor, red obstacle, blue-ish
     box side, white box top, green ball) and prints what the scan and the gauge make of it.

All three take --frames <folder> to work from frames saved by tools/record_frames.py.
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
from terrain.gauge import fine_view, measure_box


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


def cmd_box(cfg, args):
    frames = _frames.frames_from_args(args, 8)
    floor = cfg.load_floor()
    if not floor.trained:
        raise SystemExit("learn the floor first:  python3 tools/learn_colors.py floor")
    side_px, top_px = [], []
    for bgr, R, _ in frames:
        view, _, hsv, valid = polar(cfg, bgr, R)
        rows = np.nonzero((view.depressions > 12.0) & (view.depressions < cfg.near_depression_deg - 1.0))[0]
        cols = np.nonzero(np.abs(view.bearings) < 25.0)[0]
        sub = hsv[np.ix_(rows, cols)]
        not_floor = ~floor.is_floor(sub) & valid[np.ix_(rows, cols)]
        not_floor = cv2.morphologyEx(not_floor.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)).astype(bool)
        if not_floor.sum() < 150:
            continue
        rr, cc = np.nonzero(not_floor)
        px = sub[rr, cc].astype(np.float32)
        # two clusters in (hue on a circle, saturation, value)
        ang = px[:, 0] * (math.pi / 90.0)
        sat = px[:, 1] / 255.0
        feat = np.stack([np.cos(ang) * sat, np.sin(ang) * sat, px[:, 2] / 255.0], axis=1).astype(np.float32)
        crit = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 50, 1e-4)
        _, lab, _ = cv2.kmeans(feat, 2, None, crit, 5, cv2.KMEANS_PP_CENTERS)
        lab = lab.ravel()
        if min((lab == 0).sum(), (lab == 1).sum()) < 60:
            continue
        # the side face is the cluster lower in the picture (larger depression = larger row)
        lower = 0 if rr[lab == 0].mean() > rr[lab == 1].mean() else 1
        # drop each patch's rim: pixels on an edge are a blend of two colours
        for which, store in ((lower, side_px), (1 - lower, top_px)):
            patch = np.zeros(not_floor.shape, np.uint8)
            patch[rr[lab == which], cc[lab == which]] = 1
            core = cv2.erode(patch, np.ones((3, 3), np.uint8)).astype(bool)
            if core.sum() < 30:
                core = patch.astype(bool)
            store.append(sub[core].astype(np.float32))
    if len(side_px) < max(1, len(frames) // 2):
        raise SystemExit("could not find a two-coloured box in view. Check: the floor model (run `check`), the box is "
                         "30-50 cm ahead with its side and top in different colours, nothing else within 1 m.")
    side = FM.ColorRange.from_samples(np.concatenate(side_px))
    top = FM.ColorRange.from_samples(np.concatenate(top_px))
    print(f"box side: {side.to_dict()}")
    print(f"box top : {top.to_dict()}")
    # sanity: the two classes and the floor must not overlap
    bgr, R, _ = frames[-1]
    view, _, hsv, valid = polar(cfg, bgr, R)
    rows = (view.depressions > 12.0) & (view.depressions < cfg.near_depression_deg - 1.0)
    region = hsv[rows]
    fl = floor.is_floor(region)
    for name, rng in (("side", side), ("top", top)):
        clash = float(np.mean(rng.mask(region)[fl])) if fl.any() else 0.0
        if clash > 0.02:
            print(f"  WARNING: {100 * clash:.0f} % of the floor also matches the box {name} colour. "
                  "Pick a colour that contrasts with this floor (tape or paint) and learn again.")
    both = float(np.mean(side.mask(region) & top.mask(region)))
    if both > 0.005:
        print("  WARNING: the side and top ranges overlap; their colours are too similar.")

    def fmt(r):
        d = r.to_dict()
        return "{" + ", ".join(f"{k}: {v}" for k, v in d.items()) + "}"

    if args.dry_run:
        print("(dry run: terrain.yaml not changed)")
        return
    ok = _frames.set_yaml_value(cfg.path, "colors", "box_side", fmt(side)) and \
        _frames.set_yaml_value(cfg.path, "colors", "box_top", fmt(top))
    print(f"written to {cfg.path}" if ok else f"could not find colors.box_side / box_top in {cfg.path}; add them by hand")
    cfg2 = TerrainConfig(cfg.path)
    gview = fine_view(cfg2.model.scaled(bgr.shape[1], bgr.shape[0]), cfg2.pose)
    m = measure_box(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), gview, cfg2.box_side, cfg2.box_top, R_level_body=R)
    if m is None:
        print("  the gauge does not see the box with these colours yet: run `check` and look at look.jpg")
    else:
        print(f"  gauge reading with the current camera pose: {m.height * 1000:.0f} mm tall, {m.depth * 1000:.0f} mm deep, "
              f"near face {m.near_x * 1000:.0f} mm ahead of the body centre (calibrate the camera pose next)")


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
        kind = "the box" if ob.is_box else ("tall" if ob.tall else "low, unknown")
        height = "to the horizon" if not math.isfinite(ob.height) else f"about {ob.height * 1000:.0f} mm tall"
        print(f"scan: {kind}: {ob.nearest:.2f} m from the lens at {ob.nearest_bearing_deg:+.0f} deg, "
              f"{ob.width:.2f} m wide, {height}")
    gview = fine_view(cfg.model.scaled(w, h), cfg.pose)
    ms = [measure_box(cv2.cvtColor(f[0], cv2.COLOR_BGR2RGB), gview, cfg.box_side, cfg.box_top, R_level_body=f[1])
          for f in frames]
    ms = [m for m in ms if m is not None]
    if ms:
        med = lambda k: float(np.median([getattr(m, k) for m in ms]))  # noqa: E731
        print(f"gauge ({len(ms)}/{len(frames)} frames): height {med('height') * 1000:.1f} mm, depth {med('depth') * 1000:.0f} mm, "
              f"near face {med('near_x') * 1000:.0f} mm ahead of the body centre, skew {math.degrees(med('skew')):+.1f} deg, "
              f"width {med('width') * 1000:.0f} mm")
        print(json.dumps({"foot_dep": round(med("foot_dep"), 2), "top_dep": round(med("top_dep"), 2)}))
    else:
        print("gauge: the two-colour box is not in view (or not within 0.9 m)")
    print(f"wrote {args.out}: camera view on top, classes below; yellow line = horizon, magenta = near_depression_deg "
          f"({cfg.near_depression_deg:.0f}). The robot's own body must be entirely BELOW the magenta line.")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    for name in ("floor", "box", "check"):
        p = sub.add_parser(name)
        _frames.add_frame_args(p)
        if name == "floor":
            p.add_argument("--add", action="store_true", help="blend into the existing floor model instead of replacing it")
        if name == "box":
            p.add_argument("--dry-run", action="store_true", help="print the ranges without writing terrain.yaml")
        if name == "check":
            p.add_argument("--out", default="look.jpg")
    args = ap.parse_args()
    cfg = TerrainConfig(args.config)
    {"floor": cmd_floor, "box": cmd_box, "check": cmd_check}[args.cmd](cfg, args)


if __name__ == "__main__":
    main()
