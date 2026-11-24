#!/usr/bin/env python3
"""Teach one camera what the ball looks like, and how big it appears.

Colour (do this first, for each camera, in the room and lighting you will run in):

    python3 tools/calibrate_color.py elp_left

  1. it photographs the scene with no ball in view;
  2. you move the ball around 0.5-1.5 m from that camera for ten seconds,
     low and high;
  3. whatever changed and is yellow-green is taken to be the ball. Its colour
     limits are set as loose as they can be while the empty room still shows
     nothing ball-coloured, and written to config/calib/<camera>_color.yaml.

Size (afterwards; makes the range estimate right):

    python3 tools/calibrate_color.py elp_left --distance 0.5 1 1.5 2

  You hold the ball still at each distance from the lens (tape measure), and
  the apparent-size corrections are written to the same file. Any two or more
  distances work if the farthest is at least 1.5 times the nearest; go as far
  as the room allows (1 2 3 if you have 3 m).

The ELP cameras are opened directly, so stop the dodge node first. The fisheye
needs the robot stack running.
"""
from __future__ import annotations

import argparse
import math
import os
import time

import yaml

from _common import ROOT, cv2, load_config, np, open_feed, run_tool, wait_first

from dodge.camera_models import Camera
from dodge.colorfit import fit_color
from dodge.config import COLOUR_FIT, save_calib
from dodge.detect import BallDetector

PRIOR_LO = np.array([15, 60, 50], dtype=np.uint8)     # anything from yellow to green, reasonably saturated
PRIOR_HI = np.array([90, 255, 255], dtype=np.uint8)


def grab(feed, n: int, gap: float = 0.0) -> list:
    frames = []
    while len(frames) < n:
        item = feed.get(5.0)
        if item is None:
            raise SystemExit("camera stopped delivering frames")
        frames.append(item[1])
        if gap:
            time.sleep(gap)
    return frames


def stop_if_dark(cfg: dict, name: str, frames: list) -> None:
    """Colour cannot be judged in a near-black picture: say what to do instead of failing later."""
    level = float(np.mean([f[::8, ::8].mean() for f in frames]))
    if level >= 25.0:
        return
    if cfg["cameras"][name].get("source") == "v4l2":
        fix = "Set its exposure for this room first:\n    python3 tools/tune_exposure.py %s" % name
    else:
        fix = "The room is too dark for it: add light."
    raise SystemExit("The picture from %s is too dark to work with (average %.0f of 255). %s\nNothing was written."
                     % (name, level, fix))


def has_colour_calibration(cfg: dict, name: str) -> bool:
    path = os.path.join(os.path.dirname(cfg["_path"]), "calib", name + "_color.yaml")
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
    except OSError:
        return False
    color = ((data.get("cameras") or {}).get(name) or {}).get("color") or {}
    return color.get("fit") == COLOUR_FIT


def calibrate_color(feed, cfg: dict, name: str, seconds: float) -> None:
    input("\nStep 1: make sure NO ball is in view of %s (put them behind you). Press Enter..." % name)
    bg_frames = grab(feed, 8, 0.1)
    stop_if_dark(cfg, name, bg_frames)
    # Everything from here on works on pictures with the room's colour cast taken out,
    # exactly as the detector will see them.
    seer = BallDetector.for_camera(cfg["cameras"][name], cfg["detector"])
    bg_frames = [seer.prepared(f) for f in bg_frames]
    if seer.balance.strength() > 1.8:
        print("  note: this camera's picture has a strong colour cast (blue x%.2f, red x%.2f to correct it). It is being\n"
              "  corrected, but colours are poorer for it: tools/tune_exposure.py %s sets the white balance." % (
                  seer.balance.gains[0], seer.balance.gains[2], name))
    bg = np.median(np.stack(bg_frames), axis=0).astype(np.uint8)
    input("Step 2: hold the ball 0.5-1.5 m from the camera, hand behind it, and move it slowly around\n"
          "        the picture for %.0f seconds: near and far, low and above the camera's eye level (a thrown\n"
          "        ball comes in from above, and its underside is darker). Press Enter to start..." % seconds)
    samples = []
    crops = []                 # small pictures of the ball where it was sampled: (picture, centre, radius)
    used = 0
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    end = time.time() + seconds
    while time.time() < end:
        item = feed.get(5.0)
        if item is None:
            break
        bgr = seer.balance.apply(item[1])          # same correction as the empty room got: do not follow the ball
        changed = (cv2.absdiff(bgr, bg).max(axis=2) > 40).astype(np.uint8) * 255
        hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
        cand = cv2.bitwise_and(changed, cv2.inRange(hsv, PRIOR_LO, PRIOR_HI))
        cand = cv2.morphologyEx(cand, cv2.MORPH_OPEN, kernel)
        n, labels, stats, _ = cv2.connectedComponentsWithStats(cand, connectivity=8)
        if n <= 1:
            continue
        i = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        x, y, w, h, area = (int(v) for v in stats[i])
        if area < 150 or max(w, h) > 2.2 * min(w, h) or area < 0.55 * math.pi / 4 * w * h:
            continue                                   # too small, or not round: probably not the ball
        core = cv2.erode((labels == i).astype(np.uint8), kernel)   # stay off the blurry rim
        px = hsv[core > 0]
        if len(px) < 30:
            continue
        if len(px) > 1500:
            px = px[np.random.default_rng(used).choice(len(px), 1500, replace=False)]
        samples.append(px)
        used += 1
        if len(crops) < 80:
            pad = max(w, h)
            x0, y0 = max(0, x - pad), max(0, y - pad)
            crops.append((bgr[y0:y + h + pad, x0:x + w + pad].copy(), (x + w / 2.0 - x0, y + h / 2.0 - y0), max(w, h) / 2.0))
    if used < 8:
        raise SystemExit("Only saw the ball clearly in %d frames. Check it is lit, in view, and that nothing else "
                         "yellow-green is moving. Nothing was written." % used)
    px = np.concatenate(samples)
    cam_cfg = cfg["cameras"][name]
    det_cfg = dict(cfg["detector"], max_blobs=50, balance={"enabled": False})     # these pictures are corrected already

    def judge(color: dict):
        """The same limits must leave the empty room empty: (ball-like patches found, share of it ball-coloured)."""
        det = BallDetector.for_camera(cam_cfg, det_cfg, color=color)
        return sum(len(det.detect(f)) for f in bg_frames), max(det.cover(f) for f in bg_frames)

    def recall(color: dict) -> float:
        """...and must still find the ball in the pictures it was sampled from."""
        det = BallDetector(color, det_cfg)
        hits = 0
        for crop, (cx, cy), radius in crops:
            hits += any(math.hypot(b.u - cx, b.v - cy) <= radius for b in det.detect(crop))
        return hits / float(len(crops))

    color, rep = fit_color(px, judge, recall)
    print("\nball colour from %d pictures, %d pixels:" % (used, len(px)))
    print("  hue %d-%d (OpenCV 0-179), saturation >= %d, brightness >= %d, colourfulness >= %d (and some of it >= %d)" % (
        color["h_min"], color["h_max"], color["s_min"], color["v_min"], color["c_min"], color["c_seed"]))
    print("  with these limits the ball is found in %.0f%% of the pictures it was sampled from,\n"
          "  and the empty room shows %s" % (100 * rep["found"], "nothing ball-coloured." if not rep["false"] else
                                           "%d ball-like patch%s." % (rep["false"], "" if rep["false"] == 1 else "es")))
    headroom = 100 * (1 - min(1.0, rep["dimmest"]))
    print("  the ball can be about %.0f%% dimmer than in these pictures and still be found." % headroom)
    if rep["false"]:
        print("  WARNING: something in view is the ball's colour and no setting separates it. Remove it from the room\n"
              "  (python3 tools/view.py %s --mask shows what it is), then run this step again." % name)
    if rep["found"] < 0.9:
        print("  WARNING: the ball is lost in some of its own sample pictures. In this light its colour is too close to\n"
              "  the noise in the picture. More light is the fix; then tune_exposure.py and this step again.")
    elif headroom < 30 or rep["ball_chroma"] < 45:
        print("  note: that is not much margin. The underside of a ball in flight is darker than a ball held in the\n"
              "  light, so some throws will be picked up late or not at all. More light in the room gives more margin\n"
              "  (then tune_exposure.py and this step again).")
    color["fit"] = COLOUR_FIT
    path = save_calib(name + "_color", {"cameras": {name: {"color": color}}})
    print("  written to %s" % path)
    print("\nNext, the size step:  python3 tools/calibrate_color.py %s --distance 0.5 1 1.5 2" % name)


def calibrate_size(feed, cfg: dict, name: str, distances: list) -> None:
    cam_cfg = cfg["cameras"][name]
    if not has_colour_calibration(cfg, name):
        raise SystemExit("%s has no colour calibration yet, so this step cannot tell the ball from the room.\n"
                         "Do the colour step first, then run this command again:\n"
                         "    python3 tools/calibrate_color.py %s" % (name, name))
    stop_if_dark(cfg, name, grab(feed, 5))
    camera = Camera.from_config(name, cam_cfg)
    det = BallDetector.for_camera(cam_cfg, cfg["detector"])
    radius = cfg["ball"]["diameter_m"] / 2.0
    pairs = []
    failed_frame = None
    for dist in distances:
        input("\nHold the ball still with its centre %.2f m from the %s lens, roughly in the middle of the picture. "
              "Press Enter..." % (dist, name))
        widths, expect = [], []
        nothing = at_edge = wrong_size = 0
        frames = grab(feed, 25)
        for bgr in frames:
            blobs = det.detect(bgr)
            if not blobs:
                nothing += 1
                continue
            cam = camera.with_resolution(bgr.shape[1], bgr.shape[0])
            omega = 2.0 * math.pi * (1.0 - math.cos(math.asin(min(1.0, radius / dist))))
            best = None
            for b in blobs:                      # the ball is the patch whose size fits this distance
                if b.touches_edge:
                    continue
                want = 2.0 * math.sqrt(omega / cam.model.solid_angle_per_px(b.u, b.v) / math.pi)
                miss = abs(math.log(b.diameter_px / want))
                if miss < math.log(1.7) and (best is None or miss < best[0]):
                    best = (miss, b.diameter_px, want)
            if best is None:
                if all(b.touches_edge for b in blobs):
                    at_edge += 1
                else:
                    wrong_size += 1
                continue
            widths.append(best[1])
            expect.append(best[2])
        if len(widths) < 10:
            failed_frame = frames[-1]
            why = max((nothing, "nothing in the picture has the ball's calibrated colour"),
                      (wrong_size, "ball-coloured patches were found, but none of them the size of a ball %.1f m away" % dist),
                      (at_edge, "the ball was cut off by the edge of the picture"))[1]
            print("  ball found in only %d of %d frames (%s); skipped" % (len(widths), len(frames), why))
            continue
        pairs.append((float(np.median(expect)), float(np.median(widths))))
        print("  ball is %.1f px wide; %.1f px expected" % (pairs[-1][1], pairs[-1][0]))
    if failed_frame is not None:
        out_dir = os.path.join(ROOT, "data", "snapshots")
        os.makedirs(out_dir, exist_ok=True)
        out = os.path.join(out_dir, name + "_size_problem.jpg")
        mask = cv2.cvtColor(det.mask(failed_frame), cv2.COLOR_GRAY2BGR)
        cv2.imwrite(out, np.hstack([failed_frame, mask]))
        print("\nWhere the ball was not found, the picture and what counted as ball-coloured (white) are saved side by "
              "side in\n  %s\nIf the ball is not white there, the light or exposure has changed since the colour step: "
              "redo\n  python3 tools/calibrate_color.py %s\nYou can also watch it live:  python3 tools/view.py %s --mask"
              % (out, name, name))
    if not pairs:
        raise SystemExit("No usable measurements. Nothing was written.")
    if len(pairs) < min(2, len(distances)):
        raise SystemExit("Only one of the %d distances gave a measurement, which is not enough to trust. "
                         "Nothing was written." % len(distances))
    exp = np.array([p[0] for p in pairs])
    got = np.array([p[1] for p in pairs])
    if len(pairs) >= 2 and exp.max() / exp.min() > 1.3:
        a, c = np.polyfit(exp, got, 1)                 # measured = a * true + c
        offset, scale = float(c), float(1.0 / a)
    else:
        offset = float(cam_cfg.get("size_offset_px", 0.0))
        scale = float(np.mean(exp / np.maximum(got - offset, 1.0)))
        print("  (%s: keeping size_offset_px = %.2f and fitting only the scale. For both, "
              "use two or more distances, the farthest at least 1.5 times the nearest)"
              % ("one distance" if len(pairs) == 1 else "the distances are too alike", offset))
    if not (0.7 < scale < 1.4) or not (-1.5 < offset < 3.0):
        raise SystemExit("Corrections look wrong (scale %.2f, offset %.2f px). Check the distances, the ball diameter "
                         "in config/dodge.yaml and the camera's lens settings. Nothing was written." % (scale, offset))
    path = save_calib(name + "_color", {"cameras": {name: {"size_scale": round(scale, 4), "size_offset_px": round(offset, 3)}}})
    print("\nsize_scale %.3f, size_offset_px %.2f written to %s" % (scale, offset, path))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("camera")
    ap.add_argument("--distance", type=float, nargs="+", help="known lens-to-ball distances in metres (size calibration)")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--config", default=None)
    args = ap.parse_args()
    cfg = load_config(args.config)
    feed = open_feed(cfg, args.camera, max_fps=10)
    wait_first(feed)
    try:
        if args.distance:
            calibrate_size(feed, cfg, args.camera, args.distance)
        else:
            calibrate_color(feed, cfg, args.camera, args.seconds)
    finally:
        feed.close()


if __name__ == "__main__":
    run_tool(main)
