#!/usr/bin/env python3
"""Teach one camera what the ball looks like, and how big it appears.

Colour (do this first, for each camera, in the room and lighting you will run in):

    python3 tools/calibrate_color.py elp_left

  1. it photographs the scene with no ball in view;
  2. you wave the ball around 0.5-1.5 m from that camera for ten seconds;
  3. whatever changed and is yellow-green is taken to be the ball, and its
     colour range is written to config/calib/<camera>_color.yaml.

Size (afterwards; makes the range estimate right):

    python3 tools/calibrate_color.py elp_left --distance 1.0 2.0 3.0

  You hold the ball still at each distance from the lens (tape measure), and
  the apparent-size corrections are written to the same file.

The ELP cameras are opened directly, so stop the dodge node first. The fisheye
needs the robot stack running.
"""
from __future__ import annotations

import argparse
import math
import time

from _common import cv2, load_config, np, open_feed, run_tool, wait_first

from dodge.camera_models import Camera
from dodge.config import save_calib
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


def calibrate_color(feed, cfg: dict, name: str, seconds: float) -> None:
    input("\nStep 1: make sure NO ball is in view of %s (put them behind you). Press Enter..." % name)
    bg_frames = grab(feed, 8, 0.1)
    bg = np.median(np.stack(bg_frames), axis=0).astype(np.uint8)
    input("Step 2: hold the ball 0.5-1.5 m from the camera, hand behind it, and move it slowly around\n"
          "        the picture, near and far, for %.0f seconds. Press Enter to start..." % seconds)
    samples = []
    used = 0
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    end = time.time() + seconds
    while time.time() < end:
        item = feed.get(5.0)
        if item is None:
            break
        bgr = item[1]
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
    if used < 8:
        raise SystemExit("Only saw the ball clearly in %d frames. Check it is lit, in view, and that nothing else "
                         "yellow-green is moving. Nothing was written." % used)
    px = np.concatenate(samples)
    h_lo, h_hi = np.percentile(px[:, 0], [1, 99])
    s_lo = np.percentile(px[:, 1], 2)
    v_lo = np.percentile(px[:, 2], 2)
    color = {"h_min": int(max(0, h_lo - 4)), "h_max": int(min(179, h_hi + 4)),
             "s_min": int(max(50, s_lo - 25)), "s_max": 255, "v_min": int(max(40, v_lo - 30)), "v_max": 255}
    # The same thresholds must leave the empty scene empty.
    det_cfg = dict(cfg["detector"])
    for _ in range(12):
        false = sum(len(BallDetector.for_camera(cfg["cameras"][name], det_cfg, color=color).detect(f)) for f in bg_frames)
        if false == 0:
            break
        color["s_min"] = min(250, color["s_min"] + 8)
    print("\nball colour from %d frames, %d pixels:" % (used, len(px)))
    print("  hue %d-%d (OpenCV 0-179), saturation >= %d, brightness >= %d" % (
        color["h_min"], color["h_max"], color["s_min"], color["v_min"]))
    print("  brightness of ball pixels: median %.0f%s" % (
        float(np.median(px[:, 2])), "  (dark: raise exposure_100us or gain for this camera)" if np.median(px[:, 2]) < 90 else ""))
    if false:
        print("  WARNING: %d ball-coloured patches remain in the EMPTY scene even after tightening. Something in view "
              "is the ball's colour; remove it or expect false tracks." % false)
    path = save_calib(name + "_color", {"cameras": {name: {"color": color}}})
    print("  written to %s" % path)


def calibrate_size(feed, cfg: dict, name: str, distances: list) -> None:
    cam_cfg = cfg["cameras"][name]
    camera = Camera.from_config(name, cam_cfg)
    det = BallDetector.for_camera(cam_cfg, cfg["detector"])
    radius = cfg["ball"]["diameter_m"] / 2.0
    pairs = []
    for dist in distances:
        input("\nHold the ball still with its centre %.2f m from the %s lens, roughly in the middle of the picture. "
              "Press Enter..." % (dist, name))
        widths, expect = [], []
        for bgr in grab(feed, 25):
            blobs = det.detect(bgr)
            if not blobs or blobs[0].touches_edge:
                continue
            b = blobs[0]
            cam = camera.with_resolution(bgr.shape[1], bgr.shape[0])
            sr = cam.model.solid_angle_per_px(b.u, b.v)
            omega = 2.0 * math.pi * (1.0 - math.cos(math.asin(min(1.0, radius / dist))))
            widths.append(b.diameter_px)
            expect.append(2.0 * math.sqrt(omega / sr / math.pi))
        if len(widths) < 10:
            print("  ball not found reliably at this distance; skipped")
            continue
        pairs.append((float(np.median(expect)), float(np.median(widths))))
        print("  ball is %.1f px wide; %.1f px expected" % (pairs[-1][1], pairs[-1][0]))
    if not pairs:
        raise SystemExit("No usable measurements. Nothing was written.")
    exp = np.array([p[0] for p in pairs])
    got = np.array([p[1] for p in pairs])
    if len(pairs) >= 2 and exp.max() / exp.min() > 1.3:
        a, c = np.polyfit(exp, got, 1)                 # measured = a * true + c
        offset, scale = float(c), float(1.0 / a)
    else:
        offset = float(cam_cfg.get("size_offset_px", 0.0))
        scale = float(np.mean(exp / np.maximum(got - offset, 1.0)))
        print("  (one distance only: keeping size_offset_px = %.2f; use two or more distances to fit it)" % offset)
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
