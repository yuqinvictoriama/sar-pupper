#!/usr/bin/env python3
"""Measure the fisheye's lit disc, so the detector can ignore the lens rim.

    python3 tools/fit_image_circle.py fisheye            # look, print, save a preview picture
    python3 tools/fit_image_circle.py fisheye --write    # also write config/calib/image_circle.yaml

A fisheye lens lights a disc in the middle of the picture and leaves the
corners black. Right at the edge of the disc the lens splits colours, and part
of that fringe is the ball's green. The detector therefore ignores everything
beyond a little inside the rim (image_circle in config/dodge.yaml). The disc
sits in a slightly different place on every camera, so measure it once.

Point the robot at a normally lit room (not into the dark, not at a lamp) and
keep it still for the two seconds this takes. The robot stack must be running.
"""
from __future__ import annotations

import argparse
import os

from _common import ROOT, cv2, load_config, np, open_feed, run_tool, wait_first

from dodge.config import save_calib


def fit_circle(pts: np.ndarray):
    """Least-squares circle through points, (cx, cy, r)."""
    a = np.c_[2.0 * pts, np.ones(len(pts))]
    b = (pts ** 2).sum(axis=1)
    sol = np.linalg.lstsq(a, b, rcond=None)[0]
    return float(sol[0]), float(sol[1]), float(np.sqrt(max(sol[2] + sol[0] ** 2 + sol[1] ** 2, 0.0)))


def robust_circle(pts: np.ndarray):
    """Circle through the points that agree with each other; the rest (not on the rim) are dropped."""
    cx, cy, r = fit_circle(pts)
    for _ in range(8):
        d = np.hypot(pts[:, 0] - cx, pts[:, 1] - cy) - r
        keep = np.abs(d) <= max(2.0, 2.0 * float(np.median(np.abs(d))))
        if keep.sum() < 40 or keep.all():
            break
        pts = pts[keep]
        cx, cy, r = fit_circle(pts)
    return cx, cy, r, len(pts)


def find_image_circle(frames: list):
    """The lit disc in a set of frames of a still scene: (cx, cy, r) in pixels, or None.

    Takes the brightest value each pixel reaches (so a dark object does not eat
    into the disc). A first guess comes from the outline of everything that is
    not black. That outline also follows the glow just outside the rim and any
    lens flare, so the answer is then refined: along 360 spokes from the centre,
    the rim is where the picture gets darker fastest; a circle is fitted to
    those points, leaving out spokes that hit something else (a dark part of the
    scene, a flare, the edge of the frame).
    """
    peak = np.max(np.stack([cv2.cvtColor(f, cv2.COLOR_BGR2GRAY) for f in frames]), axis=0)
    h, w = peak.shape
    corner = np.concatenate([peak[:40, :40].ravel(), peak[:40, -40:].ravel(), peak[-40:, :40].ravel(), peak[-40:, -40:].ravel()])
    threshold = int(min(80, max(18, float(np.percentile(corner, 99)) + 12)))
    lit = (peak > threshold).astype(np.uint8)
    lit = cv2.morphologyEx(lit, cv2.MORPH_CLOSE, np.ones((15, 15), np.uint8))
    lit = cv2.morphologyEx(lit, cv2.MORPH_OPEN, np.ones((9, 9), np.uint8))
    contours, _ = cv2.findContours(lit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    if not contours:
        return None
    c = max(contours, key=cv2.contourArea)
    if cv2.contourArea(c) < 0.08 * w * h:
        return None
    (cx, cy), r = cv2.minEnclosingCircle(cv2.convexHull(c))
    smooth = cv2.GaussianBlur(peak, (0, 0), 2.0).astype(np.float32)
    angles = np.radians(np.arange(0.0, 360.0, 1.0))
    for _ in range(3):
        radii = np.arange(0.55 * r, 1.2 * r, 1.0)
        xs = cx + np.outer(np.cos(angles), radii)
        ys = cy + np.outer(np.sin(angles), radii)
        inside = (xs >= 1) & (xs <= w - 2) & (ys >= 1) & (ys <= h - 2)
        prof = cv2.remap(smooth, xs.astype(np.float32), ys.astype(np.float32), cv2.INTER_LINEAR)
        drop = prof[:, :-4] - prof[:, 4:]                       # how much darker 4 px further out
        drop[~(inside[:, :-4] & inside[:, 4:])] = -1.0
        k = np.argmax(drop, axis=1)
        strong = drop[np.arange(len(angles)), k] > 12.0         # a real edge, not a gentle fade
        if strong.sum() < 60:
            return None
        rim_r = radii[k[strong] + 2]
        pts = np.c_[cx + rim_r * np.cos(angles[strong]), cy + rim_r * np.sin(angles[strong])]
        cx, cy, r, used = robust_circle(pts)
        if used < 60:
            return None
    if not (0.15 * min(w, h) < r < 0.75 * max(w, h)):
        return None
    return cx, cy, r


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("camera", nargs="?", default="fisheye")
    ap.add_argument("--write", action="store_true", help="save the result to config/calib/image_circle.yaml")
    ap.add_argument("--margin", type=float, default=None, help="fraction of the radius to ignore at the rim (default: keep the config's)")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()
    cfg = load_config(args.config)
    cam_cfg = cfg["cameras"][args.camera]
    if cam_cfg.get("model") != "double_sphere":
        raise SystemExit("%s is not a fisheye: its picture fills the frame, so there is no rim to ignore." % args.camera)
    feed = open_feed(cfg, args.camera, max_fps=10)
    wait_first(feed)
    frames = []
    while len(frames) < 20:
        item = feed.get(5.0)
        if item is None:
            raise SystemExit("camera stopped delivering frames")
        frames.append(item[1])
    feed.close()
    found = find_image_circle(frames)
    if found is None:
        raise SystemExit("Could not find the lit disc. Point the camera at a normally lit room and try again. Nothing was written.")
    h, w = frames[-1].shape[:2]
    intr = cam_cfg.get("intrinsics", {})
    ref_w, ref_h = float(intr.get("width", w)), float(intr.get("height", h))
    cx, cy, r = found[0] * ref_w / w, found[1] * ref_h / h, found[2] * 0.5 * (ref_w / w + ref_h / h)
    old = cam_cfg.get("image_circle") or {}
    margin = float(args.margin if args.margin is not None else old.get("margin_frac", 0.08))
    print("lit disc: centre (%.0f, %.0f), radius %.0f px in a %.0f x %.0f frame" % (cx, cy, r, ref_w, ref_h))
    if old.get("r"):
        print("config has: centre (%.0f, %.0f), radius %.0f" % (old["cx"], old["cy"], old["r"]))
    print("the detector will ignore everything beyond %.0f px from that centre (%.0f%% inside the rim)" % (r * (1 - margin), 100 * margin))
    show = frames[-1].copy()
    cv2.circle(show, (int(found[0]), int(found[1])), int(found[2]), (0, 255, 255), 2)
    cv2.circle(show, (int(found[0]), int(found[1])), int(found[2] * (1 - margin)), (0, 0, 255), 2)
    out_dir = os.path.join(ROOT, "data", "snapshots")
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, args.camera + "_circle.jpg")
    cv2.imwrite(out, show)
    print("preview %s  (yellow = rim found, red = where the detector stops looking)" % out)
    if args.write:
        path = save_calib("image_circle", {"cameras": {args.camera: {"image_circle": {
            "cx": round(cx, 1), "cy": round(cy, 1), "r": round(r, 1), "margin_frac": margin}}}})
        print("written to %s" % path)
    else:
        print("If the yellow circle sits on the rim in the preview, run again with --write.")


if __name__ == "__main__":
    run_tool(main)
