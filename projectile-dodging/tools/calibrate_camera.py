#!/usr/bin/env python3
"""Measure an ELP camera's lens with a printed checkerboard.

    python3 tools/calibrate_camera.py elp_left --cols 9 --rows 6 --square 0.024

cols x rows are the INNER corners of the board, square is the side of one
square in metres. Show the board to the camera at many angles and positions,
including the edges and corners of the picture; it takes a view every time the
board has moved. After --views views it solves for the lens and writes
config/calib/<camera>_intrinsics.yaml.

Until this is done the config assumes an ideal 120 degree lens, which is good
enough to start with. The fisheye already has its calibration from the lab.
"""
from __future__ import annotations

import argparse
import math

from _common import cv2, load_config, np, open_feed, run_tool, wait_first

from dodge.config import save_calib


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("camera")
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--square", type=float, default=0.024)
    ap.add_argument("--views", type=int, default=25)
    ap.add_argument("--config", default=None)
    args = ap.parse_args()
    cfg = load_config(args.config)
    if cfg["cameras"][args.camera].get("model") != "pinhole":
        raise SystemExit("This tool is for the ELP cameras. The fisheye uses the lab's double-sphere calibration.")
    feed = open_feed(cfg, args.camera, max_fps=8)
    wait_first(feed)
    pattern = (args.cols, args.rows)
    objp = np.zeros((args.cols * args.rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2) * args.square
    obj_pts, img_pts, last, size = [], [], None, None
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
    print("Show the checkerboard. Collecting %d views..." % args.views)
    try:
        while len(obj_pts) < args.views:
            item = feed.get(5.0)
            if item is None:
                raise SystemExit("camera stopped delivering frames")
            gray = cv2.cvtColor(item[1], cv2.COLOR_BGR2GRAY)
            size = (gray.shape[1], gray.shape[0])
            found, corners = cv2.findChessboardCorners(gray, pattern, flags)
            if not found:
                continue
            if last is not None and float(np.abs(corners - last).mean()) < 0.03 * size[0]:
                continue                                   # board has not moved enough
            corners = cv2.cornerSubPix(gray, corners, (7, 7), (-1, -1),
                                       (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01))
            obj_pts.append(objp)
            img_pts.append(corners)
            last = corners
            c = corners.reshape(-1, 2).mean(axis=0)
            print("  view %d/%d (board centre at %.0f, %.0f)" % (len(obj_pts), args.views, c[0], c[1]))
    finally:
        feed.close()
    rms, K, dist, _r, _t = cv2.calibrateCamera(obj_pts, img_pts, size, None, None)
    hfov = 2.0 * math.degrees(math.atan(size[0] / 2.0 / K[0, 0]))
    print("\nreprojection error %.2f px (under 1 is good); horizontal field of view about %.0f degrees" % (rms, hfov))
    if rms > 2.5:
        raise SystemExit("Error too large: views were blurred or the board size is wrong. Nothing was written.")
    intr = {"fx": float(K[0, 0]), "fy": float(K[1, 1]), "cx": float(K[0, 2]), "cy": float(K[1, 2]),
            "width": int(size[0]), "height": int(size[1]), "dist": [float(v) for v in dist.reshape(-1)[:5]]}
    path = save_calib(args.camera + "_intrinsics", {"cameras": {args.camera: {"intrinsics": intr}}})
    print("written to %s" % path)


if __name__ == "__main__":
    run_tool(main)
