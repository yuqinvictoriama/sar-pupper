#!/usr/bin/env python3
"""Measure an ELP camera's lens with a checkerboard (printed, or shown on a laptop or tablet screen).

    python3 tools/calibrate_camera.py --make-board board.png     # a board to show full screen
    python3 tools/calibrate_camera.py elp_left --cols 9 --rows 6 --square 0.024

cols x rows are the INNER corners of the board (the --make-board picture has
9 x 6), square is the side of one square in metres (it does not matter for the
lens; any value will do). Show the board to the camera at many angles and
positions; it takes a view every time the board has moved. After --views views
it solves for the lens and writes config/calib/<camera>_intrinsics.yaml.

THE EDGES MATTER MOST. Straight behind the robot the ball is at the outer edge
of both rear pictures (the left edge of elp_left, the right edge of elp_right),
high up. Hold the board there, and in the top corners, for at least a third of
the views. The tool says at the end how far out the views reached and refuses
a lens fit that bends back inside the picture.

Until this is done the config assumes an ideal 120 degree lens. In simulation,
a lens 4 degrees off at the edge of the picture (ordinary barrel distortion)
cut the dodges of lobs from straight behind from 97 % to 60 %. The fisheye
already has its calibration from the lab.
"""
from __future__ import annotations

import argparse
import math

from _common import cv2, load_config, np, open_feed, run_tool, wait_first

from dodge.config import save_calib


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("camera", nargs="?")
    ap.add_argument("--make-board", metavar="PNG", help="write a checkerboard picture to show full screen, and stop")
    ap.add_argument("--cols", type=int, default=9)
    ap.add_argument("--rows", type=int, default=6)
    ap.add_argument("--square", type=float, default=0.024)
    ap.add_argument("--views", type=int, default=25)
    ap.add_argument("--config", default=None)
    args = ap.parse_args()
    if args.make_board:
        make_board(args.make_board, args.cols, args.rows)
        return
    if not args.camera:
        ap.error("name the camera (elp_left or elp_right), or use --make-board")
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
    pts = np.concatenate([p.reshape(-1, 2) for p in img_pts])
    print("\nthe views reached x %.0f-%.0f and y %.0f-%.0f of %d x %d" % (pts[:, 0].min(), pts[:, 0].max(),
                                                                       pts[:, 1].min(), pts[:, 1].max(), size[0], size[1]))
    reach = (pts[:, 0].min() / size[0], 1.0 - pts[:, 0].max() / size[0], pts[:, 1].min() / size[1])
    if max(reach) > 0.15:
        print("NOTE: the board never came near the %s of the picture. Directions there are extrapolated; "
              "straight behind the robot the ball is at the outer edge, high up. Better to run it again."
              % ", ".join(n for n, r in zip(("left edge", "right edge", "top"), reach) if r > 0.15))
    fit = None
    for flags_cal, what in ((0, "five terms"), (cv2.CALIB_FIX_K3, "k3 fixed at 0")):
        rms, K, dist, _r, _t = cv2.calibrateCamera(obj_pts, img_pts, size, None, None, flags=flags_cal)
        d = [float(v) for v in dist.reshape(-1)[:5]]
        bad = lens_folds(K, d, size)
        print("fit with %s: reprojection error %.2f px%s" % (what, rms, "; bends back inside the picture" if bad else ""))
        if not bad:
            fit = (rms, K, d)
            break
    if fit is None:
        raise SystemExit("The lens fit bends back inside the picture (too few views near the edges). Nothing was written.")
    rms, K, d = fit
    hfov = 2.0 * math.degrees(math.atan(size[0] / 2.0 / K[0, 0]))
    print("reprojection error %.2f px (under 1 is good); horizontal field of view about %.0f degrees (before distortion)" % (rms, hfov))
    if rms > 2.5:
        raise SystemExit("Error too large: views were blurred or the board size is wrong. Nothing was written.")
    intr = {"fx": float(K[0, 0]), "fy": float(K[1, 1]), "cx": float(K[0, 2]), "cy": float(K[1, 2]),
            "width": int(size[0]), "height": int(size[1]), "dist": d}
    path = save_calib(args.camera + "_intrinsics", {"cameras": {args.camera: {"intrinsics": intr}}})
    print("written to %s" % path)


def lens_folds(K, dist, size) -> bool:
    """True if the fitted distortion cannot be inverted somewhere inside the picture (the polynomial
    turns back before the corners), so directions there would be nonsense."""
    from dodge.camera_models import Pinhole
    m = Pinhole(K[0, 0], K[1, 1], K[0, 2], K[1, 2], size[0], size[1], dist)
    w, h = size
    edge = np.array([[x, y] for x in np.linspace(0, w - 1, 41) for y in (0, h - 1)] +
                    [[x, y] for y in np.linspace(0, h - 1, 25) for x in (0, w - 1)], dtype=float)
    _rays, ok = m.unproject(edge)
    return not bool(ok.all())


def make_board(path: str, cols: int, rows: int, square_px: int = 120) -> None:
    """A (cols + 1) x (rows + 1) checkerboard with a white border, to show full screen."""
    board = np.full(((rows + 3) * square_px, (cols + 3) * square_px), 255, np.uint8)
    for r in range(rows + 1):
        for c in range(cols + 1):
            if (r + c) % 2 == 0:
                y, x = (r + 1) * square_px, (c + 1) * square_px
                board[y:y + square_px, x:x + square_px] = 0
    cv2.imwrite(path, board)
    print("wrote %s: show it full screen on a laptop or tablet, screen brightness up, and hold the screen still "
          "in front of the camera at many angles. %d x %d inner corners: use --cols %d --rows %d." % (path, cols, rows, cols, rows))


if __name__ == "__main__":
    run_tool(main)
