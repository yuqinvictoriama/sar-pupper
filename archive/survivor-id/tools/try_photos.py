#!/usr/bin/env python3
"""Run the pose model and the arms-up rule on photos, and save what it saw.

    python3 tools/try_photos.py photo1.jpg folder_of_photos/ --out /tmp/out
    python3 tools/try_photos.py --camera fisheye frame.jpg      # a frame saved from one of Pupper's cameras

Works on a laptop (pip install ultralytics; slow but fine) and on the robot (Hailo; stop the
other vision nodes first, the accelerator serves one program at a time).

An ordinary photo says nothing about how far away people are, so for photos the range limits
are not applied and only the pose is judged (--hfov is the photo's guessed field of view; it
matters little). With --camera the real lens model, mounting and range limits are used.
"""
from __future__ import annotations

import argparse
import copy
import glob
import os

import cv2
import numpy as np

import _common
import draw
from survivor.cameras import FrameView, Rig
from survivor.config import load_config
from survivor.gesture import evaluate
from survivor.person import observe


def photo_view(width: int, height: int, hfov_deg: float):
    from dodge.camera_models import Camera
    cfg = {"model": "pinhole", "intrinsics": {"hfov_deg": hfov_deg, "width": width, "height": height},
           "extrinsics": {"x": 0.0, "y": 0.0, "z": 0.0, "yaw_deg": 0.0}}
    return FrameView(Camera.from_config("photo", cfg))


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("paths", nargs="+", help="image files or folders")
    ap.add_argument("--out", default="try_photos_out")
    ap.add_argument("--backend", default=_common.default_backend(), choices=("hailo", "torch"))
    ap.add_argument("--weights", default=None, help=".hef (hailo) or .pt (torch); default from the config")
    ap.add_argument("--camera", default=None, help="fisheye, elp_left or elp_right: the frame came from that camera")
    ap.add_argument("--hfov", type=float, default=60.0)
    args = ap.parse_args()

    cfg = load_config()
    files = []
    for p in args.paths:
        files += sorted(glob.glob(os.path.join(p, "*"))) if os.path.isdir(p) else [p]
    pose = _common.make_pose(args.backend, cfg, args.weights)
    rig = Rig(cfg) if args.camera else None
    rule = cfg
    if not args.camera:
        rule = copy.deepcopy(cfg)
        rule["gesture"]["min_range_m"], rule["gesture"]["max_range_m"] = 0.0, 1e9
    os.makedirs(args.out, exist_ok=True)
    R = np.eye(3)
    total = signals = 0
    for f in files:
        img = cv2.imread(f)
        if img is None:
            continue
        if rig:
            view = rig.wide_view(args.camera, (img.shape[1], img.shape[0]))
            picture = view.render(img)
        else:
            view, picture = photo_view(img.shape[1], img.shape[0], args.hfov), img
        dets = pose(picture)
        canvas = picture.copy()
        verdicts = []
        for d in dets:
            obs = observe(d, view, R, 0.0, rule)
            res = evaluate(obs, rule)
            draw.person(canvas, obs, res, conf_thr=float(cfg["gesture"]["kpt_conf"]), show_range=bool(rig))
            verdicts.append("UP" if res.up else res.why)
            signals += int(res.up)
        total += len(dets)
        cv2.imwrite(os.path.join(args.out, os.path.splitext(os.path.basename(f))[0] + ".jpg"), canvas)
        print("%-40s %d people  %s" % (os.path.basename(f), len(dets), "; ".join(verdicts)))
    pose.close()
    print("\n%d pictures, %d people, %d showing the signal. Annotated copies in %s" % (len(files), total, signals, args.out))


if __name__ == "__main__":
    main()
