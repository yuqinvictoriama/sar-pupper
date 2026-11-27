#!/usr/bin/env python3
"""How long does each camera's work take on this computer? Measured on saved throws, no robot needed.

    python3 tools/bench_detect.py data/throws/20261126-155217-dodge [more folders]
    python3 tools/bench_detect.py data/throws/20261126-155217-dodge --profile elp_left

For every camera in the recordings it decodes each saved picture and searches it
for the ball, exactly as the camera's process does on the robot, and prints the
time per picture for each step and how many blobs came out. Run it on the
Pupper with the robot stack stopped to see what the Pi can do when nothing else
wants the processor; "frames a second possible" is then the ceiling for
max_process_fps. --profile shows where the search spends its time.
"""
from __future__ import annotations

import argparse
import cProfile
import os
import pstats
import sys
import time

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from dodge.config import load_config  # noqa: E402
from dodge.pipeline import FramePipeline  # noqa: E402
from dodge.recording import RecordingReader  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recordings", nargs="+")
    ap.add_argument("--config", default=None)
    ap.add_argument("--profile", default="", metavar="CAMERA", help="print where this camera's search spends its time")
    ap.add_argument("--threads", type=int, default=1, help="OpenCV threads (the node uses 1)")
    args = ap.parse_args()
    cv2.setNumThreads(args.threads)
    cfg = load_config(args.config)
    frames: dict = {}
    for path in args.recordings:
        reader = RecordingReader(path)
        absent = [c for c in reader.cameras if not os.path.exists(os.path.join(path, c + ".bin"))]
        if absent:                           # only the small files were copied off the robot
            print("skipped %s: no pictures for %s" % (path, ", ".join(absent)), file=sys.stderr)
            continue
        for cam in reader.cameras:
            if cam in cfg["cameras"]:
                frames.setdefault(cam, []).extend(reader.read(cam, off, ln) for _t, off, ln in reader.index[cam])
        reader.close()
    print("%-10s %7s %9s %10s %10s %10s %7s %s" % ("camera", "frames", "size", "decode ms", "search ms", "worst ms", "blobs", "frames a second possible"))
    for cam, jpegs in frames.items():
        pipe = FramePipeline(cam, cfg)
        dec, det, blobs, size = [], [], [], (0, 0)
        prof = cProfile.Profile() if args.profile == cam else None
        full = 0
        for i, data in enumerate(jpegs):
            if prof:
                prof.enable()
            res = pipe.process_jpeg(float(i) / 30.0, np.frombuffer(data, np.uint8))
            if prof:
                prof.disable()
            if i >= 2:                       # the first pictures set up the colour balance and caches
                dec.append(res.decode_ms)
                det.append(res.detect_ms)
                blobs.append(len(res.blobs))
                full += bool(res.blobs)
            size = res.size
        if not det:
            continue
        total = float(np.mean(dec) + np.mean(det))
        print("%-10s %7d %9s %10.1f %10.1f %10.1f %7.1f %.0f" % (
            cam, len(jpegs), "%dx%d" % size, np.mean(dec), np.mean(det), np.max(det), np.mean(blobs), 1000.0 / total))
        print("%-10s the slowest picture took %.1f ms (decode + search); %d of %d pictures had a blob to pass on" % (
            "", float(np.max(np.array(dec) + np.array(det))), full, len(dec)))
        if prof:
            pstats.Stats(prof).sort_stats("cumulative").print_stats(22)


if __name__ == "__main__":
    main()
