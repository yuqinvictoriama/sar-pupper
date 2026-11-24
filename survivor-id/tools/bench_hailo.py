#!/usr/bin/env python3
"""Does the pose model load on this robot, what does it return, and how fast is the whole cycle?

    python3 tools/bench_hailo.py                 # on the Pupper, nothing else using the Hailo
    python3 tools/bench_hailo.py --photo p.jpg   # also say who it finds in a photo

Plan, Phase 0 items 1 and 6. It prints:
  - the model's output tensors and their value ranges (scores should be 0..1 or logits; the
    decoder takes either), so a model that is not YOLOv8-pose is obvious at once
  - milliseconds per inference including letterboxing and decoding -> inferences per second,
    the number config/survivor.yaml's budget.inferences_per_s should be set from
  - what the CPU side of a cycle costs: JPEG decode, the fisheye's wide view, a zoom patch
"""
from __future__ import annotations

import argparse
import time

import cv2
import numpy as np

import _common
from survivor.cameras import Rig
from survivor.config import load_config


def timed(fn, n: int) -> float:
    t0 = time.perf_counter()
    for _ in range(n):
        fn()
    return (time.perf_counter() - t0) * 1000.0 / n


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--backend", default=_common.default_backend(), choices=("hailo", "torch"))
    ap.add_argument("--weights", default=None)
    ap.add_argument("--photo", default=None)
    ap.add_argument("-n", type=int, default=100)
    args = ap.parse_args()

    cfg = load_config()
    pose = _common.make_pose(args.backend, cfg, args.weights)
    image = cv2.imread(args.photo) if args.photo else None
    if image is None:
        image = np.random.default_rng(0).integers(0, 255, (720, 1280, 3), dtype=np.uint8)

    if hasattr(pose, "output_shapes"):
        pose(image)
        print("output tensors (want 3 grids x [64, 1, 51] channels):")
        for name, shape in pose.output_shapes.items():
            buf = pose._bindings.output(name).get_buffer()
            print("  %-40s %-16s min %8.3f  max %8.3f" % (name, shape, float(buf.min()), float(buf.max())))
    people = pose(image)
    print("people found: %d %s" % (len(people), [round(p.score, 2) for p in people]))

    pose(image)
    ms = timed(lambda: pose(image), args.n)
    print("\ninference incl. letterbox and decode: %.1f ms  ->  %.0f per second" % (ms, 1000.0 / ms))
    if hasattr(pose, "last_inference_ms"):
        print("  of which on the chip: %.1f ms" % pose.last_inference_ms)
    print("  config budget.inferences_per_s is %s; set it to about 80 %% of the measured rate"
          % cfg["budget"]["inferences_per_s"])

    rig = Rig(cfg)
    print("\nCPU side of a cycle:")
    for name, cam in cfg["cameras"].items():
        intr = cam["intrinsics"]
        frame = np.random.default_rng(1).integers(0, 255, (int(intr["height"]), int(intr["width"]), 3), dtype=np.uint8)
        ok, jpeg = cv2.imencode(".jpg", cv2.GaussianBlur(frame, (0, 0), 3), [cv2.IMWRITE_JPEG_QUALITY, 85])
        wide = rig.wide_view(name)
        patch = rig.patch_view(name, None, rig.cameras[name].R[:, 2], 40.0, int(cfg["budget"]["patch_px"]))
        wide.render(frame), patch.render(frame)          # build the lookup tables first
        print("  %-10s JPEG decode %5.1f ms   wide view %5.1f ms   zoom patch %5.1f ms" % (
            name, timed(lambda: cv2.imdecode(jpeg, cv2.IMREAD_COLOR), 20), timed(lambda: wide.render(frame), 20),
            timed(lambda: patch.render(frame), 20)))
    cycle_hz = float(cfg["budget"]["cycle_hz"])
    print("  (a cycle at %.0f Hz has %.0f ms; one wide pass and one or two zoom passes must fit)" % (cycle_hz, 1000.0 / cycle_hz))
    pose.close()


if __name__ == "__main__":
    main()
