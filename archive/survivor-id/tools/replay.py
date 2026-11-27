#!/usr/bin/env python3
"""Run survivor identification on a recording, off the robot or on it.

    python3 tools/replay.py data/recordings/20261124-101500                    # events and a timeline
    python3 tools/replay.py REC --frames /tmp/replay                           # also a picture per cycle
    python3 tools/replay.py REC --backend torch --weights yolov8s-pose.pt      # on a laptop (pip install ultralytics)

A recording is a folder in the dodging component's format (all three cameras' JPEGs and the
IMU). Make one with  robot/start.sh --record data/recordings  (the survivor node saves what
it looks at), or with projectile-dodging/tools/record.py.

This is how thresholds get tuned (plan, Phase 2): change config/survivor.yaml, replay the
same recordings, compare the events. Tune on what the ROBOT's model returns (--backend hailo,
on the Pupper): the laptop's float model is a little more confident than the quantised one.

Not replayed: /cmd_vel is not in a recording, so while the robot walks with nobody in view the
pin does not move with it. Bearings and turns are right (they come from the cameras and IMU).
"""
from __future__ import annotations

import argparse
import os

import cv2
import numpy as np

import _common
import overlay
from survivor.config import load_config
from survivor.perception import SurvivorPerception


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    ap.add_argument("--backend", default=_common.default_backend(), choices=("hailo", "torch"))
    ap.add_argument("--weights", default=None)
    ap.add_argument("--config", default=None)
    ap.add_argument("--frames", default=None, help="folder for an annotated picture per cycle")
    ap.add_argument("--every", type=int, default=1, help="with --frames: save every Nth cycle")
    args = ap.parse_args()

    cfg = load_config(args.config)
    from dodge.recording import RecordingReader
    rec = RecordingReader(args.recording)
    cams = [c for c in rec.cameras if c in cfg["cameras"]]
    if not cams:
        raise SystemExit("no frames from %s in %s (it has: %s)" % (list(cfg["cameras"]), args.recording, rec.cameras))
    pose = _common.make_pose(args.backend, cfg, args.weights)
    core = SurvivorPerception(cfg)
    imu = rec.imu()
    period = 1.0 / float(cfg["budget"]["cycle_hz"])
    if args.frames:
        os.makedirs(args.frames, exist_ok=True)

    i_imu, next_step, cycle, t0, last_phase = 0, None, 0, None, None
    events = []
    for t, cam, jpeg in rec.frames(cams):
        t0 = t if t0 is None else t0
        while i_imu < len(imu) and imu[i_imu][0] <= t:
            core.set_imu(imu[i_imu][0], imu[i_imu][1])
            i_imu += 1
        data = np.frombuffer(jpeg, dtype=np.uint8)
        core.set_frame(cam, t, lambda d=data: cv2.imdecode(d, cv2.IMREAD_COLOR))
        next_step = t if next_step is None else next_step
        if t < next_step:
            continue
        next_step += period
        core.step(t, lambda image, view: pose(image))
        cycle += 1
        for e in core.pop_events():
            events.append(e)
            print("%7.2f s  EVENT %s" % (t - t0, e))
        phase = core.phase(t)
        if phase != last_phase:
            print("%7.2f s  %s" % (t - t0, phase))
            last_phase = phase
        if args.frames and cycle % args.every == 0:
            picture = overlay.mosaic(core, t, note="%.2f s" % (t - t0))
            if picture is not None:
                cv2.imwrite(os.path.join(args.frames, "%05d.jpg" % cycle), picture)
    pose.close()
    names = [e["event"] for e in events]
    print("\n%d cycles over %.1f s from %s. Events: %s" % (
        cycle, 0.0 if t0 is None else t - t0, ", ".join(cams), ", ".join(names) or "none"))
    if args.frames:
        print("pictures in %s" % args.frames)


if __name__ == "__main__":
    main()
