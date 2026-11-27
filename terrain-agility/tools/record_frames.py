#!/usr/bin/env python3
"""Save camera frames (with the IMU attitude at that moment) for calibrating and checking
off the robot.

    python3 tools/record_frames.py data/floor            # 10 frames
    python3 tools/record_frames.py data/box40 -n 20

Every other tool takes --frames <folder> to work from these instead of the live camera.
"""
import argparse

import _frames


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("folder")
    ap.add_argument("-n", type=int, default=10)
    ap.add_argument("--topic", default="/camera/image_raw/compressed")
    args = ap.parse_args()
    frames = _frames.grab(args.n, topic=args.topic)
    _frames.save(frames, args.folder)
    h, w = frames[0][0].shape[:2]
    print(f"saved {len(frames)} frames ({w} x {h}) to {args.folder}")


if __name__ == "__main__":
    main()
