#!/usr/bin/env python3
"""Play a four-channel recording through the partner's localize_angle.py, unchanged, as the robot would.

    python3 tools/sound_replay.py recordings/dji_channels_20261127_101500.wav
    python3 tools/sound_replay.py calls.wav --truth 15 --threshold 0.02

The recording is any 16-bit WAV with the four DJI channels in order (test_channels.py saves one;
`arecord -D hw:<card> -c 4 -r 48000 -f S16_LE calls.wav` makes one on the Pupper). For each sound
event it prints what his script printed and what the bridge (robot/dji_bearing.py) makes of it: the
check, the confidence, and whether navigation would use it. --truth gives the real direction
(degrees, positive RIGHT, as his script prints) to see the error.

Needs numpy only (a stand-in replaces sounddevice). Settings come from config/demo.yaml, section
dji; --threshold and the like override them for this run.
"""
import argparse
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "robot"))

from dji_bearing import Localizer  # noqa: E402
from nav.config import load_config  # noqa: E402


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("wav", type=Path)
    ap.add_argument("--truth", type=float, help="the real bearing, degrees, positive right")
    ap.add_argument("--threshold", type=float)
    ap.add_argument("--frame-seconds", type=float)
    ap.add_argument("--speed", type=float, default=20.0, help="times faster than real time")
    ap.add_argument("--config")
    args = ap.parse_args()
    cfg = load_config(args.config)
    if args.threshold is not None:
        cfg.dji.threshold = args.threshold
    if args.frame_seconds is not None:
        cfg.dji.frame_seconds = args.frame_seconds
    env = dict(os.environ, PYTHONPATH=str(ROOT / "tests" / "fake_sounddevice"),
               FAKE_SD_AUDIO=str(args.wav.resolve()), FAKE_SD_SPEED=str(args.speed))
    use_from = cfg.sound.min_confidence
    rows = []

    def show(r, _t):
        rows.append(r)
        used = r["bearing"] is not None and r["confidence"] >= use_from
        e = ""
        if args.truth is not None and r["bearing"] is not None:
            e = "  error %+.1f" % ((r["bearing"] - args.truth + 180) % 360 - 180)
        print("%-44s fit %.2f  residual %+6.2f  rms %.4f  %-12s %s%s" % (
            "bearing " + r["text"], r["fit"], r["residual"], r["rms"], r["check"],
            "USED" if used else "not used", e))

    loc = Localizer(cfg.dji, on_reading=show, python=sys.executable, env=env)
    print("running:", " ".join(loc.cmd))
    loc.start()
    loc.proc.wait()
    loc.reader.join(5)
    for line in loc.log:
        if line.startswith(("Could not", "Error", "Four-channel", "Selected")):
            print(line)
    used = [r for r in rows if r["bearing"] is not None and r["confidence"] >= use_from]
    print("%d sound events, %d bearings navigation would use (sound.min_confidence %.2f)" % (
        len(rows), len(used), use_from))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
