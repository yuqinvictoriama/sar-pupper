#!/usr/bin/env python3
"""Try the "You saved me, Pupper!" detector without ROS.

  python3 tools/keyword_test.py --list                    # which microphones are there
  python3 tools/keyword_test.py --wav a.wav b.wav         # run on recordings
  python3 tools/keyword_test.py --live                    # listen now, with a level meter
  python3 tools/keyword_test.py --live --device DJI --channels 2 --record heard.wav

Use --record on the robot, with the survivor behind the real cover, and keep
the file: it is what thresholds get tuned on afterwards.
"""
import argparse
import sys
import time
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
from keyphrase import PhraseSpotter  # noqa: E402
from keyphrase.audio import list_input_devices, mic_blocks, read_wav, wav_blocks, write_wav  # noqa: E402


def meter(db: float) -> str:
    filled = int(np.clip((db + 70) / 70 * 30, 0, 30))
    return "#" * filled + "." * (30 - filled)


def main() -> int:
    cfg = yaml.safe_load((HERE / "config" / "keyword.yaml").read_text())["audio"]
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--list", action="store_true", help="list microphones and exit")
    ap.add_argument("--wav", nargs="+", help="recordings to run the detector on")
    ap.add_argument("--live", action="store_true", help="listen on a microphone")
    ap.add_argument("--device", default=cfg["device"])
    ap.add_argument("--channels", type=int, default=cfg["channels"])
    ap.add_argument("--rate", type=int, default=cfg["sample_rate"])
    ap.add_argument("--seconds", type=float, default=0, help="stop after this long (0 = until Ctrl+C)")
    ap.add_argument("--record", help="also save what was heard to this WAV file")
    args = ap.parse_args()

    if args.list:
        for i, name, ch, rate in list_input_devices():
            print(f"{i:3d}  {ch} ch  {rate:.0f} Hz  {name}")
        return 0

    if args.wav:
        total = 0
        for path in args.wav:
            pcm, rate = read_wav(path)
            spotter = PhraseSpotter(channels=pcm.shape[1])
            hits = [h for block, r in wav_blocks(path) for h in spotter.feed(block, r)]
            total += len(hits)
            what = ", ".join(f"{h.phrase} at {h.time_s:.1f} s (mic {h.channel})" for h in hits) or "nothing"
            print(f"{path}: {len(pcm) / rate:.1f} s, {pcm.shape[1]} ch, {rate} Hz -> {what}")
        return 0 if total else 1

    if args.live:
        spotter = PhraseSpotter(channels=args.channels)
        kept, start, last, count = [], time.time(), 0.0, 0
        print("listening. Say: You saved me, Pupper!   (Ctrl+C to stop)")
        try:
            for block, rate in mic_blocks(args.device, args.channels, args.rate):
                if args.record:
                    kept.append(block)
                for h in spotter.feed(block, rate):
                    count += 1
                    print(f"\n>>> HEARD IT ({h.phrase}, mic {h.channel}) at {time.time() - start:.1f} s")
                now = time.time()
                if now - last > 0.2:
                    last = now
                    loud = max(spotter.level_db)
                    print(f"\r{meter(loud)} {loud:6.1f} dB   heard {count}", end="", flush=True)
                if args.seconds and now - start > args.seconds:
                    break
        except KeyboardInterrupt:
            pass
        print()
        if args.record and kept:
            write_wav(args.record, np.concatenate(kept), args.rate)
            print(f"saved {args.record}")
        return 0

    ap.print_help()
    return 2


if __name__ == "__main__":
    sys.exit(main())
