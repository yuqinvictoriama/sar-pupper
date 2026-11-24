#!/usr/bin/env python3
"""Run the whole dodge pipeline on a recording, offline. No ROS needed.

    python3 tools/replay.py data/recordings/rear-lobs-01
    python3 tools/replay.py data/events/20261124-153012 --out /tmp/frames   # also write annotated frames
    python3 tools/replay.py --labels data/labels.csv                        # score many recordings

It decodes every frame, detects the ball, tracks it and reports what the dodge
node would have decided and when, using the current config (so you can change
thresholds and replay). Works on tools/record.py recordings and on the short
recordings the dodge node saves around each dodge.

labels.csv has one line per recording:  <folder>,<hit|miss|none>
  hit   a throw that would have hit the robot   -> a dodge is expected
  miss  a throw that passes by                  -> no dodge expected
  none  no throw at all                         -> no dodge expected
"""
from __future__ import annotations

import argparse
import csv
import os

from _common import ROOT, cv2, load_config, np

from dodge.core import DodgeCore
from dodge.detect import BallDetector, draw_blobs
from dodge.recording import RecordingReader


def replay(path: str, cfg: dict, out: str | None = None, cameras=None, latency: float = 0.03, quiet: bool = False) -> dict:
    rec = RecordingReader(path)
    core = DodgeCore(cfg, armed=False)
    dets = {name: BallDetector.for_camera(cam, cfg["detector"]) for name, cam in cfg["cameras"].items()}
    imu = rec.imu()
    imu_i = 0
    stats = {cam: {"frames": 0, "with_ball": 0} for cam in rec.cameras}
    events, first_ball, t_first, t_last = [], None, None, None
    confirmed_time = 0.0
    last_t = None
    if out:
        os.makedirs(out, exist_ok=True)
    for t_raw, cam, jpeg in rec.frames(cameras):
        if cam not in dets:
            continue
        t = t_raw + float(cfg["cameras"][cam].get("time_offset_s", 0.0))
        t_now = t + latency
        while imu_i < len(imu) and imu[imu_i][0] <= t_now:
            core.set_imu(imu[imu_i][0], imu[imu_i][1], imu[imu_i][2])
            imu_i += 1
        bgr = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            continue
        t_first = t if t_first is None else t_first
        blobs = dets[cam].detect(bgr)
        core.process_blobs(cam, t, blobs, t_now, (bgr.shape[1], bgr.shape[0]))
        core.command(t_now)
        stats[cam]["frames"] += 1
        stats[cam]["with_ball"] += 1 if blobs else 0
        if blobs and first_ball is None:
            first_ball = t
        if core.tracker.confirmed() and last_t is not None:
            confirmed_time += max(0.0, min(0.1, t - last_t))
        last_t, t_last = t, t
        for e in core.pop_events():
            e["t_rel"] = round(e["t"] - t_first, 3)
            events.append(e)
            if not quiet:
                if e["event"] == "dodge":
                    th, pl = e["threat"], e["plan"]
                    print("  %7.3f s  DODGE  ball from %+.0f deg, %.2f s to contact, range %.1f m -> vx %+.1f vy %+.1f for %.2f s (%s)%s" % (
                        e["t_rel"], th["bearing_deg"], th["tau"], th["range_m"], pl["vx"], pl["vy"], pl["duration"],
                        pl["reason"], "  [late]" if e.get("late") else ""))
                else:
                    print("  %7.3f s  %s" % (e["t_rel"], e["event"]))
        if out and (blobs or stats[cam]["frames"] % 10 == 0):
            snap = core.snapshot(t_now)
            show = draw_blobs(bgr, blobs)
            cv2.putText(show, "%s %.3f %s" % (cam, t - t_first, snap["state"]), (10, 28), cv2.FONT_HERSHEY_SIMPLEX,
                        0.8, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.imwrite(os.path.join(out, "%09.3f_%s.jpg" % (t - t_first, cam)), show)
    rec.close()
    dodges = [e for e in events if e["event"] == "dodge"]
    result = {"path": path, "duration": (t_last - t_first) if t_first is not None else 0.0, "stats": stats,
              "events": events, "dodges": len(dodges), "tracked_s": confirmed_time,
              "first_ball": None if first_ball is None else first_ball - t_first}
    if not quiet:
        print("  %.1f s, " % result["duration"] + ", ".join(
            "%s %d frames (%d with a ball-coloured blob)" % (c, s["frames"], s["with_ball"]) for c, s in stats.items()))
        print("  ball tracked for %.2f s; %d dodge(s)" % (confirmed_time, len(dodges)))
    return result


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording", nargs="?")
    ap.add_argument("--labels", help="CSV of <folder>,<hit|miss|none> to score")
    ap.add_argument("--out", help="write annotated frames here")
    ap.add_argument("--cameras", default="")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()
    cfg = load_config(args.config)
    cams = [c.strip() for c in args.cameras.split(",") if c.strip()] or None
    if args.labels:
        base = os.path.dirname(os.path.abspath(args.labels))
        with open(args.labels) as f:
            rows = [r for r in csv.reader(f) if len(r) >= 2 and not r[0].startswith("#")]
        tally = {"hit": [0, 0], "miss": [0, 0], "none": [0, 0]}
        for folder, label in ((r[0].strip(), r[1].strip().lower()) for r in rows):
            p = folder if os.path.isabs(folder) else os.path.join(base, folder)
            if not os.path.isdir(p):
                p = os.path.join(ROOT, folder)
            r = replay(p, cfg, cameras=cams, quiet=True)
            dodged = r["dodges"] > 0
            tally.setdefault(label, [0, 0])
            tally[label][0] += 1
            tally[label][1] += 1 if dodged else 0
            ok = dodged == (label == "hit")
            print("%-5s %-4s %s  (%s)" % ("ok" if ok else "WRONG", label, folder, "dodged" if dodged else "no dodge"))
        print("\nhits dodged:            %d of %d" % (tally["hit"][1], tally["hit"][0]))
        print("misses dodged anyway:   %d of %d" % (tally["miss"][1], tally["miss"][0]))
        print("dodges with no throw:   %d of %d" % (tally["none"][1], tally["none"][0]))
        return
    if not args.recording:
        ap.error("give a recording folder, or --labels")
    print(args.recording)
    replay(args.recording, cfg, out=args.out, cameras=cams)


if __name__ == "__main__":
    main()
