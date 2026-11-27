#!/usr/bin/env python3
"""What the motors' start-up homing did, and the battery. Moves nothing.

    python3 robot/homing_report.py                  # after robot/start.sh; reads /tmp/sar-dodge/robot.log
    python3 robot/homing_report.py --battery-only   # just the battery
    python3 robot/homing_report.py --log some.log

Every time the robot stack starts, the course's motor driver finds each joint's zero by driving it
slowly against its end stop: first the four knees, then the four hips (the fore-and-aft swing), then
the four abductors. It decides a joint has reached its stop when the motor falls about 0.36 rad behind
where it was told to be. Anything that holds a leg back before the real stop gives the same signal:
the floor or a mat the leg drags on, a cable, a part bolted on beside it, the robot lying on one
side, or a battery too low for the motor to keep up. The joint's zero is then wrong by however far
it still had to go, and that leg stands further forward or back than the others for the whole run.
Nothing can re-measure it afterwards except homing again: robot/stop.sh, then robot/start.sh with
the legs free.

This prints how long each joint took to reach its stop (the driver logs the moments) and points
out left-right pairs that differ a lot, which is what a held-back leg looks like. A difference can
also just mean the legs lay differently when it started, so it is a pointer, not proof: the test is
whether the legs LOOK even once homing has finished.
"""
from __future__ import annotations

import argparse
import os
import re
import subprocess
import sys

JOINTS = ["front right abductor", "front right hip", "front right knee",
          "front left abductor", "front left hip", "front left knee",
          "back right abductor", "back right hip", "back right knee",
          "back left abductor", "back left hip", "back left knee"]
STAGES = [("knees", [2, 5, 8, 11]), ("hips (fore and aft)", [1, 4, 7, 10]), ("abductors (sideways)", [0, 3, 6, 9])]
PAIRS = [(2, 5), (8, 11), (1, 4), (7, 10), (0, 3), (6, 9)]          # right, left of the same leg pair and joint
HOMING_SPEED = 1.5        # rad/s, pupper_v3_description
UNEVEN_S = 0.30           # a left-right pair this far apart is worth a look (0.45 rad of travel)
LINE = re.compile(r"\[(\d+\.\d+)\] \[ControlBoardHardwareInterface\]: (Homing actuators|Homed actuator (\d+)|Finished homing)")
BATTERY_TOOL = os.path.expanduser("~/pupperv3-monorepo/robot/utils/check_batt_voltage.py")
BATTERY_WARN_V = 16.5     # the course's own monitor warns below this (3.3 V a cell, five cells) ...
BATTERY_OFF_V = 15.5      # ... and switches the Pi off below this


def parse(text: str) -> dict:
    """The LAST homing run in a log: {"start": t, "homed": {joint: t}, "finished": t or None}, or {} if none."""
    run: dict = {}
    for m in LINE.finditer(text):
        t = float(m.group(1))
        if m.group(2).startswith("Homing actuators"):
            run = {"start": t, "homed": {}, "finished": None}
        elif not run:
            continue
        elif m.group(3) is not None:
            run["homed"].setdefault(int(m.group(3)), t)
        else:
            run["finished"] = t
    return run


def times(run: dict) -> dict:
    """{joint: seconds from the start of its stage to reaching its stop} for the joints that got there."""
    out = {}
    stage_start = run["start"]
    for _name, joints in STAGES:
        done = [run["homed"][j] for j in joints if j in run["homed"]]
        for j in joints:
            if j in run["homed"]:
                out[j] = max(0.0, run["homed"][j] - stage_start)
        if len(done) < len(joints):
            break                                   # the next stage only starts once this one is complete
        stage_start = max(done)
    return out


def findings(run: dict) -> list:
    """Plain sentences about what looks wrong; empty if nothing does."""
    if not run:
        return ["The log has no homing in it: the robot stack has not got as far as the motors. See the log for why."]
    notes = []
    took = times(run)
    missing = [JOINTS[j] for j in range(12) if j not in run["homed"]]
    if missing:
        notes.append("Homing has not finished: still waiting for %s. If it stays like this, that motor is not "
                     "answering (its cable, or the battery)." % ", ".join(missing))
    for a, b in PAIRS:
        if a in took and b in took and abs(took[a] - took[b]) >= UNEVEN_S:
            first, second = (a, b) if took[a] < took[b] else (b, a)
            notes.append("The %s reached its stop %.2f s before the %s (about %.1f rad less travel). If the legs "
                         "look uneven, the first one was held back before its real stop."
                         % (JOINTS[first], abs(took[a] - took[b]), JOINTS[second], abs(took[a] - took[b]) * HOMING_SPEED))
    return notes


def battery():
    """(volts, per cent) from the course's battery reader, or None if it cannot be read (not on the robot)."""
    if not os.path.exists(BATTERY_TOOL):
        return None
    try:
        r = subprocess.run([sys.executable, BATTERY_TOOL], capture_output=True, text=True, timeout=8)
    except (subprocess.TimeoutExpired, OSError):
        return None
    m = re.search(r"(\d+)%\s+(\d+\.\d+)V", r.stdout)
    return (float(m.group(2)), int(m.group(1))) if m else None


def battery_line(reading) -> tuple:
    """(text, is it a problem)."""
    if reading is None:
        return "battery: could not be read", False
    volts, pct = reading
    if volts < 5.0:
        return "battery: none (the Pi is on a wall supply; the motors have no power)", True
    text = "battery: %.1f V, about %d %%" % (volts, pct)
    if volts < BATTERY_OFF_V:
        return text + "  <-- EMPTY: below %.1f V the Pi switches itself off. Change it now." % BATTERY_OFF_V, True
    if volts < BATTERY_WARN_V:
        return text + ("  <-- LOW: the motors cannot keep up, so homing stops short and the legs end up uneven; and "
                       "below %.1f V the Pi switches itself off. Change or charge it before starting." % BATTERY_OFF_V), True
    return text, False


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--log", default="/tmp/sar-dodge/robot.log")
    ap.add_argument("--battery-only", action="store_true")
    ap.add_argument("--brief", action="store_true", help="only say something if there is a problem")
    args = ap.parse_args()
    text, low = battery_line(battery())
    problems = 1 if low else 0
    if low or not args.brief:
        print(text)
    if args.battery_only:
        return problems
    try:
        with open(args.log, errors="replace") as f:
            run = parse(f.read())
    except OSError:
        print("no log at %s: start the robot with  bash robot/start.sh" % args.log)
        return 1
    notes = findings(run)
    if run and not args.brief:
        took = times(run)
        for name, joints in STAGES:
            print("homing, %-21s %s" % (name + ":", "   ".join(
                "%s %s" % (JOINTS[j].rsplit(" ", 1)[0], ("%.2f s" % took[j]) if j in took else "--") for j in joints)))
        if run["finished"] is not None:
            print("homing finished in %.1f s" % (run["finished"] - run["start"]))
    for n in notes:
        print("NOTE: " + n)
    if notes:
        print("If the legs are uneven:  bash robot/stop.sh , hold the robot off the floor by its body (or put it on "
              "its stand) so no leg\ntouches anything, then  bash robot/start.sh  and keep holding until the legs stop moving.")
    return problems + (1 if notes else 0)


if __name__ == "__main__":
    sys.exit(main())
