#!/usr/bin/env python3
"""Find the two ELP USB cameras and tell the config which is left and which is right.

    python3 tools/find_cameras.py            # list USB cameras, their modes, save a snapshot of each
    python3 tools/find_cameras.py --assign   # then: cover the LEFT camera when asked; writes config/calib/devices.yaml

Cameras are named by the USB port they are plugged into (/dev/v4l/by-path/...),
because /dev/video numbers change between boots. If you move a camera to
another port, run --assign again.
"""
from __future__ import annotations

import argparse
import glob
import os
import shutil
import subprocess
import time

from _common import ROOT, cv2, load_config

from dodge.config import save_calib


def usb_cameras() -> list:
    out = []
    for path in sorted(glob.glob("/dev/v4l/by-path/*")):
        base = os.path.basename(path)
        if "usb" in base and base.endswith("video-index0"):
            out.append(path)
    return out


def modes(device: str) -> str:
    if not shutil.which("v4l2-ctl"):
        return "  (install v4l-utils to list modes: sudo apt install v4l-utils)"
    text = subprocess.run(["v4l2-ctl", "-d", device, "--list-formats-ext"], capture_output=True, text=True).stdout
    lines, fmt, size = [], "", ""
    for raw in text.splitlines():
        s = raw.strip()
        if s.startswith("["):
            fmt = s.split("'")[1] if "'" in s else s
        elif s.startswith("Size:"):
            size = s.split()[-1]
        elif s.startswith("Interval:") and "(" in s:
            fps = s.split("(")[1].split()[0]
            lines.append((fmt, size, float(fps)))
    best = {}
    for fmt, size, fps in lines:
        best[(fmt, size)] = max(fps, best.get((fmt, size), 0.0))
    return "\n".join("  %-5s %-10s up to %3.0f fps" % (f, s, v) for (f, s), v in sorted(best.items()))


def snapshot(device: str):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    frame = None
    for _ in range(12):          # let auto exposure settle
        ok, f = cap.read()
        if ok:
            frame = f
    cap.release()
    return frame


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--assign", action="store_true", help="work out left/right and write config/calib/devices.yaml")
    args = ap.parse_args()
    cams = usb_cameras()
    if not cams:
        raise SystemExit("No USB cameras found under /dev/v4l/by-path. Are the ELP cameras plugged in?")
    snap_dir = os.path.join(ROOT, "data", "snapshots")
    os.makedirs(snap_dir, exist_ok=True)
    for i, dev in enumerate(cams):
        print("\n[%d] %s -> %s" % (i, dev, os.path.realpath(dev)))
        print(modes(dev))
        frame = snapshot(dev)
        if frame is None:
            print("  could not read a frame (in use by another program?)")
        else:
            out = os.path.join(snap_dir, "usb%d.jpg" % i)
            cv2.imwrite(out, frame)
            print("  snapshot %s  (%dx%d, mean brightness %.0f)" % (out, frame.shape[1], frame.shape[0], frame.mean()))
    if not args.assign:
        print("\nRun again with --assign to set left/right.")
        return
    if len(cams) < 2:
        raise SystemExit("Need two USB cameras to assign left and right; found %d." % len(cams))
    input("\nCover the LEFT rear camera's lens completely with your hand (left = the robot's left), then press Enter...")
    time.sleep(0.5)
    levels = []
    for dev in cams:
        frame = snapshot(dev)
        levels.append(255.0 if frame is None else float(frame.mean()))
        print("  %s brightness %.0f" % (os.path.basename(dev), levels[-1]))
    order = sorted(range(len(cams)), key=lambda i: levels[i])
    if levels[order[1]] - levels[order[0]] < 15:
        raise SystemExit("Could not tell which camera was covered (brightness too similar). Try again in a brighter room.")
    left = cams[order[0]]
    others = [c for c in cams if c != left]
    if len(others) > 1:
        print("More than two USB cameras: taking %s as the right camera." % others[0])
    right = others[0]
    path = save_calib("devices", {"cameras": {"elp_left": {"device": left}, "elp_right": {"device": right}}})
    print("\nleft  = %s\nright = %s\nwritten to %s" % (left, right, path))
    cfg = load_config()
    print("config now has elp_left.device = %s" % cfg["cameras"]["elp_left"]["device"])


if __name__ == "__main__":
    main()
