#!/usr/bin/env python3
"""Find the two ELP USB cameras, check they work, and tell the config which is left and which is right.

    python3 tools/find_cameras.py            # what is plugged in, whether each camera works, and why not if not
    python3 tools/find_cameras.py --assign   # then: cover the LEFT camera when asked; writes config/calib/devices.yaml
    python3 tools/find_cameras.py --both     # run both rear cameras at once, the way the dodge node does

Cameras are named by the USB socket they are plugged into (/dev/v4l/by-path/...),
because /dev/video numbers change between boots. If you move a plug, run
--assign again. Stop the dodge node, view.py and record.py first: a USB camera
serves one program at a time.
"""
from __future__ import annotations

import argparse
import os
import shutil
import subprocess
import time

from _common import ROOT, cv2, load_config, run_tool

from usb_camera import (DODGE_NODE, FREE_THE_CAMERAS, UsbCamera, apply_controls, open_problem, usb_camera_nodes,
                        who_holds)

from dodge.config import save_calib

REAR = ("elp_left", "elp_right")


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
    """One picture with the camera's own automatic exposure (so it is viewable in any light). None if it will not deliver."""
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    apply_controls(device, {"auto_exposure": True}, cap, log=lambda _m: None)
    frame = None
    end = time.time() + 6.0
    got = 0
    while got < 25 and time.time() < end:      # let the automatic exposure settle
        ok, f = cap.read()
        if ok and f is not None:
            frame = f
            got += 1
    cap.release()
    return frame


def power_notes() -> list:
    """What the Pi itself reports about USB power. Lines to print; empty when nothing stands out."""
    notes = []
    if shutil.which("vcgencmd"):
        def vc(*a):
            try:
                return subprocess.run(["vcgencmd", *a], capture_output=True, text=True, timeout=3).stdout.strip()
            except (subprocess.TimeoutExpired, OSError):
                return ""
        limit = vc("get_config", "usb_max_current_enable")
        if limit.endswith("=0"):
            notes.append("The Pi is limiting all four USB sockets together to 0.6 A (usb_max_current_enable=0). It does "
                         "that when it is not powered by a 5 A USB-C supply, as on the robot. Two cameras can exceed it. "
                         "To allow 1.6 A: add the line  usb_max_current_enable=1  to /boot/firmware/config.txt and reboot.")
        throttled = vc("get_throttled")
        try:
            bits = int(throttled.split("=")[1], 16)
        except (IndexError, ValueError):
            bits = 0
        if bits & 0x10001:                       # bit 0: under-voltage now, bit 16: since boot
            notes.append("The Pi has seen low supply voltage since it booted (%s): USB devices can drop out when that "
                         "happens. Check the battery." % throttled)
    return notes


def usb_health() -> list:
    """Power notes plus recent kernel complaints about USB: printed when a camera is not working."""
    notes = power_notes()
    try:
        log = subprocess.run(["dmesg"], capture_output=True, text=True, timeout=3)
        lines = log.stdout.splitlines() if log.returncode == 0 else []
    except (subprocess.TimeoutExpired, OSError):
        lines = []
    bad = [ln for ln in lines if any(k in ln for k in ("over-current", "Not enough bandwidth", "uvcvideo: Failed",
                                                      "device descriptor read", "USB disconnect", "error -71", "error -110", "error -32"))]
    if bad:
        notes.append("Recent kernel messages about USB (newest last):")
        notes.extend("    " + ln for ln in bad[-6:])
    elif not lines:
        notes.append("(Could not read the kernel log. If a camera misbehaves, look at:  sudo dmesg | grep -i -E 'usb|uvc' | tail -20)")
    return notes


def stream_together(cfg: dict, seconds: float = 4.0) -> bool:
    """Run both rear cameras at once in the configured mode and report the frame rate of each."""
    counts = {n: 0 for n in REAR}
    cams = []
    for name in REAR:
        c = dict(cfg["cameras"][name])
        c["max_process_fps"] = 15              # count arrivals; decode little
        th = UsbCamera(name, c, lambda n, *_a: counts.__setitem__(n, counts[n] + 1), log=lambda m: print("  " + m))
        th.start()
        cams.append(th)
    time.sleep(seconds + 1.5)
    rates = {th.cam_name: th.fps_in for th in cams}
    problems = {th.cam_name: th.problem for th in cams}
    for th in cams:
        th.close()
    ok = True
    for name in REAR:
        want = float(cfg["cameras"][name]["capture"].get("fps", 0))
        good = rates[name] > 20
        ok = ok and good
        print("  %-9s %5.0f frames/s%s" % (name, rates[name], "" if good else "   <-- NOT WORKING: " + (problems[name] or "no pictures")))
        if good and want and rates[name] < 0.6 * want:
            print("            (the config asks for %.0f; this camera delivers what it can in this mode)" % want)
    if not ok:
        nodes = {c["path"]: c for c in usb_camera_nodes()}
        ctrl = {nodes[cfg["cameras"][n]["device"]]["controller"] for n in REAR if cfg["cameras"][n].get("device") in nodes}
        print("\n  If each camera works on its own (the list above) but not both together, try in this order:")
        if len(ctrl) == 1:
            print("  - Both cameras are on the same USB controller (%s) and share its bandwidth. Move ONE camera to another "
                  "socket and run this tool again, until the list shows one camera on xhci-hcd.0 and the other on "
                  "xhci-hcd.1. Then --assign again." % next(iter(ctrl)))
        print("  - USB power: see the notes below.")
        print("  - A lower mode for both cameras: in a new file config/calib/mode.yaml, set capture fps to 60 for "
              "elp_left and elp_right (same layout as config/dodge.yaml).")
    return ok


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--assign", action="store_true", help="work out left/right and write config/calib/devices.yaml")
    ap.add_argument("--both", action="store_true", help="only run the two assigned cameras at the same time")
    args = ap.parse_args()
    cfg = load_config()
    if args.both:
        missing = [n for n in REAR if not cfg["cameras"][n].get("device")]
        if missing:
            raise SystemExit("%s not assigned yet: run  python3 tools/find_cameras.py --assign" % ", ".join(missing))
        print("Both rear cameras at once, %.0f s:" % 4)
        ok = stream_together(cfg)
        for note in ([] if ok else usb_health()):
            print("  " + note)
        raise SystemExit(0 if ok else 1)

    cams = usb_camera_nodes()
    if not cams:
        print("No USB camera is present (nothing under /dev/v4l/by-path).\n"
              "Check both camera plugs. If they are in, the cameras are not getting power or not being recognised:")
        for note in usb_health():
            print("  " + note)
        raise SystemExit(1)
    snap_dir = os.path.join(ROOT, "data", "snapshots")
    os.makedirs(snap_dir, exist_ok=True)
    working = []
    busy = 0                                         # cameras the dodge node is reading: not a fault
    print("%d USB camera%s present:" % (len(cams), "" if len(cams) == 1 else "s"))
    for i, cam in enumerate(cams):
        dev = cam["path"]
        print("\n[%d] %s" % (i, dev))
        print("  %s -> %s, USB controller %s socket %s, %s Mbit/s%s" % (
            cam["product"] or "camera", cam["real"], cam["controller"] or "?", cam["port"] or "?", cam["speed"] or "?",
            ", asks for up to %d mA" % cam["max_power_ma"] if cam["max_power_ma"] else ""))
        holders = who_holds(cam["real"])
        frame = None if holders else snapshot(dev)
        if any(h[1] == DODGE_NODE for h in holders):
            busy += 1
            print("  in use by the dodge node (so it works; not tested here)")
            continue
        if frame is None:
            print("  NOT WORKING: %s" % open_problem(dev))
            continue
        working.append(cam)
        print(modes(dev))
        out = os.path.join(snap_dir, "usb%d.jpg" % i)
        cv2.imwrite(out, frame)
        print("  works. snapshot %s  (%dx%d, with automatic exposure)" % (out, frame.shape[1], frame.shape[0]))

    print("\nIn the config:")
    paths = {c["path"] for c in cams}
    reals = {c["real"]: c["path"] for c in cams}
    stale = False
    for name in REAR:
        dev = cfg["cameras"][name].get("device", "")
        here = bool(dev) and (dev in paths or os.path.realpath(dev) in reals)
        stale = stale or not here
        print("  %-9s = %s%s" % (name, dev or "(not set)", "" if here else "   <-- no camera there now"))
    health = usb_health()

    if busy and len(working) + busy >= 2:
        print("\n" + FREE_THE_CAMERAS)
        print("Nothing is wrong with the cameras. Run this tool again after that if you want them tested.")
        raise SystemExit(1)
    if len(working) < 2:
        print("\nOnly %d of the 2 rear cameras works, so the dodge node would be blind on one side." % len(working))
        if len(cams) < 2:
            print("The Pi does not see the other camera at all. For that camera:\n"
                  "  1. Push its plug in firmly at both ends, or try another USB socket on the Pi.\n"
                  "  2. Run  lsusb . If it lists only one camera, the other is not powered or its cable is bad:\n"
                  "     swap the two cameras' sockets. If the fault moves with the camera, it is the camera or its\n"
                  "     cable; if it stays with the socket, it is power or that socket.\n"
                  "  3. Then run this tool again (and --assign, since sockets changed).")
        else:
            print("The Pi sees it but it will not deliver pictures; the line marked NOT WORKING above says why.")
        for note in health:
            print("  " + note)
        raise SystemExit(1)
    if not args.assign:
        if stale:
            print("\nRun again with --assign to set left/right.")
        else:
            print("\nBoth assigned cameras are present. Running them together, as the dodge node will:")
            if not stream_together(cfg):
                for note in health:
                    print("  " + note)
                raise SystemExit(1)
            print("\nBoth cameras deliver pictures. If one has been moved or swapped since they were assigned, run again with --assign.")
            for note in power_notes():
                print("Note: " + note)
        return

    if len(working) != 2:
        raise SystemExit("\nNeed exactly two working USB cameras to assign left and right; %d work. Unplug the extra "
                         "one and run this again." % len(working))
    input("\nCover the LEFT rear camera's lens completely with your hand (left = the robot's left), then press Enter...")
    time.sleep(0.5)
    levels = []
    for cam in working:
        frame = snapshot(cam["path"])
        if frame is None:
            raise SystemExit("%s stopped delivering pictures: %s" % (cam["path"], open_problem(cam["path"])))
        levels.append(float(frame.mean()))
        print("  %s brightness %.0f" % (os.path.basename(cam["path"]), levels[-1]))
    dark, bright = (0, 1) if levels[0] < levels[1] else (1, 0)
    if levels[bright] - levels[dark] < 15 or levels[dark] > 0.6 * levels[bright]:
        raise SystemExit("Could not tell which camera was covered (brightness too similar). Cover the lens fully, "
                         "in a lit room, and try again. Nothing was written.")
    left, right = working[dark]["path"], working[bright]["path"]
    path = save_calib("devices", {"cameras": {"elp_left": {"device": left}, "elp_right": {"device": right}}})
    print("\nleft  = %s\nright = %s\nwritten to %s" % (left, right, path))
    print("\nRunning both together, as the dodge node will:")
    if not stream_together(load_config()):
        for note in health:
            print("  " + note)
        raise SystemExit(1)
    for note in power_notes():
        print("Note: " + note)
    print("\nNext: python3 tools/tune_exposure.py all")


if __name__ == "__main__":
    run_tool(main)
