#!/usr/bin/env python3
"""Set a rear camera's exposure, gain and white balance for the room it is in.

    python3 tools/tune_exposure.py all            # both rear cameras, one after the other
    python3 tools/tune_exposure.py elp_left
    python3 tools/tune_exposure.py elp_left --max-exposure-ms 12     # darker room: allow more motion smear

A thrown ball needs a short exposure or it smears, so the cameras do not run on
automatic exposure. The price is a dark picture indoors unless the gain is
raised to match. This tool looks at the picture and finds the settings: it
keeps the exposure short (3 ms unless told otherwise) and raises the gain until
the picture is bright enough; only if the gain runs out does it lengthen the
exposure. It then locks the white balance the camera chose, so the ball's
colour does not drift, and writes everything to config/calib/exposure.yaml.

Run it in the room and lighting you will throw in, with the camera looking at
the room (not at a lamp or a window), and again if the lighting changes much.
Then redo tools/calibrate_color.py for that camera.
Stop the dodge node, view.py and record.py first.
"""
from __future__ import annotations

import argparse
import math
import os
import shutil
import subprocess
import time

from _common import ROOT, cv2, load_config, np, run_tool

from usb_camera import (EXPOSURE_AUTO, EXPOSURE_TIME, WB_AUTO, control_name, open_capture, open_problem, read_bgr,
                        read_controls, set_control)

from dodge.config import save_calib


def brightness(frame) -> float:
    return float(cv2.cvtColor(frame[::4, ::4], cv2.COLOR_BGR2GRAY).mean())


class Tuner:
    def __init__(self, name: str, cam_cfg: dict):
        self.name = name
        self.cfg = cam_cfg
        self.device = cam_cfg.get("device", "")
        self.cap, self.raw = open_capture(self.device, cam_cfg.get("capture", {}), log=lambda m: print("  " + m))
        if self.cap is None:
            raise SystemExit("%s: cannot open the camera: %s" % (name, open_problem(self.device)))
        self.controls = read_controls(self.device)
        self.c_auto = control_name(self.controls, *EXPOSURE_AUTO)
        self.c_exp = control_name(self.controls, *EXPOSURE_TIME)
        self.c_gain = "gain" if "gain" in self.controls else None
        self.c_wb_auto = control_name(self.controls, *WB_AUTO)
        self.c_wb = "white_balance_temperature" if "white_balance_temperature" in self.controls else None
        if not self.c_exp:
            raise SystemExit("%s: this camera does not offer an exposure-time control; nothing to tune." % name)
        self.fps = float(self.cap.get(cv2.CAP_PROP_FPS)) or 60.0
        self.last = None

    def close(self) -> None:
        self.cap.release()

    def set(self, control, value) -> None:
        if control:
            err = set_control(self.device, control, value)
            if err:
                print("  could not set %s=%s (%s)" % (control, value, err))

    def frames(self, n: int, settle_s: float = 0.0) -> list:
        """n fresh frames, after letting a control change take effect."""
        end = time.time() + settle_s
        skipped = 0
        while time.time() < end or skipped < 6:
            if read_bgr(self.cap, self.raw) is None:
                time.sleep(0.01)
            skipped += 1
            if skipped > 400:
                break
        out = []
        tries = 0
        while len(out) < n and tries < n * 5:
            f = read_bgr(self.cap, self.raw)
            tries += 1
            if f is not None:
                out.append(f)
        if not out:
            raise SystemExit("%s: the camera stopped delivering pictures: %s" % (self.name, open_problem(self.device)))
        self.last = out[-1]
        return out

    def level(self, settle_s: float = 0.25) -> float:
        return float(np.median([brightness(f) for f in self.frames(5, settle_s)]))

    # -- the search --------------------------------------------------------------
    def lowest_reaching(self, control, lo: int, hi: int, target: float) -> tuple:
        """Smallest setting of a control (brightness rises with it) that reaches target. Returns (value, brightness)."""
        self.set(control, hi)
        top = self.level()
        if top < target:
            return hi, top
        best = (hi, top)
        tol = max(1, (hi - lo) // 40)
        while hi - lo > tol:
            mid = (lo + hi) // 2
            self.set(control, mid)
            b = self.level()
            if b >= target:
                hi, best = mid, (mid, b)
            else:
                lo = mid
        self.set(control, best[0])
        return best

    def lock_white_balance(self, keep: bool) -> int | None:
        fallback = self.cfg.get("capture", {}).get("wb_temperature")
        if keep or not (self.c_wb and self.c_wb_auto):
            return fallback
        info = self.controls[self.c_wb]
        self.set(self.c_auto, 3)                    # let the camera see a normally exposed picture while it decides
        self.set(self.c_wb_auto, 1)
        self.frames(3, 2.0)
        try:
            out = subprocess.run(["v4l2-ctl", "-d", self.device, "-C", self.c_wb], capture_output=True, text=True, timeout=5).stdout
            value = int(out.strip().split(":")[-1])
        except (ValueError, IndexError, subprocess.TimeoutExpired, OSError):
            value = None
        if value is None or not (info.get("min", 0) <= value <= info.get("max", 10 ** 6)):
            value = fallback if fallback is not None else info.get("default")
        self.set(self.c_wb_auto, 0)
        if value is not None:
            self.set(self.c_wb, value)
        return value

    def tune(self, target: float, prefer_ms: float, max_ms: float, keep_wb: bool) -> dict:
        e_info = self.controls[self.c_exp]
        e_min, e_max = e_info.get("min", 1), e_info.get("max", 5000)
        frame_limit = int(0.9 * 10000.0 / self.fps)                 # exposure must fit inside one frame
        e_cap = max(e_min, min(e_max, frame_limit, int(round(max_ms * 10))))
        e_pref = max(e_min, min(e_cap, int(round(prefer_ms * 10))))
        g_info = self.controls.get(self.c_gain, {}) if self.c_gain else {}
        g_min, g_max = g_info.get("min", 0), g_info.get("max", 0)

        wb = self.lock_white_balance(keep_wb)
        self.set(self.c_auto, 1)                                    # manual from here on
        if "exposure_dynamic_framerate" in self.controls:
            self.set("exposure_dynamic_framerate", 0)
        exposure, gain = e_pref, g_min
        self.set(self.c_exp, exposure)
        self.set(self.c_gain, gain)
        level = self.level(0.4)
        print("  at %.1f ms and the lowest gain the picture averages %.0f of 255 (aiming for about %.0f)" % (
            exposure / 10.0, level, target))
        if level > 1.2 * target:                                    # bright room: shorten the exposure instead
            exposure, level = self.lowest_reaching(self.c_exp, e_min, e_pref, target)
        elif level < target:
            if self.c_gain and g_max > g_min:
                gain, level = self.lowest_reaching(self.c_gain, g_min, g_max, target)
            if level < 0.85 * target and e_cap > e_pref:            # gain alone is not enough: lengthen the exposure
                exposure, level = self.lowest_reaching(self.c_exp, e_pref, e_cap, target)
        final = self.frames(40)
        per_frame = np.array([brightness(f) for f in final])
        level = float(np.median(per_frame))
        flicker = float((np.percentile(per_frame, 95) - np.percentile(per_frame, 5)) / max(level, 1.0))
        clipped = float((cv2.cvtColor(final[-1], cv2.COLOR_BGR2GRAY) >= 250).mean())
        return {"exposure_100us": int(exposure), "gain": int(gain) if self.c_gain else None, "wb_temperature": wb,
                "level": level, "flicker": flicker, "clipped": clipped, "gain_max": g_max, "exposure_cap": e_cap,
                "frame": final[-1]}


def smear_note(cam_cfg: dict, exposure_100us: int, ball_d: float) -> str:
    intr = cam_cfg.get("intrinsics", {})
    width = float(intr.get("width", 1280))
    f = float(intr["fx"]) if "fx" in intr else (width / 2.0) / math.tan(math.radians(float(intr.get("hfov_deg", 120.0)) / 2.0))
    t = exposure_100us / 10000.0
    return ("a ball crossing the view at 8 m/s, 2 m away, smears %.0f px during one exposure (it is %.0f px wide there)"
            % (8.0 * t * f / 2.0, ball_d * f / 2.0))


def tune_camera(cfg: dict, name: str, args) -> bool:
    cam_cfg = cfg["cameras"][name]
    print("\n%s" % name)
    tuner = Tuner(name, cam_cfg)
    try:
        r = tuner.tune(args.target, args.exposure_ms, args.max_exposure_ms, args.keep_wb)
    finally:
        tuner.close()
    capture = {"auto_exposure": False, "exposure_100us": r["exposure_100us"]}
    if r["gain"] is not None:
        capture["gain"] = r["gain"]
    if r["wb_temperature"] is not None:
        capture["wb_temperature"] = int(r["wb_temperature"])
    path = save_calib("exposure", {"cameras": {name: {"capture": capture}}})
    snap_dir = os.path.join(ROOT, "data", "snapshots")
    os.makedirs(snap_dir, exist_ok=True)
    snap = os.path.join(snap_dir, name + "_tuned.jpg")
    cv2.imwrite(snap, r["frame"])
    print("  exposure %.1f ms, gain %s%s, white balance %s -> picture averages %.0f of 255" % (
        r["exposure_100us"] / 10.0, r["gain"] if r["gain"] is not None else "(no control)",
        " (the camera's maximum)" if r["gain"] is not None and r["gain"] >= r["gain_max"] else "",
        ("%d K" % r["wb_temperature"]) if r["wb_temperature"] is not None else "left alone", r["level"]))
    print("  " + smear_note(cam_cfg, r["exposure_100us"], cfg["ball"]["diameter_m"]))
    print("  written to %s; snapshot %s" % (path, snap))
    ok = True
    if r["exposure_100us"] > round(args.exposure_ms * 10) and r["level"] >= 0.6 * args.target:
        print("  note: the gain alone was not enough, so the exposure is longer than the %.0f ms asked for. Fast balls\n"
              "  will smear more and read as further away than they are. More light in the room lets it come back down\n"
              "  (run this again after)." % args.exposure_ms)
    if r["level"] < 0.6 * args.target:
        ok = False
        print("  STILL TOO DARK. The room does not give this camera enough light at a fast shutter. Either add light\n"
              "  (the reliable fix: a bright room, or daylight), or accept more smear:\n"
              "      python3 tools/tune_exposure.py %s --max-exposure-ms %.0f" % (name, min(30.0, max(args.max_exposure_ms * 1.5, 10.0))))
    if r["clipped"] > 0.05:
        print("  note: %.0f%% of the picture is pure white (a lamp or window in view?). Fine as long as the ball is not "
              "thrown in front of it." % (100 * r["clipped"]))
    if r["flicker"] > 0.12:
        print("  note: the brightness changes by about %.0f%% from frame to frame: the room lights flicker with the mains.\n"
              "  The ball's colour will flicker with it. Daylight or a different lamp is better; calibrate_color.py will\n"
              "  widen the colour range to cope if you leave it." % (100 * r["flicker"]))
    return ok


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("camera", help="elp_left, elp_right or all")
    ap.add_argument("--target", type=float, default=90.0, help="average picture brightness to aim for, 0-255 (default 90)")
    ap.add_argument("--exposure-ms", type=float, default=3.0, help="exposure to keep if the light allows (default 3)")
    ap.add_argument("--max-exposure-ms", type=float, default=8.0, help="longest exposure it may fall back to (default 8)")
    ap.add_argument("--keep-wb", action="store_true", help="leave the white balance at the configured value")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()
    if not shutil.which("v4l2-ctl"):
        raise SystemExit("This tool needs v4l2-ctl:  sudo apt install -y v4l-utils")
    cfg = load_config(args.config)
    usb = [n for n, c in cfg["cameras"].items() if c.get("source") == "v4l2" and c.get("enabled", True)]
    names = usb if args.camera == "all" else [args.camera]
    for name in names:
        if name not in cfg["cameras"]:
            raise SystemExit("No camera called %s in the config (have: %s)." % (name, ", ".join(cfg["cameras"])))
        if cfg["cameras"][name].get("source") != "v4l2":
            raise SystemExit("%s is not a USB camera. The fisheye's exposure is automatic; to fix it, start the stack "
                             "with  bash robot/start.sh --fisheye-exposure 4000  (microseconds)." % name)
    results = [tune_camera(cfg, name, args) for name in names]
    print("\nNext: python3 tools/view.py <camera> to look, then tools/calibrate_color.py for each camera.")
    if not all(results):
        raise SystemExit(1)


if __name__ == "__main__":
    run_tool(main)
