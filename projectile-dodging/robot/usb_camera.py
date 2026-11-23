"""USB (V4L2) camera capture for the two ELP cameras.

One thread per camera. It asks the camera for MJPG at the configured mode,
fixes exposure and white balance (so the ball's colour does not drift), and
hands every frame it is asked to process to a callback:

    callback(name, t_capture, jpeg_bytes_or_None, bgr_frame)

t_capture is wall-clock time (time.time() scale) of when the camera captured
the frame, taken from the kernel's buffer timestamp when that is available.
"""
from __future__ import annotations

import shutil
import subprocess
import threading
import time

import cv2
import numpy as np


def list_controls(device: str) -> dict:
    """Control names the camera offers, from v4l2-ctl (empty if v4l2-ctl is missing)."""
    if not shutil.which("v4l2-ctl"):
        return {}
    try:
        out = subprocess.run(["v4l2-ctl", "-d", device, "--list-ctrls"], capture_output=True, text=True, timeout=5).stdout
    except (subprocess.TimeoutExpired, OSError):
        return {}
    names = {}
    for line in out.splitlines():
        parts = line.strip().split()
        if len(parts) >= 2 and parts[1].startswith("0x"):
            names[parts[0]] = line.strip()
    return names


def apply_controls(device: str, cap_cfg: dict, cap=None, log=print) -> None:
    """Fix exposure, gain and white balance. Uses v4l2-ctl when present, OpenCV properties otherwise."""
    auto = bool(cap_cfg.get("auto_exposure", False))
    exposure = cap_cfg.get("exposure_100us")
    gain = cap_cfg.get("gain")
    wb = cap_cfg.get("wb_temperature")
    names = list_controls(device)
    if names:
        sets = []

        def first(*options):
            return next((n for n in options if n in names), None)

        n = first("auto_exposure", "exposure_auto")
        if n:
            sets.append("%s=%d" % (n, 3 if auto else 1))        # UVC: 1 = manual, 3 = aperture priority
        n = first("exposure_time_absolute", "exposure_absolute")
        if n and not auto and exposure is not None:
            sets.append("%s=%d" % (n, int(exposure)))
        if gain is not None and "gain" in names:
            sets.append("gain=%d" % int(gain))
        n = first("white_balance_automatic", "white_balance_temperature_auto")
        if n and wb is not None:
            sets.append("%s=0" % n)
        if wb is not None and "white_balance_temperature" in names:
            sets.append("white_balance_temperature=%d" % int(wb))
        for item in sets:      # one at a time: a rejected control must not block the others
            r = subprocess.run(["v4l2-ctl", "-d", device, "-c", item], capture_output=True, text=True)
            if r.returncode != 0:
                log("%s: could not set %s (%s)" % (device, item, r.stderr.strip()))
        return
    if cap is None:
        return
    log("%s: v4l2-ctl not found, setting camera controls through OpenCV (less reliable)" % device)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3 if auto else 1)
    if not auto and exposure is not None:
        cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure))
    if gain is not None:
        cap.set(cv2.CAP_PROP_GAIN, float(gain))
    if wb is not None:
        cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        cap.set(cv2.CAP_PROP_WB_TEMPERATURE, float(wb))


def open_capture(device: str, cap_cfg: dict, log=print):
    """Open the camera in MJPG at the requested mode. Returns (capture, raw_mode)."""
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None, False
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cap_cfg.get("width", 1280)))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cap_cfg.get("height", 720)))
    cap.set(cv2.CAP_PROP_FPS, float(cap_cfg.get("fps", 120)))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    apply_controls(device, cap_cfg, cap, log)
    # Raw mode: get the camera's JPEG bytes untouched (cheap to record, decoded by us).
    raw = False
    if cap_cfg.get("raw_mjpg", True):
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        ok, buf = cap.read()
        if ok and buf is not None and buf.dtype == np.uint8 and buf.size > 4:
            flat = buf.reshape(-1)
            raw = bool(flat[0] == 0xFF and flat[1] == 0xD8)
        if not raw:
            cap.set(cv2.CAP_PROP_CONVERT_RGB, 1)
    return cap, raw


class UsbCamera(threading.Thread):
    def __init__(self, name: str, cam_cfg: dict, callback, log=print):
        super().__init__(daemon=True, name="usb-" + name)
        self.cam_name = name
        self.cfg = cam_cfg
        self.device = cam_cfg.get("device", "")
        self.cap_cfg = cam_cfg.get("capture", {})
        self.max_fps = float(cam_cfg.get("max_process_fps", 0) or 0)
        self.callback = callback
        self.log = log
        self.running = True
        self.raw = False
        self.size = (0, 0)
        self.fps_in = 0.0          # frames arriving from the camera per second
        self.fps_out = 0.0         # frames handed to the callback per second
        self.connected = False

    def stop(self) -> None:
        self.running = False

    def run(self) -> None:
        while self.running:
            cap, self.raw = open_capture(self.device, self.cap_cfg, self.log)
            if cap is None:
                self.connected = False
                self.log("%s: cannot open %r; retrying in 2 s" % (self.cam_name, self.device))
                time.sleep(2.0)
                continue
            self.connected = True
            self.log("%s: opened %s %dx%d @ %.0f fps (%s)" % (
                self.cam_name, self.device, cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
                cap.get(cv2.CAP_PROP_FPS), "camera JPEG passed through" if self.raw else "decoded by OpenCV"))
            try:
                self._loop(cap)
            except Exception as exc:  # a camera glitch must not take the node down
                self.log("%s: capture error: %r" % (self.cam_name, exc))
            finally:
                cap.release()
                self.connected = False
            if self.running:
                time.sleep(1.0)

    def _loop(self, cap) -> None:
        mono_to_wall = time.time() - time.monotonic()
        last_out = 0.0
        fails = 0
        n_in = n_out = 0
        window = time.monotonic()
        while self.running:
            if not cap.grab():
                fails += 1
                if fails > 30:
                    raise RuntimeError("camera stopped delivering frames")
                time.sleep(0.005)
                continue
            fails = 0
            now_mono = time.monotonic()
            # Kernel buffer timestamp (CLOCK_MONOTONIC, ms); fall back to "now".
            stamp = cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.0
            t_cap = stamp if (0.0 <= now_mono - stamp < 0.5) else now_mono
            n_in += 1
            due = self.max_fps <= 0 or (t_cap - last_out) >= (1.0 / self.max_fps) * 0.9
            if due:
                ok, buf = cap.retrieve()
                if ok and buf is not None:
                    jpeg = None
                    if self.raw:
                        jpeg = buf.reshape(-1)
                        frame = cv2.imdecode(jpeg, cv2.IMREAD_COLOR)
                    else:
                        frame = buf
                    if frame is not None and frame.ndim == 3:
                        last_out = t_cap
                        n_out += 1
                        self.size = (frame.shape[1], frame.shape[0])
                        self.callback(self.cam_name, t_cap + mono_to_wall, jpeg, frame)
            if now_mono - window >= 1.0:
                self.fps_in = n_in / (now_mono - window)
                self.fps_out = n_out / (now_mono - window)
                n_in = n_out = 0
                window = now_mono
                mono_to_wall = time.time() - time.monotonic()
