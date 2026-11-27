"""USB (V4L2) camera capture for the two ELP cameras.

It asks the camera for MJPG at the configured mode, fixes exposure and white
balance (so the ball's colour does not drift), and hands the newest frame to a
callback each time the callback is free (see UsbCamera for how it stays fresh):

    callback(name, t_capture, jpeg_bytes_or_None, bgr_frame)

t_capture is wall-clock time (time.time() scale) of when the camera captured
the frame, taken from the kernel's buffer timestamp when that is available.

Also here, because the tools need the same answers as the node: which USB
cameras are plugged in (usb_camera_nodes), why one will not open
(open_problem), and reading and setting camera controls through v4l2-ctl.
"""
from __future__ import annotations

import glob
import os
import re
import shutil
import subprocess
import threading
import time

import cv2
import numpy as np

DARK_MEAN = 25.0        # mean grey level (0-255) below which a picture is too dark to find the ball in


# ------------------------------------------------------------------ which cameras are there

def usb_camera_nodes() -> list:
    """USB cameras present now, one entry per camera.

    Each is a dict: path (/dev/v4l/by-path/... name, stable for a USB socket),
    real (/dev/videoN), controller and port (which USB socket), product, speed
    (Mbit/s) and max_power_ma as the camera reports them.
    The Pi lists every camera under two by-path names (...-usb-... and
    ...-usbv2-...); they are the same camera and are returned once.
    """
    by_real = {}
    for path in sorted(glob.glob("/dev/v4l/by-path/*")):
        base = os.path.basename(path)
        if "usb" not in base or not base.endswith("video-index0"):
            continue
        real = os.path.realpath(path)
        by_real.setdefault(real, path)          # sorted: the plain "-usb-" name comes first
    out = []
    for real, path in sorted(by_real.items(), key=lambda kv: kv[1]):
        info = {"path": path, "real": real, "controller": "", "port": "", "product": "", "speed": 0, "max_power_ma": 0}
        m = re.search(r"platform-([\w.\-]+?)-usb(?:v\d)?-\d+:([\d.]+):", os.path.basename(path))
        if m:
            info["controller"], info["port"] = m.group(1), m.group(2)
        info.update(_sysfs_usb_info(real))
        out.append(info)
    return out


def _sysfs_usb_info(real: str) -> dict:
    """Product name, speed and declared current of the USB device behind /dev/videoN."""
    out = {}
    node = os.path.realpath(os.path.join("/sys/class/video4linux", os.path.basename(real), "device"))
    for _ in range(4):                           # walk up from the interface to the USB device
        if os.path.exists(os.path.join(node, "idVendor")):
            break
        node = os.path.dirname(node)
    else:
        return out

    def read(name):
        try:
            with open(os.path.join(node, name)) as f:
                return f.read().strip()
        except OSError:
            return ""
    out["product"] = (read("manufacturer") + " " + read("product")).strip()
    try:
        out["speed"] = int(float(read("speed") or 0))
    except ValueError:
        pass
    m = re.match(r"(\d+)", read("bMaxPower"))
    if m:
        out["max_power_ma"] = int(m.group(1))
    return out


def who_holds(real: str) -> list:
    """Other programs that have this device open: [(pid, name)]. Only sees this user's programs."""
    out = []
    me = os.getpid()
    for fd_dir in glob.glob("/proc/[0-9]*/fd"):
        pid = int(fd_dir.split("/")[2])
        if pid == me:
            continue
        try:
            links = [os.readlink(os.path.join(fd_dir, f)) for f in os.listdir(fd_dir)]
        except OSError:
            continue
        if real in links:
            try:
                with open("/proc/%d/cmdline" % pid, "rb") as f:
                    cmd = f.read().replace(b"\0", b" ").decode(errors="replace").strip()
            except OSError:
                cmd = "?"
            out.append((pid, cmd[:80]))
    return out


def open_problem(device: str) -> str:
    """Why a camera will not open, in one sentence, with what to do."""
    if not device:
        return "no device is set for it: run  python3 tools/find_cameras.py --assign"
    if not os.path.exists(device):
        present = [c["path"] for c in usb_camera_nodes()]
        seen = ("USB cameras present now: " + ", ".join(os.path.basename(p) for p in present)) if present \
            else "No USB camera is present at all right now"
        return ("there is no camera on that USB socket (%s). It is unplugged, has lost power, or was moved to "
                "another socket since find_cameras.py --assign ran. %s. Run  python3 tools/find_cameras.py"
                % (os.path.basename(device), seen))
    real = os.path.realpath(device)
    try:
        os.close(os.open(real, os.O_RDWR | os.O_NONBLOCK))
    except PermissionError:
        return "no permission to open %s: add this user to the 'video' group (sudo usermod -aG video $USER) and log in again" % real
    except OSError as exc:
        return "%s is there but will not open (%s): unplug the camera and plug it back in" % (real, exc.strerror)
    holders = who_holds(real)
    if holders:
        return "%s is in use by another program: %s. Stop it (the dodge node, view.py, record.py and a browser all count)" % (
            real, "; ".join("pid %d %s" % h for h in holders))
    return ("%s is there but refuses to deliver video. Usual causes: another program has it (check: sudo fuser -v %s), "
            "too little USB power, or a marginal cable or socket (unplug it and plug it back in)" % (real, real))


# ------------------------------------------------------------------ camera controls

def read_controls(device: str) -> dict:
    """Controls the camera offers: name -> {min, max, step, default, value, inactive}. Empty without v4l2-ctl."""
    if not shutil.which("v4l2-ctl"):
        return {}
    try:
        out = subprocess.run(["v4l2-ctl", "-d", device, "--list-ctrls"], capture_output=True, text=True, timeout=5).stdout
    except (subprocess.TimeoutExpired, OSError):
        return {}
    controls = {}
    for line in out.splitlines():
        parts = line.strip().split()
        if len(parts) < 2 or not parts[1].startswith("0x"):
            continue
        info = {"inactive": "inactive" in line}
        for key, value in re.findall(r"(min|max|step|default|value)=(-?\d+)", line):
            info[key] = int(value)
        controls[parts[0]] = info
    return controls


def list_controls(device: str) -> dict:
    """Kept for callers that only need the names."""
    return read_controls(device)


def set_control(device: str, name: str, value) -> str:
    """Set one control. Returns "" on success, the camera's complaint otherwise."""
    try:
        r = subprocess.run(["v4l2-ctl", "-d", device, "-c", "%s=%d" % (name, int(value))],
                           capture_output=True, text=True, timeout=5)
    except (subprocess.TimeoutExpired, OSError) as exc:
        return str(exc)
    return "" if r.returncode == 0 else (r.stderr.strip() or "v4l2-ctl failed")


def control_name(controls: dict, *options):
    return next((n for n in options if n in controls), None)


EXPOSURE_AUTO = ("auto_exposure", "exposure_auto")
EXPOSURE_TIME = ("exposure_time_absolute", "exposure_absolute")
WB_AUTO = ("white_balance_automatic", "white_balance_temperature_auto")


def apply_controls(device: str, cap_cfg: dict, cap=None, log=print, nudge_wb: bool = False) -> None:
    """Fix exposure, gain and white balance. Uses v4l2-ctl when present, OpenCV properties otherwise.

    nudge_wb: set the white balance to a neighbouring value first, then to the wanted one. The ELP
    cameras report a fixed white balance as set but go on showing another (seen on the Pupper,
    2026-11-26: "fixed at 3858 K" with red three times blue) until the value is changed while they
    stream; only a change makes them act on it.
    """
    auto = bool(cap_cfg.get("auto_exposure", False))
    exposure = cap_cfg.get("exposure_100us")
    gain = cap_cfg.get("gain")
    wb = cap_cfg.get("wb_temperature")
    names = read_controls(device)
    if names:
        sets = []
        n = control_name(names, *EXPOSURE_AUTO)
        if n:
            sets.append((n, 3 if auto else 1))                  # UVC: 1 = manual, 3 = aperture priority
        if "exposure_dynamic_framerate" in names:
            sets.append(("exposure_dynamic_framerate", 0))      # never trade frame rate for brightness
        n = control_name(names, *EXPOSURE_TIME)
        if n and not auto and exposure is not None:
            sets.append((n, exposure))
        if gain is not None and "gain" in names:
            sets.append(("gain", gain))
        n = control_name(names, *WB_AUTO)
        if n and wb is not None:
            sets.append((n, 1 if wb == "auto" else 0))
        if wb is not None and wb != "auto" and "white_balance_temperature" in names:
            if nudge_wb:
                info = names["white_balance_temperature"]
                other = int(wb) + 150 if int(wb) + 150 <= info.get("max", 6500) else int(wb) - 150
                sets.append(("white_balance_temperature", other))
            sets.append(("white_balance_temperature", wb))
        for name, value in (cap_cfg.get("controls") or {}).items():   # anything else, by its v4l2-ctl name
            if name in names:
                sets.append((name, value))
            else:
                log("%s: this camera has no control called %s" % (device, name))
        for name, value in sets:       # one at a time: a rejected control must not block the others
            err = set_control(device, name, value)
            if err:
                log("%s: could not set %s=%s (%s)" % (device, name, value, err))
        return
    if cap is None:
        return
    log("%s: v4l2-ctl not found, setting camera controls through OpenCV (less reliable); "
        "sudo apt install v4l-utils" % device)
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 3 if auto else 1)
    if not auto and exposure is not None:
        cap.set(cv2.CAP_PROP_EXPOSURE, float(exposure))
    if gain is not None:
        cap.set(cv2.CAP_PROP_GAIN, float(gain))
    if wb == "auto":
        cap.set(cv2.CAP_PROP_AUTO_WB, 1)
    elif wb is not None:
        cap.set(cv2.CAP_PROP_AUTO_WB, 0)
        cap.set(cv2.CAP_PROP_WB_TEMPERATURE, float(wb))


def describe_controls(device: str, cap_cfg: dict | None = None) -> str:
    """What the camera says its exposure, gain and white balance are; flags any that differ from what was asked."""
    c = read_controls(device)
    if not c:
        return ""
    cap_cfg = cap_cfg or {}
    parts, wrong = [], []
    n_auto, n_exp = control_name(c, *EXPOSURE_AUTO), control_name(c, *EXPOSURE_TIME)
    if n_auto and c[n_auto].get("value") == 3:
        parts.append("exposure automatic")
        if not cap_cfg.get("auto_exposure", False):
            wrong.append("exposure should be fixed")
    elif n_exp:
        parts.append("exposure %.1f ms" % (c[n_exp].get("value", 0) / 10.0))
        want = cap_cfg.get("exposure_100us")
        if want is not None and c[n_exp].get("value") != int(want):
            wrong.append("exposure should be %.1f ms" % (int(want) / 10.0))
    if "gain" in c:
        parts.append("gain %d" % c["gain"].get("value", 0))
        if cap_cfg.get("gain") is not None and c["gain"].get("value") != int(cap_cfg["gain"]):
            wrong.append("gain should be %d" % int(cap_cfg["gain"]))
    n_wb = control_name(c, *WB_AUTO)
    want_wb = cap_cfg.get("wb_temperature")
    if n_wb and c[n_wb].get("value"):
        parts.append("white balance automatic")
        if want_wb not in (None, "auto"):
            wrong.append("white balance should be fixed at %d K" % int(want_wb))
    elif "white_balance_temperature" in c:
        parts.append("white balance fixed at %d K" % c["white_balance_temperature"].get("value", 0))
        if want_wb == "auto":
            wrong.append("white balance should be automatic")
        elif want_wb is not None and c["white_balance_temperature"].get("value") != int(want_wb):
            wrong.append("white balance should be %d K" % int(want_wb))
    text = ", ".join(parts)
    if wrong:
        text += "   <-- THE CAMERA DID NOT TAKE THE SETTINGS: " + "; ".join(wrong)
    return text


# ------------------------------------------------------------------ capture

def open_capture(device: str, cap_cfg: dict, log=print):
    """Open the camera in MJPG at the requested mode. Returns (capture, raw_mode)."""
    if not device or not os.path.exists(device):
        return None, False                       # (OpenCV would only print a puzzling warning)
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        return None, False
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(cap_cfg.get("width", 1280)))
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(cap_cfg.get("height", 720)))
    cap.set(cv2.CAP_PROP_FPS, float(cap_cfg.get("fps", 120)))
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    apply_controls(device, cap_cfg, cap, log)
    # Some cameras only take (or keep) white balance and exposure settings once they
    # are streaming, and fall back to their defaults when a stream starts. So start
    # the stream, then set everything again, then check what the camera reports.
    cap.grab()
    apply_controls(device, cap_cfg, cap, log, nudge_wb=True)
    state = describe_controls(device, cap_cfg)
    if state:
        log("camera settings: %s" % state)
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


def read_bgr(cap, raw: bool):
    """One decoded frame from a capture opened by open_capture, or None."""
    ok, buf = cap.read()
    if not ok or buf is None:
        return None
    frame = cv2.imdecode(buf.reshape(-1), cv2.IMREAD_COLOR) if raw else buf
    return frame if frame is not None and frame.ndim == 3 else None


class UsbCamera(threading.Thread):
    """One USB camera. Two threads, so that the picture being looked at is always a fresh one:

    - the capture thread (this one) takes every frame off the camera the moment it arrives, so
      nothing waits in the camera's queue and goes stale;
    - the work thread asks for "the next frame", decodes it and hands it to the callback.

    While the callback is busy the frames that arrive are let go. The frame it gets next is the
    first one captured after it came back, never one that had been waiting. (Before 2026-11-26
    one thread did both: every frame that took long to process left a queue of old frames behind
    it, and the node was always looking about one processing time into the past.)
    """

    def __init__(self, name: str, cam_cfg: dict, callback, log=print):
        super().__init__(daemon=True, name="usb-" + name)
        self.cam_name = name
        self.cfg = cam_cfg
        self.device = cam_cfg.get("device", "")
        self.cap_cfg = cam_cfg.get("capture", {})
        self.max_fps = float(cam_cfg.get("max_process_fps", 0) or 0)
        self.stale_s = float(cam_cfg.get("stale_frame_s", 0.045))   # a frame older than this on arrival is let go
        self.callback = callback
        self.log = log
        self.running = True
        self.raw = False
        self.size = (0, 0)
        self.fps_in = 0.0          # frames arriving from the camera per second
        self.fps_out = 0.0         # frames handed to the callback per second
        self.decode_ms = 0.0       # time the latest frame took to decode
        self.brightness = -1.0     # mean grey level of the latest frame (0-255), -1 before the first
        self.connected = False
        self.problem = ""          # why it is not delivering, when it is not
        self._warned_dark = False
        self._want = threading.Event()     # the work thread is free and wants the next frame
        self._have = threading.Event()     # a frame is waiting in _slot
        self._slot = None                  # (t_capture wall clock, buffer)
        self._capturing = False
        self._n_out = 0
        self._frame = None                 # latest decoded frame, for the brightness check

    def stop(self) -> None:
        self.running = False

    def reapply_controls(self) -> None:
        """Send exposure, gain and white balance to the camera again while it streams (see apply_controls)."""
        if self.connected:
            apply_controls(self.device, self.cap_cfg, None, self.log, nudge_wb=True)

    def close(self, timeout: float = 3.0) -> bool:
        """Stop and wait for the thread, so the camera is released before the program exits.

        (Exiting while the thread is still inside OpenCV aborts the whole program.)
        Returns False if the thread did not finish in time.
        """
        self.running = False
        if self.is_alive() and threading.current_thread() is not self:
            self.join(timeout)
        return not self.is_alive()

    def run(self) -> None:
        while self.running:
            cap, self.raw = open_capture(self.device, self.cap_cfg, self.log)
            if cap is None:
                self.connected = False
                problem = open_problem(self.device)
                if problem != self.problem:          # say why once, not every two seconds
                    self.problem = problem
                    self.log("%s: cannot open the camera: %s" % (self.cam_name, problem))
                    self.log("%s: will keep trying every 2 s" % self.cam_name)
                self._sleep(2.0)
                continue
            self.connected = True
            self.problem = ""
            self.log("%s: opened %s %dx%d @ %.0f fps (%s)" % (
                self.cam_name, self.device, cap.get(cv2.CAP_PROP_FRAME_WIDTH), cap.get(cv2.CAP_PROP_FRAME_HEIGHT),
                cap.get(cv2.CAP_PROP_FPS), "camera JPEG passed through" if self.raw else "decoded by OpenCV"))
            self._slot = None
            self._have.clear()
            self._want.clear()
            self._capturing = True
            worker = threading.Thread(target=self._work, daemon=True, name="usb-work-" + self.cam_name)
            worker.start()
            try:
                self._loop(cap)
            except Exception as exc:  # a camera glitch must not take the node down
                self.problem = "capture error: %r" % (exc,)
                self.log("%s: %s" % (self.cam_name, self.problem))
            finally:
                self._capturing = False
                self._have.set()                 # wake the work thread so it can finish
                if threading.current_thread() is not worker:
                    worker.join(3.0)
                cap.release()
                self.connected = False
            if self.running:
                self._sleep(1.0)

    def _sleep(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while self.running and time.monotonic() < end:
            time.sleep(0.05)

    def _loop(self, cap) -> None:
        """Capture thread: take every frame off the camera at once; keep one only when the work thread wants it."""
        mono_to_wall = time.time() - time.monotonic()
        last_out = 0.0
        fails = 0
        n_in = 0
        skipped = 0
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
            if due and self._want.is_set():
                if now_mono - t_cap > self.stale_s and skipped < 8:
                    skipped += 1          # this one waited in the camera's queue (this thread was held up):
                    continue              # a newer one is right behind it
                skipped = 0
                ok, buf = cap.retrieve()
                if ok and buf is not None:
                    last_out = t_cap
                    self._slot = (t_cap + mono_to_wall, buf)
                    self._want.clear()
                    self._have.set()
            if now_mono - window >= 1.0:
                self.fps_in = n_in / (now_mono - window)
                self.fps_out = self._n_out / (now_mono - window)
                n_in = self._n_out = 0
                window = now_mono
                mono_to_wall = time.time() - time.monotonic()
                frame = self._frame
                if frame is not None and frame.ndim == 3:
                    self.brightness = float(frame[::16, ::16].mean())
                    if self.brightness < DARK_MEAN and not self._warned_dark:
                        self._warned_dark = True
                        self.log("%s: the picture is very dark (average %.0f of 255): the ball will not be found. "
                                 "Run  python3 tools/tune_exposure.py %s" % (self.cam_name, self.brightness, self.cam_name))

    def _work(self) -> None:
        """Work thread: ask for the next frame, decode it, hand it to the callback."""
        while self.running and self._capturing:
            self._want.set()
            if not self._have.wait(0.2):
                continue
            self._have.clear()
            item, self._slot = self._slot, None
            if item is None:
                continue
            t_wall, buf = item
            try:
                jpeg = None
                if self.raw:
                    t0 = time.monotonic()
                    jpeg = buf.reshape(-1)
                    frame = cv2.imdecode(jpeg, cv2.IMREAD_COLOR)
                    self.decode_ms = (time.monotonic() - t0) * 1000.0
                else:
                    frame = buf
                if frame is None or frame.ndim != 3:
                    continue
                self._frame = frame
                self._n_out += 1
                self.size = (frame.shape[1], frame.shape[0])
                self.callback(self.cam_name, t_wall, jpeg, frame)
            except Exception as exc:      # one bad frame must not end the camera
                self.problem = "frame error: %r" % (exc,)
                self.log("%s: %s" % (self.cam_name, self.problem))
                time.sleep(0.2)
