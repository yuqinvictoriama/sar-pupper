"""USB (V4L2) camera capture for the two ELP cameras.

One thread per camera. It asks the camera for MJPG at the configured mode,
fixes exposure and white balance (so the ball's colour does not drift), and
hands every frame it is asked to process to a callback:

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


def apply_controls(device: str, cap_cfg: dict, cap=None, log=print) -> None:
    """Fix exposure, gain and white balance. Uses v4l2-ctl when present, OpenCV properties otherwise."""
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
        self.brightness = -1.0     # mean grey level of the latest frame (0-255), -1 before the first
        self.connected = False
        self.problem = ""          # why it is not delivering, when it is not
        self._warned_dark = False

    def stop(self) -> None:
        self.running = False

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
            try:
                self._loop(cap)
            except Exception as exc:  # a camera glitch must not take the node down
                self.problem = "capture error: %r" % (exc,)
                self.log("%s: %s" % (self.cam_name, self.problem))
            finally:
                cap.release()
                self.connected = False
            if self.running:
                self._sleep(1.0)

    def _sleep(self, seconds: float) -> None:
        end = time.monotonic() + seconds
        while self.running and time.monotonic() < end:
            time.sleep(0.05)

    def _loop(self, cap) -> None:
        mono_to_wall = time.time() - time.monotonic()
        last_out = 0.0
        fails = 0
        n_in = n_out = 0
        window = time.monotonic()
        frame = None
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
                if frame is not None and frame.ndim == 3:
                    self.brightness = float(frame[::16, ::16].mean())
                    if self.brightness < DARK_MEAN and not self._warned_dark:
                        self._warned_dark = True
                        self.log("%s: the picture is very dark (average %.0f of 255): the ball will not be found. "
                                 "Run  python3 tools/tune_exposure.py %s" % (self.cam_name, self.brightness, self.cam_name))
