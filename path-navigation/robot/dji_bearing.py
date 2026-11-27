"""Runs the partner's four-microphone script, unchanged, and reads the bearings it prints.

The sound-direction module is Max's `sound-localization/localize_angle.py` (DJI Mic 3
receiver in Quadraphonic mode, four transmitters at the corners of a 120 x 234 mm
rectangle, GCC-PHAT delays, least-squares plane-wave direction). It is a command-line
program that prints one line per sound event. Nothing in that folder is edited or
imported here: this file starts the script as a child process with the settings in
config/demo.yaml (section `dji`) and parses its output, so the code that runs on the
robot is exactly the code he wrote and tested.

One line of his output (localize_angle.py, the print near the end of main):

    RMS=0.0312 | delays vs CH1: CH2=  -3.21, CH3= +31.90, CH4= +28.70 samples | bearing=+11.2° | fit magnitude=0.98

    bearing        degrees, 0 = Pupper's nose, POSITIVE TO THE RIGHT, +-180 behind
                   (navigation's convention is positive to the LEFT: the conversion is
                   topics.sound_positive = right, done once in robot/sound_adapter.py)
    "uncertain near 0° (angle would be ...)" and "uncertain (near-zero delays)"
                   all four channels heard the sound at almost the same instant. With four
                   microphones in a rectangle that is not "straight ahead": it is no
                   direction at all (sound from overhead, a channel copied to the others,
                   or electrical noise). Such lines give no bearing.
    fit magnitude  length of the fitted direction vector. 1 for an ideal plane wave from a
                   source level with the microphones; cos(elevation) for one above them;
                   his script warns above 1.25 (the delays do not fit the geometry) and
                   below 0.15.

Each bearing gets a confidence from two checks made here (his numbers are not changed):

    check          confidence   when
    ok             1.0          the delays add up and the fit is within dji.fit_low..fit_high
    fit            0.3          fit outside 0.5..1.25 (on this course the survivor is never more
                                than about 35 degrees above the microphones, so fit >= 0.8)
    inconsistent   0.0          CH4 - (CH2 + CH3) is more than dji.max_residual_samples: one
                                microphone pair locked onto a wrong peak (SOUND_INTEGRATION.md)
    uncertain      no bearing   his script said "uncertain"

sound.min_confidence (0.5) then decides which are used; all of them are logged.

No ROS here: robot/sound_node.py publishes what this returns, and the tests run it on a laptop.
"""
from __future__ import annotations

import math
import os
import re
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent          # path-navigation/

_FLOAT = r"([+-]?(?:\d+\.?\d*|\.\d+)(?:[eE][+-]?\d+)?|[+-]?inf|nan)"
_LINE = re.compile(
    r"RMS=\s*" + _FLOAT + r"\s*\|\s*delays vs CH1:\s*CH2=\s*" + _FLOAT + r"\s*,\s*CH3=\s*" + _FLOAT +
    r"\s*,\s*CH4=\s*" + _FLOAT + r"\s*samples\s*\|\s*bearing=(?P<angle>.*?)\s*\|\s*fit magnitude=\s*" + _FLOAT)
_PLAIN_ANGLE = re.compile(r"^\s*" + _FLOAT + r"\s*°?\s*$")
_SELECTED = re.compile(r"Selected \[(\d+)\] (.*) with (\d+) input channel")


def command(dji, python: str | None = None) -> list[str]:
    """The command line that starts his script with the demo's settings (config `dji`)."""
    script = Path(dji.script)
    if not script.is_absolute():
        script = ROOT / script
    cmd = [python or dji.python or sys.executable, "-u", str(script),
           "--rate", str(int(dji.rate)),
           "--width-mm", repr(float(dji.width_mm)),
           "--height-mm", repr(float(dji.height_mm)),
           "--threshold", repr(float(dji.threshold)),
           "--frame-seconds", repr(float(dji.frame_seconds)),
           "--deadband-samples", repr(float(dji.deadband_samples))]
    if str(dji.device or "").strip():
        cmd += ["--device", str(dji.device).strip()]
    return cmd


def parse_line(line: str, dji=None) -> dict | None:
    """One line of his output -> a reading, or None if the line is not an estimate.

    A reading is {"bearing": deg (positive right, his convention) or None if uncertain,
                  "uncertain": bool, "rms", "fit", "delays": [CH2, CH3, CH4] in samples,
                  "confidence": 0..1}
    """
    m = _LINE.search(line)
    if not m:
        return None
    rms, d2, d3, d4 = (float(m.group(i)) for i in range(1, 5))
    fit = float(m.group(6))
    angle_text = m.group("angle")
    plain = _PLAIN_ANGLE.match(angle_text)
    bearing = float(plain.group(1)) if plain else None
    if bearing is not None and not math.isfinite(bearing):
        bearing = None
    fit_low = getattr(dji, "fit_low", 0.5)
    fit_high = getattr(dji, "fit_high", 1.25)
    caution = getattr(dji, "caution_confidence", 0.3)
    max_residual = getattr(dji, "max_residual_samples", 3.0)
    # The four microphones are a rectangle, so p4 - p1 = (p2 - p1) + (p3 - p1), and for any direction the
    # delays must add up: CH4 = CH2 + CH3. One pair that locked onto a wrong correlation peak breaks this.
    residual = d4 - (d2 + d3)
    if bearing is None:
        confidence, check = 0.0, "uncertain"
    elif abs(residual) > max_residual:
        confidence, check = 0.0, "inconsistent"
    elif not fit_low <= fit <= fit_high:
        confidence, check = caution, "fit"
    else:
        confidence, check = 1.0, "ok"
    return {"bearing": bearing, "uncertain": bearing is None, "rms": rms, "fit": fit,
            "delays": [d2, d3, d4], "residual": round(residual, 2), "check": check,
            "confidence": confidence, "text": angle_text.strip()}


def parse_selected(line: str):
    """'Selected [3] Wireless Mic Rx ... with 4 input channel(s).' -> (index, name, channels) or None."""
    m = _SELECTED.search(line)
    return (int(m.group(1)), m.group(2).strip(), int(m.group(3))) if m else None


def expected_delays(bearing_right_deg: float, width_mm: float = 120.0, height_mm: float = 234.0,
                    rate: int = 48000, c: float = 343.0) -> list[float]:
    """What his script should print for a far source level with the mics (CH2, CH3, CH4 minus CH1, samples).

    For the on-robot check: a clap straight ahead should print about [0, +32.7, +32.7];
    from the right, [-16.8, 0, -16.8]. Same geometry and sign as localize_angle.py.
    """
    w, h = width_mm / 1000.0, height_mm / 1000.0
    pos = [(-w / 2, -h / 2), (w / 2, -h / 2), (-w / 2, h / 2), (w / 2, h / 2)]   # x right, y back
    a = math.radians(bearing_right_deg)
    u = (math.sin(a), -math.cos(a))                                               # towards the source
    t = [-(px * u[0] + py * u[1]) / c for px, py in pos]
    return [(t[i] - t[0]) * rate for i in (1, 2, 3)]


class Localizer:
    """His script as a child process: started, read line by line, restarted if it stops.

    on_reading(reading, t) is called for every estimate line; everything else he prints is
    kept in `log` (the last 40 lines) and the last of it in status()["last_message"].
    """

    def __init__(self, dji, on_reading=None, python: str | None = None, env: dict | None = None,
                 clock=time.monotonic):
        self.dji = dji
        self.cmd = command(dji, python)
        self.on_reading = on_reading
        self.env = dict(os.environ if env is None else env)
        self.env.setdefault("PYTHONIOENCODING", "utf-8")    # his output has a degree sign
        self.env["PYTHONUNBUFFERED"] = "1"
        self.clock = clock
        self.lock = threading.Lock()
        self.proc = None
        self.reader = None
        self.stopping = False
        self.log: deque = deque(maxlen=40)
        self.device = None                 # (index, name, channels) once his script says which
        self.started_at = None
        self.restarts = -1
        self.exit_code = None
        self.counts = {"bearings": 0, "uncertain": 0, "fit": 0, "inconsistent": 0}
        self.last = None
        self.t_last = None

    # ------------------------------------------------------------------ process
    def start(self) -> None:
        with self.lock:
            if self.proc is not None and self.proc.poll() is None:
                return
            self.restarts += 1
            self.device, self.exit_code = None, None
            self.started_at = self.clock()
            self.proc = subprocess.Popen(
                self.cmd, cwd=str(Path(self.cmd[2]).parent), env=self.env, stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding="utf-8",
                errors="replace", bufsize=1)
            self.reader = threading.Thread(target=self._read, args=(self.proc,), daemon=True)
            self.reader.start()

    def _read(self, proc) -> None:
        for raw in proc.stdout:
            self.feed(raw.rstrip("\n"))
        code = proc.wait()
        with self.lock:
            if proc is self.proc:
                self.exit_code = code

    def feed(self, line: str) -> None:
        """One line of his output (the reader thread calls this; tests may too)."""
        now = self.clock()
        reading = parse_line(line, self.dji)
        if reading is None:
            if line.strip():
                with self.lock:
                    self.log.append(line)
                    selected = parse_selected(line)
                    if selected:
                        self.device = selected
            return
        with self.lock:
            self.last, self.t_last = reading, now
            if reading["uncertain"]:
                self.counts["uncertain"] += 1
            else:
                self.counts["bearings"] += 1
                if reading["check"] in ("fit", "inconsistent"):
                    self.counts[reading["check"]] += 1
        if self.on_reading is not None:
            self.on_reading(reading, now)

    def running(self) -> bool:
        with self.lock:
            return self.proc is not None and self.proc.poll() is None

    def listening(self) -> bool:
        """Running, and it has opened a device with four channels (his script exits if it cannot)."""
        with self.lock:
            alive = self.proc is not None and self.proc.poll() is None
            return alive and self.device is not None and self.device[2] >= 4

    def poll(self) -> bool:
        """Restart it if it has stopped and restart_s has passed. True if it was restarted."""
        if self.stopping or self.running():
            return False
        with self.lock:
            if self.reader is not None and self.reader.is_alive():
                return False                # still draining its last lines
            since = 1e9 if self.started_at is None else self.clock() - self.started_at
        if since < float(self.dji.restart_s):
            return False
        self.start()
        return True

    def stop(self, timeout: float = 2.0) -> None:
        """Ctrl+C, as a person would stop it (he catches KeyboardInterrupt and closes the stream)."""
        self.stopping = True
        with self.lock:
            proc = self.proc
        if proc is None or proc.poll() is not None:
            return
        try:
            proc.send_signal(2)             # SIGINT
            proc.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            proc.kill()
            proc.wait(timeout=timeout)

    # ------------------------------------------------------------------ report
    def status(self) -> dict:
        with self.lock:
            alive = self.proc is not None and self.proc.poll() is None
            listening = alive and self.device is not None and self.device[2] >= 4
            last = self.last
            return {
                "alive": bool(listening),
                "module": "sound-localization/localize_angle.py",
                "running": bool(alive),
                "device": self.device[1] if self.device else None,
                "channels": self.device[2] if self.device else None,
                "pid": self.proc.pid if self.proc is not None else None,
                "restarts": max(0, self.restarts),
                "exit_code": self.exit_code,
                "bearings": self.counts["bearings"],
                "uncertain": self.counts["uncertain"],
                "fit_caution": self.counts["fit"],
                "inconsistent": self.counts["inconsistent"],
                "last_bearing_right_deg": None if last is None else last["bearing"],
                "last_fit": None if last is None else last["fit"],
                "last_rms": None if last is None else last["rms"],
                "last_check": None if last is None else last["check"],
                "last_age_s": None if self.t_last is None else round(self.clock() - self.t_last, 2),
                "last_message": self.log[-1] if self.log else None,
                "threshold": float(self.dji.threshold),
            }
