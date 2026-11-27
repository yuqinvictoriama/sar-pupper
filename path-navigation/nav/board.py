"""The latest word from every other component, in one place.

The robot node's ROS callbacks write here; so does the simulator. Twenty
times a second `inputs()` hands the mission what is new, and `snapshot()`
builds the one dictionary Mission Control draws, the same on the robot and
in simulation.
"""
from __future__ import annotations

import math
import threading
from dataclasses import asdict

from . import events as EV
from .config import DemoConfig
from .mission import Inputs, Mission


class Board:
    def __init__(self, cfg: DemoConfig | None = None):
        self.cfg = cfg or DemoConfig()
        self.lock = threading.RLock()
        self.stale = self.cfg.safety.stale_s
        self.t0 = None
        # latest values and when they arrived
        self.yaw, self.tilt_deg, self.t_imu = 0.0, 0.0, -1e9
        self.cmd, self.t_cmd = None, -1e9
        self.mux_source, self.t_mux = "", -1e9
        self.sound_status, self.t_sound, self.t_bearing = {}, -1e9, -1e9
        self.keyword_status, self.t_keyword, self.t_heard, self.last_phrase = {}, -1e9, -1e9, ""
        self.terrain, self.t_terrain = {}, -1e9
        self.obstacles, self.t_obstacles = [], -1e9
        self.dodge, self.t_dodge = {}, -1e9
        self.walking, self.t_walking = None, -1e9
        self.voice_listeners, self.t_voice = None, -1e9
        self.last_dodge_event = None
        self.last_crossing = {}
        # queues, emptied by inputs()
        self._bearings: list = []
        self._keyword = ""
        self._crossing: list = []
        self._dodge_events: list = []
        self._estop = False
        self._commands: list = []
        # the log Mission Control shows
        self.log: list = []
        self.said = {"event": "", "line": "", "t": -1e9}
        self.wanted = {"arm_dodge": False, "terrain_mode": "off"}

    # ------------------------------------------------------------------ writers
    def imu(self, t: float, yaw: float, tilt_deg: float = 0.0) -> None:
        with self.lock:
            self.yaw, self.tilt_deg, self.t_imu = float(yaw), float(tilt_deg), t

    def cmd_vel(self, t: float, vx: float, vy: float, wz: float) -> None:
        with self.lock:
            self.cmd, self.t_cmd = (float(vx), float(vy), float(wz)), t

    def mux(self, t: float, source: str) -> None:
        with self.lock:
            self.mux_source, self.t_mux = str(source), t

    def bearing(self, t: float, rad: float, confidence: float = 1.0) -> None:
        with self.lock:
            self._bearings.append((float(rad), float(confidence)))
            self.t_bearing = self.t_sound = t

    def sound_alive(self, t: float, status: dict | None = None) -> None:
        with self.lock:
            self.sound_status, self.t_sound = dict(status or {}), t

    def keyword_heard(self, t: float, phrase: str) -> None:
        with self.lock:
            self._keyword = phrase or "HEARD"
            self.last_phrase, self.t_heard = self._keyword, t

    def keyword_alive(self, t: float, status: dict | None = None) -> None:
        with self.lock:
            self.keyword_status, self.t_keyword = dict(status or {}), t

    def terrain_state(self, t: float, status: dict) -> None:
        with self.lock:
            self.terrain, self.t_terrain = dict(status), t

    def terrain_obstacles(self, t: float, obstacles: list) -> None:
        with self.lock:
            self.obstacles, self.t_obstacles = list(obstacles), t

    def crossing(self, t: float, status: dict) -> None:
        with self.lock:
            self._crossing.append(dict(status))
            self.last_crossing = dict(status, t=t)

    def dodge_state(self, t: float, status: dict) -> None:
        with self.lock:
            self.dodge, self.t_dodge = dict(status), t

    def dodge_event(self, t: float, event: dict) -> None:
        with self.lock:
            self._dodge_events.append(dict(event))
            if event.get("event") in ("dodge", "throw_seen"):
                self.last_dodge_event = dict(event, t_seen=t)

    def estop(self, t: float) -> None:
        with self.lock:
            self._estop = True

    def walking_active(self, t: float, active: bool) -> None:
        with self.lock:
            self.walking, self.t_walking = bool(active), t

    def voice(self, t: float, listeners) -> None:
        with self.lock:
            self.voice_listeners, self.t_voice = listeners, t

    def command(self, text: str) -> None:
        with self.lock:
            self._commands.append(str(text))

    # ------------------------------------------------------------------ for the mission
    def alive(self, t: float) -> dict:
        s = self.stale
        return {
            "imu": t - self.t_imu < s,
            "walking": bool(self.walking) and t - self.t_walking < 3.0,
            "sound": t - self.t_sound < max(s, 2.5),
            "keyword": t - self.t_keyword < 2.5 and bool(self.keyword_status.get("alive", True)),
            "terrain": t - self.t_terrain < 2.5,
            "dodge": t - self.t_dodge < s,
            "voice": bool(self.voice_listeners) and t - self.t_voice < 5.0,
        }

    def inputs(self, t: float) -> Inputs:
        with self.lock:
            if self.t0 is None:
                self.t0 = t
            alive = self.alive(t)
            terrain = dict(self.terrain)
            terrain["alive"] = alive["terrain"]
            terrain["scan_fresh"] = t - self.t_obstacles < 1.5
            dodge = dict(self.dodge)
            dodge["alive"] = alive["dodge"]
            inp = Inputs(
                t=t, yaw=self.yaw, imu_ok=alive["imu"], tilt_deg=self.tilt_deg,
                cmd_vel=self.cmd if t - self.t_cmd < 0.5 else None,
                mux_source=self.mux_source if t - self.t_mux < 1.0 else "",
                bearings=self._bearings, keyword=self._keyword, terrain=terrain, crossing=self._crossing,
                obstacles=self.obstacles if terrain["scan_fresh"] else [], dodge=dodge,
                dodge_events=self._dodge_events, estop=self._estop, alive=alive, commands=self._commands)
            self._bearings, self._keyword, self._crossing, self._dodge_events = [], "", [], []
            self._estop, self._commands = False, []
            return inp

    def record(self, t: float, out, mission: Mission) -> None:
        """Keep what the mission just reported, for the log and the display."""
        with self.lock:
            self.wanted = {"arm_dodge": out.arm_dodge, "terrain_mode": out.terrain_mode}
            for name, details in out.events:
                line = EV.LINES.get(name, "")
                self.log.append({"t": round(t - (self.t0 or t), 1), "run_s": mission.status()["run_s"], "event": name,
                                 "details": details, "line": line, "state": mission.state})
                if line:
                    self.said = {"event": name, "line": line, "t": t}
            del self.log[:-200]

    # ------------------------------------------------------------------ for Mission Control
    def snapshot(self, t: float, mission: Mission, mode: str = "robot", extra: dict | None = None) -> dict:
        with self.lock:
            alive = self.alive(t)
            nav = mission.status()
            d = self.dodge
            snap = {
                "mode": mode, "t": round(t - (self.t0 or t), 2), "nav": nav, "alive": alive,
                "course": asdict(self.cfg.course), "config_source": self.cfg.source,
                "limits": {"moe_deg": self.cfg.sound.moe_deg, "min_bearings": self.cfg.sound.min_bearings,
                           "keyword_from": self.cfg.arrive.keyword_from, "cover_from": self.cfg.arrive.cover_from,
                           "hard_stop": self.cfg.arrive.hard_stop, "cruise": self.cfg.walk.cruise,
                           "gauntlet": self.cfg.walk.gauntlet, "require": list(self.cfg.safety.require)},
                "sound": {"alive": alive["sound"], "status": self.sound_status,
                          "last_bearing_age": round(t - self.t_bearing, 1) if self.t_bearing > -1e8 else None},
                "keyword": {"alive": alive["keyword"], "status": self.keyword_status, "last_phrase": self.last_phrase,
                            "heard_age": round(t - self.t_heard, 1) if self.t_heard > -1e8 else None},
                "terrain": {"alive": alive["terrain"], "state": self.terrain.get("state", ""),
                            "mode": self.terrain.get("mode", ""), "reason": self.terrain.get("reason", ""),
                            "box": self.terrain.get("box"), "scan_ms": self.terrain.get("scan_ms"),
                            "obstacles": self.obstacles if t - self.t_obstacles < 1.5 else [],
                            "crossing": {k: v for k, v in self.last_crossing.items() if k in
                                         ("state", "why", "progress", "stage", "duration", "advance", "plan")}},
                "dodge": {"alive": alive["dodge"], "state": d.get("state", ""), "armed": bool(d.get("armed", False)),
                          "can_dodge": d.get("can_dodge"), "cannot_reason": d.get("cannot_reason", ""),
                          "tracks": d.get("tracks", []), "cameras": d.get("cameras", {}),
                          "stats": d.get("stats", {}), "last_event": self.last_dodge_event},
                "mux": self.mux_source if t - self.t_mux < 1.0 else "",
                "cmd": None if self.cmd is None or t - self.t_cmd > 0.5 else [round(v, 3) for v in self.cmd],
                "tilt_deg": round(self.tilt_deg, 1), "yaw_deg": round(math.degrees(self.yaw), 1),
                "wanted": dict(self.wanted),
                "said": dict(self.said, age=round(t - self.said["t"], 1) if self.said["line"] else None),
                "events": self.log[-60:],
                "phrase": EV.PHRASE,
            }
            snap["said"].pop("t", None)
            if extra:
                snap.update(extra)
            return snap
