"""The dodge reflex: detections in, velocity command out.

    core = DodgeCore(cfg)
    core.set_imu(t, quat_wxyz, gyro)            # as IMU samples arrive
    core.process_blobs("elp_left", t_frame, blobs, t_now)   # per camera frame
    cmd = core.command(t_now)                   # (vx, vy, wz) or None, at ~100 Hz

State machine:  SEARCH -> TRACK -> THREAT -> DODGE -> RECOVER -> SEARCH

No ROS in here, so the same code runs on the robot (robot/dodge_node.py), on
recordings (tools/replay.py) and in the simulator (dodge/sim.py).
"""
from __future__ import annotations

import math
import threading
from collections import deque
from enum import Enum

import numpy as np

from .camera_models import build_cameras
from .escape import EscapePlanner, FreeSpace
from .measure import measure_blobs
from .threat import Threat, ThreatAssessor
from .track import Tracker, accel


class State(Enum):
    SEARCH = "SEARCH"      # nothing tracked
    TRACK = "TRACK"        # a ball is tracked, no danger (yet)
    THREAT = "THREAT"      # a hit is predicted but the robot cannot or need not move yet
    DODGE = "DODGE"        # moving out of the way
    RECOVER = "RECOVER"    # standing still until level and steady


def quat_to_rot(w: float, x: float, y: float, z: float) -> np.ndarray:
    """Rotation matrix of a unit quaternion (body -> world)."""
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def predict_pixel(camera, hint, t: float, det_cfg: dict):
    """(u, v, half_size_px): where a tracked ball should appear in one camera's picture taken at time t.

    hint is DodgeCore.track_hint(). The ball is flown along its arc from the time of the hint to t.
    None when there is no hint or the ball is outside this camera's view.
    """
    if hint is None:
        return None
    x, t_track, R, drag_k = hint
    dt = t - t_track
    v = x[3:]
    p_w = x[:3] + v * dt + 0.5 * accel(v, drag_k) * dt * dt
    p_b = R.T @ p_w
    uv, ok, _ = camera.project_base(p_b[None, :])
    if not ok[0]:
        return None
    sr = camera.model.solid_angle_per_px(float(uv[0, 0]), float(uv[0, 1]))
    if not (sr > 0):
        return None
    px_per_rad = 1.0 / math.sqrt(sr)
    half = max(float(det_cfg.get("roi_min_px", 96)), math.radians(float(det_cfg.get("roi_half_deg", 14.0))) * px_per_rad)
    return float(uv[0, 0]), float(uv[0, 1]), half


class DodgeCore:
    def __init__(self, cfg: dict, armed: bool = False):
        self.cfg = cfg
        self.ball = cfg["ball"]
        self.cameras = build_cameras(cfg)
        self.tracker = Tracker(cfg["tracker"], self.ball)
        self.assessor = ThreatAssessor(cfg["threat"], cfg["robot"], self.ball)
        self.planner = EscapePlanner(cfg["escape"], cfg["robot"], self.ball, self.assessor.margin)
        tc, fc = cfg["threat"], cfg["fsm"]
        self.confirm_cycles = int(tc.get("confirm_cycles", 2))
        self.trigger_margin = float(tc.get("trigger_margin_s", 0.3))
        self.min_tau = float(tc.get("min_tau_s", 0.05))
        self.idle_closing = 0.5 * float(tc.get("min_closing_speed", 1.0))
        self.refractory = float(fc.get("refractory_s", 1.0))
        self.recover_min = float(fc.get("recover_min_s", 0.3))
        self.recover_max = float(fc.get("recover_max_s", 1.5))
        self.level_tilt = float(fc.get("level_tilt_deg", 20.0))
        self.still_gyro = float(fc.get("still_gyro_rad_s", 0.6))
        self.abort_tilt = float(fc.get("abort_tilt_deg", 40.0))
        self.ramp_out = max(1e-3, float(cfg["escape"].get("ramp_out_s", 0.2)))

        self.lock = threading.RLock()
        self.armed = bool(armed)
        self.can_dodge = True
        self.cannot_reason = ""
        self.state = State.SEARCH
        self.free_space: FreeSpace | None = None
        self.imu = deque(maxlen=600)            # (t, R_wb, tilt_deg, gyro_norm)
        self.robot_vel = (0.0, 0.0, 0.0)        # (t, vx, vy): the velocity the robot is being asked to walk at
        self.threat_counts: dict = {}           # track id -> consecutive threat verdicts
        self.plan = None
        self.threat = None
        self.t_dodge = 0.0
        self.t_recover = 0.0
        self.t_ready = 0.0                      # no new dodge before this time
        self.dry_run = False
        self.events: deque = deque(maxlen=200)
        self.last_threats: list = []
        self.stats = {"frames": 0, "detections": 0, "dodges": 0}
        self._scaled: dict = {}
        self._flight = None                     # a ball seen in flight that has not (yet) led to a dodge
        self.hold = ""                          # why a predicted hit has not led to a dodge yet
        self.report_undodged = bool(fc.get("report_undodged", True))
        self.undodged_travel = float(fc.get("undodged_min_travel_m", 0.5))

    # ------------------------------------------------------------------ inputs
    def set_armed(self, armed: bool) -> None:
        with self.lock:
            self.armed = bool(armed)

    def set_can_dodge(self, ok: bool, reason: str = "") -> None:
        with self.lock:
            self.can_dodge, self.cannot_reason = bool(ok), reason

    def set_robot_velocity(self, t: float, vx: float, vy: float) -> None:
        """The walking command currently in force (from /cmd_vel), so the robot's own motion is not mistaken for the ball's."""
        with self.lock:
            self.robot_vel = (t, float(vx), float(vy))

    def set_free_space(self, clearances) -> None:
        with self.lock:
            self.free_space = FreeSpace(clearances) if clearances is not None and len(clearances) else None

    def set_imu(self, t: float, quat_wxyz, gyro=(0.0, 0.0, 0.0)) -> None:
        R = quat_to_rot(*quat_wxyz)
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2])))))
        g = math.sqrt(sum(float(v) ** 2 for v in gyro))
        with self.lock:
            self.imu.append((t, R, tilt, g))

    def emergency_stop(self, t: float) -> None:
        """The joystick e-stop was pressed: drop everything."""
        with self.lock:
            if self.state in (State.DODGE, State.RECOVER):
                self._event(t, "abort", reason="emergency stop")
            self.state, self.plan = State.SEARCH, None
            self.t_ready = t + self.refractory

    def orientation(self, t: float | None = None) -> np.ndarray:
        """Body-to-world rotation at time t (latest if None, identity with no IMU)."""
        with self.lock:
            if not self.imu:
                return np.eye(3)
            if t is None:
                return self.imu[-1][1]
            best = self.imu[-1]
            for item in reversed(self.imu):     # newest first; stop once we pass t
                if abs(item[0] - t) <= abs(best[0] - t):
                    best = item
                elif item[0] < t:
                    break
            return best[1]

    def camera_for(self, cam: str, size=None):
        """The camera model, rescaled if frames arrive at another resolution than it was calibrated at."""
        camera = self.cameras[cam]
        if size is None or (int(size[0]), int(size[1])) == (camera.model.width, camera.model.height):
            return camera
        key = (cam, int(size[0]), int(size[1]))
        if key not in self._scaled:
            self._scaled[key] = camera.with_resolution(key[1], key[2])
        return self._scaled[key]

    # ------------------------------------------------------------------ per camera frame
    def process_blobs(self, cam: str, t_frame: float, blobs, t_now: float | None = None, size=None):
        """Feed one frame's blobs. Returns True if a dodge started on this frame."""
        camera = self.camera_for(cam, size)
        meas = measure_blobs(camera, blobs, self.ball["diameter_m"])
        return self.process_measurements(cam, t_frame, meas, t_now)

    def process_measurements(self, cam: str, t_frame: float, meas, t_now: float | None = None) -> bool:
        t_now = t_frame if t_now is None else t_now
        with self.lock:
            self.stats["frames"] += 1
            self.stats["detections"] += len(meas)
            self.tracker.update(t_frame, meas, self.orientation(t_frame))
            return self._evaluate(t_now)

    def _watch_flight(self, t_now: float, tracks, threats=()) -> None:
        """Notice throws that never led to a dodge, so they can be looked at afterwards: a "throw_seen"
        event is reported once such a ball is gone, with what the assessor made of it."""
        flying = [tr for tr in tracks if tr.flying]
        if flying:
            if self._flight is None:
                self._flight = {"t0": t_now, "t_last": t_now, "dodged": False, "closest_m": float("inf"),
                                "verdicts": {}, "last": None, "hits": 0, "travel": 0.0}
            f = self._flight
            f["t_last"] = t_now
            f["hits"] = max(f["hits"], max(tr.hits for tr in flying))
            f["closest_m"] = min(f["closest_m"], min(tr.range_m for tr in flying))
            f["travel"] = max(f["travel"], max(tr.travel_m for tr in flying))
            ids = {tr.id for tr in flying}
            for th in threats:
                if th.track_id in ids:
                    f["verdicts"][th.reason] = f["verdicts"].get(th.reason, 0) + 1
                    f["last"] = th.summary()
        elif self._flight is not None:
            f, self._flight = self._flight, None
            # Only something that really crossed the room counts as a throw worth reporting.
            if (self.report_undodged and not f["dodged"] and f["hits"] >= 5 and f["travel"] >= self.undodged_travel
                    and t_now >= self.t_ready):
                self._event(t_now, "throw_seen", seen_s=round(f["t_last"] - f["t0"], 3), travel_m=round(f["travel"], 2),
                            closest_m=round(f["closest_m"], 2), verdicts=f["verdicts"], threat=f["last"])

    def _evaluate(self, t_now: float) -> bool:
        tracks = self.tracker.confirmed()
        if self.state in (State.DODGE, State.RECOVER):
            if self._flight is not None:
                self._flight["dodged"] = True
            return False
        if not tracks:
            self._watch_flight(t_now, tracks)
            self.state = State.SEARCH
            self.threat_counts.clear()
            self.last_threats = []
            return False
        R = self.orientation(None)
        tv, vx, vy = self.robot_vel
        own = R @ np.array([vx, vy, 0.0]) if (t_now - tv) < 0.6 else None
        threats = []
        for tr in tracks:
            if not self.assessor.use_bearing and not tr.flying and tr.closing_speed < self.idle_closing:
                # A still thing (a ball lying on the floor, a green cushion): the assessor would say the
                # same, after flying its arc forward. A cluttered room has many of these in every picture.
                threats.append(Threat(tr.id, reason="not approaching"))
            else:
                threats.append(self.assessor.assess(tr, t_now, self.tracker, R, own))
        self.last_threats = threats
        self._watch_flight(t_now, tracks, threats)
        live = {th.track_id for th in threats if th.is_threat}
        self.threat_counts = {k: v for k, v in self.threat_counts.items() if k in live}
        for k in live:
            self.threat_counts[k] = self.threat_counts.get(k, 0) + 1
        danger = [th for th in threats if th.is_threat]
        self.hold = ""
        if not danger:
            self.state = State.TRACK
            return False
        th = min(danger, key=lambda d: d.tau)
        self.state = State.THREAT
        if t_now < self.t_ready or th.tau < self.min_tau:
            self.hold = "just dodged" if t_now < self.t_ready else "too late to matter"
            return False
        plan = self.planner.plan(th, self.free_space)
        late = bool(th.tau <= plan.t_need)                # no time left to wait for a better estimate
        if th.tau > plan.t_need + self.trigger_margin:
            self.hold = "hit predicted, time to spare"
            return False                                  # plenty of time: keep watching
        if not late:
            if self.threat_counts[th.track_id] < self.confirm_cycles:
                self.hold = "hit predicted, waiting for a second picture to agree"
                return False                              # want the verdict repeated first
            if not th.confident:
                self.hold = "hit predicted, but where it passes is still uncertain"
                return False                              # sideways prediction still vague
        if not self.can_dodge:
            self.hold = "cannot dodge: " + self.cannot_reason
            self._event(t_now, "cannot_dodge", reason=self.cannot_reason, threat=th.summary())
            self.t_ready = t_now + self.refractory        # do not repeat the report every frame
            return False
        self.plan, self.threat = plan, th
        self.t_dodge = t_now
        self.dry_run = not self.armed
        self.state = State.DODGE
        self.stats["dodges"] += 1
        if self._flight is not None:
            self._flight["dodged"] = True
        self._event(t_now, "dodge", dry_run=self.dry_run, late=late, threat=th.summary(), plan=plan.summary())
        return True

    # ------------------------------------------------------------------ command out
    def command(self, t_now: float):
        """Velocity command (vx, vy, wz) to publish now, or None to stay silent."""
        with self.lock:
            if self.state == State.DODGE:
                tilt = self.imu[-1][2] if self.imu else 0.0
                if tilt > self.abort_tilt:
                    self._event(t_now, "abort", reason="tilt %.0f deg" % tilt)
                    self._start_recover(t_now)
                elif t_now - self.t_dodge >= self.plan.duration + self.ramp_out:
                    self._start_recover(t_now)
                else:
                    # After the burst the command is eased to zero instead of dropped: on the Pupper the
                    # body's tilt peaked just AFTER a burst ended (26 deg, and one fall, 2026-11-26).
                    over = t_now - self.t_dodge - self.plan.duration
                    k = 1.0 if over <= 0.0 else max(0.0, 1.0 - over / self.ramp_out)
                    return None if self.dry_run else (self.plan.vx * k, self.plan.vy * k, 0.0)
            if self.state == State.RECOVER:
                held = t_now - self.t_recover
                if held >= self.recover_max or (held >= self.recover_min and self._steady(t_now)):
                    self.state = State.SEARCH
                    self.t_ready = t_now + self.refractory
                    self.threat_counts.clear()
                    self._event(t_now, "recovered", dry_run=self.dry_run, held_s=round(held, 2))
                    self.plan = None
                    return None
                return None if self.dry_run else (0.0, 0.0, 0.0)
            return None

    def _start_recover(self, t: float) -> None:
        self.state = State.RECOVER
        self.t_recover = t

    def _steady(self, t_now: float) -> bool:
        if not self.imu:
            return True
        recent = [s for s in self.imu if s[0] >= self.imu[-1][0] - 0.25]
        return all(s[2] <= self.level_tilt and s[3] <= self.still_gyro for s in recent)

    def _event(self, t: float, kind: str, **data) -> None:
        self.events.append(dict(t=round(t, 4), event=kind, **data))

    def pop_events(self) -> list:
        with self.lock:
            out = list(self.events)
            self.events.clear()
            return out

    # ------------------------------------------------------------------ helpers for the node
    def track_hint(self):
        """Where the most urgent tracked ball is, in a form any camera can turn into "look here" for its next
        picture: (state, time of the state, body-to-world rotation, drag constant), or None when nothing
        that is closing in is tracked. See predict_pixel."""
        with self.lock:
            tracks = [tr for tr in self.tracker.confirmed() if tr.closing_speed > 0.5]
            if not tracks:
                return None
            tr = max(tracks, key=lambda k: k.closing_speed)
            return tr.x.copy(), float(tr.t), self.orientation(None).copy(), float(self.tracker.drag_k)

    def predicted_pixel(self, cam: str, t: float, size=None):
        """(u, v, half_size_px) where the most urgent tracked ball should appear in this camera, or None."""
        return predict_pixel(self.camera_for(cam, size), self.track_hint(), t, self.cfg["detector"])

    def trace_state(self):
        """What the core holds right now, for the throw recorder: None while nothing is tracked."""
        with self.lock:
            if not self.tracker.tracks and self.state == State.SEARCH:
                return None
            tracks = [{"id": tr.id, "t0": round(tr.t0, 4), "hits": tr.hits, "confirmed": tr.confirmed,
                       "flying": tr.flying, "p": [round(float(v), 3) for v in tr.x[:3]],
                       "v": [round(float(v), 2) for v in tr.x[3:]], "t": round(tr.t, 4), "seen": round(tr.t_last, 4),
                       "cam": tr.last_cam} for tr in self.tracker.tracks]
            return {"state": self.state.value, "tracks": tracks, "hold": self.hold,
                    "threats": [th.summary() for th in self.last_threats]}

    def snapshot(self, t_now: float) -> dict:
        """Small status dictionary for /dodge/state and logs."""
        with self.lock:
            tracks = []
            for tr in self.tracker.tracks:
                az, el = tr.azimuth_elevation_deg()
                tracks.append({"id": tr.id, "confirmed": tr.confirmed, "hits": tr.hits,
                               "range_m": round(tr.range_m, 2), "azimuth_deg": round(az, 1),
                               "elevation_deg": round(el, 1), "closing_speed": round(tr.closing_speed, 2),
                               "cams": sorted(tr.cams)})
            return {"t": round(t_now, 3), "state": self.state.value, "armed": self.armed,
                    "can_dodge": self.can_dodge, "cannot_reason": self.cannot_reason,
                    "tracks": tracks, "threats": [th.summary() for th in self.last_threats],
                    "plan": self.plan.summary() if self.plan is not None else None,
                    "stats": dict(self.stats)}
