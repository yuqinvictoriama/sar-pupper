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
from .threat import ThreatAssessor
from .track import Tracker


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
        self.refractory = float(fc.get("refractory_s", 1.0))
        self.recover_min = float(fc.get("recover_min_s", 0.3))
        self.recover_max = float(fc.get("recover_max_s", 1.5))
        self.level_tilt = float(fc.get("level_tilt_deg", 20.0))
        self.still_gyro = float(fc.get("still_gyro_rad_s", 0.6))
        self.abort_tilt = float(fc.get("abort_tilt_deg", 40.0))

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

    def _evaluate(self, t_now: float) -> bool:
        tracks = self.tracker.confirmed()
        if self.state in (State.DODGE, State.RECOVER):
            return False
        if not tracks:
            self.state = State.SEARCH
            self.threat_counts.clear()
            self.last_threats = []
            return False
        R = self.orientation(None)
        tv, vx, vy = self.robot_vel
        own = R @ np.array([vx, vy, 0.0]) if (t_now - tv) < 0.6 else None
        threats = [self.assessor.assess(tr, t_now, self.tracker, R, own) for tr in tracks]
        self.last_threats = threats
        live = {th.track_id for th in threats if th.is_threat}
        self.threat_counts = {k: v for k, v in self.threat_counts.items() if k in live}
        for k in live:
            self.threat_counts[k] = self.threat_counts.get(k, 0) + 1
        danger = [th for th in threats if th.is_threat]
        if not danger:
            self.state = State.TRACK
            return False
        th = min(danger, key=lambda d: d.tau)
        self.state = State.THREAT
        if t_now < self.t_ready or th.tau < self.min_tau:
            return False
        plan = self.planner.plan(th, self.free_space)
        late = bool(th.tau <= plan.t_need)                # no time left to wait for a better estimate
        if th.tau > plan.t_need + self.trigger_margin:
            return False                                  # plenty of time: keep watching
        if not late:
            if self.threat_counts[th.track_id] < self.confirm_cycles:
                return False                              # want the verdict repeated first
            if not th.confident:
                return False                              # sideways prediction still vague
        if not self.can_dodge:
            self._event(t_now, "cannot_dodge", reason=self.cannot_reason, threat=th.summary())
            self.t_ready = t_now + self.refractory        # do not repeat the report every frame
            return False
        self.plan, self.threat = plan, th
        self.t_dodge = t_now
        self.dry_run = not self.armed
        self.state = State.DODGE
        self.stats["dodges"] += 1
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
                elif t_now - self.t_dodge >= self.plan.duration:
                    self._start_recover(t_now)
                else:
                    return None if self.dry_run else (self.plan.vx, self.plan.vy, 0.0)
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
    def predicted_pixel(self, cam: str, t: float, size=None):
        """(u, v, half_size_px) where the most urgent tracked ball should appear in this camera, or None."""
        with self.lock:
            tracks = [tr for tr in self.tracker.confirmed() if tr.closing_speed > 0.5]
            if not tracks:
                return None
            tr = max(tracks, key=lambda k: k.closing_speed)
            x, _P = self.tracker.predict_state(tr.x, tr.P, t - tr.t)
            R = self.orientation(t)
        camera = self.camera_for(cam, size)
        p_b = R.T @ x[:3]
        uv, ok, _ = camera.project_base(p_b[None, :])
        if not ok[0]:
            return None
        det = self.cfg["detector"]
        sr = camera.model.solid_angle_per_px(float(uv[0, 0]), float(uv[0, 1]))
        if not (sr > 0):
            return None
        px_per_rad = 1.0 / math.sqrt(sr)
        half = max(float(det.get("roi_min_px", 96)), math.radians(float(det.get("roi_half_deg", 14.0))) * px_per_rad)
        return float(uv[0, 0]), float(uv[0, 1]), half

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
