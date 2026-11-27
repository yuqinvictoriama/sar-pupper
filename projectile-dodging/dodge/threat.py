"""Will the tracked ball hit the robot, when, and from where?

The track is flown forward on a ballistic arc. The robot is a box in
base_link. The answer is the closest the ball's surface gets to the box before
it lands, the time that happens (time-to-contact), and how sure we are.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .track import accel


@dataclass
class Threat:
    track_id: int
    is_threat: bool = False
    confident: bool = False
    reason: str = ""
    tau: float = float("inf")          # s until contact / closest approach
    miss_m: float = float("inf")       # clearance between ball surface and body (<= 0: predicted hit)
    sigma_miss: float = 0.0
    sigma_lat: float = 0.0
    lateral_offset: float = 0.0        # where the ball passes, measured along `left_of_travel` from the body centre
    along_offset: float = 0.0          # same, along the direction of travel
    approach_dir: np.ndarray = field(default_factory=lambda: np.array([1.0, 0.0]))  # horizontal travel direction, base_link
    bearing_deg: float = 0.0           # direction the ball comes FROM, base_link (0 = ahead, 90 = left)
    descent_deg: float = 0.0
    closing_speed: float = 0.0
    range_m: float = 0.0
    speed: float = 0.0

    def summary(self) -> dict:
        return {"track": self.track_id, "threat": self.is_threat, "confident": self.confident, "reason": self.reason,
                "tau": round(self.tau, 3) if math.isfinite(self.tau) else None,
                "miss_m": round(self.miss_m, 3) if math.isfinite(self.miss_m) else None,
                "sigma_lat": round(self.sigma_lat, 3), "bearing_deg": round(self.bearing_deg, 1),
                "descent_deg": round(self.descent_deg, 1), "closing_speed": round(self.closing_speed, 2),
                "range_m": round(self.range_m, 2)}


class ThreatAssessor:
    def __init__(self, cfg: dict, robot: dict, ball: dict):
        self.cfg = cfg
        box = robot["box"]
        self.lo = np.array([box["x_min"], box["y_min"], -float(robot["hip_height_m"])])
        self.hi = np.array([box["x_max"], box["y_max"], box["z_top"]])
        self.center_xy = np.array([(box["x_min"] + box["x_max"]) / 2.0, (box["y_min"] + box["y_max"]) / 2.0])
        self.ground_z = -float(robot["hip_height_m"])
        self.r_ball = ball["diameter_m"] / 2.0
        self.ts = np.arange(0.0, float(cfg.get("horizon_s", 1.6)), float(cfg.get("step_s", 0.01)))
        self.min_hits = int(cfg.get("min_hits", 4))
        self.min_age = float(cfg.get("min_age_s", 0.04))
        self.min_closing = float(cfg.get("min_closing_speed", 1.0))
        self.max_closing = float(cfg.get("max_closing_speed", 15.0))
        self.min_range = float(cfg.get("min_range_m", 0.25))
        self.min_ball_speed = float(cfg.get("min_ball_speed", 1.5))
        self.lobs_count = bool(cfg.get("judge_rising_balls", True))
        self.margin = float(cfg.get("margin_m", 0.05))
        self.k_sigma = float(cfg.get("k_sigma", 1.5))
        self.sigma_cap = float(cfg.get("sigma_cap_m", 0.45))
        self.sigma_lat_max = float(cfg.get("sigma_lat_max_m", 0.2))
        self.camera_support = float(cfg.get("camera_support", 0.4))

    def assess(self, track, t_now: float, tracker, R_wb: np.ndarray | None = None,
               robot_vel_w: np.ndarray | None = None) -> Threat:
        """robot_vel_w: the robot's own velocity in the world frame, if known. The track is relative to the
        robot, so a still object looks like it approaches while the robot walks toward it; this removes that."""
        th = Threat(track.id)
        if R_wb is None:
            R_wb = np.eye(3)
        if not track.confirmed or track.hits < self.min_hits:
            th.reason = "too few detections"
            return th
        if track.age < self.min_age:
            th.reason = "too new"
            return th
        x, P = tracker.predict_state(track.x, track.P, t_now - track.t)
        p, v = x[:3], x[3:]
        rng = float(np.linalg.norm(p))
        th.range_m = rng
        th.speed = float(np.linalg.norm(v))
        th.closing_speed = float(-np.dot(p, v) / rng) if rng > 1e-6 else 0.0
        own = float(np.dot(p, robot_vel_w) / rng) if (robot_vel_w is not None and rng > 1e-6) else 0.0
        v_fit = track.fitted_velocity()
        ground_speed = None
        if v_fit is not None:
            ground_speed = float(np.linalg.norm(v_fit + (robot_vel_w if robot_vel_w is not None else 0.0)))
        if th.closing_speed - max(own, 0.0) < self.min_closing:
            # A lob is not closing in yet while it climbs: early in its flight it mostly goes up. So a ball
            # that is plainly in flight is judged by where its arc comes down, not by its speed toward the robot.
            lobbed = self.lobs_count and getattr(track, "flying", False) and ground_speed is not None \
                and ground_speed >= self.min_ball_speed and float(v[2]) > 0.0
            if not lobbed:
                th.reason = "not approaching"
                return th
        elif self.camera_support > 0.0:
            # It approaches according to all cameras put together. One camera alone must see it approach
            # too: cameras that disagree about a still ball's range look, together, like a ball rushing in.
            seen = track.camera_closing()
            if seen is None:
                th.reason = "approach not yet seen three times by one camera"
                return th
            if seen - max(own, 0.0) < self.camera_support * (th.closing_speed - max(own, 0.0)):
                th.reason = "no single camera sees it approach (the cameras disagree about its range)"
                return th
        if th.closing_speed > self.max_closing or rng < self.min_range:
            th.reason = "implausible track"
            return th
        if ground_speed is not None and ground_speed < self.min_ball_speed:
            th.reason = "not moving (a still object)"
            return th

        ts = self.ts
        a = accel(v, tracker.drag_k)
        traj_w = p[None, :] + v[None, :] * ts[:, None] + 0.5 * a[None, :] * (ts * ts)[:, None]
        above = traj_w[:, 2] > self.ground_z + self.r_ball
        if not above[0]:
            th.reason = "on or below the floor"
            return th
        n_valid = len(ts) if above.all() else int(np.argmin(above))   # stop where it lands
        traj_b = traj_w[:n_valid] @ R_wb                               # rows: R_wb^T p
        gap = np.maximum(np.maximum(self.lo - traj_b, traj_b - self.hi), 0.0)
        dist = np.sqrt((gap * gap).sum(axis=1)) - self.r_ball
        hit = np.nonzero(dist <= self.margin)[0]
        i = int(hit[0]) if len(hit) else int(np.argmin(dist))
        th.tau = float(ts[i])
        th.miss_m = float(dist[i])

        v_tau_b = R_wb.T @ (v + a * ts[i])
        vh = v_tau_b[:2]
        vh_norm = float(np.linalg.norm(vh))
        if vh_norm > 0.3:
            u = vh / vh_norm
        else:                                     # dropping almost straight down: use where it is coming from
            pb = (R_wb.T @ p)[:2]
            pn = float(np.linalg.norm(pb))
            u = -pb / pn if pn > 1e-6 else np.array([1.0, 0.0])
        th.approach_dir = u
        th.bearing_deg = math.degrees(math.atan2(-u[1], -u[0]))
        th.descent_deg = math.degrees(math.atan2(-v_tau_b[2], max(vh_norm, 1e-6)))
        left = np.array([-u[1], u[0]])
        rel = traj_b[i, :2] - self.center_xy
        th.lateral_offset = float(rel @ left)
        th.along_offset = float(rel @ u)

        tau = th.tau
        P_tau = P[:3, :3] + tau * (P[:3, 3:] + P[3:, :3]) + tau * tau * P[3:, 3:]
        left_w = R_wb @ np.array([left[0], left[1], 0.0])
        th.sigma_lat = float(math.sqrt(max(0.0, left_w @ P_tau @ left_w)))
        g = gap[i]
        gn = float(np.linalg.norm(g))
        # direction of the miss: from the body toward the ball where they are closest
        sign = np.where(traj_b[i] > self.hi, 1.0, -1.0)
        miss_dir_w = R_wb @ (g * sign / gn) if gn > 1e-6 else left_w
        th.sigma_miss = float(math.sqrt(max(0.0, miss_dir_w @ P_tau @ miss_dir_w)))

        th.confident = th.sigma_lat <= self.sigma_lat_max
        reach = self.margin + self.k_sigma * min(th.sigma_miss, self.sigma_cap)
        if th.miss_m <= reach:
            th.is_threat = True
            th.reason = "predicted hit" if th.miss_m <= self.margin else "possible hit"
        else:
            th.reason = "will miss"
        return th
