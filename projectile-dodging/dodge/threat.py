"""Will the tracked ball hit the robot, when, and from where?

The track is flown forward on a ballistic arc. The robot is a box in
base_link. The answer is the closest the ball's surface gets to the box before
it lands, the time that happens (time-to-contact), and how sure we are.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .track import G, accel


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
    unseen_s: float = 0.0              # how long since a camera last saw it

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
        self.use_bearing = bool(cfg.get("use_bearing", True))
        self.min_climb = float(cfg.get("min_climb", 0.02))
        self.min_span = float(cfg.get("min_span_s", 0.045))
        self.two_look_wait = float(cfg.get("two_look_wait_s", 0.07))
        self.pool = tuple(tuple(g) for g in cfg.get("bearing_pool", [["elp_left", "elp_right"]]))
        self.bearing_max_closing = float(cfg.get("bearing_max_closing_speed", 7.0))
        self.aim_z = 0.0          # height the ball is taken to be aimed at: the middle of the body (base_link origin)
        self.loom_min = float(cfg.get("loom_min", 0.3))
        self.loom_support = float(cfg.get("loom_support", 0.3))

    def assess(self, track, t_now: float, tracker, R_wb: np.ndarray | None = None,
               robot_vel_w: np.ndarray | None = None) -> Threat:
        if not self.use_bearing:
            return self._assess_by_size(track, t_now, tracker, R_wb, robot_vel_w)
        return self._assess_by_bearing(track, t_now, tracker, R_wb, robot_vel_w)

    def _assess_by_bearing(self, track, t_now: float, tracker, R_wb: np.ndarray | None = None,
                           robot_vel_w: np.ndarray | None = None) -> Threat:
        """Is this ball on a course to the robot, judged from how its DIRECTION moves (Track.hit_course)?

        Seen on the Pupper, 2026-11-26: judged by size, a dodge came 0.12-0.18 s before the ball (it needs
        0.6 s), or during the thrower's wind-up, or for a ball lying in a basket whose blob flickered.
        A ball in free flight that will reach the robot climbs in elevation at g / (2 x closing speed)
        while its azimuth holds. That gives the closing speed from three looks of one camera, good to a
        few per cent, wherever the ball is and however small. The range, from size, is only needed for
        how long it still has to travel, where 15 % off is 15 % off the time and no more.

        What this cannot tell: whether the ball will come down on the robot or sail over it. A ball that
        will fall short does not climb and is left alone; one that goes long is dodged as well.
        """
        th = Threat(track.id)
        if R_wb is None:
            R_wb = np.eye(3)
        th.unseen_s = t_now - track.t_last
        # A ball the rear cameras saw twice, climbing, and then not again went up out of their sight
        # (see Tracker._climbs): it is judged on those two looks once a third is overdue.
        two = bool(getattr(track, "climber", False)) and track.hits == 2 and th.unseen_s >= self.two_look_wait
        if not two and (not track.confirmed or track.hits < self.min_hits):
            th.reason = "too few detections"
            return th
        if track.age < (0.02 if two else self.min_age):
            th.reason = "too new"
            return th
        looks = 2 if two else 3
        span = 0.02 if two else self.min_span
        hc = track.hit_course(min_looks=looks, min_span=span, pool=self.pool)
        if hc is None:
            th.reason = "not yet seen three times by one camera"
            return th
        drop = max(0.0, float(hc["origin"][2]) - self.aim_z)
        if drop > 0.0 and hc["q_dot"] >= G / (4.0 * self.bearing_max_closing) and hc["q_dot"] * hc["span"] >= 0.5 * self.min_climb:
            # second pass: the same looks, as seen from the point on the body the ball is taken to be aimed at
            v0 = G / (2.0 * hc["q_dot"])
            hc = track.hit_course(min_looks=looks, min_span=span, pool=self.pool, drop=drop, arrive=(v0, float(np.median(hc["ts"] + hc["r_h"] / v0))))
        q_dot, az_dot = hc["q_dot"], hc["az_dot"]
        r_mid = float(np.median(hc["r_h"]))
        th.range_m = r_mid * math.sqrt(1.0 + hc["q"] ** 2)
        own_closing = 0.0
        if robot_vel_w is not None and r_mid > 1e-3:
            # The robot's own walking turns the direction to a STILL thing too (one hanging ahead climbs
            # as the robot nears it). Take out what its walking alone would do to a still thing there.
            ux, uy, uz = (-float(c) for c in robot_vel_w)
            x, y, z = r_mid * math.cos(hc["az_mid"]), r_mid * math.sin(hc["az_mid"]), r_mid * hc["q_mid"]
            q_dot -= (uz * r_mid - z * (x * ux + y * uy) / r_mid) / (r_mid * r_mid)
            az_dot -= (x * uy - y * ux) / (r_mid * r_mid)
            own_closing = max(0.0, -(x * ux + y * uy) / r_mid)
        if q_dot * hc["span"] < self.min_climb or q_dot < G / (2.0 * self.bearing_max_closing):
            th.reason = "not approaching"          # still, sinking, or drifting: its bearing does not climb as an incoming ball's does
            return th
        v_c = G / (2.0 * q_dot)
        th.closing_speed = v_c
        if v_c < self.min_closing:
            th.reason = "not approaching"          # going almost straight up
            return th
        # When it arrives: each look's range, travelled at v_c, gives a time; take the middle one.
        arrive = hc["ts"] + hc["r_h"] / v_c
        T = float(np.median(arrive))
        tau = T - t_now
        if tau > float(self.ts[-1]):
            th.reason = "too far off"
            return th
        if tau < -0.15:
            th.reason = "already past"
            return th
        # Where the size CAN speak, it must not contradict: a ball this near, coming this fast, would
        # have grown plainly between the looks. (A ball lifted out of a basket 1.5 m away climbs like
        # a throw from there, and stays the same size.)
        if hc["ranges_good"] and len(hc["ts"]) >= 3:
            span = float(hc["ts"][-1] - hc["ts"][0])
            if span > 0.05 and v_c * span >= self.loom_min * r_mid:
                tc = hc["ts"] - hc["ts"].mean()
                grown = float(-(tc * (hc["r_h"] - hc["r_h"].mean())).sum() / (tc * tc).sum()) - own_closing
                if grown < self.loom_support * v_c:
                    th.reason = "climbing like a throw but not getting bigger (handled, not thrown)"
                    return th
        # The ball's state in the middle of the looks. Over the ground it flies straight at v_c along a
        # line that passes the lens at some distance b to the side; seen from the lens, its azimuth then
        # turns at b v_c / (D^2 + b^2), D being how far along that line it still has to go. So the
        # azimuth's rate gives b. From there it is flown forward on its arc, seen or not.
        s_mid = max(T - hc["t_mid"], 0.02)
        D = v_c * s_mid
        w = az_dot
        disc = v_c * v_c - 4.0 * w * w * D * D
        if abs(w) < 1e-6:
            b = 0.0
        elif disc <= 0.0:
            b = math.copysign(D, w)                # turning too fast to be passing near at all
        else:
            b = (v_c - math.sqrt(disc)) / (2.0 * w)
        az_line = hc["az_mid"] - math.atan2(b, D)
        e_along = np.array([math.cos(az_line), math.sin(az_line), 0.0])
        e_side = np.array([-math.sin(az_line), math.cos(az_line), 0.0])
        r_h = math.hypot(D, b)
        q = hc["q_mid"]
        vz = -(v_c * D / r_h) * q + r_h * q_dot
        p = hc["origin"] + D * e_along + b * e_side + np.array([0.0, 0.0, r_h * q - drop])
        v = -v_c * e_along + np.array([0.0, 0.0, vz])
        dt = t_now - hc["t_mid"]
        p = p + v * dt + np.array([0.0, 0.0, -0.5 * G * dt * dt])
        v = v + np.array([0.0, 0.0, -G * dt])
        rng = float(np.linalg.norm(p))
        th.range_m = rng
        th.speed = float(np.linalg.norm(v))
        if rng < self.min_range:
            th.reason = "implausible track"
            return th
        left, i, gap, traj_b = self._fly(th, p, v, 0.0, R_wb)
        if left is None:
            return th
        # How sure: the sideways miss is az_dot x v_c x s^2, so its scatter follows the azimuth fit's.
        th.sigma_lat = float(hc["sig_az_dot"] * v_c * s_mid * s_mid + max(hc["rms_az"], 0.004) * D)
        th.sigma_miss = th.sigma_lat
        th = self._verdict(th)
        if th.is_threat:
            track.keep_until = max(getattr(track, "keep_until", 0.0), t_now + th.tau + 0.1)
        return th

    def _assess_by_size(self, track, t_now: float, tracker, R_wb: np.ndarray | None = None,
               robot_vel_w: np.ndarray | None = None) -> Threat:
        """The earlier way (threat.use_bearing: false): the ball's speed toward the robot comes from the
        tracking filter, that is, from how fast the ball grows in the picture.

        robot_vel_w: the robot's own velocity in the world frame, if known. The track is relative to the
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

        left, i, gap, traj_b = self._fly(th, p, v, tracker.drag_k, R_wb)
        if left is None:
            return th
        tau = th.tau
        left_w = R_wb @ np.array([left[0], left[1], 0.0])
        P_tau = P[:3, :3] + tau * (P[:3, 3:] + P[3:, :3]) + tau * tau * P[3:, 3:]
        th.sigma_lat = float(math.sqrt(max(0.0, left_w @ P_tau @ left_w)))
        g = gap[i]
        gn = float(np.linalg.norm(g))
        # direction of the miss: from the body toward the ball where they are closest
        sign = np.where(traj_b[i] > self.hi, 1.0, -1.0)
        miss_dir_w = R_wb @ (g * sign / gn) if gn > 1e-6 else left_w
        th.sigma_miss = float(math.sqrt(max(0.0, miss_dir_w @ P_tau @ miss_dir_w)))

        return self._verdict(th)

    def _fly(self, th: Threat, p: np.ndarray, v: np.ndarray, drag_k: float, R_wb: np.ndarray):
        """Fly the ball forward from (p, v) and fill in when and where it comes nearest the body.
        Returns (left, i, gap, traj_b), or (None, ...) with th.reason set if it is already down."""
        ts = self.ts
        a = accel(v, drag_k)
        traj_w = p[None, :] + v[None, :] * ts[:, None] + 0.5 * a[None, :] * (ts * ts)[:, None]
        above = traj_w[:, 2] > self.ground_z + self.r_ball
        if not above[0]:
            th.reason = "on or below the floor"
            return None, None, None, None
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

        return left, i, gap, traj_b

    def _verdict(self, th: Threat) -> Threat:
        th.confident = th.sigma_lat <= self.sigma_lat_max
        reach = self.margin + self.k_sigma * min(th.sigma_miss, self.sigma_cap)
        if th.miss_m <= reach:
            th.is_threat = True
            th.reason = "predicted hit" if th.miss_m <= self.margin else "possible hit"
        else:
            th.reason = "will miss"
        return th
