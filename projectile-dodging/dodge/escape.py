"""Choose the escape move.

The robot steps sideways to the ball's ground track. Pupper is fast fore-aft
and slow sideways, so the plan commands both axes at once with signs chosen so
each carries the robot away from the track; the side that clears soonest wins.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np


def lag_distance(t: float, v: float, tau: float) -> float:
    """Distance covered t seconds after a velocity step v through a first-order lag tau."""
    if t <= 0.0:
        return 0.0
    if tau <= 1e-6:
        return v * t
    return v * (t - tau * (1.0 - math.exp(-t / tau)))


def lag_time(d: float, v: float, tau: float) -> float:
    """Time to cover distance d after a velocity step v through a first-order lag tau."""
    if d <= 0.0:
        return 0.0
    if v <= 1e-6:
        return float("inf")
    lo, hi = 0.0, d / v + 6.0 * max(tau, 0.0) + 1e-3
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        if lag_distance(mid, v, tau) < d:
            lo = mid
        else:
            hi = mid
    return hi


@dataclass
class EscapePlan:
    vx: float
    vy: float
    direction: np.ndarray = field(default_factory=lambda: np.zeros(2))  # unit vector, base_link
    mode: str = "perpendicular"
    speed: float = 0.0          # speed along `direction` the command delivers (m/s)
    clear_m: float = 0.0        # travel needed for the ball to miss
    t_clear: float = 0.0        # time to travel that far
    t_need: float = 0.0         # reaction + t_clear
    duration: float = 0.0       # how long to hold the command
    reason: str = ""

    def summary(self) -> dict:
        return {"vx": round(self.vx, 2), "vy": round(self.vy, 2), "mode": self.mode,
                "direction": [round(float(self.direction[0]), 2), round(float(self.direction[1]), 2)],
                "clear_m": round(self.clear_m, 3), "t_clear": round(self.t_clear, 3),
                "t_need": round(self.t_need, 3), "duration": round(self.duration, 3), "reason": self.reason}


def describe(vx: float, vy: float) -> str:
    """The commanded move in the robot's own terms."""
    fore = "forward" if vx > 0 else "backward" if vx < 0 else ""
    side = "left" if vy > 0 else "right" if vy < 0 else ""
    if fore and side:
        return "run %s-%s" % (fore, side)
    if fore:
        return "run %s" % fore
    return "sidestep %s" % side if side else "stay"


class FreeSpace:
    """Clearance around the robot in equal sectors (index 0 = straight ahead, counter-clockwise)."""

    def __init__(self, clearances):
        self.c = [float(v) for v in clearances]

    def clearance(self, direction) -> float:
        if not self.c:
            return float("inf")
        ang = math.atan2(direction[1], direction[0]) % (2.0 * math.pi)
        n = len(self.c)
        return self.c[int(round(ang / (2.0 * math.pi / n))) % n]


class EscapePlanner:
    def __init__(self, cfg: dict, robot: dict, ball: dict, margin: float):
        self.cfg = cfg
        box = robot["box"]
        self.hx = (box["x_max"] - box["x_min"]) / 2.0
        self.hy = (box["y_max"] - box["y_min"]) / 2.0
        self.height = box["z_top"] + float(robot["hip_height_m"])
        self.r_ball = ball["diameter_m"] / 2.0
        self.margin = margin
        self.vx_f = float(cfg.get("vx_forward", 1.0))
        self.vx_b = float(cfg.get("vx_backward", 0.8))
        self.vy = float(cfg.get("vy", 0.5))
        self.comp_min = float(cfg.get("component_min", 0.25))
        self.lag = float(cfg.get("accel_lag_s", 0.2))
        self.react = float(cfg.get("react_s", 0.12))
        self.deadband = float(cfg.get("center_deadband_m", 0.04))
        self.extra = float(cfg.get("extra_m", 0.08))
        self.max_disp = float(cfg.get("max_displacement_m", 0.4))
        self.min_dur = float(cfg.get("min_duration_s", 0.25))
        self.max_dur = float(cfg.get("max_duration_s", 0.8))
        self.past_arrival = float(cfg.get("hold_past_arrival_s", 0.15))
        self.along = bool(cfg.get("along_track", False))
        self.along_min_descent = float(cfg.get("along_track_min_descent_deg", 45.0))
        self.free_needed = float(cfg.get("free_space_needed_m", 0.6))

    def command_for(self, d):
        """(vx, vy, speed along d) of the fastest allowed command that moves along unit vector d."""
        vx = vy = 0.0
        if abs(d[0]) >= self.comp_min:
            vx = self.vx_f if d[0] > 0 else -self.vx_b
        if abs(d[1]) >= self.comp_min:
            vy = self.vy if d[1] > 0 else -self.vy
        return vx, vy, vx * d[0] + vy * d[1]

    def _candidate(self, d, disp, mode, tau=float("inf")):
        vx, vy, speed = self.command_for(d)
        disp = max(disp, 0.0)
        t_clear = lag_time(disp, speed, self.lag)
        travel = min(disp + self.extra, self.max_disp)
        # hold the command until the extra travel is covered, but not past the
        # ball's arrival (plus a moment) and never beyond the hard cap
        dur = min(lag_time(travel, speed, self.lag), self.max_dur, tau + self.past_arrival)
        dur = max(dur, self.min_dur)
        return EscapePlan(vx, vy, np.asarray(d, dtype=float), mode, speed, disp, t_clear, self.react + t_clear, dur)

    def plan(self, th, free_space: FreeSpace | None = None) -> EscapePlan:
        u = np.asarray(th.approach_dir, dtype=float)
        left = np.array([-u[1], u[0]])
        s = th.lateral_offset
        need = self.r_ball + self.margin
        half_perp = abs(left[0]) * self.hx + abs(left[1]) * self.hy
        cands = []
        for sign in (1.0, -1.0):
            # moving toward the side the ball passes on means going past it: longer
            c = self._candidate(sign * left, half_perp + need + sign * s, "perpendicular", th.tau)
            c.reason = "%s, across the ball's path" % describe(c.vx, c.vy)
            cands.append(c)
        if self.along and th.descent_deg >= self.along_min_descent:
            half_along = abs(u[0]) * self.hx + abs(u[1]) * self.hy
            run_out = self.height / math.tan(math.radians(max(th.descent_deg, 1.0)))
            a = th.along_offset
            safety = 0.15   # along-track prediction is the least certain
            c = self._candidate(-u, half_along - a + need + safety, "along", th.tau)
            c.reason = "%s toward the thrower, under the ball" % describe(c.vx, c.vy)
            cands.append(c)
            c = self._candidate(u, half_along + a + run_out + need + safety, "along", th.tau)
            c.reason = "%s away along the ball's path" % describe(c.vx, c.vy)
            cands.append(c)

        def blocked(c):
            return free_space is not None and free_space.clearance(c.direction) < self.free_needed

        open_c = [c for c in cands if not blocked(c)]
        pool = open_c if open_c else cands
        if abs(s) < self.deadband and len(pool) > 1:
            # dead centre: nothing to choose on geometry, so prefer open space, then forward motion
            pool = sorted(pool, key=lambda c: (round(c.t_clear, 2), -c.vx))
        else:
            pool = sorted(pool, key=lambda c: c.t_clear)
        best = pool[0]
        if not open_c and free_space is not None:
            best = max(cands, key=lambda c: free_space.clearance(c.direction))
            room = max(0.0, free_space.clearance(best.direction) - 0.15)
            best.duration = min(best.duration, max(self.min_dur, lag_time(room, best.speed, self.lag)))
            best.reason += " (both sides blocked: shortened)"
        return best
