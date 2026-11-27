"""Walking around an obstacle by the short way: tangent steering on the range scan.

Each tick: of all headings that clear every known obstacle point by the clearance distance,
take the one closest to where the robot wants to go. When the goal heading itself is clear
that is the goal heading (walk straight); when it is blocked, the nearest clear heading
grazes the obstacle at exactly the clearance - the tangent - which is the locally shortest
way round (the idea of TangentBug, Kamon, Rimon and Rivlin 1998). The robot walks forward
and turns at the same time; it does not stop and does not sidestep.

Obstacle points that leave the camera's view are remembered for a few seconds and moved with
the robot's own motion (commanded speed and IMU yaw), so the robot does not turn back into
something it has just passed.

Pure numpy. Angles in radians, body frame: x forward, y left, + angle = to the left.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class SteerConfig:
    clearance: float = 0.22          # m, obstacle point to the robot's centreline
    look_ahead: float = 1.0          # m of path that must be clear
    speed: float = 0.35              # m/s forward while avoiding
    min_speed: float = 0.20          # m/s: the walking policy barely moves below this
    yaw_gain: float = 1.8            # 1/s
    max_yaw_rate: float = 1.0        # rad/s
    turn_in_place_above: float = math.radians(70.0)   # heading error beyond which it stops walking and turns
    heading_step: float = math.radians(2.0)
    heading_span: float = math.radians(110.0)
    memory_time: float = 6.0         # s that unseen obstacle points are kept
    speed_scale: float = 1.0         # actual / commanded walking speed (measure it)
    done_heading: float = math.radians(8.0)
    min_gap: float = 0.45            # m: a gap narrower than this is not passable


class LocalMap:
    """Obstacle foot points around the robot, in the body frame, with their age."""

    def __init__(self, cfg: SteerConfig | None = None):
        self.cfg = cfg or SteerConfig()
        self.points = np.zeros((0, 2))
        self.stamps = np.zeros(0)

    def clear(self):
        self.points = np.zeros((0, 2))
        self.stamps = np.zeros(0)

    def move(self, dx: float, dy: float, dyaw: float) -> None:
        """The robot moved dx, dy (m, in its old body frame) and turned dyaw (rad, + left)."""
        if not len(self.points):
            return
        p = self.points - np.array([dx, dy])
        c, s = math.cos(-dyaw), math.sin(-dyaw)
        self.points = p @ np.array([[c, s], [-s, c]])

    def add_scan(self, t: float, points, seen_lo: float, seen_hi: float, max_range: float) -> None:
        """New scan points (k, 2). Remembered points inside the sector the camera has just
        looked at (bearings seen_lo..seen_hi, out to max_range) are replaced by what it saw."""
        if len(self.points):
            az = np.arctan2(self.points[:, 1], self.points[:, 0])
            rng = np.hypot(self.points[:, 0], self.points[:, 1])
            in_view = (az > seen_lo) & (az < seen_hi) & (rng < max_range) & (rng > 0.18)
            fresh = t - self.stamps < self.cfg.memory_time
            keep = ~in_view & fresh
            self.points, self.stamps = self.points[keep], self.stamps[keep]
        points = np.asarray(points, dtype=float).reshape(-1, 2)
        if len(points):
            self.points = np.vstack([self.points, points])
            self.stamps = np.concatenate([self.stamps, np.full(len(points), t)])


CONTACT_RADIUS = 0.15      # m: a point this close to the body centre is in the way of every heading


def _clear_headings(points, headings, clearance, look_ahead):
    """For each heading, is the straight path of length look_ahead clear by ``clearance``?

    A point counts against a heading only if walking that way brings the robot nearer to it.
    A point already beside the robot, a little inside the clearance (the corner it is
    rounding, read a centimetre or two short), does not block headings that lead away from
    it; without this, one such point blocked every heading and the robot stood still beside
    the obstacle it had all but passed. Only a point within ``CONTACT_RADIUS`` blocks them all."""
    if not len(points):
        return np.ones(len(headings), dtype=bool)
    ux, uy = np.cos(headings)[:, None], np.sin(headings)[:, None]
    proj = points[None, :, 0] * ux + points[None, :, 1] * uy
    along = np.clip(proj, 0.0, look_ahead)
    dist = np.hypot(points[None, :, 0] - along * ux, points[None, :, 1] - along * uy)
    touching = np.hypot(points[:, 0], points[:, 1])[None, :] < min(clearance, CONTACT_RADIUS)
    return ~((dist < clearance) & ((proj > 0.0) | touching)).any(axis=1)


@dataclass
class SteerResult:
    vx: float
    wz: float
    heading: float           # rad, the heading it is steering to (body frame)
    blocked: bool            # the goal heading is not clear
    side: int                # +1 passing with the obstacle on its right (going left), -1 the reverse, 0 none
    stuck: bool = False      # no clear heading at all


def steer(points, goal_heading: float, cfg: SteerConfig | None = None, side: int = 0) -> SteerResult:
    """One steering decision.

    points: (k, 2) obstacle points in the body frame. goal_heading: where the robot wants to
    go, relative to its nose. side: the side it committed to on an earlier tick (0 = free
    choice); once it starts round one way it keeps going that way.
    """
    cfg = cfg or SteerConfig()
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    n = int(round(cfg.heading_span / cfg.heading_step))
    offsets = np.arange(-n, n + 1) * cfg.heading_step
    headings = goal_heading + offsets
    clear = _clear_headings(points, headings, cfg.clearance, cfg.look_ahead)
    i_goal = n
    blocked = not clear[i_goal]
    if not blocked:
        chosen, new_side = goal_heading, 0
    else:
        left = next((k for k in range(i_goal + 1, len(headings)) if clear[k]), None)
        right = next((k for k in range(i_goal - 1, -1, -1) if clear[k]), None)
        if side > 0 and left is not None:
            pick = left
        elif side < 0 and right is not None:
            pick = right
        elif left is None and right is None:
            return SteerResult(0.0, 0.0, goal_heading, True, side, stuck=True)
        elif left is None or right is None:
            pick = left if left is not None else right
        else:
            pick = left if _detour_cost(points, headings, left, i_goal, cfg) <= \
                _detour_cost(points, headings, right, i_goal, cfg) else right
        chosen = float(headings[pick])
        new_side = 1 if pick > i_goal else -1
    wz = float(np.clip(cfg.yaw_gain * chosen, -cfg.max_yaw_rate, cfg.max_yaw_rate))
    if abs(chosen) > cfg.turn_in_place_above:
        vx = 0.0
    else:
        vx = max(cfg.min_speed, cfg.speed * math.cos(chosen))
    return SteerResult(vx, wz, chosen, blocked, new_side)


def _detour_cost(points, headings, pick, i_goal, cfg):
    """Length of the path that goes round on this side: out to the obstacle's edge along the
    chosen heading, then on to a point far ahead along the goal heading."""
    lo, hi = sorted((float(headings[i_goal]), float(headings[pick])))
    az = np.arctan2(points[:, 1], points[:, 0])
    rng = np.hypot(points[:, 0], points[:, 1])
    between = (az >= lo - 0.2) & (az <= hi + 0.2)
    d_edge = float(rng[between].min()) if between.any() else cfg.look_ahead
    goal = 3.0 * np.array([math.cos(headings[i_goal]), math.sin(headings[i_goal])])
    tangent = d_edge * np.array([math.cos(headings[pick]), math.sin(headings[pick])])
    return d_edge + float(np.linalg.norm(goal - tangent))


def path_clear(points, heading: float, half_width: float, length: float) -> bool:
    """Is the corridor of this half-width, straight along ``heading`` for ``length``, free?"""
    points = np.asarray(points, dtype=float).reshape(-1, 2)
    return bool(_clear_headings(points, np.array([heading]), half_width, length)[0])
