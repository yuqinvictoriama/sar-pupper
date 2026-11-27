"""Where Pupper is, without odometry.

Pupper has no wheel counts and no depth sensor. What it does have is the
velocity the mux actually sent to the legs (/cmd_vel, so dodges and
walk-arounds are included) and the IMU's yaw. Those are integrated into a
position in a frame fixed at the start, and that position is read against
"the line": the direction of the cry.

    s   metres along the line from the start mark
    d   metres off the line, left positive

Three things keep the estimate honest: speed factors measured on the demo
floor, the low box as a landmark (terrain agility measures the distance to
it within about a centimetre, and its place on the course is known), and
counting nothing while the legs are not listening (a crossing, and the
seconds the walking controller takes to come back).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

from .heading import wrap


@dataclass
class Line:
    x0: float           # where the line was drawn from, start frame
    y0: float
    heading: float      # rad, start frame (IMU yaw)
    s0: float           # distance already covered at that point


class Reckoner:
    def __init__(self, forward_factor: float = 1.0, sideways_factor: float = 1.0, lag_s: float = 0.2):
        self.kf, self.ks, self.lag = float(forward_factor), float(sideways_factor), float(lag_s)
        self.reset()

    def reset(self) -> None:
        self.x = self.y = 0.0           # start frame: origin where the run began, axes the IMU's
        self.yaw = 0.0
        self.vx = self.vy = 0.0         # what the legs are believed to be doing (body frame, after the lag)
        self.t = None
        self.line: Line | None = None
        self.first_heading: float | None = None     # the first line's heading: the course's x axis
        self.distance = 0.0             # path length walked, for the log

    # ------------------------------------------------------------------ motion
    def update(self, t: float, cmd, yaw: float, legs_listening: bool = True) -> None:
        """cmd: (vx, vy, wz) the mux last sent to the legs, or None for nothing."""
        dt = 0.0 if self.t is None else max(0.0, min(0.5, t - self.t))
        self.t, self.yaw = t, yaw
        tx, ty = (cmd[0] * self.kf, cmd[1] * self.ks) if (cmd is not None and legs_listening) else (0.0, 0.0)
        if not legs_listening:
            self.vx = self.vy = 0.0
        elif dt > 0.0:
            a = 1.0 if self.lag <= 0.0 else min(1.0, dt / self.lag)
            self.vx += (tx - self.vx) * a
            self.vy += (ty - self.vy) * a
        c, s = math.cos(yaw), math.sin(yaw)
        dx, dy = (self.vx * c - self.vy * s) * dt, (self.vx * s + self.vy * c) * dt
        self.x += dx
        self.y += dy
        self.distance += math.hypot(dx, dy)

    def advance(self, metres: float) -> None:
        """The body moved this far straight ahead without velocity commands (a crossing)."""
        self.x += metres * math.cos(self.yaw)
        self.y += metres * math.sin(self.yaw)
        self.distance += abs(metres)

    # ------------------------------------------------------------------ the line
    def set_line(self, heading: float, s0: float | None = None) -> None:
        """Draw the line from where Pupper stands now. s0 None keeps the distance covered so far."""
        if s0 is None:
            s0 = self.s
        self.line = Line(self.x, self.y, wrap(heading), float(s0))
        if self.first_heading is None:
            self.first_heading = self.line.heading

    @property
    def s(self) -> float:
        if self.line is None:
            return 0.0
        L = self.line
        return L.s0 + (self.x - L.x0) * math.cos(L.heading) + (self.y - L.y0) * math.sin(L.heading)

    @property
    def d(self) -> float:
        if self.line is None:
            return 0.0
        L = self.line
        return -(self.x - L.x0) * math.sin(L.heading) + (self.y - L.y0) * math.cos(L.heading)

    @property
    def heading_error(self) -> float:
        """Yaw relative to the line, left positive."""
        return 0.0 if self.line is None else wrap(self.yaw - self.line.heading)

    def set_s(self, s: float) -> None:
        """A landmark says the distance along the line is s: slide the line's zero to agree."""
        if self.line is not None:
            self.line.s0 += float(s) - self.s

    def point(self, s: float) -> tuple[float, float]:
        """The point on the line at distance s, start frame."""
        L = self.line
        return L.x0 + (s - L.s0) * math.cos(L.heading), L.y0 + (s - L.s0) * math.sin(L.heading)

    def bearing_to(self, s: float) -> float:
        """Where the point at distance s on the line is, from Pupper's nose (rad, left positive)."""
        px, py = self.point(s)
        return wrap(math.atan2(py - self.y, px - self.x) - self.yaw)

    # ------------------------------------------------------------------ for drawing
    def course_pose(self) -> tuple[float, float, float]:
        """(x, y, yaw) in the course frame: origin at the start mark, x along the first line."""
        h = self.first_heading
        if h is None:
            return 0.0, 0.0, 0.0
        c, s = math.cos(-h), math.sin(-h)
        return self.x * c - self.y * s, self.x * s + self.y * c, wrap(self.yaw - h)

    def course_line(self) -> dict | None:
        """The current line in the course frame, for the map."""
        if self.line is None or self.first_heading is None:
            return None
        h, L = self.first_heading, self.line
        c, s = math.cos(-h), math.sin(-h)
        return {"x0": L.x0 * c - L.y0 * s, "y0": L.x0 * s + L.y0 * c, "heading": wrap(L.heading - h), "s0": L.s0}
