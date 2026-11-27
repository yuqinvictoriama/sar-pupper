"""Closed-loop simulation of walking around obstacles: the real state machine and steering
code, a simple model of the robot's walking (it follows speed commands with a lag and an
error), and a range scan cast geometrically with the camera's field of view.

    python3 sim/avoid_sim.py                 # one run, printed
    python3 sim/avoid_sim.py --plot out.png  # with a picture of the path (needs matplotlib)

It checks the navigation logic - which side, how close, how much longer than the shortest
way, when it lets go - not perception (tests/test_perception.py does that on rendered
fisheye images).
"""

from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from terrain.floor_model import BOX_SIDE, OBSTACLE  # noqa: E402
from terrain.scan import Obstacle  # noqa: E402
from terrain.state_machine import AVOID, CRUISE, HOLD, FsmConfig, Inputs, TerrainFsm  # noqa: E402


@dataclass
class Rect:
    cx: float
    cy: float
    depth: float          # along world x
    width: float          # along world y
    tall: bool = True
    is_box: bool = False

    def corners(self):
        hx, hy = self.depth / 2, self.width / 2
        return np.array([[self.cx - hx, self.cy - hy], [self.cx + hx, self.cy - hy],
                         [self.cx + hx, self.cy + hy], [self.cx - hx, self.cy + hy]])

    def distance(self, p):
        dx = max(abs(p[0] - self.cx) - self.depth / 2, 0.0)
        dy = max(abs(p[1] - self.cy) - self.width / 2, 0.0)
        return math.hypot(dx, dy)


@dataclass
class AvoidSimConfig:
    obstacles: list = field(default_factory=lambda: [Rect(1.5, 0.0, 0.30, 0.40)])
    goal_distance: float = 3.2          # m straight ahead
    intent_speed: float = 0.35          # m/s the "driver" asks for
    fov_deg: float = 100.0              # the camera sees bearings within +-this
    max_range: float = 2.5
    camera_x: float = 0.145
    scan_hz: float = 8.0
    control_hz: float = 20.0
    speed_scale: float = 0.9            # actual / commanded speed (unknown to the robot)
    lag: float = 0.25                   # s, first-order response of the walk to commands
    range_noise: float = 0.015          # m
    yaw_drift: float = 0.0              # rad/s gyro drift
    seed: int = 0
    max_time: float = 40.0


def cast_scan(pose, rects, cfg, rng):
    """Obstacle foot points seen from the camera, body frame, one per degree of bearing."""
    x, y, th = pose
    cam = np.array([x + cfg.camera_x * math.cos(th), y + cfg.camera_x * math.sin(th)])
    pts, owner = [], []
    for b in np.arange(-cfg.fov_deg, cfg.fov_deg + 0.1, 1.0):
        a = th + math.radians(b)
        d = np.array([math.cos(a), math.sin(a)])
        best, who = None, -1
        for k, r in enumerate(rects):
            c = r.corners()
            for i in range(4):
                p, q = c[i], c[(i + 1) % 4]
                e = q - p
                den = d[0] * e[1] - d[1] * e[0]
                if abs(den) < 1e-12:
                    continue
                w = p - cam
                t = (w[0] * e[1] - w[1] * e[0]) / den
                u = (w[0] * d[1] - w[1] * d[0]) / den
                if t > 0.02 and 0.0 <= u <= 1.0 and (best is None or t < best):
                    best, who = t, k
        if best is not None and best < cfg.max_range:
            rr = best + rng.normal(0, cfg.range_noise)
            # into the body frame (origin at the body centre)
            pts.append([cfg.camera_x + rr * math.cos(math.radians(b)), rr * math.sin(math.radians(b))])
            owner.append(who)
    return np.array(pts).reshape(-1, 2), np.array(owner, dtype=int)


def to_obstacles(pts, owner, rects):
    out = []
    for k in sorted(set(owner.tolist())):
        p = pts[owner == k]
        az = np.degrees(np.arctan2(p[:, 1], p[:, 0]))
        rng = np.hypot(p[:, 0], p[:, 1])
        i = int(np.argmin(rng))
        out.append(Obstacle(left_deg=float(az.max()), right_deg=float(az.min()), nearest=float(rng.min()),
                            nearest_bearing_deg=float(az[i]), left_xy=tuple(p[np.argmax(az)]),
                            right_xy=tuple(p[np.argmin(az)]), width=float(np.linalg.norm(p[np.argmax(az)] - p[np.argmin(az)])),
                            height=float("inf") if rects[k].tall else 0.03, tall=rects[k].tall,
                            is_box=rects[k].is_box, columns=len(p), points=p))
    return out


def shortest_detour(rects, clearance, start, goal):
    """Length of the shortest path from start to goal that keeps ``clearance`` from every
    rectangle (visibility graph over the corners pushed out by the clearance)."""
    nodes = [np.array(start, dtype=float), np.array(goal, dtype=float)]
    for r in rects:
        hx, hy = r.depth / 2 + clearance, r.width / 2 + clearance
        for sx in (-1, 1):
            for sy in (-1, 1):
                # corner of the inflated rectangle, pushed out so the rounded corner is covered
                nodes.append(np.array([r.cx + sx * hx, r.cy + sy * hy]))

    def free(p, q):
        for r in rects:
            for s in np.linspace(0, 1, 60):
                if r.distance(p + (q - p) * s) < clearance - 0.012:
                    return False
        return True

    n = len(nodes)
    dist = np.full(n, np.inf)
    dist[0] = 0
    done = np.zeros(n, dtype=bool)
    for _ in range(n):
        i = int(np.argmin(np.where(done, np.inf, dist)))
        if done[i] or not np.isfinite(dist[i]):
            break
        done[i] = True
        for j in range(n):
            if not done[j] and free(nodes[i], nodes[j]):
                dist[j] = min(dist[j], dist[i] + float(np.linalg.norm(nodes[j] - nodes[i])))
    return float(dist[1])


@dataclass
class AvoidResult:
    reached: bool
    time: float
    path_length: float
    shortest: float
    min_clearance: float       # m, from the body centre to the nearest obstacle
    longest_stop: float        # s
    end_offset: float          # m, sideways from the original line at the end
    states: list
    events: list
    path: np.ndarray = field(default=None, repr=False)

    def line(self):
        return (f"{'REACHED' if self.reached else 'DID NOT REACH'} in {self.time:.1f} s: path {self.path_length:.2f} m "
                f"({self.path_length / self.shortest:.2f} x the shortest clear path {self.shortest:.2f} m), "
                f"closest {self.min_clearance * 1000:.0f} mm from an obstacle, longest stop {self.longest_stop:.1f} s, "
                f"ends {self.end_offset * 1000:+.0f} mm beside the original line")


def run_avoid(cfg: AvoidSimConfig | None = None, fsm_cfg: FsmConfig | None = None) -> AvoidResult:
    cfg = cfg or AvoidSimConfig()
    rng = np.random.default_rng(cfg.seed)
    fsm = TerrainFsm(fsm_cfg or FsmConfig())
    pose = np.array([0.0, 0.0, 0.0])
    vel = np.zeros(3)
    dt = 1.0 / cfg.control_hz
    t, next_scan = 0.0, 0.0
    yaw_bias = 0.0
    path, states, events = [pose[:2].copy()], [], []
    min_clear, stop_t, longest_stop, length = np.inf, 0.0, 0.0, 0.0
    pts = np.zeros((0, 2))
    obstacles = []
    while t < cfg.max_time:
        new_scan = t >= next_scan
        if new_scan:
            pts, owner = cast_scan(pose, cfg.obstacles, cfg, rng)
            obstacles = to_obstacles(pts, owner, cfg.obstacles)
            next_scan += 1.0 / cfg.scan_hz
        # the "driver": walk straight along world x (it knows nothing about obstacles)
        intent = (cfg.intent_speed, 0.0, float(np.clip(-1.5 * pose[2], -0.8, 0.8)))
        yaw_bias += cfg.yaw_drift * dt
        out = fsm.tick(Inputs(t=t, yaw=pose[2] + yaw_bias, level=True, obstacles=obstacles,
                              points=pts if new_scan else None,
                              seen_sector=(-math.radians(cfg.fov_deg), math.radians(cfg.fov_deg)),
                              scan_range=cfg.max_range, intent=intent))
        if out.event:
            events.append((round(t, 2), out.event))
        if not states or states[-1] != out.state:
            states.append(out.state)
        cmd = out.cmd if out.cmd is not None else intent        # the mux: terrain outranks the driver
        target = np.array(cmd) * np.array([cfg.speed_scale, cfg.speed_scale, 1.0])
        vel += (target - vel) * min(1.0, dt / cfg.lag)
        c, s = math.cos(pose[2]), math.sin(pose[2])
        step = np.array([(vel[0] * c - vel[1] * s) * dt, (vel[0] * s + vel[1] * c) * dt, vel[2] * dt])
        pose = pose + step
        length += float(np.hypot(step[0], step[1]))
        t += dt
        path.append(pose[:2].copy())
        min_clear = min(min_clear, min(r.distance(pose[:2]) for r in cfg.obstacles))
        if np.hypot(vel[0], vel[1]) < 0.03 and out.state != CRUISE:
            stop_t += dt
            longest_stop = max(longest_stop, stop_t)
        else:
            stop_t = 0.0
        if pose[0] >= cfg.goal_distance or out.state == HOLD and t > 5:
            break
    # The robot rejoins its original heading, not its original line (it has no odometry), so
    # it ends beside where it was going; compare with the shortest clear path to where it ended.
    reached = bool(pose[0] >= cfg.goal_distance)
    shortest = shortest_detour(cfg.obstacles, fsm.cfg.steer.clearance, (0.0, 0.0), tuple(pose[:2]))
    return AvoidResult(reached, t, length, shortest, float(min_clear), longest_stop, float(pose[1]), states, events, np.array(path))


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--offset", type=float, default=0.0, help="obstacle centre left of the path (m)")
    ap.add_argument("--width", type=float, default=0.40)
    ap.add_argument("--depth", type=float, default=0.30)
    ap.add_argument("--plot")
    args = ap.parse_args()
    cfg = AvoidSimConfig(obstacles=[Rect(1.5, args.offset, args.depth, args.width)])
    res = run_avoid(cfg)
    print(res.line())
    for t, e in res.events:
        print(f"  {t:6.2f} s  {e}")
    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, ax = plt.subplots(figsize=(8, 4))
        for r in cfg.obstacles:
            c = r.corners()
            ax.fill(c[:, 0], c[:, 1], color="0.6")
        ax.plot(res.path[:, 0], res.path[:, 1], "b-")
        ax.set_aspect("equal")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        fig.savefig(args.plot, dpi=120, bbox_inches="tight")


if __name__ == "__main__":
    main()
