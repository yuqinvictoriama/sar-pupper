"""Throw simulator for testing the dodge core without a robot.

Simulates a ball in flight (gravity + drag), what each camera would detect
(with pixel noise, frame rate and delay), and a robot that responds to velocity
commands through a delay and a first-order lag. Used by tests/ and by
tools/sim_report.py. It checks the logic and the timing budget; it cannot check
colour detection or the real robot's response.
"""
from __future__ import annotations

import math

import numpy as np

from .core import DodgeCore
from .detect import Blob
from .track import G, drag_coefficient


class Throw:
    """A ball trajectory in the world frame (origin = robot's base_link at release, z up)."""

    def __init__(self, p0, v0, drag_k: float, floor_z: float, radius: float, dt: float = 0.001, t_max: float = 3.0):
        self.dt = dt
        p, v = np.array(p0, dtype=float), np.array(v0, dtype=float)
        pts = [p.copy()]
        t = 0.0
        while t < t_max and p[2] > floor_z + radius:
            a = np.array([0.0, 0.0, -G]) - drag_k * np.linalg.norm(v) * v
            v = v + a * dt
            p = p + v * dt
            pts.append(p.copy())
            t += dt
        self.pts = np.array(pts)
        self.t_land = (len(pts) - 1) * dt

    def position(self, t: float):
        """Ball position at time t, or None once it has landed."""
        if t < 0 or t > self.t_land:
            return None
        i = t / self.dt
        i0 = min(int(i), len(self.pts) - 1)
        i1 = min(i0 + 1, len(self.pts) - 1)
        f = i - i0
        return self.pts[i0] * (1 - f) + self.pts[i1] * f


def aimed_throw(cfg: dict, bearing_deg: float, distance: float, release_h: float = 1.0, rise: float = 0.7,
                offset=(0.0, 0.0), target_z: float = 0.05) -> Throw:
    """A throw from `distance` m away at `bearing_deg` (0 = ahead, 90 = left) that would land on the robot.

    release_h  height of the hand above the floor
    rise       how far the ball climbs above the release point (0 = thrown flat)
    offset     (sideways, along) shift of the aim point in metres, sideways = to the left of the ball's travel
    """
    ball, robot = cfg["ball"], cfg["robot"]
    k = drag_coefficient(ball)
    hip = float(robot["hip_height_m"])
    floor_z, radius = -hip, ball["diameter_m"] / 2.0
    b = math.radians(bearing_deg)
    start = np.array([distance * math.cos(b), distance * math.sin(b), release_h - hip])
    box = robot["box"]
    centre = np.array([(box["x_min"] + box["x_max"]) / 2.0, 0.0])
    travel = centre - start[:2]
    travel /= np.linalg.norm(travel)
    left = np.array([-travel[1], travel[0]])
    target = centre + offset[0] * left + offset[1] * travel
    horiz = target - start[:2]
    dist_h = float(np.linalg.norm(horiz))
    direction = horiz / dist_h
    vz0 = math.sqrt(2.0 * G * rise) if rise > 0 else 0.0

    def reach(vh: float) -> float:
        p = np.array([0.0, start[2]])
        v = np.array([vh, vz0])
        dt = 0.001
        for _ in range(4000):
            sp = math.hypot(v[0], v[1])
            v = v + np.array([-k * sp * v[0], -G - k * sp * v[1]]) * dt
            p = p + v * dt
            if v[1] < 0 and p[1] <= target_z:
                break
        return p[0]

    lo, hi = 0.3, 40.0
    for _ in range(50):
        mid = 0.5 * (lo + hi)
        if reach(mid) < dist_h:
            lo = mid
        else:
            hi = mid
    vh = 0.5 * (lo + hi)
    v0 = np.array([direction[0] * vh, direction[1] * vh, vz0])
    return Throw(start, v0, k, floor_z, radius)


class SimCamera:
    """What one camera reports about the ball: a noisy blob per frame, delivered late."""

    def __init__(self, camera, fps: float, latency: float, rng, centroid_sigma: float = 0.3,
                 diameter_sigma: float = 0.35, min_area: float = 28.3, p_detect: float = 0.97, phase: float | None = None):
        self.camera, self.fps, self.latency, self.rng = camera, fps, latency, rng
        self.centroid_sigma, self.diameter_sigma = centroid_sigma, diameter_sigma
        self.min_area, self.p_detect = min_area, p_detect
        self.next_frame = rng.uniform(0, 1.0 / fps) if phase is None else phase
        self.pending = []      # (t_available, t_exposure, blobs)

    def blob_for(self, p_rel) -> Blob | None:
        cam = self.camera
        uv, ok, p_cam = cam.project_base(np.asarray(p_rel)[None, :])
        if not ok[0]:
            return None
        u, v = float(uv[0, 0]), float(uv[0, 1])
        rng_m = float(np.linalg.norm(p_cam[0]))
        r = self.ball_radius
        if rng_m <= r * 1.5:
            return None
        theta = 2.0 * math.asin(r / rng_m)
        omega = 2.0 * math.pi * (1.0 - math.cos(theta / 2.0))
        sr = cam.model.solid_angle_per_px(u, v)
        if not (sr > 0):
            return None
        d = 2.0 * math.sqrt(omega / sr / math.pi) * getattr(self, "size_bias", 1.0) + self.rng.normal(0, self.diameter_sigma)
        if d <= 0:
            return None
        area = math.pi * d * d / 4.0
        if area < self.min_area or self.rng.uniform() > self.p_detect:
            return None
        u += self.rng.normal(0, self.centroid_sigma)
        v += self.rng.normal(0, self.centroid_sigma)
        w, h = cam.model.width, cam.model.height
        edge = u - d / 2 <= 0 or v - d / 2 <= 0 or u + d / 2 >= w - 1 or v + d / 2 >= h - 1
        return Blob(u, v, area, int(u - d / 2), int(v - d / 2), int(math.ceil(d)), int(math.ceil(d)), bool(edge))

    def step(self, t: float, balls_rel) -> None:
        """Expose a frame if one is due. balls_rel: ball positions relative to the robot, base_link."""
        while self.next_frame <= t:
            blobs = []
            for p in balls_rel:
                b = self.blob_for(p)
                if b is not None:
                    blobs.append(b)
            self.pending.append((self.next_frame + self.latency, self.next_frame, blobs))
            self.next_frame += 1.0 / self.fps

    def ready(self, t: float):
        out = [f for f in self.pending if f[0] <= t]
        self.pending = [f for f in self.pending if f[0] > t]
        return out


DEFAULT_CAMERA_TIMING = {
    # name: (processed frames per second, delay from exposure to the core seeing the blobs)
    "fisheye": (30.0, 0.070),
    "elp_left": (60.0, 0.035),
    "elp_right": (60.0, 0.035),
}


def mujoco_response(vx: float, vy: float):
    """How the walking policy answers a velocity step, per axis: ((gain_x, lag_x), (gain_y, lag_y)).

    Fitted to the MuJoCo study of default_policy.json (reports/falls-mujoco, vx_scan.txt: distance 0.5 s
    after the decision, 40-50 ms actuation delay; use with actuation_delay about 0.105 s, which is the
    0.06 s step delay of that study plus its 45 ms). Steady speeds there were within about 10 % of the
    command fore-aft and 0.36-0.44 m/s for a sideways command of 0.5. From a forward command of 0.5 up the
    policy changes to a faster gait that covers 0.09-0.10 m in that half second whatever the command
    (0.51 or 1.0): a lag that grows with the command reproduces that.
    """
    if vx >= 0.5:
        x = (1.0, max(0.2, -0.13 + 0.83 * vx))     # 0.51 -> 0.29 s, 0.8 -> 0.53 s, 1.0 -> 0.70 s
    elif vx > 0.0:
        x = (1.0, 0.15)
    else:
        x = (1.0, 0.2)
    return x, (0.8, 0.2)


def box_clearance(robot: dict, radius: float, p_rel) -> float:
    """Distance from the ball's surface to the robot's body outline (negative = touching)."""
    box = robot["box"]
    lo = np.array([box["x_min"], box["y_min"], -float(robot["hip_height_m"])])
    hi = np.array([box["x_max"], box["y_max"], box["z_top"]])
    gap = np.maximum(np.maximum(lo - p_rel, p_rel - hi), 0.0)
    return float(np.linalg.norm(gap)) - radius


def run_closed_loop(cfg: dict, throw: Throw, seed: int = 0, armed: bool = True, timing: dict | None = None,
                    robot_lag: float = 0.2, actuation_delay: float = 0.06, floor_balls=(), t_start: float = 100.0,
                    timestamp_error: float = 0.0, policy_clip=(1.5, 0.5), trace: bool = False,
                    reliable_px: float = 6.0, size_bias: float = 1.0, more_throws=(), robot_response=None) -> dict:
    """Fly one throw at a robot running the dodge core. Returns what happened.

    floor_balls: positions (world) of extra balls lying still, to check they are ignored.
    more_throws: (Throw, delay_s) pairs, further balls released delay_s after the first. Each one's
    result is in "balls" (the first ball's is also the top-level result, as before).
    robot_response: None for one lag (robot_lag) on both axes, or a function (vx, vy) -> ((gain_x, lag_x),
    (gain_y, lag_y)) for a robot whose answer depends on the command, e.g. mujoco_response.
    """
    rng = np.random.default_rng(seed)
    core = DodgeCore(cfg, armed=armed)
    timing = timing or DEFAULT_CAMERA_TIMING
    cams = []
    for name, cam in core.cameras.items():
        fps, lat = timing[name]
        sc = SimCamera(cam, fps, lat, rng, diameter_sigma=float(cam.cfg.get("size_sigma_px", 0.4)) * 0.8,
                       min_area=math.pi * reliable_px ** 2 / 4.0)
        sc.ball_radius = cfg["ball"]["diameter_m"] / 2.0
        sc.size_bias = size_bias
        cams.append((name, sc))
    radius = cfg["ball"]["diameter_m"] / 2.0
    robot_p = np.zeros(3)
    robot_v = np.zeros(2)
    cmd_queue = []          # (t_effective, vx, vy)
    cmd_now = np.zeros(2)
    dt = 0.002
    next_cmd_poll = 0.0
    min_clear = float("inf")
    min_clear_static = float("inf")     # the same throw at a robot that never moves
    trigger_t = None
    first_seen = None
    log = []
    t = 0.0
    extra = [(th, float(dl)) for th, dl in more_throws]
    xclear = [float("inf")] * len(extra)
    xstatic = [float("inf")] * len(extra)
    t_end = max([throw.t_land] + [th.t_land + dl for th, dl in extra])
    while t <= t_end + 0.05:
        ball = throw.position(t)
        balls_rel = [np.asarray(fb, dtype=float) - robot_p for fb in floor_balls]
        if ball is not None:
            balls_rel.append(ball - robot_p)
            min_clear = min(min_clear, box_clearance(cfg["robot"], radius, ball - robot_p))
            min_clear_static = min(min_clear_static, box_clearance(cfg["robot"], radius, ball))
        for k, (th, dl) in enumerate(extra):
            b = th.position(t - dl)
            if b is not None:
                balls_rel.append(b - robot_p)
                xclear[k] = min(xclear[k], box_clearance(cfg["robot"], radius, b - robot_p))
                xstatic[k] = min(xstatic[k], box_clearance(cfg["robot"], radius, b))
        for name, sc in cams:
            sc.step(t, balls_rel)
            for _avail, t_exp, blobs in sc.ready(t):
                if blobs and first_seen is None and ball is not None:
                    first_seen = t
                started = core.process_blobs(name, t_start + t_exp + timestamp_error, blobs, t_start + t)
                if started and trigger_t is None:
                    trigger_t = t
        if t >= next_cmd_poll:
            next_cmd_poll += 0.01
            c = core.command(t_start + t)
            vx, vy = (c[0], c[1]) if c is not None else (0.0, 0.0)
            vx = max(-policy_clip[0], min(policy_clip[0], vx))
            vy = max(-policy_clip[1], min(policy_clip[1], vy))
            cmd_queue.append((t + actuation_delay, vx, vy))
        while cmd_queue and cmd_queue[0][0] <= t:
            _, vx, vy = cmd_queue.pop(0)
            cmd_now = np.array([vx, vy])
            core.set_robot_velocity(t_start + t, vx, vy)     # as the node hears the mux's output on /cmd_vel
        if robot_response is None:
            robot_v += (cmd_now - robot_v) * (dt / max(robot_lag, 1e-3))
        else:
            (gx, lx), (gy, ly) = robot_response(float(cmd_now[0]), float(cmd_now[1]))
            robot_v[0] += (gx * cmd_now[0] - robot_v[0]) * (dt / max(lx, 1e-3))
            robot_v[1] += (gy * cmd_now[1] - robot_v[1]) * (dt / max(ly, 1e-3))
        robot_p[:2] += robot_v * dt
        if trace:
            log.append((t, core.state.value, robot_p.copy()))
        t += dt
    events = core.pop_events()
    dodge = next((e for e in events if e["event"] == "dodge"), None)
    return {"would_hit": min_clear_static <= 0.0, "hit": min_clear <= 0.0, "min_clearance": min_clear,
            "static_clearance": min_clear_static, "triggered": trigger_t is not None, "trigger_t": trigger_t,
            "time_left": (throw.t_land - trigger_t) if trigger_t is not None else None,
            "first_seen": first_seen, "flight_time": throw.t_land, "robot_shift": robot_p[:2].copy(),
            "dodge_event": dodge, "events": events, "log": log,
            "dodges": sum(1 for e in events if e["event"] == "dodge"),
            "balls": [{"would_hit": min_clear_static <= 0.0, "hit": min_clear <= 0.0, "min_clearance": min_clear}]
                     + [{"would_hit": xs <= 0.0, "hit": xc <= 0.0, "min_clearance": xc} for xc, xs in zip(xclear, xstatic)]}
