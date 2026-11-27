"""The terrain-agility state machine. Pure Python; robot/terrain_node.py feeds it and carries
out what it asks for, and sim/avoid_sim.py drives the same code in simulation.

    CRUISE -> (tall obstacle ahead)  AVOID ------------------------------> CRUISE
           -> (the two-colour box)   APPROACH -> GAUGE -> (no)  AVOID
                                                        -> (yes) ALIGN -> GAUGE ... -> PLAN -> STEP_OVER -> CRUISE
    any state -> HOLD (stand still, report) when something is wrong

In CRUISE it is silent: whoever is driving (planner, Gemini, person follower) drives. It
takes over the velocity command only while it is dealing with an obstacle, and it needs to
know where the robot was going: the "intent" (the driver's own command) and the IMU heading
at the moment it took over.

Each ``tick`` returns the velocity to publish (or None to stay silent) and, in PLAN, the box
to plan a crossing for; the node runs the planner and the crossing and reports back through
``plan_result`` and ``crossing_result``.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import decision as D
from .gauge import BoxGauge
from .steering import LocalMap, SteerConfig, path_clear, steer

CRUISE, AVOID, APPROACH, GAUGE, ALIGN, PLAN, STEP_OVER, HOLD = (
    "CRUISE", "AVOID", "APPROACH", "GAUGE", "ALIGN", "PLAN", "STEP_OVER", "HOLD")


@dataclass
class FsmConfig:
    corridor_half: float = 0.18        # m: half-width of the lane an obstacle must be in to matter
    engage_range: float = 1.2          # m from the body centre: start avoiding a tall obstacle
    box_engage_range: float = 1.0      # m: start approaching the step-over box
    gauge_distance: float = 0.36       # m, body centre to the box's near face, to measure from
    gauge_tolerance: float = 0.04
    approach_speed: float = 0.22       # m/s
    settle_time: float = 0.7           # s standing still before measuring
    gauge_timeout: float = 4.0         # s
    align_yaw_rate: float = 0.5        # rad/s pulses
    align_side_speed: float = 0.22     # m/s pulses
    align_min_pulse: float = 0.30      # s
    align_max_tries: int = 5
    align_skew_deg: float = 4.0        # square enough
    align_centre: float = 0.05         # m, centred enough
    intent_timeout: float = 0.4        # s without a driver's command -> stop meddling
    intent_min_speed: float = 0.05     # m/s: slower than this the driver is not going anywhere
    hold_time: float = 2.0             # s to stand in HOLD before trying again
    avoid_timeout: float = 25.0
    stall_time: float = 6.0            # s of turning on the spot without finding a way -> hold
    robot_half_track: float = 0.125
    steer: SteerConfig = field(default_factory=SteerConfig)
    decision: D.DecisionConfig = field(default_factory=D.DecisionConfig)


@dataclass
class Inputs:
    t: float
    yaw: float                               # rad, IMU heading (any fixed reference)
    level: bool = True                       # body tilt small, walking controller holding the legs
    obstacles: list = field(default_factory=list)    # scan.Obstacle
    points: np.ndarray = None                # (k, 2) scan points, body frame
    seen_sector: tuple = (-1.7, 1.7)         # rad, bearings the camera has just looked at
    scan_range: float = 2.5
    box: object = None                       # gauge.BoxMeasurement from this frame (when asked for)
    intent: tuple = None                     # (vx, vy, wz) the driver is commanding, or None
    ball_tracked: bool = False
    dodge_event: bool = False                # the dodge reflex just moved the robot


@dataclass
class Outputs:
    cmd: tuple = None                        # (vx, vy, wz) to publish, or None = say nothing
    want_gauge: bool = False                 # run the fine box gauge on the next frames
    plan_box: object = None                  # crawl_planner.BoxSpec to plan and cross (once, on entering PLAN)
    state: str = CRUISE
    event: str = ""                          # something worth publishing/logging


class TerrainFsm:
    def __init__(self, cfg: FsmConfig | None = None):
        self.cfg = cfg or FsmConfig()
        self.state = CRUISE
        self.map = LocalMap(self.cfg.steer)
        self.gauge = BoxGauge(self.cfg.decision.min_frames)
        self.goal_yaw = 0.0
        self.side = 0
        self.t_state = 0.0
        self.t_last = None
        self.yaw_last = None
        self.last_cmd = (0.0, 0.0, 0.0)
        self.align_tries = 0
        self.pulse = None                    # (cmd, until)
        self.estimate = None
        self.decision = None
        self.intent_t = -1e9
        self.intent = (0.0, 0.0, 0.0)
        self._plan_sent = False
        self.hold_reason = ""
        self.ignore_box_until = -1e9         # a box judged too low to matter is left alone until then

    # ------------------------------------------------------------------ helpers
    def _go(self, state, t):
        self.state = state
        self.t_state = t

    def _goal_heading(self, yaw):
        return (self.goal_yaw - yaw + math.pi) % (2 * math.pi) - math.pi

    def _blocking(self, obstacles, heading=0.0):
        """Nearest obstacle with any point inside the lane along ``heading``."""
        best = None
        c, s = math.cos(heading), math.sin(heading)
        for ob in obstacles:
            if ob.points is None or not len(ob.points):
                continue
            along = ob.points[:, 0] * c + ob.points[:, 1] * s
            across = -ob.points[:, 0] * s + ob.points[:, 1] * c
            inside = (along > 0.0) & (np.abs(across) < self.cfg.corridor_half)
            if inside.any():
                d = float(along[inside].min())
                if best is None or d < best[0]:
                    best = (d, ob)
        return best

    def plan_result(self, ok: bool, why: str = "") -> None:
        """The node's answer to plan_box: crossing started (ok) or no plan (then go around)."""
        if self.state != PLAN:
            return
        if ok:
            self.state = STEP_OVER
        else:
            self.decision = D.Decision(D.GO_AROUND, [why or "no crossing plan"])
            self.state = AVOID
            self.side = 0

    def crossing_result(self, ok: bool, why: str = "") -> None:
        if self.state != STEP_OVER:
            return
        if ok:
            self.map.clear()
            self.state = CRUISE
        else:
            self.hold_reason = f"crossing failed: {why}"
            self.state = HOLD

    # ------------------------------------------------------------------ the tick
    def tick(self, inp: Inputs) -> Outputs:
        cfg = self.cfg
        t = inp.t
        dt = 0.0 if self.t_last is None else max(0.0, min(0.5, t - self.t_last))
        dyaw = 0.0 if self.yaw_last is None else (inp.yaw - self.yaw_last + math.pi) % (2 * math.pi) - math.pi
        self.t_last, self.yaw_last = t, inp.yaw
        out = Outputs(state=self.state)

        # where the robot has moved since the last tick (what it was told to do, and the IMU's yaw)
        if self.state in (CRUISE,):
            vx, vy = self.intent[0], self.intent[1]
        else:
            vx, vy = self.last_cmd[0], self.last_cmd[1]
        k = cfg.steer.speed_scale
        self.map.move(vx * k * dt, vy * k * dt, dyaw)
        if inp.points is not None:
            self.map.add_scan(t, inp.points, inp.seen_sector[0], inp.seen_sector[1], inp.scan_range)

        if inp.intent is not None:
            self.intent, self.intent_t = inp.intent, t
        driving = (t - self.intent_t) < cfg.intent_timeout and (
            abs(self.intent[0]) > cfg.intent_min_speed or abs(self.intent[1]) > cfg.intent_min_speed)

        if inp.dodge_event and self.state in (AVOID, APPROACH, GAUGE, ALIGN):
            # a dodge moved the robot: everything measured so far is stale
            self.gauge.reset()
            self.map.clear()
            self.side = 0
            self._go(CRUISE, t)
            out.event = "dodge: measurements dropped, re-assessing"

        if self.state == STEP_OVER:
            out.state = STEP_OVER
            return out                       # the crossing owns the legs; say nothing
        if self.state == PLAN:
            out.cmd = (0.0, 0.0, 0.0)
            if not self._plan_sent:
                self._plan_sent = True
                from .crawl_planner import BoxSpec
                e = self.estimate
                out.plan_box = BoxSpec(height=self.decision.height_used, depth=e.depth, near_x=e.near_x, skew=e.skew)
                out.event = (f"planning a crossing: box {e.height * 1000:.0f} x {e.depth * 1000:.0f} mm, "
                             f"{e.near_x * 1000:.0f} mm ahead, {math.degrees(e.skew):+.1f} deg")
            out.state = PLAN
            self.last_cmd = out.cmd
            return out

        if self.state == HOLD:
            out.cmd = (0.0, 0.0, 0.0)
            if t - self.t_state > cfg.hold_time and not self.hold_reason.startswith("crossing failed"):
                self._go(CRUISE, t)
            out.state = self.state
            self.last_cmd = out.cmd
            return out

        if not inp.level and self.state != CRUISE:
            self.hold_reason = "robot not level"
            self._go(HOLD, t)
            out.cmd, out.state, out.event = (0.0, 0.0, 0.0), HOLD, "holding: robot not level"
            self.last_cmd = out.cmd
            return out

        if self.state == CRUISE:
            self.last_cmd = (0.0, 0.0, 0.0)
            if not driving or not inp.level:
                return out
            # the driver's direction of travel, as a heading in the body frame
            heading = math.atan2(self.intent[1], self.intent[0]) if abs(self.intent[0]) + abs(self.intent[1]) > 1e-6 else 0.0
            obstacles = inp.obstacles
            if t < self.ignore_box_until:
                obstacles = [o for o in obstacles if not o.is_box]
            hit = self._blocking(obstacles, heading)
            if hit is None:
                return out
            dist, ob = hit
            if ob.is_box and dist < cfg.box_engage_range:
                self.goal_yaw = inp.yaw + heading
                self.gauge.reset()
                self.align_tries = 0
                self._go(APPROACH, t)
                out.event = f"box ahead at {dist:.2f} m: approaching to measure it"
            elif not ob.is_box and dist < cfg.engage_range:
                self.goal_yaw = inp.yaw + heading
                self.side = 0
                self.decision = D.Decision(D.GO_AROUND, ["tall" if ob.tall else "not the step-over box"])
                self._go(AVOID, t)
                out.event = f"obstacle ahead at {dist:.2f} m ({'tall' if ob.tall else 'low, unknown'}): going around"
            if self.state == CRUISE:
                return out

        if self.state in (AVOID, APPROACH) and not driving:
            # nobody is asking the robot to go anywhere any more: stop meddling
            self._go(CRUISE, t)
            out.cmd, out.state, out.event = (0.0, 0.0, 0.0), CRUISE, "driver stopped: standing down"
            self.last_cmd = out.cmd
            return out

        if self.state == APPROACH:
            hit = self._blocking([o for o in inp.obstacles if o.is_box], self._goal_heading(inp.yaw))
            if hit is None:
                if t - self.t_state > 2.0:
                    self._go(CRUISE, t)
                    out.event = "box lost from view"
                out.cmd = (0.0, 0.0, 0.0)
            else:
                dist, ob = hit
                near = dist                   # along the lane, from the body centre
                if near <= cfg.gauge_distance + cfg.gauge_tolerance:
                    self._go(GAUGE, t)
                    out.cmd = (0.0, 0.0, 0.0)
                else:
                    wz = float(np.clip(1.5 * self._goal_heading(inp.yaw), -0.6, 0.6))
                    out.cmd = (cfg.approach_speed, 0.0, wz)

        elif self.state == GAUGE:
            out.cmd = (0.0, 0.0, 0.0)
            if t - self.t_state > cfg.settle_time:
                out.want_gauge = True
                if inp.box is not None or t - self.t_state > cfg.settle_time + 0.3:
                    self.gauge.add(inp.box)
                if self.gauge.ready or t - self.t_state > cfg.gauge_timeout:
                    est = self.gauge.estimate() if self.gauge.ready else None
                    self.estimate = est
                    out.event = self._after_gauge(est, inp, t)

        elif self.state == ALIGN:
            cmd, until = self.pulse
            if t < until:
                out.cmd = cmd
            else:
                out.cmd = (0.0, 0.0, 0.0)
                self.gauge.reset()
                self._go(GAUGE, t)

        elif self.state == AVOID:
            goal = self._goal_heading(inp.yaw)
            res = steer(self.map.points, goal, cfg.steer, self.side)
            if res.stuck:
                self.hold_reason = "blocked on both sides"
                self._go(HOLD, t)
                out.cmd, out.event = (0.0, 0.0, 0.0), "holding: no way round (blocked on both sides)"
            else:
                if res.blocked:
                    self.side = res.side
                out.cmd = (res.vx, 0.0, res.wz)
                if res.vx > 0.0:
                    self._t_progress = t
                clear_ahead = path_clear(self.map.points, goal, cfg.steer.clearance, cfg.steer.look_ahead)
                if clear_ahead and abs(goal) < cfg.steer.done_heading and not res.blocked:
                    self._go(CRUISE, t)
                    self.side = 0
                    out.cmd = None
                    out.event = "obstacle passed: back on the original heading"
                elif t - max(self.t_state, getattr(self, "_t_progress", -1e9)) > cfg.stall_time:
                    self.hold_reason = "no way round found"
                    self._go(HOLD, t)
                    out.cmd, out.event = (0.0, 0.0, 0.0), "holding: turned on the spot without finding a way round"
                elif t - self.t_state > cfg.avoid_timeout:
                    self.hold_reason = "avoidance took too long"
                    self._go(HOLD, t)
                    out.cmd, out.event = (0.0, 0.0, 0.0), "holding: could not get round in time"

        out.state = self.state
        if out.cmd is not None:
            self.last_cmd = out.cmd
        return out

    def _after_gauge(self, est, inp, t) -> str:
        cfg = self.cfg
        if est is None:
            self.decision = D.decide(None, cfg.decision)
            self.side = 0
            self._go(AVOID, t)
            return f"decision: {self.decision}"
        # first get square to it and centred, then close enough, then decide
        skew_deg = math.degrees(est.skew)
        if self.align_tries < cfg.align_max_tries:
            cmd = None
            if abs(skew_deg) > cfg.align_skew_deg:
                # skew > 0: the left end is farther away -> turn left (positive yaw) to face it
                dur = max(cfg.align_min_pulse, 0.8 * abs(est.skew) / cfg.align_yaw_rate)
                cmd = (0.0, 0.0, math.copysign(cfg.align_yaw_rate, est.skew))
            elif est.ends_seen and abs(est.y_centre) > cfg.align_centre:
                dur = max(cfg.align_min_pulse, 0.8 * abs(est.y_centre) / cfg.align_side_speed)
                cmd = (0.0, math.copysign(cfg.align_side_speed, est.y_centre), 0.0)
            elif est.near_x > cfg.gauge_distance + cfg.gauge_tolerance:
                dur = max(cfg.align_min_pulse, 0.8 * (est.near_x - cfg.gauge_distance) / cfg.approach_speed)
                cmd = (cfg.approach_speed, 0.0, 0.0)
            if cmd is not None:
                self.align_tries += 1
                self.pulse = (cmd, t + dur)
                self._go(ALIGN, t)
                return (f"aligning ({self.align_tries}): skew {skew_deg:+.1f} deg, off-centre {est.y_centre * 1000:+.0f} mm, "
                        f"{est.near_x * 1000:.0f} mm away")
        self.decision = D.decide(est, cfg.decision, landing_clear=self._landing_clear(est), robot_ready=inp.level,
                                 ball_tracked=inp.ball_tracked, robot_half_track=cfg.robot_half_track)
        if self.decision.action == D.STEP_OVER:
            self._plan_sent = False
            self._go(PLAN, t)
        elif self.decision.action == D.IGNORE:
            # walk over it with the normal gait; do not stop for it again while passing it
            speed = max(0.15, math.hypot(self.intent[0], self.intent[1]))
            depth = est.depth if math.isfinite(est.depth) else 0.1
            self.ignore_box_until = t + 1.5 * (est.near_x + depth + 0.4) / speed
            self._go(CRUISE, t)
        else:
            self.side = 0
            self._go(AVOID, t)
        return (f"decision: {self.decision} [h {est.height * 1000:.0f} +- {est.height_sigma * 1000:.1f} mm, "
                f"d {est.depth * 1000:.0f} mm, {est.near_x * 1000:.0f} mm away, skew {skew_deg:+.1f} deg]")

    def _landing_clear(self, est) -> bool:
        """The strip just beyond the box is floor (as far as the camera sees over the box), and
        no remembered obstacle point lies in it."""
        landing = getattr(est, "landing_floor", float("nan"))
        if math.isfinite(landing) and landing < self.cfg.decision.landing_min_floor:
            return False
        if not len(self.map.points) or not math.isfinite(est.depth):
            return True
        p = self.map.points
        x0 = est.near_x + est.depth + 0.05
        beyond = (p[:, 0] > x0) & (p[:, 0] < x0 + self.cfg.decision.landing_clear) & (np.abs(p[:, 1]) < 0.2)
        return not bool(beyond.any())
