"""Plays a crawl plan and watches it. Pure Python: the same class drives the robot (through
robot/stepover_node.py) and the simulator (sim/crossing_sim.py), so what is tested in
simulation is the code that runs on the robot.

Each call to ``update`` takes the time, the measured joint angles and the body tilt, and
returns the 12 joint targets plus the gains to send. Besides following the plan it

  * blends from wherever the legs are to the plan's first pose;
  * stops a descending foot when it meets the ground (or the box top) early, instead of
    pushing on down to a fixed height;
  * backs a swing out along the path it came if the swinging leg is blocked;
  * and, on a tilt or tracking fault, stops and softens the joints so the robot settles
    instead of fighting.

The forward command controllers this drives have no tilt e-stop of their own; this is it.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .crawl_planner import CrawlPlan

# states
IDLE = "idle"
BLEND = "blend"          # easing from the measured pose to the plan's first pose
RUN = "run"
REVERSE = "reverse"      # backing a blocked swing out
SETTLE = "settle"        # fault: softening the joints
DONE = "done"            # plan finished; holding the final (default) stance
STOPPED = "stopped"      # fault handled; joints soft, robot resting


@dataclass
class PlayerConfig:
    kp: float = 7.5
    kd: float = 0.25
    blend_time: float = 0.6            # s, measured pose -> first pose of the plan
    tilt_limit_deg: float = 20.0       # body tilt that aborts the crossing
    stance_error_limit: float = 0.15   # rad, a planted leg's joint off its expected angle
    stance_error_time: float = 0.20    # s it has to persist
    swing_error_limit: float = 0.12    # rad, the swinging leg off its target (it hit something)
    swing_error_time: float = 0.10
    guard_error: float = 0.035         # rad, descending foot: error that means "touched down"
    guard_time: float = 0.03           # s it has to persist
    settle_time: float = 1.5           # s to soften the joints after a fault
    settle_kp: float = 1.5
    settle_kd: float = 0.6
    hold_done_kp: float = 7.5
    # The joints answer a command some milliseconds late (the course's robot model: 15 ms).
    # Compared with where the plan is now, a moving joint then looks off by speed x delay,
    # and in simulation 25 ms of delay was enough for a free swing to read as blocked. So
    # each joint is checked against the closest point of the plan over this much recent
    # time: a joint that is merely late matches somewhere along it, a blocked one does not.
    lag_window: float = 0.06           # s


@dataclass
class Command:
    q: np.ndarray           # 12 joint position targets
    kp: float
    kd: float
    state: str
    phase: str = ""
    swing_leg: int = -1
    progress: float = 0.0   # 0..1 through the plan
    event: str = ""         # non-empty on the update where something happened
    info: dict = field(default_factory=dict)


class TrajectoryPlayer:
    def __init__(self, plan: CrawlPlan, cfg: PlayerConfig | None = None):
        if not plan.ok:
            raise ValueError(f"cannot play a failed plan: {plan.reason}")
        self.plan = plan
        self.cfg = cfg or PlayerConfig()
        self.state = IDLE
        self.result = ""                 # why it ended: "done", "tilt", "stance error", "blocked"
        self._t_last = None
        self._tp = 0.0                   # time along the plan
        self._blend_t0 = None
        self._q_start = None
        self._last_cmd = plan.q_cmd[0].copy()
        self._offset = np.zeros(12)      # per-joint offset from an early touchdown, fading out
        self._offset_decay = 0.0         # rad/s per joint
        self._stance_bad = 0.0
        self._swing_bad = 0.0
        self._guard_bad = 0.0
        self._guard_done_phase = -1
        self._settle_t0 = None
        self._reverse_to = 0.0
        self.max_tilt_deg = 0.0
        self.events: list[tuple[float, str]] = []

    # ------------------------------------------------------------------ helpers
    def _sample(self, tp):
        """Plan values at plan time tp: (q_cmd, q_des, index)."""
        t = self.plan.t
        tp = float(np.clip(tp, t[0], t[-1]))
        i = int(np.searchsorted(t, tp, side="left"))
        if i == 0:
            return self.plan.q_cmd[0].copy(), self.plan.q_des[0].copy(), 0
        a = (tp - t[i - 1]) / (t[i] - t[i - 1])
        q_cmd = self.plan.q_cmd[i - 1] * (1 - a) + self.plan.q_cmd[i] * a
        q_des = self.plan.q_des[i - 1] * (1 - a) + self.plan.q_des[i] * a
        return q_cmd, q_des, i

    def _lagged_error(self, q_meas, q_des_now):
        """Per joint, the smallest |planned - measured| over the last lag_window of the plan."""
        err = np.abs(q_des_now - q_meas)
        w = self.cfg.lag_window
        if w <= 0.0:
            return err
        for s in np.linspace(w / 4, w, 4):
            err = np.minimum(err, np.abs(self._sample(self._tp - s)[1] - q_meas))
        return err

    def _event(self, t, text):
        self.events.append((t, text))
        return text

    def _fault(self, t, why):
        self.state = SETTLE
        self.result = why
        self._settle_t0 = t
        return self._event(t, f"abort: {why}")

    # ------------------------------------------------------------------ API
    def start(self, t: float, q_meas) -> None:
        self.state = BLEND
        self._blend_t0 = t
        self._t_last = t
        self._q_start = np.asarray(q_meas, dtype=float).copy()
        self._tp = float(self.plan.t[0])

    def stop(self, t: float, why: str = "stopped") -> None:
        """External stop (e-stop, operator): soften and rest."""
        if self.state in (BLEND, RUN, REVERSE, DONE):
            self._fault(t, why)

    @property
    def finished(self) -> bool:
        return self.state in (DONE, STOPPED)

    def update(self, t: float, q_meas, tilt_rad: float) -> Command:
        cfg = self.cfg
        plan = self.plan
        q_meas = np.asarray(q_meas, dtype=float)
        dt = 0.0 if self._t_last is None else max(0.0, min(0.1, t - self._t_last))
        self._t_last = t
        tilt_deg = float(np.degrees(tilt_rad))
        self.max_tilt_deg = max(self.max_tilt_deg, tilt_deg)
        event = ""

        if self.state == IDLE:
            return Command(q_meas.copy(), 0.0, cfg.kd, IDLE)

        if self.state in (BLEND, RUN, REVERSE) and tilt_deg > cfg.tilt_limit_deg:
            event = self._fault(t, f"tilt {tilt_deg:.0f} deg")

        if self.state == BLEND:
            a = min(1.0, (t - self._blend_t0) / cfg.blend_time)
            s = a * a * (3 - 2 * a)
            q = self._q_start * (1 - s) + plan.q_cmd[0] * s
            self._last_cmd = q
            if a >= 1.0:
                self.state = RUN
                event = self._event(t, "plan started")
            return Command(q, cfg.kp, cfg.kd, self.state, "blend", -1, 0.0, event)

        if self.state in (RUN, REVERSE):
            if self.state == RUN:
                self._tp += dt
            else:
                self._tp -= dt
                if self._tp <= self._reverse_to:
                    self._tp = self._reverse_to
                    event = self._fault(t, "blocked: swing foot met something; foot put back")
            q_cmd, q_des, i = self._sample(self._tp)
            swing = int(plan.swing_leg[i])
            guard = int(plan.guard_leg[i])
            phase = plan.phase_names[int(plan.phase[i])]
            err = self._lagged_error(q_meas, q_des) if self.state == RUN else np.abs(q_des - q_meas)

            if self.state == RUN:
                # planted legs: should sit at their desired angles (the command already carries
                # the load compensation), so a large error means a slip, a collision or a fall
                stance_mask = np.ones(12, dtype=bool)
                if swing >= 0:
                    stance_mask[3 * swing:3 * swing + 3] = False
                if np.abs(err[stance_mask]).max() > cfg.stance_error_limit:
                    self._stance_bad += dt
                    if self._stance_bad > cfg.stance_error_time:
                        event = self._fault(t, f"stance joint {np.abs(err[stance_mask]).max():.2f} rad off in {phase}")
                else:
                    self._stance_bad = 0.0

                in_air = swing >= 0 and plan.weights[i, swing] < 0.5
                if in_air and self.state == RUN:
                    e_sw = np.abs(err[3 * swing:3 * swing + 3]).max()
                    if guard == swing and self._guard_done_phase != int(plan.phase[i]):
                        # final descent: an error here is the foot arriving early
                        if e_sw > cfg.guard_error:
                            self._guard_bad += dt
                            if self._guard_bad > cfg.guard_time:
                                # hold the foot where it is: keep this command as an offset on
                                # top of the rest of the plan, fading over the next 0.3 s
                                ph = int(plan.phase[i])
                                j_end = int(np.searchsorted(plan.phase, ph, side="right")) - 1
                                target_end = plan.q_cmd[j_end]
                                sl = slice(3 * swing, 3 * swing + 3)
                                self._offset[sl] = (q_cmd + self._offset)[sl] - target_end[sl]
                                self._offset_decay = float(np.abs(self._offset[sl]).max()) / 0.3
                                self._tp = float(plan.t[j_end])
                                self._guard_done_phase = ph
                                self._guard_bad = 0.0
                                event = self._event(t, f"touchdown early in {phase}")
                                q_cmd, q_des, i = self._sample(self._tp)
                        else:
                            self._guard_bad = 0.0
                    elif guard != swing:
                        if e_sw > cfg.swing_error_limit:
                            self._swing_bad += dt
                            if self._swing_bad > cfg.swing_error_time:
                                # back out along the path to the start of this swing
                                first = i
                                while first > 0 and plan.swing_leg[first - 1] == swing and plan.weights[first - 1, swing] < 0.5:
                                    first -= 1
                                self._reverse_to = float(plan.t[first])
                                self.state = REVERSE
                                self._swing_bad = 0.0
                                event = self._event(t, f"swing blocked in {phase}; backing out")
                        else:
                            self._swing_bad = 0.0

            # fade any touchdown offset
            if self._offset_decay > 0.0:
                step = self._offset_decay * dt
                self._offset = np.sign(self._offset) * np.maximum(np.abs(self._offset) - step, 0.0)
                if not self._offset.any():
                    self._offset_decay = 0.0
            q = q_cmd + self._offset
            self._last_cmd = q
            progress = (self._tp - plan.t[0]) / max(1e-9, plan.t[-1] - plan.t[0])
            if self.state == RUN and self._tp >= plan.t[-1]:
                self.state = DONE
                self.result = "done"
                event = self._event(t, "plan finished")
            if self.state in (RUN, REVERSE, DONE):
                return Command(q, cfg.kp, cfg.kd, self.state, phase, swing, float(progress), event)

        if self.state == SETTLE:
            a = min(1.0, (t - self._settle_t0) / cfg.settle_time)
            kp = cfg.kp + (cfg.settle_kp - cfg.kp) * a
            kd = cfg.kd + (cfg.settle_kd - cfg.kd) * a
            if a >= 1.0:
                self.state = STOPPED
                event = event or self._event(t, "settled")
            return Command(self._last_cmd.copy(), kp, kd, self.state, "settle", -1, 0.0, event)

        if self.state == DONE:
            return Command(self._last_cmd.copy(), cfg.hold_done_kp, cfg.kd, DONE, "done", -1, 1.0, event)

        return Command(self._last_cmd.copy(), cfg.settle_kp, cfg.settle_kd, STOPPED, "stopped", -1, 0.0, event)
