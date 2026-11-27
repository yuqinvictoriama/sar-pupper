"""Taking the legs from the walking controller, playing a plan, and giving them back.

Pure Python, so the simulator runs the same sequence as the robot. robot/stepover_node.py
calls ``update`` at its streaming rate and publishes what comes back on the three forward
command controllers; it tells this class when the controller switch has gone through, and
about an e-stop or stale sensor data.

Stages
  PRE     before the switch is confirmed. Position = the measured angles, kp = 0, damping
          only. Anything published before the controllers are active is dropped by them;
          what reaches them in the first milliseconds after activation therefore cannot
          jerk a leg (there is no stiffness in it), and the legs only see damping for the
          few tens of milliseconds the switch takes.
  RAMP    switch confirmed: hold the angles measured at that moment and bring kp up over
          ``ramp_time``.
  PLAY    the trajectory player has the legs (it blends from the held pose into the plan).
  FINAL   plan finished: hold the final (default) stance until the walking controller is back.
  SOFT    the player aborted: soft joints, the robot settles onto its belly or the box.
  LIMP    e-stop or stale data: kp = 0, light damping - the same as the walking controllers'
          e-stop. Stays there until the node releases the controllers.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import player as P
from .kinematics import LEGS, leg_limits

PRE, RAMP, PLAY, FINAL, SOFT, LIMP = "pre", "ramp", "play", "final", "soft", "limp"


@dataclass
class HandoverConfig:
    pre_kd: float = 0.6            # damping while the switch is in progress
    ramp_time: float = 0.06        # s, kp 0 -> full after the switch is confirmed
    switch_timeout: float = 2.0    # s to wait in PRE for the switch before giving up
    estop_kd: float = 0.1          # damping when limp (the walking controllers' estop_kd)
    kp_max: float = 7.5            # never command more stiffness than the walking controller's own
    kd_max: float = 0.8


@dataclass
class LegCommand:
    q: np.ndarray        # 12 position targets, inside the hardware limits
    kp: float
    kd: float
    stage: str
    event: str = ""
    phase: str = ""
    progress: float = 0.0


_LIMITS = np.concatenate([leg_limits(leg) for leg in LEGS], axis=0)     # (12, 2)


def clamp_to_limits(q) -> np.ndarray:
    return np.clip(np.asarray(q, dtype=float), _LIMITS[:, 0], _LIMITS[:, 1])


class Handover:
    def __init__(self, player: P.TrajectoryPlayer, cfg: HandoverConfig | None = None):
        self.player = player
        self.cfg = cfg or HandoverConfig()
        self.stage = PRE
        self.result = ""            # "done" or the reason it ended otherwise
        self._t0 = None
        self._t_ramp = None
        self._q_hold = None
        self._q_last = None

    @property
    def holding_legs(self) -> bool:
        """The forward controllers must stay active (someone has to hold the robot up)."""
        return self.stage in (RAMP, PLAY, FINAL, SOFT)

    @property
    def finished(self) -> bool:
        return self.stage in (FINAL, LIMP) or (self.stage == SOFT and self.player.state == P.STOPPED)

    def _limp(self, q_meas, why):
        if self.stage != LIMP:
            self.stage = LIMP
            self.result = self.result if self.result not in ("", "done") else why
            if self.player.state in (P.BLEND, P.RUN, P.REVERSE, P.DONE):
                self.player.state = P.STOPPED
                self.player.result = why
            return f"limp: {why}"
        return ""

    def update(self, t: float, q_meas, tilt_rad: float, switched: bool, estop: bool = False,
               stale: bool = False) -> LegCommand:
        cfg = self.cfg
        q_meas = np.asarray(q_meas, dtype=float)
        if self._t0 is None:
            self._t0 = t
        event = ""
        if estop:
            event = self._limp(q_meas, "e-stop")
        elif stale and self.stage != LIMP:
            event = self._limp(q_meas, "joint or IMU data stopped arriving")

        if self.stage == LIMP:
            return LegCommand(clamp_to_limits(q_meas), 0.0, cfg.estop_kd, LIMP, event)

        if self.stage == PRE:
            if switched:
                self.stage = RAMP
                self._t_ramp = t
                self._q_hold = q_meas.copy()
            elif t - self._t0 > cfg.switch_timeout:
                event = self._limp(q_meas, "controller switch did not complete")
                return LegCommand(clamp_to_limits(q_meas), 0.0, cfg.estop_kd, LIMP, event)
            else:
                return LegCommand(clamp_to_limits(q_meas), 0.0, cfg.pre_kd, PRE)

        if self.stage == RAMP:
            a = min(1.0, (t - self._t_ramp) / max(1e-6, cfg.ramp_time))
            kp = min(cfg.kp_max, self.player.cfg.kp) * a
            kd = cfg.pre_kd + (self.player.cfg.kd - cfg.pre_kd) * a
            if a >= 1.0:
                self.stage = PLAY
                self.player.start(t, self._q_hold)
                event = "legs taken over"
            self._q_last = self._q_hold
            return LegCommand(clamp_to_limits(self._q_hold), kp, min(kd, cfg.kd_max), self.stage if a < 1.0 else RAMP, event)

        cmd = self.player.update(t, q_meas, tilt_rad)
        self._q_last = cmd.q
        if self.player.state == P.DONE:
            if self.stage != FINAL:
                self.stage, self.result = FINAL, "done"
        elif self.player.state in (P.SETTLE, P.STOPPED):
            if self.stage != SOFT:
                self.stage, self.result = SOFT, self.player.result or "aborted"
        return LegCommand(clamp_to_limits(cmd.q), min(cmd.kp, cfg.kp_max), min(cmd.kd, cfg.kd_max), self.stage,
                          cmd.event, cmd.phase, cmd.progress)
