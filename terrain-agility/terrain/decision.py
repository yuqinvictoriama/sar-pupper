"""Step over, or walk around? Thresholds with a guard band, taken while standing still.

The answer is "step over" only if every check passes; anything doubtful is walked around.
The height limit is not one number: it depends on the box's depth, and comes from the
planning envelope that sim/build_templates.py writes (which boxes the crawl planner can
actually cross), tightened by what the physics simulation showed.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

STEP_OVER = "step_over"
GO_AROUND = "go_around"
IGNORE = "ignore"

ENVELOPE_FILE = Path(__file__).resolve().parent.parent / "config" / "crawl_templates.json"
# Used when the envelope file is missing: depth (mm) -> tallest crossable box (m).
DEFAULT_ENVELOPE = {76: 0.030, 82: 0.030, 89: 0.030, 95: 0.030, 102: 0.030}


def load_envelope(path=None) -> dict:
    p = Path(path) if path else ENVELOPE_FILE
    if p.exists():
        raw = json.loads(p.read_text()).get("planning_envelope_max_height_by_depth_mm", {})
        env = {int(float(k)): float(v) for k, v in raw.items() if float(v) > 0}
        if env:
            return env
    return dict(DEFAULT_ENVELOPE)


@dataclass
class DecisionConfig:
    prefer_step_over: bool = True      # a steppable box is stepped over even if going around is quicker
    ignore_below: float = 0.010        # m: lower than this is walked over with the normal gait
    height_cap: float = 0.030          # m: never step over anything taller, whatever the envelope says
    height_guard: float = 0.003        # m the box may be taller than it reads; must still be inside the envelope
    sigma_k: float = 2.0               # the height must clear the limit by this many sigmas
    max_height_sigma: float = 0.004    # m: a noisier reading than this is not trusted
    min_frames: int = 10
    min_width: float = 0.40            # m
    side_margin: float = 0.08          # m of box beyond each side of the robot's feet line
    max_skew_deg: float = 5.0          # after aligning
    near_min: float = 0.29             # m, body centre to the near face: the range of starting
    near_max: float = 0.43             #    distances the stored footstep patterns were verified from
    landing_clear: float = 0.35        # m of clear floor needed beyond the box
    landing_min_floor: float = 0.85    # fraction of that strip that must look like floor
    envelope: dict = field(default_factory=load_envelope)

    def max_height(self, depth: float) -> float:
        """Tallest crossable box for this depth (m): the envelope at the two nearest tabulated
        depths, whichever is lower; zero outside the table."""
        if not self.envelope or not math.isfinite(depth):
            return 0.0
        d_mm = depth * 1000.0
        keys = sorted(self.envelope)
        if d_mm < keys[0] - 3 or d_mm > keys[-1] + 3:
            return 0.0
        lower = max([k for k in keys if k <= d_mm + 1e-6], default=keys[0])
        upper = min([k for k in keys if k >= d_mm - 1e-6], default=keys[-1])
        return min(self.envelope[lower], self.envelope[upper], self.height_cap)


@dataclass
class Decision:
    action: str
    reasons: list            # why not step over (empty when stepping over)
    height_used: float = 0.0  # m: the height to plan with (as measured)

    def __str__(self):
        return self.action if not self.reasons else f"{self.action} ({'; '.join(self.reasons)})"


def decide(est, cfg: DecisionConfig | None = None, landing_clear: bool = True, robot_ready: bool = True,
           ball_tracked: bool = False, robot_half_track: float = 0.125) -> Decision:
    """est: gauge.BoxEstimate (or None if the box could not be measured)."""
    cfg = cfg or DecisionConfig()
    if est is None:
        return Decision(GO_AROUND, ["the obstacle could not be measured (not the two-colour box, or not in clear view)"])
    if est.height + cfg.sigma_k * min(est.height_sigma, 0.01) < cfg.ignore_below:
        return Decision(IGNORE, [f"only {est.height * 1000:.0f} mm tall"])
    why = []
    if est.frames < cfg.min_frames:
        why.append(f"only {est.frames} good frames (need {cfg.min_frames})")
    if not (est.height_sigma <= cfg.max_height_sigma):
        why.append(f"height reading too noisy (sigma {est.height_sigma * 1000:.1f} mm)")
    if not math.isfinite(est.depth):
        why.append("depth not measured (top colour not seen)")
        limit = 0.0
    else:
        limit = cfg.max_height(est.depth)
        if limit <= 0.0:
            why.append(f"depth {est.depth * 1000:.0f} mm is outside what can be crossed")
    # The box may be taller than it reads: by the guard, or by the reading's own scatter if
    # that is larger. That worst case has to be inside the envelope. The crossing itself is
    # planned for the height as read: the gait keeps `clearance_top` above the box, which
    # covers an under-read, and a foot stepping onto the top feels for it (terrain/player.py).
    h_test = est.height + max(cfg.height_guard, cfg.sigma_k * (est.height_sigma if math.isfinite(est.height_sigma) else 1.0))
    if limit > 0.0 and h_test > limit + 1e-9:
        why.append(f"height {est.height * 1000:.0f} mm (up to {h_test * 1000:.0f} with its uncertainty) is over the "
                   f"{limit * 1000:.0f} mm limit for this depth")
    if est.width < cfg.min_width and est.ends_seen:
        why.append(f"only {est.width * 1000:.0f} mm wide (need {cfg.min_width * 1000:.0f})")
    need = robot_half_track + cfg.side_margin
    if est.ends_seen and (est.y_left < need or est.y_right > -need):
        why.append("the robot is not centred on the box with room to spare on both sides")
    if not (cfg.near_min <= est.near_x <= cfg.near_max):
        why.append(f"standing {est.near_x * 1000:.0f} mm from it, outside the {cfg.near_min * 1000:.0f}-"
                   f"{cfg.near_max * 1000:.0f} mm a crossing can start from")
    if abs(math.degrees(est.skew)) > cfg.max_skew_deg:
        why.append(f"{abs(math.degrees(est.skew)):.0f} deg off square")
    if not landing_clear:
        why.append("no clear floor beyond the box")
    if not robot_ready:
        why.append("robot not standing level in a walking controller")
    if ball_tracked:
        why.append("a ball is being tracked")
    if not cfg.prefer_step_over:
        why.append("stepping over is switched off (prefer_step_over)")
    if why:
        return Decision(GO_AROUND, why)
    return Decision(STEP_OVER, [], est.height)
