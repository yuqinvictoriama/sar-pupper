"""What to do about the obstacle that has just been measured: step over it, scale across it,
or walk around it. Thresholds with a guard band, taken while standing still.

Three answers, by the obstacle's shape:

    step over      a low bar, shallow fore-aft: each foot crosses it in one stride, using its
                   top once as a stepping stone ("board": the stored footstep patterns)
    scale across   a low slab, deep enough to walk on: onto it, along it, off the far side
                   ("slab": terrain/crawl_planner.plan_scale_across)
    walk around    everything else: tall, or its height unreadable, or not flat and rigid

Both crossings put feet on the obstacle's top, so both need its height and depth read to a
few millimetres, which needs a side face seen at its foot (terrain/gauge.py). Anything
doubtful is walked around, and the reasons say which check failed.

The height limit is not one number for the low bar: it depends on the bar's depth, and comes
from the planning envelope that sim/build_templates.py writes (which boxes the crawl planner
can actually cross), tightened by what the physics simulation showed.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field
from pathlib import Path

STEP_OVER = "step_over"
SCALE_ACROSS = "scale_across"
GO_AROUND = "go_around"
IGNORE = "ignore"
CROSSINGS = (STEP_OVER, SCALE_ACROSS)

BOARD, SLAB = "board", "slab"          # how it is crossed (which planner): Decision.method

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
    prefer_step_over: bool = True      # something that can be crossed is crossed, even if going around is quicker
    step_over: bool = True             # allow stepping over a low bar
    scale_across: bool = True          # allow scaling across a slab
    ignore_below: float = 0.010        # m: lower than this is walked over with the normal gait
    height_cap: float = 0.030          # m: never put a foot on anything taller, whatever the envelope says
    height_guard: float = 0.003        # m it may be taller than it reads; must still be inside the limit
    sigma_k: float = 2.0               # the height must clear the limit by this many sigmas
    max_height_sigma: float = 0.004    # m: a noisier reading than this is not trusted
    min_frames: int = 10
    min_width: float = 0.40            # m
    side_margin: float = 0.08          # m of obstacle beyond each side of the robot's feet line
    max_skew_deg: float = 5.0          # after aligning
    near_min: float = 0.29             # m, body centre to the near face: the range of starting
    near_max: float = 0.43             #    distances the crossings were verified from
    landing_clear: float = 0.35        # m of clear floor needed beyond it
    landing_min_floor: float = 0.85    # fraction of that strip that must look like floor
    slab_min_depth: float = 0.20       # m: from this depth it is a slab, walked on with all four feet
    slab_max_depth: float = 0.62       # m: the deepest slab verified
    min_depth_ratio: float = 2.6       # depth must be this many times the height: a round bar read as a box
    #                                    comes out about 2 to 2.4, and a round bar is no foothold
    envelope: dict = field(default_factory=load_envelope)

    def max_height(self, depth: float) -> float:
        """Tallest low bar that can be stepped over at this depth (m): the envelope at the two
        nearest tabulated depths, whichever is lower; zero outside the table."""
        if not self.envelope or not math.isfinite(depth):
            return 0.0
        d_mm = depth * 1000.0
        keys = sorted(self.envelope)
        if d_mm < keys[0] - 3 or d_mm > keys[-1] + 3:
            return 0.0
        lower = max([k for k in keys if k <= d_mm + 1e-6], default=keys[0])
        upper = min([k for k in keys if k >= d_mm - 1e-6], default=keys[-1])
        return min(self.envelope[lower], self.envelope[upper], self.height_cap)

    def crossing_for(self, depth: float):
        """(action, method, height limit) for an obstacle this deep, or None if no crossing
        is built for that depth."""
        if not math.isfinite(depth):
            return None
        if self.slab_min_depth - 0.003 <= depth <= self.slab_max_depth + 0.003:
            return SCALE_ACROSS, SLAB, self.height_cap
        limit = self.max_height(depth)
        if limit > 0.0:
            return STEP_OVER, BOARD, limit
        return None


@dataclass
class Decision:
    action: str
    reasons: list            # why it is not crossed (empty when it is)
    height_used: float = 0.0  # m: the height to plan with (as measured)
    method: str = ""         # BOARD or SLAB when crossing

    def __str__(self):
        return self.action if not self.reasons else f"{self.action} ({'; '.join(self.reasons)})"


def decide(est, cfg: DecisionConfig | None = None, landing_clear: bool = True, robot_ready: bool = True,
           ball_tracked: bool = False, robot_half_track: float = 0.125) -> Decision:
    """est: gauge.BoxEstimate (or None if nothing could be measured)."""
    cfg = cfg or DecisionConfig()
    if est is None:
        return Decision(GO_AROUND, ["the obstacle could not be measured (its foot is not in clear view)"])
    known = math.isfinite(est.height)
    upper = getattr(est, "height_upper", float("inf"))
    if known and est.height + cfg.sigma_k * min(est.height_sigma, 0.01) < cfg.ignore_below:
        return Decision(IGNORE, [f"only {est.height * 1000:.0f} mm tall"])
    if not known and upper + cfg.height_guard < cfg.ignore_below:
        return Decision(IGNORE, [f"no taller than {upper * 1000:.0f} mm"])
    if not getattr(est, "far_seen", True):
        return Decision(GO_AROUND, ["taller than the camera: its far edge is not in view"])

    why = []
    crossing = None
    if not known:
        why.append("its height could not be read: no side face seen at its foot (an edge the colour of the floor "
                   f"or of its top, or a round shape); it is at most {upper * 1000:.0f} mm tall" if math.isfinite(upper)
                   else "its height could not be read")
    elif not math.isfinite(est.depth):
        why.append("its depth could not be read (far edge not seen)")
    else:
        crossing = cfg.crossing_for(est.depth)
        if crossing is None:
            lo, hi = min(cfg.envelope, default=0), max(cfg.envelope, default=0)
            why.append(f"depth {est.depth * 1000:.0f} mm is outside what can be crossed ({lo}-{hi} mm stepped over, "
                       f"{cfg.slab_min_depth * 1000:.0f}-{cfg.slab_max_depth * 1000:.0f} mm scaled across)")
    if known and est.frames < cfg.min_frames:
        why.append(f"only {est.frames} good frames (need {cfg.min_frames})")
    if known and not (est.height_sigma <= cfg.max_height_sigma):
        why.append(f"height reading too noisy (sigma {est.height_sigma * 1000:.1f} mm)")
    if crossing is not None:
        action, method, limit = crossing
        # It may be taller than it reads: by the guard, or by the reading's own scatter if
        # that is larger. That worst case has to be inside the limit. The crossing itself is
        # planned for the height as read: the gait keeps `clearance_top` above the top, which
        # covers an under-read, and a foot stepping onto it feels for it (terrain/player.py).
        h_test = est.height + max(cfg.height_guard, cfg.sigma_k * (est.height_sigma if math.isfinite(est.height_sigma) else 1.0))
        if h_test > limit + 1e-9:
            why.append(f"height {est.height * 1000:.0f} mm (up to {h_test * 1000:.0f} with its uncertainty) is over the "
                       f"{limit * 1000:.0f} mm limit for this depth")
        if est.depth < cfg.min_depth_ratio * est.height:
            why.append(f"{est.height * 1000:.0f} mm tall but only {est.depth * 1000:.0f} mm deep: too narrow a top to trust as a foothold")
        if not (cfg.step_over if action == STEP_OVER else cfg.scale_across):
            why.append(f"{action.replace('_', ' ')} is switched off")
    if est.width < cfg.min_width and est.ends_seen:
        why.append(f"only {est.width * 1000:.0f} mm wide (need {cfg.min_width * 1000:.0f})")
    need = robot_half_track + cfg.side_margin
    if est.ends_seen and (est.y_left < need or est.y_right > -need):
        why.append("the robot is not centred on it with room to spare on both sides")
    if not (cfg.near_min <= est.near_x <= cfg.near_max):
        why.append(f"standing {est.near_x * 1000:.0f} mm from it, outside the {cfg.near_min * 1000:.0f}-"
                   f"{cfg.near_max * 1000:.0f} mm a crossing can start from")
    if abs(math.degrees(est.skew)) > cfg.max_skew_deg:
        why.append(f"{abs(math.degrees(est.skew)):.0f} deg off square")
    if not landing_clear:
        why.append("no clear floor beyond it")
    if not robot_ready:
        why.append("robot not standing level in a walking controller")
    if ball_tracked:
        why.append("a ball is being tracked")
    if not cfg.prefer_step_over:
        why.append("crossing is switched off (prefer_step_over)")
    if why or crossing is None:
        return Decision(GO_AROUND, why)
    return Decision(crossing[0], [], est.height, crossing[1])
