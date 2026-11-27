"""Planned step-over gait: a quasi-static crawl generated from the measured box.

Three feet are always on the ground. Before each foot lifts, the body moves so the centre of
mass is inside the triangle of the other three feet; every swing is checked for clearance
between the whole leg (thigh, knee housing, shin, foot) and the box, for joint limits (the
hardware limits and the mic-bar-safe set), for reach, and for static motor torque.
A box that cannot be crossed inside those checks produces no plan.

Footholds. A foot may be planted on the floor before or beyond the box, and (if the box is
deep enough and ``allow_step_on`` is set) on the middle of the box's top. Pupper's knees all
point backward, so a front foot cannot land beyond the box without the body coming far
forward; while the other front foot is still behind the box that puts the centre of mass
outside the support triangle. The planner therefore usually sends the first front foot onto
the box top, the second straight over, and so on - whatever sequence passes the checks.

Planning frame ("world"): origin on the floor under the body centre at the start, x forward
along the robot's heading, y left, z up. The box's near face is ``near_x`` ahead of the body
centre (at y = 0) and may be skewed; it is treated as wide.

Pure numpy, no ROS. The trajectory this produces is played by ``player.TrajectoryPlayer``
on the robot and in simulation alike.
"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np

from . import kinematics as K
from . import stability as S

FR, FL, BR, BL = 0, 1, 2, 3
LEG_SHORT = ("FR", "FL", "BR", "BL")


@dataclass
class BoxSpec:
    """The obstacle, as measured. ``near_x``: distance from the body centre (base_link) to
    the near face along the robot's heading, at y = 0. ``skew``: rotation of the box about
    the vertical, radians; positive means the near face is farther away on the left."""

    height: float
    depth: float
    near_x: float
    skew: float = 0.0

    def faces(self, y):
        """(near_x, far_x) of the box at lateral position y."""
        near = self.near_x + np.asarray(y) * math.tan(self.skew)
        return near, near + self.depth / math.cos(self.skew)


@dataclass
class CrawlConfig:
    hip_height: float = 0.160        # hip-axis height above the floor while crossing (m)
    stance_widen: float = 0.020      # front feet placed this much farther out while crossing (m)
    stance_widen_rear: float = 0.040  # ... and rear feet (a wide rear stance is what balances a front step)
    clearance: float = 0.015         # required gap, fore-aft, between any leg part and the box's faces (m)
    clearance_top: float = 0.008     # required gap above the box's top (m); the box's position is
    #                                  uncertain fore-aft, much less so in height
    clearance_support: float = 0.005  # gap to the box for the shin of a leg whose foot stands on its top (m)
    allow_step_on: bool = True       # a foot may use the middle of the box top as a foothold
    top_min_depth: float = 0.055     # shallowest box whose top is used as a foothold (m)
    top_edge_margin: float = 0.012   # a foot on the top keeps its centre this far from the top's edges (m)
    top_landing: float = 0.42        # a foot lands on the top this fraction of the depth in from the near edge
    foot_gap: float = 0.015          # gap between the foot and a box face when planted beside it (m)
    final_extra: float = 0.015       # the rear feet's final place is this much farther beyond the box (m)
    step_lift: float = 0.018         # least foot lift while a foot travels (m)
    swing_cell: float = 0.006        # resolution of the swing-path search in the fore-aft plane (m)
    swing_headroom: float = 0.066    # how far above the higher of start/end/box the foot may go (m)
    min_margin: float = 0.015        # smallest allowed support margin (m)
    target_margin: float = 0.035     # margin the body shift aims for (m)
    front_margin: float = 0.026      # with a rear foot lifted, the centre of mass stays this far behind the front feet's line (m)
    reach_fraction: float = 0.97     # leg may extend to this fraction of its full length
    joint_margin: float = 0.03       # rad kept spare inside every joint range while planning
    torque_limit: float = 2.2        # static torque allowed per motor (N m); hardware limit is 3.0
    max_stride: float = 0.18         # longest single foot move considered (m)
    min_step: float = 0.02           # shortest useful foot move (m)
    kp: float = 7.5                  # position gain the trajectory will be played with (N m/rad)
    kd: float = 0.25                 # damping gain it will be played with (N m s/rad)
    sag_gain: float = 1.0            # scales the load compensation (raise if the real joints sag more than modelled)
    mic_bars: bool = True            # keep hip pitch inside the mic-bar-safe range
    # (mass kg, [x, y, z] in the body frame) for hardware that is not in the robot model
    extra_masses: tuple = ()
    dt: float = 0.02                 # trajectory sample period (s)
    max_joint_speed: float = 4.0     # rad/s: the trajectory is slowed wherever a joint would exceed this
    # timing
    rise_time: float = 0.8
    shift_speed: float = 0.10        # m/s
    shift_time_min: float = 0.40
    unload_time: float = 0.15
    load_time: float = 0.15
    swing_speed: float = 0.25        # m/s, foot speed along its path
    lower_speed: float = 0.12        # m/s, final descent onto the foothold
    segment_time_min: float = 0.20
    time_scale: float = 1.0          # > 1 slows the whole trajectory down
    max_steps: int = 26              # most foot swings in one plan
    max_evaluations: int = 1500      # search budget (candidate steps evaluated)
    pair_lead: float = 0.08          # a foot is discouraged from leading its left/right partner by more (m)
    stand_off: float = 0.020         # the counted approach stops with the front feet this far before the take-off line (m)
    hold_time: float = 0.0           # s to stand at the crossing height before the first step (tests)


@dataclass
class Step:
    leg: int
    start: np.ndarray        # foot centre, world (3,)
    end: np.ndarray
    path: np.ndarray         # foot-centre waypoints from start to end, world (k, 3)
    on_top: bool             # the foot lands on the box top
    body: np.ndarray         # body position during the swing, world (3,)
    body_path: np.ndarray    # waypoints the body moves through to get there, (k, 3), last == body
    margin: float
    clearance: float
    torque: float
    crosses: bool            # the swing passes over an edge of the box


@dataclass
class CrawlPlan:
    ok: bool
    reason: str = ""
    box: BoxSpec | None = None
    config: CrawlConfig | None = None
    steps: list = field(default_factory=list)
    t: np.ndarray | None = None          # (N,) seconds
    q_des: np.ndarray | None = None      # (N, 12) joint angles the legs should reach
    q_cmd: np.ndarray | None = None      # (N, 12) commanded angles (q_des + sag compensation)
    body: np.ndarray | None = None       # (N, 3) planned body position, world
    feet: np.ndarray | None = None       # (N, 4, 3) planned foot centres, world
    weights: np.ndarray | None = None    # (N, 4) planned load share flag per foot (1 planted, 0 in air)
    swing_leg: np.ndarray | None = None  # (N,) index of the leg in the air, or -1
    guard_leg: np.ndarray | None = None  # (N,) leg in guarded descent (stop on contact), or -1
    phase: np.ndarray | None = None      # (N,) phase index
    phase_names: list = field(default_factory=list)
    margin: np.ndarray | None = None     # (N,) planned support margin (m)
    template: dict | None = None         # the stored footstep pattern this plan came from, if any

    @property
    def duration(self) -> float:
        return float(self.t[-1]) if self.t is not None else 0.0

    def summary(self) -> str:
        if not self.ok:
            return f"no plan: {self.reason}"
        swings = len(self.steps)
        if not swings:
            return f"plan: hip height {self.config.hip_height * 1000:.0f} mm, no foot swings, {self.duration:.1f} s"
        cross = [s for s in self.steps if s.crosses]
        on_top = [s for s in self.steps if s.on_top]
        lines = [
            f"plan: hip height {self.config.hip_height * 1000:.0f} mm, {swings} foot swings "
            f"({len(cross)} over a box edge, {len(on_top)} onto the box top), {self.duration:.1f} s, "
            f"min support margin {min(s.margin for s in self.steps) * 1000:.0f} mm, "
            f"min leg-box clearance {min(s.clearance for s in self.steps) * 1000:.0f} mm, "
            f"max static torque {max(s.torque for s in self.steps):.2f} N m"
        ]
        for i, s in enumerate(self.steps):
            lines.append(
                f"  {i + 1:2d} {LEG_SHORT[s.leg]} x {s.start[0] * 1000:+5.0f} -> {s.end[0] * 1000:+5.0f} mm"
                f"{' ONTOP' if s.on_top else ('  OVER' if s.crosses else '      ')}  body ({s.body[0] * 1000:+4.0f}, {s.body[1] * 1000:+3.0f})"
                f"  margin {s.margin * 1000:3.0f}  clear {min(s.clearance, 0.999) * 1000:3.0f}  torque {s.torque:.2f}")
        return "\n".join(lines)


def _min_jerk(u):
    u = np.clip(u, 0.0, 1.0)
    return u * u * u * (10.0 - 15.0 * u + 6.0 * u * u)


def _flood(free, seed):
    """Cells of ``free`` (G, nx, nz) connected to ``seed`` (same shape) through free cells
    (4-neighbours), for every grid G at once."""
    reach = seed & free
    while True:
        grow = reach.copy()
        grow[:, 1:, :] |= reach[:, :-1, :]
        grow[:, :-1, :] |= reach[:, 1:, :]
        grow[:, :, 1:] |= reach[:, :, :-1]
        grow[:, :, :-1] |= reach[:, :, 1:]
        grow &= free
        if (grow == reach).all():
            return reach
        reach = grow


def _grid_path(free, start, goal):
    """Shortest 4-connected path of cells from start to goal through ``free`` (nx, nz)."""
    nx, nz = free.shape
    parent = -np.ones((nx, nz, 2), dtype=int)
    parent[start] = start
    frontier = [start]
    while frontier and parent[goal][0] < 0:
        nxt = []
        for (i, k) in frontier:
            for di, dk in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                a, b = i + di, k + dk
                if 0 <= a < nx and 0 <= b < nz and free[a, b] and parent[a, b, 0] < 0:
                    parent[a, b] = (i, k)
                    nxt.append((a, b))
        frontier = nxt
    if parent[goal][0] < 0:
        return None
    cells = [goal]
    while cells[-1] != start:
        cells.append(tuple(int(v) for v in parent[cells[-1]]))
    return cells[::-1]


def _straighten(cells, free):
    """Drop intermediate cells wherever a straight line between two cells stays in ``free``."""
    def visible(c1, c2):
        n = max(abs(c2[0] - c1[0]), abs(c2[1] - c1[1])) * 2
        for j in range(1, n):
            a = int(round(c1[0] + (c2[0] - c1[0]) * j / n))
            b = int(round(c1[1] + (c2[1] - c1[1]) * j / n))
            if not free[a, b]:
                return False
        return True

    out = [cells[0]]
    i = 0
    while i < len(cells) - 1:
        k = len(cells) - 1
        while k > i + 1 and not visible(cells[i], cells[k]):
            k -= 1
        out.append(cells[k])
        i = k
    return out


def _resample(path, spacing):
    """Points along a polyline (k, 3) every ``spacing`` metres, both ends included."""
    seg = np.linalg.norm(np.diff(path, axis=0), axis=1)
    total = float(seg.sum())
    if total < 1e-9:
        return path[:1].copy()
    n = max(2, int(math.ceil(total / spacing)) + 1)
    s_at = np.concatenate([[0.0], np.cumsum(seg)])
    u = np.linspace(0.0, total, n)
    return np.stack([np.interp(u, s_at, path[:, d]) for d in range(3)], axis=-1)


class _Planner:
    def __init__(self, box: BoxSpec, cfg: CrawlConfig):
        self.box, self.cfg = box, cfg
        self.h0, feet_b = K.default_stance()
        self.default_feet_body = feet_b.copy()
        self.default_feet_body[:, 2] = -self.h0 + K.FOOT_RADIUS
        self.zf = K.FOOT_RADIUS
        self.has_box = box.height > 1e-4 and box.depth > 1e-4
        self.z_top = box.height + K.FOOT_RADIUS
        self.use_top = self.has_box and cfg.allow_step_on and box.depth >= cfg.top_min_depth
        self.mass = K.total_mass(cfg.extra_masses)
        # required gaps for a foot resting on the top: thigh and knee as usual (measured to
        # the inflated box), shin close by design, foot touching (measured to the real box)
        self._thr_top = np.array([cfg.clearance_support, 0.0015, -0.0025])
        self.trace = None      # set to a list to record why candidate steps were rejected

    # ------------------------------------------------------------------ geometry helpers
    def zone(self, y):
        """Foot-centre x interval in which a foot may not be planted on the floor, at y."""
        if not self.has_box:
            return math.inf, -math.inf
        near, far = self.box.faces(y)
        pad = K.FOOT_ENVELOPE_RADIUS + self.cfg.foot_gap
        return float(near - pad), float(far + pad)

    def top_x(self, y):
        """Where a foot is set down on the box top. Not the middle: the foot rolls forward as
        the body passes over it, so it starts a little short of the middle."""
        near, far = self.box.faces(y)
        depth = far - near
        return float(near + max(self.cfg.top_edge_margin + 0.005, self.cfg.top_landing * depth))

    def ik_all(self, feet_w, body):
        """Joint angles for feet (4, 3) or (..., 4, 3) world and body (..., 3) world.
        -> q (..., 12), ok (...)."""
        body = np.asarray(body, dtype=float)
        feet_w = np.asarray(feet_w, dtype=float)
        qs, ok = [], np.ones(np.broadcast_shapes(body.shape[:-1], feet_w.shape[:-2]), dtype=bool)
        for i, leg in enumerate(K.LEGS):
            rel = feet_w[..., i, :] - body
            q, o = K.leg_ik(leg, rel, self.cfg.mic_bars, self.cfg.joint_margin)
            reach = np.linalg.norm(rel - K.hip_position(leg), axis=-1)
            ok &= o & (reach <= self.cfg.reach_fraction * K.LEG_MAX_REACH)
            qs.append(np.broadcast_to(q, ok.shape + (3,)))
        return np.concatenate(qs, axis=-1), ok

    def rolled_feet(self, feet_w, body_from, body_to):
        """Where the planted feet end up when the body moves from ``body_from`` (3,) to
        ``body_to`` (..., 3): ball feet roll, so their centres travel a little with the shin.
        Returns (..., 4, 3)."""
        q_from, _ = self.ik_all(feet_w, body_from)
        q_to, _ = self.ik_all(feet_w, body_to)
        out = np.broadcast_to(feet_w, q_to.shape[:-1] + (4, 3)).copy()
        for i, leg in enumerate(K.LEGS):
            out[..., i, :] += K.rolling_shift(K.shin_rotation(leg, q_from[3 * i:3 * i + 3]),
                                              K.shin_rotation(leg, q_to[..., 3 * i:3 * i + 3]))
        return out

    def clearance(self, leg_i, q, body, foot_y):
        """Leg-to-box clearance score for leg ``leg_i`` with joints q (..., 3) and body (..., 3);
        a value >= cfg.clearance passes. The box is inflated by ``clearance`` fore and aft and
        by ``clearance_top`` upward, and no part of the leg may enter the inflated box. A foot
        over the middle of the box top is allowed to rest on it, with its shin close by."""
        cfg = self.cfg
        pts = K.leg_points(K.LEGS[leg_i], q)
        hip, knee, foot = pts["hip"] + body, pts["knee"] + body, pts["foot"] + body
        floor_ok = knee[..., 2] - K.KNEE_RADIUS >= 0.004
        if not self.has_box:
            return np.where(floor_ok, 1.0, -1.0)
        near, far = self.box.faces(foot_y)
        h = self.box.height
        infl = S.leg_box_gaps(hip, knee, foot, near - cfg.clearance, far + cfg.clearance, h + cfg.clearance_top)
        free = infl.min(axis=-1)
        real = S.leg_box_gaps(hip, knee, foot, near, far, h)
        on_top = np.minimum(infl[..., :2].min(axis=-1), (real[..., 2:] - self._thr_top).min(axis=-1))
        # A foot may rest on the top only with its centre this far in from either edge: a ball
        # foot rolls forward as the body passes over it, and must not reach the edge.
        edge = cfg.top_edge_margin
        over_top = ((foot[..., 0] >= near + edge) & (foot[..., 0] <= far - edge)
                    & (foot[..., 2] - K.FOOT_RADIUS >= h - 0.0015))
        eff = np.where(over_top, on_top, free) + cfg.clearance
        return np.where(floor_ok, eff, -1.0)

    def all_clearance(self, q12, body, feet_w):
        out = None
        for i in range(4):
            c = self.clearance(i, q12[..., 3 * i:3 * i + 3], body, feet_w[i][1])
            out = c if out is None else np.minimum(out, c)
        return out

    def _front_slack(self, com, feet):
        """With a rear foot in the air, tipping forward over the line through the two front
        feet is the fall nothing catches (and front-leg sag feeds it: the nose drops, the
        weight moves forward). So the centre of mass must be ``front_margin`` behind that
        line, not just ``min_margin``. Returns the ordinary-margin equivalent:
        min_margin + (distance behind the line - front_margin), to be min-ed with the margin.
        com (..., 3) and feet (..., 4, 3) or (4, 3), world."""
        fr, fl = feet[..., FR, :2], feet[..., FL, :2]
        u = (com[..., 1] - fr[..., 1]) / np.where(np.abs(fl[..., 1] - fr[..., 1]) < 1e-6, 1e-6, fl[..., 1] - fr[..., 1])
        x_line = fr[..., 0] + np.clip(u, 0.0, 1.0) * (fl[..., 0] - fr[..., 0])
        return self.cfg.min_margin + (x_line - com[..., 0]) - self.cfg.front_margin

    def com_world(self, q12, body):
        return S.center_of_mass_batch(q12, self.cfg.extra_masses) + body

    # ------------------------------------------------------------------ one candidate step
    def evaluate(self, feet_w, body, leg, target):
        """Best body position for swinging ``leg`` to ``target`` (x, y, z), or None.

        A grid of body positions around the current one is tested; the cheap checks run on
        the whole grid and the expensive ones only on what is left. For each surviving body
        position the foot's free space in the fore-aft plane (reachable, inside the joint
        sets, whole leg clear of the box) is flooded from the take-off point; the swing is
        possible if the flood reaches the landing point.
        Returns (body (3,), margin, clearance, torque, path (k, 3), crosses, body_path (k, 3),
        feet (4, 3) as they will be, after rolling, when the swing starts)."""
        cfg = self.cfg
        p0 = feet_w[leg].copy()
        p1 = np.asarray(target, dtype=float)
        crosses = False
        if self.has_box:
            near, far = self.box.faces(p0[1])
            pad = K.FOOT_ENVELOPE_RADIUS + 0.002
            crosses = bool(max(p0[0], p1[0]) > near - pad and min(p0[0], p1[0]) < far + pad)
        leg_name = K.LEGS[leg]
        hip = K.hip_position(leg_name)
        reach_max = cfg.reach_fraction * K.LEG_MAX_REACH
        stance = [i != leg for i in range(4)]
        rec = {"leg": LEG_SHORT[leg], "to": [round(float(v), 3) for v in p1]}
        if self.trace is not None:
            self.trace.append(rec)

        gx = body[0] + np.arange(-0.05, 0.1501, 0.01)
        gy = np.arange(-0.06, 0.0601, 0.01)
        na, nb = len(gx), len(gy)
        GX, GY = np.meshgrid(gx, gy, indexing="ij")
        B = np.stack([GX.ravel(), GY.ravel(), np.full(GX.size, body[2])], axis=-1)    # (G, 3)
        idx = np.arange(len(B))

        # 1. all four feet reachable (where they will have rolled to), legs clear of the box,
        #    and the body balanced on all four: where the body may be while no foot is lifted
        F = self.rolled_feet(feet_w, body, B)                                 # (G, 4, 3)
        q_start, ok = self.ik_all(F, B)
        B, q_start, idx, F = B[ok], q_start[ok], idx[ok], F[ok]
        rec["stance_ik"] = len(B)
        if not len(B):
            return None
        clear = self.all_clearance(q_start, B, feet_w)
        com0 = self.com_world(q_start, B)
        keep = (clear >= cfg.clearance - 1e-4) & (
            S.support_margin(com0[:, :2], F[:, :, :2], [True] * 4) >= 0.010)
        B, q_start, clear, idx, com0, F = B[keep], q_start[keep], clear[keep], idx[keep], com0[keep], F[keep]
        rec["stance_clear"] = len(B)
        if not len(B):
            return None
        # which of those can be reached from the current body position without leaving the set
        free_b = np.zeros(na * nb, dtype=bool)
        free_b[idx] = True
        a0 = 5
        b0 = int(round((body[1] - gy[0]) / 0.01))
        if not (0 <= b0 < nb):
            return None
        start = a0 * nb + b0
        free_b[start] = True
        parent = np.full(na * nb, -1, dtype=int)
        cost = np.full(na * nb, np.inf)
        parent[start] = start
        cost[start] = 0.0
        frontier = [start]
        while frontier:                      # breadth-first over the 8-connected grid
            nxt = []
            for c in frontier:
                ca, cb = divmod(c, nb)
                for da in (-1, 0, 1):
                    for db in (-1, 0, 1):
                        a2, b2 = ca + da, cb + db
                        if (da or db) and 0 <= a2 < na and 0 <= b2 < nb:
                            n2 = a2 * nb + b2
                            if free_b[n2] and parent[n2] < 0:
                                parent[n2] = c
                                cost[n2] = cost[c] + 0.01 * math.hypot(da, db)
                                nxt.append(n2)
            frontier = nxt
        keep = parent[idx] >= 0
        B, q_start, clear, idx, com0, F = B[keep], q_start[keep], clear[keep], idx[keep], com0[keep], F[keep]
        rec["reachable"] = len(B)
        if not len(B):
            return None

        # 2. swing foot reachable at the end of its path (cheap) ...
        q_end, ok_end = K.leg_ik(leg_name, p1 - B, cfg.mic_bars, cfg.joint_margin)
        ok_end &= np.linalg.norm(p1 - B - hip, axis=-1) <= reach_max
        # ... and roughly balanced on the other three feet (swing leg where it starts)
        rough = S.support_margin(com0[:, :2], F[:, :, :2], stance)
        keep = ok_end & (rough >= cfg.min_margin - 0.012)
        B, q_start, clear, idx, F = B[keep], q_start[keep], clear[keep], idx[keep], F[keep]
        rec["end_and_rough_balance"] = len(B)
        if not len(B):
            return None

        # 3. the swing: flood the foot's free space from take-off towards landing
        c = cfg.swing_cell
        x_lo, x_hi = min(p0[0], p1[0]), max(p0[0], p1[0])
        n_in = max(1, int(math.ceil((x_hi - x_lo) / c)))
        xs = x_lo + (x_hi - x_lo) * np.arange(-4, n_in + 5) / n_in if x_hi - x_lo > 1e-6 else \
            x_lo + c * np.arange(-4, 5)
        i1 = int(np.argmin(np.abs(xs - p1[0])))
        # the foot starts where it has rolled to, which differs a little per body position
        i0 = np.argmin(np.abs(xs[None, :] - F[:, leg, 0:1]), axis=1)          # (G,)
        z_base = min(p0[2], p1[2])
        z_ceiling = max(p0[2], p1[2], (self.z_top if self.has_box else 0.0)) + cfg.swing_headroom
        zs = np.arange(z_base, z_ceiling + 1e-9, c)
        k0 = int(round((p0[2] - z_base) / c))
        k1 = int(round((p1[2] - z_base) / c))
        zs[k0], zs[k1] = p0[2], p1[2]
        nx, nz = len(xs), len(zs)
        frac = np.clip((xs - p0[0]) / (p1[0] - p0[0]), 0.0, 1.0) if abs(p1[0] - p0[0]) > 1e-6 else np.zeros(nx)
        ys = p0[1] + (p1[1] - p0[1]) * frac
        P = np.empty((nx, nz, 3))
        P[..., 0] = xs[:, None]
        P[..., 1] = ys[:, None]
        P[..., 2] = zs[None, :]
        # a travelling foot stays lifted; only directly above take-off and landing may it be low
        G = len(B)
        rows = np.arange(G)
        z_min = np.full((G, nx), max(p0[2], p1[2]) + cfg.step_lift)
        z_min[:, i1] = p1[2]
        z_min[rows, i0] = np.minimum(z_min[rows, i0], p0[2])
        lifted = zs[None, None, :] >= z_min[:, :, None] - 1e-9                # (G, nx, nz)

        rel = P[None] - B[:, None, None, :]                                  # (G, nx, nz, 3)
        q_sw, free = K.leg_ik(leg_name, rel, cfg.mic_bars, cfg.joint_margin)
        free &= np.linalg.norm(rel - hip, axis=-1) <= reach_max
        free &= lifted
        if self.has_box:
            # (the box's faces are taken at the lateral position the foot has at each point
            #  of its path: on a skewed box they differ between take-off and landing)
            c_sw = self.clearance(leg, q_sw, B[:, None, None, :], ys[None, :, None])
            free &= c_sw >= cfg.clearance - 1e-4
        seed = np.zeros_like(free)
        seed[rows, i0, k0] = True
        free[rows, i0, k0] = True       # the foot is there now
        reach = _flood(free, seed)
        keep = reach[:, i1, k1]
        rec["swing_ok"] = int(keep.sum())
        B, q_start, clear, idx, F, i0 = B[keep], q_start[keep], clear[keep], idx[keep], F[keep], i0[keep]
        free, q_sw = free[keep], q_sw[keep]
        rec["kinematic"] = len(B)
        if not len(B):
            return None

        # 4. balance with the swing foot at take-off and at landing (the path itself is
        #    checked for the chosen body position below)
        margin = np.full(len(B), np.inf)
        rows = np.arange(len(B))
        for end in (0, 1):
            q12 = q_start.copy()
            q12[:, 3 * leg:3 * leg + 3] = q_sw[rows, i0, k0, :] if end == 0 else q_sw[:, i1, k1, :]
            com = self.com_world(q12, B)
            margin = np.minimum(margin, S.support_margin(com[:, :2], F[:, :, :2], stance))
            if leg in (BR, BL):
                margin = np.minimum(margin, self._front_slack(com, F))
        q_mid, com_mid = q12, com
        rec["best_margin_mm"] = round(float(margin.max()) * 1000, 1)
        keep = margin >= cfg.min_margin
        rec["balanced"] = int(keep.sum())
        if not keep.any():
            return None

        # 5. static torque with three feet carrying
        weights = np.array([1.0 if st else 0.0 for st in stance])
        forces = S.contact_forces(com_mid[:, :2], F[:, :, :2], weights, self.mass)
        tau = np.abs(S.joint_torques(q_mid, forces)).max(axis=-1)
        keep &= tau <= cfg.torque_limit
        rec["torque_ok"] = int(keep.sum())

        if not keep.any():
            return None
        rec["shift_ok"] = int(keep.sum())

        score = np.where(keep, np.minimum(margin, cfg.target_margin) - 0.10 * cost[idx], -np.inf)
        for _ in range(6):
            j = int(np.argmax(score))
            if not np.isfinite(score[j]):
                return None
            # the foot's way through its free space, straightened, and re-checked for balance
            cells = _grid_path(free[j], (int(i0[j]), k0), (i1, k1))
            cells = _straighten(cells, free[j])
            path = np.array([[xs[a_], ys[a_], zs[b_]] for a_, b_ in cells])
            path[0], path[-1] = F[j, leg], p1
            dense = _resample(path, 0.01)
            q_path, ok_path = K.leg_ik(leg_name, dense - B[j], cfg.mic_bars, cfg.joint_margin)
            q12 = np.repeat(q_start[j][None], len(dense), axis=0)
            q12[:, 3 * leg:3 * leg + 3] = q_path
            com = self.com_world(q12, B[j])
            m_path = float(S.support_margin(com[:, :2], F[j, :, :2], stance).min())
            if leg in (BR, BL):
                m_path = min(m_path, float(self._front_slack(com, F[j]).min()))
            if ok_path.all() and m_path >= cfg.min_margin:
                margin[j] = min(margin[j], m_path)
                break
            score[j] = -np.inf
        else:
            return None
        # 6. the body's way there: back along the breadth-first tree, straightened
        cells = [int(idx[j])]
        while cells[-1] != start:
            cells.append(int(parent[cells[-1]]))
        cells.reverse()
        free_body = free_b.reshape(na, nb)
        way = _straighten([divmod(cc, nb) for cc in cells], free_body)
        body_path = np.array([[gx[a_], gy[b_], body[2]] for a_, b_ in way[1:]]).reshape(-1, 3)
        if len(body_path) == 0:
            body_path = B[j:j + 1].copy()
        return (B[j].copy(), float(margin[j]), float(clear[j]), float(tau[j]), path, crosses, body_path, F[j].copy())

    # ------------------------------------------------------------------ the footstep sequence
    def candidates(self, foot, final_x):
        """Footholds to try for a foot, farthest first: (x, z, on_top). Floor positions every
        20 mm outside the box's keep-out zone, the take-off line just before the box, landings
        just beyond it, and the middle of the box top."""
        cfg = self.cfg
        x, y = float(foot[0]), float(foot[1])
        lo, hi = self.zone(y)
        top = min(final_x, x + cfg.max_stride)
        xs = list(np.arange(top, x + cfg.min_step - 1e-9, -0.02))
        if x < lo - 1e-6:
            xs.append(lo)
        if hi <= x + cfg.max_stride:
            xs += [hi, hi + 0.01]
        out = [(round(float(v), 6), self.zf, False) for v in xs
               if not (lo + 1e-6 < v < hi - 1e-6) and v - x >= cfg.min_step - 1e-9 and v <= final_x + 1e-9]
        if self.use_top:
            xt = self.top_x(y)
            if cfg.min_step <= xt - x <= cfg.max_stride:
                out.append((round(xt, 6), self.z_top, True))
        return sorted(set(out), key=lambda c: -c[0])

    def setup(self, travel=None):
        """Start state and targets. Returns an error string, or None when ready."""
        cfg = self.cfg
        body = np.array([0.0, 0.0, self.h0])
        feet = self.default_feet_body + body
        feet[:, 2] = self.zf
        widen = np.array([cfg.stance_widen, cfg.stance_widen, cfg.stance_widen_rear, cfg.stance_widen_rear])
        self.wide_y = self.default_feet_body[:, 1] + np.sign(self.default_feet_body[:, 1]) * widen
        self.final_y = self.default_feet_body[:, 1].copy()
        if self.has_box:
            # the rear feet finish a little beyond the landing line: by the end of a long plan
            # the feet are several millimetres from where they were planned to be
            rear_final = max(self.zone(self.final_y[BR])[1], self.zone(self.final_y[BL])[1]) + cfg.final_extra
            self.final_body_x = rear_final - self.default_feet_body[BR, 0]
        else:
            self.final_body_x = float(travel or 0.0)
        self.final_x = self.default_feet_body[:, 0] + self.final_body_x
        if self.has_box:
            for i in range(4):
                lo, _ = self.zone(feet[i, 1])
                if feet[i, 0] > lo + 1e-6:
                    return "a foot is already at or past the box's take-off line; the robot is too close to the box to start"
        self.start_feet = feet
        self.start_body = np.array([0.0, 0.0, cfg.hip_height])
        _, ok0 = self.ik_all(feet, self.start_body)
        if not ok0:
            return "cannot stand at the crossing height"
        self.evals = 0
        return None

    def unfinished(self, feet, on_top):
        remaining = self.final_x - feet[:, 0]
        lateral = np.abs(self.final_y - feet[:, 1])
        return remaining, [remaining[i] >= 0.006 or lateral[i] >= 0.004 or on_top[i] for i in range(4)]

    def try_step(self, feet, on_top, body_c, leg, x_new, z_new, top, narrow=False):
        """Evaluate one step; returns (Step, feet after, on_top after, body after) or None.
        The foot lands at the widened crossing stance unless it is at its final place (or
        ``narrow`` asks for the default stance width)."""
        at_final = abs(x_new - self.final_x[leg]) < 0.003 and not top
        y_new = self.final_y[leg] if (at_final or narrow) else self.wide_y[leg]
        self.evals += 1
        res = self.evaluate(feet, body_c, leg, (x_new, y_new, z_new))
        if res is None:
            return None
        b_new, margin, clear, tau, fpath, crosses, bpath, feet_rolled = res
        feet2 = feet_rolled.copy()
        start_pos = feet2[leg].copy()
        feet2[leg] = [x_new, y_new, z_new]
        top2 = list(on_top)
        top2[leg] = top
        step = Step(leg, start_pos, feet2[leg].copy(), fpath, top, b_new, bpath, margin, clear, tau, crosses)
        return step, feet2, top2, b_new

    def crawl_cycle(self, feet, on_top, body_c, targets, narrow=False):
        """One plain crawl cycle on clear floor: each foot to its x in ``targets`` (4,), a rear
        foot first (lifting it needs no lean), then the front foot on the same side.
        Returns (steps, feet, on_top, body) or None."""
        steps = []
        for leg in (BL, FL, BR, FR):
            if targets[leg] - feet[leg, 0] < 0.5 * self.cfg.min_step:
                continue
            res = self.try_step(feet, on_top, body_c, leg, float(targets[leg]), self.zf, False, narrow)
            if res is None:
                return None
            st, feet, on_top, body_c = res
            steps.append(st)
        return steps, feet, on_top, body_c

    def crawl_forward(self, feet, on_top, body_c, distance, narrow=True):
        """Crawl cycles that carry every foot ``distance`` ahead of where it is now (absolute
        targets, so rolling does not accumulate). Returns (steps, feet, on_top, body) or None."""
        x0 = feet[:, 0].copy()
        for max_stride in (0.08, 0.06, 0.045):
            n = max(1, int(math.ceil(distance / max_stride)))
            state = (feet.copy(), list(on_top), body_c.copy(), [])
            for c in range(n):
                res = self.crawl_cycle(state[0], state[1], state[2], x0 + distance * (c + 1) / n, narrow)
                if res is None:
                    break
                state = (res[1], res[2], res[3], state[3] + res[0])
            else:
                return state[3], state[0], state[1], state[2]
        return None

    def plan_cycles(self, distance):
        """A plain crawl straight ahead by ``distance`` on clear floor."""
        err = self.setup(travel=distance)
        if err:
            return CrawlPlan(False, err, self.box, self.cfg)
        res = self.crawl_forward(self.start_feet.copy(), [False] * 4, self.start_body.copy(), distance)
        if res is None:
            return CrawlPlan(False, "a crawl step failed the balance or reach checks", self.box, self.cfg)
        steps = res[0]
        return self.finish(steps)

    def finish(self, steps):
        final_body = np.array([self.final_body_x, 0.0, self.cfg.hip_height])
        return self.build(steps, final_body)

    # ---- templates: a footstep sequence written relative to the box, replayed on a new box
    def encode(self, steps):
        """A plan's footsteps as a template: [[leg, kind, offset_mm], ...] with kind
        'lo' (before the box, offset from the take-off line), 'top' (middle of the box top),
        'hi' (beyond the box, offset from the landing line) or 'final'."""
        out = []
        for st in steps:
            lo, hi = self.zone(st.end[1])
            if st.on_top:
                out.append([LEG_SHORT[st.leg], "top", 0])
            elif abs(st.end[0] - self.final_x[st.leg]) < 0.003:
                out.append([LEG_SHORT[st.leg], "final", 0])
            elif st.end[0] <= lo + 1e-4:
                out.append([LEG_SHORT[st.leg], "lo", int(round((st.end[0] - lo) * 1000))])
            else:
                out.append([LEG_SHORT[st.leg], "hi", int(round((st.end[0] - hi) * 1000))])
        return out

    def plan_template(self, template):
        """Replay a template on this box, checking every step in full. None if any step fails."""
        err = self.setup()
        if err:
            return CrawlPlan(False, err, self.box, self.cfg)
        cfg = self.cfg
        feet, on_top, body_c = self.start_feet.copy(), [False] * 4, self.start_body.copy()
        steps = []
        # Counted approach: plain crawl cycles, at the default stance width, until the front
        # feet are ``stand_off`` before the take-off line - the stance the templates were found
        # from. The camera cannot see the box this close, so this is how the last part is done.
        lo_front = min(self.zone(feet[FL, 1])[0], self.zone(feet[FR, 1])[0])
        need = lo_front - max(feet[FL, 0], feet[FR, 0]) - cfg.stand_off
        if need >= cfg.min_step:
            res = self.crawl_forward(feet, on_top, body_c, need)
            if res is None:
                self.failed_at = -1
                return None
            new_steps, feet, on_top, body_c = res
            steps += new_steps
        self.approach_steps = len(steps)
        for leg_name, kind, off in template:
            leg = LEG_SHORT.index(leg_name)
            # the box's lines where the foot will land (on a skewed box they depend on y):
            # the widened crossing stance, or the default stance width for its final place
            lo, hi = self.zone(self.wide_y[leg])
            if kind == "top":
                if not self.use_top:
                    return None
                x_new, z_new, top = self.top_x(self.wide_y[leg]), self.z_top, True
            elif kind == "final":
                x_new, z_new, top = float(self.final_x[leg]), self.zf, False
            elif kind == "lo":
                x_new, z_new, top = lo + off / 1000.0, self.zf, False
            else:
                x_new, z_new, top = hi + off / 1000.0, self.zf, False
            x_new = min(x_new, float(self.final_x[leg]))
            if not top and abs(x_new - self.final_x[leg]) < 0.003:
                lo, hi = self.zone(self.final_y[leg])
            if not top and lo + 1e-6 < x_new < hi - 1e-6:
                return None
            lateral = abs(self.final_y[leg] - feet[leg, 1])
            if x_new - feet[leg, 0] < 0.6 * cfg.min_step and not (kind == "final" and (lateral >= 0.004 or on_top[leg])):
                continue                     # this foot is already there on this box: skip the step
            res = self.try_step(feet, on_top, body_c, leg, x_new, z_new, top)
            if res is None and kind in ("lo", "hi", "top"):
                # The pattern was found on another box; a foothold that does not quite work
                # here is tried a little shorter or longer before the pattern is given up.
                for delta in ((-0.006, 0.006) if top else (-0.010, 0.010, -0.020, 0.020, -0.030, 0.030)):
                    x_try = min(x_new + delta, float(self.final_x[leg]))
                    if x_try - feet[leg, 0] < cfg.min_step or (not top and lo + 1e-6 < x_try < hi - 1e-6):
                        continue
                    if top:
                        near, far = self.box.faces(self.wide_y[leg])
                        if not (near + cfg.top_edge_margin + 0.003 <= x_try <= far - cfg.top_edge_margin - 0.003):
                            continue
                    res = self.try_step(feet, on_top, body_c, leg, x_try, z_new, top)
                    if res is not None:
                        break
            if res is None:
                self.failed_at = len(steps)
                return None
            st, feet, on_top, body_c = res
            steps.append(st)
        # whatever is left: bring each foot to its final place
        for _ in range(8):
            _, todo = self.unfinished(feet, on_top)
            if not any(todo):
                break
            moved = False
            for leg in (FL, FR, BL, BR):
                if todo[leg] and not (steps and steps[-1].leg == leg and sum(todo) > 1):
                    res = self.try_step(feet, on_top, body_c, leg, float(self.final_x[leg]), self.zf, False)
                    if res is not None:
                        st, feet, on_top, body_c = res
                        steps.append(st)
                        moved = True
                        break
            if not moved:
                return None
        else:
            return None
        plan = self.finish(steps)
        return plan if plan.ok else None

    def plan(self, travel=None):
        """Search for a footstep sequence from scratch (depth-first, best-looking move first,
        with backtracking: a long stride that leaves the other feet unable to follow is undone)."""
        cfg = self.cfg
        err = self.setup(travel)
        if err:
            return CrawlPlan(False, err, self.box, cfg)
        final_x = self.final_x
        order = (FL, FR, BL, BR)
        pair = {FL: FR, FR: FL, BL: BR, BR: BL}
        seen = set()

        def key(feet, on_top, last_leg):
            return (tuple(np.round(feet[:, 0] / 0.005).astype(int)), tuple(np.round(feet[:, 1] / 0.005).astype(int)),
                    tuple(on_top), last_leg)

        def moves(feet, on_top, last_leg):
            remaining, unfinished = self.unfinished(feet, on_top)
            out = []
            for rank, leg in enumerate(sorted(order, key=lambda i: -remaining[i])):
                if not unfinished[leg]:
                    continue
                if leg == last_leg and any(unfinished[j] for j in range(4) if j != leg):
                    continue
                if remaining[leg] < cfg.min_step and not on_top[leg]:
                    cands = [(float(final_x[leg]), self.zf, False)]   # small or lateral correction
                else:
                    cands = self.candidates(feet[leg], final_x[leg])
                for x_new, z_new, top in cands:
                    # prefer progress, but not one foot of a pair running far ahead of the other
                    ahead = max(0.0, x_new - feet[pair[leg], 0] - cfg.pair_lead)
                    score = (x_new - feet[leg, 0]) - 2.0 * ahead - 0.05 * rank
                    out.append((score, leg, x_new, z_new, top))
            out.sort(key=lambda m: -m[0])
            return unfinished, out

        def search(feet, on_top, body_c, last_leg, depth):
            unfinished, options = moves(feet, on_top, last_leg)
            if not any(unfinished):
                return []
            if depth >= cfg.max_steps:
                return None
            k = key(feet, on_top, last_leg)
            if k in seen:
                return None
            seen.add(k)
            for _, leg, x_new, z_new, top in options:
                if self.evals >= cfg.max_evaluations:
                    return None
                res = self.try_step(feet, on_top, body_c, leg, x_new, z_new, top)
                if res is None:
                    continue
                step, feet2, top2, b_new = res
                rest = search(feet2, top2, b_new, leg, depth + 1)
                if rest is not None:
                    return [step] + rest
            return None

        steps = search(self.start_feet.copy(), [False] * 4, self.start_body.copy(), -1, 0)
        if steps is None:
            why = ("search budget used up" if self.evals >= cfg.max_evaluations else
                   "no footstep sequence satisfies the balance, reach, clearance and torque limits")
            return CrawlPlan(False, f"{why} at hip height {cfg.hip_height * 1000:.0f} mm "
                                    "(box too tall or too deep)", self.box, cfg)
        return self.finish(steps)

    # ------------------------------------------------------------------ trajectory
    def build(self, steps, final_body):
        cfg = self.cfg
        ts = cfg.time_scale
        body = np.array([0.0, 0.0, self.h0])
        feet = self.default_feet_body + body
        feet[:, 2] = self.zf

        seg_body, seg_feet, seg_w, seg_swing, seg_guard, seg_phase = [], [], [], [], [], []
        names = []

        def add(name, n, body_fn, feet_fn, w_fn, swing=-1, guard_fn=None):
            idx = len(names)
            names.append(name)
            for k in range(1, n + 1):
                u = k / n
                seg_body.append(body_fn(u))
                seg_feet.append(feet_fn(u))
                seg_w.append(w_fn(u))
                seg_swing.append(swing)
                seg_guard.append(swing if (guard_fn is not None and guard_fn(u)) else -1)
                seg_phase.append(idx)

        def nsamp(T):
            return max(1, int(round(T * ts / cfg.dt)))

        ones = np.ones(4)

        def body_move(b0, b1):
            return lambda u: b0 + (b1 - b0) * _min_jerk(u)

        # rise to the crossing height
        b1 = np.array([body[0], body[1], cfg.hip_height])
        f0 = feet.copy()
        add("rise", nsamp(cfg.rise_time), body_move(body.copy(), b1), lambda u, f=f0: f, lambda u: ones)
        body = b1
        if cfg.hold_time > 0:
            add("hold", nsamp(cfg.hold_time), lambda u, b=b1.copy(): b, lambda u, f=f0: f, lambda u: ones)

        for si, s in enumerate(steps):
            # shift the body over the support triangle (through the planned waypoints)
            for wp in s.body_path:
                dist = float(np.linalg.norm(wp - body))
                if dist > 1e-4:
                    T = max(cfg.shift_time_min, dist / cfg.shift_speed)
                    add(f"shift{si + 1}", nsamp(T), body_move(body.copy(), wp.copy()),
                        lambda u, f=feet.copy(): f, lambda u: ones)
                    body = wp.copy()
            leg = s.leg
            w_air = ones.copy()
            w_air[leg] = 0.0
            fa = feet.copy()
            bb = body.copy()
            # unload
            add(f"unload{si + 1}", nsamp(cfg.unload_time), lambda u, b=bb: b, lambda u, f=fa: f,
                lambda u, w=w_air: ones + (w - ones) * _min_jerk(u), swing=leg)
            p1 = s.end.copy()

            def moving(a, b, f=fa, leg=leg):
                def fn(u):
                    out = f.copy()
                    out[leg] = a + (b - a) * _min_jerk(u)
                    return out
                return fn

            # along the planned polyline, coming to rest at each corner (exact, so the
            # clearance that was checked is the clearance that is flown)
            last = len(s.path) - 2
            for k in range(len(s.path) - 1):
                a, b = s.path[k].copy(), s.path[k + 1].copy()
                final_descent = k == last and b[2] < a[2] - 1e-4
                speed = cfg.lower_speed if final_descent else cfg.swing_speed
                T = max(cfg.segment_time_min, float(np.linalg.norm(b - a)) / speed)
                add(f"swing{si + 1}.{k + 1}", nsamp(T), lambda u, b_=bb: b_, moving(a, b), lambda u, w=w_air: w,
                    swing=leg, guard_fn=(lambda u: u > 0.5) if final_descent else None)
            feet[leg] = p1
            fb = feet.copy()
            add(f"load{si + 1}", nsamp(cfg.load_time), lambda u, b=bb: b, lambda u, f=fb: f,
                lambda u, w=w_air: w + (ones - w) * _min_jerk(u), swing=leg)

        # centre the body over the final stance, then lower to the default height
        dist = float(np.linalg.norm(final_body - body))
        if dist > 1e-4:
            T = max(cfg.shift_time_min, dist / cfg.shift_speed)
            add("centre", nsamp(T), body_move(body.copy(), final_body.copy()), lambda u, f=feet.copy(): f, lambda u: ones)
            body = final_body.copy()
        b_end = np.array([body[0], body[1], self.h0])
        add("lower_body", nsamp(cfg.rise_time), body_move(body.copy(), b_end), lambda u, f=feet.copy(): f, lambda u: ones)

        body_arr = np.array(seg_body)
        feet_arr = np.array(seg_feet)
        w_arr = np.array(seg_w)
        n = len(body_arr)
        phase_arr = np.array(seg_phase)
        swing_arr = np.array(seg_swing)

        # Rolling. A planted ball foot rolls as its shin turns, so its centre does not stay
        # where it was put down. Integrate that through every phase in which the body moves,
        # so the plan knows where the feet really are (measured from the box) at each lift.
        off = np.zeros((4, 3))
        q_ref, _ = self.ik_all(self.default_feet_body + np.array([0.0, 0.0, self.h0]), np.array([0.0, 0.0, self.h0]))
        R_ref = [K.shin_rotation(leg, q_ref[3 * i:3 * i + 3]) for i, leg in enumerate(K.LEGS)]
        for ph in range(len(names)):
            ii = np.nonzero(phase_arr == ph)[0]
            if not len(ii):
                continue
            name = names[ph]
            leg = int(swing_arr[ii[0]])
            if leg < 0:                                   # body moves, all four feet planted
                base = feet_arr[ii] + off
                Fp = base.copy()
                for _ in range(3):
                    for i, lname in enumerate(K.LEGS):
                        q, _ = K.leg_ik(lname, Fp[:, i, :] - body_arr[ii], cfg.mic_bars)
                        shift = K.rolling_shift(R_ref[i], K.shin_rotation(lname, q))
                        Fp[:, i, :] = base[:, i, :] + shift
                feet_arr[ii] = Fp
                for i, lname in enumerate(K.LEGS):
                    off[i] = Fp[-1, i] - (base[-1, i] - off[i])
                    q, _ = K.leg_ik(lname, Fp[-1, i] - body_arr[ii[-1]], cfg.mic_bars)
                    R_ref[i] = K.shin_rotation(lname, q)
            else:                                         # body still; one leg unloading / flying / loading
                planned = feet_arr[ii].copy()
                feet_arr[ii] = planned + off
                if name.startswith("swing"):
                    first = name.endswith(".1")
                    if first:
                        # the path starts where the search expected the foot; ease any small
                        # difference from where rolling actually left it over this segment
                        delta = (self._last_planted[leg] + off[leg]) - planned[0, leg] if hasattr(self, "_last_planted") else np.zeros(3)
                        u = np.arange(1, len(ii) + 1) / len(ii)
                        feet_arr[ii, leg] = planned[:, leg] + delta * (1.0 - _min_jerk(u))[:, None]
                    else:
                        feet_arr[ii, leg] = planned[:, leg]
                elif name.startswith("load"):
                    feet_arr[ii, leg] = planned[:, leg]
                    off[leg] = 0.0
                    q, _ = K.leg_ik(K.LEGS[leg], planned[-1, leg] - body_arr[ii[-1]], cfg.mic_bars)
                    R_ref[leg] = K.shin_rotation(K.LEGS[leg], q)
                elif name.startswith("unload"):
                    self._last_planted = {i: planned[-1, i].copy() for i in range(4)}
        q_des = np.zeros((n, 12))
        ok = np.ones(n, dtype=bool)
        for i, leg in enumerate(K.LEGS):
            q, o = K.leg_ik(leg, feet_arr[:, i, :] - body_arr, cfg.mic_bars)
            q_des[:, 3 * i:3 * i + 3] = q
            ok &= o
        if not ok.all():
            bad = int(np.argmin(ok))
            return CrawlPlan(False, f"trajectory left the reachable set in phase {names[seg_phase[bad]]}",
                             self.box, cfg, steps)
        com = S.center_of_mass_batch(q_des, cfg.extra_masses) + body_arr
        forces = S.contact_forces(com[:, :2], feet_arr[:, :, :2], w_arr, self.mass)
        comp = cfg.sag_gain * S.sag_compensation(q_des, forces, cfg.kp)
        # a position loop with damping lags a moving target by (kd / kp) * velocity; lead the
        # command by that much so the joints follow the plan instead of trailing it
        # Time base. Samples are cfg.dt apart, stretched wherever a joint would otherwise have
        # to move faster than max_joint_speed (a nearly straight leg lifting its foot needs a
        # large knee movement for a small foot movement).
        step = np.abs(np.diff(q_des, axis=0)).max(axis=1) / cfg.max_joint_speed
        step = np.maximum(cfg.dt, step)
        if len(step) >= 5:                                # widen each slow stretch a little, then smooth its edges
            pad = np.pad(step, 2, mode="edge")
            step = np.max(np.stack([pad[k:k + len(step)] for k in range(5)]), axis=0)
            pad = np.pad(step, 2, mode="edge")
            step = np.maximum(np.mean(np.stack([pad[k:k + len(step)] for k in range(5)]), axis=0), cfg.dt)
        t = cfg.dt + np.concatenate([[0.0], np.cumsum(step)])
        qd_des = np.gradient(q_des, t, axis=0)
        q_cmd = q_des + comp + (cfg.kd / cfg.kp) * qd_des
        lim = np.concatenate([K.leg_model(leg).hw_limits for leg in K.LEGS], axis=0)
        q_cmd = np.clip(q_cmd, lim[:, 0], lim[:, 1])
        margin = np.empty(n)
        for k in range(n):
            stance = [w_arr[k, i] > 0.5 for i in range(4)]
            margin[k] = S.support_margin(com[k, :2], feet_arr[k, :, :2], stance)
        return CrawlPlan(
            True, "", self.box, cfg, steps,
            t=t, q_des=q_des, q_cmd=q_cmd, body=body_arr, feet=feet_arr,
            weights=w_arr, swing_leg=np.array(seg_swing), guard_leg=np.array(seg_guard),
            phase=np.array(seg_phase), phase_names=names, margin=margin,
        )


HIP_HEIGHTS = (0.160, 0.155, 0.165)
TEMPLATE_MARGIN_STEPS = (0.004, 0.0)      # extra balance margin (m) a stored pattern is tried with, in turn
_MIRROR = {"FL": "FR", "FR": "FL", "BL": "BR", "BR": "BL"}
_LAST_GOOD = []                           # the stored pattern the last successful plan used
MAX_TEMPLATES = 20                        # patterns tried per request (nearest boxes first): bounds planning time
TEMPLATE_FILE = Path(__file__).resolve().parent.parent / "config" / "crawl_templates.json"
_TEMPLATES = None


def load_templates(path=None):
    """Footstep templates found offline (sim/build_templates.py). Each is a dict with the
    steps, the crossing height they were found at and the box they were found for."""
    global _TEMPLATES
    if path is None and _TEMPLATES is not None:
        return _TEMPLATES
    p = Path(path) if path else TEMPLATE_FILE
    templates = json.loads(p.read_text())["templates"] if p.exists() else []
    if path is None:
        _TEMPLATES = templates
    return templates


def _as_template(tpl, default_height):
    if isinstance(tpl, dict):
        return tpl
    return {"steps": tpl, "hip_height": default_height, "height": None, "depth": None}


def plan_crossing(box: BoxSpec, cfg: CrawlConfig | None = None, hip_heights=None, templates=None,
                  search: bool = True, max_templates: int | None = MAX_TEMPLATES) -> CrawlPlan:
    """Plan a crossing of ``box`` starting from the default stance. See the module docstring.

    Fast path: footstep templates (sequences found offline, written relative to the box) are
    replayed on this box at the crossing height they were found at, each step checked in
    full; the first that passes is used. Templates found for the most similar box are tried
    first. If none fits and ``search`` is set, a footstep sequence is searched for from
    scratch, which can take a minute. ``hip_heights`` restricts the crossing heights."""
    cfg = cfg or CrawlConfig()
    if box.height < 0 or box.depth < 0:
        return CrawlPlan(False, "box dimensions must be positive", box, cfg)
    if abs(box.skew) > math.radians(12.0):
        return CrawlPlan(False, "box is more than 12 degrees off square; align first", box, cfg)
    templates = [_as_template(t, cfg.hip_height) for t in (load_templates() if templates is None else templates)]

    def distance(t):
        if t["height"] is None:
            return 0.0
        return abs(t["height"] - box.height) / 0.005 + abs(t["depth"] - box.depth) / 0.010

    templates = sorted(templates, key=distance)
    if hip_heights is not None:
        templates = [t for t in templates if any(abs(t["hip_height"] - h) < 1e-6 for h in hip_heights)]
    templates = templates[:max_templates]
    # the pattern that worked last time goes first: the same box is usually crossed again
    templates.sort(key=lambda t: 0 if any(t is g for g in _LAST_GOOD) else 1)
    for tpl in templates:
        # Each pattern is tried as stored and mirrored left-right (the robot is symmetric; on
        # a skewed box one handedness often fits where the other does not), each with a little
        # more balance margin than required first (which is how it was found, and gives the
        # better plan), then with the required margin.
        mirrored = {**tpl, "steps": [[_MIRROR[leg], kind, off] for leg, kind, off in tpl["steps"]], "mirror_of": tpl}
        for variant in (tpl, mirrored):
            for extra in TEMPLATE_MARGIN_STEPS:
                trial = replace(cfg, hip_height=tpl["hip_height"], min_margin=cfg.min_margin + extra)
                plan = _Planner(box, trial).plan_template(variant["steps"])
                if plan is not None and plan.ok:
                    plan.template = variant
                    _LAST_GOOD[:] = [tpl]
                    return plan
    last = CrawlPlan(False, "no stored footstep pattern fits this box", box, cfg)
    if search:
        heights = list(hip_heights) if hip_heights is not None else [cfg.hip_height] + [
            h for h in HIP_HEIGHTS if abs(h - cfg.hip_height) > 1e-6]
        for hh in heights:
            last = _Planner(box, replace(cfg, hip_height=hh)).plan()
            if last.ok:
                return last
    return last


def plan_crawl(distance: float, cfg: CrawlConfig | None = None) -> CrawlPlan:
    """Plan a plain crawl straight ahead by ``distance`` metres on clear floor (used for the
    counted approach steps and for bring-up tests)."""
    cfg = cfg or CrawlConfig()
    if distance < cfg.min_step:
        return CrawlPlan(False, "distance too short for a crawl step", None, cfg)
    return _Planner(BoxSpec(0.0, 0.0, 10.0), cfg).plan_cycles(distance)


def plan_stand_test(cfg: CrawlConfig | None = None, hold: float = 3.0) -> CrawlPlan:
    """Bring-up test: rise to the crossing height, stand for ``hold`` seconds, lower again.
    No foot leaves the ground."""
    cfg = replace(cfg or CrawlConfig(), hold_time=hold)
    planner = _Planner(BoxSpec(0.0, 0.0, 10.0), cfg)
    err = planner.setup(travel=0.0)
    if err:
        return CrawlPlan(False, err, planner.box, cfg)
    return planner.finish([])
