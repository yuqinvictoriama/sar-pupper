"""Measuring an obstacle from close by: where it is, how tall, how deep, how wide and how
square to the robot, from one fisheye frame. No colours are taught: the only thing known
beforehand is what the floor looks like.

Per bearing (one column of a fine polar view), going outward along the floor:
    foot              the first not-floor cells            -> distance  D = hc / tan(dep_foot)
    side face         cells at the foot that do not look   -> height    h = hc * (1 - tan(dep_edge) / tan(dep_foot))
                      like the obstacle's top
    far edge          where floor shows again              -> depth     from (hc - h) / tan(dep_far)

The "top" is whatever colour most of the obstacle has in the picture (seen from above, a low
obstacle shows mostly its top); the side face is the band at its foot that differs from it:
another material, another colour, or the same colour in shadow. A height is reported only if
that band is there on most bearings and gives the same height on them.

Without a side face the height cannot be read from one look, but two limits still hold: the
obstacle is no taller than ``height_upper`` (a wall at its foot hiding the same floor) and no
deeper than ``extent`` (a flat mark covering the same floor). A small round or one-colour
thing is known well enough from those two alone.

The feet of all bearings lie on a line: its offset is the distance to the near face, its
slope the skew. Height uses a ratio of two angles, so an error in body pitch nearly cancels;
distance does not, so measure it close.

``BoxGauge`` gathers several frames and reports medians with their spread.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .camera_model import PolarView


@dataclass
class BoxMeasurement:
    near_x: float            # m: near face ahead of the body centre, on the robot's centreline (y = 0)
    skew: float              # rad: + = the near face is farther away on the left
    height: float            # m; nan if no side face was seen (then see height_upper)
    depth: float             # m, fore-aft; nan if the height or the far edge was not seen
    width: float             # m visible
    y_left: float            # m: its ends, body frame (left > right)
    y_right: float
    columns: int             # bearings used
    height_spread: float     # m: scatter of the height between bearings (robust std)
    ends_seen: bool          # both ends of the obstacle are inside the view
    # raw angles straight ahead (bearings within +-6 deg), for tools/calibrate_camera.py
    foot_dep: float = float("nan")   # deg below the horizon: foot of the near face
    top_dep: float = float("nan")    # deg: its top edge
    landing_floor: float = float("nan")   # fraction of the strip beyond it that looks like floor (nan: not checked)
    height_upper: float = float("inf")    # m: it is no taller than this (inf: it reaches the top of the view)
    extent: float = float("inf")          # m: it is no deeper than this, fore-aft (inf: far edge not seen)
    side_seen: float = 0.0                # fraction of the bearings on which a side face was seen
    far_seen: bool = True                 # its far edge is in view (it is lower than the camera)

    @property
    def y_centre(self) -> float:
        return 0.5 * (self.y_left + self.y_right)


@dataclass
class GaugeConfig:
    min_columns: int = 12
    min_run: int = 3                 # cells of not-floor in a row that start the obstacle
    floor_gap: int = 4               # cells of floor in a row that end it
    min_side_cells: int = 3          # cells of side face a column needs for a height reading (0.2 deg each)
    side_fraction: float = 0.6       # share of the bearings that must show a side face for the height to count
    top_sample: float = 0.6          # the far part of the obstacle, this share of it, is taken as "the top"
    colour_min: float = 16.0         # CIELAB distance from the top colour that always counts as "not the top"
    colour_k: float = 1.6            # ... or this many times the top's own spread (its 95th percentile)
    end_trim_deg: float = 2.0        # bearings ignored at each end of the obstacle
    max_range: float = 0.9           # m: farther than this a reading is not trusted
    lane_deg: float = 20.0           # the obstacle measured is the nearest one within this of straight ahead
    far_edge_deg: float = 12.0       # bearings within this of straight ahead are used for the far edge
    floor_patch_cells: int = 20      # the nearest strip of floor in view, this many cells (4 deg), is "this floor"
    floor_min: float = 10.0          # CIELAB distance from that strip's colour that always counts as "not this floor"
    floor_k: float = 2.0             # ... or this many times the strip's own spread
    jump: float = 0.08               # m: range step between neighbouring bearings that splits obstacles


def fine_view(model, pose, bearing_span=(-70.0, 70.0), depression_span=(4.0, 68.0)) -> PolarView:
    """The polar view the gauge uses: 0.5 deg in bearing, 0.2 deg in depression (the lens
    gives about 0.25 deg per pixel at full resolution)."""
    return PolarView(model, pose, (bearing_span[0], bearing_span[1], 0.5), (depression_span[0], depression_span[1], 0.2))


def _fit_line(x, y):
    """x = a + b * y, least squares with one round of outlier trimming. -> a, b"""
    A = np.stack([np.ones_like(y), y], axis=1)
    coef, *_ = np.linalg.lstsq(A, x, rcond=None)
    res = x - A @ coef
    mad = np.median(np.abs(res - np.median(res))) + 1e-6
    keep = np.abs(res - np.median(res)) < 3.0 * 1.4826 * mad + 0.002
    if keep.sum() >= 4:
        coef, *_ = np.linalg.lstsq(A[keep], x[keep], rcond=None)
    return float(coef[0]), float(coef[1])


def _run_up(mask, n):
    """True where a cell and the n-1 cells above it (smaller depression) are all True."""
    out = mask.copy()
    for k in range(1, n):
        out[k:] &= mask[:-k]
        out[:k] = False
    return out


def measure_obstacle(image_rgb, view: PolarView, floor, R_level_body=None, camera_height=None,
                     cfg: GaugeConfig | None = None, landing: float = 0.35, ball=None,
                     near_depression_deg: float | None = None, info: dict | None = None):
    """One frame -> BoxMeasurement of the nearest obstacle straight ahead, or None if there is
    none within range whose foot is in view.

    floor: floor_model.FloorModel (trained). ball: an optional floor_model.ColorRange that is
    never an obstacle. ``landing``: metres of floor beyond the obstacle to check for clear floor."""
    import cv2

    cfg = cfg or GaugeConfig()
    hc = view.pose.height if camera_height is None else camera_height
    pol, valid = view.sample(image_rgb, R_level_body)
    hsv = cv2.cvtColor(pol, cv2.COLOR_RGB2HSV)
    floor_m = floor.is_floor(hsv) & valid
    if ball is not None:
        floor_m |= ball.mask(hsv) & valid
    ob = valid & ~floor_m
    ob = cv2.morphologyEx(ob.astype(np.uint8), cv2.MORPH_OPEN, np.ones((3, 3), np.uint8)).astype(bool)
    dep = view.depressions
    res = dep[1] - dep[0]
    n_dep, n_az = ob.shape
    r_near = n_dep - 1 if near_depression_deg is None else int(np.searchsorted(dep, near_depression_deg, side="right")) - 1
    r_near = max(0, min(n_dep - 1, r_near))

    # per bearing: the foot (nearest run of not-floor) and the far edge (where floor shows again)
    ob_run = _run_up(ob, cfg.min_run)
    fl_run = _run_up(floor_m, cfg.floor_gap)
    base = np.full(n_az, -1)
    far = np.full(n_az, -1)                 # row of the farthest obstacle cell; -1: reaches the top of the view
    for j in range(n_az):
        hits = np.nonzero(ob_run[:r_near + 1, j])[0]
        if not len(hits):
            continue
        b = int(hits[-1])
        if b >= r_near or not floor_m[b + 1:r_near + 1, j].any():
            continue                        # cut off at the bottom of the view: its foot is not seen
        base[j] = b
        above = np.nonzero(fl_run[:b, j])[0]
        far[j] = int(above[-1]) + 1 if len(above) else -1
    cols = np.nonzero(base >= 0)[0]
    if len(cols) < cfg.min_columns:
        return None
    lab = cv2.cvtColor(pol, cv2.COLOR_RGB2LAB).astype(np.float32)

    # The foot, looked at again. The floor model answers a general question (does this cell
    # look like any of the floor it was taught?), and a side face in a floor-like colour - a
    # raw wooden edge on a wooden floor - can pass for floor. Right here a sharper question
    # can be asked: does the cell look like the floor the robot is standing in front of? The
    # nearest strip of floor in view, right at the robot's feet, is the sample. Cells at the
    # obstacle's foot that do not look like it are part of the obstacle, and the foot moves
    # down to them.
    p_lo = max(0, r_near - cfg.floor_patch_cells + 1)
    clear = base[cols] < p_lo - 2                       # (bearings on which that strip is in front of the obstacle)
    patch = lab[p_lo:r_near + 1][:, cols[clear]][floor_m[p_lo:r_near + 1][:, cols[clear]]] if clear.any() else np.zeros((0, 3))
    if len(patch) >= 200:
        centre_f = np.median(patch, axis=0)
        limit_f = max(cfg.floor_min, cfg.floor_k * float(np.percentile(np.linalg.norm(patch - centre_f, axis=1), 95)))
        for j in cols[clear]:
            r, misses, b = base[j] + 1, 0, base[j]
            while r < p_lo and misses < 2:
                if np.linalg.norm(lab[r, j] - centre_f) > limit_f:
                    b, misses = r, 0
                else:
                    misses += 1
                r += 1
            base[j] = b
    d_foot_all = dep[base[cols]] + 0.5 * res
    rng_all = hc / np.tan(np.radians(d_foot_all))
    x_all, y_all = view.to_body_xy(view.bearings[cols], rng_all)

    # split into obstacles (neighbouring bearings whose feet are close) and take the nearest one ahead
    groups, start = [], 0
    for i in range(1, len(cols) + 1):
        if i == len(cols) or cols[i] - cols[i - 1] > 2 or math.hypot(x_all[i] - x_all[i - 1], y_all[i] - y_all[i - 1]) > cfg.jump:
            if i - start >= cfg.min_columns:
                groups.append((start, i))
            start = i
    if not groups:
        return None
    ahead = [g for g in groups if np.abs(view.bearings[cols[g[0]:g[1]]]).min() <= cfg.lane_deg]
    g0, g1 = min(ahead or groups, key=lambda g: float(rng_all[g[0]:g[1]].min()))
    cols = cols[g0:g1]
    if float(np.median(rng_all[g0:g1])) > cfg.max_range:
        return None
    d_foot = dep[base[cols]] + 0.5 * res
    rng = hc / np.tan(np.radians(d_foot))
    x, y = view.to_body_xy(view.bearings[cols], rng)
    az = view.bearings[cols]
    ends_seen = bool(cols[0] > 0 and cols[-1] < n_az - 1)
    y_right, y_left = float(y[0]), float(y[-1])
    width = float(math.hypot(x[-1] - x[0], y[-1] - y[0]))
    trim = int(round(cfg.end_trim_deg / (view.bearings[1] - view.bearings[0])))
    inner = slice(trim, len(cols) - trim) if len(cols) - 2 * trim >= cfg.min_columns else slice(None)
    a, b_slope = _fit_line(x[inner], y[inner])
    skew = math.atan(b_slope)

    # the top's colour: the far part of the obstacle on every bearing
    b_rows, f_rows = base[cols], np.where(far[cols] >= 0, far[cols], 0)
    sample = []
    quarter = len(cols) // 4        # (the middle half of its bearings: towards its ends the end faces come into view)
    for j, br, fr in list(zip(cols, b_rows, f_rows))[quarter:len(cols) - quarter]:
        hi = fr + max(1, int(cfg.top_sample * (br - fr)))
        sample.append(lab[fr:hi, j][ob[fr:hi, j]])
    sample = np.concatenate(sample) if sample else np.zeros((0, 3), np.float32)
    side_rows = np.full(len(cols), -1)
    if len(sample) >= 30:
        centre = np.median(sample, axis=0)
        spread = float(np.percentile(np.linalg.norm(sample - centre, axis=1), 95))
        limit = max(cfg.colour_min, cfg.colour_k * spread)
        for i, (j, br, fr) in enumerate(zip(cols, b_rows, f_rows)):
            # up from the foot: cells that are not the top's colour, until two in a row are
            unlike = np.linalg.norm(lab[fr:br + 1, j] - centre, axis=1) > limit
            r = len(unlike) - 1
            while r >= 0 and (unlike[r] or (r >= 1 and unlike[r - 1])):
                r -= 1
            n_side = len(unlike) - 1 - r
            if cfg.min_side_cells <= n_side < len(unlike) - 2:
                side_rows[i] = br - n_side + 1           # the top-most side cell
    has_side = side_rows >= 0
    side_seen = float(np.mean(has_side[inner]))
    if info is not None:            # for tools/learn_colors.py check: what was taken for what, per bearing
        info.update(columns=cols, foot_rows=b_rows, far_rows=far[cols], side_rows=side_rows, samples=len(sample),
                    top_limit=(limit if len(sample) >= 30 else None))
    d_edge = np.where(has_side, dep[np.maximum(side_rows, 0)] - 0.5 * res, np.nan)

    height, spread_h = float("nan"), float("nan")
    if side_seen >= cfg.side_fraction:
        ok = has_side[inner]
        h_cols = hc * (1.0 - np.tan(np.radians(d_edge[inner][ok])) / np.tan(np.radians(d_foot[inner][ok])))
        height = float(np.median(h_cols))
        spread_h = float(1.4826 * np.median(np.abs(h_cols - height)))

    # the far edge: the limits it gives, and (with the height known) the depth
    far_ok = (far[cols] >= 0)[inner]
    far_seen = bool(far_ok.sum() >= max(cfg.min_columns // 2, 0.7 * len(far_ok)))
    depth, extent, height_upper, landing_floor = float("nan"), float("inf"), float("inf"), float("nan")
    if far_seen:
        d_far = dep[far[cols][inner][far_ok]] - 0.5 * res
        az_f, rng_f = az[inner][far_ok], rng[inner][far_ok]
        # Only bearings near straight ahead look over the far edge: to either side the line of
        # sight leaves the top over one of its ends instead, which is nearer.
        mid = np.abs(az_f) <= cfg.far_edge_deg
        if mid.sum() < cfg.min_columns // 2:
            mid = np.ones(len(az_f), dtype=bool)
        r_flat = hc / np.tan(np.radians(d_far))                 # where the ray over the far edge meets the floor
        xf, yf = view.to_body_xy(az_f, r_flat)
        extent = float((np.median((xf - b_slope * yf)[mid]) - a) * math.cos(skew))
        height_upper = float(np.median((hc * (1.0 - rng_f / r_flat))[mid]))
        h_used = height if math.isfinite(height) and height < hc else 0.0
        r_far = (hc - h_used) / np.tan(np.radians(d_far))
        if math.isfinite(height):
            xf, yf = view.to_body_xy(az_f, r_far)
            depth = float((np.median((xf - b_slope * yf)[mid]) - a) * math.cos(skew))
        fracs = []
        for j, d_e, r_face in zip(cols[inner][far_ok], d_far, r_far):
            # over the far edge the camera sees the floor from here on, out to the end of the strip
            d_end = math.degrees(math.atan2(hc, r_face + landing))
            rows = np.nonzero((dep < d_e - 2 * res) & (dep > d_end))[0]
            if len(rows) >= 3:
                fracs.append(float(floor_m[rows, j].mean()))
        if len(fracs) >= cfg.min_columns // 2:
            # the worst bearing counts (a post in the strip is a few bearings wide); a median
            # over 5 neighbours first, so one noisy column does not
            f = np.array(fracs)
            if len(f) >= 5:
                f = np.array([np.median(f[max(0, k - 2):k + 3]) for k in range(len(f))])
            landing_floor = float(f.min())

    straight = np.abs(az[inner]) < 6.0
    foot_dep = float(np.median(d_foot[inner][straight])) if straight.sum() >= 5 else float("nan")
    edge_ok = straight & has_side[inner]
    top_dep = float(np.median(d_edge[inner][edge_ok])) if edge_ok.sum() >= 5 else float("nan")
    return BoxMeasurement(near_x=a, skew=skew, height=height, depth=depth, width=width, y_left=y_left,
                          y_right=y_right, columns=int(len(cols)), height_spread=spread_h, ends_seen=ends_seen,
                          foot_dep=foot_dep, top_dep=top_dep, landing_floor=landing_floor,
                          height_upper=height_upper, extent=extent, side_seen=side_seen, far_seen=far_seen)


@dataclass
class BoxEstimate:
    """Medians over several frames, with how much they scattered."""

    near_x: float
    skew: float
    height: float            # nan: not read (no side face seen)
    depth: float             # nan: not read
    width: float
    y_left: float
    y_right: float
    frames: int
    height_sigma: float      # m, frame-to-frame
    near_sigma: float
    ends_seen: bool
    landing_floor: float = float("nan")   # fraction of the strip beyond it that looks like floor
    height_upper: float = float("inf")    # m: it is no taller than this
    extent: float = float("inf")          # m: it is no deeper than this
    side_seen: float = 0.0                # typical fraction of the bearings showing a side face
    far_seen: bool = True                 # lower than the camera: its far edge is in view

    @property
    def y_centre(self) -> float:
        return 0.5 * (self.y_left + self.y_right)


class BoxGauge:
    """Collects per-frame measurements while the robot stands still."""

    def __init__(self, frames_needed: int = 10, max_frames: int = 25):
        self.frames_needed = frames_needed
        self.max_frames = max_frames
        self.samples: list[BoxMeasurement] = []
        self.misses = 0

    def reset(self):
        self.samples.clear()
        self.misses = 0

    def add(self, m: BoxMeasurement | None):
        if m is None:
            self.misses += 1
            return
        self.samples.append(m)
        if len(self.samples) > self.max_frames:
            self.samples.pop(0)

    @property
    def ready(self) -> bool:
        return len(self.samples) >= self.frames_needed

    def estimate(self) -> BoxEstimate | None:
        if not self.samples:
            return None

        def values(name):
            v = np.array([getattr(s, name) for s in self.samples], dtype=float)
            return v[np.isfinite(v)]

        def med(name, most=False):
            """Median of the frames that have the value; with ``most``, nan unless most frames do."""
            v = values(name)
            if not len(v) or (most and len(v) < 0.6 * len(self.samples)):
                return float("nan")
            return float(np.median(v))

        def sig(name):
            v = values(name)
            return float(1.4826 * np.median(np.abs(v - np.median(v)))) if len(v) > 1 else float("inf")

        def upper(name):
            v = np.array([getattr(s, name) for s in self.samples], dtype=float)
            return float(np.median(v)) if len(v) else float("inf")      # inf when most frames say inf

        height = med("height", most=True)
        # frame-to-frame scatter, but never claim better than the scatter between bearings
        h_sigma = max(sig("height"), med("height_spread") / math.sqrt(max(1.0, med("columns")))) if math.isfinite(height) else float("inf")
        return BoxEstimate(near_x=med("near_x"), skew=med("skew"), height=height,
                           depth=med("depth", most=True) if math.isfinite(height) else float("nan"),
                           width=med("width"), y_left=med("y_left"), y_right=med("y_right"),
                           frames=len(self.samples), height_sigma=h_sigma, near_sigma=sig("near_x"),
                           ends_seen=bool(np.mean([s.ends_seen for s in self.samples]) > 0.5),
                           landing_floor=med("landing_floor"), height_upper=upper("height_upper"),
                           extent=upper("extent"), side_seen=med("side_seen"),
                           far_seen=bool(np.mean([s.far_seen for s in self.samples]) > 0.5))


def solve_camera_mount(observations, box_height: float, x_forward: float, solve_x_forward: bool = False,
                       height0: float = 0.25):
    """Camera height and pitch correction (and, optionally, how far the lens is ahead of the
    body centre) from the box seen square-on at known distances.

    observations: list of (distance from the BODY CENTRE to the box's near face (m),
    depression of the face's foot (deg), depression of its top edge (deg)), the angles as
    measured straight ahead with the current pose assumption (BoxMeasurement.foot_dep,
    .top_dep). Least squares on the angles:

        foot_i + c = atan(hc / (D_i - xf))          top_i + c = atan((hc - h) / (D_i - xf))

    Two distances are enough for hc and c. ``solve_x_forward`` also fits xf, which needs
    three or more distances spread over at least 20 cm; otherwise measure xf with a ruler.

    Returns (camera_height, pitch_correction_deg, x_forward, rms_deg): the last is the fit
    residual - a few tenths of a degree when the distances and box height are right.
    """
    obs = np.asarray(observations, dtype=float)
    if len(obs) < 2 or np.ptp(obs[:, 0]) < 0.08:
        raise ValueError("needs the box at two or more distances at least 8 cm apart")
    if solve_x_forward and (len(obs) < 3 or np.ptp(obs[:, 0]) < 0.20):
        raise ValueError("fitting the lens position needs three or more distances spread over 20 cm")
    D, foot, top = obs[:, 0], obs[:, 1], obs[:, 2]
    use_top = np.isfinite(top)

    def residual(p):
        hc, c, xf = p
        r = D - xf
        res = [np.degrees(np.arctan2(hc, r)) - (foot + c)]
        if use_top.any():
            res.append((np.degrees(np.arctan2(hc - box_height, r)) - (top + c))[use_top])
        return np.concatenate(res)

    p = np.array([height0, 0.0, x_forward], dtype=float)
    free = [0, 1, 2] if solve_x_forward else [0, 1]
    for _ in range(40):
        r0 = residual(p)
        J = np.zeros((len(r0), len(free)))
        for k, idx in enumerate(free):
            dp = np.zeros(3)
            dp[idx] = 1e-5
            J[:, k] = (residual(p + dp) - r0) / 1e-5
        step, *_ = np.linalg.lstsq(J, -r0, rcond=None)
        p[free] += step
        if np.abs(step).max() < 1e-9:
            break
    hc, c, xf = (float(v) for v in p)
    if not (0.10 < hc < 0.45 and abs(c) < 30.0 and 0.0 < xf < 0.30):
        raise ValueError("no sensible solution; check the distances, the box height and the colours")
    return hc, c, xf, float(np.sqrt(np.mean(residual(p) ** 2)))
