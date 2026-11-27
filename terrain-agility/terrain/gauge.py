"""Measuring the step-over box: height, depth, distance, skew and width from one fisheye
frame, using its two colours (sides one colour, top another).

Per bearing (one column of a fine polar view):
    foot of the front face   -> distance      D = hc / tan(dep_foot)
    top of the side colour   -> height        h = hc * (1 - tan(dep_top) / tan(dep_foot))
    far end of the top colour-> depth         from (hc - h) / tan(dep_far)
The feet of all bearings lie on a line: its offset is the distance to the near face, its
slope the skew; the far edges give a second line and the depth. Height uses a ratio of two
angles, so an error in body pitch nearly cancels; distance does not, so measure it close.

``BoxGauge`` gathers several frames and reports medians with their spread.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .camera_model import PolarView
from .floor_model import ColorRange


@dataclass
class BoxMeasurement:
    near_x: float            # m: near face ahead of the body centre, on the robot's centreline (y = 0)
    skew: float              # rad: + = the near face is farther away on the left
    height: float            # m
    depth: float             # m, fore-aft; nan if the top colour was not seen
    width: float             # m visible
    y_left: float            # m: its ends, body frame (left > right)
    y_right: float
    columns: int             # bearings used
    height_spread: float     # m: scatter of the height between bearings (robust std)
    ends_seen: bool          # both ends of the box are inside the view
    # raw angles straight ahead (bearings within +-6 deg), for tools/calibrate_camera.py
    foot_dep: float = float("nan")   # deg below the horizon: foot of the near face
    top_dep: float = float("nan")    # deg: its top edge
    landing_floor: float = float("nan")   # fraction of the strip beyond the box that looks like floor (nan: not checked)

    @property
    def y_centre(self) -> float:
        return 0.5 * (self.y_left + self.y_right)


@dataclass
class GaugeConfig:
    min_columns: int = 12
    min_side_cells: int = 3          # cells of side colour in a column
    max_gap_cells: int = 3           # mixed-colour cells allowed between side and top
    end_trim_deg: float = 2.0        # bearings ignored at each end of the box
    max_range: float = 0.9           # m: farther than this a height reading is not trusted


def fine_view(model, pose, bearing_span=(-55.0, 55.0), depression_span=(4.0, 68.0)) -> PolarView:
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


def measure_box(image_rgb, view: PolarView, side: ColorRange, top: ColorRange, R_level_body=None,
                camera_height=None, cfg: GaugeConfig | None = None, floor=None, landing: float = 0.35):
    """One frame -> BoxMeasurement, or None if the box's side colour is not clearly in view.

    With a ``floor`` model (floor_model.FloorModel) it also checks the landing strip: the
    ``landing`` metres of floor beyond the box, as far as the camera can see over the box."""
    import cv2

    cfg = cfg or GaugeConfig()
    hc = view.pose.height if camera_height is None else camera_height
    pol, valid = view.sample(image_rgb, R_level_body)
    hsv = cv2.cvtColor(pol, cv2.COLOR_RGB2HSV)
    side_m = side.mask(hsv) & valid
    top_m = top.mask(hsv) & valid & ~side_m
    kernel = np.ones((3, 3), np.uint8)
    side_m = cv2.morphologyEx(side_m.astype(np.uint8), cv2.MORPH_OPEN, kernel).astype(bool)
    dep = view.depressions
    res = dep[1] - dep[0]
    n_dep, n_az = side_m.shape

    cols, d_foot, d_top, d_far = [], [], [], []
    for j in range(n_az):
        rows = np.nonzero(side_m[:, j])[0]
        if len(rows) < cfg.min_side_cells:
            continue
        base = rows[-1]                               # nearest side cell (largest depression)
        if base >= n_dep - 2:                         # cut off by the bottom of the view
            continue
        # the run of side colour ending at the foot
        k = len(rows) - 1
        while k > 0 and rows[k - 1] == rows[k] - 1:
            k -= 1
        s_top = rows[k]
        if base - s_top + 1 < cfg.min_side_cells:
            continue
        # top colour directly above (allowing a few mixed cells at the edge)
        far = None
        r = s_top - 1
        gap = 0
        while r >= 0 and not top_m[r, j] and gap < cfg.max_gap_cells:
            r -= 1
            gap += 1
        if r >= 0 and top_m[r, j]:
            first_top = r
            while r > 0 and top_m[r - 1, j]:
                r -= 1
            far = r
            # the side/top boundary sits in the middle of any mixed cells between them
            d_top.append(0.5 * ((dep[s_top] - 0.5 * res) + (dep[first_top] + 0.5 * res)))
        else:
            d_top.append(dep[s_top] - 0.5 * res)
        cols.append(j)
        d_foot.append(dep[base] + 0.5 * res)
        d_far.append(dep[far] - 0.5 * res if far is not None else np.nan)
    if len(cols) < cfg.min_columns:
        return None
    cols = np.array(cols)
    # keep the widest run of adjacent bearings (the box, not a stray patch of its colour)
    breaks = np.nonzero(np.diff(cols) > 2)[0]
    starts = np.concatenate([[0], breaks + 1])
    ends = np.concatenate([breaks, [len(cols) - 1]])
    i_best = int(np.argmax(ends - starts))
    sel = slice(starts[i_best], ends[i_best] + 1)
    cols = cols[sel]
    d_foot, d_top, d_far = np.array(d_foot)[sel], np.array(d_top)[sel], np.array(d_far)[sel]
    if len(cols) < cfg.min_columns:
        return None
    az = view.bearings[cols]
    ends_seen = bool(cols[0] > 0 and cols[-1] < n_az - 1)

    rng = hc / np.tan(np.radians(d_foot))
    if np.median(rng) > cfg.max_range:
        return None
    x, y = view.to_body_xy(az, rng)
    y_right, y_left = float(y[0]), float(y[-1])
    width = float(math.hypot(x[-1] - x[0], y[-1] - y[0]))
    trim = int(round(cfg.end_trim_deg / (view.bearings[1] - view.bearings[0])))
    inner = slice(trim, len(cols) - trim) if len(cols) - 2 * trim >= cfg.min_columns else slice(None)
    a, b = _fit_line(x[inner], y[inner])
    skew = math.atan(b)

    h_cols = hc * (1.0 - np.tan(np.radians(d_top[inner])) / np.tan(np.radians(d_foot[inner])))
    height = float(np.median(h_cols))
    spread = float(1.4826 * np.median(np.abs(h_cols - height)))

    depth = float("nan")
    far_ok = np.isfinite(d_far[inner])
    if far_ok.sum() >= cfg.min_columns // 2 and height < hc:
        r_far = (hc - height) / np.tan(np.radians(d_far[inner][far_ok]))
        xf, yf = view.to_body_xy(az[inner][far_ok], r_far)
        a_far = float(np.median(xf - b * yf))
        depth = (a_far - a) * math.cos(skew)
    landing_floor = float("nan")
    if floor is not None and math.isfinite(depth) and far_ok.sum() >= cfg.min_columns // 2:
        floor_m = floor.is_floor(hsv) & valid
        fracs = []
        for j, d_edge, r_face in zip(cols[inner][far_ok], d_far[inner][far_ok], r_far):
            # over the far top edge the camera sees the floor from here on, out to the end of the strip
            d_end = math.degrees(math.atan2(hc, r_face + landing))
            rows = np.nonzero((dep < d_edge - 2 * res) & (dep > d_end))[0]
            if len(rows) >= 3:
                fracs.append(float(floor_m[rows, j].mean()))
        if len(fracs) >= cfg.min_columns // 2:
            # the worst bearing counts (a post in the strip is a few bearings wide); a median
            # over 5 neighbours first, so one noisy column does not
            f = np.array(fracs)
            if len(f) >= 5:
                f = np.array([np.median(f[max(0, k - 2):k + 3]) for k in range(len(f))])
            landing_floor = float(f.min())
    ahead = np.abs(az[inner]) < 6.0
    foot_dep = float(np.median(d_foot[inner][ahead])) if ahead.sum() >= 5 else float("nan")
    top_dep = float(np.median(d_top[inner][ahead])) if ahead.sum() >= 5 else float("nan")
    return BoxMeasurement(near_x=a, skew=skew, height=height, depth=depth, width=width, y_left=y_left,
                          y_right=y_right, columns=int(len(cols)), height_spread=spread, ends_seen=ends_seen,
                          foot_dep=foot_dep, top_dep=top_dep, landing_floor=landing_floor)


@dataclass
class BoxEstimate:
    """Medians over several frames, with how much they scattered."""

    near_x: float
    skew: float
    height: float
    depth: float
    width: float
    y_left: float
    y_right: float
    frames: int
    height_sigma: float      # m, frame-to-frame
    near_sigma: float
    ends_seen: bool
    landing_floor: float = float("nan")   # fraction of the strip beyond the box that looks like floor

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

        def med(name):
            v = np.array([getattr(s, name) for s in self.samples], dtype=float)
            v = v[np.isfinite(v)]
            return float(np.median(v)) if len(v) else float("nan")

        def sig(name):
            v = np.array([getattr(s, name) for s in self.samples], dtype=float)
            v = v[np.isfinite(v)]
            return float(1.4826 * np.median(np.abs(v - np.median(v)))) if len(v) > 1 else float("inf")

        # frame-to-frame scatter, but never claim better than the scatter between bearings
        h_sigma = max(sig("height"), med("height_spread") / math.sqrt(max(1.0, med("columns"))))
        return BoxEstimate(near_x=med("near_x"), skew=med("skew"), height=med("height"), depth=med("depth"),
                           width=med("width"), y_left=med("y_left"), y_right=med("y_right"),
                           frames=len(self.samples), height_sigma=h_sigma, near_sigma=sig("near_x"),
                           ends_seen=bool(np.mean([s.ends_seen for s in self.samples]) > 0.5),
                           landing_floor=med("landing_floor"))


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
