"""Range scan from the classified polar view: for every bearing, how far along the floor the
first obstacle is, and how tall it could be.

Per bearing (one column of the polar view), walking outward from the robot along the floor:
the first run of not-floor cells is the obstacle's foot, and its range is
camera_height / tan(depression). Continuing upward, the obstacle ends where floor shows
again (its far edge, seen against the floor beyond it) or it reaches the horizon, in which
case it is at least as tall as the camera.

One camera cannot tell a tall thin thing from a low deep one in a single look: both cover
the same stretch of the picture. What a look does give is two limits. The obstacle is no
taller than a wall standing at its foot that would hide the same floor (``heights``, an
upper limit), and - if it is no deeper than the deepest thing worth crossing - no lower
than ``min_heights``. Something whose lower limit is already too tall to cross is walked
around from a distance; anything else is walked up to and measured (terrain/gauge.py).
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .camera_model import PolarView, height_from_angles
from .floor_model import BALL, BOX_SIDE, BOX_TOP, FLOOR, INVALID, OBSTACLE  # noqa: F401


@dataclass
class ScanConfig:
    max_range: float = 2.5            # m from the lens; farther obstacles are ignored
    near_depression_deg: float = 60.0  # steeper than this the camera sees the robot itself
    min_run: int = 3                  # cells of not-floor in a row that count as an obstacle
    floor_gap: int = 3                # cells of floor in a row that end an obstacle
    tall_height: float = 0.060        # m: certainly at least this tall -> walked around without a closer look
    cross_max_depth: float = 0.70     # m: nothing deeper than this (fore-aft) is ever crossed
    jump: float = 0.20                # m: range step between neighbouring bearings that splits obstacles
    min_columns: int = 3              # bearings an obstacle must span


@dataclass
class Scan:
    bearings_deg: np.ndarray          # (n,)
    ranges: np.ndarray                # (n,) m from the lens along the floor; inf = clear
    heights: np.ndarray               # (n,) m, the most it could be; inf = reaches the horizon; nan where clear
    base_dep: np.ndarray              # (n,) depression of the obstacle's foot, deg
    top_dep: np.ndarray               # (n,) depression of its top edge, deg (<= 0: at/above horizon)
    kinds: np.ndarray                 # (n,) OBSTACLE where there is a hit; 0 where clear
    seen: np.ndarray                  # (n,) the camera can see the floor at this bearing
    x: np.ndarray = None              # (n,) obstacle foot in the body frame (m); nan where clear
    y: np.ndarray = None
    far: np.ndarray = None            # (n,) m from the lens: where the ray over its far edge meets the floor; inf = horizon
    min_heights: np.ndarray = None    # (n,) m, the least it could be if no deeper than cross_max_depth; inf = horizon


@dataclass
class Obstacle:
    left_deg: float                   # bearings of its two ends (left > right)
    right_deg: float
    nearest: float                    # m from the lens
    nearest_bearing_deg: float
    left_xy: tuple                    # its two ends on the floor, body frame
    right_xy: tuple
    width: float                      # m, distance between the ends
    height: float                     # m, the most it could be (inf = at least camera height)
    tall: bool                        # certainly too tall to cross: walk around it
    is_box: bool                      # not certainly tall: it may be low enough to step over or scale across,
    #                                   so it is walked up to and measured (the name is from the two-colour box)
    columns: int
    points: np.ndarray = field(default=None, repr=False)   # (k, 2) foot points, body frame
    min_height: float = 0.0           # m, the least it could be (see the module docstring)
    extent: float = float("nan")      # m, the most it could be deep (the floor it hides, fore-aft); inf = to the horizon


def _erode_down(mask, n):
    """True where a cell and the n-1 cells below it (larger depression) are all True."""
    out = mask.copy()
    for k in range(1, n):
        out[:-k] &= mask[k:]
        out[-k:] = False
    return out


def build_scan(labels: np.ndarray, view: PolarView, cfg: ScanConfig | None = None, camera_height=None) -> Scan:
    """labels: (n_depression, n_bearing) from floor_model.classify on a polar view."""
    cfg = cfg or ScanConfig()
    hc = view.pose.height if camera_height is None else camera_height
    dep = view.depressions
    n_dep, n_az = labels.shape
    rows = np.nonzero((dep > np.degrees(np.arctan2(hc, cfg.max_range))) & (dep <= cfg.near_depression_deg))[0]
    r_lo, r_hi = (rows[0], rows[-1]) if len(rows) else (0, -1)
    obstacle = (labels == OBSTACLE) | (labels == BOX_SIDE) | (labels == BOX_TOP)
    floor = (labels == FLOOR) | (labels == BALL)
    # an obstacle foot: a run of obstacle cells going UP from this row (smaller depression)
    ob_run = obstacle.copy()
    fl_run = floor.copy()
    for k in range(1, cfg.min_run):
        ob_run[k:] &= obstacle[:-k]
        ob_run[:k] = False
    for k in range(1, cfg.floor_gap):
        fl_run[k:] &= floor[:-k]
        fl_run[:k] = False

    ranges = np.full(n_az, np.inf)
    heights = np.full(n_az, np.nan)
    base_dep = np.full(n_az, np.nan)
    top_dep = np.full(n_az, np.nan)
    kinds = np.zeros(n_az, dtype=np.uint8)
    seen = np.zeros(n_az, dtype=bool)
    far = np.full(n_az, np.nan)
    min_heights = np.full(n_az, np.nan)
    horizon_row = int(np.searchsorted(dep, 0.0))
    for j in range(n_az):
        col_valid = labels[r_lo:r_hi + 1, j] != INVALID
        if col_valid.mean() < 0.5:
            continue
        seen[j] = True
        hits = np.nonzero(ob_run[r_lo:r_hi + 1, j])[0]
        if not len(hits):
            continue
        base = r_lo + hits[-1]                     # nearest: largest depression
        # upward from the foot until floor shows again
        above = np.nonzero(fl_run[:base, j])[0]
        top = above[-1] + 1 if len(above) else 0      # first obstacle row below that floor run
        base_dep[j] = dep[base] + 0.5 * (dep[1] - dep[0])      # lower edge of the foot cell
        top_dep[j] = dep[top] - 0.5 * (dep[1] - dep[0]) if top > 0 else -90.0
        if top <= horizon_row:
            top_dep[j] = min(top_dep[j], 0.0)
        ranges[j] = float(view.ground_range(base_dep[j], hc))
        heights[j] = float(height_from_angles(hc, base_dep[j], top_dep[j]))
        far[j] = float(view.ground_range(top_dep[j], hc)) if top_dep[j] > 0.0 else np.inf
        # if it is cross_max_depth deep, its far top edge is that far beyond its foot, and
        # the ray over it tells how high that edge is
        min_heights[j] = hc * (1.0 - (ranges[j] + cfg.cross_max_depth) / far[j]) if np.isfinite(far[j]) else np.inf
        kinds[j] = OBSTACLE
    x, y = view.to_body_xy(view.bearings, ranges)
    x = np.where(np.isfinite(ranges), x, np.nan)
    y = np.where(np.isfinite(ranges), y, np.nan)
    return Scan(view.bearings.copy(), ranges, heights, base_dep, top_dep, kinds, seen, x, y, far, min_heights)


def find_obstacles(scan: Scan, cfg: ScanConfig | None = None) -> list:
    """Group neighbouring bearings with a hit into obstacles (right to left)."""
    cfg = cfg or ScanConfig()
    out = []
    n = len(scan.ranges)
    j = 0
    while j < n:
        if not np.isfinite(scan.ranges[j]):
            j += 1
            continue
        k = j
        while (k + 1 < n and np.isfinite(scan.ranges[k + 1])
               and np.hypot(scan.x[k + 1] - scan.x[k], scan.y[k + 1] - scan.y[k]) < cfg.jump):
            k += 1
        cols = np.arange(j, k + 1)
        if len(cols) >= cfg.min_columns:
            r = scan.ranges[cols]
            h = scan.heights[cols]
            finite = h[np.isfinite(h)]
            frac_inf = float(np.mean(~np.isfinite(h)))
            height = float("inf") if frac_inf > 0.3 else (float(np.percentile(finite, 80)) if len(finite) else float("inf"))
            i_near = cols[int(np.argmin(r))]
            pts = np.stack([scan.x[cols], scan.y[cols]], axis=1)
            if scan.min_heights is None:
                lowest, extent = (float("inf") if not np.isfinite(height) else 0.0), float("nan")
            else:
                lo = scan.min_heights[cols]
                # a low percentile: "certainly tall" should not hang on a few bearings
                lowest = float("inf") if frac_inf > 0.3 else float(np.percentile(lo[np.isfinite(lo)], 30))
                ext = (scan.far[cols] - r)[np.isfinite(scan.far[cols])]
                extent = float("inf") if frac_inf > 0.3 or not len(ext) else float(np.median(ext))
            tall = bool(lowest >= cfg.tall_height)
            out.append(Obstacle(
                left_deg=float(scan.bearings_deg[k]), right_deg=float(scan.bearings_deg[j]),
                nearest=float(r.min()), nearest_bearing_deg=float(scan.bearings_deg[i_near]),
                left_xy=(float(scan.x[k]), float(scan.y[k])), right_xy=(float(scan.x[j]), float(scan.y[j])),
                width=float(np.hypot(scan.x[k] - scan.x[j], scan.y[k] - scan.y[j])),
                height=height, tall=tall, is_box=not tall, columns=len(cols), points=pts,
                min_height=lowest, extent=extent))
        j = k + 1
    return out


def scan_points(scan: Scan) -> np.ndarray:
    """Obstacle foot points (k, 2) in the body frame."""
    ok = np.isfinite(scan.ranges)
    return np.stack([scan.x[ok], scan.y[ok]], axis=1)
