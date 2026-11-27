"""Range scan from the classified polar view: for every bearing, how far along the floor the
first obstacle is, and how tall it looks.

Per bearing (one column of the polar view), walking outward from the robot along the floor:
the first run of not-floor cells is the obstacle's foot, and its range is
camera_height / tan(depression). Continuing upward, the obstacle ends where floor shows
again (its top edge, seen against the floor beyond it) or it reaches the horizon, in which
case it is at least as tall as the camera.
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .camera_model import PolarView, height_from_angles
from .floor_model import BALL, BOX_SIDE, BOX_TOP, FLOOR, INVALID, OBSTACLE


@dataclass
class ScanConfig:
    max_range: float = 2.5            # m from the lens; farther obstacles are ignored
    near_depression_deg: float = 60.0  # steeper than this the camera sees the robot itself
    min_run: int = 3                  # cells of not-floor in a row that count as an obstacle
    floor_gap: int = 3                # cells of floor in a row that end an obstacle
    tall_height: float = 0.080        # m: looks at least this tall -> "tall" without stopping
    jump: float = 0.20                # m: range step between neighbouring bearings that splits obstacles
    min_columns: int = 3              # bearings an obstacle must span


@dataclass
class Scan:
    bearings_deg: np.ndarray          # (n,)
    ranges: np.ndarray                # (n,) m from the lens along the floor; inf = clear
    heights: np.ndarray               # (n,) m; inf = reaches the horizon; nan where clear
    base_dep: np.ndarray              # (n,) depression of the obstacle's foot, deg
    top_dep: np.ndarray               # (n,) depression of its top edge, deg (<= 0: at/above horizon)
    kinds: np.ndarray                 # (n,) label of the foot cell (OBSTACLE or BOX_SIDE); 0 where clear
    seen: np.ndarray                  # (n,) the camera can see the floor at this bearing
    x: np.ndarray = None              # (n,) obstacle foot in the body frame (m); nan where clear
    y: np.ndarray = None


@dataclass
class Obstacle:
    left_deg: float                   # bearings of its two ends (left > right)
    right_deg: float
    nearest: float                    # m from the lens
    nearest_bearing_deg: float
    left_xy: tuple                    # its two ends on the floor, body frame
    right_xy: tuple
    width: float                      # m, distance between the ends
    height: float                     # m, apparent (inf = at least camera height)
    tall: bool
    is_box: bool                      # its foot has the step-over box's side colour
    columns: int
    points: np.ndarray = field(default=None, repr=False)   # (k, 2) foot points, body frame


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
        # the foot cell itself is often a blend of floor and box; look just above it too
        first_side = next((r for r in range(base, max(base - 3, 0), -1) if labels[r, j] == BOX_SIDE), None)
        is_box = first_side is not None
        if is_box:
            # the step-over box: its front face ends where its side colour ends
            top = first_side
            while top > 0 and labels[top - 1, j] == BOX_SIDE:
                top -= 1
        else:
            # upward from the foot until floor shows again
            above = np.nonzero(fl_run[:base, j])[0]
            top = above[-1] + 1 if len(above) else 0      # first obstacle row below that floor run
        base_dep[j] = dep[base] + 0.5 * (dep[1] - dep[0])      # lower edge of the foot cell
        top_dep[j] = dep[top] - 0.5 * (dep[1] - dep[0]) if top > 0 else -90.0
        if top <= horizon_row:
            top_dep[j] = min(top_dep[j], 0.0)
        ranges[j] = float(view.ground_range(base_dep[j], hc))
        heights[j] = float(height_from_angles(hc, base_dep[j], top_dep[j]))
        kinds[j] = BOX_SIDE if is_box else OBSTACLE
    x, y = view.to_body_xy(view.bearings, ranges)
    x = np.where(np.isfinite(ranges), x, np.nan)
    y = np.where(np.isfinite(ranges), y, np.nan)
    return Scan(view.bearings.copy(), ranges, heights, base_dep, top_dep, kinds, seen, x, y)


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
            out.append(Obstacle(
                left_deg=float(scan.bearings_deg[k]), right_deg=float(scan.bearings_deg[j]),
                nearest=float(r.min()), nearest_bearing_deg=float(scan.bearings_deg[i_near]),
                left_xy=(float(scan.x[k]), float(scan.y[k])), right_xy=(float(scan.x[j]), float(scan.y[j])),
                width=float(np.hypot(scan.x[k] - scan.x[j], scan.y[k] - scan.y[j])),
                height=height, tall=bool(height >= cfg.tall_height),
                is_box=bool(np.mean(scan.kinds[cols] == BOX_SIDE) > 0.5), columns=len(cols), points=pts))
        j = k + 1
    return out


def scan_points(scan: Scan) -> np.ndarray:
    """Obstacle foot points (k, 2) in the body frame."""
    ok = np.isfinite(scan.ranges)
    return np.stack([scan.x[ok], scan.y[ok]], axis=1)
