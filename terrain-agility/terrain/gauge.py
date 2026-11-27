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
    far_dep: float = float("nan")    # deg: the far edge of its top
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
    # A side face in a colour the floor model passes as floor (a raw wooden edge on a wooden floor):
    # found by comparing the cells under the obstacle with the floor just beyond them.
    band_height: float = 0.070       # m: the tallest side face looked for this way
    floor_ref_deg: float = 2.5       # that much floor beyond the foot is "this floor"
    chroma_min: float = 8.0          # CIELAB a/b distance from it that always counts as "not this floor"
    chroma_k: float = 2.0            # ... or this many times that floor's own scatter (its 90th percentile)
    light_min: float = 25.0          # the same for lightness, which glare on a shiny floor moves a lot
    light_k: float = 3.0
    face_tol: float = 12.0           # CIELAB: cells of one side face are within this of each other
    shadow_drop: float = 10.0        # a shadow at the foot: darker than the floor beyond it by this much (L)...
    shadow_chroma: float = 20.0      # ... while within this of its colour (a/b)
    shadow_max_deg: float = 1.6      # and no thicker than this; it is not part of the obstacle
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


def _floor_reference(lab, floor_ok, lo, hi, j, spread=3):
    """The floor's colour in rows lo..hi-1 around bearing j: (median Lab, scatter of its a/b,
    scatter of its L), or None if too little floor shows there."""
    j0, j1 = max(0, j - spread), min(lab.shape[1], j + spread + 1)
    lo, hi = max(0, lo), min(lab.shape[0], hi)
    if hi - lo < 4:
        return None
    cells = lab[lo:hi, j0:j1][floor_ok[lo:hi, j0:j1]]
    if len(cells) < 0.5 * (hi - lo) * (j1 - j0):
        return None
    centre = np.median(cells, axis=0)
    d = cells - centre
    k = int(0.9 * (len(cells) - 1))             # (the 90th percentile; np.percentile is slow for many small calls)
    return centre, float(np.partition(np.hypot(d[:, 1], d[:, 2]), k)[k]), float(np.partition(np.abs(d[:, 0]), k)[k])


def _face_end(cells, ref, cfg) -> int:
    """cells: Lab going down from just under an obstacle. ref: _floor_reference of the floor
    beyond. -> index of the last cell of a side face in them, or -1 if there is none."""
    centre, s_ab, s_l = ref
    lim_ab, lim_l = max(cfg.chroma_min, cfg.chroma_k * s_ab), max(cfg.light_min, cfg.light_k * s_l)
    d = cells - centre
    if len(d) >= 2 and all(math.hypot(d[i, 1], d[i, 2]) <= lim_ab and abs(d[i, 0]) <= lim_l for i in (0, 1)):
        return -1                   # floor straight away, as under most things: nothing more to look at
    unlike = (np.hypot(d[:, 1], d[:, 2]) > lim_ab) | (np.abs(d[:, 0]) > lim_l)
    end, misses = -1, 0
    for i, u in enumerate(unlike.tolist()):
        if u:
            end, misses = i, 0
        else:
            misses += 1
            if misses >= 2:
                break
    else:
        return -1                   # unlike the floor all the way down: no foot inside the window
    cells = cells[:end + 1]
    if len(cells) < 3:
        return -1                   # a cell or two: the blur at the obstacle's edge, not a face
    # the even-coloured part: from the first three cells that agree, for as long as they keep to it
    near = np.linalg.norm(cells[:, None, :] - cells[None, :, :], axis=2) <= cfg.face_tol
    starts = [i for i in range(len(cells) - 2) if near[i, i + 1] and near[i, i + 2] and near[i + 1, i + 2]]
    if not starts:
        return -1
    colour = np.median(cells[starts[0]:starts[0] + 3], axis=0)
    if math.hypot(colour[1] - centre[1], colour[2] - centre[2]) <= 1.5 * lim_ab and abs(colour[0] - centre[0]) <= lim_l:
        return -1                   # that even colour is the floor's own: the fading edge of a shadow, not a face
    like = np.linalg.norm(cells - colour, axis=1) <= cfg.face_tol
    end, i = starts[0] + 2, starts[0] + 3
    while i < len(cells) and (like[i] or (i + 1 < len(cells) and like[i + 1])):
        end, i = (i if like[i] else end), i + 1
    return end


def _side_bands(ob, floor_m, lab, dep, hc, r_near, cfg):
    """Cells under an obstacle that the floor model passed as floor but that are part of it.

    The floor model answers a general question (does this cell look like any floor it was
    taught?), and a side face in a floor-like colour - a raw wooden edge on a wooden floor -
    passes. Under an obstacle a sharper question can be asked: do these cells look like the
    floor just beyond them, at the same viewing angle and in the same light? Going down from
    the obstacle, the cells that do not are a side face, followed perhaps by its shadow or,
    on a shiny floor, its mirror image. The face is the part in one even colour (a face is a
    flat surface in one light; a shadow's edge and a reflection fade), so it ends where that
    colour does. It counts if it ends in floor within the tallest side face looked for, and
    forms a strip along the obstacle rather than a patch on one bearing.

    Returns (the mask with those cells added, the foot's row on each bearing where a face was
    found this way, else -1).
    """
    import cv2

    n_dep, n_az = ob.shape
    res = dep[1] - dep[0]
    w = max(4, int(round(cfg.floor_ref_deg / res)))
    # the foot's row, if this row were the top edge of a side face so tall: first for a face
    # low enough to step on (its floor reference is then close by, in the same light), then
    # for the tallest looked for
    t = np.tan(np.radians(np.clip(dep, 0.5, 89.0)))
    lasts = [np.searchsorted(dep, np.degrees(np.arctan(t * hc / max(hc - bh, 0.02))))
             for bh in (min(0.035, cfg.band_height), cfg.band_height)]
    r_first = int(np.searchsorted(dep, math.degrees(math.atan2(hc, 1.5 * cfg.max_range))))
    floor_ok = floor_m & ~ob
    bottoms = _run_up(ob, cfg.min_run)[:-1] & ~ob[1:]
    band = np.zeros_like(ob)
    refs = {}
    k_shadow = int(round(cfg.shadow_max_deg / res))
    done = np.full(n_az, -1)        # per bearing: the last row already accounted for (a face and what lies at its foot)
    for r, j in sorted(zip(*np.nonzero(bottoms)), key=lambda rj: (rj[1], rj[0])):
        if r < r_first or r <= done[j]:
            continue
        for last in lasts:
            m = min(int(last[r]), r_near - w)       # (close up, as much of that window as is in view)
            if m <= r + 2:
                break
            key = (m // 3, j // 4)                 # (one reference serves the bottoms next to each other)
            if key not in refs:
                refs[key] = _floor_reference(lab, floor_ok, m + 1, m + 1 + w, 4 * (j // 4) + 2)
            end = -1 if refs[key] is None else _face_end(lab[r + 1:m + 1, j], refs[key], cfg)
            if end >= 0:
                band[r + 1:r + 2 + end, j] = True
                done[j] = r + 1 + end + k_shadow
                break
    foot = np.full(n_az, -1)
    if not band.any():
        return ob, foot
    n, labels, stats, _ = cv2.connectedComponentsWithStats(cv2.dilate(band.astype(np.uint8), np.ones((1, 5), np.uint8)),
                                                           connectivity=8)
    keep = np.zeros(n, dtype=bool)
    keep[1:] = stats[1:, cv2.CC_STAT_WIDTH] >= cfg.min_columns + 4
    band &= keep[labels]
    has = band.any(axis=0)
    foot[has] = (n_dep - 1 - np.argmax(band[::-1], axis=0))[has]
    return ob | band, foot


def _drop_narrow(ob, min_columns):
    """The obstacle mask without the patches narrower than ``min_columns`` bearings: glare on
    a shiny floor, specks. They are too small to measure, and left in they would be taken for
    the foot of whatever stands behind them."""
    import cv2

    n, labels, stats, _ = cv2.connectedComponentsWithStats(ob.astype(np.uint8), connectivity=8)
    keep = np.zeros(n, dtype=bool)
    keep[1:] = stats[1:, cv2.CC_STAT_WIDTH] >= min_columns
    return keep[labels]


def _trim_shadow(base, cols, lab, floor_ok, r_near, cfg, res, face_foot):
    """Move each foot up past the shadow lying at it: a thin strip that is darker than both
    the floor beyond it and the obstacle above it, in the floor's own colour. Where
    _side_bands found the face's foot, anything thin just under it is shadow or reflection."""
    w = max(4, int(round(cfg.floor_ref_deg / res)))
    k_max = int(round(cfg.shadow_max_deg / res))
    for j in cols:
        b = int(base[j])
        if face_foot[j] >= 0:
            if 0 <= b - face_foot[j] <= k_max:
                base[j] = face_foot[j]
            continue
        ref = _floor_reference(lab, floor_ok, b + 2, min(b + 2 + w, r_near + 1), j)
        if ref is None:
            continue
        centre = ref[0]
        k = 0
        while k <= k_max and b - k - 3 >= 0:
            c = lab[b - k, j]
            if c[0] < centre[0] - cfg.shadow_drop and math.hypot(c[1] - centre[1], c[2] - centre[2]) < cfg.shadow_chroma:
                k += 1
            else:
                break
        if k == 0 or k > k_max:
            continue                # no shadow, or a dark face rather than a shadow
        if np.median(lab[b - k - 2:b - k + 1, j, 0]) > np.median(lab[b - k + 1:b + 1, j, 0]) + cfg.shadow_drop:
            base[j] = b - k
    return base


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
    lab = cv2.cvtColor(pol, cv2.COLOR_RGB2LAB).astype(np.float32)
    ob, face_foot = _side_bands(ob, floor_m, lab, dep, hc, r_near, cfg)
    ob = _drop_narrow(ob, cfg.min_columns)
    floor_m = floor_m & ~ob                 # (a side face found by _side_bands is not floor)

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
    base = _trim_shadow(base, cols, lab, floor_m, r_near, cfg, res, face_foot)
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
    far_ahead = straight & (far[cols][inner] >= 0)
    far_dep = float(np.median(dep[far[cols][inner][far_ahead]] - 0.5 * res)) if far_ahead.sum() >= 5 else float("nan")
    return BoxMeasurement(near_x=a, skew=skew, height=height, depth=depth, width=width, y_left=y_left,
                          y_right=y_right, columns=int(len(cols)), height_spread=spread_h, ends_seen=ends_seen,
                          foot_dep=foot_dep, top_dep=top_dep, far_dep=far_dep, landing_floor=landing_floor,
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
                       height0: float = 0.25, box_depth: float | None = None, details: dict | None = None):
    """Camera height and pitch correction (and, optionally, how far the lens is ahead of the
    body centre) from the box seen square-on at known distances.

    observations: rows of (distance from the BODY CENTRE to the box's near face (m),
    depression of the face's foot (deg), depression of its top edge (deg)[, depression of the
    far edge of its top (deg)]), the angles as measured straight ahead with the current pose
    assumption (BoxMeasurement.foot_dep, .top_dep, .far_dep); nan where one was not seen.
    Least squares on the angles:

        foot_i + c = atan(hc / (D_i - xf))          top_i + c = atan((hc - h) / (D_i - xf))
        far_i + c  = atan((hc - h) / (D_i + depth - xf))        (with ``box_depth``)

    The far edge is what makes the fit firm: it is a second, much longer distance in every
    picture, where the foot and the top edge are 15 mm apart. Two distances are enough for hc
    and c. ``solve_x_forward`` also fits xf, which needs three or more distances spread over
    at least 20 cm; otherwise measure xf with a ruler.

    Returns (camera_height, pitch_correction_deg, x_forward, rms_deg): the last is the fit
    residual - a few tenths of a degree when the distances and box sizes are right.
    ``details``, if given, gets "residuals": per observation, (foot, top, far) in degrees
    (model minus measured; nan where not used).
    """
    obs = np.asarray(observations, dtype=float)
    if len(obs) < 2 or np.ptp(obs[:, 0]) < 0.08:
        raise ValueError("needs the box at two or more distances at least 8 cm apart")
    if solve_x_forward and (len(obs) < 3 or np.ptp(obs[:, 0]) < 0.20):
        raise ValueError("fitting the lens position needs three or more distances spread over 20 cm")
    D, foot, top = obs[:, 0], obs[:, 1], obs[:, 2]
    far = obs[:, 3] if obs.shape[1] > 3 and box_depth else np.full(len(obs), np.nan)
    use_foot, use_top, use_far = np.isfinite(foot), np.isfinite(top), np.isfinite(far)
    if use_foot.sum() < 2:
        raise ValueError("the foot of the box was seen at fewer than two distances")

    def parts(p):
        hc, c, xf = p
        r = D - xf
        return (np.degrees(np.arctan2(hc, r)) - (foot + c),
                np.degrees(np.arctan2(hc - box_height, r)) - (top + c),
                np.degrees(np.arctan2(hc - box_height, r + (box_depth or 0.0))) - (far + c))

    def residual(p):
        f, t, e = parts(p)
        return np.concatenate([f[use_foot], t[use_top], e[use_far]])

    p = np.array([height0, 0.0, x_forward], dtype=float)
    free = [0, 1, 2] if solve_x_forward else [0, 1]
    for _ in range(60):
        r0 = residual(p)
        J = np.zeros((len(r0), len(free)))
        for k, idx in enumerate(free):
            dp = np.zeros(3)
            dp[idx] = 1e-5
            J[:, k] = (residual(p + dp) - r0) / 1e-5
        step, *_ = np.linalg.lstsq(J, -r0, rcond=None)
        p[free] += np.clip(step, -0.5 * np.array([0.1, 10.0, 0.1])[free], 0.5 * np.array([0.1, 10.0, 0.1])[free])
        if np.abs(step).max() < 1e-9:
            break
    if details is not None:
        details["residuals"] = np.stack(parts(p), axis=1)
    hc, c, xf = (float(v) for v in p)
    if not (0.10 < hc < 0.45 and abs(c) < 30.0 and 0.0 < xf < 0.30):
        raise ValueError("no sensible solution; check the distances, the box height and the colours")
    return hc, c, xf, float(np.sqrt(np.mean(residual(p) ** 2)))
