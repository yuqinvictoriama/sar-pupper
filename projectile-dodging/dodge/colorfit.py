"""Work out a camera's colour profile for the ball from samples of the ball and pictures of the empty room.

    fit_color(ball_hsv, judge) -> (profile, report)

Used by tools/calibrate_color.py; kept here, free of cameras and prompts, so it
can be tested on stored pictures.
"""
from __future__ import annotations

import numpy as np


def chroma(hsv_px: np.ndarray) -> np.ndarray:
    return hsv_px[:, 1].astype(np.int32) * hsv_px[:, 2].astype(np.int32) // 255


def passing(hsv_px: np.ndarray, color: dict, bar: str = "c_min") -> float:
    """Share of the sampled ball pixels that the profile accepts (bar="c_seed": that clear the higher bar)."""
    ok = ((hsv_px[:, 0] >= color["h_min"]) & (hsv_px[:, 0] <= color["h_max"]) & (hsv_px[:, 1] >= color["s_min"])
          & (hsv_px[:, 2] >= color["v_min"]) & (chroma(hsv_px) >= color[bar]))
    return float(ok.mean())


def fit_color(ball_hsv: np.ndarray, judge, recall=None, min_keep: float = 0.8, max_cover: float = 0.005, min_recall: float = 0.9):
    """Colour profile from ball pixels (N x 3, OpenCV HSV), tightened until the empty room shows no ball.

    judge(profile) must return (patches, cover) for pictures of the room
    without the ball: how many ball-like patches a detector with that profile
    finds, and what share of the picture counts as ball-coloured at all.
    recall(profile), if given, must return the share of the sample pictures
    of the ball in which the detector finds it; a limit is never raised past
    the point where that drops below min_recall.

    The hue range is the ball's own. The other limits start loose, so the
    whole ball is outlined, its shaded side too, and a small or poorly lit
    ball is still found. They are then raised only as far as needed to leave
    the empty room empty (no patches, and no more than max_cover of it
    ball-coloured, or a ball would merge with what is behind it), and never so
    far that less than min_keep of the ball's pixels would still pass.
    """
    hue, sat, val = ball_hsv[:, 0], ball_hsv[:, 1], ball_hsv[:, 2]
    c = chroma(ball_hsv)
    h_lo, h_hi = np.percentile(hue, [1, 99])
    spread = max(0.0, 10.0 - (h_hi - h_lo)) / 2.0           # never narrower than about 18 hue steps in all
    color = {
        "h_min": int(max(0, np.floor(h_lo - 4 - spread))), "h_max": int(min(179, np.ceil(h_hi + 4 + spread))),
        "s_min": int(max(40, np.percentile(sat, 2) - 25)), "s_max": 255,
        # brightness is a weak test (the ball's underside in flight is far darker
        # than the ball held in the light): keep it loose, colourfulness does the work
        "v_min": int(max(35, 0.45 * np.percentile(val, 5))), "v_max": 255,
        "c_min": int(max(15, 0.6 * np.percentile(c, 5))),
        # a far ball is a few pixels of ball blurred into its background, so its
        # colour is weaker than that of the ball sampled close up: start lower
        "c_seed": int(max(25, 0.75 * np.percentile(c, 20))),
    }
    color["c_seed"] = max(color["c_seed"], color["c_min"] + 5)
    keep_pct = 100 * (1 - min_keep)
    # (limit to raise, how far it may go, step)
    plan = (("c_seed", int(np.percentile(c, 50)), 3), ("c_min", int(np.percentile(c, keep_pct)), 3),
            ("s_min", int(np.percentile(sat, keep_pct)), 5), ("c_seed", int(np.percentile(c, 65)), 3))
    patches, cover = judge(color)
    # First the hue range: if what shows up in the empty room is a neighbouring
    # colour (a cream wall, a yellow floor), closing the range in on the ball's
    # own hue removes it at no cost to the ball in poor light.
    for key, target in (("h_min", int(np.floor(np.percentile(hue, 4)))), ("h_max", int(np.ceil(np.percentile(hue, 96))))):
        if not (patches > 0 or cover > max_cover) or target == color[key]:
            continue
        ends = dict(color)
        ends[key] = target
        if recall is not None and recall(ends) < min_recall:
            continue
        got = judge(ends)
        if (got[1] > max_cover, got[0]) >= (cover > max_cover, patches):
            continue                                         # no better: leave this side alone
        direction = 1 if target > color[key] else -1
        for value in range(color[key] + direction, target + direction, direction):   # the least trim that does as well
            trial = dict(color)
            trial[key] = value
            if judge(trial) == got or value == target:
                color, (patches, cover) = trial, got
                break
    tried = [(cover > max_cover, patches, color)]
    for key, limit, step in plan:
        while (patches > 0 or cover > max_cover) and color[key] + step <= limit:
            if key == "c_seed" and patches == 0:
                break                                        # the higher bar does nothing about cover
            trial = dict(color)
            trial[key] += step
            if key == "c_min":
                trial["c_seed"] = max(trial["c_seed"], trial["c_min"] + 5)
            if recall is not None and recall(trial) < min_recall:
                break                                        # would start losing the ball itself
            color = trial
            patches, cover = judge(color)
            tried.append((cover > max_cover, patches, color))
    # Something in the room may simply be the ball's colour, and no limit removes
    # it. Then the loosest profile that did as well as the tightest is the one
    # to keep: tightening further only costs the ball.
    best = min(t[:2] for t in tried)
    color = next(t[2] for t in tried if t[:2] == best)
    patches, cover = judge(color)
    med = float(np.median(c))
    report = {"kept": passing(ball_hsv, color), "seeds": passing(ball_hsv, color, "c_seed"), "false": int(patches),
              "found": float(recall(color)) if recall is not None else None,
              "cover": float(cover), "ball_hue": (float(h_lo), float(h_hi)), "ball_chroma": med,
              "ball_brightness": float(np.median(val)),
              # how much dimmer the ball can get before too little of it clears the higher bar
              "dimmest": float(color["c_seed"]) / max(float(np.percentile(c, 80)), 1.0)}
    return color, report
