"""Walking a line, not a heading.

After a walk-around or a dodge Pupper is off to one side. Holding the heading
alone would carry that offset all the way to the cover, so the command aims
at a point a metre ahead on the line and the offset shrinks as it walks.

No sideways command: sideways is Pupper's slow axis, and keeping the nose on
the line keeps the fisheye on what is ahead.
"""
from __future__ import annotations

import math

from .heading import wrap


def clip(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


def line_command(d: float, heading_error: float, speed: float, walk, bias: float = 0.0):
    """(vx, vy, wz) to follow the line.

    d              metres left of the line
    heading_error  yaw minus the line's heading, rad
    speed          m/s wanted along the line
    walk           config.Walk
    bias           rad added to the wanted heading (used to home on the cover)
    """
    max_off = math.radians(walk.max_off_line_deg)
    wanted = clip(-math.atan2(d, walk.look_ahead), -max_off, max_off) + bias
    err = wrap(wanted - heading_error)
    wz = clip(walk.yaw_gain * err, -walk.max_yaw_rate, walk.max_yaw_rate)
    limit = math.radians(walk.turn_in_place_deg)
    if abs(err) >= limit:
        return 0.0, 0.0, wz                      # facing well away: turn first
    vx = max(walk.min_forward, speed * (1.0 - 0.6 * abs(err) / limit))
    return min(vx, speed), 0.0, wz


def turn_command(error: float, orient):
    """(vx, vy, wz) to turn on the spot through `error` rad (left positive)."""
    wz = clip(orient.gain * error, -orient.turn_rate, orient.turn_rate)
    if abs(wz) < orient.min_rate:
        wz = math.copysign(orient.min_rate, error)
    return 0.0, 0.0, wz
