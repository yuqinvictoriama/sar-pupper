"""Angles that wrap, and turning several noisy bearings into one direction.

Everything here is in radians. 179 degrees and -179 degrees are 2 degrees
apart; every difference and every average in nav/ goes through these.
"""
from __future__ import annotations

import math
from dataclasses import dataclass


def wrap(a: float) -> float:
    """Into (-pi, pi]."""
    a = (a + math.pi) % (2.0 * math.pi) - math.pi
    return a if a != -math.pi else math.pi


def diff(a: float, b: float) -> float:
    """The short way round from b to a."""
    return wrap(a - b)


def circular_mean(angles) -> float:
    return math.atan2(sum(math.sin(a) for a in angles), sum(math.cos(a) for a in angles))


def circular_median(angles) -> float:
    """The one of the angles with the least total distance to the others.

    Unlike the mean, one echo off a wall does not drag it: with three
    bearings at 10, 12 and 95 degrees it answers 12. It is always one of
    the bearings that was heard, never a point between two that disagree.
    """
    angles = list(angles)
    if not angles:
        raise ValueError("no angles")
    return min(angles, key=lambda c: sum(abs(diff(a, c)) for a in angles))


@dataclass
class Fix:
    heading: float      # rad, in the frame the bearings were given in
    count: int          # bearings that agreed
    spread: float       # rad, the furthest of them from the median
    rejected: int       # bearings that did not agree (echoes, other voices)


class FixCollector:
    """Bearings in, one fix out once enough of them agree.

    Bearings are stored as headings in a frame that does not turn with the
    robot (IMU yaw + bearing), so Pupper turning between two cries does not
    smear them.
    """

    def __init__(self, min_bearings: int = 3, spread: float = math.radians(8.0), window_s: float = 14.0):
        self.min_bearings, self.spread, self.window_s = int(min_bearings), float(spread), float(window_s)
        self.items: list[tuple[float, float]] = []      # (time, heading)

    def clear(self) -> None:
        self.items.clear()

    def add(self, t: float, heading: float) -> None:
        self.items.append((t, wrap(heading)))

    def _fresh(self, t: float) -> list[float]:
        self.items = [(ti, h) for ti, h in self.items if t - ti <= self.window_s]
        return [h for _, h in self.items]

    def count(self, t: float) -> int:
        return len(self._fresh(t))

    def first_time(self):
        return self.items[0][0] if self.items else None

    def best(self, t: float, prior: float | None = None) -> Fix | None:
        """The fix from what has been heard so far, however few; None if nothing.

        The largest group of bearings that agree (within `spread` of one of them) wins.
        When two groups are the same size, the one nearer `prior` (where the voice is
        expected) wins; without a prior, the tighter one. So two bearings 40 degrees
        apart never average to a direction nobody called from.
        """
        hs = self._fresh(t)
        if not hs:
            return None
        groups = []
        for c in hs:
            members = [h for h in hs if abs(diff(h, c)) <= self.spread]
            centre = circular_mean(members)
            width = max(abs(diff(h, centre)) for h in members)
            off = abs(diff(centre, prior)) if prior is not None else width
            groups.append((-len(members), off, width, centre, len(members)))
        _, _, width, centre, n = min(groups)
        return Fix(centre, n, width, len(hs) - n)

    def fix(self, t: float, prior: float | None = None) -> Fix | None:
        """A fix only once `min_bearings` agree."""
        f = self.best(t, prior)
        return f if f is not None and f.count >= self.min_bearings else None
