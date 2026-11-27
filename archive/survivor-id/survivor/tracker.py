"""Who is where: dead reckoning, the people tracker, and the survivor pin.

There is no odometry on Pupper, so positions are kept in a drifting local frame: heading
from the IMU, travel from the velocity commands actually sent to the legs. While a person
is in view every detection re-anchors them, so the drift only matters while they are out
of view (plan: "the pin is carried forward by dead reckoning").
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from . import appearance
from .cameras import wrap
from .gesture import GestureFilter, GestureResult
from .person import PersonObs


def best_matching(pairs: list) -> list:
    """pairs: (cost, track id, detection index, track). The set of matches with the lowest total
    cost, so two people standing close do not swap tracks just because one pair was nearest."""
    if not pairs:
        return []
    tracks = {p[1]: p[3] for p in pairs}
    tids, js = sorted(tracks), sorted({p[2] for p in pairs})
    big = 1e6
    cost = np.full((len(tids), len(js)), big)
    for c, tid, j, _ in pairs:
        cost[tids.index(tid), js.index(j)] = c
    try:
        from scipy.optimize import linear_sum_assignment
        rows, cols = linear_sum_assignment(cost)
        chosen = [(r, c) for r, c in zip(rows, cols) if cost[r, c] < big]
    except ImportError:                       # nearest-first: fine for people who are not shoulder to shoulder
        chosen, used_r, used_c = [], set(), set()
        for c, tid, j, _ in sorted(pairs, key=lambda p: p[:3]):
            r, k = tids.index(tid), js.index(j)
            if r not in used_r and k not in used_c:
                chosen.append((r, k))
                used_r.add(r)
                used_c.add(k)
    return [(tracks[tids[r]], js[c]) for r, c in chosen]


class DeadReckoner:
    """Body pose (x, y, yaw) in the local frame."""

    def __init__(self, factor: float = 1.0):
        self.factor = float(factor)
        self.x = self.y = self.yaw = 0.0
        self.travelled = 0.0
        self.t = None
        self.cmd = (0.0, 0.0, 0.0)
        self.t_cmd = -1e9
        self._imu_yaw = None
        self._t_imu = -1e9

    def set_cmd(self, t: float, vx: float, vy: float, wz: float) -> None:
        self.advance(t)
        self.cmd, self.t_cmd = (float(vx), float(vy), float(wz)), t

    def set_imu_yaw(self, t: float, yaw: float) -> None:
        self.advance(t)
        if self._imu_yaw is not None and t - self._t_imu < 0.5:
            self.yaw += wrap(yaw - self._imu_yaw)
        self._imu_yaw, self._t_imu = yaw, t

    def advance(self, t: float) -> None:
        if self.t is None or t <= self.t:
            self.t = t if self.t is None else self.t
            return
        # A command counts for 0.6 s after it was last repeated: the mux repeats the active
        # source 20 times a second and goes quiet when nobody is driving.
        moving = max(0.0, min(t, self.t_cmd + 0.6) - self.t)
        vx, vy, wz = self.cmd
        use_cmd_yaw = t - self._t_imu > 0.5       # no IMU: fall back on the commanded turn rate
        while moving > 1e-9:
            dt = min(moving, 0.05)
            if use_cmd_yaw:
                self.yaw += wz * dt
            c, s = math.cos(self.yaw), math.sin(self.yaw)
            dx, dy = (vx * c - vy * s) * dt * self.factor, (vx * s + vy * c) * dt * self.factor
            self.x, self.y = self.x + dx, self.y + dy
            self.travelled += math.hypot(dx, dy)
            moving -= dt
        self.t = t

    def to_local(self, xy_body) -> np.ndarray:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.array([self.x + c * xy_body[0] - s * xy_body[1], self.y + s * xy_body[0] + c * xy_body[1]])

    def to_body(self, xy_local) -> np.ndarray:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = xy_local[0] - self.x, xy_local[1] - self.y
        return np.array([c * dx + s * dy, -s * dx + c * dy])


@dataclass
class Track:
    id: int
    xy: np.ndarray                    # local frame
    t_first: float
    t_last: float
    cam: str
    obs: PersonObs
    filter: GestureFilter
    hits: int = 1
    gesture: GestureResult = field(default_factory=GestureResult)
    appearance: dict = field(default_factory=dict)       # camera -> descriptor
    obs_by_cam: dict = field(default_factory=dict)       # camera -> latest PersonObs from it
    t_zoom: float = -1e9              # frame time of the last zoom pass
    t_maybe: float = -1e9             # last time an arm looked raised
    t_up: float = -1e9                # last frame that showed the signal
    travelled_at_seen: float = 0.0
    announced: bool = False           # "candidate" event sent
    wide_height: dict = field(default_factory=dict)      # camera -> how tall they were in its last wide pass (model input pixels)
    v: np.ndarray = field(default_factory=lambda: np.zeros(2))   # local frame, m/s, across the line of sight only

    def best_cam(self, now: float, max_age: float = 1.0) -> str:
        """The camera to judge this person by: one that sees all of them if any does, else the latest."""
        fresh = [(o.clipped, -o.t, cam) for cam, o in self.obs_by_cam.items() if now - o.t <= max_age]
        return min(fresh)[2] if fresh else self.cam


class PeopleTracker:
    def __init__(self, cfg: dict, dr: DeadReckoner):
        self.cfg, self.t, self.dr = cfg, cfg["tracker"], dr
        self.tracks: list = []
        self.survivor: Track | None = None
        self._next = 1

    # ---------------------------------------------------------------- geometry
    def polar(self, track: Track) -> tuple:
        """(bearing rad, ground range m) of a track in the body frame, now."""
        b = self.dr.to_body(track.xy)
        return math.atan2(b[1], b[0]), float(math.hypot(b[0], b[1]))

    def visible(self, track: Track, now: float, within: float | None = None) -> bool:
        return now - track.t_last <= (float(self.t["visible_s"]) if within is None else within)

    def sigma(self, track: Track, now: float) -> float:
        """Rough 1-sigma position error, metres."""
        _, rng = self.polar(track)
        drift = float(self.t["drift_fraction"]) * max(0.0, self.dr.travelled - track.travelled_at_seen)
        return 0.15 * rng + (0.0 if self.visible(track, now) else drift)

    def _expected(self, track: Track, now: float) -> np.ndarray:
        """Where the track should be now (local frame): people who are walking keep walking for a moment.
        This is what keeps two tracks apart when one person walks past another."""
        return track.xy + track.v * min(max(now - track.t_last, 0.0), float(self.t["predict_s"]))

    def _cost(self, track: Track, obs: PersonObs, desc, now: float, lost_survivor: bool) -> float | None:
        e = self.dr.to_body(self._expected(track, now))
        b_t, r_t = math.atan2(e[1], e[0]), float(math.hypot(e[0], e[1]))
        gate_b = math.radians(float(self.t["reacquire_gate_deg" if lost_survivor else "gate_deg"]))
        gate_r = math.log(float(self.t["reacquire_range_ratio" if lost_survivor else "gate_range_ratio"]))
        drift = float(self.t["drift_fraction"]) * max(0.0, self.dr.travelled - track.travelled_at_seen)
        gate_b += math.atan2(drift, max(r_t, 0.3))
        d_b = abs(wrap(obs.bearing - b_t)) / gate_b
        r_o = obs.ground_range
        d_r = abs(math.log(max(r_o, 0.1) / max(r_t, 0.1))) / gate_r if (r_o and obs.s_from != "box") else 0.0
        cost = math.hypot(d_b, d_r)
        if cost > 1.0:
            return None
        d_app = appearance.distance(track.appearance.get(obs.cam), desc)
        if d_app is not None:
            if lost_survivor:
                if d_app > float(self.t["appearance_max"]):
                    return None                    # a stranger near where the survivor was
                cost += d_app
            else:
                cost += 0.5 * d_app
        return cost

    # ---------------------------------------------------------------- updates
    def _apply(self, track: Track, obs: PersonObs, desc, now: float, alpha: float) -> None:
        if obs.position is not None:
            xy = self.dr.to_local(obs.position[:2])
            gap = now - track.t_last
            a = 1.0 if gap > 1.5 else alpha
            if obs.s_from == "box" and gap <= 1.5:          # range is a guess: move in bearing only
                _, r_t = self.polar(track)
                d = obs.position[:2] / max(float(np.linalg.norm(obs.position[:2])), 1e-6)
                xy = self.dr.to_local(d * r_t)
            if gap > 1.5:
                track.xy, track.v = xy, np.zeros(2)
            else:
                expected = self._expected(track, now)
                miss = xy - expected
                track.xy = expected + a * miss
                if gap >= 0.03:
                    # Speed across the line of sight only: bearing is measured to a fraction of a
                    # degree, range from body size is not good enough to take a speed from.
                    line = track.xy - np.array([self.dr.x, self.dr.y])
                    line = line / max(float(np.linalg.norm(line)), 1e-6)
                    across = np.array([-line[1], line[0]])
                    v = track.v + float(self.t["velocity_gain"]) * across * float(miss @ across) / gap
                    v = across * float(v @ across)
                    top = float(self.t["max_speed"])
                    track.v = v * min(1.0, top / max(float(np.linalg.norm(v)), 1e-6))
        track.t_last, track.cam, track.obs = now, obs.cam, obs
        track.hits += 1
        if not obs.zoom:
            track.wide_height[obs.cam] = obs.height_px
        track.obs_by_cam[obs.cam] = obs
        track.travelled_at_seen = self.dr.travelled
        if desc is not None:
            rate = 1.0 if obs.cam not in track.appearance else float(self.t["appearance_blend"])
            track.appearance[obs.cam] = appearance.blend(track.appearance.get(obs.cam), desc, rate)

    def update(self, observations: list, descriptors: list, now: float) -> list:
        """Match one camera's detections to tracks. Returns [(track, obs)], new tracks included."""
        free_obs = list(range(len(observations)))
        pairs, out = [], []
        lost = self.survivor if (self.survivor is not None and not self.visible(self.survivor, now)) else None
        for track in self.tracks:
            if track is lost:
                continue
            for j in free_obs:
                c = self._cost(track, observations[j], descriptors[j], now, False)
                if c is not None:
                    pairs.append((c, track.id, j, track))
        for track, j in best_matching(pairs):
            free_obs.remove(j)
            self._apply(track, observations[j], descriptors[j], now, float(self.t["smooth"]))
            out.append((track, observations[j]))
        if lost is not None:                      # the survivor was out of view: is one of the rest them?
            cands = sorted((c, j) for c, j in ((self._cost(lost, observations[j], descriptors[j], now, True), j)
                                               for j in free_obs) if c is not None)
            clear = len(cands) == 1 or (len(cands) > 1 and cands[1][0] > cands[0][0] * float(self.t["ambiguous_ratio"]) + 1e-6)
            if cands and clear:
                j = cands[0][1]
                free_obs.remove(j)
                self._apply(lost, observations[j], descriptors[j], now, 1.0)
                out.append((lost, observations[j]))
            elif cands:
                free_obs = [j for j in free_obs if j not in {c[1] for c in cands}]   # unsure: wait for a clearer view
        for j in free_obs:
            obs = observations[j]
            if obs.position is None:
                continue
            track = Track(self._next, self.dr.to_local(obs.position[:2]), now, now, obs.cam, obs,
                          GestureFilter.from_config(self.cfg), travelled_at_seen=self.dr.travelled)
            self._next += 1
            track.obs_by_cam[obs.cam] = obs
            track.wide_height[obs.cam] = obs.height_px
            if descriptors[j] is not None:
                track.appearance[obs.cam] = descriptors[j]
            self.tracks.append(track)
            out.append((track, obs))
        return out

    def pick(self, track: Track, seen: list, now: float):
        """From [(obs, descriptor)] found in a patch aimed at `track`, the one that is this track's:
        the best match that no other track has a better claim to. None if there is none."""
        best = None
        for obs, desc in seen:
            lost = track is self.survivor and not self.visible(track, now)
            mine = self._cost(track, obs, desc, now, lost)
            if mine is None:
                continue
            others = [c for c in (self._cost(o, obs, desc, now, False) for o in self.tracks if o is not track) if c is not None]
            if others and min(others) < mine:
                continue
            if best is None or mine < best[0]:
                best = (mine, obs, desc)
        return None if best is None else best[1:]

    def update_track(self, track: Track, obs: PersonObs, desc, now: float) -> None:
        """A zoom pass looked straight at this track: no matching needed."""
        self._apply(track, obs, desc, now, min(1.0, float(self.t["smooth"]) + 0.2))

    def prune(self, now: float) -> None:
        keep = []
        for tr in self.tracks:
            limit = float(self.t["forget_s"] if tr.hits >= int(self.t["confirm_hits"]) else self.t["tentative_forget_s"])
            if tr is self.survivor or now - tr.t_last <= limit:
                keep.append(tr)
        self.tracks = keep

    def confirmed(self, track: Track) -> bool:
        return track.hits >= int(self.t["confirm_hits"])
