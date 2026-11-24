"""The signal: both arms thrown high in the air (plan, Pipeline steps 7 and 8).

Everything is measured in the level frame (up = against gravity, whatever the camera or the
robot's tilt) and in torso lengths. Heights are differences of tangents of elevation and
sideways distances are differences of azimuth, both divided by the person's body scale
(PersonObs.unit), so "0.4" means 0.4 torso lengths (about 19 cm) at any distance, from any
camera height.

Per arm, all of:
    a  shoulder, elbow and wrist found
    b  wrist above eye/ear level by wrist_above_head torso lengths
    c  elbow above its shoulder by elbow_above_shoulder
    d  the arm as a whole (shoulder to wrist) within arm_max_tilt_deg of straight up, and
       upper arm and forearm each within segment_max_tilt_deg (looser: at 3 m an elbow is
       placed to within a few pixels, and the two short segments swing by 10-15 degrees
       from one frame to the next; the whole arm does not)
    e  shoulder-to-wrist at least arm_min_length (an arm pointed at the camera looks short)
and for the person:
    f  range inside [min_range_m, max_range_m], and not cut off at the top of the frame

Both arms must pass. Side-on (shoulders less than profile_shoulders apart sideways) the far
arm can be lost against the near one, so one passing arm is enough if the other is simply not
found. An arm that is found and fails always fails the person, and so does a half-found one
(joints the model was unsure of, between kpt_conf_weak and kpt_conf) whose wrist is below its
shoulder: an arm hanging behind the body is hidden, not raised. Left and right are never told
apart.

What a single low camera cannot tell apart: an arm raised, and an arm pointed at the robot from
above its eye line. Both put the hand high in the picture. The range limit (nothing inside
min_range_m can be the signal) and the wrist-above-head test remove the common case, a person
reaching down toward the robot.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

from .person import PersonObs
from .pose_decode import K

ARMS = (("l_shoulder", "l_elbow", "l_wrist"), ("r_shoulder", "r_elbow", "r_wrist"))


@dataclass
class GestureResult:
    up: bool = False               # the signal, in this frame
    maybe: bool = False            # an arm is raised: worth a closer look
    profile: bool = False
    arms: tuple = ("missing", "missing")      # per arm: "pass", "fail" or "missing"
    elbows_up: bool = False        # an elbow is above its shoulder (used when the hands are out of frame)
    shoulders_found: int = 0
    why: str = ""                  # first reason it is not the signal
    undecided: bool = False        # not the signal from here, but no arm was seen to be down either


def _across(obs: PersonObs, a: int, b: int) -> float:
    """Sideways offset of joint b from joint a (difference of azimuth, radians ~ distance / range)."""
    return (obs.az[b] - obs.az[a] + math.pi) % (2 * math.pi) - math.pi


def _tilt_from_up(obs: PersonObs, a: int, b: int) -> float:
    """Angle between the segment a -> b and straight up, degrees."""
    return math.degrees(math.atan2(abs(_across(obs, a, b)), obs.tan_el[b] - obs.tan_el[a]))


def _length(obs: PersonObs, a: int, b: int) -> float:
    return math.hypot(_across(obs, a, b), obs.tan_el[b] - obs.tan_el[a])


def head_level(obs: PersonObs, thr: float) -> float | None:
    """Eye/ear level, as a tangent of elevation."""
    idx = [K[n] for n in ("l_eye", "r_eye", "l_ear", "r_ear") if obs.conf[K[n]] >= thr]
    if not idx and obs.conf[K["nose"]] >= thr:
        idx = [K["nose"]]
    return None if not idx else float(obs.tan_el[idx].mean())


def check_arm(obs: PersonObs, names, head: float | None, g: dict) -> tuple:
    """("pass" | "fail" | "missing", reason)."""
    thr = float(g["kpt_conf"])
    sh, el, wr = (K[n] for n in names)
    if not (obs.conf[sh] >= thr and obs.conf[el] >= thr and obs.conf[wr] >= thr):
        weak = float(g.get("kpt_conf_weak", thr))
        if obs.conf[sh] >= weak and obs.conf[wr] >= weak and obs.tan_el[wr] < obs.tan_el[sh]:
            return "fail", "the half-hidden arm is down"
        return "missing", "arm not found"
    u = obs.unit
    if u is None or head is None:
        return "fail", "no body scale" if u is None else "head not found"
    if obs.tan_el[wr] - head < float(g["wrist_above_head"]) * u:
        return "fail", "wrist not above the head"
    if obs.tan_el[el] - obs.tan_el[sh] < float(g["elbow_above_shoulder"]) * u:
        return "fail", "elbow not above the shoulder"
    part = float(g.get("segment_max_tilt_deg", g["arm_max_tilt_deg"]))
    if (_tilt_from_up(obs, sh, wr) > float(g["arm_max_tilt_deg"])
            or _tilt_from_up(obs, sh, el) > part or _tilt_from_up(obs, el, wr) > part):
        return "fail", "arm not pointing up"
    if _length(obs, sh, wr) < float(g["arm_min_length"]) * u:
        return "fail", "arm too short (pointing at the camera?)"
    return "pass", ""


def evaluate(obs: PersonObs, cfg: dict) -> GestureResult:
    g = cfg["gesture"]
    thr = float(g["kpt_conf"])
    head = head_level(obs, thr)
    res = GestureResult()
    checks = [check_arm(obs, names, head, g) for names in ARMS]
    res.arms = tuple(c[0] for c in checks)
    ls, rs = K["l_shoulder"], K["r_shoulder"]
    res.shoulders_found = int(obs.conf[ls] >= thr) + int(obs.conf[rs] >= thr)
    for sh, el, wr in ((K[a], K[b], K[c]) for a, b, c in ARMS):
        if obs.conf[sh] >= thr and obs.conf[el] >= thr and obs.el[el] > obs.el[sh]:
            res.elbows_up = True
            res.maybe = True
        if obs.conf[sh] >= thr and obs.conf[wr] >= thr and obs.el[wr] > obs.el[sh]:
            res.maybe = True
    if obs.unit is not None and res.shoulders_found == 2:
        res.profile = abs(_across(obs, ls, rs)) < float(g["profile_shoulders"]) * obs.unit

    passed = res.arms.count("pass")
    arms_ok = passed == 2 or (passed == 1 and "missing" in res.arms and res.profile)
    if not arms_ok:
        res.why = next((c[1] for c in checks if c[0] == "fail"), None) or \
            ("one arm hidden and the person is not side-on" if passed == 1 else "arms not found")
    elif obs.clipped_top:
        res.why = "cut off at the top of the frame"
    elif obs.range_m is None or obs.s_from == "box":
        res.why = "range unknown"
    elif not (float(g["min_range_m"]) <= obs.ground_range <= float(g["max_range_m"])):
        res.why = "range %.1f m outside %.1f-%.1f m" % (obs.ground_range, float(g["min_range_m"]), float(g["max_range_m"]))
    res.up = arms_ok and not res.why
    # Worth turning to look at, if the camera that saw this cannot settle it: nothing says an arm is down.
    res.undecided = not res.up and "fail" not in res.arms
    return res


@dataclass
class GestureFilter:
    """Step 8: the signal must show in most readings over a window before it counts."""
    window_s: float = 0.8
    min_frames: int = 5
    min_fraction: float = 0.7
    samples: deque = field(default_factory=deque)      # (t, up)

    @classmethod
    def from_config(cls, cfg: dict) -> "GestureFilter":
        g = cfg["gesture"]
        return cls(float(g["window_s"]), int(g["min_frames"]), float(g["min_fraction"]))

    def add(self, t: float, up: bool) -> None:
        self.samples.append((t, bool(up)))
        while self.samples and self.samples[0][0] < t - self.window_s:
            self.samples.popleft()

    def fraction(self, t: float) -> float:
        recent = [u for ts, u in self.samples if ts >= t - self.window_s]
        return sum(recent) / len(recent) if recent else 0.0

    def confirmed(self, t: float) -> bool:
        recent = [(ts, u) for ts, u in self.samples if ts >= t - self.window_s]
        if len(recent) < self.min_frames:
            return False
        # The readings must span most of the window: five quick frames are not a held pose.
        if recent[-1][0] - recent[0][0] < 0.75 * self.window_s:
            return False
        return sum(u for _, u in recent) / len(recent) >= self.min_fraction

    def reset(self) -> None:
        self.samples.clear()
