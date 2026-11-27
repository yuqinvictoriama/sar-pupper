#!/usr/bin/env python3
"""The whole demo run, in two dimensions, on a laptop.

    python3 sim/course_sim.py                  one run, printed
    python3 sim/course_sim.py --runs 100       how often it ends in front of the cover
    python3 sim/course_sim.py --moe 8          with a worse sound module (margin of error, degrees)

What is real and what is a stand-in:

    real      the mission (nav/mission.py), the position estimate and the line following;
              terrain agility's state machine and steering (terrain-agility/terrain/state_machine.py),
              fed a range scan cast geometrically with the camera's field of view
    stand-in  the sound module (the true bearing plus noise and the odd echo, one or two per cry)
              the walk (follows the mux's command with a lag and a speed error the mission does not know)
              the IMU (true yaw plus a slow drift)
              the crossing (the body is moved forward over about 50 s, as the step-over node would)
              the dodge node (sees a ball late in its flight, bursts across its path, then recovers)
              the survivor, the throwers and the phrase detector (the script in "Demo instructions")

So it checks the logic that strings the components together: zones, the line, the second
listen, who has the legs, when the reflexes are switched on. It says nothing about perception,
real sound in a real room, or the legs. Mission Control's simulation mode runs this same class.
"""
from __future__ import annotations

import argparse
import math
import random
import sys
from dataclasses import dataclass, fields
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
TERRAIN = ROOT.parent / "terrain-agility"
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(TERRAIN))

from nav import events as EV  # noqa: E402
from nav.board import Board  # noqa: E402
from nav.config import DemoConfig, load_config  # noqa: E402
from nav.heading import wrap  # noqa: E402
from nav.mission import IDLE, RESCUE, ZONE1, ZONE2, Mission  # noqa: E402
from sim.avoid_sim import Rect, to_obstacles  # noqa: E402  (terrain-agility/sim)
from terrain import state_machine as SM  # noqa: E402
from terrain.gauge import BoxMeasurement  # noqa: E402

DT = 0.05
MUX_ORDER = ["/teleop_cmd_vel", "/dodge_cmd_vel", "/terrain_cmd_vel", "/nav_cmd_vel"]
MUX_TIMEOUT = 0.5
BODY_HALF_X, BODY_HALF_Y = 0.20, 0.12       # the box a ball has to miss
BALL_R = 0.037
CAMERA_X = 0.145


@dataclass
class SimOptions:
    seed: int = 0
    start_yaw_deg: float | None = None      # which way Pupper faces on the start mark; None = random
    moe_deg: float | None = None            # the sound module's margin of error; None = the config's
    echo_p: float = 0.08                    # chance that a bearing is an echo, well off the true direction
    per_cry: int = 2                        # bearings the sound module reports for one cry
    speed_factor: float = 0.92              # actual / commanded walking speed (the mission assumes 1.0)
    walk_lag: float = 0.25
    yaw_drift_deg_min: float = 1.5          # IMU drift
    crossing_s: float | None = None         # None: what terrain agility's simulation takes (slab 81 s, bar 52 s)
    plan_s: float = 4.0
    throws: bool = True
    throw_gap_s: float = 3.0
    flight_s: float = 1.0
    aim_sigma: float = 0.12                 # m, how well the throwers hit where Pupper is
    dodge_react_s: float = 0.55             # the dodge node moves this long before the ball would land
    phrase_p: float = 0.8                   # chance the detector hears each saying of the phrase
    survivor_calls: bool = True
    second_call: bool = True
    says_phrase: bool = True


class CourseSim:
    def __init__(self, cfg: DemoConfig | None = None, opt: SimOptions | None = None, motion: bool = True):
        self.cfg = cfg or DemoConfig()
        self.opt = opt or SimOptions()
        self.rng = random.Random(self.opt.seed)
        self.np_rng = np.random.default_rng(self.opt.seed)
        c = self.cfg.course
        self.board = Board(self.cfg)
        self.mission = Mission(self.cfg, motion=motion)
        self.t = 0.0
        yaw0 = self.opt.start_yaw_deg if self.opt.start_yaw_deg is not None else self.rng.uniform(-180.0, 180.0)
        self.pose = np.array([0.0, 0.0, math.radians(yaw0)])        # truth, course frame
        self.vel = np.zeros(3)
        self.imu_offset = self.rng.uniform(-math.pi, math.pi)       # the IMU's zero is wherever it woke up
        self.imu_drift = 0.0
        self.path = [(0.0, 0.0)]
        self.survivor = (c.cover_near + 0.45, 0.0)
        # The course's obstacles, then the cover. To the walking scan a thing is "certainly tall" (walked
        # around from a distance) or "may be low" (walked up to and measured): terrain agility's own rule.
        self.rects = []
        for ob in c.obstacles:
            tall = float(ob.get("height", 0.02)) >= 0.08
            self.rects.append(Rect(float(ob["near"]) + float(ob["depth"]) / 2, 0.0, float(ob["depth"]), float(ob["width"]),
                                   tall=tall, is_box=not tall))
        self.rects.append(Rect(c.cover_near + 0.03, 0.0, 0.06, c.cover_width))
        self.crossed = set()                # indices of obstacles a crossing went over
        self.min_clear = {"tall": 9.9, "cover": 9.9}
        # terrain agility (the real state machine)
        self.fsm = SM.TerrainFsm(SM.FsmConfig())
        self.terrain_enabled = False
        self.next_scan = 0.0
        self.obstacles = []
        self.points = np.zeros((0, 2))
        self.cross = None                   # the crossing in progress: dict
        self.legs_back_at = -1e9
        # the mux
        self.sources = {name: (-1e9, None) for name in MUX_ORDER}
        self.nudge_until = -1e9
        self.nudge_cmd = (0.0, 0.0, 0.0)
        # dodging
        self.dodge_armed = False
        self.dodge_state = "SEARCH"
        self.balls: list = []               # in flight and landed
        self.burst = None                   # (cmd, until, recover_until)
        self.next_throw = 0.0
        self.throw_side = 1
        self.throws = self.hits = self.dodged = 0
        # the survivor
        self.voice_until = -1e9             # Pupper is talking until then (the survivor waits)
        self.calls_due: list = []           # times the survivor will cry out
        self.pending_bearings: list = []    # (time, bearing)
        self.call_until = -1e9
        self.phrase_at = None
        self.said_phrase = 0
        self.refix_calls_made = False
        self.next_first_call = None
        self.survivor_says = ""
        self.survivor_says_until = -1e9

    # ------------------------------------------------------------------ operator
    def command(self, text: str) -> None:
        if text.startswith("nudge:"):       # a tap on the joystick, to show the pause: "nudge:<vx>,<vy>"
            try:
                vx, vy = (float(v) for v in text[6:].split(","))
            except ValueError:
                return
            self.nudge_cmd, self.nudge_until = (vx, vy, 0.0), self.t + 1.0
            return
        if text.startswith("goto:") and self.mission.state not in ("PREFLIGHT", "SEEK", "ORIENT", "CONFIRM", ZONE1,
                                                                   "REFIX", ZONE2, "ARRIVE"):
            # starting from a stage: the operator has put Pupper there, facing along the line
            c = self.cfg.course
            at = {"REFIX": c.refix_at + 0.1, "ZONE2": c.refix_at + 0.1, "ARRIVE": c.length, "RESCUE": c.length}
            self.crossed = set(range(len(c.obstacles)))
            self.pose[:] = (at.get(text[5:], 0.0), 0.0, 0.0)
            self.vel[:] = 0.0
            self.path = [(float(self.pose[0]), 0.0)]
            self.balls = []
        self.board.command(text)

    # ------------------------------------------------------------------ geometry
    def imu_yaw(self) -> float:
        return wrap(self.pose[2] + self.imu_offset + self.imu_drift)

    def true_bearing(self) -> float:
        return wrap(math.atan2(self.survivor[1] - self.pose[1], self.survivor[0] - self.pose[0]) - self.pose[2])

    def cast(self):
        """Obstacle foot points as the fisheye would give them: body frame, one per 2 degrees."""
        x, y, th = self.pose
        cam = np.array([x + CAMERA_X * math.cos(th), y + CAMERA_X * math.sin(th)])
        bearings = np.radians(np.arange(-100.0, 100.1, 2.0))
        ang = th + bearings
        d = np.stack([np.cos(ang), np.sin(ang)], axis=1)
        best = np.full(len(bearings), np.inf)
        owner = np.full(len(bearings), -1)
        for k, r in enumerate(self.rects):
            c = r.corners()
            for i in range(4):
                p, q = c[i], c[(i + 1) % 4]
                e = q - p
                den = d[:, 0] * e[1] - d[:, 1] * e[0]
                w = p - cam
                with np.errstate(divide="ignore", invalid="ignore"):
                    tt = (w[0] * e[1] - w[1] * e[0]) / den
                    u = (w[0] * d[:, 1] - w[1] * d[:, 0]) / den
                ok = (np.abs(den) > 1e-12) & (tt > 0.02) & (u >= 0.0) & (u <= 1.0) & (tt < best)
                best = np.where(ok, tt, best)
                owner = np.where(ok, k, owner)
        seen = best < 2.5
        rr = best[seen] + self.np_rng.normal(0.0, 0.012, int(seen.sum()))
        b = bearings[seen]
        pts = np.stack([CAMERA_X + rr * np.cos(b), rr * np.sin(b)], axis=1) if len(rr) else np.zeros((0, 2))
        return pts, owner[seen]

    # ------------------------------------------------------------------ the survivor and the sound module
    def _hear_pupper(self, events) -> None:
        for name, _ in events:
            if not EV.is_spoken(name):
                continue
            self.voice_until = max(self.voice_until, self.t + EV.speak_time(name) - EV.SETTLE_AFTER_SPEECH_S)
            if name == "start" and self.opt.survivor_calls:
                self.next_first_call = self.voice_until + 0.6
            elif name == "refix" and self.opt.survivor_calls and self.opt.second_call:
                self.calls_due = [self.voice_until + 0.7, self.voice_until + 2.7]
            elif name == "arrived" and self.opt.says_phrase:
                self.phrase_at = self.voice_until + 0.7

    def _cry(self) -> None:
        """One "Help! Help me!": the sound module reports a bearing or two while it lasts."""
        self.call_until = self.t + 1.2
        self.survivor_says, self.survivor_says_until = "Help! Help me!", self.t + 1.4
        moe = self.opt.moe_deg if self.opt.moe_deg is not None else self.cfg.sound.moe_deg
        sigma = math.radians(moe) / 2.0             # the margin of error read as two standard deviations
        for k in range(self.opt.per_cry):
            b = self.true_bearing() + self.rng.gauss(0.0, sigma)
            if self.rng.random() < self.opt.echo_p:
                b += math.radians(self.rng.choice((-1, 1)) * self.rng.uniform(30.0, 90.0))
            self.pending_bearings.append((self.t + 0.5 + 0.4 * k, wrap(b)))

    def _survivor(self) -> None:
        st = self.mission.state
        if self.next_first_call is not None and self.t >= self.next_first_call:
            if st in (ZONE1, "REFIX", ZONE2, "ARRIVE", RESCUE, IDLE, "DONE"):
                self.next_first_call = None          # Pupper is walking: stop calling
            elif self.t >= self.voice_until + 0.3:
                self._cry()
                self.next_first_call = self.t + 4.0
            else:
                self.next_first_call = self.voice_until + 0.6
        while self.calls_due and self.t >= self.calls_due[0]:
            self.calls_due.pop(0)
            self._cry()
        if self.phrase_at is not None and self.t >= self.phrase_at:
            if st == RESCUE or st in (IDLE, "DONE"):
                self.phrase_at = None
            elif self.t < self.voice_until + 0.3:
                self.phrase_at = self.voice_until + 0.7
            else:
                self.said_phrase += 1
                self.survivor_says, self.survivor_says_until = EV.PHRASE, self.t + 1.6
                near = math.hypot(self.survivor[0] - self.pose[0], self.survivor[1] - self.pose[1]) < 2.5
                if near and self.rng.random() < self.opt.phrase_p:
                    self.board.keyword_heard(self.t + 1.2, "YOU_SAVED_ME_PUPPER")
                self.phrase_at = self.t + 3.0
        due = [b for b in self.pending_bearings if b[0] <= self.t]
        self.pending_bearings = [b for b in self.pending_bearings if b[0] > self.t]
        for _, b in due:
            # the module reports relative to the nose as it is now
            self.board.bearing(self.t, b, 1.0)

    # ------------------------------------------------------------------ terrain agility
    def _terrain(self, nav_cmd):
        """Tick the real state machine; returns its velocity command or None."""
        mode = self.board.wanted["terrain_mode"]
        self.terrain_enabled = mode != "off"
        if self.terrain_enabled and mode != self.fsm.mode:
            self.fsm.set_mode(mode)
        t = self.t
        if self.cross is not None:
            self._crossing_step()
        scanning = self.cross is None or self.cross["stage"] == "planning"
        new_scan = scanning and t >= self.next_scan
        if new_scan:
            self.next_scan = t + 1.0 / 8.0
            self.points, owner = self.cast()
            self.obstacles = to_obstacles(self.points, owner, self.rects) if len(self.points) else []
            self.board.terrain_obstacles(t, [
                {"nearest": round(o.nearest, 3), "bearing_deg": round(o.nearest_bearing_deg, 1),
                 "left_deg": round(o.left_deg, 1), "right_deg": round(o.right_deg, 1),
                 "left_xy": [round(float(v), 3) for v in o.left_xy], "right_xy": [round(float(v), 3) for v in o.right_xy],
                 "width": round(o.width, 3), "height": None if not math.isfinite(o.height) else round(o.height, 3),
                 "tall": o.tall, "is_box": o.is_box} for o in self.obstacles])
        if not self.terrain_enabled:
            self.board.terrain_state(t, {"state": "DISABLED", "mode": "off", "reason": ""})
            return None
        intent = nav_cmd if nav_cmd is not None else None
        inp = SM.Inputs(t=t, yaw=self.imu_yaw(), level=True, obstacles=self.obstacles,
                        points=self.points if new_scan else None,
                        seen_sector=(-math.radians(100.0), math.radians(100.0)), scan_range=2.5, intent=intent,
                        ball_tracked=self.dodge_state in ("TRACK", "THREAT", "DODGE"),
                        dodge_event=self._dodge_moved)
        self._dodge_moved = False
        was_gauging = getattr(self, "_want_gauge", False)
        if was_gauging:
            inp.box = self._measure_box()
        out = self.fsm.tick(inp)
        self._want_gauge = out.want_gauge
        if out.plan_box is not None:
            b = out.plan_box
            slab = getattr(out, "plan_method", "") == "slab"
            # Body travel and time as terrain agility measured them in physics through its step-over node
            # (2026-11-26): a slab is scaled across, depth + 0.49 m from the usual 0.36 m stop, 81 s;
            # a 19 x 89 mm bar is stepped over, 0.58 m, 52 s.
            advance = (b.near_x - 0.36) + b.depth + 0.49 if slab else b.near_x + b.depth + 0.13
            duration = self.opt.crossing_s or (81.0 if slab else 52.0)
            self.cross = {"stage": "planning", "until": t + self.opt.plan_s, "advance": advance, "moved": 0.0,
                          "duration": duration, "slab": slab}
            self.crossed.add(getattr(self, "_gauged", 0))
            self.board.crossing(t, {"state": "planning"})
        status = {"state": out.state, "mode": self.fsm.mode,
                  "reason": self.fsm.hold_reason if self.fsm.state == SM.HOLD else "", "scan_ms": 0.0}
        e = self.fsm.estimate
        if e is not None and out.state not in (SM.CRUISE, SM.AVOID, SM.APPROACH, SM.GAUGE, SM.HOLD, SM.REPORT):
            status["box"] = {"near_x": round(e.near_x, 4), "height": round(e.height, 4), "depth": round(e.depth, 4),
                             "skew": round(e.skew, 4), "width": round(e.width, 4)}
        self.board.terrain_state(t, status)
        return out.cmd

    def _measure_box(self):
        """What the fine gauge would read, standing still in front of the nearest low obstacle."""
        x, y, th = self.pose
        if abs(math.cos(th)) < 0.3:
            return None
        best = None
        for k, ob in enumerate(self.cfg.course.obstacles):
            box = self.rects[k]
            if box.tall:
                continue
            near_x = (box.cx - box.depth / 2 - x) / math.cos(th)
            if 0.15 < near_x < 0.7 and (best is None or near_x < best[0]):
                best = (near_x, k, ob, box)
        if best is None:
            return None
        near_x, k, ob, box = best
        near_x += self.rng.gauss(0.0, 0.004)
        self._gauged = k
        # its ends in the body frame (left positive)
        y_left = (box.width / 2 - y) / max(0.3, math.cos(th)) - near_x * math.tan(th)
        y_right = (-box.width / 2 - y) / max(0.3, math.cos(th)) - near_x * math.tan(th)
        height, depth = float(ob.get("height", 0.02)), box.depth
        if ob.get("shape") == "round":
            # A round bar has no side face and no flat top. Terrain agility's gauge reads a 50 mm one as
            # "20 to 25 mm tall, 45 to 55 mm deep" (its plan.md), which its decision refuses: no foothold.
            height, depth = 0.45 * height, 1.0 * depth
        seen = dict(near_x=near_x, skew=-th + self.rng.gauss(0.0, 0.004),
                    height=height + self.rng.gauss(0.0, 0.0007), depth=depth + self.rng.gauss(0.0, 0.002),
                    width=box.width, y_left=y_left, y_right=y_right, columns=60, height_spread=0.001,
                    ends_seen=True, landing_floor=1.0,
                    height_upper=height + 0.004, extent=depth + 0.01, side_seen=1.0, far_seen=True)
        known = {f.name for f in fields(BoxMeasurement)}        # older terrain code has fewer fields
        return BoxMeasurement(**{k_: v for k_, v in seen.items() if k_ in known})

    def _crossing_step(self) -> None:
        cr, t = self.cross, self.t
        if cr["stage"] == "planning":
            if t >= cr["until"]:
                self.fsm.plan_result(True)
                cr.update(stage="crossing", t0=t)
                self.board.crossing(t, {"state": "started", "duration": cr["duration"], "advance": round(cr["advance"], 3),
                                        "plan": "plan: %d foot swings" % (40 if cr["slab"] else 33)})
                cr["next_progress"] = t
            return
        frac = min(1.0, (t - cr["t0"]) / cr["duration"])
        ease = 0.5 - 0.5 * math.cos(math.pi * frac)
        step = cr["advance"] * 1.02 * ease - cr["moved"]            # it lands a little long: the reckoning must cope
        cr["moved"] += step
        self.pose[0] += step * math.cos(self.pose[2])
        self.pose[1] += step * math.sin(self.pose[2])
        if t >= cr["next_progress"]:
            cr["next_progress"] = t + 0.5
            self.board.crossing(t, {"state": "progress", "progress": round(frac, 3), "stage": "play"})
        if frac >= 1.0:
            self.fsm.crossing_result(True)
            self.board.crossing(t, {"state": "done", "seconds": cr["duration"], "advance": round(cr["advance"], 3)})
            self.legs_back_at = t + self.cfg.reckon.restart_s - 0.5
            self.cross = None

    # ------------------------------------------------------------------ throwers and the dodge node
    _dodge_moved = False

    def _dodge(self):
        """Throwers, balls and the dodge reflex; returns the dodge node's velocity command or None."""
        c, t, opt = self.cfg.course, self.t, self.opt
        self.dodge_armed = bool(self.board.wanted["arm_dodge"])
        x, y, th = self.pose
        in_window = c.throw_from <= x <= c.throw_to and self.mission.state == ZONE2
        if opt.throws and in_window and t >= self.next_throw and not any(b["flying"] for b in self.balls):
            side = self.throw_side
            self.throw_side = -side
            self.next_throw = t + opt.throw_gap_s
            self.throws += 1
            lead = self.vel[0] * opt.flight_s * 0.8          # they aim a little ahead of a walking dog
            tx = x + lead * math.cos(th) + self.rng.gauss(0.0, opt.aim_sigma)
            ty = y + lead * math.sin(th) + self.rng.gauss(0.0, opt.aim_sigma)
            self.balls.append({"x0": c.thrower_along, "y0": side * c.thrower_side, "x1": tx, "y1": ty, "t0": t,
                               "t1": t + opt.flight_s, "flying": True, "side": side, "dodged": False, "hit": False,
                               "x": c.thrower_along, "y": side * c.thrower_side, "z": 1.0})
        cmd, tracks = None, []
        for b in self.balls:
            if not b["flying"]:
                continue
            f = (t - b["t0"]) / (b["t1"] - b["t0"])
            b["x"], b["y"] = b["x0"] + (b["x1"] - b["x0"]) * f, b["y0"] + (b["y1"] - b["y0"]) * f
            b["z"] = max(0.0, 1.0 + 3.2 * f - 4.2 * f * f)          # a lob: up, then down to the floor
            rel = (b["x"] - x, b["y"] - y)
            rng_m = math.hypot(*rel)
            az = math.degrees(wrap(math.atan2(rel[1], rel[0]) - th))
            if f > 0.2:
                tracks.append({"id": self.throws, "confirmed": f > 0.3, "range_m": round(rng_m, 2),
                               "azimuth_deg": round(az, 1), "elevation_deg": round(math.degrees(math.atan2(b["z"], rng_m)), 1),
                               "closing_speed": round(rng_m / max(0.05, b["t1"] - t), 2), "cams": ["elp_left" if az > 0 else "elp_right"]})
                if self.dodge_state == "SEARCH":
                    self.dodge_state = "TRACK"
            # where it will land, in the body frame
            lx = (b["x1"] - x) * math.cos(th) + (b["y1"] - y) * math.sin(th)
            ly = -(b["x1"] - x) * math.sin(th) + (b["y1"] - y) * math.cos(th)
            threat = abs(lx) < BODY_HALF_X + BALL_R + 0.10 and abs(ly) < BODY_HALF_Y + BALL_R + 0.10
            if threat and self.dodge_state in ("TRACK", "SEARCH"):
                self.dodge_state = "THREAT"
            if threat and self.burst is None and b["t1"] - t <= opt.dodge_react_s and not b.get("judged"):
                b["judged"] = True
                # across the ball's ground track, away from where it lands
                ux, uy = b["x1"] - b["x0"], b["y1"] - b["y0"]
                n = math.hypot(ux, uy) or 1.0
                # the ball's direction in the body frame, and its left-hand normal
                bx = (ux * math.cos(th) + uy * math.sin(th)) / n
                by = (-ux * math.sin(th) + uy * math.cos(th)) / n
                left = (-by, bx)
                side_of_landing = lx * left[0] + ly * left[1]
                sign = -1.0 if side_of_landing > 0 else 1.0
                dx, dy = sign * left[0], sign * left[1]
                vx = (1.0 if dx > 0 else -0.8) if abs(dx) >= 0.25 else 0.0
                vy = math.copysign(0.5, dy) if abs(dy) >= 0.25 else 0.0
                if vx and vy:
                    vx, vy = 0.8 * vx, 0.8 * vy
                event = {"t": round(t, 3), "event": "dodge", "dry_run": not self.dodge_armed, "late": False,
                         "threat": {"tau": round(b["t1"] - t, 2), "azimuth_deg": round(az, 1)},
                         "plan": {"vx": vx, "vy": vy, "mode": "perpendicular"}}
                self.board.dodge_event(t, event)
                if self.dodge_armed:
                    self.burst = ((vx, vy, 0.0), t + 0.42, t + 0.42 + 0.6)
                    self.dodge_state = "DODGE"
                    self._dodge_moved = True
                    b["dodged"] = True
            if t >= b["t1"]:
                b["flying"] = False
                b["x"], b["y"], b["z"] = b["x1"], b["y1"], 0.0
                hit = bool(abs(lx) < BODY_HALF_X + BALL_R and abs(ly) < BODY_HALF_Y + BALL_R)
                b["hit"] = hit
                if hit:
                    self.hits += 1
                elif b["dodged"]:
                    self.dodged += 1
                elif not b.get("judged"):
                    self.board.dodge_event(t, {"t": round(t, 3), "event": "throw_seen"})
        if self.burst is not None:
            burst_cmd, until, recover = self.burst
            if t < until:
                cmd = burst_cmd
            elif t < recover:
                self.dodge_state = "RECOVER"
                cmd = (0.0, 0.0, 0.0)
            else:
                self.burst = None
        if self.burst is None and not tracks and self.dodge_state != "SEARCH":
            self.dodge_state = "SEARCH"
        self.board.dodge_state(t, {
            "state": self.dodge_state, "armed": self.dodge_armed, "can_dodge": self.cross is None,
            "cannot_reason": "" if self.cross is None else "no walking controller active (mid-crossing)",
            "tracks": tracks, "stats": {"dodges": self.dodged + self.hits, "throws": self.throws},
            "cameras": {"fisheye": {"fps": 28.0, "latency_ms": 45.0, "alive": True},
                        "elp_left": {"fps": 30.0, "latency_ms": 38.0, "alive": True},
                        "elp_right": {"fps": 30.0, "latency_ms": 38.0, "alive": True}}})
        return cmd

    # ------------------------------------------------------------------ one step
    def step(self) -> None:
        t = self.t = round(self.t + DT, 4)
        board = self.board
        self.imu_drift += math.radians(self.opt.yaw_drift_deg_min) / 60.0 * DT
        board.imu(t, self.imu_yaw(), 0.0)
        board.walking_active(t, self.cross is None or self.cross["stage"] == "planning")
        board.sound_alive(t, {"alive": True, "mics": 4})
        board.keyword_alive(t, {"alive": True, "device": "simulated", "level_db": [
            -22.0 + self.rng.uniform(-3, 3) if t < self.survivor_says_until else -55.0 + self.rng.uniform(-2, 2)
            for _ in range(4)]})
        board.voice(t, 1)
        self._survivor()

        # the mission
        inp = board.inputs(t)
        out = self.mission.tick(inp)
        board.record(t, out, self.mission)
        self._hear_pupper(out.events)
        if out.cmd is not None:
            self.sources["/nav_cmd_vel"] = (t, out.cmd)
        nav_fresh = self.sources["/nav_cmd_vel"]
        nav_intent = nav_fresh[1] if t - nav_fresh[0] < 0.4 else None

        # the reflexes
        tcmd = self._terrain(nav_intent)
        if tcmd is not None:
            self.sources["/terrain_cmd_vel"] = (t, tcmd)
        dcmd = self._dodge()
        if dcmd is not None:
            self.sources["/dodge_cmd_vel"] = (t, dcmd)
        if t < self.nudge_until:
            self.sources["/teleop_cmd_vel"] = (t, self.nudge_cmd)

        # the mux: the first input, in priority order, heard from in the last half second
        active, cmd = "none", (0.0, 0.0, 0.0)
        for name in MUX_ORDER:
            ts, c = self.sources[name]
            if c is not None and t - ts <= MUX_TIMEOUT:
                active, cmd = name, c
                break
        board.mux(t, active)
        board.cmd_vel(t, *cmd)

        # the walk
        legs = self.cross is None and t >= self.legs_back_at
        k = self.opt.speed_factor
        target = np.array([cmd[0] * k, cmd[1] * k, cmd[2]]) if legs else np.zeros(3)
        self.vel += (target - self.vel) * min(1.0, DT / self.opt.walk_lag)
        if not legs:
            self.vel[:] = 0.0
        th = self.pose[2]
        self.pose[0] += (self.vel[0] * math.cos(th) - self.vel[1] * math.sin(th)) * DT
        self.pose[1] += (self.vel[0] * math.sin(th) + self.vel[1] * math.cos(th)) * DT
        self.pose[2] = wrap(th + self.vel[2] * DT)
        if math.hypot(self.pose[0] - self.path[-1][0], self.pose[1] - self.path[-1][1]) > 0.03:
            self.path.append((round(float(self.pose[0]), 3), round(float(self.pose[1]), 3)))
        p = self.pose[:2]
        for k, rect in enumerate(self.rects[:-1]):      # the obstacles it did not cross: how close it came
            if k not in self.crossed:
                self.min_clear["tall"] = min(self.min_clear["tall"], rect.distance(p))
        self.min_clear["cover"] = min(self.min_clear["cover"], self.rects[-1].distance(p))

    # ------------------------------------------------------------------ what Mission Control draws
    def snapshot(self) -> dict:
        x, y, th = (float(v) for v in self.pose)
        truth = {
            "robot": [round(x, 3), round(y, 3), round(th, 4)],
            "path": self.path[-400:],
            "survivor": list(self.survivor),
            "survivor_says": self.survivor_says if self.t < self.survivor_says_until else "",
            "balls": [{"x": round(b["x"], 3), "y": round(b["y"], 3), "z": round(b["z"], 2), "flying": b["flying"],
                       "hit": b["hit"], "dodged": b["dodged"]} for b in self.balls[-12:]],
            "throws": self.throws, "hits": self.hits, "dodged": self.dodged,
            "true_bearing_deg": round(math.degrees(self.true_bearing()), 1),
            "crossing": None if self.cross is None else self.cross["stage"],
        }
        return self.board.snapshot(self.t, self.mission, mode="sim", extra={"truth": truth})

    # ------------------------------------------------------------------ a whole run, for tests
    def run(self, max_time: float = 420.0, start: str = "start!") -> dict:
        self.step()
        self.command("motion_on")
        self.command(start)
        events = []
        seen = 0
        while self.t < max_time:
            self.step()
            log = self.board.log
            events += [(e["t"], e["event"]) for e in log[seen:]]
            seen = len(log)
            if self.mission.state == RESCUE and self.t - self.mission.t_state > 2.0:
                break
            if self.mission.state in ("HOLD", "DONE", IDLE) and self.t > 5.0:
                break
        c = self.cfg.course
        x, y, th = (float(v) for v in self.pose)
        nose = x + 0.2 * math.cos(th)
        in_front = abs(y) <= c.cover_width / 2 - 0.05 and c.cover_near - 1.2 <= nose <= c.cover_near + 0.02
        st = self.mission.status()
        return {"state": self.mission.state, "reached": bool(self.mission.state == RESCUE and in_front),
                "time": round(self.t, 1), "x": round(x, 2), "y": round(y, 2), "est_s": st["s"], "est_d": st["d"],
                "throws": self.throws, "hits": self.hits, "dodged": self.dodged, "reason": st["reason"],
                "clear_tall": round(self.min_clear["tall"], 2), "clear_cover": round(self.min_clear["cover"], 2),
                "crossings": sum(n == "crossed" for _, n in events), "walk_arounds": sum(n == "walked_around" for _, n in events),
                "phrase_said": self.said_phrase, "events": events}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", type=int, default=1)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--moe", type=float, default=None, help="sound module margin of error, degrees (default: the config's)")
    ap.add_argument("--no-second-call", action="store_true", help="the survivor does not call again at 2 m")
    ap.add_argument("--no-throws", action="store_true")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()
    cfg = load_config(args.config)
    results = []
    for k in range(args.runs):
        opt = SimOptions(seed=args.seed + k, moe_deg=args.moe, second_call=not args.no_second_call,
                         throws=not args.no_throws)
        sim = CourseSim(cfg, opt)
        res = sim.run()
        results.append(res)
        if args.runs == 1:
            for t, name in res["events"]:
                line = EV.LINES.get(name, "")
                print(f"  {t:6.1f} s  {name:16s} {line}")
        print(f"run {args.seed + k}: {'REACHED the cover' if res['reached'] else 'DID NOT: ' + res['state'] + ' ' + res['reason']}"
              f" in {res['time']:.0f} s, stopped at x {res['x']:.2f} m, {res['y']:+.2f} m off the line "
              f"(believed s {res['est_s']:.2f}); crossed {res['crossings']}, walked around {res['walk_arounds']}; "
              f"{res['throws']} throws, {res['hits']} hits; closest {res['clear_tall']:.2f} m to an obstacle it went around")
    if args.runs > 1:
        ok = sum(r["reached"] for r in results)
        off = sorted(abs(r["y"]) for r in results)
        throws, hits = sum(r["throws"] for r in results), sum(r["hits"] for r in results)
        print(f"\nCrossings per run: {sorted(set(r['crossings'] for r in results))}; walk-arounds per run: "
              f"{sorted(set(r['walk_arounds'] for r in results))}.")
        print(f"{ok} of {len(results)} runs ended in front of the cover. Sideways miss: median {off[len(off) // 2]:.2f} m, "
              f"worst {off[-1]:.2f} m. Balls: {hits} hits in {throws} throws. "
              f"Median time {sorted(r['time'] for r in results)[len(results) // 2]:.0f} s.")


if __name__ == "__main__":
    main()
