"""Reaching the survivor: the approach / arrival state machine (plan, "Reaching the survivor").

Pure logic, ticked about 20 times a second by robot/approach_node.py:

    out = mission.tick(Inputs(now=..., status=<perception status>, mux_active=..., walking=..., ...))
    out.cmd       (vx, vy, wz) to publish on /survivor_cmd_vel, or None = publish nothing
    out.state     SCAN, CANDIDATE, LOOK, LOCKED, APPROACH, BLOCKED, REACQUIRE, ARRIVED, CELEBRATE, STAY or PAUSED
    out.events    [{"event": "blocked" | "arrived", ...}]
    out.actions   things for the node to do: "bark" (a sound), and through the robot server
                  "clear_queue", "stand", "celebrate" (the bounce), "walk". The node answers
                  "celebrate" with mission.action_done("celebrate", ok) when the trick has finished.

It never gives up on a pinned survivor by itself: blocked -> detour, out of view -> walk to the
remembered spot, nothing there -> look around, then round again. Only a cancelled pin ends it.
With motion=False (the default on the robot until you say otherwise) it goes through the same
states but asks for nothing that moves the robot; the node also drops its velocity commands.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

IDLE, LOOK, PREPARE, APPROACH, DETOUR, REACQUIRE, ARRIVED, CELEBRATE, STAY = (
    "IDLE", "LOOK", "PREPARE", "APPROACH", "DETOUR", "REACQUIRE", "ARRIVED", "CELEBRATE", "STAY")
REPORT = {PREPARE: "LOCKED", DETOUR: "BLOCKED"}       # internal name -> the plan's name


@dataclass
class Inputs:
    now: float
    status: dict | None = None        # perception status (survivor_node's /survivor/perception)
    status_age: float = 0.0
    mux_active: str = "none"          # /cmd_vel_mux/active_source
    walking: bool = True              # a walking controller holds the legs
    fallen: bool = False
    level: bool = True
    terrain_state: str | None = None  # terrain agility's state; None = it is not running
    terrain_blocked: bool = False     # terrain agility just reported no way past
    dodge_quiet_s: float = 1e9        # seconds since the dodge reflex last did anything
    server: bool = True               # the robot server (queue, tricks) can be reached


@dataclass
class Outputs:
    state: str = "SCAN"
    cmd: tuple | None = None
    events: list = field(default_factory=list)
    actions: list = field(default_factory=list)
    note: str = ""


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


class Mission:
    def __init__(self, cfg: dict, motion: bool = False, drive: str | None = None):
        self.cfg, self.m, self.a = cfg, cfg["mission"], cfg["arrival"]
        self.motion = bool(motion)
        self.drive = drive or self.m.get("drive", "direct")
        self.state = IDLE
        self.t_state = 0.0
        self.target_id = None
        self.celebrated: set = set()
        self.look_done: dict = {}         # track id -> when we last turned to look at it
        self.look_id = None
        self._reset_target()

    def _reset_target(self) -> None:
        self.cleared = False
        self.t_stand = -1e9
        self.t_walking = None
        self.stall = None                 # (since, range then)
        self.blocks = 0
        self.block_range = None
        self.detour = None                # (phase, until, direction)
        self.search = None                # (phase, since, spot index)
        self.away_since = None
        self.cel = None                   # celebration: {"phase", "since"}
        self.pending: dict = {}           # action name -> result (None while running)

    def set_motion(self, on: bool) -> None:
        self.motion = bool(on)

    def action_done(self, name: str, ok: bool = True) -> None:
        self.pending[name] = bool(ok)

    def _go(self, state: str, now: float) -> None:
        if state != self.state:
            self.state, self.t_state = state, now
            self.stall = None

    # ------------------------------------------------------------------ helpers
    def _steer(self, bearing: float, rng: float, speed: float) -> tuple:
        """Face, then go: turn in place when the target is well off the nose, else walk and steer."""
        if abs(bearing) > math.radians(float(self.m["face_deg"])):
            rate = clamp(float(self.m["yaw_kp"]) * abs(bearing), 0.4, float(self.m["turn_rate"]))
            return (0.0, 0.0, math.copysign(rate, bearing))
        wz = clamp(float(self.m["yaw_kp"]) * bearing, -float(self.m["turn_rate"]), float(self.m["turn_rate"]))
        return (speed, 0.0, wz)

    def _speed(self, rng: float) -> float:
        return float(self.m["speed_far"] if rng > float(self.m["slow_range_m"]) else self.m["speed_near"])

    def _mine(self, inp: Inputs) -> bool:
        return inp.mux_active == self.m["cmd_topic"]

    def _terrain_running(self, inp: Inputs) -> bool:
        return inp.terrain_state is not None

    def _paused(self, inp: Inputs) -> str:
        if inp.mux_active in self.m["joystick_topics"]:
            return "joystick"
        if inp.fallen:
            return "fallen: a person has to set Pupper on its feet"
        if inp.terrain_state in self.m["terrain_pause_states"]:
            return "terrain agility is crossing an obstacle"
        return ""

    @staticmethod
    def _local_to_body(pose: dict, x: float, y: float) -> tuple:
        yaw = math.radians(pose["yaw_deg"])
        dx, dy = x - pose["x"], y - pose["y"]
        bx, by = math.cos(yaw) * dx + math.sin(yaw) * dy, -math.sin(yaw) * dx + math.cos(yaw) * dy
        return math.atan2(by, bx), math.hypot(bx, by)

    # ------------------------------------------------------------------ the tick
    def tick(self, inp: Inputs) -> Outputs:
        out = Outputs()
        st = inp.status
        if st is None or inp.status_age > float(self.m["status_stale_s"]):
            self._go(IDLE, inp.now)
            self.target_id = None
            out.state, out.note = "SCAN", "no word from the survivor node"
            return out
        tgt = st.get("target")
        if tgt is None:
            if self.target_id is not None:                 # the pin was cancelled
                self.target_id = None
                self._reset_target()
                self._go(IDLE, inp.now)
            return self._idle(inp, st, out)
        if tgt["id"] != self.target_id:
            self.target_id = tgt["id"]
            self._reset_target()
            self._go(PREPARE, inp.now)
            out.actions.append("bark")                     # locked: say so at once, no network needed
        handler = {PREPARE: self._prepare, APPROACH: self._approach, DETOUR: self._detour, REACQUIRE: self._reacquire,
                   ARRIVED: self._arrived, CELEBRATE: self._celebrate, STAY: self._stay}.get(self.state)
        if handler is None:                                # was looking around when the lock came
            self._go(PREPARE, inp.now)
            handler = self._prepare
        if self.state in (APPROACH, DETOUR, REACQUIRE):
            why = self._paused(inp)
            if not why and self.motion and not inp.walking:
                self._go(PREPARE, inp.now)                 # legs were taken away (trick, relax): get them back
                handler = self._prepare
            elif why:
                self.stall = None
                out.state, out.note = "PAUSED", why
                return out
        handler(inp, st, tgt, out)
        out.state = REPORT.get(self.state, self.state)
        return out

    # ------------------------------------------------------------------ nobody pinned
    def _idle(self, inp: Inputs, st: dict, out: Outputs) -> Outputs:
        m, now = self.m, inp.now
        tracks = {t["id"]: t for t in st.get("tracks", [])}
        if self.state == LOOK:
            tr = tracks.get(self.look_id)
            done = (tr is None or abs(tr["bearing_deg"]) <= float(m["look_done_deg"])
                    or now - self.t_state > float(m["look_max_s"]) or self._paused(inp) or not inp.walking)
            if done:
                self.look_done[self.look_id] = now
                self._go(IDLE, now)
            else:
                out.cmd = (0.0, 0.0, math.copysign(float(m["look_rate"]), tr["bearing_deg"]))
                out.state = "LOOK"
                return out
        look = st.get("look")
        if (look and not self._paused(inp) and inp.walking
                and now - self.look_done.get(look["id"], -1e9) >= float(m["look_cooldown_s"])):
            self.look_id = look["id"]
            self._go(LOOK, now)
            out.cmd = (0.0, 0.0, math.copysign(float(m["look_rate"]), look["bearing_deg"]))
            out.state = "LOOK"
            return out
        self._go(IDLE, now)
        out.state = "CANDIDATE" if st.get("phase") == "CANDIDATE" else "SCAN"
        return out                                         # cmd None: whoever else is driving keeps driving

    # ------------------------------------------------------------------ on lock: clear the way
    def _prepare(self, inp: Inputs, st: dict, tgt: dict, out: Outputs) -> None:
        now = inp.now
        if not self.motion:                                # dry run: nothing to get ready
            if now - self.t_state >= 0.5:
                self._go(APPROACH, now)
            return
        if not self.cleared and inp.server:
            self.cleared = True
            out.actions.append("clear_queue")              # Gemini's queued moves, follow mode
        if inp.walking and not self._paused(inp):
            out.cmd = (0.0, 0.0, 0.0)                      # hold still: zeros keep everything below us in the mux quiet
        if inp.fallen:
            out.note = "fallen: a person has to set Pupper on its feet"
            return
        if not inp.walking:
            self.t_walking = None
            if inp.server and now - self.t_stand >= float(self.m["prepare_retry_s"]):
                self.t_stand = now
                out.actions.append("stand")
            out.note = "waiting for the walking controller"
            return
        if self.t_walking is None:
            self.t_walking = now
        ready = float(self.a["walk_ready_s"]) if now - self.t_stand < 30.0 else 0.3
        if now - max(self.t_walking, self.t_stand) >= ready or (self.t_stand < 0 and now - self.t_walking >= 0.3):
            self._go(APPROACH, now)

    # ------------------------------------------------------------------ walking up
    def _approach(self, inp: Inputs, st: dict, tgt: dict, out: Outputs) -> None:
        m, now = self.m, inp.now
        b, r = math.radians(tgt["bearing_deg"]), float(tgt["range_m"])
        if not tgt["visible"]:
            self._go(REACQUIRE, now)
            self.search = ("GOTO", now, 0)
            return self._reacquire(inp, st, tgt, out)
        if r <= float(m["stop_m"]) or tgt.get("box_frac", 0.0) >= float(m["hard_stop_box_frac"]):
            self._go(ARRIVED, now)
            out.cmd = (0.0, 0.0, 0.0)
            out.actions.append("bark")
            out.events.append({"event": "arrived", "t": round(now, 3), "id": tgt["id"], "range_m": round(r, 2)})
            return
        if self.drive == "route" and r > float(m["handover_m"]):
            out.note = "route mapping drives until %.1f m" % float(m["handover_m"])
            return                                         # publish nothing
        if self.block_range is not None and r <= self.block_range - 0.5:
            self.blocks, self.block_range = 0, None        # real progress since the last blockage
        out.cmd = self._steer(b, r, self._speed(r))
        # Blocked? Terrain agility says so, or (when it is not running) we push forward and get no closer.
        blocked = inp.terrain_blocked
        if not self._terrain_running(inp) and self._mine(inp) and out.cmd[0] >= float(m["stall_speed"]):
            if self.stall is None or r <= self.stall[1] - float(m["stall_progress_m"]):
                self.stall = (now, r)
            elif now - self.stall[0] >= float(m["stall_s"]):
                blocked = True
        else:
            self.stall = None
        if blocked:
            self.blocks += 1
            self.block_range = r if self.block_range is None else min(self.block_range, r)
            side = 1.0 if (b >= 0) == (self.blocks % 2 == 1) else -1.0      # first toward the survivor's side, then alternate
            self.detour = ("BACK", now + float(m["detour_back_m"]) / float(m["detour_back_speed"]), side)
            self._go(DETOUR, now)
            out.events.append({"event": "blocked", "t": round(now, 3), "id": tgt["id"], "range_m": round(r, 2),
                               "side": "left" if side > 0 else "right"})
            out.cmd = (-float(m["detour_back_speed"]), 0.0, 0.0)

    def _detour(self, inp: Inputs, st: dict, tgt: dict, out: Outputs) -> None:
        """Blind fallback: back up, sidestep, try again; other side next time, then wider."""
        m, now = self.m, inp.now
        phase, until, side = self.detour
        if phase == "BACK":
            if now < until:
                out.cmd = (-float(m["detour_back_speed"]), 0.0, 0.0)
                return
            widths = [float(w) for w in m["detour_widths_m"]]
            width = widths[min((self.blocks - 1) // 2, len(widths) - 1)]
            self.detour = ("SIDE", now + width / float(m["detour_side_speed"]), side)
            phase, until = "SIDE", self.detour[1]
        if now < until:
            out.cmd = (0.0, side * float(m["detour_side_speed"]), 0.0)
            return
        self._go(APPROACH, now)
        self._approach(inp, st, tgt, out)

    # ------------------------------------------------------------------ out of view
    def _reacquire(self, inp: Inputs, st: dict, tgt: dict, out: Outputs) -> None:
        m, now = self.m, inp.now
        if tgt["visible"]:
            self._go(APPROACH, now)
            return self._approach(inp, st, tgt, out)
        if self.drive == "route":
            out.note = "out of view: route mapping drives to the pin"
            return
        if self.search is None:
            self.search = ("GOTO", now, 0)
        phase, since, spot = self.search
        b, r = math.radians(tgt["bearing_deg"]), float(tgt["range_m"])
        if phase == "GOTO":                                # rung 4: head for the remembered spot
            if r > float(m["pin_reached_m"]):
                out.cmd = self._steer(b, r, float(m["speed_near"]))
                return
            self.search = phase, since, spot = ("WAIT", now, 0)
        if phase == "WAIT":                                # rung 5: first just look (three cameras see all round)
            out.cmd = (0.0, 0.0, 0.0)
            if now - since >= float(m["search_wait_s"]):
                self.search = ("SPIN", now, 0)
            return
        if phase == "SPIN":                                # one slow turn, for anyone cut off in the rear views
            out.cmd = (0.0, 0.0, float(m["search_turn_rate"]))
            if now - since >= 2.0 * math.pi / float(m["search_turn_rate"]):
                self.search = ("SPOT", now, 0)
            return
        # SPOT: four places a metre around the pin, then back to the pin and round again (rung 6)
        radius = float(m["search_radius_m"])
        ang = spot * math.pi / 2.0
        sb, sr = self._local_to_body(st["pose"], tgt["x"] + radius * math.cos(ang), tgt["y"] + radius * math.sin(ang))
        if sr <= 0.3 or now - since > 8.0:
            self.search = ("SPOT", now, spot + 1) if spot < 3 else ("GOTO", now, 0)
            out.cmd = (0.0, 0.0, 0.0)
            return
        out.cmd = self._steer(sb, sr, float(m["speed_near"]))

    # ------------------------------------------------------------------ arrival
    def _arrived(self, inp: Inputs, st: dict, tgt: dict, out: Outputs) -> None:
        out.cmd = (0.0, 0.0, 0.0)
        if inp.now - self.t_state >= float(self.a["hold_s"]):
            self._go(CELEBRATE, inp.now)
            self.cel = {"phase": "WAIT", "since": inp.now}

    def _celebrate(self, inp: Inputs, st: dict, tgt: dict, out: Outputs) -> None:
        a, now, c = self.a, inp.now, self.cel
        if c["phase"] == "WAIT":
            out.cmd = (0.0, 0.0, 0.0)
            skip = ("" if self.motion else "dry run") or ("already celebrated" if tgt["id"] in self.celebrated else "") \
                or ("" if inp.server else "robot server not reachable") or ("" if inp.level else "not standing level") \
                or ("survivor too close" if tgt["visible"] and tgt["range_m"] < float(self.m["backoff_m"]) else "") \
                or ("" if inp.walking else "not in the walking controller")
            if not skip and inp.dodge_quiet_s < float(a["clear_air_s"]):
                if now - c["since"] <= float(a["clear_air_wait_s"]):
                    out.note = "waiting for the air to clear before the bounce"
                    return
                skip = "balls still flying"
            self.celebrated.add(tgt["id"])
            if skip:
                out.note = "no bounce: " + skip
                self._go(STAY, now)
                return
            self.pending.pop("celebrate", None)
            out.actions.append("celebrate")
            self.cel = {"phase": "BOUNCE", "since": now}
            return
        if c["phase"] == "BOUNCE":                         # the trick controller has the legs: say nothing to the mux
            done = self.pending.get("celebrate")
            if done is None and now - c["since"] < float(a["trick_timeout_s"]):
                return
            self.cel = {"phase": "WALK", "since": now, "asked": -1e9, "walking_since": None}
            return
        # WALK: back to the walking controller. The robot server hands the legs back itself after a
        # trick that ends standing; if it has not (or the trick never ran), ask for it.
        if inp.walking:
            if c["walking_since"] is None:
                c["walking_since"] = now
            if now - c["walking_since"] >= (float(a["walk_ready_s"]) if now - c["asked"] < 30.0 else 0.5):
                self._go(STAY, now)
            return
        c["walking_since"] = None
        if inp.server and not inp.fallen and now - c["since"] >= 1.0 and now - c["asked"] >= float(self.m["prepare_retry_s"]):
            c["asked"] = now
            out.actions.append("walk")

    def _stay(self, inp: Inputs, st: dict, tgt: dict, out: Outputs) -> None:
        m, now = self.m, inp.now
        out.cmd = (0.0, 0.0, 0.0)                          # zeros hold the mux: nothing below us moves Pupper away
        if self._paused(inp):
            out.cmd = None
            return
        if not tgt["visible"]:
            self.away_since = None
            if tgt["since_seen_s"] > 3.0:
                self._go(REACQUIRE, now)
                self.search = ("GOTO", now, 0)
            return
        r = float(tgt["range_m"])
        if r < float(m["backoff_m"]):
            out.cmd = (-float(m["backoff_speed"]), 0.0, 0.0)
        if r > float(m["stay_leave_m"]):
            self.away_since = now if self.away_since is None else self.away_since
            if now - self.away_since >= float(m["stay_leave_s"]):
                self.away_since = None
                self._go(APPROACH, now)
        else:
            self.away_since = None
