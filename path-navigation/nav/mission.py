"""The demo mission: one state machine that strings the other components together.

    IDLE -> PREFLIGHT -> SEEK -> ORIENT -> CONFIRM -> ZONE1 -> REFIX -> ZONE2 -> ARRIVE -> RESCUE -> DONE
                          ^_______|  (fix rejected)
    any active state <-> PAUSED (joystick, e-stop, fall, operator)     any active state -> HOLD (needs a person)

It decides which zone Pupper is in, where the line is, and which reflexes are
switched on. It never decides a dodge or a footstep: projectile dodging and
terrain agility sit above it in the velocity mux and interrupt it as designed.

`tick(Inputs)` is called 20 times a second and returns the velocity to publish
(or None for nothing at all), the events that happened, and the settings it
wants for the two reflexes. Nothing in here knows about ROS.

Two rules of the mux shape the code:
  - any message, even zeros, takes the mux input. So a state that only listens
    publishes nothing (after half a second of zeros to brake);
  - terrain agility acts only while a driver is asking to move. So a state
    that wants to move keeps publishing while terrain or dodging has the legs.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

from . import events as EV
from .config import DemoConfig
from .heading import Fix, FixCollector, circular_mean, diff, wrap
from .reckon import Reckoner
from .steer import line_command, turn_command

IDLE, PREFLIGHT, SEEK, ORIENT, CONFIRM, ZONE1, REFIX, ZONE2, ARRIVE, RESCUE, DONE, PAUSED, HOLD = (
    "IDLE", "PREFLIGHT", "SEEK", "ORIENT", "CONFIRM", "ZONE1", "REFIX", "ZONE2", "ARRIVE", "RESCUE", "DONE",
    "PAUSED", "HOLD")

RUN_ORDER = [SEEK, ORIENT, CONFIRM, ZONE1, REFIX, ZONE2, ARRIVE, RESCUE]
ACTIVE = {PREFLIGHT, SEEK, ORIENT, CONFIRM, ZONE1, REFIX, ZONE2, ARRIVE}
LISTENING = {SEEK, CONFIRM, REFIX}
TERRAIN_IDLE = {"", "CRUISE", "REPORT", "DISABLED", "OFF"}
JOYSTICK = ("/teleop_cmd_vel", "/joystick_cmd_vel")
ZERO = (0.0, 0.0, 0.0)


@dataclass
class Inputs:
    t: float                                    # seconds, any steady clock
    yaw: float = 0.0                            # rad, IMU heading (any fixed zero)
    imu_ok: bool = True
    tilt_deg: float = 0.0
    cmd_vel: tuple | None = None                # what the mux last sent to the legs; None if nothing lately
    mux_source: str = ""                        # the mux input that has the legs
    bearings: list = field(default_factory=list)    # new since the last tick: (rad from the nose, left positive, confidence)
    keyword: str = ""                           # the phrase variant just heard, "" for none
    terrain: dict = field(default_factory=dict)     # {"alive", "state", "mode", "reason", "box": {...} | None, "scan_fresh"}
    crossing: list = field(default_factory=list)    # new /terrain/crossing_status messages (dicts)
    obstacles: list = field(default_factory=list)   # /terrain/obstacles, if fresh
    dodge: dict = field(default_factory=dict)       # {"alive", "state", "armed", "can_dodge"}
    dodge_events: list = field(default_factory=list)
    estop: bool = False                         # the gamepad e-stop was pressed since the last tick
    alive: dict = field(default_factory=dict)       # imu, walking, sound, keyword, terrain, dodge, voice -> bool
    commands: list = field(default_factory=list)    # operator: start, start!, pause, resume, end, abort, phrase,
    #                                                 motion_on, motion_off, goto:<STATE>, bearing:<degrees>


@dataclass
class Outputs:
    cmd: tuple | None = None                    # (vx, vy, wz) to publish on /nav_cmd_vel, or None = publish nothing
    events: list = field(default_factory=list)  # [(name, details dict)]
    arm_dodge: bool = False                     # what the dodge node should be
    terrain_mode: str = "off"                   # full | avoid | report | off
    state: str = IDLE


class Mission:
    def __init__(self, cfg: DemoConfig | None = None, motion: bool = False):
        self.cfg = cfg or DemoConfig()
        self.motion = bool(motion)              # False: dry run. Listens, reports, publishes no velocity
        r = self.cfg.reckon
        self.reck = Reckoner(r.forward_factor, r.sideways_factor, r.lag_s)
        self.state = IDLE
        self.t = 0.0
        self.t_state = 0.0
        self.t_run = None
        self._events: list = []
        self._reset_run()

    # ------------------------------------------------------------------ bookkeeping
    def _reset_run(self) -> None:
        s = self.cfg.sound
        self.collector = FixCollector(s.min_bearings, math.radians(s.spread_deg), s.window_s)
        self.heard: list = []                   # recent bearings, for the display: [t, heading, used]
        self.first_fix: Fix | None = None
        self.target = None                      # rad, start frame: where the cry is
        self.reorients = 0
        self.quiet_until = -1e9                 # Pupper is talking until then: do not listen
        self.still_since = None
        self.brake_until = -1e9
        self.settle_t = None
        self.listen_from = None
        self.no_sound_said = -1e9
        self.seek_turn = None
        self.operator_bearing = None
        # zone 1
        self.crossing_active = False
        self.crossing_progress = 0.0
        self.crossing_advance = self.cfg.reckon.crossing_advance
        self.crossing_counted = 0.0
        self.crossing_deadline = None           # a crossing that has not reported its end by then is lost
        self.restart_until = -1e9
        self.passed = 0                         # obstacles of the course behind it: crossed or walked around
        self.landmark_due = False               # the step-over node has just said it is planning a crossing
        self.crossing_planning = False
        self.terrain_prev = ""
        self.terrain_idle_since = None
        self.terrain_hold_since = None
        self.terrain_holds: list = []           # when terrain agility last gave up on a way round
        self.back_off_until = -1e9              # reversing a little to give it a second look
        self.back_offs = 0
        self.avoid_next = False                 # terrain agility got stuck at an obstacle: walk around it from further back
        self.progress_mark = None               # (time, s): the last time it had clearly moved on
        self.said_stepping = False
        # zone 2
        self.dodges = 0
        self.last_dodge_said = -1e9
        self.blocked_since = None
        self.blocked_avoiding = False
        self.cover_seen = False
        self.cover_range = None
        self.keyword_ignored = 0
        # arrive
        self.arrived_said = -1e9
        self.gave_up = False
        self.arrive_why = ""
        # pause / hold
        self.resume_state = None
        self.reason = ""
        self.joystick_last = -1e9
        self.missing: list = []
        self.checks: dict = {}
        self.would = None

    def _go(self, state: str) -> None:
        self.state = state
        self.t_state = self.t
        self.settle_t = None
        self.progress_mark = None
        self.listen_from = None
        if state in LISTENING or state in (ARRIVE, RESCUE, PAUSED, HOLD, DONE):
            self.brake_until = self.t + self.cfg.walk.brake_s
        if state in LISTENING:
            self.collector.clear()
            self.operator_bearing = None

    def _say(self, name: str, **details) -> None:
        self._events.append((name, details))
        if EV.is_spoken(name):
            self.quiet_until = max(self.quiet_until, self.t + EV.speak_time(name))

    def _note(self, name: str, **details) -> None:
        self._events.append((name, details))

    @property
    def quiet(self) -> bool:
        """Pupper is speaking (or has only just stopped): the microphones cannot be trusted."""
        return self.t < self.quiet_until

    # ------------------------------------------------------------------ operator
    def _preflight(self, inp: Inputs) -> list:
        self.checks = {name: bool(inp.alive.get(name, False)) for name in
                       ["imu", "walking", "sound", "keyword", "terrain", "dodge", "voice"]}
        return [n for n in self.cfg.safety.require if not self.checks.get(n, False)]

    def _command(self, text: str, inp: Inputs) -> None:
        text = text.strip()
        if text == "motion_on":
            self.motion = True
            self._note("motion", on=True)
        elif text == "motion_off":
            self.motion = False
            self._note("motion", on=False)
        elif text in ("start", "start!"):
            if self.state not in (IDLE, DONE):
                return
            self.missing = self._preflight(inp)
            blocking = [m for m in self.missing if m == "imu" or text == "start"]
            if blocking:
                self._note("preflight_failed", missing="+".join(blocking))
                return
            self._reset_run()
            self.missing = self._preflight(inp)
            self.reck.reset()
            self.t_run = self.t
            self._go(PREFLIGHT)
            self._say("start")
        elif text == "pause":
            if self.state in ACTIVE:
                self._pause("operator")
        elif text == "resume":
            if self.state in (PAUSED, HOLD) and self.resume_state:
                back, self.resume_state = self.resume_state, None
                self._go(back)
                self._note("resumed", state=back)
            elif self.state == RESCUE and self.resume_state:
                # the phrase was heard by mistake: back to where it was
                back, self.resume_state = self.resume_state, None
                self._go(back)
                self._note("resumed", state=back)
        elif text in ("end", "abort"):
            if self.state == RESCUE:
                self._note("stand_down")
                self._go(DONE)
            elif self.state not in (IDLE, DONE):
                self._say("cancelled")
                self._go(DONE)
        elif text == "phrase":
            self._keyword("OPERATOR", operator=True)
        elif text.startswith("bearing:"):
            try:
                self.operator_bearing = math.radians(float(text.split(":", 1)[1]))
            except ValueError:
                pass
        elif text.startswith("goto:"):
            self._jump(text.split(":", 1)[1].strip().upper(), inp)

    def _jump(self, state: str, inp: Inputs) -> None:
        """Start the run from a later stage: Pupper has been placed there, facing along the line."""
        if state not in RUN_ORDER or self.state not in (IDLE, DONE):
            return                                  # only from idle, as Mission Control's stage buttons are
        c = self.cfg.course
        at = {SEEK: 0.0, ORIENT: 0.0, CONFIRM: 0.0, ZONE1: 0.0, REFIX: c.refix_at, ZONE2: c.refix_at,
              ARRIVE: c.length, RESCUE: c.length}[state]
        self._reset_run()
        self.reck.reset()
        self.reck.yaw = inp.yaw
        self.t_run = self.t
        if state not in (SEEK,):
            self.reck.set_line(inp.yaw, at)
            self.target = inp.yaw
        if state in (REFIX, ZONE2, ARRIVE, RESCUE):
            self.passed = len(self.cfg.course.obstacles)
        self._note("jumped", state=state, s=at)
        self._go(state)
        if state == REFIX:
            self._say("refix")
        elif state == ARRIVE:
            self._say("arrived")
            self.arrived_said = self.t
        elif state == RESCUE:
            self._say("saved", distance_m=int(round(self.reck.s)))

    def _pause(self, why: str) -> None:
        if self.state in (PAUSED, HOLD):
            return
        self.resume_state, self.reason = self.state, why
        self._note("paused", why=why.replace(" ", "_")[:24])
        self._go(PAUSED)

    def _hold(self, why: str) -> None:
        if self.state == HOLD:
            return
        self.resume_state = self.state if self.state != PAUSED else self.resume_state
        self.reason = why
        self._go(HOLD)
        self._say("hold")

    def _keyword(self, phrase: str, operator: bool = False) -> None:
        if self.state == ARRIVE or (self.state == ZONE2 and self.reck.s >= self.cfg.arrive.keyword_from):
            self.resume_state = self.state
            self._go(RESCUE)
            self._say("saved", distance_m=int(round(self.reck.s)))
        elif self.state in ACTIVE:
            self.keyword_ignored += 1
            self._note("keyword_ignored", state=self.state, s=round(self.reck.s, 1))

    # ------------------------------------------------------------------ the tick
    def tick(self, inp: Inputs) -> Outputs:
        self.t = inp.t
        self._events = []
        cfg = self.cfg

        # is Pupper standing still? (bearings count only then)
        moving = inp.cmd_vel is not None and (abs(inp.cmd_vel[0]) > 0.02 or abs(inp.cmd_vel[1]) > 0.02 or abs(inp.cmd_vel[2]) > 0.05)
        if moving or self.crossing_active:
            self.still_since = None
        elif self.still_since is None:
            self.still_since = inp.t

        # the step-over node's reports
        for st in inp.crossing:
            self._crossing(st)
        legs_listening = not self.crossing_active and inp.t >= self.restart_until
        self.reck.update(inp.t, inp.cmd_vel, inp.yaw, legs_listening)

        for text in inp.commands:
            self._command(text, inp)
        if inp.keyword:
            self._keyword(inp.keyword)

        # what stops an active run
        if inp.mux_source in JOYSTICK:
            self.joystick_last = inp.t
        if self.state in ACTIVE:
            if inp.estop:
                self._pause("e-stop pressed")
            elif inp.mux_source in JOYSTICK:
                self._pause("joystick")
            elif not inp.imu_ok:
                self._hold("the IMU has gone quiet")
            elif self.crossing_active and self.crossing_deadline is not None and inp.t > self.crossing_deadline:
                # the step-over node's "done" never came: it has stopped, or the message was lost
                self.crossing_active = self.crossing_planning = False
                self.crossing_deadline = None
                self._hold("the crossing has not reported its end")
            elif inp.tilt_deg > cfg.safety.tilt_deg and not self.crossing_active:
                self._pause("not standing level (fallen or lifted)")
            elif (self.state in (ORIENT, ZONE1, ZONE2) and self.motion and legs_listening
                  and inp.t - self.restart_until > 2.0 and inp.alive.get("walking") is False):
                self._pause("the walking controller is not active")
        elif self.state == PAUSED and self.reason == "joystick":
            if inp.t - self.joystick_last > cfg.safety.joystick_release_s and self.resume_state:
                back, self.resume_state = self.resume_state, None
                self._go(back)
                self._note("resumed", state=back)

        handler = {PREFLIGHT: self._preflight_state, SEEK: self._seek, ORIENT: self._orient, CONFIRM: self._confirm,
                   ZONE1: self._zone1, REFIX: self._refix, ZONE2: self._zone2, ARRIVE: self._arrive}.get(self.state)
        cmd = handler(inp) if handler else None
        if cmd is None and inp.t < self.brake_until and self.state != IDLE:
            cmd = ZERO                          # brake, then go silent
        if self.state == DONE and inp.t - self.t_state > 2.0:
            self._go(IDLE)

        self.terrain_prev = str(inp.terrain.get("state", "") or "")
        self.would = cmd
        out = Outputs(cmd=cmd if self.motion else None, events=self._events, state=self.state)
        out.arm_dodge, out.terrain_mode = self._modes()
        return out

    def _modes(self) -> tuple[bool, str]:
        """What the two reflexes should be doing in this state."""
        if not self.motion:
            return False, "report" if self.state in ACTIVE else "off"
        if self.state == ZONE1:
            # full while a course obstacle is still ahead; after the last one anything in the way is walked around
            if self.avoid_next:
                return False, "avoid"
            return False, "full" if self.passed < len(self.cfg.course.obstacles) else "avoid"
        if self.state == REFIX:
            return False, "report"
        if self.state == ZONE2:
            return bool(self.cfg.gauntlet.arm_dodge), "avoid" if self.blocked_avoiding else "report"
        if self.state == ARRIVE:
            return bool(self.cfg.gauntlet.arm_dodge), "report"
        return False, "off"

    # ------------------------------------------------------------------ listening
    def _listen(self, inp: Inputs) -> bool:
        """Take in the bearings that can be trusted. True while Pupper is able to listen."""
        cfg = self.cfg.sound
        still = self.still_since is not None and inp.t - self.still_since >= cfg.still_s
        ok = still and not self.quiet
        if ok and self.listen_from is None:
            self.listen_from = inp.t
        for b in inp.bearings:
            bearing, conf = float(b[0]), (float(b[1]) if len(b) > 1 else 1.0)
            used = ok and conf >= cfg.min_confidence
            heading = wrap(inp.yaw + bearing)
            self.heard.append([inp.t, heading, used])
            if used:
                self.collector.add(inp.t, heading)
        self.heard = [h for h in self.heard if inp.t - h[0] < 30.0][-24:]
        return ok

    def _operator_fix(self, inp: Inputs) -> Fix | None:
        if self.operator_bearing is None:
            return None
        heading = wrap(inp.yaw + self.operator_bearing)
        self.operator_bearing = None
        self.heard.append([inp.t, heading, True])
        return Fix(heading, 99, 0.0, 0)

    # ------------------------------------------------------------------ states
    def _preflight_state(self, inp: Inputs):
        if not self.quiet:
            self._go(SEEK)
        return None

    def _seek(self, inp: Inputs):
        cfg = self.cfg.sound
        if self.seek_turn is not None:              # turning a quarter to listen from another angle
            err = diff(self.seek_turn, inp.yaw)
            if abs(err) < math.radians(self.cfg.orient.done_deg) or inp.t - self.t_state > self.cfg.orient.timeout_s:
                self.seek_turn = None
                self._go(SEEK)
                return ZERO
            return turn_command(err, self.cfg.orient)
        listening = self._listen(inp)
        fix = self._operator_fix(inp) or self.collector.fix(inp.t)
        if fix is not None:
            self.first_fix, self.target = fix, fix.heading
            self._note("fix", bearing=round(math.degrees(diff(fix.heading, inp.yaw))), count=min(fix.count, 9),
                       spread=round(math.degrees(fix.spread), 1))
            self._go(ORIENT)
            self._say("heard")
            return None
        if listening and self.listen_from is not None:
            waited = inp.t - self.listen_from
            if waited > cfg.turn_after_s:
                self.seek_turn = wrap(inp.yaw + math.pi / 2)
                self._note("turning_to_listen")
                self.t_state = inp.t
                self.listen_from = None
            elif waited > cfg.no_sound_after_s and inp.t - self.no_sound_said > cfg.no_sound_after_s:
                self.no_sound_said = inp.t
                self._say("no_sound")
        return None

    def _orient(self, inp: Inputs):
        cfg = self.cfg.orient
        self._listen(inp)                           # only to log what arrives; nothing is used while turning
        err = diff(self.target, inp.yaw)
        if abs(err) < math.radians(cfg.done_deg):
            if self.settle_t is None:
                self.settle_t = inp.t
            if inp.t - self.settle_t >= cfg.settle_s:
                self._go(CONFIRM)
            return ZERO
        self.settle_t = None
        if inp.t - self.t_state > cfg.timeout_s:
            self._hold("the turn did not finish")
            return None
        return turn_command(err, cfg)

    def _walk_from_here(self, heading: float) -> None:
        self.reck.x = self.reck.y = 0.0             # the start mark: turning on the spot went nowhere
        self.reck.line = None
        self.reck.first_heading = None
        self.reck.set_line(heading, 0.0)
        self.target = heading
        self._note("line_set", heading=round(math.degrees(diff(heading, self.reck.yaw)), 1))
        self._go(ZONE1)

    def _confirm(self, inp: Inputs):
        cfg = self.cfg.orient
        self._listen(inp)
        fix = self._operator_fix(inp)
        if fix is None:
            best = self.collector.best(inp.t, prior=self.target)
            if best is not None and best.count >= cfg.confirm_min_bearings:
                fix = best
        if fix is not None:
            off = diff(fix.heading, self.target)
            if abs(off) <= math.radians(cfg.confirm_ok_deg) or fix.count >= 99:
                # two looks at the same voice: the second was taken nose-on, the first from wherever
                # Pupper happened to face. Blend them (confirm_blend = weight of the first).
                w = 0.0 if fix.count >= 99 else cfg.confirm_blend
                heading = circular_mean([self.target] * int(round(w * 10)) + [fix.heading] * int(round((1 - w) * 10)))
                self._walk_from_here(heading)
            elif abs(off) > math.radians(cfg.confirm_redo_deg) or self.reorients >= cfg.max_reorients:
                if self.reorients >= cfg.max_reorients:
                    self._walk_from_here(fix.heading)
                else:
                    self._note("fix_rejected", off=round(math.degrees(off)))
                    self.reorients += 1
                    self._go(SEEK)
            else:
                self.reorients += 1
                self.target = fix.heading
                self._go(ORIENT)
            return None
        if self.listen_from is not None and inp.t - self.listen_from > cfg.confirm_timeout_s:
            self._walk_from_here(self.target)       # nothing more heard: go with the first fix
        return None

    def _crossing(self, st: dict) -> None:
        state = st.get("state")
        if state == "planning" and not st.get("dry_run"):
            self.landmark_due = True                # Pupper is standing a measured distance from the obstacle
            self.crossing_planning = True
        elif state in ("no_plan", "refused"):
            self.crossing_planning = False          # terrain agility walks around it instead
        if state == "started":
            self.crossing_planning = False
            self.crossing_active = True
            self.crossing_progress = 0.0
            advance = st.get("advance")             # metres the plan moves the body; there are no velocity commands
            self.crossing_advance = float(advance) if isinstance(advance, (int, float)) else self.cfg.reckon.crossing_advance
            self.crossing_counted = 0.0
            duration = st.get("duration")
            duration = float(duration) if isinstance(duration, (int, float)) else 90.0
            self.crossing_deadline = self.t + max(1.5 * duration, duration + 30.0)
        elif state == "progress" and self.crossing_active:
            if self.crossing_deadline is not None:      # still reporting: a slow crossing, not a lost one
                self.crossing_deadline = max(self.crossing_deadline, self.t + 30.0)
            self.crossing_progress = max(self.crossing_progress, min(1.0, float(st.get("progress", 0.0) or 0.0)))
            step = self.crossing_advance * self.crossing_progress - self.crossing_counted
            self.reck.advance(step)                 # count it as it happens, so the distance never jumps
            self.crossing_counted += step
        elif state == "done" and not st.get("dry_run"):
            if self.crossing_planning and not self.crossing_active:
                self.crossing_counted = 0.0         # its "started" was missed: count this crossing all the same
                self.crossing_advance = self.cfg.reckon.crossing_advance
                self.crossing_active = True
            self.crossing_planning = False
            if self.crossing_active:
                advance = st.get("advance")
                total = float(advance) if isinstance(advance, (int, float)) else self.crossing_advance
                self.reck.advance(total - self.crossing_counted)
                self.restart_until = self.t + self.cfg.reckon.restart_s
                self._note("crossed", s=round(self.reck.s, 2))
                self.passed += 1
                self.said_stepping = False          # the next crossing is announced too
            self.crossing_active = False
            self.crossing_progress = 1.0
        elif state == "failed":
            was = self.crossing_active
            self.crossing_active = self.crossing_planning = False
            if was and self.state in ACTIVE:
                self._hold("the crossing stopped: " + str(st.get("why", ""))[:60])

    def _terrain_watch(self, inp: Inputs) -> str:
        """Follow terrain agility's state; returns it."""
        ts = str(inp.terrain.get("state", "") or "")
        if ts in TERRAIN_IDLE:
            if self.terrain_idle_since is None:
                self.terrain_idle_since = inp.t
        else:
            self.terrain_idle_since = None
        if ts == "HOLD":
            if self.terrain_hold_since is None:
                self.terrain_hold_since = inp.t
                self.terrain_holds = [h for h in self.terrain_holds if inp.t - h < 30.0] + [inp.t]
            stuck = inp.t - self.terrain_hold_since > 8.0 or len(self.terrain_holds) >= 2
            if stuck and self.back_offs < self.cfg.walk.back_off_tries and "crossing" not in str(inp.terrain.get("reason", "")):
                room = self._room_behind()
                # It stands for 2 s and tries again by itself. Twice without getting on usually means it
                # has ended up too close to the obstacle's corner: step back and let it look again.
                self.back_offs += 1
                self.avoid_next = True              # and do not walk up to measure it again: go around from here
                self.terrain_holds = []
                self.terrain_hold_since = None
                # Reversing is blind: never back into an obstacle of the course already behind it
                self.back_off_len = max(0.0, min(self.cfg.walk.back_off_m, room))
                if self.back_off_len < 0.10:
                    self.back_off_len = 0.0             # no room: only ask terrain agility to go around from here
                self.back_off_until = inp.t + 2.0 + self.back_off_len / self.cfg.walk.back_off_speed
                self._note("backing_off", s=round(self.reck.s, 2), tries=self.back_offs, m=round(self.back_off_len, 2))
            elif stuck:
                self.terrain_holds = []
                self._hold("terrain agility cannot find a way: " + str(inp.terrain.get("reason", ""))[:60])
        else:
            self.terrain_hold_since = None
        return ts

    def _room_behind(self) -> float:
        """Metres Pupper may reverse before its tail (0.2 m behind the centre, plus 0.15 m margin, for the
        estimate's error) reaches the far edge of an obstacle behind it that it walked around, or that is
        too tall to cross. A low one it crossed is not counted: reversing over it is what it did before."""
        s, around = self.reck.s, getattr(self, "went_around", [])
        behind = [float(o["near"]) + float(o.get("depth", 0.1)) for k, o in enumerate(self.cfg.course.obstacles)
                  if k in around or float(o.get("height", 0.0)) >= 0.08]
        behind = [far for far in behind if far < s]
        return 9.9 if not behind else s - max(behind) - 0.35

    def _back_off(self, inp: Inputs):
        """While backing off: the command (None = stand and let terrain agility let go). Else False."""
        if inp.t >= self.back_off_until:
            return False
        left = self.back_off_until - inp.t
        walk = self.cfg.walk
        if left > getattr(self, "back_off_len", walk.back_off_m) / walk.back_off_speed:
            return None                             # first 2 s: say nothing, so terrain agility stands down
        return (-walk.back_off_speed, 0.0, 0.0)

    def _stalled(self, inp: Inputs, limit_s: float = 25.0) -> bool:
        """Walking states only: True (and HOLD) if Pupper has not got 15 cm further in `limit_s`."""
        waiting = self.crossing_active or inp.t < self.restart_until + 1.0 or self.crossing_planning
        if self.progress_mark is None or waiting or not self.motion or self.reck.s - self.progress_mark[1] > 0.15:
            self.progress_mark = (inp.t, self.reck.s)
            return False
        if inp.t - self.progress_mark[0] > limit_s:
            self.progress_mark = None
            self._hold("no progress along the line for %d s" % limit_s)
            return True
        return False

    def _zone1(self, inp: Inputs):
        cfg, c = self.cfg, self.cfg.course
        ts = self._terrain_watch(inp)
        prev = self.terrain_prev
        if self.state != ZONE1:
            return None
        if self.motion and not inp.terrain.get("alive", False) and inp.t - self.t_state > 2.0:
            self._hold("terrain agility has gone quiet")
            return None
        if self._stalled(inp):
            return None

        # An obstacle about to be crossed is a landmark: terrain agility is standing a measured distance
        # from it, and its place on the course is known. (Taken from the step-over node's report, not
        # from terrain's state names, which may grow.)
        obstacles = c.obstacles
        if self.landmark_due:
            self.landmark_due = False
            box = inp.terrain.get("box") or {}
            near = box.get("near_x")
            near = float(near) if isinstance(near, (int, float)) else cfg.reckon.gauge_distance
            if self.passed < len(obstacles):
                before, there = self.reck.s, float(obstacles[self.passed]["near"]) - near
                if abs(there - before) < 1.0:       # further off than that, it is not the obstacle expected next
                    self.reck.set_s(there)
                self._note("box_landmark", s=round(self.reck.s, 2), was=round(before, 2),
                           obstacle=str(obstacles[self.passed].get("name", self.passed + 1))[:24])
        if self.crossing_active and not self.said_stepping:
            self.said_stepping = True
            self._say("stepping_over")
        if ts == "AVOID" and prev != "AVOID":
            self._say("going_around")
        if prev == "AVOID" and ts in TERRAIN_IDLE:
            self.passed += 1                        # walked around: a course obstacle, or something in its place
            self.avoid_next = False
            self.back_offs = 0
            self._note("walked_around", s=round(self.reck.s, 2), d=round(self.reck.d, 2))

        if self.crossing_active or inp.t < self.restart_until:
            return None                             # the legs are not in the walking controller: ask for nothing
        backing = self._back_off(inp)
        if backing is not False:
            return backing
        idle = ts in TERRAIN_IDLE
        idle_for = inp.t - self.terrain_idle_since if self.terrain_idle_since is not None else 0.0
        s = self.reck.s
        passed = (self.passed >= len(obstacles) or not cfg.refix.need_walk_around
                  or s >= c.refix_at + cfg.refix.fallback_extra)
        if s >= c.refix_at and idle and idle_for >= 0.5 and passed:
            if cfg.refix.enabled:
                self._go(REFIX)
                self._say("refix")
            else:
                self._go(ZONE2)
            return None
        if not idle:
            # terrain agility has the legs. Keep asking to walk straight on: it treats the
            # driver's command as "where the robot was going" and lets go if the driver stops.
            return (cfg.walk.cruise, 0.0, 0.0)
        # On the line, so Pupper meets each obstacle centred, except while it is beside one it has gone
        # around: the line runs through that obstacle, and steering back onto it there turns Pupper into
        # the obstacle's corner. Then it only holds the line's direction until the obstacle is behind it.
        d = self.reck.d
        if abs(d) > 0.12:
            for ob in obstacles:
                if float(ob["near"]) - 0.6 <= s <= float(ob["near"]) + float(ob.get("depth", 0.1)) + 0.30:
                    d = 0.0
        return line_command(d, self.reck.heading_error, cfg.walk.cruise, cfg.walk)

    def _refix(self, inp: Inputs):
        cfg, c = self.cfg.refix, self.cfg.course
        self._listen(inp)
        if self.listen_from is None:
            return None
        fix = self._operator_fix(inp)
        decided = fix is not None
        expected = wrap(inp.yaw + self.reck.bearing_to(c.cover_near + 0.3))     # where the survivor should be
        if fix is None:
            best = self.collector.best(inp.t, prior=expected)
            first = self.collector.first_time()
            if best is not None and best.count >= cfg.min_bearings:
                if best.count >= cfg.min_bearings + 1 or inp.t - first >= cfg.gather_s:
                    fix, decided = best, True
        if decided:
            off = diff(fix.heading, expected)
            if abs(off) <= math.radians(cfg.accept_deg) or fix.count >= 99:
                old = self.reck.line.heading
                self.reck.set_line(fix.heading)
                self._note("line_redrawn", change=round(math.degrees(diff(fix.heading, old)), 1),
                           off_line=round(self.reck.d, 2))
            else:
                self._note("refix_rejected", off=round(math.degrees(off)))
            self._go(ZONE2)
        elif inp.t - self.listen_from > cfg.timeout_s:
            self._note("refix_timeout")
            self._go(ZONE2)
        return None

    def _tall_ahead(self, inp: Inputs, cone_deg: float):
        """The nearest tall obstacle within the cone: (range from the lens, centre bearing rad, width) or None."""
        best = None
        for ob in inp.obstacles:
            if not ob.get("tall") or ob.get("is_box"):
                continue
            left, right = float(ob.get("left_deg", 0.0)), float(ob.get("right_deg", 0.0))
            if right > cone_deg or left < -cone_deg:
                continue
            rng = float(ob.get("nearest", 9.9))
            if best is None or rng < best[0]:
                best = (rng, math.radians(0.5 * (left + right)), float(ob.get("width", 0.0)),
                        float(ob.get("bearing_deg", 0.0)))
        return best

    def _zone2(self, inp: Inputs):
        cfg = self.cfg
        ts = self._terrain_watch(inp)
        if self.state != ZONE2:
            return None
        s = self.reck.s

        for e in inp.dodge_events:
            if e.get("event") == "dodge" and not e.get("dry_run"):
                # Pupper's voice hears of it from /dodge/event itself; here it is only counted
                self.dodges += 1
                self._note("dodged", n=self.dodges, s=round(s, 1))

        if s >= cfg.arrive.hard_stop:
            return self._arrived("as far as I go", inp)
        scan_ok = bool(inp.terrain.get("scan_fresh", False))
        if not scan_ok and s >= cfg.arrive.blind_stop:
            return self._arrived("by distance, the camera is quiet", inp)

        bias = 0.0
        if s >= cfg.arrive.cover_from:
            hit = self._tall_ahead(inp, cfg.arrive.cover_cone_deg)
            if hit is not None:
                rng, centre, width, _ = hit
                if not self.cover_seen:
                    self.cover_seen = True
                    self._note("cover_seen", range=round(rng, 2), s=round(s, 2))
                self.cover_range = rng
                if rng <= cfg.arrive.stop_range:
                    return self._arrived("the cover is in front of me", inp)
                if cfg.arrive.home_on_cover and rng > 0.8 and 0.5 <= width <= 2.5:
                    limit = math.radians(cfg.arrive.home_max_deg)
                    bias = max(-limit, min(limit, centre))
        else:
            # something tall in the lane where nothing should be: stop, look again, then go round it
            hit = self._tall_ahead(inp, 25.0)
            in_lane = False
            if hit is not None and hit[0] <= cfg.gauntlet.obstacle_stop_range:
                in_lane = abs(hit[0] * math.sin(math.radians(hit[3]))) <= cfg.gauntlet.lane_half + 0.15
            backing = self._back_off(inp)
            if backing is not False:
                return backing
            if self.blocked_avoiding:
                if ts in TERRAIN_IDLE and not in_lane and self.terrain_prev in TERRAIN_IDLE and \
                        inp.t - self.blocked_since > cfg.gauntlet.obstacle_wait_s + 1.5:
                    self.blocked_avoiding, self.blocked_since = False, None
                return (cfg.walk.gauntlet, 0.0, 0.0) if ts not in TERRAIN_IDLE else \
                    line_command(self.reck.d, self.reck.heading_error, cfg.walk.gauntlet, cfg.walk)
            if in_lane:
                if self.blocked_since is None:
                    self.blocked_since = inp.t
                    self._note("blocked", range=round(hit[0], 2), s=round(s, 2))
                if inp.t - self.blocked_since > cfg.gauntlet.obstacle_wait_s:
                    self.blocked_avoiding = True
                    self._say("going_around")
                return ZERO
            self.blocked_since = None

        if self.mux_dodging(inp):
            # the dodge has the legs; keep the wish alive so the mux hands straight back
            return line_command(self.reck.d, self.reck.heading_error, cfg.walk.gauntlet, cfg.walk)
        return line_command(self.reck.d, self.reck.heading_error, cfg.walk.gauntlet, cfg.walk, bias=bias)

    @staticmethod
    def mux_dodging(inp: Inputs) -> bool:
        return inp.mux_source == "/dodge_cmd_vel"

    def _arrived(self, why: str, inp: Inputs):
        self.arrive_why = why
        self._go(ARRIVE)
        self._say("arrived", distance_m=int(round(self.reck.s)))
        self.arrived_said = inp.t
        return ZERO

    def _arrive(self, inp: Inputs):
        cfg = self.cfg.arrive
        waited = inp.t - self.t_state
        if not self.gave_up and waited > cfg.give_up_s:
            self.gave_up = True                     # nobody answered: tell the team, keep waiting
            self._note("no_answer", waited=int(waited))
            self._say("hold")
        elif not self.gave_up and not self.quiet and inp.t - self.arrived_said > cfg.repeat_s:
            self.arrived_said = inp.t
            self._say("arrived")
        return None

    # ------------------------------------------------------------------ reporting
    def status(self) -> dict:
        r, c = self.reck, self.cfg.course
        x, y, yaw = r.course_pose()
        fix = self.collector.best(self.t) if self.state in LISTENING else None
        zone = {ZONE1: "approach", REFIX: "second listen", ZONE2: "gauntlet", ARRIVE: "at the cover",
                RESCUE: "with the survivor"}.get(self.state, "")
        cue = EV.cue(self.state, quiet=self.quiet, s=r.s, course=c,
                     crossing=self.crossing_active or self.crossing_planning,
                     reason=self.reason, motion=self.motion)
        would = getattr(self, "would", None)
        return {
            "state": self.state, "in_state_s": round(self.t - self.t_state, 1),
            "run_s": round(self.t - self.t_run, 1) if self.t_run is not None and self.state != IDLE else 0.0,
            "motion": self.motion, "zone": zone,
            "s": round(r.s, 3), "d": round(r.d, 3), "heading_error_deg": round(math.degrees(r.heading_error), 1),
            "pose": [round(x, 3), round(y, 3), round(yaw, 4)], "line": r.course_line(),
            "has_line": r.line is not None, "walked_m": round(r.distance, 2),
            "listening": self.state in LISTENING and not self.quiet and self.listen_from is not None,
            "quiet": self.quiet, "quiet_for_s": round(max(0.0, self.quiet_until - self.t), 1),
            "bearings": [{"age": round(self.t - t, 1), "deg": round(math.degrees(diff(h, r.yaw)), 1), "used": u}
                         for t, h, u in self.heard],
            "fix": None if fix is None else {"deg": round(math.degrees(diff(fix.heading, r.yaw)), 1), "count": fix.count,
                                             "spread_deg": round(math.degrees(fix.spread), 1), "rejected": fix.rejected},
            "target_deg": None if self.target is None else round(math.degrees(diff(self.target, r.yaw)), 1),
            "would_cmd": None if would is None else [round(v, 3) for v in would],
            "reason": self.reason if self.state in (PAUSED, HOLD) else "",
            "resume_state": self.resume_state, "arrive_why": self.arrive_why,
            "checks": self.checks, "missing": self.missing,
            "crossing": {"active": self.crossing_active, "progress": round(self.crossing_progress, 3)},
            "passed": self.passed, "obstacles": len(c.obstacles), "cover_seen": self.cover_seen,
            "cover_range": self.cover_range, "dodges": self.dodges, "keyword_ignored": self.keyword_ignored,
            "cue": cue,
        }
