#!/usr/bin/env python3
"""Soak the mission state machine: thousands of randomised simulated runs, with faults.

    python3 tools/soak_sim.py --runs 2000                  randomised courses and faults, 8 processes
    python3 tools/soak_sim.py --runs 500 --clean           randomised courses, no faults at all
    python3 tools/soak_sim.py --repro 1234                 one seed again, every fault it drew, with a trace
    python3 tools/soak_sim.py --repro 1234 --keep a,b      the same seed with only the named faults

Each seed draws a course (which of the bar, slab, carton and round bar, in what order, where) and a
set of faults, then runs sim/course_sim.py's CourseSim with the faults injected between the simulated
components and the Board, exactly where the ROS callbacks would deliver them on the robot:

    sound     the first cry late, never, noisy (big margin of error, many echoes), sign flipped,
              a fixed zero offset, bearings delivered late
    terrain   /terrain/state silent for a while (dropout), or frozen at one value (stall);
              /terrain/obstacles silent; crossing_status 'started' / 'progress' / 'done' messages
              lost, or sent without their 'advance'
    operator  pause, resume, abort, end, start, phrase, goto:<stage>, motion on/off, a joystick nudge,
              at random moments
    body      the walk's real speed (speed_factor), IMU drift, walk lag; the IMU or the walking
              controller going quiet for a while

and watches for:

    exception          anything raised by the mission, the board, the simulator or a snapshot
    illegal            a state change the mission's own diagram does not allow (every _go is recorded)
    resume_wrong       a resume that went somewhere other than the state that was paused
    deadlock           a walking or turning state with no progress (true pose, estimate, state, crossing
                       report) for --deadlock seconds of sim time, and the mission has not said HOLD
    false_finish       "the cover is in front of me" (and then RESCUE) without being in front of the cover
    cover_edge_finish  RESCUE in front of the cover's end (within 10 cm of it), not 5 cm inside it
    off_target_finish  RESCUE by the survivor's phrase after an arrival by distance (hard stop, camera quiet)
                       that is not in front of the cover: the designed fallback, reported apart
    moves_when_stopped a velocity other than zero wanted in IDLE / DONE / PAUSED / HOLD / RESCUE
    modes              dodging armed outside ZONE2/ARRIVE, or terrain 'full' outside ZONE1
    walked_through     the body centre inside an obstacle it was not crossing (the sim does not block)
    goto_mid_run       a goto:<stage> (Mission Control's stage buttons) accepted while PAUSED, HOLD or RESCUE
    bad_json           a Mission Control snapshot that is not strict JSON (NaN or infinity)

Each failure is minimised: faults are removed one at a time while the same failure class still
happens, so the report gives a seed and the smallest set of faults that still shows it. All of it is
simulation: it checks the logic, not the robot, the sound module or terrain agility's perception.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import random
import sys
import traceback
from collections import Counter, defaultdict
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))

from nav import mission as M  # noqa: E402
from nav.config import DemoConfig, apply  # noqa: E402
from nav.heading import wrap  # noqa: E402
from sim.course_sim import CourseSim, SimOptions  # noqa: E402

# ------------------------------------------------------------------ the course pieces
BAR = {"name": "low bar", "shape": "flat", "depth": 0.089, "width": 0.60, "height": 0.019}
SLAB = {"name": "slab", "shape": "flat", "depth": 0.38, "width": 0.565, "height": 0.015}
CARTON = {"name": "carton", "shape": "flat", "depth": 0.348, "width": 0.455, "height": 0.13}
ROUND = {"name": "round bar", "shape": "round", "depth": 0.05, "width": 0.70, "height": 0.05}

# ------------------------------------------------------------------ the mission's own diagram
S = M
ALLOWED = {
    S.IDLE: {S.PREFLIGHT} | set(M.RUN_ORDER),               # start, or goto:<stage>
    S.PREFLIGHT: {S.SEEK},
    S.SEEK: {S.SEEK, S.ORIENT},                              # SEEK -> SEEK after a quarter turn
    S.ORIENT: {S.CONFIRM, S.HOLD},
    S.CONFIRM: {S.ZONE1, S.SEEK, S.ORIENT},
    S.ZONE1: {S.REFIX, S.ZONE2, S.HOLD},
    S.REFIX: {S.ZONE2},
    S.ZONE2: {S.ARRIVE, S.RESCUE, S.HOLD},
    S.ARRIVE: {S.RESCUE},
    S.RESCUE: {S.DONE, S.ARRIVE, S.ZONE2},                   # end; or resume after a phrase heard by mistake
    S.PAUSED: {S.HOLD},                                      # plus resume_state, checked separately
    S.HOLD: set(),                                           # resume_state only
    S.DONE: {S.IDLE, S.PREFLIGHT} | set(M.RUN_ORDER),
}
ANY_ACTIVE_TO = {S.PAUSED, S.HOLD, S.DONE}
WATCHED = {S.PREFLIGHT, S.SEEK, S.ORIENT, S.CONFIRM, S.ZONE1, S.REFIX, S.ZONE2}  # must keep making progress
STOPPED = {S.IDLE, S.DONE, S.PAUSED, S.HOLD, S.RESCUE}
OPERATOR_CMDS = ["pause", "resume", "resume", "abort", "end", "start!", "phrase", "motion_off", "motion_on",
                 "nudge:0,0.3", "goto:ZONE1", "goto:REFIX", "goto:ZONE2", "goto:ARRIVE", "bearing:30"]


# ------------------------------------------------------------------ one scenario, from a seed
def scenario(seed: int, clean: bool = False) -> dict:
    """Everything a run depends on, drawn from the seed. faults: name -> parameters."""
    r = random.Random(seed * 7919 + 17)
    pieces = [BAR, SLAB] + [p for p in (CARTON, ROUND) if r.random() < 0.35]
    if r.random() < 0.5:
        r.shuffle(pieces)
    elif r.random() < 0.3:
        pieces = [BAR, SLAB]                                # the course as laid out
    obstacles, at = [], r.uniform(0.40, 0.60)
    for p in pieces:
        obstacles.append(dict(p, near=round(at, 3)))
        at += p["depth"] + r.uniform(0.35, 0.75)
    last_far = obstacles[-1]["near"] + obstacles[-1]["depth"]
    refix_at = round(max(2.0, last_far + 0.6), 2)
    course = {"obstacles": obstacles, "refix_at": refix_at, "throw_from": round(max(3.0, refix_at + 1.0), 2)}
    opt = {"seed": seed, "moe_deg": r.choice([3.0, 5.0, 5.0, 8.0]), "second_call": r.random() < 0.85,
           "says_phrase": r.random() < 0.9, "throws": r.random() < 0.8}
    faults: dict = {}
    if clean:
        return {"seed": seed, "course": course, "opt": opt, "faults": faults}

    def maybe(p):
        return r.random() < p

    if maybe(0.15):
        faults["sound_late"] = {"until": round(r.uniform(20.0, 120.0), 1)}
    if maybe(0.05):
        faults["sound_never"] = {}
    if maybe(0.15):
        faults["sound_noisy"] = {"moe_deg": round(r.uniform(10.0, 30.0), 1), "echo_p": round(r.uniform(0.15, 0.4), 2)}
    if maybe(0.10):
        faults["sound_flipped"] = {}
    if maybe(0.08):
        faults["sound_offset"] = {"deg": round(r.choice((-1, 1)) * r.uniform(10.0, 40.0), 1)}
    if maybe(0.10):
        faults["bearing_delay"] = {"s": round(r.uniform(1.0, 4.0), 2)}
    if maybe(0.15):
        t0 = r.uniform(15.0, 200.0)
        faults["terrain_dropout"] = {"t0": round(t0, 1), "t1": round(t0 + r.uniform(0.5, 30.0), 1)}
    if maybe(0.15):
        t0 = r.uniform(15.0, 200.0)
        faults["terrain_stall"] = {"t0": round(t0, 1), "t1": round(t0 + r.uniform(2.0, 60.0), 1)}
    if maybe(0.10):
        t0 = r.uniform(15.0, 250.0)
        faults["scan_dropout"] = {"t0": round(t0, 1), "t1": round(t0 + r.uniform(1.0, 40.0), 1)}
    for kind in ("started", "progress", "done"):
        if maybe(0.06):
            faults["crossing_drop_" + kind] = {"p": r.choice([1.0, 1.0, 0.5])}
    if maybe(0.12):
        faults["crossing_no_advance"] = {}
    if maybe(0.25):
        faults["odometry"] = {"speed_factor": round(r.uniform(0.7, 1.25), 3), "yaw_drift_deg_min": round(r.uniform(0.0, 8.0), 2),
                              "walk_lag": round(r.uniform(0.1, 0.6), 2)}
    if maybe(0.05):
        t0 = r.uniform(15.0, 200.0)
        faults["imu_dropout"] = {"t0": round(t0, 1), "t1": round(t0 + r.uniform(0.5, 5.0), 1)}
    if maybe(0.06):
        t0 = r.uniform(15.0, 200.0)
        faults["walking_dropout"] = {"t0": round(t0, 1), "t1": round(t0 + r.uniform(0.5, 6.0), 1)}
    if maybe(0.35):
        for k in range(r.randint(1, 4)):
            faults["cmd%d" % k] = {"t": round(r.uniform(2.0, 300.0), 2), "text": r.choice(OPERATOR_CMDS)}
    return {"seed": seed, "course": course, "opt": opt, "faults": faults}


# ------------------------------------------------------------------ the simulator with faults
class SoakSim(CourseSim):
    def __init__(self, sc: dict, deadlock_s: float = 90.0):
        cfg = apply(DemoConfig(), {"course": sc["course"]})
        f = sc["faults"]
        opt = SimOptions(**sc["opt"])
        if "sound_never" in f:
            opt.survivor_calls = False
        if "sound_noisy" in f:
            opt.moe_deg, opt.echo_p = f["sound_noisy"]["moe_deg"], f["sound_noisy"]["echo_p"]
        if "odometry" in f:
            for k, v in f["odometry"].items():
                setattr(opt, k, v)
        super().__init__(cfg, opt, motion=True)
        self.f, self.deadlock_s = f, deadlock_s
        self.frng = random.Random(sc["seed"] * 31 + 5)
        self.cmds = sorted((v["t"], k, v["text"]) for k, v in f.items() if k.startswith("cmd"))
        self.problems: list = []
        self.transitions: list = []
        self.events: list = []
        self.frozen = None
        self.operator_phrase = False
        self.jumped = False
        self.mark = None
        self._wrap()

    # ---- where the faults go in
    def _window(self, name: str) -> bool:
        w = self.f.get(name)
        return bool(w) and w["t0"] <= self.t < w["t1"]

    def _wrap(self) -> None:
        b, m = self.board, self.mission
        raw = {n: getattr(b, n) for n in ("terrain_state", "terrain_obstacles", "crossing", "imu", "walking_active",
                                          "record")}

        def terrain_state(t, status):
            if self._window("terrain_dropout"):
                return
            if self._window("terrain_stall"):
                if self.frozen is None:
                    self.frozen = dict(status)
                return raw["terrain_state"](t, self.frozen)
            self.frozen = None
            raw["terrain_state"](t, status)

        def terrain_obstacles(t, obstacles):
            if not self._window("scan_dropout"):
                raw["terrain_obstacles"](t, obstacles)

        def crossing(t, status):
            kind = status.get("state")
            drop = self.f.get("crossing_drop_" + str(kind))
            if drop and self.frng.random() < drop["p"]:
                return
            if "crossing_no_advance" in self.f:
                status = {k: v for k, v in status.items() if k != "advance"}
            raw["crossing"](t, status)

        def imu(t, yaw, tilt=0.0):
            if not self._window("imu_dropout"):
                raw["imu"](t, yaw, tilt)

        def walking_active(t, active):
            raw["walking_active"](t, False if self._window("walking_dropout") else active)

        def record(t, out, mission):
            self.events += [(round(t, 2), n) for n, _ in out.events]
            self.last_out = out
            raw["record"](t, out, mission)

        b.terrain_state, b.terrain_obstacles, b.crossing = terrain_state, terrain_obstacles, crossing
        b.imu, b.walking_active, b.record = imu, walking_active, record

        go = m._go

        def recorded_go(state):
            self.transitions.append((round(m.t, 2), m.state, state, m.resume_state))
            self._check_edge(m.state, state)
            go(state)

        m._go = recorded_go

        jump = m._jump

        def recorded_jump(state, inp):
            before = m.state
            self._jumping = True
            try:
                jump(state, inp)
            finally:
                self._jumping = False
            if m.state != before and before not in (S.IDLE, S.DONE):
                # Mission Control's stage buttons: accepted from PAUSED, HOLD and RESCUE as well
                self.problem("goto_mid_run", f"goto:{state} accepted in {before} (reck.s was reset)")

        m._jump = recorded_jump

    def _cry(self) -> None:
        n = len(self.pending_bearings)
        super()._cry()
        f = self.f
        for i in range(n, len(self.pending_bearings)):
            t, b = self.pending_bearings[i]
            if "sound_flipped" in f:
                b = -b
            if "sound_offset" in f:
                b += math.radians(f["sound_offset"]["deg"])
            if "bearing_delay" in f:
                t += f["bearing_delay"]["s"]
            self.pending_bearings[i] = (t, wrap(b))

    # ---- the checks
    def problem(self, kind: str, detail: str) -> None:
        if not any(p[0] == kind for p in self.problems):
            self.problems.append((kind, round(self.t, 2), detail))

    _jumping = False

    def _check_edge(self, a: str, b: str) -> None:
        m = self.mission
        if self._jumping:
            self._paused_from = b
            return
        if a in (S.PAUSED, S.HOLD) and b not in (S.HOLD, S.DONE):
            # a resume: must go back to where it was
            if b != getattr(self, "_paused_from", b):
                self.problem("resume_wrong", f"{a} -> {b}, was paused from {self._paused_from}")
            return
        if b in (S.PAUSED, S.HOLD) and a not in (S.PAUSED, S.HOLD):
            self._paused_from = a
        if b == S.PAUSED and a == S.HOLD:
            self.problem("illegal", "HOLD -> PAUSED")
        ok = b in ALLOWED.get(a, set()) or (a in M.ACTIVE and b in ANY_ACTIVE_TO) or (a == S.RESCUE and b == S.DONE)
        if b == S.DONE and a not in (S.IDLE, S.DONE):
            ok = True                                   # end / abort from anywhere in a run
        if not ok:
            self.problem("illegal", f"{a} -> {b} at s={m.reck.s:.2f}")

    def command(self, text: str) -> None:
        if text == "phrase":
            self.operator_phrase = True
        if text.startswith("goto:"):
            self.jumped = True
        if text in ("start", "start!"):
            self.jumped = False
            self.operator_phrase = False
            # a second Start is pressed wherever Pupper now stands, not on the start mark: where it then
            # ends up says nothing about navigation
            self.restarted = getattr(self, "restarted", None) is not None
        super().command(text)

    def after_step(self) -> None:
        m, t = self.mission, self.t
        out = getattr(self, "last_out", None)
        would = m.would
        if would is not None and any(not math.isfinite(v) for v in would):
            self.problem("nan", f"command {would} in {m.state}")
        if m.state in STOPPED and would is not None and any(abs(v) > 1e-9 for v in would):
            self.problem("moves_when_stopped", f"{m.state} wants {tuple(round(v, 3) for v in would)}")
        if out is not None:
            if out.arm_dodge and m.state not in (S.ZONE2, S.ARRIVE):
                self.problem("modes", f"dodging armed in {m.state}")
            if out.terrain_mode == "full" and m.state != S.ZONE1:
                self.problem("modes", f"terrain full in {m.state}")
        # progress, for the deadlock watch
        x, y, th = (float(v) for v in self.pose)
        key = (m.state, m.crossing_active, m.crossing_planning, m.motion)
        if self.mark is None or self.mark[0] != key or math.hypot(x - self.mark[2], y - self.mark[3]) > 0.05 \
                or abs(wrap(th - self.mark[4])) > math.radians(10.0) or abs(m.reck.s - self.mark[5]) > 0.05:
            self.mark = (key, t, x, y, th, m.reck.s)
        elif m.state in WATCHED and m.motion and t - self.mark[1] > self.deadlock_s:
            self.problem("deadlock", f"{m.state} for {t - self.mark[1]:.0f} s, crossing_active={m.crossing_active} "
                                     f"planning={m.crossing_planning} terrain={self.board.terrain.get('state')} "
                                     f"s={m.reck.s:.2f} x={x:.2f}")
        # the body through an obstacle it is not crossing
        for k, rect in enumerate(self.rects[:-1]):
            if k not in self.crossed and rect.distance((x, y)) < 0.02 and self.cross is None:
                self.problem("walked_through", f"obstacle {k} ({self.cfg.course.obstacles[k]['name']}) at x={x:.2f}")
        if m.state == S.ARRIVE and m.arrive_why == "the cover is in front of me" and self._just(S.ARRIVE):
            if not self.in_front(0.10):
                self.problem("false_finish", f"'cover in front' at x={x:.2f} y={y:.2f}")

    def _just(self, state: str) -> bool:
        return self.transitions and self.transitions[-1][2] == state and self.transitions[-1][0] >= self.t - 0.06

    def in_front(self, margin: float = -0.05) -> bool:
        """course_sim's "reached" (margin -0.05: 5 cm inside the cover's ends). A false claim is judged
        with margin +0.10: the body centre more than 10 cm beyond the cover's end."""
        c = self.cfg.course
        x, y, th = (float(v) for v in self.pose)
        nose = x + 0.2 * math.cos(th)
        return abs(y) <= c.cover_width / 2 + margin and c.cover_near - 1.2 <= nose <= c.cover_near + 0.02

    # ---- a whole run
    def soak(self, max_time: float = 600.0) -> dict:
        self.step()
        self.command("motion_on")
        self.command("start!")
        next_snap = 0.0
        while self.t < max_time:
            while self.cmds and self.cmds[0][0] <= self.t:
                self.command(self.cmds.pop(0)[2])
            late = self.f.get("sound_late")
            if late and self.next_first_call is not None and self.t < late["until"]:
                self.next_first_call = max(self.next_first_call, late["until"])
            self.step()
            self.after_step()
            if self.t >= next_snap:
                next_snap = self.t + 5.0
                try:
                    json.dumps(self.snapshot(), allow_nan=False, default=str)
                except ValueError as exc:
                    self.problem("bad_json", str(exc)[:80])
            st = self.mission.state
            if not self.cmds and st in (S.RESCUE, S.HOLD, S.DONE, S.IDLE) and self.t - self.mission.t_state > 3.0:
                break
            if not self.cmds and st == S.ARRIVE and self.t - self.mission.t_state > 60.0:
                break
            if not self.cmds and st == S.PAUSED and self.t - self.mission.t_state > 10.0:
                break
        m = self.mission
        if m.state == S.RESCUE and not self.in_front() and not self.operator_phrase and not self.jumped \
                and not getattr(self, "restarted", False):
            # By distance (hard stop, camera quiet) the mission says "I'm here" without claiming the cover,
            # and the survivor's phrase then ends it: the designed fallback, counted apart from a false claim.
            kind = "off_target_finish"
            if m.arrive_why == "the cover is in front of me":
                kind = "false_finish" if not self.in_front(0.10) else "cover_edge_finish"
            self.problem(kind, f"RESCUE at x={self.pose[0]:.2f} y={self.pose[1]:.2f}, arrived '{m.arrive_why}'")
        return {"state": m.state, "reason": m.reason, "t": round(self.t, 1), "x": round(float(self.pose[0]), 2),
                "y": round(float(self.pose[1]), 2), "reached": m.state == S.RESCUE and self.in_front(),
                "passed": m.passed, "crossed": sorted(self.crossed)}


def run_one(sc: dict, deadlock_s: float = 90.0, trace: bool = False) -> dict:
    sim = None
    try:
        sim = SoakSim(sc, deadlock_s)
        res = sim.soak()
        problems = sim.problems
    except Exception as exc:                            # noqa: BLE001  (that is what is being looked for)
        tb = traceback.extract_tb(exc.__traceback__)
        where = "%s:%d" % (Path(tb[-1].filename).name, tb[-1].lineno) if tb else "?"
        res = {"state": getattr(getattr(sim, "mission", None), "state", "?"), "reason": "", "reached": False}
        problems = [("exception", round(getattr(sim, "t", 0.0), 2), f"{type(exc).__name__} at {where}: {exc}"[:200])]
        if trace:
            traceback.print_exc()
    out = {"seed": sc["seed"], "faults": sorted(sc["faults"]), "problems": problems, **res}
    if trace and sim is not None:
        out["transitions"] = sim.transitions
        out["events"] = sim.events
    return out


def minimise(sc: dict, kind: str, deadlock_s: float) -> list:
    """Drop faults one at a time while `kind` still happens. Returns the faults that are needed."""
    keep = dict(sc["faults"])
    changed = True
    while changed:
        changed = False
        for name in sorted(keep):
            trial = dict(sc, faults={k: v for k, v in keep.items() if k != name})
            if any(p[0] == kind for p in run_one(trial, deadlock_s)["problems"]):
                del keep[name]
                changed = True
    return sorted(keep)


def _job(args):
    seed, clean, deadlock_s = args
    sc = scenario(seed, clean)
    res = run_one(sc, deadlock_s)
    res["minimal"] = {}
    for kind, _, _ in res["problems"]:
        res["minimal"][kind] = minimise(sc, kind, deadlock_s) if sc["faults"] else []
    res["course"] = [o["name"] for o in sc["course"]["obstacles"]]
    return res


def outcome(r: dict) -> str:
    if r["reached"]:
        return "reached"
    if r["state"] == "HOLD":
        return "HOLD (%s) [%s]" % ((r.get("reason") or "").split(":")[0], "+".join(r.get("course", [])))
    reason = (r.get("reason") or "").split(":")[0]
    return f"{r['state']}" + (f" ({reason})" if reason else "")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--runs", type=int, default=200)
    ap.add_argument("--start", type=int, default=0, help="first seed")
    ap.add_argument("--clean", action="store_true", help="randomised courses, no faults")
    ap.add_argument("--jobs", type=int, default=os.cpu_count() or 2)
    ap.add_argument("--deadlock", type=float, default=90.0, help="seconds of sim time without progress")
    ap.add_argument("--out", default=None, help="write one JSON line per run here")
    ap.add_argument("--repro", type=int, default=None, help="run this one seed, with a trace")
    ap.add_argument("--keep", default=None, help="with --repro: only these faults (comma-separated)")
    args = ap.parse_args()

    if args.repro is not None:
        sc = scenario(args.repro, args.clean)
        if args.keep is not None:
            names = {n for n in args.keep.split(",") if n}
            sc["faults"] = {k: v for k, v in sc["faults"].items() if k in names}
        print(json.dumps({k: sc[k] for k in ("course", "opt", "faults")}, indent=1))
        res = run_one(sc, args.deadlock, trace=True)
        for t, a, b, rs in res.pop("transitions", []):
            print(f"  {t:7.2f}  {a:9s} -> {b:9s}  (resume_state {rs})")
        print("events:", " ".join(f"{t:.0f}:{n}" for t, n in res.pop("events", [])))
        print(json.dumps(res, indent=1, default=str))
        return

    seeds = [(args.start + k, args.clean, args.deadlock) for k in range(args.runs)]
    results, sink = [], open(args.out, "w") if args.out else None
    with ProcessPoolExecutor(max_workers=args.jobs) as pool:
        for k, res in enumerate(pool.map(_job, seeds, chunksize=4)):
            results.append(res)
            if sink:
                sink.write(json.dumps(res, default=str) + "\n")
                sink.flush()
            if (k + 1) % 100 == 0:
                print(f"  {k + 1} runs", file=sys.stderr)
    if sink:
        sink.close()

    outcomes = Counter(outcome(r) for r in results)
    print(f"\n{len(results)} runs (seeds {args.start}..{args.start + args.runs - 1}{', no faults' if args.clean else ''}).")
    print("Outcomes:")
    for name, n in outcomes.most_common():
        print(f"  {n:5d}  {name}")
    groups = defaultdict(list)
    for r in results:
        for kind, t, detail in r["problems"]:
            groups[(kind, tuple(r["minimal"].get(kind, [])))].append((r["seed"], t, detail))
    print("Failures (class, minimal faults: count, first seeds):" if groups else "No failures.")
    for (kind, minimal), items in sorted(groups.items(), key=lambda kv: -len(kv[1])):
        seeds_ = ", ".join(str(s) for s, _, _ in items[:5])
        print(f"  {kind:18s} [{','.join(minimal) or 'no faults'}]: {len(items)}  seeds {seeds_}")
        print(f"      e.g. seed {items[0][0]} at {items[0][1]} s: {items[0][2]}")


if __name__ == "__main__":
    main()
