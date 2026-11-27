#!/usr/bin/env python3
"""Many simulated runs of the three behaviours on the course obstacles, with the things that
will differ on the robot randomised, to find out how often each one works and how it fails.

    python3 tools/sweep_sim.py --cases bar,slab --runs 100 --workers 4 --out runs.jsonl
    python3 tools/sweep_sim.py --summary runs.jsonl            # the table again, from saved runs

Laptop only (needs MuJoCo and the Pupper model: set PUPPER_XML as for the tests). Nothing
here talks to the robot.

Each run goes through the same code the robot runs, joined by simple stand-ins where the
robot itself would be:

  1. **Camera.** A fisheye frame is rendered (sim/render_fisheye.py) from where the lens
     really is - height, pitch, roll and fore-aft position drawn around the measured 165 mm
     and 11 deg up - and read with the pose the robot *believes* (the calibrated nominal,
     or ``--assumed yaml``: whatever config/terrain.yaml says). Ten frames, each with fresh
     sensor noise and JPEG, as the camera node delivers them.
  2. **Measure, square up, decide.** terrain/gauge.py measures; the real state machine's
     ``_after_gauge`` (terrain/state_machine.py) asks for turn / sidestep / forward pulses
     until it is square and centred, then decides. A pulse moves the robot by what it
     commanded times a walking factor drawn per run (the real walking policy is not in the
     simulation); then the obstacle is measured again from there.
  3. **Cross.** A crossing is planned as robot/stepover_node.py plans it (the configured
     CrawlConfig, no search for the bar) from the *measured* obstacle, and played in MuJoCo
     (sim/crossing_sim.py's model, robot and contact set-up) against the *true* obstacle,
     with floor friction and payload drawn per run.
  4. **Walk around** (carton, round bar): the scan from rendered frames decides whether it
     is certainly tall (walked around from afar) or worth a closer look (then measured and
     decided as above); the walk-around itself runs in sim/avoid_sim.py with the walking
     factor drawn per run.

Cases: ``bar`` (19 x 89 x 600 mm flat bar, stepped over), ``slab`` (15 x 565 x 380 mm,
scaled across), ``carton`` (455 x 348 x 130 mm, walked around), ``roundbar`` (50 mm x 700 mm,
walked around), ``sequence`` (the bar, then the slab 0.70 m beyond it, crossed one after the
other in the same physics run).

What is not simulated: the walking policy (pulses and approaches are teleports by a random
factor of what was commanded), glare and uneven light (tools/stress_gauge.py does that on real
frames), the body bobbing while it walks, the IMU.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
import traceback
from dataclasses import replace
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

# The course obstacles (m)
BAR = dict(height=0.019, depth=0.089, width=0.600, mass=0.6)
SLAB = dict(height=0.015, depth=0.380, width=0.565, mass=2.0)
CARTON = dict(height=0.130, depth=0.348, width=0.455)
ROUND = dict(diameter=0.050, length=0.700)
SLAB_BEYOND_BAR = 0.70          # near face to near face (deploy.md, path-navigation demo.yaml)
HARDWOOD = (168, 120, 72)       # RGB of the rendered wooden floor (as in tests/test_perception.py)
NOMINAL_POSE = dict(height=0.165, pitch_deg=-11.0, x_forward=0.150)    # what the frames of 26 Nov showed

CASES = ("bar", "slab", "carton", "roundbar", "sequence")


# ---------------------------------------------------------------------------- sampling
def sample(case: str, k: int, seed: int, assumed: str, cam_spread: float = 1.0) -> dict:
    """Everything random about run k of a case. ``cam_spread`` scales how far the lens may be
    from where the robot believes it is (1: +-10 mm in height, +-2 deg in pitch)."""
    rng = np.random.default_rng([seed, CASES.index(case), k])
    p = dict(case=case, k=k, seed=seed, assumed=assumed, cam_spread=cam_spread)
    # where the lens really is, around what the robot's frames showed
    p["cam_height"] = NOMINAL_POSE["height"] + cam_spread * float(rng.uniform(-0.010, 0.010))
    p["cam_pitch_deg"] = NOMINAL_POSE["pitch_deg"] + cam_spread * float(rng.uniform(-2.0, 2.0))
    p["cam_roll_deg"] = float(rng.normal(0.0, 0.5))
    p["cam_x_forward"] = NOMINAL_POSE["x_forward"] + cam_spread * float(rng.uniform(-0.010, 0.010))
    # the obstacle where the approach leaves the robot (near face centre, body frame)
    p["near"] = float(rng.uniform(0.33, 0.41))
    p["lateral"] = float(rng.uniform(-0.10, 0.10))
    p["yaw_deg"] = float(rng.uniform(-8.0, 8.0))
    # the walking policy's response to the squaring-up pulses: actual / commanded
    p["k_fwd"] = float(rng.uniform(0.45, 0.65))          # 0.53 measured forwards on the owner's floor
    p["k_turn"] = float(rng.uniform(0.40, 1.00))
    p["k_side"] = float(rng.uniform(0.30, 0.90))
    p["floor_friction"] = float(rng.uniform(0.35, 1.0))
    p["payload_scale"] = float(rng.uniform(0.7, 1.3))   # true head hardware / what the planner assumes
    p["render_seed"] = int(rng.integers(1 << 30))
    if case in ("carton", "roundbar"):
        p["near"] = float(rng.uniform(0.33, 0.41))
        p["lateral"] = float(rng.uniform(-0.15, 0.15))
        p["yaw_deg"] = float(rng.uniform(-20.0, 20.0))
        p["walk_speed_scale"] = float(rng.uniform(0.45, 0.65))
        p["walk_lag"] = float(rng.uniform(0.15, 0.40))
    if case == "sequence":
        p["slab_lateral"] = p["lateral"] + float(rng.uniform(-0.05, 0.05))
        p["slab_yaw_deg"] = float(rng.uniform(-4.0, 4.0))
        p["slab_stop"] = float(rng.uniform(0.33, 0.40))         # where the second approach stops
        p["approach_heading_deg"] = float(rng.normal(0.0, 1.5))  # heading error of that walk
    return p


# ---------------------------------------------------------------------------- shared state per worker
_W = {}


def _setup():
    if _W:
        return _W
    import cv2
    from terrain.config import TerrainConfig
    cfg = TerrainConfig()
    _W.update(cv2=cv2, cfg=cfg)
    return _W


def _poses(p):
    from terrain.camera_model import CameraPose
    W = _setup()
    true = CameraPose(height=p["cam_height"], pitch_deg=p["cam_pitch_deg"], roll_deg=p["cam_roll_deg"],
                      x_forward=p["cam_x_forward"])
    assumed = CameraPose(**NOMINAL_POSE) if p["assumed"] == "nominal" else W["cfg"].pose
    return true, assumed


def _floor_model(true, assumed, seed):
    """The floor's colour as tools/learn_colors.py learns it: a clear view, read with the
    pose the robot believes."""
    from sim.render_fisheye import Scene, render
    from terrain import floor_model as FM
    from terrain.camera_model import PolarView
    W = _setup()
    view = PolarView(W["cfg"].model, assumed)
    pol, _ = view.sample(render(Scene(floor=HARDWOOD, seed=seed), W["cfg"].model, true))
    hsv = W["cv2"].cvtColor(pol, W["cv2"].COLOR_RGB2HSV)
    fm = FM.FloorModel()
    fm.learn(hsv[np.ix_((view.depressions > 20) & (view.depressions < 58), np.abs(view.bearings) < 35)])
    return fm


def _jpeg_frames(base_rgb, rng, n, noise=3.0):
    """n camera frames of one scene: fresh sensor noise each, through JPEG (quality 90)."""
    cv2 = _setup()["cv2"]
    out = []
    for _ in range(n):
        img = np.clip(base_rgb.astype(np.float32) + rng.normal(0, noise, base_rgb.shape), 0, 255).astype(np.uint8)
        ok, buf = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])
        out.append(cv2.imdecode(buf, cv2.IMREAD_UNCHANGED))
    return out


def _scene_objects(obs):
    """Render objects for obstacles given in the body frame: dicts with kind, x, y (near face
    centre / nearest point of a round bar), yaw_deg (+ = turned anticlockwise seen from above)."""
    from sim.render_fisheye import BEIGE, WHITE, Cuboid, Cylinder
    boxes, cyls = [], []
    for o in obs:
        if o["kind"] in ("bar", "slab"):
            g = BAR if o["kind"] == "bar" else SLAB
            boxes.append(Cuboid(o["x"], o["y"], g["depth"], g["width"], g["height"], yaw_deg=o["yaw_deg"], side=BEIGE, top=WHITE))
        elif o["kind"] == "carton":
            boxes.append(Cuboid(o["x"], o["y"], CARTON["depth"], CARTON["width"], CARTON["height"], yaw_deg=o["yaw_deg"],
                                side=WHITE, top=WHITE, side_shade=0.8))
        elif o["kind"] == "roundbar":
            cyls.append(Cylinder(o["x"], o["y"], ROUND["diameter"], ROUND["length"], yaw_deg=o["yaw_deg"]))
    return boxes, cyls


def _render(obs, true, seed):
    from sim.render_fisheye import Scene, render
    boxes, cyls = _scene_objects(obs)
    return render(Scene(boxes=boxes, cylinders=cyls, floor=HARDWOOD, seed=seed, noise=0.0), _setup()["cfg"].model, true)


def _gauge(obs, true, assumed, floor, rng, seed):
    """Ten frames through terrain/gauge.py -> BoxEstimate (or None)."""
    from terrain.gauge import BoxGauge, fine_view, measure_obstacle
    W = _setup()
    cfg = W["cfg"]
    view = fine_view(cfg.model, assumed)
    g = BoxGauge(cfg.decision.min_frames)
    for img in _jpeg_frames(_render(obs, true, seed), rng, cfg.decision.min_frames):
        g.add(measure_obstacle(img, view, floor, cfg=cfg.gauge, ball=cfg.ball, near_depression_deg=cfg.near_depression_deg,
                               landing=cfg.decision.landing_clear))
    return g.estimate()


def _move(obs, dx, dy, dth):
    """The robot moved (dx, dy) in its old frame and turned dth (+ left): the obstacles in the new frame."""
    c, s = math.cos(-dth), math.sin(-dth)
    out = []
    for o in obs:
        x, y = o["x"] - dx, o["y"] - dy
        out.append(dict(o, x=c * x - s * y, y=s * x + c * y, yaw_deg=o["yaw_deg"] - math.degrees(dth)))
    return out


def _est_dict(e):
    if e is None:
        return None
    keys = ("near_x", "skew", "height", "depth", "width", "y_left", "y_right", "frames", "height_sigma", "landing_floor",
            "height_upper", "extent", "side_seen", "far_seen", "ends_seen")
    return {k: (None if isinstance(getattr(e, k), float) and not math.isfinite(getattr(e, k)) else getattr(e, k)) for k in keys}


def measure_and_decide(obs, p, true, assumed, floor, rng, log):
    """The GAUGE -> ALIGN -> GAUGE ... -> decision loop of the real state machine, with the
    robot's pulses applied to the scene. Returns (decision, estimate, obstacles as they now
    lie in the body frame, pulses)."""
    from terrain.state_machine import ALIGN, GAUGE, Inputs, TerrainFsm
    W = _setup()
    fsm = TerrainFsm(W["cfg"].fsm)
    t = 100.0
    pulses = []
    for attempt in range(W["cfg"].fsm.align_max_tries + 2):
        est = _gauge(obs, true, assumed, floor, rng, p["render_seed"] + 17 * attempt)
        fsm.state, fsm.estimate = GAUGE, est
        event = fsm._after_gauge(est, Inputs(t=t, yaw=0.0, level=True), t)
        log.append(event)
        if fsm.state != ALIGN:
            return fsm.decision, est, obs, pulses
        (vx, vy, wz), t_end = fsm.pulse
        if p.get("patch_align"):
            wz = -wz                # the proposed fix: turn towards square, not away from it (see --patch-align)
        dur = t_end - t
        dx = vx * dur * p["k_fwd"] + rng.normal(0, 0.004)
        dy = vy * dur * p["k_side"] + rng.normal(0, 0.004)
        dth = wz * dur * p["k_turn"] + math.radians(rng.normal(0, 0.4))
        pulses.append((round(vx, 3), round(vy, 3), round(wz, 3), round(dur, 2)))
        obs = _move(obs, dx, dy, dth)
        t = t_end + 1.5
    return fsm.decision, est, obs, pulses


# ---------------------------------------------------------------------------- physics
def _yaw_quat(yaw):
    return [math.cos(yaw / 2), 0.0, 0.0, math.sin(yaw / 2)]


def _quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return [w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2]


class World:
    """The Pupper model with the course obstacles placed where they truly are (world frame =
    the robot's body frame at the start), built from sim/crossing_sim.py."""

    def __init__(self, obs_world, p):
        import mujoco
        from sim import crossing_sim as CS
        from terrain import kinematics as K
        W = _setup()
        crawl = W["cfg"].crawl
        self.mj, self.K, self.CS = mujoco, K, CS
        true_masses = tuple((m * p["payload_scale"], pos) for m, pos in crawl.extra_masses)
        geoms = [BAR if o["kind"] == "bar" else SLAB for o in obs_world]
        more = tuple((g["height"], g["depth"], g["width"], o["x"], g["mass"]) for g, o in zip(geoms, obs_world))
        self.sim_cfg = CS.SimConfig(box_height=0.0, more_boxes=more, floor_friction=p["floor_friction"],
                                    extra_masses=true_masses, kp=crawl.kp, kd=crawl.kd)
        self.model = CS.build_model(CS.find_model(), self.sim_cfg, 10.0)
        m = self.model
        # place each obstacle: build_model puts them square on the centreline; move them to the truth
        self.obs = []
        for k, o in enumerate(obs_world):
            g = BAR if o["kind"] == "bar" else SLAB
            body = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"box{k + 2}")
            jnt = m.body_jntadr[body]
            adr = m.jnt_qposadr[jnt]
            yaw = math.radians(o["yaw_deg"])
            cx = o["x"] + 0.5 * g["depth"] * math.cos(yaw)
            cy = o["y"] + 0.5 * g["depth"] * math.sin(yaw)
            m.qpos0[adr:adr + 7] = [cx, cy, g["height"] / 2, *_yaw_quat(yaw)]
            geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, f"box{k + 2}_geom")
            self.obs.append(dict(o, body=body, geom=geom, start=np.array([cx, cy]), g=g))
        self.data = mujoco.MjData(m)
        self.robot = CS.Robot(m, self.data)
        self.robot.reset_standing()
        self.box_geoms = {o["geom"]: i for i, o in enumerate(self.obs)}
        self.foot_geoms = {}
        for i, leg in enumerate(K.LEGS):
            b3 = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"leg_{leg}_3")
            for gi in range(m.ngeom):
                if m.geom_bodyid[gi] == b3 and m.geom_type[gi] == mujoco.mjtGeom.mjGEOM_SPHERE:
                    self.foot_geoms[gi] = i
        self.stand(1.0)

    def stand(self, seconds):
        """The walking controller holding the default pose (as tests/test_robot_nodes.py does)."""
        self.robot.command(self.K.DEFAULT_JOINT_POS, 7.5, 0.25)
        self.robot.step(seconds)

    def pose(self):
        pos = self.robot.position()
        return float(pos[0]), float(pos[1]), self.robot.yaw()

    def teleport(self, dx, dy, dth):
        """Stand-in for the walking policy: the body moves (dx, dy) in its own frame and turns
        dth, the legs keep their pose; then it settles."""
        d = self.data
        x, y, yaw = self.pose()
        c, s = math.cos(yaw), math.sin(yaw)
        d.qpos[0] += c * dx - s * dy
        d.qpos[1] += s * dx + c * dy
        d.qpos[3:7] = _quat_mul(_yaw_quat(dth), list(d.qpos[3:7]))
        d.qvel[:6] = 0.0
        self.mj.mj_forward(self.model, d)
        self.stand(1.0)

    def relative(self, o):
        """An obstacle's near face centre and yaw in the robot's body frame now."""
        x, y, yaw = self.pose()
        yaw_o = math.radians(o["yaw_deg"])
        dx, dy = o["x"] - x, o["y"] - y
        c, s = math.cos(-yaw), math.sin(-yaw)
        return dict(kind=o["kind"], x=c * dx - s * dy, y=s * dx + c * dy, yaw_deg=math.degrees(yaw_o - yaw))

    def cross(self, est, method, target):
        """Plan from the measurement as robot/stepover_node.py does; play it here."""
        from terrain.crawl_planner import BoxSpec, plan_crossing, plan_scale_across
        from terrain.handover import Handover
        from terrain.player import DONE, STOPPED, TrajectoryPlayer
        W = _setup()
        cfg = W["cfg"]
        crawl = cfg.crawl
        K, mj, m, d, robot = self.K, self.mj, self.model, self.data, self.robot
        q = robot.joints()
        shift = max(-0.03, min(0.03, float(np.mean(K.all_feet(q)[:, 0] - K.default_stance()[1][:, 0]))))
        box = BoxSpec(height=est.height, depth=est.depth, near_x=est.near_x - shift, skew=est.skew)
        t0 = time.perf_counter()
        plan = plan_scale_across(box, crawl) if method == "slab" else plan_crossing(box, crawl, search=cfg.allow_search)
        out = dict(plan_s=round(time.perf_counter() - t0, 1), planned=bool(plan.ok), plan_reason=plan.reason if not plan.ok else "")
        if not plan.ok:
            return out
        player = TrajectoryPlayer(plan, replace(cfg.player, kp=crawl.kp, kd=crawl.kd))
        hand = Handover(player)
        o_t = self.obs[target]
        start = robot.position().copy()
        box_start = [d.xpos[o["body"]].copy() for o in self.obs]
        force = np.zeros(6)
        unplanned, foot_side, touches = 0.0, 0.0, {}
        dt, t, t_hold = 1.0 / 100.0, 0.0, None
        while True:
            cmd = hand.update(t, robot.joints(), robot.tilt(), switched=t >= 0.05)
            robot.command(cmd.q, cmd.kp, cmd.kd)
            robot.step(dt)
            t += dt
            for c in range(d.ncon):
                con = d.contact[c]
                if con.geom1 in self.box_geoms:
                    other = con.geom2
                elif con.geom2 in self.box_geoms:
                    other = con.geom1
                else:
                    continue
                if m.geom_bodyid[other] == 0 or other in self.box_geoms:
                    continue
                if other in self.foot_geoms and abs(con.frame[2]) > 0.8:
                    continue                    # a foot standing on a top
                mj.mj_contactForce(m, d, c, force)
                fn = float(force[0])
                if other in self.foot_geoms:
                    foot_side = max(foot_side, fn)
                unplanned = max(unplanned, fn)
                name = mj.mj_id2name(m, mj.mjtObj.mjOBJ_GEOM, other) or ("foot_" + K.LEGS[self.foot_geoms[other]]
                                                                            if other in self.foot_geoms else f"geom{other}")
                touches[name] = round(max(touches.get(name, 0.0), fn), 1)
            if player.state == DONE and t_hold is None:
                t_hold = t + 0.5
            if (t_hold is not None and t >= t_hold) or player.state == STOPPED or t > plan.duration + 30.0:
                break
        pos = robot.position()
        g = o_t["g"]
        yaw_o = math.radians(o_t["yaw_deg"])
        n = np.array([math.cos(yaw_o), math.sin(yaw_o)])
        far_pt = np.array([o_t["x"], o_t["y"]]) + g["depth"] * n
        rear = [d.site_xpos[mj.mj_name2id(m, mj.mjtObj.mjOBJ_SITE, f"leg_{leg}_3_foot_site")][:2] for leg in ("back_r", "back_l")]
        beyond = min(float((r - far_pt) @ n) for r in rear)
        moved = max(float(np.linalg.norm(d.xpos[o["body"]][:2] - b0[:2])) for o, b0 in zip(self.obs, box_start))
        ok = player.result == "done" and robot.tilt() < math.radians(10) and pos[2] > 0.13 and beyond > 0.0
        out.update(result=player.result or "timeout", crossed=bool(ok), duration=round(t, 1),
                   max_tilt_deg=round(player.max_tilt_deg, 1), rear_beyond_far_mm=round(beyond * 1000, 1),
                   unplanned_contact_n=round(unplanned, 1), foot_side_n=round(foot_side, 1), touches=touches,
                   box_moved_mm=round(moved * 1000, 1), final_z=round(float(pos[2]), 3),
                   advance=round(float(pos[0] - start[0]), 3),
                   planned_advance=round(float(plan.body[-1, 0] - plan.body[0, 0]), 3),
                   events=[e for _, e in player.events][-4:])
        self.stand(1.0)
        return out


# ---------------------------------------------------------------------------- the cases
def _truth_vs_est(o, est, g):
    """Measurement errors against the truth (o: obstacle in the body frame)."""
    if est is None:
        return {}
    yaw = math.radians(o["yaw_deg"])
    # the near face where it crosses the robot's centreline (y = 0)
    near_c = o["x"] + o["y"] * math.tan(yaw) if abs(math.cos(yaw)) > 0.2 else o["x"]
    true_skew = -yaw
    f = lambda v: None if v is None or not math.isfinite(v) else round(v * 1000, 1)     # noqa: E731
    return dict(err_near_mm=f(est.near_x - near_c), err_height_mm=f(est.height - g["height"]),
                err_depth_mm=f(est.depth - g["depth"]), err_skew_deg=None if not math.isfinite(est.skew) else
                round(math.degrees(est.skew - true_skew), 2), true_near=round(near_c, 3), true_skew_deg=round(math.degrees(true_skew), 2),
                true_offset_mm=round(o["y"] * 1000, 1))


def _crossing_case(p, rec, true, assumed, floor, rng, kind):
    from terrain import decision as D
    g = BAR if kind == "bar" else SLAB
    want = D.STEP_OVER if kind == "bar" else D.SCALE_ACROSS
    obs = [dict(kind=kind, x=p["near"], y=p["lateral"], yaw_deg=p["yaw_deg"])]
    log = []
    dec, est, obs, pulses = measure_and_decide(obs, p, true, assumed, floor, rng, log)
    rec.update(decision=dec.action, method=dec.method, reasons=dec.reasons, est=_est_dict(est), pulses=pulses,
               events=log[-3:], **_truth_vs_est(obs[0], est, g))
    if dec.action != want:
        rec["outcome"] = "refused" if dec.action == D.GO_AROUND else f"wrong_decision:{dec.action}"
        return
    if p.get("no_physics"):
        rec["outcome"] = "decided_ok"
        return
    w = World(obs, p)
    rec["physics"] = w.cross(est, dec.method, 0)
    rec["outcome"] = _physics_outcome(rec["physics"])


def _physics_outcome(ph):
    if not ph["planned"]:
        return "no_plan"
    if not ph["crossed"]:
        return f"failed:{ph['result']}" if ph["result"] != "done" else "failed:short_or_fell"
    if ph["box_moved_mm"] > 10.0:
        return "crossed_box_pushed"
    if ph["unplanned_contact_n"] > 10.0:
        return "crossed_hard_touch"
    return "ok"


def _sequence_case(p, rec, true, assumed, floor, rng):
    from terrain import decision as D
    yaw0 = math.radians(p["yaw_deg"])
    bar = dict(kind="bar", x=p["near"], y=p["lateral"], yaw_deg=p["yaw_deg"])
    # the slab's near face 0.70 m beyond the bar's, along the robot's starting heading
    slab = dict(kind="slab", x=p["near"] + SLAB_BEYOND_BAR, y=p["slab_lateral"], yaw_deg=p["slab_yaw_deg"])
    log = []
    dec, est, now, pulses = measure_and_decide([bar, slab], p, true, assumed, floor, rng, log)
    rec.update(bar_decision=dec.action, bar_reasons=dec.reasons, bar_est=_est_dict(est), bar_pulses=pulses,
               **{f"bar_{k}": v for k, v in _truth_vs_est(now[0], est, BAR).items()})
    if dec.action != D.STEP_OVER:
        rec["outcome"] = "bar_refused" if dec.action == D.GO_AROUND else f"bar_wrong_decision:{dec.action}"
        return
    w = World(now, p)              # world frame = the body frame after squaring up to the bar
    ph = w.cross(est, dec.method, 0)
    rec["bar_physics"] = ph
    out = _physics_outcome(ph)
    if out not in ("ok", "crossed_hard_touch"):
        rec["outcome"] = "bar_" + out
        return
    # the walk up to the slab (a teleport: the walking policy is not simulated)
    rel = w.relative(w.obs[1])
    rec["slab_after_bar"] = dict(x=round(rel["x"], 3), y=round(rel["y"], 3), yaw_deg=round(rel["yaw_deg"], 2))
    walk = rel["x"] - p["slab_stop"]
    if walk > 0.0:
        w.teleport(walk, 0.0, math.radians(p["approach_heading_deg"]))
    rel = w.relative(w.obs[1])
    log2 = []
    dec2, est2, rel_after, pulses2 = measure_and_decide([rel], p, true, assumed, floor, rng, log2)
    rec.update(slab_decision=dec2.action, slab_reasons=dec2.reasons, slab_est=_est_dict(est2), slab_pulses=pulses2,
               **{f"slab_{k}": v for k, v in _truth_vs_est(rel_after[0], est2, SLAB).items()})
    if dec2.action != D.SCALE_ACROSS:
        rec["outcome"] = "slab_refused" if dec2.action == D.GO_AROUND else f"slab_wrong_decision:{dec2.action}"
        return
    # apply the squaring-up pulses to the physical robot as well
    for (vx, vy, wz, dur) in pulses2:
        w.teleport(vx * dur * p["k_fwd"], vy * dur * p["k_side"], wz * dur * p["k_turn"])
    rel_now = w.relative(w.obs[1])
    rec["slab_rel_at_crossing"] = dict(x=round(rel_now["x"], 3), y=round(rel_now["y"], 3), yaw_deg=round(rel_now["yaw_deg"], 2))
    ph2 = w.cross(est2, dec2.method, 1)
    rec["slab_physics"] = ph2
    out2 = _physics_outcome(ph2)
    rec["outcome"] = "ok" if out2 == "ok" and out == "ok" else ("slab_" + out2 if out2 != "ok" else "bar_" + out)


class _RotRect:
    """sim/avoid_sim.py's Rect, turned: what the walk-around simulation needs of an obstacle."""

    def __init__(self, cx, cy, depth, width, yaw, tall=True, is_box=False):
        self.cx, self.cy, self.yaw = cx, cy, yaw
        c, s = abs(math.cos(yaw)), abs(math.sin(yaw))
        self._d, self._w = depth, width
        self.depth, self.width = depth * c + width * s, depth * s + width * c     # axis-aligned bounds (shortest_detour)
        self.tall, self.is_box = tall, is_box

    def corners(self):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        hx, hy = self._d / 2, self._w / 2
        loc = np.array([[-hx, -hy], [hx, -hy], [hx, hy], [-hx, hy]])
        return loc @ np.array([[c, s], [-s, c]]) + np.array([self.cx, self.cy])

    def distance(self, p):
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        dx, dy = p[0] - self.cx, p[1] - self.cy
        lx, ly = c * dx + s * dy, -s * dx + c * dy
        return math.hypot(max(abs(lx) - self._d / 2, 0.0), max(abs(ly) - self._w / 2, 0.0))


def _scan_look(obs, true, assumed, floor, rng, seed):
    """What the walking scan makes of the obstacle (robot/terrain_node.py's per-frame path):
    the obstacle in the lane, or None."""
    from terrain import floor_model as FM
    from terrain.camera_model import PolarView
    from terrain.scan import build_scan, find_obstacles
    W = _setup()
    cfg, cv2 = W["cfg"], W["cv2"]
    img = _jpeg_frames(_render(obs, true, seed), rng, 1)[0]
    view = PolarView(cfg.model, assumed)
    pol, valid = view.sample(img)
    labels = FM.classify(cv2.cvtColor(pol, cv2.COLOR_RGB2HSV), valid, floor, ball=cfg.ball)
    obstacles = find_obstacles(build_scan(labels, view, cfg.scan), cfg.scan)
    lane = [o for o in obstacles if o.points is not None and np.any(np.abs(o.points[:, 1]) < cfg.fsm.corridor_half + 0.05)]
    return min(lane, key=lambda o: o.nearest) if lane else None


def _walk_around_case(p, rec, true, assumed, floor, rng, kind):
    from sim.avoid_sim import AvoidSimConfig, run_avoid
    from terrain import decision as D
    W = _setup()
    cfg = W["cfg"]
    # 1. walking towards it: what the scan says at each distance, until the state machine would commit
    looks, verdict = [], None
    for dist in (1.25, 1.15, 1.05, 0.95, 0.85, 0.75, 0.65, 0.55):
        o = dict(kind=kind, x=dist, y=p["lateral"], yaw_deg=p["yaw_deg"])
        ob = _scan_look([o], true, assumed, floor, rng, p["render_seed"] + int(dist * 100))
        if ob is None:
            looks.append((dist, "unseen"))
            continue
        looks.append((dist, "tall" if ob.tall else ("may_be_low" if ob.is_box else "other"), round(ob.min_height, 3),
                      round(ob.height, 3) if math.isfinite(ob.height) else None))
        rng_body = ob.nearest + cfg.pose.x_forward
        if ob.tall and rng_body <= cfg.fsm.engage_range:
            verdict = "tall_from_afar"
            break
        if ob.is_box and rng_body <= cfg.fsm.box_engage_range:
            verdict = "approach"
            break
    rec["looks"] = looks
    rec["first_look"] = verdict
    if verdict is None:
        rec["outcome"] = "never_engaged"
        return
    if verdict == "approach":
        obs = [dict(kind=kind, x=p["near"], y=p["lateral"], yaw_deg=p["yaw_deg"])]
        log = []
        dec, est, obs, pulses = measure_and_decide(obs, p, true, assumed, floor, rng, log)
        rec.update(decision=dec.action, reasons=dec.reasons, est=_est_dict(est), pulses=pulses)
        if dec.action in D.CROSSINGS:
            rec["outcome"] = f"DANGER_would_cross:{dec.action}"
            return
        if dec.action == D.IGNORE:
            rec["outcome"] = "DANGER_ignored"
            return
    # 2. the walk-around itself (the obstacle as refused: tall to the state machine from now on)
    if kind == "carton":
        dims = (CARTON["depth"], CARTON["width"])
    else:
        dims = (ROUND["diameter"], ROUND["length"])
    yaw = math.radians(p["yaw_deg"])
    start_dist = 1.5
    rect = _RotRect(start_dist + 0.5 * dims[0] * math.cos(yaw), p["lateral"] + 0.5 * dims[0] * math.sin(yaw), dims[0], dims[1], yaw)
    acfg = AvoidSimConfig(obstacles=[rect], speed_scale=p["walk_speed_scale"], lag=p["walk_lag"],
                          camera_x=cfg.pose.x_forward, seed=p["render_seed"] % 100000, max_time=60.0)
    res = run_avoid(acfg, replace(cfg.fsm))
    rec["avoid"] = dict(reached=res.reached, time=round(res.time, 1), ratio=round(res.path_length / res.shortest, 3),
                        min_clear_mm=round(res.min_clearance * 1000), longest_stop=round(res.longest_stop, 1),
                        states=res.states, last_events=[e for _, e in res.events][-3:])
    if not res.reached:
        rec["outcome"] = "avoid_did_not_reach" + (":hold" if "HOLD" in res.states else "")
    elif res.min_clearance < 0.15:
        rec["outcome"] = "avoid_too_close"
    else:
        rec["outcome"] = "ok"


def run_one(p: dict) -> dict:
    t0 = time.perf_counter()
    rec = dict(p)
    try:
        rng = np.random.default_rng(p["render_seed"])
        true, assumed = _poses(p)
        floor = _floor_model(true, assumed, p["render_seed"] % 1000)
        if p["case"] in ("bar", "slab"):
            _crossing_case(p, rec, true, assumed, floor, rng, p["case"])
        elif p["case"] == "sequence":
            _sequence_case(p, rec, true, assumed, floor, rng)
        else:
            _walk_around_case(p, rec, true, assumed, floor, rng, p["case"])
    except Exception as e:      # a crash is a result too: keep going
        rec["outcome"] = f"error:{type(e).__name__}"
        rec["error"] = traceback.format_exc()[-1500:]
    rec["wall_s"] = round(time.perf_counter() - t0, 1)
    return rec


# ---------------------------------------------------------------------------- summary
def wilson(k, n, z=1.96):
    if n == 0:
        return (0.0, 0.0)
    ph = k / n
    den = 1 + z * z / n
    c = (ph + z * z / (2 * n)) / den
    h = z * math.sqrt(ph * (1 - ph) / n + z * z / (4 * n * n)) / den
    return (max(0.0, c - h), min(1.0, c + h))


def summarise(recs) -> str:
    from collections import Counter, defaultdict
    by = defaultdict(list)
    for r in recs:
        tag = r.get("assumed", "nominal") + (" align-patched" if r.get("patch_align") else "")
        if r.get("cam_spread", 1.0) != 1.0:
            tag += f" spread {r['cam_spread']:g}"
        by[(r["case"], tag)].append(r)
    lines = []
    for (case, assumed), rs in sorted(by.items()):
        n = len(rs)
        ok = sum(r["outcome"] == "ok" for r in rs)
        lo, hi = wilson(ok, n)
        lines.append(f"{case:9s} camera {assumed:32s}: {ok}/{n} ok = {100 * ok / n:.0f} % (95 % CI {100 * lo:.0f}-{100 * hi:.0f} %)")
        for what, c in Counter(r["outcome"] for r in rs).most_common():
            if what != "ok":
                lines.append(f"    {c:4d}  {what}")
        reasons = Counter()
        for r in rs:
            for key in ("reasons", "bar_reasons", "slab_reasons"):
                for why in r.get(key) or []:
                    reasons[_reason_key(why)] += 1
        for why, c in reasons.most_common(6):
            lines.append(f"          reason x{c}: {why}")
    return "\n".join(lines)


def _reason_key(why: str) -> str:
    """A refusal reason without its numbers, for counting."""
    import re
    return re.sub(r"[-+]?\d+(\.\d+)?", "#", why)[:110]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cases", default="bar,slab,carton,roundbar,sequence")
    ap.add_argument("--runs", type=int, default=20, help="runs per case")
    ap.add_argument("--start", type=int, default=0, help="first run index (to add runs to an earlier sweep)")
    ap.add_argument("--seed", type=int, default=2026)
    ap.add_argument("--workers", type=int, default=max(1, (os.cpu_count() or 2) // 2))
    ap.add_argument("--assumed", choices=("nominal", "yaml"), default="nominal",
                    help="the camera pose the robot believes: the measured nominal (165 mm, 11 deg up) or config/terrain.yaml's")
    ap.add_argument("--cam-spread", type=float, default=1.0,
                    help="how far the lens may really be from the assumed pose: 1 = +-10 mm height, +-2 deg pitch, "
                         "+-10 mm fore-aft; 0.25 = a good calibration")
    ap.add_argument("--no-physics", action="store_true", help="stop at the decision (perception and squaring up only)")
    ap.add_argument("--patch-align", action="store_true",
                    help="reverse the squaring-up turn of terrain/state_machine.py (the proposed fix; the file is not changed)")
    ap.add_argument("--out", default="sweep_runs.jsonl", help="one JSON line per run (appended)")
    ap.add_argument("--summary", help="only print the summary of an earlier --out file")
    args = ap.parse_args()
    if args.summary:
        print(summarise([json.loads(line) for line in Path(args.summary).read_text().splitlines() if line.strip()]))
        return
    cases = [c.strip() for c in args.cases.split(",") if c.strip()]
    bad = [c for c in cases if c not in CASES]
    if bad:
        raise SystemExit(f"unknown case(s) {bad}; choose from {CASES}")
    if any(c in ("bar", "slab", "sequence") for c in cases) and not args.no_physics:
        from sim.crossing_sim import find_model
        find_model()                # fail now, not in every worker
    jobs = []
    for k in range(args.start, args.start + args.runs):          # interleaved, so a stopped sweep is still balanced
        for c in cases:
            p = sample(c, k, args.seed, args.assumed, args.cam_spread)
            p["no_physics"] = args.no_physics
            p["patch_align"] = args.patch_align
            jobs.append(p)
    print(f"{len(jobs)} runs on {args.workers} workers -> {args.out}", flush=True)
    recs = []
    t0 = time.time()
    with open(args.out, "a") as f:
        if args.workers > 1:
            import multiprocessing as mp
            pool = mp.get_context("spawn").Pool(args.workers)
            it = pool.imap_unordered(run_one, jobs)
        else:
            it = map(run_one, jobs)
        for i, rec in enumerate(it, 1):
            recs.append(rec)
            f.write(json.dumps(rec, default=float) + "\n")
            f.flush()
            if i % 10 == 0 or i == len(jobs):
                print(f"{i}/{len(jobs)} done, {time.time() - t0:.0f} s", flush=True)
    print(summarise(recs))


if __name__ == "__main__":
    main()
