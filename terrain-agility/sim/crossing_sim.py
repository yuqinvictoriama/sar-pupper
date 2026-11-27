"""Physics simulation of the step-over: the Pupper v3 MuJoCo model, a box on the floor, and
the same planner and trajectory player the robot runs.

    python3 sim/crossing_sim.py --height 0.019 --depth 0.064            # one crossing, printed
    python3 sim/crossing_sim.py --height 0.03 --depth 0.064 --view      # with the viewer (laptop)

Needs ``pip install mujoco`` and the pupper_v3_description package with real meshes (the
copy inside pupperv3-monorepo needs ``git lfs pull``); point --model or $PUPPER_XML at
description/mujoco_xml/pupper_v3_complete.position.xml.

What the simulation adds to the planner's checks: real contact, motor position control with
the robot's gains and 3 N m torque limit, the legs' true collision envelope, sag, slip, and
measurement errors you inject (the box is not where the planner was told).
"""

from __future__ import annotations

import argparse
import math
import os
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from terrain import kinematics as K  # noqa: E402
from terrain.crawl_planner import BoxSpec, CrawlConfig, plan_crossing  # noqa: E402
from terrain.handover import Handover  # noqa: E402
from terrain.player import DONE, STOPPED, PlayerConfig, TrajectoryPlayer  # noqa: E402

MODEL_CANDIDATES = [
    os.environ.get("PUPPER_XML", ""),
    str(Path.home() / "pupper_v3_description/description/mujoco_xml/pupper_v3_complete.position.xml"),
    str(Path.home() / "pupperv3-monorepo/ros2_ws/src/pupper_v3_description/description/mujoco_xml/pupper_v3_complete.position.xml"),
    str(Path(__file__).resolve().parents[2] / "pupper_v3_description/description/mujoco_xml/pupper_v3_complete.position.xml"),
    str(Path(__file__).resolve().parents[3] / "pupper_v3_description/description/mujoco_xml/pupper_v3_complete.position.xml"),
]


def find_model(path: str | None = None) -> str:
    for c in ([path] if path else []) + MODEL_CANDIDATES:
        if c and Path(c).exists():
            return c
    raise FileNotFoundError("Pupper MuJoCo model not found; pass --model or set PUPPER_XML")


@dataclass
class SimConfig:
    box_height: float = 0.019
    box_depth: float = 0.064
    box_width: float = 0.50
    box_mass: float | None = 0.6          # kg; None = fixed to the floor
    box_friction: float = 0.6
    floor_friction: float = 0.8
    stand_off: float = 0.02               # front feet start this far before the take-off line (m)
    near: float | None = None             # or: the box's near face this far ahead of the body centre (m)
    # the hand-over from the walking controller, as robot/stepover_node.py does it
    handover: bool = True
    switch_delay: float = 0.05            # s the controller switch takes (legs only damped meanwhile)
    walk_kp: float = 5.5                  # gains the walking controller stands with before the hand-over
    walk_kd: float = 0.25
    skew_deg: float = 0.0                 # true rotation of the box about the vertical
    # what the planner is told, relative to the truth
    err_near: float = 0.0                 # m, + = planner thinks the box is farther than it is
    err_height: float = 0.0               # m, + = planner thinks the box is taller than it is
    err_depth: float = 0.0
    err_skew_deg: float = 0.0
    extra_masses: tuple = ()              # true extra hardware on the robot: (kg, [x, y, z])
    planner_extra_masses: tuple | None = None   # what the planner assumes (None = the truth)
    kp: float = 7.5
    kd: float = 0.25
    control_hz: float = 100.0
    joint_noise: float = 0.0              # rad, std of joint angle measurement noise
    seed: int = 0


@dataclass
class SimResult:
    planned: bool
    plan_reason: str = ""
    finished: bool = False
    result: str = ""
    crossed: bool = False                 # robot ended beyond the box, upright, plan completed
    duration: float = 0.0
    max_tilt_deg: float = 0.0
    handover_sag: float = 0.0             # m the body dropped while the legs changed hands
    final_height: float = 0.0
    final_x: float = 0.0
    final_yaw_deg: float = 0.0
    box_moved: float = 0.0                # m
    unplanned_contact_force: float = 0.0  # N, largest leg/body contact with the box that is not a foot on its top
    contacts: dict = field(default_factory=dict)   # geom name -> (max force N, first time s, phase)
    tilt_by_phase: dict = field(default_factory=dict)
    min_height: float = 1.0
    foot_side_contact_force: float = 0.0  # N, largest horizontal push of a foot against the box
    max_torque: float = 0.0
    swings: int = 0
    plan_margin: float = 0.0
    events: list = field(default_factory=list)

    def line(self) -> str:
        if not self.planned:
            return f"NO PLAN ({self.plan_reason})"
        tag = "CROSSED" if self.crossed else f"FAILED ({self.result})"
        return (f"{tag}: {self.duration:.1f} s, {self.swings} swings, max tilt {self.max_tilt_deg:.1f} deg, "
                f"box moved {self.box_moved * 1000:.1f} mm, unplanned contact {self.unplanned_contact_force:.1f} N, "
                f"max torque {self.max_torque:.2f} N m, final height {self.final_height * 1000:.0f} mm")


def build_model(xml: str, cfg: SimConfig, near_face_x: float):
    """Compile the robot model with the box, the legs' collision envelope and the gains."""
    import mujoco

    spec = mujoco.MjSpec.from_file(xml)
    for act in spec.actuators:
        act.gainprm[0] = cfg.kp
        act.biasprm[1] = -cfg.kp
        act.biasprm[2] = -cfg.kd
    for g in spec.geoms:
        if g.name == "floor":
            g.friction = [cfg.floor_friction, 0.02, 0.01]
        elif g.conaffinity == 1 and g.contype == 0:
            # The model's contacts are tuned soft for policy training (feet sink ~4 mm into
            # the floor). Real rubber feet give well under a millimetre; use MuJoCo's default.
            g.solimp[:3] = [0.9, 0.95, 0.001]

    def body(name):
        return spec.body(name) if hasattr(spec, "body") else spec.find_body(name)

    # the legs' real envelope (kinematics.*_RADIUS, measured from the meshes), so that a
    # shin or knee housing that would touch the box on the robot touches it here
    for leg in K.LEGS:
        lm = K.leg_model(leg)
        thigh, shin = body(f"leg_{leg}_2"), body(f"leg_{leg}_3")
        knee = lm.p[2]
        foot = lm.foot
        common = dict(type=mujoco.mjtGeom.mjGEOM_CAPSULE, contype=0, conaffinity=1, group=3,
                      friction=[0.5, 0.02, 0.01], rgba=[0.9, 0.5, 0.1, 0.4], density=0)
        thigh.add_geom(name=f"env_thigh_{leg}", fromto=[0, 0, 0, *knee], size=[K.THIGH_RADIUS - 0.0005, 0, 0], **common)
        split = foot * K.SHANK_SPLIT
        shin.add_geom(name=f"env_shin_u_{leg}", fromto=[0, 0, 0, *split], size=[K.SHANK_RADIUS_UPPER, 0, 0], **common)
        shin.add_geom(name=f"env_shin_l_{leg}", fromto=[*split, *(foot * 0.9)], size=[K.SHANK_RADIUS_LOWER, 0, 0], **common)

    base = body("base_link")
    for k, (mass, pos) in enumerate(cfg.extra_masses):
        b = base.add_body(name=f"extra_mass_{k}", pos=list(pos))
        b.mass = float(mass)
        b.inertia = [1e-5, 1e-5, 1e-5]
        b.ipos = [0, 0, 0]
        b.explicitinertial = True

    if cfg.box_height > 0:
        h, d, w = cfg.box_height, cfg.box_depth, cfg.box_width
        skew = math.radians(cfg.skew_deg)
        cx = near_face_x + 0.5 * d / math.cos(skew)
        # BoxSpec.skew > 0 means the near face is farther away on the left: a clockwise turn
        quat = [math.cos(skew / 2), 0, 0, -math.sin(skew / 2)]
        gkw = dict(name="box_geom", type=mujoco.mjtGeom.mjGEOM_BOX, size=[d / 2, w / 2, h / 2], contype=1,
                   conaffinity=1, friction=[cfg.box_friction, 0.02, 0.01], rgba=[0.2, 0.4, 0.9, 1.0])
        if cfg.box_mass is None:
            spec.worldbody.add_geom(pos=[cx, 0, h / 2], quat=quat, **gkw)
        else:
            bb = spec.worldbody.add_body(name="box", pos=[cx, 0, h / 2], quat=quat)
            bb.add_freejoint()
            bb.add_geom(mass=cfg.box_mass, **gkw)
    return spec.compile()


class Robot:
    """Joint order, measurement and command helpers around a compiled model."""

    def __init__(self, model, data):
        import mujoco

        self.mj = mujoco
        self.m, self.d = model, data
        self.qadr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in K.JOINT_NAMES]
        self.act = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in K.JOINT_NAMES]
        self.base = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        lim = np.concatenate([K.leg_model(leg).hw_limits for leg in K.LEGS], axis=0)
        self.lo, self.hi = lim[:, 0], lim[:, 1]

    def reset_standing(self):
        d = self.d
        self.mj.mj_resetData(self.m, d)
        d.qpos[0:3] = [0, 0, K.default_stance()[0] + 0.005]
        d.qpos[3:7] = [1, 0, 0, 0]
        for a, v in zip(self.qadr, K.DEFAULT_JOINT_POS):
            d.qpos[a] = v
        self.command(K.DEFAULT_JOINT_POS)
        self.mj.mj_forward(self.m, d)

    def joints(self):
        return np.array([self.d.qpos[a] for a in self.qadr])

    def command(self, q, kp=None, kd=None):
        if kp is not None:
            self.m.actuator_gainprm[self.act, 0] = kp
            self.m.actuator_biasprm[self.act, 1] = -kp
            self.m.actuator_biasprm[self.act, 2] = -kd
        self.d.ctrl[self.act] = np.clip(q, self.lo, self.hi)

    def tilt(self):
        R = self.d.xmat[self.base].reshape(3, 3)
        return float(math.acos(max(-1.0, min(1.0, R[2, 2]))))

    def yaw(self):
        R = self.d.xmat[self.base].reshape(3, 3)
        return float(math.atan2(R[1, 0], R[0, 0]))

    def position(self):
        return self.d.xpos[self.base].copy()

    def step(self, seconds):
        n = max(1, int(round(seconds / self.m.opt.timestep)))
        for _ in range(n):
            self.mj.mj_step(self.m, self.d)


def run_crossing(cfg: SimConfig, crawl: CrawlConfig | None = None, player_cfg: PlayerConfig | None = None,
                 model_path: str | None = None, viewer=None, record=None) -> SimResult:
    import mujoco

    rng = np.random.default_rng(cfg.seed)
    crawl = crawl or CrawlConfig()
    _, feet_b = K.default_stance()
    # true geometry: where the box's near face is, ahead of the robot's body centre
    pad = K.FOOT_ENVELOPE_RADIUS + crawl.foot_gap
    near_true = float(feet_b[0, 0]) + pad + cfg.stand_off if cfg.box_height > 0 else 10.0
    if cfg.near is not None:
        near_true = cfg.near
    model = build_model(find_model(model_path), cfg, near_true)
    data = mujoco.MjData(model)
    robot = Robot(model, data)
    robot.reset_standing()
    robot.command(K.DEFAULT_JOINT_POS, cfg.walk_kp if cfg.handover else cfg.kp, cfg.walk_kd if cfg.handover else cfg.kd)
    robot.step(1.0)                                    # settle on the floor at the default pose
    start = robot.position()

    # what perception reports (truth + injected error), relative to where the body really is
    box = BoxSpec(height=max(0.0, cfg.box_height + cfg.err_height), depth=max(0.0, cfg.box_depth + cfg.err_depth),
                  near_x=near_true - float(start[0]) + cfg.err_near,
                  skew=math.radians(cfg.skew_deg + cfg.err_skew_deg))
    from dataclasses import replace
    masses = cfg.extra_masses if cfg.planner_extra_masses is None else cfg.planner_extra_masses
    crawl = replace(crawl, kp=cfg.kp, kd=cfg.kd, extra_masses=tuple(masses))
    plan = plan_crossing(box, crawl)
    if not plan.ok:
        return SimResult(False, plan.reason)

    player = TrajectoryPlayer(plan, replace(player_cfg or PlayerConfig(), kp=cfg.kp, kd=cfg.kd))
    box_geom = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "box_geom")
    box_body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "box") if cfg.box_mass is not None and cfg.box_height > 0 else -1
    box_start = data.xpos[box_body].copy() if box_body >= 0 else None
    foot_geoms = {}
    for i, leg in enumerate(K.LEGS):
        b3 = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, f"leg_{leg}_3")
        for g in range(model.ngeom):
            if model.geom_bodyid[g] == b3 and model.geom_type[g] == mujoco.mjtGeom.mjGEOM_SPHERE:
                foot_geoms[g] = i

    res = SimResult(True, swings=len(plan.steps), plan_margin=min(s.margin for s in plan.steps) if plan.steps else 0.0)
    dt = 1.0 / cfg.control_hz
    t = 0.0
    hand = Handover(player) if cfg.handover else None
    if hand is None:
        player.start(t, robot.joints())
    force = np.zeros(6)
    t_end_hold = None
    height_before = float(robot.position()[2])
    while True:
        q = robot.joints() + (rng.normal(0, cfg.joint_noise, 12) if cfg.joint_noise else 0.0)
        if hand is not None:
            cmd = hand.update(t, q, robot.tilt(), switched=t >= cfg.switch_delay)
            if hand.stage in ("pre", "ramp"):
                res.handover_sag = max(res.handover_sag, height_before - float(robot.position()[2]))
        else:
            cmd = player.update(t, q, robot.tilt())
        robot.command(cmd.q, cmd.kp, cmd.kd)
        robot.step(dt)
        t += dt
        res.max_torque = max(res.max_torque, float(np.abs(data.actuator_force[robot.act]).max()))
        res.tilt_by_phase[cmd.phase] = max(res.tilt_by_phase.get(cmd.phase, 0.0), math.degrees(robot.tilt()))
        res.min_height = min(res.min_height, float(robot.position()[2]))
        if box_geom >= 0:
            for c in range(data.ncon):
                con = data.contact[c]
                other = con.geom2 if con.geom1 == box_geom else (con.geom1 if con.geom2 == box_geom else -1)
                if other < 0 or model.geom_bodyid[other] == 0:
                    continue
                mujoco.mj_contactForce(model, data, c, force)
                fn = float(force[0])
                normal = np.array(con.frame[:3])
                if other in foot_geoms and abs(normal[2]) > 0.8:
                    continue                           # a foot standing on the top
                if other in foot_geoms:
                    res.foot_side_contact_force = max(res.foot_side_contact_force, fn)
                res.unplanned_contact_force = max(res.unplanned_contact_force, fn)
                name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other) or f"foot_{K.LEGS[foot_geoms.get(other, 0)]}" if other in foot_geoms else (mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, other) or f"geom{other}")
                prev = res.contacts.get(name, (0.0, t, cmd.phase))
                res.contacts[name] = (max(prev[0], fn), prev[1], prev[2])
        if record is not None:
            record.append((t, data.qpos.copy()))
        if viewer is not None:
            viewer.sync()
        if player.state == DONE and t_end_hold is None:
            t_end_hold = t + 0.5
        if (t_end_hold is not None and t >= t_end_hold) or player.state == STOPPED or t > plan.duration + 30.0:
            break

    pos = robot.position()
    res.finished = player.state in (DONE, STOPPED)
    res.result = player.result or "timeout"
    res.duration = t
    res.max_tilt_deg = player.max_tilt_deg
    res.final_height = float(pos[2])
    res.final_x = float(pos[0] - start[0])
    res.final_yaw_deg = math.degrees(robot.yaw())
    res.events = list(player.events)
    if box_body >= 0:
        res.box_moved = float(np.linalg.norm((data.xpos[box_body] - box_start)[:2]))
    far_true = near_true + cfg.box_depth / math.cos(math.radians(cfg.skew_deg))
    rear_feet_x = [data.site_xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, f"leg_{leg}_3_foot_site")][0]
                   for leg in ("back_r", "back_l")]
    res.crossed = bool(player.result == "done" and robot.tilt() < math.radians(10)
                       and pos[2] > 0.13 and (cfg.box_height <= 0 or min(rear_feet_x) > far_true + start[0] * 0 ))
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--model")
    ap.add_argument("--height", type=float, default=0.019)
    ap.add_argument("--depth", type=float, default=0.064)
    ap.add_argument("--box-mass", type=float, default=0.6, help="kg; 0 = fixed to the floor")
    ap.add_argument("--stand-off", type=float, default=0.02)
    ap.add_argument("--near", type=float, default=None, help="box's near face this far ahead of the body centre (m); "
                    "0.36 is where the robot stops to measure")
    ap.add_argument("--skew", type=float, default=0.0, help="degrees")
    ap.add_argument("--err-near", type=float, default=0.0)
    ap.add_argument("--err-height", type=float, default=0.0)
    ap.add_argument("--head-mass", type=float, default=0.0, help="kg of extra hardware on the head")
    ap.add_argument("--view", action="store_true", help="open the MuJoCo viewer (needs a display)")
    ap.add_argument("--plan", action="store_true", help="print the planned footsteps")
    args = ap.parse_args()
    extra = ((args.head_mass, [0.055, 0.0, 0.13]),) if args.head_mass > 0 else ()
    cfg = SimConfig(box_height=args.height, box_depth=args.depth, box_mass=args.box_mass or None,
                    stand_off=args.stand_off, near=args.near, skew_deg=args.skew, err_near=args.err_near,
                    err_height=args.err_height, extra_masses=extra)
    if args.view:
        import mujoco
        import mujoco.viewer
        import time
        _, feet_b = K.default_stance()
        near = cfg.near if cfg.near is not None else float(feet_b[0, 0]) + K.FOOT_ENVELOPE_RADIUS + CrawlConfig().foot_gap + cfg.stand_off
        record = []
        res = run_crossing(cfg, model_path=args.model, record=record)
        print(res.line())
        model = build_model(find_model(args.model), cfg, near)
        data = mujoco.MjData(model)
        with mujoco.viewer.launch_passive(model, data) as v:
            while v.is_running():
                for _, qpos in record:
                    data.qpos[:] = qpos
                    mujoco.mj_forward(model, data)
                    v.sync()
                    time.sleep(1.0 / cfg.control_hz)
                    if not v.is_running():
                        break
        return
    res = run_crossing(cfg, model_path=args.model)
    print(res.line())
    for t, e in res.events:
        print(f"  {t:6.2f} s  {e}")


if __name__ == "__main__":
    main()
