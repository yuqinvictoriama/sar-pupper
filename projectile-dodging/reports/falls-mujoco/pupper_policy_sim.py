"""MuJoCo harness: the course default walking policy (gemini_walk) driven by dodge-style
velocity-command profiles, to find which profiles tip Pupper v3 over.

The controller side is a line-by-line port of
pupperv3-monorepo/ros2_ws/src/neural_controller/src/neural_controller.cpp (update()),
include/neural_controller/{command_filter,gait_reference,obs_spec}.hpp, with the
parameters gemini_walk gets (gemini_pupper/robot/tricks.py walk_controller_params on top
of config.yaml's neural_controller_default): model default_policy.json, speed_cap 1.5,
max_vx_forward/backward 1.5, max_body_angle 1.0 rad, game buttons off, no jump slot.

The physics side follows what the policy was trained on (pupper-mjlab
asset_zoo/robots/pupper_v3/pupper_constants.py + tasks/pupper/pupper_env_cfg.py):
4 ms Euler steps, pyramidal cone, impratio 10, 2 solver iterations, 50 Hz control
(decimation 5), kp 5.5 / kd 0.25 / 3 N m, joint frictionloss 0.125, soft contacts
solimp (0.015, 1, 0.031), condim 3, foot priority (the foot's friction is the contact's).
"""
from __future__ import annotations

import json
import math
import os
from dataclasses import dataclass, field, replace
from pathlib import Path

import numpy as np
import mujoco

MONO = Path.home() / "Desktop/CS123/pi/pupperv3-monorepo"
# Byte-identical local copies (md5 checked when copied) so a checkout elsewhere cannot
# change them under a running sweep; the originals are listed for reference.
HERE = Path(__file__).resolve().parent
SRC_POLICY_JSON = MONO / "ros2_ws/src/neural_controller/launch/default_policy.json"   # md5 f43c95fa...
SRC_MODEL_XML = MONO / ("ros2_ws/src/pupper_v3_description/description/mujoco_xml/"
                        "pupper_v3_complete.position.xml")                               # md5 78795f17...
POLICY_JSON = HERE / "model/default_policy.json"
MODEL_XML = HERE / "model/mujoco_xml/pupper_v3_complete.position.xml"
JOINTS = ["leg_front_r_1", "leg_front_r_2", "leg_front_r_3", "leg_front_l_1", "leg_front_l_2",
          "leg_front_l_3", "leg_back_r_1", "leg_back_r_2", "leg_back_r_3", "leg_back_l_1",
          "leg_back_l_2", "leg_back_l_3"]


# --------------------------------------------------------------------------- policy
class Policy:
    """RTNeural dense stack: weights[0] is (in, out) like a Keras kernel, ELU alpha 1."""

    def __init__(self, path=POLICY_JSON):
        d = json.load(open(path))
        self.meta = d
        self.layers = []
        for L in d["layers"]:
            W = np.asarray(L["weights"][0], dtype=np.float32)
            b = np.asarray(L["weights"][1], dtype=np.float32)
            self.layers.append((W, b, L["activation"]))
        self.in_size = int(d["in_shape"][1])
        assert self.layers[0][0].shape[0] == self.in_size

    def __call__(self, x):
        h = np.asarray(x, dtype=np.float32)
        for W, b, act in self.layers:
            h = h @ W + b
            if act == "elu":
                h = np.where(h > 0, h, np.expm1(np.minimum(h, 0)))
            elif act == "tanh":
                h = np.tanh(h)
            elif act in ("", None, "linear"):
                pass
            else:
                raise ValueError(act)
        return h


# --------------------------------------------------------------------------- gait reference
class GaitReference:
    """compute_gait_reference_offset() from gait_reference.hpp."""

    def __init__(self, g, default):
        self.n = int(g["n_samples"])
        self.freq = float(g["frequency"])
        self.blend_speed = float(g["blend_speed"])
        self.gallop_speed = float(g["gallop_speed"])
        self.gallop_mult = float(g["gallop_freq_mult"])
        self.dir_threshold = float(g["dir_threshold"])
        self.trot = np.asarray(g["trot_table"], float)
        self.gallop = np.asarray(g["gallop_table"], float)
        self.lift = np.asarray(g["lift_table"], float) if "lift_table" in g else None
        self.gallop_back = np.asarray(g["gallop_back_table"], float) if "gallop_back_table" in g else None
        self.default = np.asarray(default, float)

    def mode(self, vx, yaw):
        galloping = abs(vx) >= self.gallop_speed
        translating = abs(vx) >= self.dir_threshold
        return ("fast" if galloping else ("trot" if translating or self.lift is None else "lift"))

    def offset(self, t, vx, vy, yaw):
        galloping = abs(vx) >= self.gallop_speed
        freq = self.freq * (self.gallop_mult if galloping else 1.0)
        x = t * freq
        phase = x - math.trunc(x)
        translating = abs(vx) >= self.dir_threshold
        dir_signal = vx if translating else yaw
        eff = phase if dir_signal >= 0.0 else -phase
        pos = eff * self.n
        pf = math.floor(pos)
        i0 = int(pf) % self.n          # Python % is the floor-mod the C++ reimplements
        i1 = (i0 + 1) % self.n
        a = pos - pf
        slow = self.trot if (translating or self.lift is None) else self.lift
        fast = self.gallop if (dir_signal >= 0.0 or self.gallop_back is None) else self.gallop_back
        table = fast if galloping else slow
        speed = math.sqrt(vx * vx + vy * vy + yaw * yaw)
        blend = min(max(speed / self.blend_speed, 0.0), 1.0)
        ref = table[i0] * (1 - a) + table[i1] * a
        return (blend * (ref - self.default)).astype(np.float32)


def wrap_to_pi(a):
    two = 2 * math.pi
    w = math.fmod(a + math.pi, two)
    if w < 0:
        w += two
    if w == 0.0 and a > 0:
        return math.pi
    return w - math.pi


class HeadingHold:
    def __init__(self, hh, enabled=True):
        self.enabled = enabled and hh["kp"] > 0
        self.kp, self.clip = float(hh["kp"]), float(hh["clip"])
        self.yaw_th, self.walk_th = float(hh["yaw_threshold"]), float(hh["walk_threshold"])
        self.target, self.prev = 0.0, False

    def apply(self, heading, vx, vy, raw_yaw):
        if not self.enabled:
            return raw_yaw
        active = abs(raw_yaw) < self.yaw_th and math.hypot(vx, vy) > self.walk_th
        if active and not self.prev:
            self.target = heading
        self.prev = active
        if not active:
            return raw_yaw
        return min(max(self.kp * wrap_to_pi(self.target - heading), -self.clip), self.clip)


# --------------------------------------------------------------------------- sim config
@dataclass
class SimCfg:
    friction: float = 0.8              # foot-floor contact friction (feet have priority)
    act_latency: float = 0.016         # s, policy output -> motor (training: 16 ms fixed)
    imu_lag_p: float = 0.5             # P(IMU obs one control step stale), training 0.5
    obs_noise: bool = True             # training's uniform obs noise (on in play too)
    kp_scale: tuple = (1.0,) * 12      # per-motor strength
    torque_limit: tuple = (3.0,) * 12  # N m per motor
    kd_scale: float = 1.0
    mass_scale: float = 1.0
    extra_mass: float = 0.0            # kg on the head (cameras etc.)
    com_x: float = 0.0
    physics: str = "train"             # "train" = trained solver; "stiff" = 2 ms, converged, firm contacts
    control_hz: float = 50.0
    heading_hold: bool = True
    seed: int = 0


def randomized(seed, base: SimCfg | None = None, fric=(0.4, 0.8), lat=(0.020, 0.040),
               kp=(0.85, 1.05), tau=(2.6, 3.0), mass=(0.97, 1.12)):
    """Per-seed domain randomisation used for the comparison runs."""
    r = np.random.default_rng(10_000 + seed)
    b = base or SimCfg()
    return replace(b, seed=seed, friction=float(r.uniform(*fric)), act_latency=float(r.uniform(*lat)),
                   kp_scale=tuple(r.uniform(*kp, 12)), torque_limit=tuple(r.uniform(*tau, 12)),
                   kd_scale=float(r.uniform(0.9, 1.2)), mass_scale=float(r.uniform(*mass)),
                   com_x=float(r.uniform(-0.01, 0.01)))


def build_model(cfg: SimCfg):
    spec = mujoco.MjSpec.from_file(str(MODEL_XML))
    o = spec.option
    if cfg.physics == "train":
        o.timestep = 0.004
        o.integrator = mujoco.mjtIntegrator.mjINT_EULER
        o.cone = mujoco.mjtCone.mjCONE_PYRAMIDAL
        o.impratio = 10.0
        o.iterations = 2
        o.ls_iterations = 5
        o.disableflags |= mujoco.mjtDisableBit.mjDSBL_EULERDAMP
        solimp = [0.015, 1.0, 0.031]
    elif cfg.physics == "stiff":
        o.timestep = 0.002
        o.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
        o.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
        o.impratio = 10.0
        o.iterations = 50
        o.ls_iterations = 20
        solimp = [0.9, 0.95, 0.001]
    else:
        raise ValueError(cfg.physics)
    for j in spec.joints:
        if j.name in JOINTS:
            j.frictionloss = 0.125
            j.damping = [0.01, 0.0, 0.0] if hasattr(j.damping, "__len__") else 0.01
            j.armature = 0.0016
    for b in spec.bodies:
        if b.name and b.name != "world" and b.mass > 0 and cfg.mass_scale != 1.0:
            b.mass *= cfg.mass_scale
            b.inertia = [v * cfg.mass_scale for v in b.inertia]
    base = spec.body("base_link")
    if cfg.com_x:
        base.ipos = [base.ipos[0] + cfg.com_x, base.ipos[1], base.ipos[2]]
    if cfg.extra_mass > 0:
        eb = base.add_body(name="extra_mass", pos=[0.055, 0.0, 0.10])
        eb.mass = cfg.extra_mass
        eb.inertia = [1e-5] * 3
        eb.explicitinertial = True
    for g in spec.geoms:
        if g.conaffinity == 1 and g.contype == 0:  # robot collision geoms
            g.condim = 3
            g.solimp = solimp + list(g.solimp[3:])
            g.friction = [0.8, 0.02, 0.01]
            if g.type == mujoco.mjtGeom.mjGEOM_SPHERE:     # the four feet
                g.priority = 1
                g.friction = [cfg.friction, 0.02, 0.01]
        elif g.name == "floor":
            g.friction = [cfg.friction, 0.02, 0.01]
    model = spec.compile()
    return model


# --------------------------------------------------------------------------- the run
@dataclass
class Trace:
    t: list = field(default_factory=list)
    tilt: list = field(default_factory=list)
    gyro: list = field(default_factory=list)
    pos: list = field(default_factory=list)
    yaw: list = field(default_factory=list)
    cmd: list = field(default_factory=list)     # filtered (vx, vy, wz) the policy saw
    raw: list = field(default_factory=list)
    mode: list = field(default_factory=list)
    height: list = field(default_factory=list)
    estop_t: float | None = None


class PupperSim:
    def __init__(self, cfg: SimCfg, policy: Policy | None = None):
        self.cfg = cfg
        self.pol = policy or Policy()
        meta = self.pol.meta
        self.default = np.asarray(meta["default_joint_pos"], float)
        self.lo = np.asarray(meta["joint_lower_limits"], float)
        self.hi = np.asarray(meta["joint_upper_limits"], float)
        self.action_scale = float(meta["action_scale"])
        self.kp = float(meta["kp"])
        self.kd = float(meta["kd"])
        self.H = int(meta["observation_history"])
        comps = meta["obs_spec"]["components"]
        self.off = {}
        o = 0
        for c in comps:
            self.off[c["name"]] = o
            o += int(c["size"])
        self.F = o
        assert self.F == meta["single_observation_size"] and self.F * self.H == self.pol.in_size
        clip = meta["command_clip"]
        self.clip = (clip["vx"], clip["vy"], clip["wz"])
        self.speed_cap = 1.5           # gemini_walk speed_cap (vx and vy)
        self.max_vx_fwd = 1.5          # max_vx_forward / backward
        self.max_vx_back = 1.5
        self.max_body_angle = 1.0      # rad, neural_controller_default
        self.gait = GaitReference(meta["gait_reference"], self.default)
        self.hold = HeadingHold(meta["heading_hold"], cfg.heading_hold)
        self.rng = np.random.default_rng(cfg.seed)

        self.m = build_model(cfg)
        self.d = mujoco.MjData(self.m)
        m = self.m
        self.qadr = np.array([m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINTS])
        self.act = np.array([mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in JOINTS])
        self.base = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "base_link")
        gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "body_gyro")
        self.gyro_adr = m.sensor_adr[gid]
        qid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SENSOR, "body_quat")
        self.quat_adr = m.sensor_adr[qid]
        self.kp_j = self.kp * np.asarray(cfg.kp_scale)
        self.kd_j = self.kd * cfg.kd_scale * np.ones(12)
        self.tau = np.asarray(cfg.torque_limit)
        m.actuator_forcerange[self.act, 0] = -self.tau
        m.actuator_forcerange[self.act, 1] = self.tau
        self._set_gains(self.kp_j, self.kd_j)
        self.dt = m.opt.timestep
        self.decim = max(1, int(round(1.0 / cfg.control_hz / self.dt)))
        self.lat_steps = int(round(cfg.act_latency / self.dt))
        self.reset()

    def _set_gains(self, kp, kd):
        self.m.actuator_gainprm[self.act, 0] = kp
        self.m.actuator_biasprm[self.act, 1] = -kp
        self.m.actuator_biasprm[self.act, 2] = -kd

    def reset(self, height=0.16):
        m, d = self.m, self.d
        mujoco.mj_resetData(m, d)
        d.qpos[0:3] = [0, 0, height]
        d.qpos[3:7] = [1, 0, 0, 0]
        d.qpos[self.qadr] = self.default
        d.ctrl[self.act] = self.default
        mujoco.mj_forward(m, d)
        self.obs = np.zeros(self.H * self.F, np.float32)
        for i in range(self.H):
            self.obs[i * self.F + self.off["projected_gravity"] + 2] = -1.0
        self.pending = []            # (physics step due, target)
        self.k = 0                   # physics step counter
        self.t_policy = 0.0          # time since fade-in (phase clock)
        self.prev_imu = None
        self.estop = False
        self.hold.target, self.hold.prev = 0.0, False
        self.last_cmd = (0.0, 0.0, 0.0)
        self.last_mode = "trot"

    # ---- measurements
    def quat(self):
        return self.d.sensordata[self.quat_adr:self.quat_adr + 4].copy()   # w x y z

    def gyro(self):
        return self.d.sensordata[self.gyro_adr:self.gyro_adr + 3].copy()

    def tilt(self):
        R = self.d.xmat[self.base].reshape(3, 3)
        return math.acos(max(-1.0, min(1.0, R[2, 2])))

    def yaw(self):
        w, x, y, z = self.quat()
        return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))

    # ---- one policy step of NeuralController::update()
    def policy_step(self, raw_vx, raw_vy, raw_wz=0.0):
        cfg = self.cfg
        if self.estop:
            return
        vx = min(max(raw_vx, self.clip[0][0]), self.clip[0][1])
        vy = min(max(raw_vy, self.clip[1][0]), self.clip[1][1])
        wz = min(max(raw_wz, self.clip[2][0]), self.clip[2][1])
        vx = min(vx, self.max_vx_fwd)
        vx = max(vx, -self.max_vx_back)
        cap = self.speed_cap
        vx = min(max(vx, -cap), cap)
        vy = min(max(vy, -cap), cap)

        q = self.quat()
        w, x, y, z = q
        R = np.empty(9)
        mujoco.mju_quat2Mat(R, q)
        R = R.reshape(3, 3)
        grav = R.T @ np.array([0.0, 0.0, -1.0])
        if -grav[2] < math.cos(self.max_body_angle):
            self.trigger_estop()
            return
        wz = self.hold.apply(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)), vx, vy, wz)
        ang = self.gyro()

        # what the policy gets to see (training corruption: noise, then IMU lag)
        ang_o, grav_o = ang.copy(), grav.copy()
        jp = self.d.qpos[self.qadr] - self.default
        if cfg.obs_noise:
            ang_o = ang_o + self.rng.uniform(-0.1, 0.1, 3)
            grav_o = grav_o + self.rng.uniform(-0.05, 0.05, 3)
            grav_o /= max(np.linalg.norm(grav_o), 1e-6)
            jp = jp + self.rng.uniform(-0.05, 0.05, 12)
        imu_now = np.concatenate([ang_o, grav_o])
        imu = imu_now
        if self.prev_imu is not None and self.rng.random() < cfg.imu_lag_p:
            imu = self.prev_imu
        self.prev_imu = imu_now

        o, f = self.obs, self.off
        o[f["base_ang_vel"]:f["base_ang_vel"] + 3] = imu[:3]
        o[f["projected_gravity"]:f["projected_gravity"] + 3] = imu[3:]
        o[f["command"]:f["command"] + 3] = (vx, vy, wz)
        o[f["desired_world_z"]:f["desired_world_z"] + 3] = (0.0, 0.0, 1.0)
        o[f["joint_pos_rel"]:f["joint_pos_rel"] + 12] = jp
        o[f["gait_reference"]:f["gait_reference"] + 12] = self.gait.offset(self.t_policy, vx, vy, wz)
        np.clip(o, -100.0, 100.0, out=o)
        a = self.pol(o)
        # rotate history right by one frame (newest_first)
        self.obs = np.roll(o, self.F)
        la = a.copy()
        if cfg.obs_noise:
            la = la + self.rng.uniform(-0.01, 0.01, 12).astype(np.float32)
        self.obs[f["last_action"]:f["last_action"] + 12] = la
        target = np.clip(a * self.action_scale + self.default, self.lo, self.hi)
        self.pending.append((self.k + self.lat_steps, target))
        self.last_cmd = (vx, vy, wz)
        self.last_mode = self.gait.mode(vx, wz)

    def trigger_estop(self):
        """Emergency stop as the controller does it: every command 0, kd = estop_kd 0.1."""
        self.estop = True
        self.pending.clear()
        self._set_gains(np.zeros(12), 0.1 * np.ones(12))
        self.d.ctrl[self.act] = 0.0

    def physics(self, n):
        for _ in range(n):
            while self.pending and self.pending[0][0] <= self.k:
                _, tgt = self.pending.pop(0)
                if not self.estop:
                    self.d.ctrl[self.act] = tgt
            mujoco.mj_step(self.m, self.d)
            self.k += 1

    def run(self, cmd_fn, duration, t0=0.0, trace: Trace | None = None, sub=1):
        """Run `duration` s. cmd_fn(t) -> (vx, vy) raw command at sim time t (s from t0)."""
        n_ctrl = int(round(duration * self.cfg.control_hz))
        ctrl_dt = self.decim * self.dt
        for i in range(n_ctrl):
            t = t0 + i * ctrl_dt
            vx, vy = cmd_fn(t)
            self.policy_step(vx, vy, 0.0)
            if trace is not None:
                trace.cmd.append(self.last_cmd)
                trace.raw.append((vx, vy))
                trace.mode.append(self.last_mode)
                if self.estop and trace.estop_t is None:
                    trace.estop_t = t
            for s in range(sub):
                self.physics(self.decim // sub)
                if trace is not None:
                    trace.t.append(t + (s + 1) * ctrl_dt / sub)
                    trace.tilt.append(self.tilt())
                    trace.gyro.append(self.gyro())
                    trace.pos.append(self.d.xpos[self.base].copy())
                    trace.yaw.append(self.yaw())
                    trace.height.append(float(self.d.xpos[self.base][2]))
            self.t_policy += ctrl_dt
        return trace


if __name__ == "__main__":
    p = Policy()
    s = PupperSim(SimCfg())
    print("policy", POLICY_JSON.name, "in", p.in_size, "frame", s.F, s.off)
    print("control rate", s.cfg.control_hz, "Hz; physics dt", s.dt, "decimation", s.decim,
          "latency steps", s.lat_steps)
