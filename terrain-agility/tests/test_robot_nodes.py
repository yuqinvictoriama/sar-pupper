"""The ROS nodes, run off the robot: the real robot/stepover_node.py code drives the physics
simulator through a fake ROS layer (tests/fake_ros.py) in simulated time. This covers what
the pure-Python tests cannot: taking the legs from the walking controller, streaming,
giving them back, the e-stop, stale sensor data and the refusals."""
import json
import math
import sys
import time
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))
mujoco = pytest.importorskip("mujoco")

import fake_ros  # noqa: E402
from sim import crossing_sim as CS  # noqa: E402
from terrain import kinematics as K  # noqa: E402

try:
    MODEL_XML = CS.find_model()
except FileNotFoundError:
    MODEL_XML = None
pytestmark = pytest.mark.skipif(MODEL_XML is None, reason="Pupper MuJoCo model not available")

STATUS = "/terrain/crossing_status"
REQUEST = "/terrain/step_over"


class Rig:
    """Simulated robot + controller manager + the step-over node under test."""

    def __init__(self, box=None, near=0.36, switch_time=0.03, width=0.50, mass=0.6):
        self.bus = fake_ros.install()
        for name in [m for m in sys.modules if m in ("stepover_node", "estop_guard", "terrain_node")]:
            del sys.modules[name]
        sys.path.insert(0, str(ROOT / "robot"))
        import stepover_node
        from terrain.config import TerrainConfig

        h, d = box or (0.0, 0.0)
        self.sim_cfg = CS.SimConfig(box_height=h, box_depth=d, near=near, box_width=width, box_mass=mass)
        self.model = CS.build_model(MODEL_XML, self.sim_cfg, near if h > 0 else 10.0)
        self.data = mujoco.MjData(self.model)
        self.robot = CS.Robot(self.model, self.data)
        self.robot.reset_standing()
        self.robot.command(K.DEFAULT_JOINT_POS, 5.5, 0.25)
        self.robot.step(1.0)
        self.t = 100.0
        self.cmd = {}                        # interface -> latest array since the terrain controllers went active
        self.publish_joints = True
        self.at = []                         # (time, callable) to run once
        self.min_height = 1.0
        self.switch_time = switch_time
        self.cm = fake_ros.ControllerManager(wait=self._switch_wait)
        self.cm.on_switch = lambda act, deact: self.cmd.clear() if any(n in self.cm.terrain for n in act) else None
        for iface in ("position", "kp", "kd"):
            self.bus.subs.setdefault(f"/terrain_{iface}_controller/commands", []).append(
                lambda m, iface=iface: self._on_cmd(iface, m))
        self.node = stepover_node.StepOverNode(TerrainConfig())
        self.node.now = lambda: self.t
        self.node.sleep = self.advance
        self.order = list(np.random.default_rng(1).permutation(12))     # joint_states in a scrambled order
        self.advance(0.2)

    def _on_cmd(self, iface, msg):
        if self.cm.active(self.cm.terrain):                              # dropped unless the controllers are active
            self.cmd[iface] = np.array(msg.data, dtype=float)

    def _switch_wait(self):
        t0, real0 = self.t, time.monotonic()
        while self.t - t0 < self.switch_time and time.monotonic() - real0 < 0.3:
            time.sleep(0.0005)

    def advance(self, dt):
        for item in list(self.at):
            if self.t >= item[0]:
                self.at.remove(item)
                item[1]()
        if self.cm.active(self.cm.terrain):
            q = self.cmd.get("position", np.zeros(12))                   # nothing sent yet: interfaces were zeroed
            kp = float(self.cmd["kp"][0]) if "kp" in self.cmd else 0.0
            kd = float(self.cmd["kd"][0]) if "kd" in self.cmd else 0.1
        elif self.cm.active(self.cm.walking):
            q, kp, kd = K.DEFAULT_JOINT_POS, 7.5, 0.25
        else:
            q, kp, kd = self.robot.joints(), 0.0, 0.1
        self.robot.command(q, kp, kd)
        self.robot.step(dt)
        self.t += dt
        self.min_height = min(self.min_height, float(self.robot.position()[2]))
        if self.publish_joints:
            names = [K.JOINT_NAMES[i] for i in self.order]
            qpos = self.robot.joints()
            qvel = np.array([self.data.qvel[self.model.jnt_dofadr[mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n)]]
                             for n in K.JOINT_NAMES])
            self.bus.publish("/joint_states", fake_ros.JointState(name=names, position=[float(qpos[i]) for i in self.order],
                                                                  velocity=[float(qvel[i]) for i in self.order]))
        w, x, y, z = self.data.xquat[self.robot.base]
        self.bus.publish("/imu_sensor_broadcaster/imu", fake_ros.Imu(orientation=fake_ros.Quaternion(x=x, y=y, z=z, w=w)))

    def request(self, **req):
        self.bus.sent.pop(STATUS, None)
        self.bus.publish(REQUEST, fake_ros.String(data=json.dumps(req)))
        if self.node.job is not None:
            self.node.job.join(timeout=600)
            assert not self.node.job.is_alive()
        return [json.loads(m.data) for m in self.bus.sent.get(STATUS, [])]

    def states(self, statuses):
        return [s["state"] for s in statuses if s["state"] != "progress"]


def test_stand_test_takes_the_legs_and_gives_them_back():
    rig = Rig()
    h0 = float(rig.robot.position()[2])
    st = rig.request(cmd="stand", hold=1.0, id="a")
    assert rig.states(st) == ["planning", "started", "done"], st
    # legs handed over and back, exactly once each way
    assert rig.cm.history == [(tuple(rig.cm.terrain), ("neural_controller_default",)),
                              (("neural_controller_default",), tuple(rig.cm.terrain))]
    assert rig.cm.active(rig.cm.walking) and not rig.cm.active(rig.cm.terrain)
    # the body never dropped more than a few millimetres while the legs changed hands
    assert rig.min_height > h0 - 0.006, (rig.min_height, h0)
    assert abs(float(rig.robot.position()[2]) - h0) < 0.006
    beats = [m.data for m in rig.bus.sent["/terrain/stepover_heartbeat"]]
    assert beats[0] == "holding" and beats[-1] == "released"


def test_commands_stay_inside_hardware_limits_and_gains():
    rig = Rig()
    rig.request(cmd="crawl", distance=0.08, id="b")
    lim = np.concatenate([K.leg_limits(leg) for leg in K.LEGS], axis=0)
    for m in rig.bus.sent["/terrain_position_controller/commands"]:
        q = np.array(m.data)
        assert np.all(q >= lim[:, 0] - 1e-9) and np.all(q <= lim[:, 1] + 1e-9)
    assert max(max(m.data) for m in rig.bus.sent["/terrain_kp_controller/commands"]) <= 7.5 + 1e-9
    assert max(max(m.data) for m in rig.bus.sent["/terrain_kd_controller/commands"]) <= 0.8 + 1e-9


def test_crawl_moves_forward_and_stays_level():
    rig = Rig()
    x0 = float(rig.robot.position()[0])
    st = rig.request(cmd="crawl", distance=0.08, id="c")
    assert rig.states(st)[-1] == "done", st
    assert 0.05 < float(rig.robot.position()[0]) - x0 < 0.11
    assert st[-1]["max_tilt_deg"] < 6.0


def test_refuses_when_not_standing_or_no_walking_controller():
    rig = Rig()
    rig.cm.state["neural_controller_default"] = "inactive"
    st = rig.request(cmd="stand", id="d")
    assert rig.states(st) == ["refused"] and "walking controller" in st[0]["why"]
    assert rig.cm.history == []
    rig2 = Rig()
    rig2.robot.command(K.DEFAULT_JOINT_POS + 0.5, 7.5, 0.25)             # legs somewhere else entirely
    rig2.cm.state["neural_controller_default"] = "inactive"              # (hold that pose in the rig)
    rig2.cm.state["neural_controller_default"] = "active"
    rig2.node.q = K.DEFAULT_JOINT_POS + 0.5
    rig2.publish_joints = False
    rig2.node.q_time = rig2.t
    st = rig2.request(cmd="stand", id="e")
    assert rig2.states(st) == ["refused"] and "standing pose" in st[0]["why"]


def test_dry_run_and_too_tall_box_move_nothing():
    rig = Rig()
    st = rig.request(cmd="cross", height=0.060, depth=0.064, near_x=0.36, id="f")
    assert rig.states(st) == ["planning", "no_plan"] and "limit" in st[-1]["why"]
    st = rig.request(cmd="stand", dry_run=True, id="g")
    assert rig.states(st) == ["planning", "done"] and st[-1]["dry_run"]
    assert rig.cm.history == [] and "/terrain_kp_controller/commands" not in rig.bus.sent


def test_estop_mid_motion_goes_limp_and_releases():
    rig = Rig()
    rig.at.append((rig.t + 2.0, lambda: rig.bus.publish("/emergency_stop", fake_ros.Empty())))
    st = rig.request(cmd="stand", hold=6.0, id="h")
    assert rig.states(st)[-1] == "failed" and "e-stop" in st[-1]["why"], st
    assert not rig.cm.active(rig.cm.terrain) and not rig.cm.active(rig.cm.walking)     # nobody re-activates anything
    assert rig.bus.sent["/terrain_kp_controller/commands"][-1].data == [0.0] * 12
    # and a new motion is refused straight after an e-stop
    st = rig.request(cmd="stand", id="i")
    assert rig.states(st) == ["refused"]


def test_stale_joint_states_go_limp():
    rig = Rig()
    rig.at.append((rig.t + 2.0, lambda: setattr(rig, "publish_joints", False)))
    st = rig.request(cmd="stand", hold=6.0, id="j")
    assert rig.states(st)[-1] == "failed" and "stopped arriving" in st[-1]["why"], st
    assert not rig.cm.active(rig.cm.terrain)


def test_failed_switch_never_stiffens_the_legs():
    rig = Rig()
    rig.cm.fail_next = True
    st = rig.request(cmd="stand", id="k")
    assert rig.states(st) == ["planning", "refused"] and "switch" in st[-1]["why"], st
    assert rig.cm.active(rig.cm.walking)                                 # the walking controller still has the legs
    assert max(max(m.data) for m in rig.bus.sent["/terrain_kp_controller/commands"]) == 0.0


def test_full_crossing_through_the_node():
    """The whole chain: request -> plan from the gauge distance (counted approach + stored
    footstep pattern) -> hand-over -> crossing in physics -> hand-back."""
    from terrain.crawl_planner import load_templates
    if not load_templates():
        pytest.skip("config/crawl_templates.json not built")
    rig = Rig(box=(0.019, 0.089), near=0.36)
    box_body = mujoco.mj_name2id(rig.model, mujoco.mjtObj.mjOBJ_BODY, "box")
    box0 = rig.data.xpos[box_body].copy()
    st = rig.request(cmd="cross", height=0.019, depth=0.089, near_x=0.36, id="x")
    assert rig.states(st) == ["planning", "started", "done"], st[-1]
    pos = rig.robot.position()
    rear_x = min(rig.data.site_xpos[mujoco.mj_name2id(rig.model, mujoco.mjtObj.mjOBJ_SITE, f"leg_{leg}_3_foot_site")][0]
                 for leg in ("back_r", "back_l"))
    assert rear_x > 0.36 + 0.089, "rear feet are beyond the box"
    assert st[-1]["max_tilt_deg"] < 8.0
    assert float(np.linalg.norm((rig.data.xpos[box_body] - box0)[:2])) < 0.004, "the box was not pushed"
    assert pos[2] > 0.14 and rig.cm.active(rig.cm.walking)


def test_full_scale_across_through_the_node():
    """The whole chain for the slab: request -> footsteps planned on the measured slab ->
    hand-over -> onto it, along it and off it in physics -> hand-back."""
    rig = Rig(box=(0.015, 0.38), near=0.36, width=0.565, mass=2.0)
    box_body = mujoco.mj_name2id(rig.model, mujoco.mjtObj.mjOBJ_BODY, "box")
    box0 = rig.data.xpos[box_body].copy()
    x0 = float(rig.robot.position()[0])
    st = rig.request(cmd="cross", height=0.015, depth=0.38, near_x=0.36, id="s")      # the depth makes it a slab
    assert rig.states(st) == ["planning", "started", "done"], st[-1]
    started = next(x for x in st if x["state"] == "started")
    assert "onto the box top" in started["plan"] and 0.80 < started["advance"] < 0.95
    pos = rig.robot.position()
    rear_x = min(rig.data.site_xpos[mujoco.mj_name2id(rig.model, mujoco.mjtObj.mjOBJ_SITE, f"leg_{leg}_3_foot_site")][0]
                 for leg in ("back_r", "back_l"))
    assert rear_x > 0.36 + 0.38 + 0.02, "rear feet are beyond the slab"
    assert st[-1]["max_tilt_deg"] < 8.0
    assert float(np.linalg.norm((rig.data.xpos[box_body] - box0)[:2])) < 0.004, "the slab was not pushed"
    assert pos[2] > 0.14 and rig.cm.active(rig.cm.walking)
    # the distance it reports is the distance it went (the mission layer counts on it)
    assert abs((float(pos[0]) - x0) - st[-1]["advance"]) < 0.03, (pos[0] - x0, st[-1]["advance"])
    # asking for the bar's stored patterns on a slab is refused, not attempted
    st = rig.request(cmd="cross", height=0.015, depth=0.38, near_x=0.36, method="board", dry_run=True, id="b")
    assert rig.states(st) == ["planning", "no_plan"]

