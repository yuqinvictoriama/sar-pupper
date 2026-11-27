"""robot/terrain_node.py off the robot: real node code, fake ROS, synthetic fisheye frames
sent through JPEG the way the camera node delivers them."""
import json
import math
import sys
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "tests"))
cv2 = pytest.importorskip("cv2")

import fake_ros  # noqa: E402
from sim.render_fisheye import BEIGE, BROWN, WHITE, Cuboid, Cylinder, Scene, render  # noqa: E402
from terrain.camera_model import CameraPose, DoubleSphere  # noqa: E402

MODEL = DoubleSphere()
POSE = CameraPose()          # the defaults in terrain.yaml


def jpeg(scene):
    rgb = render(scene, MODEL, POSE)
    ok, buf = cv2.imencode(".jpg", cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 90])
    assert ok
    return buf.tobytes()


class Rig:
    def __init__(self, tmp_path):
        self.bus = fake_ros.install()
        sys.modules.pop("terrain_node", None)
        sys.path.insert(0, str(ROOT / "robot"))
        import terrain_node
        from terrain.config import TerrainConfig

        cfg_path = tmp_path / "terrain.yaml"
        cfg_path.write_text((ROOT / "config" / "terrain.yaml").read_text())     # no floor model next to it
        self.t = 50.0
        terrain_node.TerrainNode.now = lambda *_: self.t
        self.node = terrain_node.TerrainNode(TerrainConfig(cfg_path))
        self.p = self.node.perception
        self.p.running = False                     # frames are pushed through by hand, synchronously
        self.tick = self.node.timers[0][1]
        self.bus.publish("/imu_sensor_broadcaster/imu", fake_ros.Imu())

    def frame(self, data, gauge=False):
        (self.p._gauge if gauge else self.p._scan)(data, np.eye(3), self.t)

    def run(self, seconds, data=None, drive=0.3, gauge_frames=False):
        """Tick at 20 Hz; a new frame every 0.1 s; the test driver commanding ``drive`` m/s."""
        n = int(round(seconds * 20))
        for k in range(n):
            self.t += 0.05
            self.bus.publish("/imu_sensor_broadcaster/imu", fake_ros.Imu())
            if drive:
                self.bus.publish("/terrain_test_cmd_vel", fake_ros.Twist(linear=fake_ros.Vector3(x=drive)))
            if data is not None and k % 2 == 0:
                self.frame(data, gauge=self.p.want_gauge)
            self.tick()

    def sent(self, topic):
        return self.bus.sent.get(topic, [])

    def state(self):
        return json.loads(self.sent("/terrain/state")[-1].data)["state"]


def test_silent_on_clear_floor(tmp_path):
    rig = Rig(tmp_path)
    rig.run(1.5, jpeg(Scene(seed=1)))
    assert rig.state() == "CRUISE"
    assert not rig.sent("/terrain_cmd_vel"), "must not publish while nothing is in the way (the mux would give it control)"
    assert json.loads(rig.sent("/terrain/obstacles")[-1].data) == []


def test_tall_obstacle_takes_over_and_steers(tmp_path):
    rig = Rig(tmp_path)
    rig.run(0.5, jpeg(Scene(seed=1)))                                          # learns the floor on clear ground
    rig.run(1.0, jpeg(Scene(boxes=[Cuboid(0.75, 0.0, 0.3, 0.35, 0.30, side=BROWN, top=BROWN)], seed=2)))
    assert rig.state() == "AVOID"
    cmds = rig.sent("/terrain_cmd_vel")
    assert cmds and any(abs(c.angular.z) > 0.2 for c in cmds), "it turns away from the obstacle"
    assert any("going around" in m.data for m in rig.sent("/terrain/event"))
    # the driver stops: terrain agility lets go again
    blocked = jpeg(Scene(boxes=[Cuboid(0.75, 0.0, 0.3, 0.35, 0.30, side=BROWN, top=BROWN)], seed=2))
    n = len(cmds)
    rig.run(1.2, blocked, drive=0.0)
    assert rig.state() == "CRUISE"
    assert len(rig.sent("/terrain_cmd_vel")) <= n + 22, "it lets go within about a second of the driver going quiet"
    n = len(rig.sent("/terrain_cmd_vel"))
    rig.run(1.0, blocked, drive=0.0)
    assert len(rig.sent("/terrain_cmd_vel")) == n, "and stays silent"


def test_box_is_approached_gauged_and_handed_to_the_step_over_node(tmp_path):
    rig = Rig(tmp_path)
    rig.run(0.5, jpeg(Scene(seed=1)))
    box = lambda dist: jpeg(Scene(boxes=[Cuboid(dist, 0.0, 0.089, 0.6, 0.019)], seed=3))   # noqa: E731
    rig.run(0.6, box(0.70))
    assert rig.state() == "APPROACH"
    assert rig.sent("/terrain_cmd_vel")[-1].linear.x == pytest.approx(rig.node.cfg.fsm.approach_speed)
    rig.run(4.0, box(0.37))                                                    # arrived: it stops, settles and measures
    reqs = [json.loads(m.data) for m in rig.sent("/terrain/step_over")]
    assert len(reqs) == 1, [m.data for m in rig.sent("/terrain/event")]
    req = reqs[0]
    assert req["cmd"] == "cross"
    assert abs(req["height"] - 0.019) < 0.004
    assert abs(req["depth"] - 0.089) < 0.012 and abs(req["near_x"] - 0.37) < 0.012 and abs(req["skew"]) < 0.03
    assert rig.state() == "PLAN" and rig.sent("/terrain_cmd_vel")[-1].linear.x == 0.0     # stands still while planning
    # the step-over node's answers move it on
    status = rig.node.cfg.stepover["status_topic"]
    rig.bus.publish(status, fake_ros.String(data=json.dumps({"state": "started", "request": req["id"], "plan": "..."})))
    n = len(rig.sent("/terrain_cmd_vel"))
    rig.run(1.0, None)
    assert rig.state() == "STEP_OVER" and len(rig.sent("/terrain_cmd_vel")) == n          # silent during the crossing
    rig.bus.publish(status, fake_ros.String(data=json.dumps({"state": "done", "request": req["id"]})))
    rig.run(0.2, None)
    assert rig.state() == "CRUISE"


def test_refused_crossing_falls_back_to_going_around(tmp_path):
    rig = Rig(tmp_path)
    rig.run(0.5, jpeg(Scene(seed=1)))
    frame = jpeg(Scene(boxes=[Cuboid(0.37, 0.0, 0.089, 0.6, 0.019)], seed=3))
    rig.run(4.0, frame)
    req = json.loads(rig.sent("/terrain/step_over")[-1].data)
    rig.bus.publish(rig.node.cfg.stepover["status_topic"],
                    fake_ros.String(data=json.dumps({"state": "no_plan", "request": req["id"], "why": "test"})))
    rig.run(0.5, frame)
    assert rig.state() == "AVOID"


def test_tall_two_colour_box_is_not_stepped_over(tmp_path):
    rig = Rig(tmp_path)
    rig.run(0.5, jpeg(Scene(seed=1)))
    rig.run(4.0, jpeg(Scene(boxes=[Cuboid(0.37, 0.0, 0.089, 0.6, 0.050)], seed=4)))
    assert not rig.sent("/terrain/step_over")
    assert any("go_around" in m.data and "limit" in m.data for m in rig.sent("/terrain/event")), \
        [m.data for m in rig.sent("/terrain/event")]


def test_disable_makes_it_silent(tmp_path):
    rig = Rig(tmp_path)
    rig.run(0.5, jpeg(Scene(seed=1)))
    rig.bus.publish("/terrain/enable", fake_ros.Bool(data=False))
    rig.run(1.0, jpeg(Scene(boxes=[Cuboid(0.75, 0.0, 0.3, 0.35, 0.30, side=BROWN, top=BROWN)], seed=2)))
    assert rig.state() == "DISABLED" and not rig.sent("/terrain_cmd_vel")


def test_flat_two_colour_patch_is_walked_over_without_stopping_again(tmp_path):
    """A two-colour strip only a few millimetres high (a threshold in the box colours) is
    ignored: the node hands control back and does not approach it a second time."""
    rig = Rig(tmp_path)
    rig.run(0.5, jpeg(Scene(seed=1)))
    flat = jpeg(Scene(boxes=[Cuboid(0.37, 0.0, 0.089, 0.6, 0.006)], seed=5))
    for _ in range(40):
        rig.run(0.1, flat)
        events = [m.data for m in rig.sent("/terrain/event")]
        if any("decision: ignore" in e for e in events):
            break
    assert any("decision: ignore" in e for e in events), events
    assert not rig.sent("/terrain/step_over")
    n_events, n_cmd = len(events), len(rig.sent("/terrain_cmd_vel"))
    rig.run(2.0, flat)                 # (in the test the picture does not change; on the robot it walks on)
    assert rig.state() == "CRUISE" and len(rig.sent("/terrain_cmd_vel")) == n_cmd
    assert len(rig.sent("/terrain/event")) == n_events


# ---------------------------------------------------------------- the three course obstacles
def slab(dist):
    return jpeg(Scene(boxes=[Cuboid(dist, 0.0, 0.38, 0.565, 0.015, side=BEIGE, top=WHITE)], seed=3))


def test_slab_is_approached_gauged_and_scaled_across(tmp_path):
    rig = Rig(tmp_path)
    rig.run(0.5, jpeg(Scene(seed=1)))
    rig.run(0.6, slab(0.75))
    assert rig.state() == "APPROACH", [m.data for m in rig.sent("/terrain/event")]
    rig.run(4.0, slab(0.37))                                                   # arrived: it stops, settles and measures
    reqs = [json.loads(m.data) for m in rig.sent("/terrain/step_over")]
    assert len(reqs) == 1, [m.data for m in rig.sent("/terrain/event")]
    req = reqs[0]
    assert req["cmd"] == "cross" and req["method"] == "slab"
    assert abs(req["height"] - 0.015) < 0.004 and abs(req["depth"] - 0.38) < 0.015
    assert abs(req["near_x"] - 0.37) < 0.012 and abs(req["skew"]) < 0.03
    assert any("scale across" in m.data for m in rig.sent("/terrain/event"))
    status = rig.node.cfg.stepover["status_topic"]
    rig.bus.publish(status, fake_ros.String(data=json.dumps({"state": "started", "request": req["id"], "plan": "..."})))
    n = len(rig.sent("/terrain_cmd_vel"))
    rig.run(1.0, None)
    st = json.loads(rig.sent("/terrain/state")[-1].data)
    assert st["state"] == "SCALE_ACROSS" and st["kind"] == "scale_across" and abs(st["box"]["depth"] - 0.38) < 0.015
    assert len(rig.sent("/terrain_cmd_vel")) == n, "silent while the crawl has the legs"
    rig.bus.publish(status, fake_ros.String(data=json.dumps({"state": "done", "request": req["id"], "advance": 0.86})))
    rig.run(0.2, None)
    assert rig.state() == "CRUISE"


def test_low_bar_then_slab_are_crossed_one_after_the_other(tmp_path):
    """The demo's first stretch: a low bar with the slab 0.7 m beyond it, both in view from
    the start. The bar is measured and stepped over; then, with nothing restarted, the slab
    is measured and scaled across."""
    rig = Rig(tmp_path)
    rig.run(0.5, jpeg(Scene(seed=1)))
    both = lambda dist: jpeg(Scene(boxes=[Cuboid(dist, 0.0, 0.089, 0.6, 0.019, side=BEIGE, top=WHITE),       # noqa: E731
                                          Cuboid(dist + 0.70, 0.0, 0.38, 0.565, 0.015, side=BEIGE, top=WHITE)], seed=3))
    status = rig.node.cfg.stepover["status_topic"]
    rig.run(0.6, both(0.70))
    assert rig.state() == "APPROACH", [m.data for m in rig.sent("/terrain/event")]
    rig.run(4.0, both(0.37))
    reqs = [json.loads(m.data) for m in rig.sent("/terrain/step_over")]
    assert len(reqs) == 1, [m.data for m in rig.sent("/terrain/event")]
    bar = reqs[0]
    assert bar["method"] == "board" and abs(bar["height"] - 0.019) < 0.004 and abs(bar["depth"] - 0.089) < 0.012
    assert abs(bar["near_x"] - 0.37) < 0.012
    rig.bus.publish(status, fake_ros.String(data=json.dumps({"state": "started", "request": bar["id"], "plan": "..."})))
    rig.run(1.0, None)
    st = json.loads(rig.sent("/terrain/state")[-1].data)
    assert st["state"] == "STEP_OVER" and st["kind"] == "step_over"
    rig.bus.publish(status, fake_ros.String(data=json.dumps({"state": "done", "request": bar["id"], "advance": 0.58})))
    rig.run(0.2, None)
    assert rig.state() == "CRUISE"
    # the bar is behind; the slab is now 0.49 m ahead (0.37 + 0.70 - 0.58): approached, measured, crossed
    rig.run(0.6, slab(0.49))
    assert rig.state() == "APPROACH", [m.data for m in rig.sent("/terrain/event")]
    rig.run(4.0, slab(0.37))
    reqs = [json.loads(m.data) for m in rig.sent("/terrain/step_over")]
    assert len(reqs) == 2, [m.data for m in rig.sent("/terrain/event")]
    second = reqs[1]
    assert second["id"] != bar["id"] and second["method"] == "slab"
    assert abs(second["height"] - 0.015) < 0.004 and abs(second["depth"] - 0.38) < 0.015 and abs(second["near_x"] - 0.37) < 0.012
    rig.bus.publish(status, fake_ros.String(data=json.dumps({"state": "started", "request": second["id"], "plan": "..."})))
    rig.run(1.0, None)
    st = json.loads(rig.sent("/terrain/state")[-1].data)
    assert st["state"] == "SCALE_ACROSS" and st["kind"] == "scale_across"
    rig.bus.publish(status, fake_ros.String(data=json.dumps({"state": "done", "request": second["id"], "advance": 0.87})))
    rig.run(0.2, None)
    assert rig.state() == "CRUISE"


def test_round_bar_is_measured_and_walked_around(tmp_path):
    """A 5 cm round bar: low enough to be worth a closer look, but round (no foothold) and too
    big to carry a foot over. It is walked around, and the event says why."""
    rig = Rig(tmp_path)
    rig.run(0.5, jpeg(Scene(seed=1)))
    bar = lambda dist: jpeg(Scene(cylinders=[Cylinder(dist, 0.0, 0.05, 0.70)], seed=3))    # noqa: E731
    rig.run(0.6, bar(0.75))
    assert rig.state() == "APPROACH"
    rig.run(4.0, bar(0.34))        # (its foot is hidden under its own curve: it reads a little farther than it is)
    events = [m.data for m in rig.sent("/terrain/event")]
    assert not rig.sent("/terrain/step_over"), events
    assert any("decision: go_around" in e for e in events), events
    assert rig.state() == "AVOID"


def test_carton_is_walked_around_without_a_closer_look(tmp_path):
    """White all over and lower than the camera, but from a metre away already plainly too
    tall to cross: no approach, no stop."""
    rig = Rig(tmp_path)
    rig.run(0.5, jpeg(Scene(seed=1)))
    rig.run(1.0, jpeg(Scene(boxes=[Cuboid(1.0, 0.0, 0.348, 0.455, 0.13, side=WHITE, top=WHITE, side_shade=0.8)], seed=2)))
    states = {json.loads(m.data)["state"] for m in rig.sent("/terrain/state")}
    assert rig.state() == "AVOID" and "APPROACH" not in states and "GAUGE" not in states
    assert any("tall" in m.data for m in rig.sent("/terrain/event"))

