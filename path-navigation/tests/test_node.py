"""robot/nav_node.py and robot/sound_adapter.py off the robot: the real node code on a fake ROS."""
import json
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "robot"))
sys.path.insert(0, str(ROOT / "tests"))

import fake_ros  # noqa: E402
from nav.config import Topics  # noqa: E402
from sound_adapter import SoundAdapter  # noqa: E402

D = math.radians


# ------------------------------------------------------------------ the sound module's convention
def test_adapter_default_is_the_partners_convention_degrees_right_positive_in_json():
    # sound-localization/localize_angle.py prints 0 = nose, positive to the RIGHT; navigation is left positive
    a = SoundAdapter(Topics())
    assert a.from_text('{"bearing": 30, "confidence": 0.7}') == (pytest.approx(D(-30)), 0.7)
    assert a.from_text("-45")[0] == pytest.approx(D(45))
    assert a.from_text('{"azimuth_deg": 10}')[0] == pytest.approx(D(-10))     # other common names are understood
    left = SoundAdapter(Topics(sound_positive="left"))
    assert left.from_text('{"bearing": 30}')[0] == pytest.approx(D(30))
    assert a.from_text("help") is None and a.from_text('{"bearing": "left"}') is None and a.from_text("true") is None


def test_adapter_compass_convention_and_offset_and_radians():
    compass = SoundAdapter(Topics(sound_positive="right"))
    assert compass.convert(30) == pytest.approx(D(-30))
    turned = SoundAdapter(Topics(sound_zero_deg=90.0, sound_positive="left"))          # the module's zero is Pupper's left side
    assert turned.convert(0) == pytest.approx(D(90)) and turned.convert(-90) == pytest.approx(0.0)
    rad = SoundAdapter(Topics(sound_units="rad", sound_type="float32", sound_positive="left"))
    assert rad.parse(fake_ros.Float32(data=math.pi / 2))[0] == pytest.approx(math.pi / 2)
    wraps = SoundAdapter(Topics(sound_positive="left"))
    assert wraps.convert(350) == pytest.approx(D(-10))
    vec = SoundAdapter(Topics(sound_type="vector3", sound_positive="left"))
    assert vec.parse(fake_ros.Vector3Stamped(vector=fake_ros.Vector3(x=0.0, y=1.0)))[0] == pytest.approx(D(90))
    with pytest.raises(ValueError):
        SoundAdapter(Topics(sound_type="morse"))


def test_fake_sound_tool_inverts_the_adapter():
    sys.path.insert(0, str(ROOT / "tools"))
    import fake_sound
    for topics in (Topics(), Topics(sound_positive="right", sound_zero_deg=90.0), Topics(sound_units="rad")):
        a = SoundAdapter(topics)
        for left in (-120.0, -5.0, 0.0, 33.0, 170.0):
            assert a.convert(fake_sound.to_module(a, left)) == pytest.approx(D(left))


# ------------------------------------------------------------------ the node
class Rig:
    def __init__(self, motion, tmp_path):
        self.bus = fake_ros.install({"motion": motion, "log": False})
        sys.modules.pop("nav_node", None)
        import nav_node
        self.t = 500.0
        nav_node.NavNode.now = lambda *_: self.t
        nav_node.NavNode._voice_watch = lambda self_: None        # no HTTP in tests
        self.node = nav_node.NavNode()
        self.tick = self.node.timers[0][1]
        self.bus.publishers.add("/sound/bearing")

    def watch(self):
        """Mission Control starts listening."""
        self.bus.subs.setdefault("/nav/telemetry", []).append(lambda _m: None)

    def run(self, seconds, **topics):
        for _ in range(int(round(seconds * 20))):
            self.t += 0.05
            self.bus.publish("/imu_sensor_broadcaster/imu", fake_ros.Imu())
            self.bus.publish("/keyword/status", fake_ros.String(data='{"alive": true, "level_db": [-40]}'))
            self.bus.publish("/terrain/state", fake_ros.String(data='{"state": "CRUISE", "mode": "off"}'))
            self.bus.publish("/dodge/state", fake_ros.String(data=json.dumps({"state": "SEARCH", "armed": self.armed()})))
            for topic, text in topics.items():
                self.bus.publish(topic, fake_ros.String(data=text))
            self.tick()

    def armed(self):
        calls = [r.data for name, r in self.bus.calls if name == "/dodge/arm"]
        return calls[-1] if calls else False

    def cmd(self, text):
        self.bus.publish("/nav/command", fake_ros.String(data=text))

    def events(self):
        return [json.loads(m.data) for m in self.bus.sent.get("/nav/event", [])]

    def sent(self, topic):
        return self.bus.sent.get(topic, [])


def test_dry_run_node_listens_reports_and_publishes_no_velocity(tmp_path):
    r = Rig(False, tmp_path)
    r.run(1.5)
    assert r.sent("/nav/telemetry") == []                           # nobody is watching: nothing is built or sent
    r.watch()
    r.cmd("start")
    r.run(8.0)
    assert 30 <= len(r.sent("/nav/telemetry")) <= 45                # five a second, not twenty
    for _ in range(3):
        r.bus.publish("/sound/bearing", fake_ros.String(data='{"bearing": -90}'))   # localize_angle.py: -90 is left
        r.run(0.6)
    names = [e["event"] for e in r.events()]
    assert names[:3] == ["start", "fix", "heard"], names
    assert all(e.get("dry_run") is True for e in r.events())        # so Pupper does not say it is coming
    assert r.sent("/nav_cmd_vel") == []
    state = json.loads(r.sent("/nav/state")[-1].data)
    assert state["state"] == "ORIENT" and state["would_cmd"][2] > 0 and state["motion"] is False
    assert r.sent("/terrain/mode")[-1].data == "report"
    snap = json.loads(r.sent("/nav/telemetry")[-1].data)
    assert snap["mode"] == "robot" and snap["alive"]["sound"] and snap["alive"]["walking"] and snap["nav"]["target_deg"] > 80


def test_node_with_motion_turns_walks_and_arms_the_dodge_only_in_the_gauntlet(tmp_path):
    r = Rig(True, tmp_path)
    r.run(1.5)
    r.cmd("start")
    r.run(8.0)
    braking = r.sent("/nav_cmd_vel")                                # half a second of zeros on entering a listening state...
    assert len(braking) <= 12 and all(m.linear.x == 0 and m.angular.z == 0 for m in braking)
    r.run(1.0)
    assert len(r.sent("/nav_cmd_vel")) == len(braking)              # ...then nothing at all on the mux input
    for _ in range(3):
        r.bus.publish("/sound/bearing", fake_ros.String(data='{"bearing": 0}'))
        r.run(0.6)
    r.run(12.0)                                                     # faces it already; confirm times out; walks
    state = json.loads(r.sent("/nav/state")[-1].data)
    assert state["state"] == "ZONE1" and r.sent("/nav_cmd_vel")[-1].linear.x > 0.2
    assert r.sent("/terrain/mode")[-1].data == "full" and r.armed() is False
    assert not any(e.get("dry_run") for e in r.events())
    r.cmd("end"); r.run(3.0)
    r.cmd("goto:ZONE2"); r.run(2.0)
    assert r.armed() is True and r.sent("/terrain/mode")[-1].data == "report"
    r.bus.publish("/keyword/detected", fake_ros.String(data='{"phrase": "YOU_SAVED_ME_PUPPER", "channel": 0}'))
    r.run(0.2)
    assert json.loads(r.sent("/nav/state")[-1].data)["state"] == "ZONE2"     # too early: ignored
    r.cmd("end"); r.run(3.0)
    assert r.armed() is False
    r.cmd("goto:ARRIVE"); r.run(1.0)
    r.bus.publish("/keyword/detected", fake_ros.String(data='{"phrase": "YOU_SAVED_ME_PUPPER", "channel": 0}'))
    r.run(1.5)
    assert json.loads(r.sent("/nav/state")[-1].data)["state"] == "RESCUE" and r.armed() is False
    saved = [e for e in r.events() if e["event"] == "saved"]
    assert saved and saved[0]["distance_m"] == 6


def test_estop_message_pauses_the_run(tmp_path):
    r = Rig(True, tmp_path)
    r.run(1.0)
    r.cmd("goto:ZONE2"); r.run(0.5)
    r.bus.publish("/emergency_stop", fake_ros.Empty())
    r.run(0.2)
    assert json.loads(r.sent("/nav/state")[-1].data)["state"] == "PAUSED"
