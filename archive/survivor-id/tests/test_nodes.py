"""The three ROS nodes, built and driven with a stand-in ROS (tests/fake_ros.py).

This checks the wiring - topics, JSON, services, the calls to the robot server - not ROS
itself, the Hailo, or the robot. Those are DEPLOY.md's stages.

    python3 -m unittest discover -s tests -v
"""
from __future__ import annotations

import json
import os
import sys
import time
import unittest

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "robot"))

import fake_ros  # noqa: E402

fake_ros.install()

from world import CFG, FakeDetector, Scene, at, frames  # noqa: E402

import approach_node  # noqa: E402
import event_bridge  # noqa: E402
import survivor_node  # noqa: E402

TOPIC = CFG["mission"]["cmd_topic"]


def text(pub) -> list:
    return [json.loads(m.data) for m in pub.sent]


def wait_for(test, seconds: float = 20.0) -> bool:
    """True as soon as test() is. The limits are generous on purpose: these tests run threads in
    real time, and a cold start on a slow machine must not look like a failure."""
    end = time.time() + max(seconds, 20.0)
    while time.time() < end:
        if test():
            return True
        time.sleep(0.05)
    return False


class FakePose:
    closed = 0

    def __call__(self, image):
        raise AssertionError("the test's detector should have been used")

    def close(self):
        FakePose.closed += 1


class SurvivorNodeTests(unittest.TestCase):
    def setUp(self):
        self.who = at(30, 3.0)
        self.scene = Scene([self.who])
        self.fake = FakeDetector(self.scene, noise_px=1.0)
        fake_ros.Node.params_for_next = {"enabled": True}
        self.node = survivor_node.SurvivorNode(make_pose=FakePose)
        self.node.detector = lambda image, view: self.fake(image, view)
        self.jpegs = {cam: cv2.imencode(".jpg", img)[1].tobytes() for cam, img in frames().items()}
        self.feeding = True
        import threading
        threading.Thread(target=self.feed, daemon=True).start()

    def tearDown(self):
        self.feeding = False
        self.node.shutdown()

    def feed(self):
        """Cameras at 10 Hz and the IMU, as the robot's topics would deliver them."""
        while self.feeding:
            now = time.time()
            for cam, cfg in CFG["cameras"].items():
                msg = fake_ros.CompressedImage(data=self.jpegs[cam])
                msg.header.stamp.sec, msg.header.stamp.nanosec = int(now), int((now % 1) * 1e9)
                self.node.subs[cfg["topic"]](msg)
            self.node.subs[CFG["node"]["imu_topic"]](fake_ros.Imu())
            time.sleep(0.1)

    def test_frames_in_lock_out_then_cancel_and_disable(self):
        n = self.node
        self.assertEqual(set(n.subs), {"/camera/image_raw/compressed", "/elp_left/image/compressed",
                                       "/elp_right/image/compressed", "/imu_sensor_broadcaster/imu", "/cmd_vel"})
        events = n.pubs["/survivor/event"]
        self.assertTrue(wait_for(lambda: any(e["event"] == "locked" for e in text(events))), text(events))
        self.assertEqual([e["event"] for e in text(events)], ["candidate", "locked"])
        self.assertTrue(wait_for(lambda: text(n.pubs["/survivor/perception"])[-1]["phase"] == "LOCKED"))
        status = text(n.pubs["/survivor/perception"])[-1]
        self.assertAlmostEqual(status["target"]["bearing_deg"], 30.0, delta=5.0)
        self.assertAlmostEqual(status["target"]["range_m"], 3.0, delta=0.5)
        self.assertTrue(status["imu_alive"] and status["pose_model"])
        self.assertEqual(set(status["cameras"]), set(CFG["cameras"]))
        self.assertTrue(wait_for(lambda: n.pubs["/survivor/goal"].sent))
        goal = n.pubs["/survivor/goal"].sent[-1]
        self.assertEqual(goal.header.frame_id, "base_link")
        self.assertAlmostEqual(goal.pose.position.x, 3.0 * 0.866, delta=0.5)     # 3 m away, 30 degrees to the left
        self.assertAlmostEqual(goal.pose.position.y, 3.0 * 0.5, delta=0.4)
        def pinned():                                                              # /people is slower (5 Hz): wait for one after the lock
            sent = n.pubs["/people"].sent
            return bool(sent) and any(p["survivor"] and p["visible"] for p in json.loads(sent[-1].data)["people"])
        self.assertTrue(wait_for(pinned))
        n.pubs["/survivor/debug/compressed"].listeners = 1                         # someone opens tools/view.py
        self.assertTrue(wait_for(lambda: n.pubs["/survivor/debug/compressed"].sent))
        picture = cv2.imdecode(__import__("numpy").frombuffer(n.pubs["/survivor/debug/compressed"].sent[-1].data, "uint8"), 1)
        self.assertEqual(picture.shape[0], 360)

        answer = n.call("/survivor/cancel")
        self.assertEqual((answer.success, answer.message), (True, "pin dropped"))
        self.assertTrue(wait_for(lambda: text(events)[-1]["event"] == "cancelled"))
        self.assertTrue(wait_for(lambda: text(n.pubs["/survivor/perception"])[-1]["target"] is None))

        closed = FakePose.closed
        self.assertTrue(n.call("/survivor/disable").success)
        self.assertTrue(wait_for(lambda: FakePose.closed == closed + 1))           # the Hailo is released
        self.assertFalse(text(n.pubs["/survivor/perception"])[-1]["enabled"])
        self.assertTrue(n.call("/survivor/enable").success)
        self.assertTrue(wait_for(lambda: text(n.pubs["/survivor/perception"])[-1]["pose_model"]))


class SurvivorNodeNoModelTests(unittest.TestCase):
    def test_no_pose_model_is_reported_not_fatal(self):
        def broken():
            raise RuntimeError("HAILO_OUT_OF_PHYSICAL_DEVICES")
        fake_ros.Node.params_for_next = {"enabled": True}
        node = survivor_node.SurvivorNode(make_pose=broken)
        try:
            self.assertTrue(wait_for(lambda: node.pubs["/survivor/perception"].sent))
            status = text(node.pubs["/survivor/perception"])[-1]
            self.assertFalse(status["pose_model"])
            self.assertIn("Hailo", status["error"])
            self.assertEqual(status["phase"], "SCAN")
        finally:
            node.shutdown()


class FakeClient:
    """The robot server, as far as the approach node and the bridge can tell."""

    def __init__(self, tricks=None, up=True):
        self.calls, self.up = [], up
        self.installed = {"push_up": 0.2} if tricks is None else tricks
        self.mode_text = "mode: locomotion"
        self.busy_for = 0.3
        self.busy_until = 0.0

    def reachable(self):
        return self.up

    def mode(self):
        return self.up, self.mode_text

    def stop_now(self):
        self.calls.append("stop_now")
        return self.up, "stopped"

    def stand_up(self):
        self.calls.append("stand_up")
        return self.up, "stand up queued"

    def tricks(self):
        return self.up, dict(self.installed)

    def play(self, name):
        self.calls.append("play " + name)
        self.busy_until = time.time() + self.busy_for
        return self.up, "trick %s queued" % name

    def busy(self):
        return self.up, time.time() < self.busy_until

    def mission_event(self, name, **details):
        self.calls.append(("mission_event", name, details))
        return self.up, "MISSION EVENT: " + name


def status_msg(target=None, **kw) -> fake_ros.String:
    st = {"t": time.time(), "enabled": True, "phase": "LOCKED" if target else "SCAN", "target": target, "look": None,
          "queued": [], "people": 1 if target else 0, "tracks": [], "pose": {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}}
    st.update(kw)
    return fake_ros.String(data=json.dumps(st))


def pin(bearing=10.0, rng=3.0, visible=True) -> dict:
    return {"id": 1, "bearing_deg": bearing, "range_m": rng, "visible": visible, "since_seen_s": 0.0, "sigma_m": 0.4,
            "box_frac": 0.2, "cam": "fisheye", "x": rng, "y": 0.0}


class ApproachNodeTests(unittest.TestCase):
    def make(self, motion: bool, client=None):
        fake_ros.Node.params_for_next = {"motion": motion}
        self.client = client or FakeClient()
        node = approach_node.ApproachNode(client=self.client)
        self.addCleanup(node.shutdown)
        node.clients["/controller_manager/list_controllers"].answer = fake_ros.Msg(
            controller=[fake_ros.Msg(name="gemini_walk", state="active"), fake_ros.Msg(name="push_up", state="inactive")])
        node.on_controller_timer()
        node.subs["/imu_sensor_broadcaster/imu"](fake_ros.Imu())
        self.assertTrue(wait_for(lambda: node.server_ok, 3.0))
        return node

    def ticks(self, node, seconds: float, msg=None):
        for _ in range(int(seconds * 20)):
            if msg is not None:
                node.subs["/survivor/perception"](msg() if callable(msg) else msg)
            node.subs["/imu_sensor_broadcaster/imu"](fake_ros.Imu())
            node.on_tick()
            time.sleep(0.05)
        node.on_state_timer()
        return text(node.pubs["/survivor/state"])[-1]

    def test_dry_run_reports_but_sends_no_velocity_and_asks_the_server_for_nothing(self):
        node = self.make(False)
        self.assertEqual(self.ticks(node, 0.3, status_msg())["state"], "SCAN")
        state = self.ticks(node, 1.0, lambda: status_msg(pin()))
        self.assertEqual((state["state"], state["motion"]), ("APPROACH", False))
        self.assertIsNotNone(state["cmd"])                                # what it would send
        self.assertEqual(node.pubs[TOPIC].sent, [])                       # ... and did not
        self.assertEqual(self.client.calls, [])

    def test_motion_on_clears_the_queue_and_walks(self):
        node = self.make(False)
        answer = node.call("/survivor/motion", data=True)
        self.assertTrue(answer.success and "motion on" in answer.message)
        node.subs["/cmd_vel_mux/active_source"](fake_ros.String(data=TOPIC))
        state = self.ticks(node, 1.0, lambda: status_msg(pin(bearing=10.0)))
        self.assertEqual(state["state"], "APPROACH")
        self.assertTrue(wait_for(lambda: "stop_now" in self.client.calls, 2.0))
        sent = node.pubs[TOPIC].sent[-1]
        self.assertEqual(sent.linear.x, CFG["mission"]["speed_far"])
        self.assertGreater(sent.angular.z, 0.0)                           # steering left, toward +10 degrees
        self.assertNotIn("stand_up", self.client.calls)                   # already walking

    def test_not_walking_means_stand_up_first_and_fallen_means_wait(self):
        node = self.make(True)
        node.clients["/controller_manager/list_controllers"].answer = fake_ros.Msg(controller=[])
        node.on_controller_timer()
        state = self.ticks(node, 0.5, lambda: status_msg(pin()))
        self.assertEqual(state["state"], "LOCKED")
        self.assertTrue(wait_for(lambda: "stand_up" in self.client.calls, 2.0))
        self.assertTrue(all(m.linear.x == 0.0 for m in node.pubs[TOPIC].sent))

        node2 = self.make(True)
        self.client.mode_text = "mode: fallen over (tilt e-stop in gemini_walk). Someone has to set Pupper on its feet."
        self.assertTrue(wait_for(lambda: node2.fallen, 3.0))
        self.ticks(node2, 0.5, lambda: status_msg(pin()))
        self.assertNotIn("stand_up", self.client.calls)

    def test_arrival_barks_bounces_and_stays(self):
        node = self.make(True)
        node.subs["/cmd_vel_mux/active_source"](fake_ros.String(data=TOPIC))
        self.ticks(node, 0.6, lambda: status_msg(pin(rng=2.0)))
        state = self.ticks(node, 0.6, lambda: status_msg(pin(rng=0.65)))
        self.assertTrue(wait_for(lambda: "play push_up" in self.client.calls, 3.0), self.client.calls)
        events = text(node.pubs["/survivor/event"])
        self.assertEqual([e["event"] for e in events], ["arrived"])
        self.assertNotIn("dry_run", events[0])
        state = self.ticks(node, 0.2, lambda: status_msg(pin(rng=0.65)))
        self.assertEqual((state["state"], state["can_dodge"]), ("CELEBRATE", False))
        end = time.time() + 30.0
        while time.time() < end and state["state"] != "STAY":
            state = self.ticks(node, 0.5, lambda: status_msg(pin(rng=0.65)))
        self.assertEqual(state["state"], "STAY")
        self.assertEqual(self.client.calls.count("play push_up"), 1)
        last = node.pubs[TOPIC].sent[-1]
        self.assertEqual((last.linear.x, last.linear.y, last.angular.z), (0.0, 0.0, 0.0))    # holding position

    def test_no_trick_installed_or_no_server_still_arrives(self):
        for client in (FakeClient(tricks={}), FakeClient(up=False)):
            fake_ros.Node.params_for_next = {"motion": True}
            node = approach_node.ApproachNode(client=client)
            self.addCleanup(node.shutdown)
            node.clients["/controller_manager/list_controllers"].answer = fake_ros.Msg(
                controller=[fake_ros.Msg(name="gemini_walk", state="active")])
            node.on_controller_timer()
            time.sleep(1.2)                                               # let it notice the server (or its absence)
            self.ticks(node, 0.6, lambda: status_msg(pin(rng=2.0)))
            state = self.ticks(node, 0.6, lambda: status_msg(pin(rng=0.65)))
            end = time.time() + 30.0
            while time.time() < end and state["state"] != "STAY":
                state = self.ticks(node, 0.5, lambda: status_msg(pin(rng=0.65)))
            self.assertEqual(state["state"], "STAY", state)
            self.assertFalse(any(str(c).startswith("play") for c in client.calls))

    def test_ball_in_the_air_and_terrain_messages_are_understood(self):
        node = self.make(True)
        node.subs["/dodge/state"](fake_ros.String(data=json.dumps({"state": "SEARCH", "tracks": []})))
        self.assertGreater(time.time() - node.ball_at, 100.0)
        node.subs["/dodge/state"](fake_ros.String(data=json.dumps({"state": "TRACK", "tracks": [{"confirmed": True}]})))
        self.assertLess(time.time() - node.ball_at, 1.0)
        node.subs["/terrain/state"](fake_ros.String(data="STEP_OVER"))
        self.assertEqual(node.terrain_state, "STEP_OVER")
        node.subs["/terrain/state"](fake_ros.String(data=json.dumps({"state": "avoid"})))
        self.assertEqual(node.terrain_state, "AVOID")
        node.subs["/terrain/event"](fake_ros.String(data=json.dumps({"event": "blocked"})))
        self.assertTrue(node.terrain_blocked)
        node.subs["/cmd_vel_mux/active_source"](fake_ros.String(data=TOPIC))
        self.ticks(node, 0.6, lambda: status_msg(pin()))
        node.subs["/terrain/state"](fake_ros.String(data="STEP_OVER"))
        self.assertEqual(self.ticks(node, 0.2, lambda: status_msg(pin()))["state"], "PAUSED")


class EventBridgeTests(unittest.TestCase):
    def test_filter(self):
        f = event_bridge.Filter(CFG)
        self.assertEqual(f.accept("survivor", {"event": "locked", "range_m": 3.1}, 100.0, True), "")
        self.assertEqual(f.accept("survivor", {"event": "candidate"}, 100.0, True), "not announced")
        self.assertEqual(f.accept("survivor", {"event": "arrived", "dry_run": True}, 100.0, True), "dry run")
        self.assertEqual(f.accept("survivor", {"event": "lost"}, 100.0, False), "motion is off")
        self.assertEqual(f.accept("dodge", {"event": "dodge", "dry_run": True}, 100.0, True), "dry run")
        self.assertEqual(f.accept("dodge", {"event": "cannot_dodge"}, 100.0, True), "not announced")
        self.assertEqual(f.accept("dodge", {"event": "dodge", "dry_run": False}, 100.0, True), "")
        self.assertEqual(f.accept("dodge", {"event": "dodge", "dry_run": False}, 104.0, True), "said a moment ago")
        self.assertEqual(f.accept("dodge", {"event": "dodge", "dry_run": False}, 111.0, True), "")
        self.assertEqual(event_bridge.details_for({"event": "locked", "range_m": 3.2}), {"range": 3})
        self.assertEqual(event_bridge.details_for({"event": "arrived", "range_m": 0.68}), {"range": 0.7})
        self.assertEqual(event_bridge.details_for({"event": "blocked", "range_m": 2.0}), {})

    def test_events_reach_the_robot_server(self):
        client = FakeClient()
        node = event_bridge.EventBridge(client=client)
        self.addCleanup(node.shutdown)
        node.subs["/survivor/state"](fake_ros.String(data=json.dumps({"state": "APPROACH", "motion": True})))
        node.subs["/survivor/event"](fake_ros.String(data=json.dumps({"event": "candidate", "id": 1})))
        node.subs["/survivor/event"](fake_ros.String(data=json.dumps({"event": "locked", "id": 1, "range_m": 3.2})))
        node.subs["/dodge/event"](fake_ros.String(data=json.dumps({"event": "dodge", "dry_run": False, "plan": {}})))
        node.subs["/survivor/event"](fake_ros.String(data=json.dumps({"event": "arrived", "id": 1, "range_m": 0.7})))
        self.assertTrue(wait_for(lambda: len(client.calls) == 3, 3.0), client.calls)
        self.assertEqual(client.calls, [("mission_event", "locked", {"range": 3}), ("mission_event", "dodge", {}),
                                        ("mission_event", "arrived", {"range": 0.7})])


if __name__ == "__main__":
    unittest.main()
