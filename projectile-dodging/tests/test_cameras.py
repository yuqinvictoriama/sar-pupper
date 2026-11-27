"""Off-robot tests for the camera side of the node: fresh frames from a USB camera, and each
camera's capture and ball search running in a process of its own.

    python3 -m unittest discover -s tests -v
"""
from __future__ import annotations

import copy
import os
import sys
import tempfile
import threading
import time
import unittest

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "robot"))
sys.path.insert(0, HERE)

import camera_worker  # noqa: E402
import usb_camera  # noqa: E402
from dodge.camera_models import build_cameras  # noqa: E402
from dodge.config import load_config  # noqa: E402
from dodge.core import DodgeCore, predict_pixel  # noqa: E402
from dodge.pipeline import FramePipeline  # noqa: E402
from dodge.recording import RecordingWriter  # noqa: E402
from test_dodge import render_ball  # noqa: E402

CFG = load_config(use_calib=False)


class FakeCapture:
    """A camera that makes a frame every `period` seconds and, like a real one, holds at most `queue`
    of them for a reader that is slow: the oldest waiting frame is what the next grab returns."""

    def __init__(self, period=0.01, queue=4):
        self.period, self.queue = period, queue
        self.t0 = time.monotonic()
        self.next_index = 0          # the next frame the reader has not had yet
        self.grabbed = None
        self.released = False

    def grab(self):
        newest = int((time.monotonic() - self.t0) / self.period)
        if self.next_index + self.queue <= newest:             # the queue overflowed: what is left are the first few
            self.grabbed = self.next_index                     # that fitted, old by now
            self.next_index = self.grabbed + 1
            if self.grabbed + self.queue <= newest - self.queue:
                self.next_index = newest - self.queue + 1      # everything in between was dropped by the camera
            return True
        if self.next_index > newest:                           # nothing waiting: block until the next frame
            time.sleep(max(0.0, self.t0 + self.next_index * self.period - time.monotonic()))
        self.grabbed = self.next_index
        self.next_index += 1
        return True

    def stamp(self, index):
        return self.t0 + index * self.period

    def get(self, prop):
        if prop == cv2.CAP_PROP_POS_MSEC:
            return self.stamp(self.grabbed) * 1000.0
        return {cv2.CAP_PROP_FRAME_WIDTH: 64, cv2.CAP_PROP_FRAME_HEIGHT: 48, cv2.CAP_PROP_FPS: 1.0 / self.period}.get(prop, 0)

    def retrieve(self):
        return True, np.full((48, 64, 3), 100, dtype=np.uint8)

    def release(self):
        self.released = True


class FreshFrameTests(unittest.TestCase):
    def run_camera(self, work_s, seconds=1.2, max_fps=0):
        cap = FakeCapture()
        ages = []

        def callback(_name, t_capture, _jpeg, _bgr):
            ages.append(time.time() - t_capture)
            time.sleep(work_s)

        original = usb_camera.open_capture
        usb_camera.open_capture = lambda *_a, **_k: (cap, False)
        try:
            cam = usb_camera.UsbCamera("elp_left", {"device": "/dev/null", "max_process_fps": max_fps}, callback,
                                       log=lambda _m: None)
            cam.start()
            time.sleep(seconds)
            self.assertTrue(cam.close(3.0))
        finally:
            usb_camera.open_capture = original
        self.assertTrue(cap.released)
        return ages

    def test_a_slow_callback_still_gets_fresh_frames(self):
        # 60 ms of work per frame against a camera making a frame every 10 ms: every frame handed over must be
        # the one that has just arrived, not one that sat in the camera's queue while the last was worked on.
        ages = self.run_camera(work_s=0.06)
        self.assertGreater(len(ages), 10)
        self.assertLess(max(ages[2:]), 0.03, "frames were up to %.0f ms old when handed over" % (1000 * max(ages[2:])))

    def test_frame_rate_cap_is_kept(self):
        ages = self.run_camera(work_s=0.0, seconds=1.0, max_fps=20)
        self.assertGreater(len(ages), 14)
        self.assertLess(len(ages), 25)


def make_recording(folder, cam_name, n=12):
    """A short recording of a ball flying across one camera's view. Returns the ball's positions."""
    camera = build_cameras(CFG)[cam_name]
    writer = RecordingWriter(folder, {"note": "test"})
    points = []
    for i in range(n):
        p = np.array([-1.6 + 0.02 * i, 1.4 - 0.05 * i, 0.5 + 0.03 * i])
        img, ok = render_ball(camera, p)
        assert ok
        done, jpeg = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 92])
        assert done
        writer.add_frame(cam_name, 100.0 + i / 30.0, jpeg.tobytes())
        points.append(p)
    writer.close()
    return points


class CameraFarmTests(unittest.TestCase):
    def run_farm(self, processes):
        with tempfile.TemporaryDirectory() as d:
            make_recording(d, "elp_left")
            cfg = copy.deepcopy(CFG)
            cfg["cameras"] = {"elp_left": dict(cfg["cameras"]["elp_left"], source="recording", path=d, speed=0)}   # every frame, however slow the computer
            got, logs, lock = [], [], threading.Lock()

            def on_result(res, jpeg):
                with lock:
                    got.append((res, jpeg))

            farm = camera_worker.CameraFarm(cfg, ["elp_left"], on_result, log=logs.append, processes=processes)
            farm.start()
            farm.set_hint(None)
            end = time.time() + 20.0
            while time.time() < end and len(got) < 12:
                time.sleep(0.05)
            status = farm.status("elp_left")
            farm.stop()
        return got, logs, status

    def check(self, got, logs):
        self.assertEqual(len(got), 12, "results: %d, log: %s" % (len(got), logs))
        camera = build_cameras(CFG)["elp_left"]
        for res, jpeg in got:
            self.assertEqual(res.cam, "elp_left")
            self.assertEqual(tuple(res.size), (camera.model.width, camera.model.height))
            self.assertEqual(len(res.blobs), 1)
            self.assertIsNotNone(jpeg)
            self.assertEqual(bytes(jpeg[:2]), b"\xff\xd8")           # the camera's own JPEG, for the recorder
            self.assertLess(res.start - res.raw, 0.25)
            self.assertAlmostEqual(res.t - res.raw, CFG["cameras"]["elp_left"].get("time_offset_s", 0.0), places=6)

    def test_cameras_in_their_own_processes(self):
        got, logs, _status = self.run_farm(processes=True)
        self.check(got, logs)

    def test_cameras_as_threads(self):
        got, logs, _status = self.run_farm(processes=False)
        self.check(got, logs)


class HintTests(unittest.TestCase):
    def test_a_hint_makes_the_camera_look_where_the_ball_will_be(self):
        core = DodgeCore(CFG)
        camera = build_cameras(CFG)["elp_left"]
        pipe = FramePipeline("elp_left", CFG)
        self.assertIsNone(core.track_hint())
        # a ball coming in from the rear left, seen by the pipeline frame by frame
        p0, v = np.array([-2.2, 1.8, 0.9]), np.array([3.0, -2.5, 0.5])
        used_roi = 0
        for i in range(14):
            t = 50.0 + i / 30.0
            p = p0 + v * (i / 30.0) + np.array([0.0, 0.0, -4.905]) * (i / 30.0) ** 2
            img, ok = render_ball(camera, p)
            self.assertTrue(ok)
            res = pipe.process(t - pipe.time_offset, img)
            self.assertEqual(len(res.blobs), 1, "frame %d" % i)
            used_roi += res.roi
            core.process_blobs("elp_left", res.t, res.blobs, res.t, res.size)
            hint = core.track_hint()
            pipe.set_hint(hint)
            if hint is not None:
                pred = predict_pixel(camera, hint, res.t, CFG["detector"])
                self.assertIsNotNone(pred)
                self.assertLess(abs(pred[0] - res.blobs[0].u), 25.0)
                self.assertLess(abs(pred[1] - res.blobs[0].v), 25.0)
        self.assertGreater(used_roi, 3)


class LensDiscTests(unittest.TestCase):
    def test_fisheye_search_stays_inside_the_lit_disc_and_finds_the_same_ball(self):
        cfg = copy.deepcopy(CFG)
        camera = build_cameras(cfg)["fisheye"]
        pipe = FramePipeline("fisheye", cfg)
        m = camera.model
        x0, y0, x1, y1 = pipe.detector.search_box(m.width, m.height)
        self.assertLess((x1 - x0) * (y1 - y0), 0.5 * m.width * m.height)      # under half the frame is searched
        img, ok = render_ball(camera, [1.2, 0.1, 0.3])
        self.assertTrue(ok)
        blobs = pipe.detector.detect(img)
        self.assertEqual(len(blobs), 1)
        self.assertTrue(x0 <= blobs[0].u <= x1 and y0 <= blobs[0].v <= y1)



class NodeTests(unittest.TestCase):
    """The real robot/dodge_node.py on stand-in ROS, fed by a camera process playing a recorded throw."""

    def test_node_dodges_a_recorded_throw_seen_by_a_camera_process(self):
        import json

        import yaml

        import fake_ros
        from dodge.sim import aimed_throw
        saved = {k: sys.modules.get(k) for k in list(sys.modules) if k.split(".")[0] in (
            "rclpy", "geometry_msgs", "sensor_msgs", "std_msgs", "std_srvs", "controller_manager_msgs")}
        fake_ros.install()
        sys.modules.pop("dodge_node", None)
        try:
            import dodge_node
            with tempfile.TemporaryDirectory() as d:
                camera = build_cameras(CFG)["elp_left"]
                throw = aimed_throw(CFG, bearing_deg=150.0, distance=3.0, rise=0.7)
                writer = RecordingWriter(os.path.join(d, "rec"), {})
                n = int(throw.t_land * 30.0)
                for i in range(n):
                    img, _ok = render_ball(camera, throw.position(i / 30.0))
                    writer.add_frame("elp_left", 10.0 + i / 30.0, cv2.imencode(".jpg", img)[1].tobytes())
                writer.close()
                cfg = copy.deepcopy(CFG)
                cfg.pop("_path", None)
                cfg["cameras"] = {"elp_left": dict(cfg["cameras"]["elp_left"], source="recording",
                                                   path=os.path.join(d, "rec"), time_offset_s=0.0)}
                cfg["node"].update(require_active_controller=False, event_dir=os.path.join(d, "throws"),
                                   event_post_s=0.3, camera_processes=True)
                with open(os.path.join(d, "dodge.yaml"), "w") as f:
                    yaml.safe_dump(cfg, f)
                fake_ros.Node.params_for_next = {"config": os.path.join(d, "dodge.yaml")}
                node = dodge_node.DodgeNode()
                try:
                    events, end = [], time.time() + 25.0
                    while time.time() < end and not any(e["event"] == "dodge" for e in events):
                        node.on_cmd_timer()
                        events = [json.loads(m.data) for m in node.pubs["/dodge/event"].sent]
                        time.sleep(0.01)
                    dodges = [e for e in events if e["event"] == "dodge"]
                    self.assertEqual(len(dodges), 1, "events: %s, log: %s" % (events, node.get_logger().lines))
                    self.assertTrue(dodges[0]["dry_run"])                       # not armed: it only reports
                    self.assertEqual(node.pubs["/dodge_cmd_vel"].sent, [])
                    self.assertGreater(dodges[0]["threat"]["tau"], 0.3)         # decided with time in hand
                    node.on_status_timer()
                    state = json.loads(node.pubs["/dodge/state"].sent[-1].data)
                    cam = state["cameras"]["elp_left"]
                    self.assertTrue(cam["alive"])
                    self.assertLess(cam["latency_ms"], 150.0)
                    for key in ("decode_ms", "detect_ms", "wait_ms"):
                        self.assertIn(key, cam)
                    end = time.time() + 5.0                                     # the throw recorder saves it
                    saved_dirs = []
                    while time.time() < end and not saved_dirs:
                        time.sleep(0.1)
                        if os.path.isdir(cfg["node"]["event_dir"]):
                            saved_dirs = [p for p in os.listdir(cfg["node"]["event_dir"])
                                          if os.path.exists(os.path.join(cfg["node"]["event_dir"], p, "elp_left.idx"))]
                    self.assertEqual(len(saved_dirs), 1)
                    time.sleep(0.3)
                finally:
                    node.shutdown()
        finally:
            for k in list(sys.modules):
                if k.split(".")[0] in ("rclpy", "geometry_msgs", "sensor_msgs", "std_msgs", "std_srvs",
                                       "controller_manager_msgs", "dodge_node"):
                    del sys.modules[k]
            sys.modules.update({k: v for k, v in saved.items() if v is not None})


class ClutterTests(unittest.TestCase):
    """A room full of still green things must not keep a thrown ball from being tracked."""

    def test_a_thrown_ball_is_tracked_although_clutter_fills_every_slot(self):
        from dodge.sim import aimed_throw
        cfg = copy.deepcopy(CFG)
        cfg["tracker"]["max_tracks"] = 6
        core = DodgeCore(cfg)
        camera = build_cameras(cfg)["elp_left"]
        from dodge.measure import measure_blobs
        from dodge.detect import BallDetector
        det = BallDetector.for_camera(cfg["cameras"]["elp_left"], cfg["detector"])
        clutter = [np.array([-2.0 - 0.1 * i, 1.0 + 0.35 * i, -0.05 + 0.1 * (i % 3)]) for i in range(8)]   # 8 balls lying about
        throw = aimed_throw(cfg, bearing_deg=140.0, distance=2.8, rise=0.6)
        dodged = False
        for i in range(-30, int(throw.t_land * 30.0)):
            t = 200.0 + i / 30.0
            img, _ok = render_ball(camera, clutter[0])
            for p in clutter[1:]:
                more, ok = render_ball(camera, p)
                if ok:
                    img = np.where(more != 110, more, img)
            ball = throw.position(i / 30.0) if i >= 0 else None
            if ball is not None:
                more, ok = render_ball(camera, ball)
                if ok:
                    img = np.where(more != 110, more, img)
            blobs = det.detect(img)
            if i == -1:
                self.assertGreaterEqual(len(core.tracker.tracks), 6)        # every slot is taken before the throw
            dodged = core.process_blobs("elp_left", t, blobs, t) or dodged
        self.assertTrue(dodged, "the throw was not dodged; events: %s" % list(core.events))

    def test_a_new_blob_gets_a_track_when_every_slot_is_taken(self):
        from dodge.measure import Measurement
        from dodge.track import Tracker
        cfg = dict(CFG["tracker"], max_tracks=5)
        tracker = Tracker(cfg, CFG["ball"])

        def meas(point):
            point = np.asarray(point, dtype=float)
            rng = float(np.linalg.norm(point))
            return Measurement("elp_left", np.zeros(3), point / rng, 0.074 / rng, rng, 0.01, 0.15, False)

        still = [[-2.0, 0.5 * i - 1.0, 0.0] for i in range(5)]
        for k in range(6):                                   # five still things, seen again and again: all slots confirmed
            tracker.update(10.0 + k / 30.0, [meas(p) for p in still])
        self.assertEqual(len(tracker.confirmed()), 5)
        ball = [-1.0, 2.5, 1.2]                              # something new, nowhere near any of them
        tracker.update(10.0 + 6 / 30.0, [meas(p) for p in still] + [meas(ball)])
        nearest = min(float(np.linalg.norm(tr.x[:3] - np.array(ball))) for tr in tracker.tracks)
        self.assertLess(nearest, 0.05, "the new blob was ignored because the tracker was full")
        self.assertLessEqual(len(tracker.tracks), 5)

    def test_a_colour_calibration_that_is_not_green_is_ignored(self):
        from dodge.config import save_calib
        with tempfile.TemporaryDirectory() as d:
            save_calib("fisheye_color", {"cameras": {"fisheye": {"color": {"h_min": 9, "h_max": 28, "fit": 2},
                                                                 "size_scale": 1.05}}}, calib_dir=d)
            save_calib("elp_left_color", {"cameras": {"elp_left": {"color": {"h_min": 33, "h_max": 64, "fit": 2}}}}, calib_dir=d)
            cfg = load_config(calib_dir=d)
            self.assertEqual(cfg["cameras"]["fisheye"]["color"], CFG["cameras"]["fisheye"]["color"])
            self.assertEqual(cfg["cameras"]["fisheye"]["size_scale"], 1.05)
            self.assertEqual(cfg["cameras"]["elp_left"]["color"]["h_min"], 33)


if __name__ == "__main__":
    unittest.main()
