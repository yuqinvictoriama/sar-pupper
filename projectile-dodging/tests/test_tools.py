"""Off-robot tests for the recording format, replay and the extrinsics solver.

    python3 -m unittest discover -s tests -v
"""
from __future__ import annotations

import math
import os
import sys
import tempfile
import unittest

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "tools"))
sys.path.insert(0, HERE)

from dodge.camera_models import build_cameras, rotation_base_from_camera  # noqa: E402
from dodge.config import deep_merge, load_config, save_calib  # noqa: E402
from dodge.recording import RecordingReader, RecordingWriter, RingRecorder  # noqa: E402
from dodge.sim import aimed_throw  # noqa: E402
from test_dodge import render_ball  # noqa: E402

CFG = load_config(use_calib=False)


class ConfigTests(unittest.TestCase):
    def test_calib_files_override_the_main_config(self):
        with tempfile.TemporaryDirectory() as d:
            save_calib("elp_left_color", {"cameras": {"elp_left": {"color": {"h_min": 41}}}}, calib_dir=d)
            save_calib("elp_left_color", {"cameras": {"elp_left": {"size_scale": 0.97}}}, calib_dir=d)
            cfg = load_config(calib_dir=d)
            self.assertEqual(cfg["cameras"]["elp_left"]["color"]["h_min"], 41)
            self.assertEqual(cfg["cameras"]["elp_left"]["size_scale"], 0.97)
            self.assertEqual(cfg["cameras"]["elp_left"]["color"]["h_max"], CFG["cameras"]["elp_left"]["color"]["h_max"])
            self.assertEqual(cfg["cameras"]["elp_right"]["color"], CFG["cameras"]["elp_right"]["color"])

    def test_deep_merge_does_not_touch_its_inputs(self):
        a = {"x": {"y": 1, "z": 2}}
        out = deep_merge(a, {"x": {"y": 5}})
        self.assertEqual(a["x"]["y"], 1)
        self.assertEqual(out, {"x": {"y": 5, "z": 2}})


class RecordingTests(unittest.TestCase):
    def test_write_read_round_trip(self):
        with tempfile.TemporaryDirectory() as d:
            w = RecordingWriter(d, {"note": "test"})
            w.add_frame("a", 2.0, b"second")
            w.add_frame("b", 1.5, b"middle")
            w.add_frame("a", 1.0, b"first")
            w.add_imu(1.2, (1, 0, 0, 0), (0.1, 0.2, 0.3), (0, 0, 9.8))
            w.add_event({"event": "dodge", "t": 1.9})
            w.close()
            r = RecordingReader(d)
            self.assertEqual([(t, c, bytes(j)) for t, c, j in r.frames()],
                             [(1.0, "a", b"first"), (1.5, "b", b"middle"), (2.0, "a", b"second")])
            self.assertEqual(r.imu()[0][1], (1.0, 0.0, 0.0, 0.0))
            self.assertEqual(r.events()[0]["event"], "dodge")
            self.assertEqual(r.meta["note"], "test")
            r.close()

    def test_ring_keeps_only_recent_frames(self):
        ring = RingRecorder(1.0)
        for i in range(30):
            ring.add_frame("a", i * 0.1, b"x%d" % i)
        with tempfile.TemporaryDirectory() as d:
            ring.dump(d)
            r = RecordingReader(d)
            times = [t for t, _c, _j in r.frames()]
            r.close()
        self.assertGreaterEqual(min(times), 1.9 - 1e-9)
        self.assertAlmostEqual(max(times), 2.9)


class ReplayTests(unittest.TestCase):
    def test_replay_of_a_rendered_throw_dodges(self):
        """Full image pipeline: rendered frames -> JPEG -> recording -> replay -> one dodge, in the right direction."""
        from replay import replay
        cams = build_cameras(CFG)
        cam = cams["elp_left"]
        throw = aimed_throw(CFG, 135.0, 3.5, rise=0.8)
        offset = float(CFG["cameras"]["elp_left"]["time_offset_s"])
        with tempfile.TemporaryDirectory() as d:
            w = RecordingWriter(d, {"kind": "synthetic"})
            t = 0.0
            while t < throw.t_land:
                p = throw.position(t)
                img, _ = render_ball(cam, p)
                ok, enc = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])
                w.add_frame("elp_left", 1000.0 + t - offset, enc.tobytes())
                t += 1.0 / 60.0
            w.close()
            r = replay(d, CFG, quiet=True)
        self.assertEqual(r["dodges"], 1)
        ev = [e for e in r["events"] if e["event"] == "dodge"][0]
        self.assertLess(abs(ev["threat"]["bearing_deg"] - 135.0), 12.0)
        self.assertGreater(throw.t_land - ev["t_rel"], 0.45)        # fired with time to spare
        self.assertGreater(r["tracked_s"], 0.3)

    def test_replay_of_an_empty_scene_does_nothing(self):
        from replay import replay
        cam = build_cameras(CFG)["elp_right"]
        img = np.full((cam.model.height, cam.model.width, 3), 120, dtype=np.uint8)
        cv2.rectangle(img, (200, 300), (500, 600), (40, 90, 160), -1)      # a brown box, nothing green
        ok, enc = cv2.imencode(".jpg", img)
        with tempfile.TemporaryDirectory() as d:
            w = RecordingWriter(d)
            for i in range(40):
                w.add_frame("elp_right", 500.0 + i / 60.0, enc.tobytes())
            w.close()
            r = replay(d, CFG, quiet=True)
        self.assertEqual(r["dodges"], 0)
        self.assertEqual(r["stats"]["elp_right"]["with_ball"], 0)


class ExtrinsicsTests(unittest.TestCase):
    def test_recovers_a_mounting_error(self):
        from calibrate_extrinsics import solve
        cams = build_cameras(CFG)
        ref, elp = cams["fisheye"], cams["elp_left"]
        extr = dict(CFG["cameras"]["elp_left"]["extrinsics"])
        true = np.array([extr["yaw_deg"] + 3.0, extr["pitch_deg"] + 2.0, extr["roll_deg"] - 1.5])
        R_true = rotation_base_from_camera(*true)
        rng = np.random.default_rng(0)
        pairs = []
        for _ in range(150):
            az = math.radians(rng.uniform(65, 100))                   # the zone both cameras see
            dist = rng.uniform(1.0, 3.0)
            p = np.array([dist * math.cos(az), dist * math.sin(az), rng.uniform(-0.1, 1.2)])
            ray_f = (p - ref.t) / np.linalg.norm(p - ref.t)
            ray_c = R_true.T @ (p - elp.t)
            rng_e = float(np.linalg.norm(ray_c))
            ray_c = ray_c / rng_e + rng.normal(0, 0.002, 3)           # a little pixel noise
            pairs.append((ray_f, float(np.linalg.norm(p - ref.t)) * rng.uniform(0.92, 1.08),
                          ray_c / np.linalg.norm(ray_c), rng_e * rng.uniform(0.95, 1.05)))
        angles, before, after = solve(pairs, extr, ref.t)
        self.assertLess(np.abs(angles - true).max(), 0.6)
        self.assertLess(after, before)


if __name__ == "__main__":
    unittest.main(verbosity=2)
