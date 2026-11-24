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


class LensRimTests(unittest.TestCase):
    """The fisheye's rim is ignored: its colour fringe passes for the ball."""

    def setUp(self):
        self.cam = build_cameras(CFG)["fisheye"]
        self.circle = CFG["cameras"]["fisheye"]["image_circle"]

    def frame_with_patches(self, radii_frac):
        from test_dodge import BALL_BGR
        m = self.cam.model
        img = np.full((m.height, m.width, 3), 110, dtype=np.uint8)
        centres = []
        for i, frac in enumerate(radii_frac):
            ang = 2.3 + 1.1 * i
            c = (int(self.circle["cx"] + frac * self.circle["r"] * math.cos(ang)),
                 int(self.circle["cy"] + frac * self.circle["r"] * math.sin(ang)))
            cv2.circle(img, c, 5, BALL_BGR, -1)
            centres.append(c)
        return img, centres

    def test_patches_at_the_rim_are_ignored_and_others_kept(self):
        from dodge.detect import BallDetector
        img, centres = self.frame_with_patches([0.5, 0.80, 0.93, 0.97, 1.0])
        blobs = BallDetector.for_camera(self.cam.cfg, CFG["detector"]).detect(img)
        found = sorted((round(b.u), round(b.v)) for b in blobs)
        self.assertEqual(found, sorted(centres[:2]))
        # without the circle, all five are there
        plain = BallDetector(self.cam.cfg["color"], CFG["detector"]).detect(img)
        self.assertEqual(len(plain), 5)

    def test_mask_follows_the_frame_size(self):
        from dodge.detect import BallDetector
        img, centres = self.frame_with_patches([0.5, 0.97])
        half = cv2.resize(img, None, fx=0.5, fy=0.5, interpolation=cv2.INTER_NEAREST)
        det = BallDetector.for_camera(self.cam.cfg, dict(CFG["detector"], min_area_px=4))
        blobs = det.detect(half)
        self.assertEqual(len(blobs), 1)
        self.assertLess(abs(blobs[0].u - centres[0][0] / 2), 1.5)

    def test_measurement_outside_the_disc_is_refused(self):
        from dodge.detect import Blob
        from dodge.measure import measure_blob
        c = self.circle
        inside = Blob(c["cx"] + 0.5 * c["r"], c["cy"], 80.0, 0, 0, 10, 10, False)
        rim = Blob(c["cx"] - 0.95 * c["r"], c["cy"], 80.0, 0, 0, 10, 10, False)
        self.assertIsNotNone(measure_blob(self.cam, inside, 0.074))
        self.assertIsNone(measure_blob(self.cam, rim, 0.074))

    def test_rear_cameras_have_no_rim(self):
        from dodge.detect import BallDetector
        cam = build_cameras(CFG)["elp_left"]
        det = BallDetector.for_camera(cam.cfg, CFG["detector"])
        self.assertIsNone(det.ignore_for(1280, 720))

    def test_the_lit_disc_is_found_despite_glow_flare_and_a_dark_corner(self):
        from fit_image_circle import find_image_circle
        cx, cy, r = 712, 540, 388
        tex = cv2.GaussianBlur(np.random.default_rng(1).integers(60, 220, (1050, 1400, 3), dtype=np.uint8), (0, 0), 6)
        disc = np.zeros((1050, 1400), np.uint8)
        cv2.circle(disc, (cx, cy), r, 255, -1)
        glow = np.zeros((1050, 1400), np.uint8)
        cv2.circle(glow, (cx, cy), r + 40, 255, -1)
        img = np.zeros((1050, 1400, 3), np.uint8)
        img[(glow > 0) & (disc == 0)] = 35                                       # glow outside the rim
        img[disc > 0] = tex[disc > 0]
        cv2.ellipse(img, (cx, cy), (r + 5, r + 5), 0, 200, 250, (0, 0, 0), 120)  # a dark part of the scene
        cv2.circle(img, (cx + 60, cy - r - 10), 45, (200, 200, 160), -1)         # lens flare past the rim
        got = find_image_circle([cv2.GaussianBlur(img, (0, 0), 2)])
        self.assertIsNotNone(got)
        self.assertLess(abs(got[0] - cx), 3)
        self.assertLess(abs(got[1] - cy), 3)
        self.assertLess(abs(got[2] - r), 4)
        self.assertIsNone(find_image_circle([np.zeros((1050, 1400, 3), np.uint8)]))


class UsbCameraHelperTests(unittest.TestCase):
    def setUp(self):
        sys.path.insert(0, os.path.join(HERE, "..", "robot"))
        import usb_camera
        self.usb = usb_camera

    def test_control_list_is_parsed_with_ranges(self):
        text = """
User Controls

                     brightness 0x00980900 (int)    : min=-64 max=64 step=1 default=0 value=0
                           gain 0x00980913 (int)    : min=0 max=100 step=1 default=0 value=12
      white_balance_temperature 0x0098091a (int)    : min=2800 max=6500 step=1 default=4600 value=4600 flags=inactive

Camera Controls

                  auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=3 value=1 (Manual Mode)
         exposure_time_absolute 0x009a0902 (int)    : min=1 max=5000 step=1 default=157 value=30
"""
        import subprocess
        real_run, real_which = subprocess.run, self.usb.shutil.which

        class Done:
            stdout = text
            returncode = 0
        subprocess.run = lambda *a, **k: Done()
        self.usb.shutil.which = lambda _n: "/usr/bin/v4l2-ctl"
        try:
            c = self.usb.read_controls("/dev/video0")
        finally:
            subprocess.run, self.usb.shutil.which = real_run, real_which
        self.assertEqual(c["gain"], {"inactive": False, "min": 0, "max": 100, "step": 1, "default": 0, "value": 12})
        self.assertEqual((c["exposure_time_absolute"]["max"], c["exposure_time_absolute"]["value"]), (5000, 30))
        self.assertEqual(c["brightness"]["min"], -64)
        self.assertTrue(c["white_balance_temperature"]["inactive"])
        self.assertEqual(self.usb.control_name(c, "exposure_auto", "auto_exposure"), "auto_exposure")

    def test_each_camera_is_listed_once_although_the_pi_names_it_twice(self):
        with tempfile.TemporaryDirectory() as tmp:
            for n in ("video0", "video2"):
                open(os.path.join(tmp, n), "w").close()
            links = {"platform-xhci-hcd.0-usb-0:1:1.0-video-index0": "video0",
                     "platform-xhci-hcd.0-usbv2-0:1:1.0-video-index0": "video0",
                     "platform-xhci-hcd.1-usb-0:2:1.0-video-index0": "video2",
                     "platform-xhci-hcd.1-usbv2-0:2:1.0-video-index0": "video2",
                     "platform-xhci-hcd.1-usb-0:2:1.0-video-index1": "video2",
                     "platform-1000880000.pisp_be-video-index0": "video0"}
            for name, target in links.items():
                os.symlink(os.path.join(tmp, target), os.path.join(tmp, name))
            real_glob = self.usb.glob.glob
            self.usb.glob.glob = lambda pattern: real_glob(os.path.join(tmp, "*")) if "by-path" in pattern else real_glob(pattern)
            try:
                cams = self.usb.usb_camera_nodes()
            finally:
                self.usb.glob.glob = real_glob
        self.assertEqual([os.path.basename(c["path"]) for c in cams],
                         ["platform-xhci-hcd.0-usb-0:1:1.0-video-index0", "platform-xhci-hcd.1-usb-0:2:1.0-video-index0"])
        self.assertEqual([(c["controller"], c["port"]) for c in cams], [("xhci-hcd.0", "1"), ("xhci-hcd.1", "2")])

    def test_a_missing_camera_is_explained(self):
        msg = self.usb.open_problem("/dev/v4l/by-path/platform-xhci-hcd.0-usb-0:2:1.0-video-index0")
        self.assertIn("no camera on that USB socket", msg)
        self.assertIn("find_cameras.py", msg)
        self.assertIn("--assign", self.usb.open_problem(""))
        cap, raw = self.usb.open_capture("/dev/v4l/by-path/nothing-here", {})
        self.assertIsNone(cap)

    def test_camera_thread_reports_why_and_stops_promptly(self):
        import time as _time
        said = []
        cam = self.usb.UsbCamera("elp_right", {"device": "/dev/v4l/by-path/nothing-here"}, lambda *_a: None, log=said.append)
        cam.start()
        _time.sleep(0.4)
        t0 = _time.time()
        self.assertTrue(cam.close(3.0))
        self.assertLess(_time.time() - t0, 1.0)
        self.assertEqual(sum("cannot open the camera" in s for s in said), 1)
        self.assertIn("no camera on that USB socket", cam.problem)
