"""Off-robot tests for the recording format, replay and the extrinsics solver.

    python3 -m unittest discover -s tests -v
"""
from __future__ import annotations

import json
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
            save_calib("elp_left_color", {"cameras": {"elp_left": {"color": {"h_min": 41, "fit": 2}}}}, calib_dir=d)
            save_calib("elp_left_color", {"cameras": {"elp_left": {"size_scale": 0.97}}}, calib_dir=d)
            cfg = load_config(calib_dir=d)
            self.assertEqual(cfg["cameras"]["elp_left"]["color"]["h_min"], 41)
            self.assertEqual(cfg["cameras"]["elp_left"]["size_scale"], 0.97)
            self.assertEqual(cfg["cameras"]["elp_left"]["color"]["h_max"], CFG["cameras"]["elp_left"]["color"]["h_max"])
            self.assertEqual(cfg["cameras"]["elp_right"]["color"], CFG["cameras"]["elp_right"]["color"])

    def test_colour_limits_fitted_the_old_way_are_ignored(self):
        with tempfile.TemporaryDirectory() as d:
            save_calib("elp_left_color", {"cameras": {"elp_left": {"color": {"h_min": 41}, "size_scale": 0.9}}}, calib_dir=d)
            cfg = load_config(calib_dir=d)
            self.assertEqual(cfg["cameras"]["elp_left"]["color"], CFG["cameras"]["elp_left"]["color"])
            self.assertEqual(cfg["cameras"]["elp_left"]["size_scale"], 0.9)

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



class StepResponseTests(unittest.TestCase):
    """step_response: Pupper's IMU sends no acceleration, so the tool must not invent a distance or a lag."""

    def test_an_imu_that_sends_zeros_for_acceleration_is_recognised(self):
        import step_response as sr
        self.assertFalse(sr.has_acceleration(np.zeros((400, 3))))
        self.assertTrue(sr.has_acceleration(np.array([[0.0, 0.0, 9.8], [0.1, 0.0, 9.7]])))

    def test_the_reaction_delay_is_read_from_the_turn_rate(self):
        import step_response as sr
        rng = np.random.default_rng(3)
        t = np.arange(0.0, 4.0, 0.004)
        t_cmd, reacts = 2.0, 0.09
        gyro = rng.normal(0, 0.03, (len(t), 3))                       # standing: small wobble
        gyro[t >= t_cmd + reacts, 1] += 1.2                           # the body pitches as the legs push off
        delay, _ = sr.reaction_delay(t, gyro, t_cmd)
        self.assertAlmostEqual(delay, reacts, delta=0.012)
        still, _ = sr.reaction_delay(t, rng.normal(0, 0.03, (len(t), 3)), t_cmd)
        self.assertTrue(math.isnan(still))                            # a robot that never moved gives no delay

    def test_the_lag_is_fitted_to_timings_from_a_video(self):
        import step_response as sr
        from dodge.escape import lag_time
        speed = math.hypot(1.0, 0.5)
        points = [(d, lag_time(d, speed, 0.25)) for d in (0.15, 0.30)]
        lag, err, problem = sr.fit_lag(points, speed)
        self.assertIsNone(problem)
        self.assertAlmostEqual(lag, 0.25, delta=0.011)
        self.assertLess(err, 0.01)
        self.assertIsNotNone(sr.fit_lag([(0.30, 0.10)], speed)[2])    # 3 m/s: faster than commanded
        self.assertIsNotNone(sr.fit_lag([(0.15, 1.5)], speed)[2])     # slower than any sensible lag
        self.assertEqual(sr.parse_timed(["0.15:0.31", "0.30:0.47"]), [(0.15, 0.31), (0.30, 0.47)])
        with self.assertRaises(SystemExit):
            sr.parse_timed(["0.15"])

    def test_a_saved_lag_that_is_not_a_measurement_is_ignored(self):
        with tempfile.TemporaryDirectory() as d:
            save_calib("step_response", {"escape": {"accel_lag_s": 0.79}}, calib_dir=d)
            self.assertAlmostEqual(load_config(calib_dir=d)["escape"]["accel_lag_s"], 0.20)
            save_calib("step_response", {"escape": {"accel_lag_s": 0.24}}, calib_dir=d)
            self.assertAlmostEqual(load_config(calib_dir=d)["escape"]["accel_lag_s"], 0.24)



def make_throw_recording(folder, kind="dodge", t0=1000.0):
    """A saved throw as the dodge node writes it: a ball first seen 100 ms before the decision, out of
    sight (off the top of the picture) from 100 ms after it, a command, the mux passing it on 15 ms
    later and the body turning 110 ms after the command."""
    from dodge.recording import RecordingWriter
    event = {"t": t0, "event": "dodge", "dry_run": kind == "dry", "late": False,
             "threat": {"track": 7, "threat": True, "tau": 0.5, "miss_m": 0.02, "bearing_deg": 95.0},
             "plan": {"vx": 1.0, "vy": 0.0, "direction": [1.0, 0.0], "t_need": 0.45, "duration": 0.5, "reason": "run forward, across the ball's path"}}
    w = RecordingWriter(folder, {"kind": kind, "event": event})
    ok, jpeg = cv2.imencode(".jpg", np.full((36, 64, 3), 90, np.uint8))
    first = t0 - 0.10
    seen = first
    for i in range(int(1.3 * 60)):
        t = t0 - 0.5 + i / 60.0
        w.add_frame("elp_left", t, jpeg.tobytes())
        rec = {"k": "f", "cam": "elp_left", "t": round(t, 4), "raw": round(t, 4), "done": round(t + 0.03, 4), "ms": 9.0, "size": [1280, 720]}
        if first - 1e-6 <= t:
            visible = t <= t0 + 0.10
            if visible:
                seen = t
                rec["blobs"] = [[640.0, 300.0 - 2800.0 * (t - first), 20, 20]]      # climbing toward the top edge
            hits = int(round((seen - first) * 60)) + 1
            rec["core"] = {"state": "TRACK" if t < t0 else "DODGE",
                           "tracks": [{"id": 7, "t0": round(first, 4), "hits": hits, "confirmed": hits >= 3, "flying": hits >= 3,
                                       "p": [0.0, 2.0 - 3.0 * (t - first), 0.5], "v": [0.0, -3.0, 2.0], "t": round(t, 4),
                                       "seen": round(seen, 4), "cam": "elp_left"}],
                           "threats": [{"track": 7, "threat": hits >= 4, "tau": 0.5}]}
        w.add_trace(rec)
    if kind == "dodge":
        for i in range(50):
            w.add_trace({"k": "c", "t": round(t0 + 0.002 + i * 0.01, 4), "vx": 1.0, "vy": 0.0})
            w.add_trace({"k": "m", "t": round(t0 + 0.017 + i * 0.01, 4), "vx": 1.0, "vy": 0.0})
    rng = np.random.default_rng(1)
    for i in range(int(2.0 * 200)):
        t = t0 - 1.0 + i / 200.0
        turning = kind == "dodge" and t >= t0 + 0.112
        w.add_imu(t, (1.0, 0.0, 0.0, 0.0), (rng.normal(0, 0.02), (1.2 if turning else 0.0) + rng.normal(0, 0.02), rng.normal(0, 0.02)))
    w.add_event(event)
    w.close()
    return event


class ThrowLogTests(unittest.TestCase):
    """The throw recorder's numbers: when the ball was seen, the decision, the command, the body moving."""

    def test_the_timeline_of_a_dodge(self):
        from dodge import throwlog
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "20261126-120000-dodge")
            make_throw_recording(path)
            t = throwlog.analyse(path)
            n = t["numbers"]
            self.assertEqual(t["kind"], "dodge")
            self.assertFalse(t["late"])
            self.assertEqual(t["timing"], "ok")
            self.assertAlmostEqual(n["seen_to_decision_ms"], 100, delta=2)
            self.assertAlmostEqual(n["first_picture_delay_ms"], 30, delta=2)
            self.assertAlmostEqual(n["decision_to_command_ms"], 2, delta=1)
            self.assertAlmostEqual(n["command_to_mux_ms"], 15, delta=1)
            self.assertAlmostEqual(n["command_to_body_ms"], 110, delta=12)
            self.assertAlmostEqual(n["decision_to_body_ms"], 112, delta=12)
            self.assertAlmostEqual(n["seen_to_body_ms"], 212, delta=12)
            self.assertEqual(n["time_left_at_decision_ms"], 500)
            self.assertEqual(n["needed_ms"], 450)
            gap = t["gaps"][0]                                 # out of sight from 100 ms after the decision until it was due
            self.assertAlmostEqual(gap["duration"], 0.4, delta=0.03)
            self.assertEqual(gap["where"], "the top of the picture")
            self.assertTrue(gap["to_the_end"])
            self.assertTrue(any("top of the picture" in line for line in t["story"]))
            self.assertEqual(t["cameras"]["elp_left"]["fps"], 60.0)
            self.assertAlmostEqual(t["cameras"]["elp_left"]["age_ms"], 30, delta=1)
            self.assertTrue(os.path.exists(os.path.join(path, "throw.json")))

    def test_a_dry_run_has_no_robot_latency_and_old_recordings_say_late_in_words(self):
        from dodge import throwlog
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "20261126-120001-dry")
            make_throw_recording(path, kind="dry")
            with open(os.path.join(path, "meta.json")) as f:
                meta = json.load(f)
            meta["event"]["late"] = "False"                    # how the node used to write it
            with open(os.path.join(path, "meta.json"), "w") as f:
                json.dump(meta, f)
            t = throwlog.analyse(path)
            self.assertEqual(t["kind"], "dry")
            self.assertFalse(t["late"])
            self.assertIsNone(t["numbers"]["decision_to_body_ms"])
            self.assertEqual(t["numbers"]["seen_to_decision_ms"], 100)
            meta["event"].update(late=True, threat=dict(meta["event"]["threat"], tau=0.2))      # 200 ms left, 450 needed
            with open(os.path.join(path, "meta.json"), "w") as f:
                json.dump(meta, f)
            self.assertEqual(throwlog.analyse(path)["timing"], "late")
            meta["event"]["threat"]["tau"] = 0.42                                             # 420 of 450: close
            with open(os.path.join(path, "meta.json"), "w") as f:
                json.dump(meta, f)
            self.assertEqual(throwlog.analyse(path)["timing"], "tight")

    def test_videos_labels_and_the_summary_file(self):
        import csv as csv_mod
        from dodge import throwlog
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "20261126-120002-dodge")
            make_throw_recording(path)
            t = throwlog.analyse(path)
            made = throwlog.render_videos(path, t)
            self.assertEqual(made, {"elp_left.mp4": True, "elp_left_detections.mp4": True})
            throwlog.save_label(path, "dodged", "from the sofa")
            self.assertEqual(throwlog.load_label(path)["result"], "dodged")
            out = throwlog.write_summary(d)
            with open(out) as f:
                rows = list(csv_mod.DictReader(f))
            self.assertEqual(len(rows), 1)
            self.assertEqual(rows[0]["result"], "dodged")
            self.assertEqual(rows[0]["seen_to_decision_ms"], "100")
            self.assertEqual(rows[0]["move"], "run forward")

    def test_a_recording_without_a_live_trace_is_worked_out_from_its_pictures(self):
        from dodge import throwlog
        from dodge.recording import RecordingWriter
        cams = build_cameras(CFG)
        throw = aimed_throw(CFG, 100.0, 2.0, rise=0.5)
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "20261126-120003")
            w = RecordingWriter(path, {})
            for i in range(14):
                t = i / 60.0
                img, _ok = render_ball(cams["elp_left"], throw.position(t))
                w.add_frame("elp_left", 2000.0 + t, cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])[1].tobytes())
                w.add_imu(2000.0 + t, (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            w.close()
            t = throwlog.analyse(path, cfg=CFG)
            self.assertTrue(t["rebuilt"])
            self.assertIsNotNone(t["track"])
            self.assertGreaterEqual(t["cameras"]["elp_left"]["frames_with_ball"], 8)
            self.assertAlmostEqual(t["ball"]["first"]["bearing_deg"], 100, delta=6)

    def test_the_dodging_view_lists_throws_and_serves_their_pictures(self):
        import dodging_view
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "20261126-120004-dodge")
            make_throw_recording(path)
            lab = dodging_view.Lab([d], make_videos=False)
            self.assertFalse(lab.listing()[0]["ready"])        # not analysed yet
            dodging_view.throwlog.analyse(path)
            row = lab.listing()[0]
            self.assertTrue(row["ready"])
            self.assertEqual(row["move"], "run forward")
            self.assertEqual(row["from_deg"], 95.0)
            frames = lab.frame_index(row["id"], path, lab.throw(row["id"], path))["elp_left"]
            self.assertEqual(len(frames["t"]), 78)
            self.assertEqual(sum(1 for b in frames["ball"] if b >= 0), 13)
            self.assertEqual(lab.jpeg(path, "elp_left", 3)[:2], b"\xff\xd8")
            self.assertIsNone(lab.jpeg(path, "elp_left", 999))
            self.assertIsNone(lab.path("../" + row["id"]))    # only folders of saved throws can be asked for


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


class BallColourAndShapeTests(unittest.TestCase):
    """What the first pictures from the real cameras showed the detector has to cope with."""

    BALL = (60, 235, 140)             # neon green, BGR
    DIM_GREENISH = (25, 55, 35)       # a dark floor under a greenish light: very "saturated", hardly coloured at all
    WEAK_GREEN = (82, 122, 92)        # ball hue, but only just colourful enough

    def setUp(self):
        from dodge.detect import BallDetector
        self.BallDetector = BallDetector
        self.cam = CFG["cameras"]["elp_left"]
        self.det = BallDetector.for_camera(self.cam, CFG["detector"])

    def room(self):
        rng = np.random.default_rng(5)
        return np.clip(rng.normal(95, 6, (720, 1280, 3)), 0, 255).astype(np.uint8)

    def test_a_ball_with_dimples_and_a_ragged_edge_is_found_whole(self):
        img = self.room()
        cv2.circle(img, (640, 360), 30, self.BALL, -1)
        rng = np.random.default_rng(1)
        for ang in np.linspace(0, 2 * math.pi, 60, endpoint=False):      # bites out of the edge, as noise makes them
            r = 30 - rng.integers(0, 4)
            c = (int(640 + r * math.cos(ang)), int(360 + r * math.sin(ang)))
            cv2.circle(img, c, int(rng.integers(1, 3)), (95, 95, 95), -1)
        for dx, dy in ((-10, -8), (8, -12), (0, 6), (12, 10), (-14, 9)):  # dimples
            cv2.circle(img, (640 + dx, 360 + dy), 3, (40, 90, 60), -1)
        blobs = self.det.detect(img)
        self.assertEqual(len(blobs), 1)
        self.assertLess(abs(blobs[0].u - 640), 1.5)
        self.assertLess(abs(blobs[0].v - 360), 1.5)
        self.assertLess(abs(blobs[0].diameter_px - 60) / 60, 0.07)      # the dimples do not shrink it

    def test_a_dim_floor_with_a_colour_cast_is_not_the_ball(self):
        img = self.room()
        for i in range(12):
            cv2.circle(img, (100 + 95 * i, 600), 12, self.DIM_GREENISH, -1)
        self.assertEqual(self.det.detect(img), [])
        by_saturation_only = dict(self.cam["color"], c_min=0, c_seed=0)
        self.assertGreaterEqual(len(self.BallDetector(by_saturation_only, CFG["detector"]).detect(img)), 10)

    def test_a_far_ball_is_not_crowded_out_by_bigger_patches(self):
        img = self.room()
        for i in range(30):                                              # ball-coloured, but L-shaped: not balls
            x, y = 40 + 41 * i, 80 + (i % 3) * 190
            cv2.rectangle(img, (x, y), (x + 34, y + 9), self.BALL, -1)
            cv2.rectangle(img, (x, y), (x + 9, y + 34), self.BALL, -1)
        cv2.circle(img, (900, 650), 7, self.BALL, -1)
        blobs = self.det.detect(img)
        self.assertEqual(len(blobs), 1)
        self.assertLess(math.hypot(blobs[0].u - 900, blobs[0].v - 650), 1.5)

    def test_a_patch_needs_some_strongly_coloured_pixels(self):
        img = self.room()
        cv2.circle(img, (400, 300), 20, self.WEAK_GREEN, -1)
        self.assertEqual(self.det.detect(img), [])
        no_higher_bar = dict(self.cam["color"], c_seed=0)
        self.assertEqual(len(self.BallDetector(no_higher_bar, CFG["detector"]).detect(img)), 1)

    def test_patches_too_small_to_be_the_ball_at_throwing_range_are_ignored(self):
        img = self.room()
        cv2.circle(img, (500, 300), 2, self.BALL, -1)                    # 5 px across: a ball 5 m away
        cv2.circle(img, (800, 300), 5, self.BALL, -1)                    # 11 px: a ball 2.5 m away
        found = [(round(b.u), round(b.v)) for b in self.det.detect(img)]
        self.assertEqual(found, [(800, 300)])
        any_range = self.BallDetector.for_camera(self.cam, dict(CFG["detector"], max_range_m=0))
        self.assertEqual(len(any_range.detect(img)), 2)

    def test_colour_limits_are_fitted_to_leave_the_empty_room_empty(self):
        from dodge.colorfit import fit_color
        rng = np.random.default_rng(2)
        n = 4000
        ball = np.c_[rng.normal(50, 2.5, n), rng.normal(135, 18, n), rng.normal(150, 35, n)].clip(1, 255).astype(np.uint8)
        empty = self.room()
        cv2.circle(empty, (300, 300), 25, (78, 126, 92), -1)             # a wall patch: the ball's hue, weaker colour
        cv2.circle(empty, (700, 500), 25, (62, 150, 145), -1)            # a yellow thing: strong colour, neighbouring hue
        det_cfg = dict(CFG["detector"], max_blobs=50)

        def judge(color):
            det = self.BallDetector(color, det_cfg)
            return len(det.detect(empty)), det.cover(empty)
        color, report = fit_color(ball, judge)
        self.assertEqual(report["false"], 0)
        self.assertGreater(report["kept"], 0.8)
        self.assertGreater(color["c_seed"], color["c_min"])
        picture = empty.copy()
        cv2.circle(picture, (1000, 200), 14, (int(255 * 0.47 * 0.59), 150, int(150 * 0.72)), -1)   # the ball itself
        got = self.BallDetector(color, det_cfg).detect(picture)
        self.assertEqual(len(got), 1)
        self.assertLess(math.hypot(got[0].u - 1000, got[0].v - 200), 2)


class ColourCastTests(unittest.TestCase):
    """The same room and ball under different white balances must look the same to the detector."""

    def scene(self, cast=(1.0, 1.0, 1.0)):
        rng = np.random.default_rng(4)
        img = np.clip(rng.normal(150, 5, (720, 1280, 3)), 0, 255)       # white walls
        img[400:] = np.clip(rng.normal(60, 5, (320, 1280, 3)), 0, 255) * np.array([0.55, 0.8, 1.0])   # brown floor
        cv2.circle(img, (640, 300), 20, (60, 200, 130), -1)             # the ball
        cv2.rectangle(img, (200, 150), (260, 210), (40, 60, 200), -1)   # something red
        return np.clip(img * np.array(cast), 0, 255).astype(np.uint8)

    def test_the_ball_is_found_under_any_cast_and_the_walls_never_are(self):
        from dodge.detect import BallDetector
        for cast in ((1.0, 1.0, 1.0), (0.45, 1.0, 1.2), (1.3, 1.0, 0.75), (0.8, 1.0, 0.8), (0.5, 1.0, 0.85)):   # none, yellow, blue, green, yellow-green
            det = BallDetector.for_camera(CFG["cameras"]["elp_left"], CFG["detector"])
            blobs = det.detect(self.scene(cast))
            self.assertEqual(len(blobs), 1, "cast %s" % (cast,))
            self.assertLess(math.hypot(blobs[0].u - 640, blobs[0].v - 300), 2, "cast %s" % (cast,))
            self.assertLess(abs(blobs[0].diameter_px - 40) / 40, 0.1, "cast %s" % (cast,))

    def test_without_the_correction_a_yellow_green_cast_breaks_it(self):
        from dodge.detect import BallDetector
        det = BallDetector.for_camera(CFG["cameras"]["elp_left"], dict(CFG["detector"], balance={"enabled": False}))
        blobs = det.detect(self.scene((0.5, 1.0, 0.85)))                # the white walls are now the ball's colour
        self.assertFalse(len(blobs) == 1 and math.hypot(blobs[0].u - 640, blobs[0].v - 300) < 2)

    def test_a_ball_filling_the_view_does_not_become_the_new_white(self):
        from dodge.detect import BallDetector
        det = BallDetector.for_camera(CFG["cameras"]["elp_left"], dict(CFG["detector"], max_area_frac=0.6))
        for _ in range(3):
            det.detect(self.scene())
        close = self.scene()
        cv2.circle(close, (640, 330), 250, (60, 200, 130), -1)
        for _ in range(60):                                             # a second of it
            blobs = det.detect(close)
        self.assertLess(det.balance.strength(), 1.15)
        self.assertEqual(len(blobs), 1)


class KnockTimingTests(unittest.TestCase):
    """measure_latency: a knock moves the picture by a pixel or less, in a grainy picture."""

    def test_small_jolts_in_a_noisy_picture_are_matched_to_the_knocks(self):
        import measure_latency as ml
        rng = np.random.default_rng(7)
        scene = np.full((360, 640, 3), 110, np.uint8)                 # a room: flat areas with edges between them
        for _ in range(40):
            x, y = int(rng.integers(0, 600)), int(rng.integers(0, 330))
            cv2.rectangle(scene, (x, y), (x + int(rng.integers(20, 120)), y + int(rng.integers(20, 90))), int(rng.integers(40, 200)), -1)
        scene = cv2.GaussianBlur(scene, (0, 0), 1.5)
        fps, late = 30.0, 0.040                       # the camera stamps its frames 40 ms late
        knocks = [1.513, 3.504, 5.521, 7.509]
        meter = ml.JoltMeter()
        frame_t, frame_moved = [], []
        for i in range(int(9 * fps)):
            t = i / fps
            since = min([t - k for k in knocks if t >= k] or [9.0])
            # the camera is displaced for a few frames after a knock, less each frame: a pixel at most
            dx = 1.0 * math.exp(-since / 0.04) * (1 if int(since * fps) % 2 == 0 else -1) if since < 0.2 else 0.0
            shifted = cv2.warpAffine(scene, np.float32([[1, 0, dx], [0, 1, 0.5 * dx]]), (640, 360), flags=cv2.INTER_LINEAR,
                                     borderMode=cv2.BORDER_REFLECT)
            noisy = np.clip(shifted.astype(np.float32) + rng.normal(0, 9, shifted.shape), 0, 255).astype(np.uint8)
            moved = meter.add(noisy)
            if moved is not None:
                frame_t.append(t + late)
                frame_moved.append(moved)
        imu_t = np.arange(0, 9, 0.005)
        imu_rate = 0.02 + 0.01 * rng.random(len(imu_t))
        for k in knocks:
            imu_rate[(imu_t >= k) & (imu_t < k + 0.03)] = 1.5
        found, errors, threshold, biggest, period = ml.match_knocks(imu_t, imu_rate, frame_t, frame_moved)
        self.assertEqual(len(found), 4)
        self.assertEqual(len(errors), 4, "jolts %s against a threshold of %.2f px" % (biggest, threshold))
        self.assertLess(abs(float(np.median(errors)) - late), 0.02)
        self.assertLess(float(np.median(frame_moved)), 0.05)         # a still camera reads as still despite the noise
