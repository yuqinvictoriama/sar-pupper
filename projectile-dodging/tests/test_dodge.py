"""Off-robot tests for the dodge core. No ROS, no cameras.

    python3 -m unittest discover -s tests -v        (from the projectile-dodging folder)
"""
from __future__ import annotations

import math
import os
import sys
import unittest

import cv2
import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from dodge.camera_models import build_cameras, rotation_base_from_camera  # noqa: E402
from dodge.config import load_config  # noqa: E402
from dodge.core import DodgeCore, State, quat_to_rot  # noqa: E402
from dodge.detect import BallDetector  # noqa: E402
from dodge.escape import EscapePlanner, FreeSpace, lag_distance, lag_time  # noqa: E402
from dodge.measure import measure_blob  # noqa: E402
from dodge.sim import aimed_throw, run_closed_loop  # noqa: E402
from dodge.threat import Threat  # noqa: E402

CFG = load_config(use_calib=False)
BALL_BGR = (60, 235, 140)     # neon green


def render_ball(camera, p_base, bg=110):
    """Synthetic frame with the ball drawn exactly as this lens would see it."""
    m = camera.model
    img = np.full((m.height, m.width, 3), bg, dtype=np.uint8)
    uv, ok, p_cam = camera.project_base(np.asarray(p_base, dtype=float)[None, :])
    if not ok[0]:
        return img, False
    rng_m = float(np.linalg.norm(p_cam[0]))
    ang_r = math.asin(0.037 / rng_m)
    half = int(max(8, 4 * ang_r / math.sqrt(m.solid_angle_per_px(uv[0, 0], uv[0, 1])))) + 4
    u0, v0 = int(round(uv[0, 0])), int(round(uv[0, 1]))
    us = np.arange(max(0, u0 - half), min(m.width, u0 + half + 1))
    vs = np.arange(max(0, v0 - half), min(m.height, v0 + half + 1))
    ss = 4   # supersample for a soft edge
    acc = np.zeros((len(vs), len(us)))
    centre = p_cam[0] / rng_m
    for dy in range(ss):
        for dx in range(ss):
            gu, gv = np.meshgrid(us + (dx + 0.5) / ss - 0.5, vs + (dy + 0.5) / ss - 0.5)
            rays, valid = m.unproject(np.stack([gu.ravel(), gv.ravel()], axis=1))
            inside = (rays @ centre >= math.cos(ang_r)) & valid
            acc += inside.reshape(len(vs), len(us))
    cover = (acc / (ss * ss))[:, :, None]
    patch = img[vs[0]:vs[-1] + 1, us[0]:us[-1] + 1].astype(float)
    img[vs[0]:vs[-1] + 1, us[0]:us[-1] + 1] = (patch * (1 - cover) + np.array(BALL_BGR) * cover).astype(np.uint8)
    return img, True


class CameraModelTests(unittest.TestCase):
    def setUp(self):
        self.cams = build_cameras(CFG)

    def test_round_trip(self):
        for cam in self.cams.values():
            m = cam.model
            uv = np.array([[m.width * 0.5, m.height * 0.5], [m.width * 0.3, m.height * 0.35],
                           [m.width * 0.62, m.height * 0.7], [m.width * 0.45, m.height * 0.2]])
            rays, ok = m.unproject(uv)
            self.assertTrue(ok.all(), cam.name)
            back, ok2 = m.project(rays * 2.5)
            self.assertTrue(ok2.all(), cam.name)
            self.assertLess(np.abs(back - uv).max(), 1e-6, cam.name)

    def test_mounting_directions(self):
        # fisheye looks forward; the ELPs look backward and outward, each on its own side
        self.assertTrue(np.allclose(self.cams["fisheye"].R @ [0, 0, 1], [1, 0, 0], atol=1e-9))
        left = self.cams["elp_left"].R @ [0, 0, 1]
        right = self.cams["elp_right"].R @ [0, 0, 1]
        self.assertLess(left[0], 0)
        self.assertGreater(left[1], 0)
        self.assertLess(right[0], 0)
        self.assertLess(right[1], 0)
        for cam in self.cams.values():                      # image "down" is toward the floor
            self.assertTrue(np.allclose(cam.R @ [0, 1, 0], [0, 0, -1], atol=1e-9), cam.name)
        up = rotation_base_from_camera(0.0, pitch_deg=10.0) @ [0, 0, 1]
        self.assertGreater(up[2], 0)                        # positive pitch tilts the lens up

    def test_full_coverage(self):
        # every bearing around the robot is inside at least one camera's usable field of view
        for az in range(-180, 180, 5):
            p = np.array([3 * math.cos(math.radians(az)), 3 * math.sin(math.radians(az)), 0.6])
            seen = 0
            for cam in self.cams.values():
                uv, ok, p_cam = cam.project_base(p[None, :])
                off = math.degrees(math.acos(p_cam[0, 2] / np.linalg.norm(p_cam[0])))
                seen += int(ok[0] and off <= cam.cfg["max_angle_deg"])
            self.assertGreaterEqual(seen, 1, "bearing %d not covered" % az)


class DetectAndMeasureTests(unittest.TestCase):
    def setUp(self):
        self.cams = build_cameras(CFG)

    def check(self, name, p_base, tol):
        cam = self.cams[name]
        img, ok = render_ball(cam, p_base)
        self.assertTrue(ok)
        det = BallDetector(cam.cfg["color"], CFG["detector"])
        blobs = det.detect(img)
        self.assertEqual(len(blobs), 1, "%s at %s" % (name, p_base))
        m = measure_blob(cam, blobs[0], CFG["ball"]["diameter_m"])
        self.assertIsNotNone(m)
        true_range = float(np.linalg.norm(np.asarray(p_base) - cam.t))
        self.assertLess(abs(m.range_m - true_range) / true_range, tol, "%s range %.2f vs %.2f" % (name, m.range_m, true_range))
        err = np.linalg.norm(m.point - np.asarray(p_base))
        self.assertLess(err, tol * true_range + 0.03)
        return m

    def test_fisheye_range_and_bearing(self):
        self.check("fisheye", [1.2, 0.1, 0.3], 0.06)
        self.check("fisheye", [2.0, -1.0, 0.8], 0.10)
        self.check("fisheye", [0.8, 1.2, 0.5], 0.08)          # 56 degrees off axis

    def test_elp_range_and_bearing(self):
        self.check("elp_left", [-1.5, 1.2, 0.6], 0.05)
        self.check("elp_left", [-0.3, 2.5, 0.9], 0.06)         # near the stretched image edge
        self.check("elp_right", [-2.5, -1.5, 1.0], 0.06)
        self.check("elp_right", [-3.5, -0.4, 1.2], 0.08)

    def test_roi_matches_full_frame(self):
        cam = self.cams["elp_left"]
        img, _ = render_ball(cam, [-1.5, 1.2, 0.6])
        det = BallDetector(cam.cfg["color"], CFG["detector"])
        full = det.detect(img)[0]
        part = det.detect(img, roi=(full.u - 80, full.v - 80, full.u + 80, full.v + 80))[0]
        self.assertAlmostEqual(full.u, part.u, places=6)
        self.assertAlmostEqual(full.area, part.area, places=6)

    def test_coarse_search_finds_exactly_what_a_direct_search_finds(self):
        direct_cfg = dict(CFG["detector"], coarse_step=1)
        for name, seed in (("elp_left", 1), ("elp_right", 2), ("fisheye", 3)):
            cam = self.cams[name]
            w, h = cam.model.width, cam.model.height
            rng = np.random.default_rng(seed)
            img = np.full((h, w, 3), 110, dtype=np.float32)
            cv2.rectangle(img, (500, 200), (640, 480), BALL_BGR, -1)      # a neon shirt
            cv2.line(img, (100, 600), (400, 620), BALL_BGR, 6)            # a neon strap
            for r in (2.5, 3.0, 4.0, 5.5, 8.0, 13.0, 22.0, 45.0, 90.0):   # balls from far away to very close
                cx, cy = rng.uniform(0, w), rng.uniform(0, h)             # some run off the edge of the picture
                cv2.circle(img, (int(cx * 16), int(cy * 16)), int(r * 16), BALL_BGR, -1, cv2.LINE_AA, shift=4)
            for _ in range(300):                                          # single stray pixels of ball colour
                img[rng.integers(0, h), rng.integers(0, w)] = BALL_BGR
            img = np.clip(img + rng.normal(0, 5, img.shape), 0, 255).astype(np.uint8)
            for roi in (None, (150, 90, 1000, 650)):
                coarse = BallDetector.for_camera(cam.cfg, CFG["detector"]).detect(img, roi)
                direct = BallDetector.for_camera(cam.cfg, direct_cfg).detect(img, roi)
                self.assertGreaterEqual(len(direct), 3, name)
                key = lambda b: (b.x, b.y, b.w, b.h)
                coarse, direct = sorted(coarse, key=key), sorted(direct, key=key)
                self.assertEqual([(key(b), b.area, b.touches_edge) for b in coarse],
                                 [(key(b), b.area, b.touches_edge) for b in direct], name)
                for c, d in zip(coarse, direct):
                    self.assertAlmostEqual(c.u, d.u, places=9)
                    self.assertAlmostEqual(c.v, d.v, places=9)

    def test_hundreds_of_specks_do_not_hide_the_ball_or_swamp_the_search(self):
        cam = self.cams["elp_left"]
        img, _ = render_ball(cam, [-1.5, 1.2, 0.6])
        rng = np.random.default_rng(4)
        for _ in range(400):                                              # hundreds of small ball-coloured things
            x, y = int(rng.integers(0, cam.model.width - 6)), int(rng.integers(0, cam.model.height // 2))
            img[y:y + 5, x:x + 5] = BALL_BGR
        det = BallDetector.for_camera(cam.cfg, CFG["detector"])
        places = det._coarse_windows(img, 0, 0, cam.model.width, cam.model.height)
        self.assertIsNotNone(places)                                      # it does not give up ...
        self.assertLessEqual(len(places), det.max_places)                 # ... and looks only at the largest places
        direct = BallDetector.for_camera(cam.cfg, dict(CFG["detector"], coarse_step=1))
        ball, found = direct.detect(img)[0], det.detect(img)[0]           # the ball is the largest thing there
        self.assertEqual((found.x, found.y, found.area), (ball.x, ball.y, ball.area))

    def test_a_picture_mostly_ball_coloured_falls_back_to_a_direct_search(self):
        cam = self.cams["elp_left"]
        img = np.full((cam.model.height, cam.model.width, 3), 110, dtype=np.uint8)
        img[:, :900] = BALL_BGR
        det = BallDetector.for_camera(cam.cfg, CFG["detector"])
        self.assertIsNone(det._coarse_windows(img, 0, 0, cam.model.width, cam.model.height))
        direct = BallDetector.for_camera(cam.cfg, dict(CFG["detector"], coarse_step=1))
        self.assertEqual([(b.x, b.y, b.area) for b in det.detect(img)], [(b.x, b.y, b.area) for b in direct.detect(img)])

    def test_green_clothing_is_not_a_ball(self):
        cam = self.cams["elp_left"]
        img = np.full((cam.model.height, cam.model.width, 3), 110, dtype=np.uint8)
        cv2.rectangle(img, (500, 200), (640, 480), BALL_BGR, -1)          # a neon shirt: right colour, wrong shape
        cv2.line(img, (100, 600), (400, 620), BALL_BGR, 6)                # a neon strap
        self.assertEqual(BallDetector(cam.cfg["color"], CFG["detector"]).detect(img), [])

    def test_background_gives_nothing(self):
        cam = self.cams["fisheye"]
        img = np.random.default_rng(0).integers(60, 200, (cam.model.height, cam.model.width, 3), dtype=np.uint8)
        img = cv2.GaussianBlur(img, (0, 0), 3)   # grey-ish clutter, nothing saturated green
        self.assertEqual(BallDetector(cam.cfg["color"], CFG["detector"]).detect(img), [])


class EscapeTests(unittest.TestCase):
    def setUp(self):
        self.planner = EscapePlanner(CFG["escape"], CFG["robot"], CFG["ball"], CFG["threat"]["margin_m"])

    def threat(self, bearing_deg, lateral=0.0, tau=0.8):
        b = math.radians(bearing_deg)
        return Threat(1, True, True, "", tau, 0.0, 0.05, 0.05, lateral, 0.0,
                      np.array([-math.cos(b), -math.sin(b)]), bearing_deg, 50.0, 4.0, 2.0, 5.0)

    def test_lag_model(self):
        for d, v, tau in ((0.2, 0.5, 0.2), (0.3, 1.0, 0.15), (0.1, 0.9, 0.0)):
            self.assertAlmostEqual(lag_distance(lag_time(d, v, tau), v, tau), d, places=4)

    def test_side_ball_runs_fore_aft(self):
        plan = self.planner.plan(self.threat(90.0))          # ball from the left
        self.assertNotEqual(plan.vx, 0.0)
        self.assertEqual(plan.vy, 0.0)

    def test_front_ball_sidesteps(self):
        plan = self.planner.plan(self.threat(0.0))
        self.assertEqual(plan.vx, 0.0)
        self.assertAlmostEqual(abs(plan.vy), CFG["escape"]["vy"])

    def test_moves_away_from_where_the_ball_passes(self):
        # ball from ahead, passing on the robot's right side (its own left-of-travel is the robot's right)
        plan = self.planner.plan(self.threat(0.0, lateral=0.08))
        self.assertGreater(plan.vy, 0.0)                     # so step left
        plan = self.planner.plan(self.threat(0.0, lateral=-0.08))
        self.assertLess(plan.vy, 0.0)

    def test_diagonal_uses_both_axes_and_is_faster(self):
        plan = self.planner.plan(self.threat(45.0))
        self.assertNotEqual(plan.vx, 0.0)
        self.assertNotEqual(plan.vy, 0.0)
        self.assertLess(plan.t_clear, self.planner.plan(self.threat(0.0)).t_clear)

    def test_free_space_flips_dead_centre_choice(self):
        base = self.planner.plan(self.threat(0.0))
        sectors = [5.0] * 8
        blocked_side = 2 if base.vy > 0 else 6               # sector 2 = left, 6 = right
        sectors[blocked_side] = 0.2
        plan = self.planner.plan(self.threat(0.0), FreeSpace(sectors))
        self.assertEqual(np.sign(plan.vy), -np.sign(base.vy))


class ClosedLoopTests(unittest.TestCase):
    def run_throw(self, bearing, **kw):
        throw = aimed_throw(CFG, bearing, kw.pop("distance", 3.5), rise=kw.pop("rise", 0.8), offset=kw.pop("offset", (0.0, 0.0)))
        return run_closed_loop(CFG, throw, seed=kw.pop("seed", 1), **kw)

    def test_dodges_from_sides_and_rear(self):
        for bearing in (90, -90, 135, -135, 180, 60, -60):
            r = self.run_throw(bearing)
            self.assertTrue(r["would_hit"], "bearing %d" % bearing)
            self.assertTrue(r["triggered"], "bearing %d" % bearing)
            self.assertFalse(r["hit"], "bearing %d clearance %.2f" % (bearing, r["min_clearance"]))

    def test_direction_is_away_from_the_path(self):
        r = self.run_throw(90)       # from the left: the robot should end up displaced along x, not toward the thrower
        self.assertGreater(abs(r["robot_shift"][0]), 0.15)
        self.assertLess(abs(r["robot_shift"][1]), 0.05)

    def test_wide_throw_is_ignored(self):
        for bearing, side in ((90, 1.2), (180, -1.0), (0, 1.3), (-120, 0.9)):
            r = self.run_throw(bearing, offset=(side, 0.0))
            self.assertFalse(r["would_hit"])
            self.assertFalse(r["triggered"], "bearing %d offset %.1f" % (bearing, side))

    def test_ball_on_floor_is_ignored(self):
        hip = CFG["robot"]["hip_height_m"]
        r = self.run_throw(90, offset=(3.0, 0.0), floor_balls=[[-1.5, 0.8, -hip + 0.037], [1.2, -0.3, -hip + 0.037]])
        self.assertFalse(r["triggered"])

    def test_dry_run_reports_but_does_not_move(self):
        r = self.run_throw(-90, armed=False)
        self.assertTrue(r["triggered"])
        self.assertTrue(r["dodge_event"]["dry_run"])
        self.assertLess(np.linalg.norm(r["robot_shift"]), 1e-6)
        self.assertTrue(r["hit"])

    def test_event_is_plain_json(self):
        import json
        ev = self.run_throw(-90, armed=False)["dodge_event"]
        self.assertIsInstance(ev["late"], bool)                           # not numpy's, which is saved as the text "True"
        self.assertEqual(json.loads(json.dumps(ev))["late"], ev["late"])  # and nothing else needs default=str either

    def test_events_tell_the_story(self):
        r = self.run_throw(135)
        kinds = [e["event"] for e in r["events"]]
        self.assertIn("dodge", kinds)
        ev = r["dodge_event"]
        self.assertLess(abs(((ev["threat"]["bearing_deg"] - 135 + 180) % 360) - 180), 12)


class HighLobAndSmoothFloorTests(unittest.TestCase):
    """Throws in a small room: lobs that leave the top of a rear camera's picture, and a gentler diagonal."""

    def test_a_ball_in_flight_is_followed_while_out_of_sight(self):
        from dodge.track import Tracker
        from dodge.measure import Measurement

        def feed(tracker, velocity, frames=6):
            p = np.array([-2.0, 0.5, 0.6])
            for i in range(frames):
                t = i / 60.0
                pos = p + np.array(velocity) * t
                rng = float(np.linalg.norm(pos))
                tracker.update(t, [Measurement("elp_left", np.zeros(3), pos / rng, 0.074 / rng, rng, 0.01, 0.08, False)], np.eye(3))
            return (frames - 1) / 60.0

        flying = Tracker(CFG["tracker"], CFG["ball"])
        t_last = feed(flying, (4.0, -1.0, 3.0))
        self.assertTrue(flying.confirmed()[0].flying)
        flying.update(t_last + 0.4, [], np.eye(3))                       # unseen for 0.4 s: still followed
        self.assertEqual(len(flying.confirmed()), 1)
        flying.update(t_last + float(CFG["tracker"]["flight_coast_s"]) + 0.05, [], np.eye(3))
        self.assertEqual(len(flying.confirmed()), 0)

        still = Tracker(CFG["tracker"], CFG["ball"])
        t_last = feed(still, (0.0, 0.0, 0.0))
        self.assertFalse(still.confirmed()[0].flying)
        still.update(t_last + 0.4, [], np.eye(3))                        # a still object is dropped as before
        self.assertEqual(len(still.confirmed()), 0)

    def test_a_still_object_whose_range_jitters_is_not_a_throw(self):
        # Range comes from the blob's size, so a still green thing appears to jump back and forth along the
        # line of sight. That must not count as flight, or as a throw to report and save.
        from dodge.measure import Measurement
        core = DodgeCore(CFG, armed=True)
        core.set_can_dodge(True)
        rng_gen = np.random.default_rng(5)
        ray = np.array([-0.8, 0.5, 0.2]) / np.linalg.norm([-0.8, 0.5, 0.2])
        t = 10.0
        for _ in range(120):                                             # four seconds at 30 pictures a second
            rng = 1.8 + rng_gen.normal(0, 0.12)
            core.process_measurements("elp_left", t, [Measurement("elp_left", np.zeros(3), ray, 0.074 / rng, rng, 0.01, 0.15, False)], t)
            t += 1 / 30.0
            self.assertFalse(any(tr.flying for tr in core.tracker.tracks))
        for _ in range(40):
            core.process_measurements("elp_left", t, [], t)
            t += 1 / 30.0
        self.assertEqual(core.pop_events(), [])

    def test_high_lobs_from_the_side_and_rear_are_dodged(self):
        # The ball climbs out of the rear cameras' picture and comes down on the robot from above.
        dodged = 0
        for bearing, dist, seed in ((95, 2.0, 1), (-100, 1.9, 2), (140, 2.3, 3), (-150, 2.4, 4), (175, 3.0, 5)):
            r = run_closed_loop(CFG, aimed_throw(CFG, bearing, dist, rise=1.3), seed=seed)
            self.assertTrue(r["would_hit"])
            dodged += 0 if r["hit"] else 1
        self.assertGreaterEqual(dodged, 4)

    def test_a_lob_is_judged_while_it_is_still_climbing(self):
        # Early in a high lob the ball goes mostly up and is not yet closing in. It must be judged by where
        # its arc comes down, so the dodge starts with most of the flight still to go.
        import copy
        left_new, left_old = [], []
        old = copy.deepcopy(CFG)
        old["threat"]["use_bearing"] = False              # judged by its size, and only once it is closing in
        old["threat"]["judge_rising_balls"] = False
        for bearing, seed in ((140, 1), (-135, 2), (150, 3)):
            throw = aimed_throw(CFG, bearing, 2.3, rise=1.4)
            left_new.append(run_closed_loop(CFG, throw, seed=seed)["time_left"] or 0.0)
            left_old.append(run_closed_loop(old, throw, seed=seed)["time_left"] or 0.0)
        self.assertGreater(float(np.median(left_new)), float(np.median(left_old)) + 0.15)
        self.assertGreater(float(np.median(left_new)), 0.8)

    def test_the_diagonal_move_is_gentler(self):
        planner = EscapePlanner(dict(CFG["escape"], diagonal_scale=0.8), CFG["robot"], CFG["ball"], 0.05)
        vx, vy, _speed = planner.command_for(np.array([0.7071, 0.7071]))
        self.assertAlmostEqual(vx, 0.8 * CFG["escape"]["vx_forward"])
        self.assertAlmostEqual(vy, 0.8 * CFG["escape"]["vy"])
        vx, vy, _speed = planner.command_for(np.array([1.0, 0.0]))       # a single axis keeps its full speed
        self.assertAlmostEqual(vx, CFG["escape"]["vx_forward"])
        self.assertEqual(vy, 0.0)

    def test_a_throw_that_is_seen_but_not_dodged_is_reported(self):
        import json
        from dodge.measure import Measurement
        core = DodgeCore(CFG, armed=True)
        core.set_can_dodge(True)
        t = 50.0
        for i in range(14):                                   # a ball flying past, 1.5 m to the side
            pos = np.array([-2.0, 1.5, 0.5]) + np.array([4.0, 0.0, 1.0]) * (i / 60.0)
            rng = float(np.linalg.norm(pos))
            core.process_measurements("elp_left", t, [Measurement("elp_left", np.zeros(3), pos / rng, 0.074 / rng, rng, 0.01, 0.08, False)], t)
            t += 1 / 60.0
        for _ in range(80):                                   # then gone
            core.process_measurements("elp_left", t, [], t)
            t += 1 / 60.0
        events = core.pop_events()
        self.assertEqual([e["event"] for e in events], ["throw_seen"])
        self.assertIn("verdicts", events[0])
        json.dumps(events[0])                                 # must be plain data: it is saved and sent as JSON

        r = run_closed_loop(CFG, aimed_throw(CFG, 90, 2.0, rise=0.6), seed=3)
        self.assertIn(json.loads(json.dumps(r["dodge_event"]))["late"], (True, False))    # a real true/false, not a word
        self.assertFalse([e for e in r["events"] if e["event"] == "throw_seen"])           # a dodged throw is not also "seen"


class HandledBallTests(unittest.TestCase):
    """Balls being handled beside the robot, and balls lying about, must not set off a dodge. On the Pupper
    (2026-11-26) taking balls out of a basket 1.5 m away did, ten times in four minutes."""

    @staticmethod
    def meas(cam, ray, rng, vague=False):
        from dodge.measure import Measurement
        return Measurement(cam, np.zeros(3), ray, 0.074 / rng, rng, 0.01, 0.03 * rng, False, vague=vague)

    def test_cameras_that_disagree_about_a_still_balls_range_do_not_make_a_throw(self):
        # The same still ball reads 4.5 m in one camera (a hand over half of it) and 1.5 m in the next.
        # Put together that is a ball coming in at 10 m/s; within either camera it is not moving.
        core = DodgeCore(CFG, armed=True)
        core.set_can_dodge(True)
        ray = np.array([0.15, -1.0, 0.1]) / np.linalg.norm([0.15, -1.0, 0.1])
        t = 50.0
        seq = [("elp_right", 4.5)] * 4 + [("fisheye", 1.5), ("elp_right", 4.4), ("fisheye", 1.45), ("fisheye", 1.5)] * 4
        reasons = set()
        for cam, rng in seq:
            core.process_measurements(cam, t, [self.meas(cam, ray, rng)], t)
            reasons.update(th.reason for th in core.last_threats)
            t += 0.045
        self.assertEqual([e for e in core.pop_events() if e["event"] == "dodge"], [])
        self.assertFalse(any(tr.flying for tr in core.tracker.tracks))

    def test_an_approach_needs_three_good_looks_from_one_camera(self):
        from dodge.track import Track
        ray = np.array([-1.0, 0.0, 0.2]) / np.linalg.norm([-1.0, 0.0, 0.2])
        tr = Track(0.0, 3.0 * ray, np.eye(3) * 0.01, 7.0, "elp_left")
        tr.remember(0.045, 2.7 * ray, "elp_left")
        self.assertIsNone(tr.camera_closing(), "two looks are not enough: a still blob flickers that much")
        tr.remember(0.09, 2.4 * ray, "elp_left")
        self.assertAlmostEqual(tr.camera_closing(), 6.67, delta=0.1)
        # looks whose range cannot be trusted (cut off, or not round) do not count
        tr2 = Track(0.0, 3.0 * ray, np.eye(3) * 0.01, 7.0, "elp_left")
        tr2.remember(0.045, 2.7 * ray, "elp_left", good=False)
        tr2.remember(0.09, 2.4 * ray, "elp_left")
        self.assertIsNone(tr2.camera_closing())
        # and looks from another camera are a different ruler
        tr3 = Track(0.0, 3.0 * ray, np.eye(3) * 0.01, 7.0, "elp_left")
        tr3.remember(0.045, 1.5 * ray, "fisheye")
        tr3.remember(0.09, 1.4 * ray, "fisheye")
        self.assertIsNone(tr3.camera_closing())

    def test_a_blob_on_the_other_side_of_the_robot_is_not_the_same_ball(self):
        # Every camera's rays start at the robot. A track whose range is vague would otherwise take a blob
        # on the opposite side for itself, and the two together put the "ball" at the robot.
        from dodge.measure import Measurement
        from dodge.track import Tracker
        tracker = Tracker(CFG["tracker"], CFG["ball"])
        right = np.array([0.2, -1.0, 0.05]) / np.linalg.norm([0.2, -1.0, 0.05])
        left = np.array([-0.3, 1.0, 0.1]) / np.linalg.norm([-0.3, 1.0, 0.1])
        tracker.update(0.0, [Measurement("elp_right", np.zeros(3), right, 0.03, 2.4, 0.01, 1.5, True)])
        tracker.update(0.05, [Measurement("elp_left", np.zeros(3), left, 0.06, 1.2, 0.01, 0.05, False)])
        self.assertEqual(len(tracker.tracks), 2)
        self.assertLess(tracker.tracks[0].p[1], -2.0, "the first track stays where it was")
        self.assertEqual(tracker.tracks[0].hits, 1)

    def test_a_smeared_ball_reads_as_far_as_a_sharp_one_in_the_long_exposure_camera(self):
        from dodge.detect import Blob
        cams = build_cameras(CFG)
        fisheye = cams["fisheye"]
        self.assertTrue(fisheye.cfg.get("size_by_width"))
        u, v = 700.0, 400.0
        sharp = measure_blob(fisheye, Blob(u, v, math.pi / 4 * 14 * 14, 0, 0, 14, 14, False), CFG["ball"]["diameter_m"])
        smeared = measure_blob(fisheye, Blob(u, v, math.pi / 4 * 14 * 26, 0, 0, 14, 26, False), CFG["ball"]["diameter_m"])
        self.assertLess(abs(smeared.range_m - sharp.range_m), 0.06 * sharp.range_m)
        self.assertFalse(smeared.vague)

    def test_a_lopsided_blob_in_a_short_exposure_camera_has_a_vague_range(self):
        from dodge.detect import Blob
        elp = build_cameras(CFG)["elp_right"]
        whole = measure_blob(elp, Blob(640.0, 360.0, math.pi / 4 * 30 * 30, 0, 0, 30, 30, False), CFG["ball"]["diameter_m"])
        half = measure_blob(elp, Blob(640.0, 360.0, math.pi / 4 * 30 * 17, 0, 0, 30, 17, False), CFG["ball"]["diameter_m"])
        self.assertFalse(whole.vague)
        self.assertTrue(half.vague)
        self.assertGreaterEqual(half.sigma_r, 0.3 * half.range_m - 1e-9)

    def test_a_still_ball_found_whole_then_half_stays_left_out(self):
        from dodge.detect import Blob
        from dodge.pipeline import StillFilter
        still = StillFilter(dict(CFG["detector"].get("still") or {}, busy_level=0))    # the rule for one spot, by itself
        passed_late = 0
        for i in range(120):                       # four seconds at 30 pictures a second
            t = 5.0 + i / 30.0
            d = 28.0 if (i // 4) % 2 else 22.0     # whole, half, whole, half ...
            kept, _left = still.filter(t, [Blob(400.0, 300.0, math.pi / 4 * d * d, 0, 0, int(d), int(d), False)])
            if t > 5.0 + 2.5:
                passed_late += len(kept)
        self.assertEqual(passed_late, 0)
        # a ball that swells beyond anything the place has shown is passed on again
        kept, _left = still.filter(9.02, [Blob(400.0, 300.0, math.pi / 4 * 46.0 * 46.0, 0, 0, 46, 46, False)])
        self.assertEqual(len(kept), 1)


class CoreStateTests(unittest.TestCase):
    def test_quaternion_convention(self):
        self.assertTrue(np.allclose(quat_to_rot(1, 0, 0, 0), np.eye(3)))
        yaw90 = quat_to_rot(math.cos(math.pi / 4), 0, 0, math.sin(math.pi / 4))
        self.assertTrue(np.allclose(yaw90 @ [1, 0, 0], [0, 1, 0], atol=1e-9))

    def test_cannot_dodge_is_reported(self):
        throw = aimed_throw(CFG, 90, 3.5, rise=0.8)
        core_cfg = CFG
        # run the loop by hand so the core can be told it may not move
        from dodge.sim import SimCamera
        core = DodgeCore(core_cfg, armed=True)
        core.set_can_dodge(False, "no walking controller active")
        rng = np.random.default_rng(0)
        cam = SimCamera(core.cameras["elp_left"], 60.0, 0.03, rng)
        cam.ball_radius = 0.037
        t = 0.0
        while t < throw.t_land:
            ball = throw.position(t)
            cam.step(t, [ball])
            for _a, t_exp, blobs in cam.ready(t):
                core.process_blobs("elp_left", 50.0 + t_exp, blobs, 50.0 + t)
            self.assertIsNone(core.command(50.0 + t))
            t += 0.002
        events = core.pop_events()
        self.assertTrue(any(e["event"] == "cannot_dodge" for e in events))
        self.assertFalse(any(e["event"] == "dodge" for e in events))

    def approach_static_object(self, tell_core: bool) -> list:
        """The robot runs at 1.4 m/s toward a ball-coloured object hanging 0.75 m above it, straight ahead."""
        from dodge.sim import SimCamera
        core = DodgeCore(CFG, armed=False)
        rng = np.random.default_rng(2)
        cam = SimCamera(core.cameras["fisheye"], 30.0, 0.06, rng)
        cam.ball_radius = 0.037
        t = 0.0
        while t < 1.9:
            rel = np.array([3.0 - 1.4 * t, 0.0, 0.75])         # it only looks like it is coming at us
            cam.step(t, [rel])
            if tell_core:
                core.set_robot_velocity(20.0 + t, 1.4, 0.0)
            for _a, t_exp, blobs in cam.ready(t):
                core.process_blobs("fisheye", 20.0 + t_exp, blobs, 20.0 + t)
            core.command(20.0 + t)
            t += 0.002
        return [e["event"] for e in core.pop_events()]

    def test_own_motion_is_not_mistaken_for_a_throw(self):
        self.assertIn("dodge", self.approach_static_object(tell_core=False))      # without the walking speed it would be fooled
        self.assertNotIn("dodge", self.approach_static_object(tell_core=True))

    def test_emergency_stop_cancels_a_dodge(self):
        core = DodgeCore(CFG, armed=True)
        core.state = State.DODGE
        core.plan = EscapePlanner(CFG["escape"], CFG["robot"], CFG["ball"], 0.05).plan(
            Threat(1, True, True, "", 0.8, 0.0, 0.05, 0.05, 0.0, 0.0, np.array([0.0, -1.0]), 90.0, 50.0, 4.0, 2.0, 5.0))
        core.t_dodge = 10.0
        self.assertIsNotNone(core.command(10.05))
        core.emergency_stop(10.06)
        self.assertIsNone(core.command(10.07))
        self.assertEqual(core.state, State.SEARCH)

    def test_dodge_then_recover_then_quiet(self):
        core = DodgeCore(CFG, armed=True)
        core.state = State.DODGE
        core.plan = EscapePlanner(CFG["escape"], CFG["robot"], CFG["ball"], 0.05).plan(
            Threat(1, True, True, "", 0.8, 0.0, 0.05, 0.05, 0.0, 0.0, np.array([0.0, -1.0]), 90.0, 50.0, 4.0, 2.0, 5.0))
        core.t_dodge = 0.0
        cmd = core.command(0.1)
        self.assertNotEqual(cmd[0], 0.0)
        t, seen_zero = 0.1, False
        while t < 4.0:
            t += 0.01
            core.set_imu(t, (1.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
            cmd = core.command(t)
            if cmd == (0.0, 0.0, 0.0):
                seen_zero = True
            if cmd is None:
                break
        self.assertTrue(seen_zero)               # stood still first
        self.assertIsNone(core.command(t + 0.01))  # then went quiet so the mux hands control back
        self.assertEqual(core.state, State.SEARCH)

    def test_the_burst_eases_to_zero_instead_of_stopping_dead(self):
        # On the Pupper the body's tilt peaked just after a burst ended (26 degrees, and one fall).
        core = DodgeCore(CFG, armed=True)
        core.state = State.DODGE
        core.plan = EscapePlanner(CFG["escape"], CFG["robot"], CFG["ball"], 0.05).plan(
            Threat(1, True, True, "", 0.8, 0.0, 0.05, 0.05, 0.0, 0.0, np.array([0.0, -1.0]), 90.0, 50.0, 4.0, 2.0, 5.0))
        core.t_dodge = 0.0
        full = core.command(0.5 * core.plan.duration)
        ramp = float(CFG["escape"]["ramp_out_s"])
        half = core.command(core.plan.duration + 0.5 * ramp)
        self.assertAlmostEqual(half[0], 0.5 * full[0], delta=0.03 * abs(full[0]) + 1e-6)
        self.assertEqual(core.state, State.DODGE)
        self.assertEqual(core.command(core.plan.duration + ramp + 0.01), (0.0, 0.0, 0.0))
        self.assertEqual(core.state, State.RECOVER)


class BearingTests(unittest.TestCase):
    """Whether a ball is coming is read from how its direction moves, not from how fast it grows."""

    LENS = np.array([0.045, 0.0315, 0.152])

    def looks(self, p0, v0, n=5, dt=0.045, size_noise=0.0, seed=0, cam="elp_left"):
        """A track fed with what one camera sees of a ball in free flight from p0 with velocity v0 (world = base here)."""
        from dodge.track import Track
        rng = np.random.default_rng(seed)
        tr = None
        for i in range(n):
            t = 10.0 + i * dt
            p = np.asarray(p0, float) + np.asarray(v0, float) * (i * dt) + np.array([0.0, 0.0, -0.5 * 9.81 * (i * dt) ** 2])
            d = p - self.LENS
            r = float(np.linalg.norm(d))
            r_meas = r * (1.0 + size_noise * rng.normal())
            z = self.LENS + d / r * r_meas
            if tr is None:
                tr = Track(t, z, np.eye(3) * 0.01, 7.0, cam, ray=d / r, origin=self.LENS, range_m=r_meas)
            else:
                tr.remember(t, z, cam, ray=d / r, origin=self.LENS, range_m=r_meas)
                tr.hits += 1
                tr.t_last = t
        tr.confirmed = True
        return tr, 10.0 + (n - 1) * dt

    def assess(self, tr, t):
        from dodge.threat import ThreatAssessor
        return ThreatAssessor(CFG["threat"], CFG["robot"], CFG["ball"]).assess(tr, t, None)

    @staticmethod
    def lob(distance, bearing_deg, flight, side=0.0, release_h=0.75):
        """Start point and velocity of a ball released at release_h that comes down on the body (or side metres beside it)."""
        b = math.radians(bearing_deg)
        p0 = np.array([distance * math.cos(b), distance * math.sin(b), release_h])
        target = np.array([-side * math.sin(b), side * math.cos(b), 0.0])
        v = (target - p0) / flight
        v[2] = (0.0 - release_h + 0.5 * 9.81 * flight * flight) / flight
        return p0, v

    def test_a_lob_at_the_robot_is_recognised_from_three_looks_with_its_speed_and_time(self):
        for bearing, flight, dist in ((150.0, 0.9, 2.5), (100.0, 0.8, 2.2), (170.0, 1.0, 3.0)):
            p0, v0 = self.lob(dist, bearing, flight)
            tr, t = self.looks(p0, v0, n=3, size_noise=0.08, seed=int(bearing))     # ranges 8 % off, as from blob size
            th = self.assess(tr, t)
            self.assertTrue(th.is_threat, th.reason)
            self.assertAlmostEqual(th.closing_speed, dist / flight, delta=0.15 * dist / flight)
            self.assertAlmostEqual(th.tau, flight - 2 * 0.045, delta=0.2)

    def test_a_still_thing_whose_blob_flickers_is_not_a_throw(self):
        # The blob at one spot reads 15 px, then 23: by size that is a ball rushing in (the false dodges of 2026-11-26).
        from dodge.track import Track
        ray = np.array([-0.6, 0.7, 0.35]) / np.linalg.norm([-0.6, 0.7, 0.35])
        tr = Track(10.0, self.LENS + 2.6 * ray, np.eye(3) * 0.01, 7.0, "elp_left", ray=ray, origin=self.LENS, range_m=2.6)
        for i, r in enumerate((2.3, 2.4, 1.5, 1.4), start=1):
            tr.remember(10.0 + 0.045 * i, self.LENS + r * ray, "elp_left", ray=ray, origin=self.LENS, range_m=r)
            tr.hits += 1
            tr.t_last = 10.0 + 0.045 * i
        tr.confirmed = True
        th = self.assess(tr, 10.2)
        self.assertFalse(th.is_threat)
        self.assertEqual(th.reason, "not approaching")

    def test_a_lob_aimed_a_metre_wide_and_one_that_falls_short_are_left_alone(self):
        p0, v0 = self.lob(2.5, 140.0, 0.9, side=1.0)
        tr, t = self.looks(p0, v0, n=5)
        th = self.assess(tr, t)
        self.assertFalse(th.is_threat, th.reason)
        self.assertEqual(th.reason, "will miss")
        self.assertGreater(th.miss_m, 0.5)
        p0, v0 = self.lob(2.5, 140.0, 0.9)
        short = np.array([0.45 * v0[0], 0.45 * v0[1], 0.6])       # thrown weakly: comes down a metre and a half away
        tr, t = self.looks(p0 + short * 0.45, short + np.array([0.0, 0.0, -9.81 * 0.45]), n=5)     # seen already on its way down
        self.assertFalse(self.assess(tr, t).is_threat)

    def test_a_ball_lifted_by_hand_close_by_is_not_a_throw(self):
        # It climbs in the picture as a throw from there would, but a throw from 1.2 m would be growing fast.
        b = math.radians(-80.0)
        p0 = np.array([1.2 * math.cos(b), 1.2 * math.sin(b), 0.0])
        from dodge.track import Track
        tr = None
        for i in range(5):
            t = 10.0 + 0.045 * i
            p = p0 + np.array([0.0, 0.0, 1.3 * 0.045 * i])            # lifted straight up at 1.3 m/s
            d = p - self.LENS
            r = float(np.linalg.norm(d))
            if tr is None:
                tr = Track(t, p, np.eye(3) * 0.01, 7.0, "elp_right", ray=d / r, origin=self.LENS, range_m=r)
            else:
                tr.remember(t, p, "elp_right", ray=d / r, origin=self.LENS, range_m=r)
                tr.hits += 1
                tr.t_last = t
        tr.confirmed = True
        th = self.assess(tr, 10.18)
        self.assertFalse(th.is_threat, th.reason)
        self.assertIn("not getting bigger", th.reason)

    def feed(self, core, p0, v0, cams, n, dt=0.045, t0=30.0, top=()):
        """Give the core n looks of a ball in free flight, by the named cameras in turn. Returns the time of the last.
        top: which looks (by number) are in the top strip of the picture, from where a climbing ball leaves the view."""
        from dodge.measure import Measurement
        for i in range(n):
            t = t0 + i * dt
            p = np.asarray(p0, float) + np.asarray(v0, float) * (i * dt) + np.array([0.0, 0.0, -0.5 * 9.81 * (i * dt) ** 2])
            d = p - self.LENS
            r = float(np.linalg.norm(d))
            cam = cams[i % len(cams)]
            core.process_measurements(cam, t, [Measurement(cam, self.LENS.copy(), d / r, 0.074 / r, r, 0.01, 0.04 * r, False,
                                                           near_top=i in top)], t)
        return t0 + (n - 1) * dt

    def test_a_lob_from_straight_behind_seen_only_twice_is_still_dodged(self):
        # Straight behind, the rear cameras see only 30 degrees up: a lob is out of their sight a tenth of a
        # second after it is thrown. Two looks, then nothing, must be enough.
        from dodge.core import DodgeCore
        p0, v0 = self.lob(2.3, 180.0, 1.05)
        core = DodgeCore(CFG, armed=True)
        core.set_can_dodge(True)
        t = self.feed(core, p0, v0, ["elp_left"], 2, top=(1,))
        fired_at = None
        for k in range(1, 30):                                   # the other cameras' pictures keep coming, without the ball
            now = t + 0.016 * k
            if core.process_measurements("fisheye", now, [], now) and fired_at is None:
                fired_at = now - t
        self.assertIsNotNone(fired_at, "no dodge")
        self.assertLess(fired_at, 0.25)
        self.assertGreaterEqual(fired_at, 0.06, "it must first give a third look the chance to come")
        dodge = [e for e in core.pop_events() if e["event"] == "dodge"][0]
        self.assertAlmostEqual(abs(dodge["threat"]["bearing_deg"]), 180.0, delta=15.0)
        self.assertEqual(dodge["plan"]["vx"], 0.0)               # from behind it can only sidestep

    def test_two_looks_that_do_not_climb_like_a_lob_are_not_judged(self):
        from dodge.core import DodgeCore
        side = self.lob(2.3, 95.0, 1.05)
        behind = self.lob(2.3, 180.0, 1.05)
        for p0, v0, cams, top in (((-2.3, 0.0, 0.6), (0.0, 0.0, 0.0), ["elp_left"], (1,)),      # a still thing
                                  ((-2.3, 0.0, 0.6), (0.0, 2.5, 0.3), ["elp_left"], (1,)),      # going sideways
                                  ((2.3, 0.0, 0.6), (-2.2, 0.0, 4.6), ["fisheye"], (1,)),       # in front: the fisheye sees all of a lob
                                  (side[0], side[1], ["elp_left"], (1,)),                       # a lob, but from the side: three looks will come
                                  (behind[0], behind[1], ["elp_left"], ())):                    # behind, but mid-picture: it was hidden, not gone
            core = DodgeCore(CFG, armed=True)
            core.set_can_dodge(True)
            t = self.feed(core, p0, v0, cams, 2, top=top)
            self.assertFalse(any(tr.climber for tr in core.tracker.tracks))
            for k in range(1, 30):
                now = t + 0.016 * k
                core.process_measurements("elp_right", now, [], now)
            self.assertEqual([e for e in core.pop_events() if e["event"] == "dodge"], [])

    def test_a_second_sighting_after_a_gap_in_the_pictures_does_not_crash_the_tracker(self):
        # Found by replaying the Pupper's recordings (21 of 339 stopped here): when the pictures stall for
        # more than the 0.35 s a track remembers, its first look is gone by the time the second arrives.
        from dodge.measure import Measurement
        from dodge.track import Tracker
        tracker = Tracker(CFG["tracker"], CFG["ball"])
        ray = np.array([-0.8, 0.5, 0.2]) / np.linalg.norm([-0.8, 0.5, 0.2])
        for t in (10.0, 10.5):
            tracker.update(t, [Measurement("elp_left", np.zeros(3), ray, 0.03, 2.4, 0.01, 0.1, False)])
        self.assertEqual([(tr.hits, tr.climber) for tr in tracker.tracks], [(2, False)])

    def three_quick_looks_then_gone(self, top):
        """Three looks 24 ms apart (the sideways estimate is still vague after those), then out of sight.
        Returns how long after the last look a dodge started, or None."""
        from dodge.core import DodgeCore
        p0, v0 = self.lob(2.6, 178.0, 1.1)
        core = DodgeCore(CFG, armed=True)
        core.set_can_dodge(True)
        t = self.feed(core, p0, v0, ["elp_left"], 3, dt=0.024, top=top)
        for k in range(1, 22):
            now = t + 0.016 * k
            if core.process_measurements("fisheye", now, [], now):
                return now - t
        return None

    def test_a_ball_that_climbed_out_of_the_picture_is_dodged_without_a_surer_look(self):
        # No better look is coming, so it must not wait for one.
        fired = self.three_quick_looks_then_gone(top=(2,))
        self.assertIsNotNone(fired)
        self.assertLess(fired, 0.35)

    def test_a_ball_that_just_stops_being_seen_mid_picture_is_waited_for(self):
        # Hidden by a hand or a basket, not gone: four false dodges in the overnight replay came from not
        # telling the two apart. It is still dodged once there is no time left to wait.
        late = self.three_quick_looks_then_gone(top=())
        gone = self.three_quick_looks_then_gone(top=(2,))
        self.assertTrue(late is None or late > gone + 0.1)

    def test_a_still_patch_whose_blob_grows_does_not_narrow_the_cameras_search(self):
        # With a hint the cameras search a small window in most pictures. A flickering still patch used to
        # give one (its blob "grows"), which hid a real lob in two pictures of every three in a recording.
        from dodge.core import DodgeCore
        from dodge.measure import Measurement
        core = DodgeCore(CFG, armed=False)
        ray = np.array([-0.6, 0.7, 0.35]) / np.linalg.norm([-0.6, 0.7, 0.35])
        for i, r in enumerate((2.6, 2.3, 2.0, 1.7, 1.5, 1.4)):
            t = 40.0 + 0.045 * i
            core.process_measurements("elp_left", t, [Measurement("elp_left", self.LENS.copy(), ray, 0.074 / r, r, 0.01, 0.04 * r, False)], t)
        self.assertTrue(any(tr.confirmed and tr.closing_speed > 0.5 for tr in core.tracker.tracks), "by its size it is rushing in")
        self.assertIsNone(core.track_hint())
        p0, v0 = self.lob(2.5, 150.0, 0.9)                       # a real lob does give one
        core2 = DodgeCore(CFG, armed=False)
        self.feed(core2, p0, v0, ["elp_left"], 4)
        self.assertIsNotNone(core2.track_hint())

    def test_the_time_it_gives_does_not_depend_on_reading_the_size_right(self):
        # The same lob with every range read 15 % too far (a blob that comes out small): 15 % later, no more.
        p0, v0 = self.lob(2.5, 150.0, 0.9)
        tr, t = self.looks(p0, v0, n=4)
        good = self.assess(tr, t)
        for i, h in enumerate(tr.history):
            tr.history[i] = h[:5] + (h[5] * 1.15,) + h[6:]
        off = self.assess(tr, t)
        self.assertTrue(off.is_threat, off.reason)
        self.assertAlmostEqual(off.closing_speed, good.closing_speed, delta=0.05 * good.closing_speed)
        self.assertLess(abs(off.tau - good.tau), 0.2)


class RearPairTests(unittest.TestCase):
    """Straight behind, both rear cameras see the ball at the outer edge of their pictures, for a tenth of a
    second or so. Their looks must come together as one ball (2026-11-27)."""

    LEFT = np.array([0.045, 0.0315, 0.152])
    RIGHT = np.array([0.045, -0.0315, 0.152])

    def path(self, p0, v0, t):
        return np.asarray(p0, float) + np.asarray(v0, float) * t + np.array([0.0, 0.0, -0.5 * 9.81 * t * t])

    def meas(self, cam, p, range_factor=1.0, yaw_err_deg=0.0, top=False):
        from dodge.measure import Measurement
        lens = self.LEFT if cam == "elp_left" else self.RIGHT
        d = p - lens
        r = float(np.linalg.norm(d))
        ray = d / r
        if yaw_err_deg:
            c, s_ = math.cos(math.radians(yaw_err_deg)), math.sin(math.radians(yaw_err_deg))
            ray = np.array([c * ray[0] - s_ * ray[1], s_ * ray[0] + c * ray[1], ray[2]])
        rm = r * range_factor
        return Measurement(cam, lens.copy(), ray, 0.074 / rm, rm, 0.01, 0.02 * rm, False, near_top=top)

    def test_a_lob_from_straight_behind_seen_by_both_cameras_in_turn_is_dodged(self):
        # The cameras are not in step (their first looks 5 ms apart), and elp_right reads every range 25 %
        # short (its own size calibration). Before 2026-11-27 these made two tracks, each too short to judge.
        from dodge.core import DodgeCore
        p0, v0 = BearingTests.lob(2.4, 179.0, 1.0)
        core = DodgeCore(CFG, armed=True)
        core.set_can_dodge(True)
        looks = [(0.000, "elp_left"), (0.005, "elp_right"), (0.059, "elp_left"), (0.064, "elp_right")]
        t0, fired = 50.0, None
        for i, (dt, cam) in enumerate(looks):
            m = self.meas(cam, self.path(p0, v0, dt), 0.75 if cam == "elp_right" else 1.0, top=i >= 2)
            core.process_measurements(cam, t0 + dt, [m], t0 + dt)
        for k in range(1, 30):
            now = t0 + 0.064 + 0.016 * k
            if core.process_measurements("fisheye", now, [], now) and fired is None:
                fired = now - t0
        tracks = [tr for tr in core.tracker.tracks if tr.hits >= 2]
        self.assertEqual(len(tracks), 1, "both cameras' looks in one track")
        self.assertEqual(tracks[0].cams, {"elp_left", "elp_right"})
        self.assertIsNotNone(fired, "no dodge")
        self.assertLess(fired, 0.3)

    def test_the_gap_between_the_two_lenses_is_not_taken_for_sideways_motion(self):
        # The lenses are 63 mm apart: at 1.5-2.4 m the same ball reads 1.5-2.4 degrees apart in azimuth.
        from dodge.track import Track
        p0, v0 = BearingTests.lob(2.4, 180.0, 1.0)
        tr = None
        for i, dt in enumerate((0.0, 0.03, 0.06, 0.09)):
            cam = "elp_left" if i % 2 == 0 else "elp_right"
            m = self.meas(cam, self.path(p0, v0, dt))
            if tr is None:
                tr = Track(10.0, m.point, np.eye(3) * 0.01, 7.0, cam, ray=m.ray, origin=m.origin, range_m=m.range_m)
            else:
                tr.remember(10.0 + dt, m.point, cam, ray=m.ray, origin=m.origin, range_m=m.range_m)
        pooled = tr.hit_course(3, 0.04, pool=(("elp_left", "elp_right"),), offsets=False)
        self.assertIsNotNone(pooled)
        self.assertLess(abs(pooled["az_dot"]), 0.03)
        self.assertAlmostEqual(9.81 / (2.0 * pooled["q_dot"]), 2.4, delta=0.3)

    def test_a_degree_and_a_half_between_the_two_cameras_does_not_read_as_a_miss(self):
        # Each lens has its own error, largest at the edge of its picture, where both see a ball straight
        # behind. With two looks or more from a camera, the motion is read within each camera.
        from dodge.track import Track
        from dodge.threat import ThreatAssessor
        p0, v0 = BearingTests.lob(2.4, 180.0, 1.0)
        tr = None
        for i, dt in enumerate((0.0, 0.02, 0.05, 0.07)):
            cam = "elp_left" if i % 2 == 0 else "elp_right"
            m = self.meas(cam, self.path(p0, v0, dt), yaw_err_deg=1.5 if cam == "elp_right" else 0.0)
            if tr is None:
                tr = Track(10.0, m.point, np.eye(3) * 0.01, 7.0, cam, ray=m.ray, origin=m.origin, range_m=m.range_m)
            else:
                tr.remember(10.0 + dt, m.point, cam, ray=m.ray, origin=m.origin, range_m=m.range_m)
                tr.hits += 1
                tr.t_last = 10.0 + dt
        tr.confirmed = True
        tr.climber = True
        plain = tr.hit_course(3, 0.04, pool=(("elp_left", "elp_right"),), offsets=False)
        within = tr.hit_course(3, 0.04, pool=(("elp_left", "elp_right"),))
        self.assertGreater(abs(plain["az_dot"]), 0.15, "put together as they are, the offset looks like a turn")
        self.assertLess(abs(within["az_dot"]), 0.03)

        class Rear:
            climber_cams = {"elp_left", "elp_right"}
        th = ThreatAssessor(CFG["threat"], CFG["robot"], CFG["ball"]).assess(tr, 10.07, Rear())
        self.assertTrue(th.is_threat, th.reason)

    def test_only_the_rear_pair_gets_slack_in_range_when_matching(self):
        from dodge.measure import Measurement
        from dodge.track import Tracker
        ray = np.array([-0.97, 0.05, 0.25]) / np.linalg.norm([-0.97, 0.05, 0.25])
        tracker = Tracker(CFG["tracker"], CFG["ball"])
        tracker.update(10.0, [Measurement("elp_left", self.LEFT.copy(), ray, 0.03, 2.4, 0.01, 0.05, False)])
        tracker.update(10.03, [Measurement("elp_right", self.RIGHT.copy(), ray, 0.03, 1.8, 0.01, 0.04, False)])
        self.assertEqual(len(tracker.tracks), 1, "the other rear camera's look, 25 % short, is the same ball")
        tr = tracker.tracks[0]
        self.assertFalse(tracker._crosses(tr, "elp_right"), "a camera the track already has: no slack")
        tr.cams = {"fisheye"}
        self.assertFalse(tracker._crosses(tr, "elp_left"), "fisheye and a rear camera: no slack")
        tr.cams = {"elp_left"}
        self.assertFalse(tracker._crosses(tr, "fisheye"))
        self.assertTrue(tracker._crosses(tr, "elp_right"))

    def test_leaving_the_top_of_a_rear_picture_counts_only_where_the_fisheye_cannot_see(self):
        from dodge.track import Track
        from dodge.threat import ThreatAssessor
        assessor = ThreatAssessor(CFG["threat"], CFG["robot"], CFG["ball"])
        for bearing, expect in ((95.0, False), (170.0, True)):
            b = math.radians(bearing)
            ray = np.array([math.cos(b), math.sin(b), 0.55])
            ray /= np.linalg.norm(ray)
            tr = Track(10.0, self.LEFT + 2.0 * ray, np.eye(3) * 0.01, 7.0, "elp_left", ray=ray, origin=self.LEFT,
                       range_m=2.0, near_top=True)
            th = assessor.assess(tr, 10.2, None)
            self.assertEqual(th.left_view, expect, bearing)


if __name__ == "__main__":
    unittest.main(verbosity=2)
