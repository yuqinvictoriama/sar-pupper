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

    def test_events_tell_the_story(self):
        r = self.run_throw(135)
        kinds = [e["event"] for e in r["events"]]
        self.assertIn("dodge", kinds)
        ev = r["dodge_event"]
        self.assertLess(abs(((ev["threat"]["bearing_deg"] - 135 + 180) % 360) - 180), 12)


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


if __name__ == "__main__":
    unittest.main(verbosity=2)
