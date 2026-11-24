"""Off-robot tests for survivor identification: geometry, the gesture rule, tracking, the pin.

    python3 -m unittest discover -s tests -v
"""
from __future__ import annotations

import math
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

from world import CFG, FakeDetector, Person, Scene, at, frames  # noqa: E402

from survivor import appearance, pose_decode  # noqa: E402
from survivor.cameras import Rig, level_from_base, quat_to_rot  # noqa: E402
from survivor.gesture import GestureFilter, evaluate  # noqa: E402
from survivor.perception import LOCKED, LOST, SCAN, SurvivorPerception  # noqa: E402
from survivor.person import observe  # noqa: E402
from survivor.tracker import DeadReckoner  # noqa: E402

RIG = Rig(CFG)
# Bearings a camera sees whole. Straight behind (180) is the seam between the two rear cameras: they
# overlap by 5 degrees, a person is wider than that, so neither sees all of them (turn-to-look's job).
SECTORS = {"front": 0, "front-diagonal": 40, "side": 90, "rear-diagonal": 140, "right side": -90, "right rear": -150}
FACINGS = ("toward", "away", "left", "right")


def nearest(bearing_deg: float) -> float:
    """How close a person can stand with raised hands still in the picture: the fisheye sees the whole
    upper hemisphere; the rear cameras' frames end about 40 degrees up, which a raised hand reaches
    at about 2.8 m. Closer than that behind, the robot turns to look (tested further down)."""
    return 2.2 if camera_for(bearing_deg) == "fisheye" else 3.0


def camera_for(bearing_deg: float) -> str:
    b = (bearing_deg + 180) % 360 - 180
    return "fisheye" if abs(b) <= 62 else ("elp_left" if b > 0 else "elp_right")


def see(person: Person, cam: str, zoom: bool = False, scene: Scene | None = None, noise: float = 0.0):
    """What the pipeline makes of one person in one camera's wide (or zoom) view."""
    scene = scene or Scene()
    scene.people = [person]
    det = FakeDetector(scene, noise_px=noise)
    R_lb = level_from_base(quat_to_rot(*scene.quat()))
    view = RIG.wide_view(cam)
    dets = det(None, view)
    if not dets:
        return None
    obs = observe(dets[0], view, R_lb, 0.0, CFG)
    if zoom:
        b = CFG["budget"]
        mid = obs.box_rays[0] + obs.box_rays[1]
        mid /= np.linalg.norm(mid)
        height = math.degrees(math.acos(float(np.dot(obs.box_rays[0], obs.box_rays[1]))))
        view = RIG.patch_view(cam, None, R_lb.T @ mid, max(height * 1.7, 42.0), int(b["patch_px"]), R_lb)
        dets = det(None, view)
        if not dets:
            return None
        obs = observe(dets[0], view, R_lb, 0.0, CFG, zoom=True)
    return obs


class CameraTests(unittest.TestCase):
    def test_views_point_where_the_lens_model_says(self):
        """A point in space lands on a pixel, and that pixel's ray points back at the point: every view kind."""
        pts = {"fisheye": np.array([[3.0, 0.5, 1.2], [1.5, -1.0, 0.3]]),
               "elp_left": np.array([[-2.0, 2.5, 0.8], [-1.0, 3.0, 1.5]])}
        for cam, p in pts.items():
            wide = RIG.wide_view(cam)
            patch = RIG.patch_view(cam, None, p[0] - RIG.cameras[cam].t, 40.0, 384)
            for view in (wide, patch):
                uv, ok = view.project_base(p[:1])
                self.assertTrue(ok[0], (cam, view.kind))
                ray, _ = view.rays_base(uv)
                want = (p[0] - view.camera.t) / np.linalg.norm(p[0] - view.camera.t)
                self.assertLess(math.degrees(math.acos(min(1.0, float(ray[0] @ want)))), 0.05, (cam, view.kind))

    def test_fisheye_wide_view_is_lab_5s_frame(self):
        view = RIG.wide_view("fisheye")
        self.assertEqual((view.width, view.height), (700, 572))
        img = view.render(np.full((1050, 1400, 3), 200, np.uint8))
        self.assertEqual(img.shape, (572, 700, 3))
        self.assertGreater(int(img[286, 350, 0]), 150)       # straight ahead is in the picture

    def test_patch_keeps_people_upright_when_the_robot_is_tilted(self):
        scene = Scene()
        roll = math.radians(12)
        scene.R_tilt = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])
        obs = see(at(0, 3.0), "fisheye", zoom=True, scene=scene)
        k = pose_decode.K
        # In the patch, the hips are straight below the shoulders (same column to within a few pixels).
        sh = obs.kpts[[k["l_shoulder"], k["r_shoulder"]], 0].mean()
        hip = obs.kpts[[k["l_hip"], k["r_hip"]], 0].mean()
        self.assertLess(abs(sh - hip), 6.0)

    def test_frames_at_another_resolution_still_work(self):
        view = RIG.wide_view("elp_left", (1920, 1080))
        uv, ok = view.project_base(np.array([[-2.0, 2.5, 0.8]]))
        uv0, _ = RIG.wide_view("elp_left").project_base(np.array([[-2.0, 2.5, 0.8]]))
        self.assertTrue(ok[0])
        self.assertAlmostEqual(uv[0, 0] / uv0[0, 0], 1.5, delta=0.01)


class DecodeTests(unittest.TestCase):
    def encode(self, people, logits=False):
        """Build head tensors that describe the given (box, kpts) people, the way the model would."""
        heads = {g: {"box": np.zeros((g, g, 64), np.float32), "score": np.full((g, g, 1), -9.0 if logits else 0.0, np.float32),
                     "kpt": np.zeros((g, g, 51), np.float32)} for g in (80, 40, 20)}
        for (x1, y1, x2, y2), kpts in people:
            g = 20 if (y2 - y1) > 250 else (40 if (y2 - y1) > 120 else 80)
            stride = 640 / g
            gx, gy = int(((x1 + x2) / 2) // stride), int(((y1 + y2) / 2) // stride)
            cx, cy = (gx + 0.5) * stride, (gy + 0.5) * stride
            for side, d in enumerate(((cx - x1) / stride, (cy - y1) / stride, (x2 - cx) / stride, (y2 - cy) / stride)):
                lo = int(math.floor(d))
                frac = d - lo
                dist = np.full(16, -20.0, np.float32)        # two neighbouring bins whose mean is d
                dist[lo], dist[lo + 1] = math.log(max(1 - frac, 1e-6)), math.log(max(frac, 1e-6))
                heads[g]["box"][gy, gx, side * 16:(side + 1) * 16] = dist
            heads[g]["score"][gy, gx, 0] = 3.0 if logits else 0.95
            k = np.zeros((17, 3), np.float32)
            k[:, 0] = (kpts[:, 0] / stride - gx) / 2.0
            k[:, 1] = (kpts[:, 1] / stride - gy) / 2.0
            k[:, 2] = np.log(kpts[:, 2] / (1 - kpts[:, 2])) if logits else kpts[:, 2]
            heads[g]["kpt"][gy, gx] = k.ravel()
        return [h[r] for h in heads.values() for r in ("box", "score", "kpt")]

    def test_round_trip(self):
        rng = np.random.default_rng(1)
        people = []
        for box in ((100.0, 80.0, 220.0, 420.0), (400.0, 300.0, 460.0, 440.0), (500.0, 100.0, 540.0, 190.0)):
            k = np.zeros((17, 3))
            k[:, 0] = rng.uniform(box[0], box[2], 17)
            k[:, 1] = rng.uniform(box[1], box[3], 17)
            k[:, 2] = rng.uniform(0.2, 0.95, 17)
            people.append((box, k))
        for logits in (False, True):
            dets = pose_decode.decode(self.encode(people, logits), score_threshold=0.5)
            self.assertEqual(len(dets), 3)
            for box, k in people:
                d = min(dets, key=lambda d: abs(d.box[0] - box[0]))
                np.testing.assert_allclose(d.box, box, atol=0.6)
                np.testing.assert_allclose(d.kpts[:, :2], k[:, :2], atol=0.05)
                np.testing.assert_allclose(d.kpts[:, 2], k[:, 2], atol=0.01)

    def test_letterbox_is_undone(self):
        box, k = (200.0, 250.0, 300.0, 400.0), np.column_stack([np.linspace(210, 290, 17), np.linspace(260, 390, 17), np.full(17, 0.8)])
        frame = np.zeros((720, 1280, 3), np.uint8)
        _, scale, pad = pose_decode.letterbox(frame)
        self.assertEqual((scale, pad), (0.5, (0, 140)))
        d = pose_decode.decode(self.encode([(box, k)]), scale, pad, frame.shape)[0]
        np.testing.assert_allclose(d.box, [400, 220, 600, 520], atol=1.5)
        np.testing.assert_allclose(d.kpts[0, :2], [420, 240], atol=0.2)

    def test_overlapping_boxes_are_merged_and_shapes_checked(self):
        k = np.column_stack([np.full(17, 300.0), np.full(17, 300.0), np.full(17, 0.9)])
        heads = self.encode([((200.0, 100.0, 400.0, 500.0), k)])
        heads[1][10, 10, 0] = 0.9                      # a second, weaker hit on the same person... with no box: far away
        self.assertEqual(len(pose_decode.decode(heads)), 2)
        with self.assertRaises(ValueError):
            pose_decode.decode([np.zeros((20, 20, 7))])


class PersonTests(unittest.TestCase):
    def test_range_and_bearing_from_every_camera(self):
        """Range from body size: within 12 % and 3 degrees, 2 to 6 m, all round, wide and zoom."""
        for name, bearing in SECTORS.items():
            for dist in (2.0, 3.0, 4.0, 6.0):
                for zoom in (False, True):
                    obs = see(at(bearing, dist, pose="down"), camera_for(bearing), zoom=zoom)
                    self.assertIsNotNone(obs, (name, dist, zoom))
                    self.assertEqual(obs.s_from, "torso")
                    self.assertLess(abs(obs.ground_range - dist) / dist, 0.12, (name, dist, zoom, obs.ground_range))
                    err = abs((math.degrees(obs.bearing) - bearing + 180) % 360 - 180)
                    self.assertLess(err, 3.0, (name, dist, zoom))

    def test_range_survives_hidden_legs(self):
        obs = see(at(0, 3.0, hidden=("l_hip", "r_hip", "l_knee", "r_knee", "l_ankle", "r_ankle")), "fisheye")
        self.assertEqual(obs.s_from, "head")
        self.assertLess(abs(obs.ground_range - 3.0) / 3.0, 0.25)

    def test_rear_camera_cuts_off_raised_hands_up_close(self):
        """Plan, "easy to miss" 1: close behind, the raised hands are above the rear cameras' frame."""
        near = see(at(140, 1.3), "elp_left")
        far = see(at(140, 3.0), "elp_left")
        self.assertTrue(near.clipped_top)
        self.assertFalse(far.clipped_top)
        self.assertFalse(see(at(0, 1.3), "fisheye").clipped_top)     # the fisheye sees the whole upper hemisphere


class GestureTests(unittest.TestCase):
    def test_signal_is_seen_from_every_direction_and_every_way_round(self):
        """Tier A: out to 4 m, any bearing a camera sees whole, facing toward, away or side-on.
        Read from the zoom pass, with pixel noise."""
        missed = []
        for name, bearing in SECTORS.items():
            for dist in (nearest(bearing), 3.0, 4.0):
                for facing in FACINGS:
                    obs = see(at(bearing, dist, facing), camera_for(bearing), zoom=True, noise=1.0)
                    res = evaluate(obs, CFG) if obs is not None else None
                    if res is None or not res.up:
                        missed.append((name, dist, facing, None if res is None else res.why))
        self.assertEqual(missed, [])

    def test_the_reference_photo_counts(self):
        """The joints YOLOv8s-pose finds in the photo that defines the signal (a person seen from
        behind, both arms up in a relaxed V), placed 2.6 to 4 m from the fisheye and a rear camera. If a threshold
        is ever tightened past what that photo shows, this fails."""
        photo = {  # x, y (pixels in the 356 x 644 photo), confidence
            "nose": (165.1, 161.7, 0.08), "l_eye": (161.1, 156.0, 0.02), "r_eye": (173.0, 156.5, 0.02),
            "l_ear": (149.4, 163.3, 0.65), "r_ear": (196.8, 162.7, 0.59),
            "l_shoulder": (132.9, 201.6, 1.0), "r_shoulder": (211.3, 201.3, 1.0),
            "l_elbow": (96.2, 153.7, 0.99), "r_elbow": (251.2, 157.2, 0.99),
            "l_wrist": (57.7, 95.1, 0.91), "r_wrist": (285.9, 92.7, 0.95),
            "l_hip": (141.2, 346.0, 1.0), "r_hip": (191.2, 345.1, 1.0), "l_knee": (133.0, 463.3, 1.0),
            "r_knee": (190.6, 466.5, 1.0), "l_ankle": (118.4, 573.9, 0.95), "r_ankle": (190.5, 584.9, 0.96)}
        px = np.array([photo[n] for n in pose_decode.KPT_NAMES])
        metres_per_px = 0.48 / (345.5 - 201.5)             # their torso is the model's 0.48 m
        feet_y, mid_x = 590.0, 172.0
        for cam, bearing in (("fisheye", 0.0), ("fisheye", 50.0), ("elp_left", 120.0)):
            for dist in (2.6, 3.0, 4.0):
                b = math.radians(bearing)
                side = np.array([-math.sin(b), math.cos(b), 0.0])       # the photo's left, in the world
                pts = (np.array([dist * math.cos(b), dist * math.sin(b), 0.0])
                       + np.outer((mid_x - px[:, 0]) * metres_per_px, side)
                       + np.outer((feet_y - px[:, 1]) * metres_per_px, [0.0, 0.0, 1.0]))
                view = RIG.wide_view(cam)
                uv, ok = view.project_base(Scene().to_base(pts))
                self.assertTrue(ok.all(), (cam, dist))
                box = np.array([uv[:, 0].min() - 4, uv[:, 1].min() - 4, uv[:, 0].max() + 4, uv[:, 1].max() + 4])
                det = pose_decode.PoseDet(box, 0.9, np.column_stack([uv, px[:, 2]]))
                res = evaluate(observe(det, view, np.eye(3), 0.0, CFG), CFG)
                self.assertTrue(res.up, (cam, dist, res.why))
                self.assertEqual(res.arms, ("pass", "pass"))

    def test_side_on_needs_the_profile_exception(self):
        obs = see(at(0, 3.0, "left"), "fisheye", zoom=True)
        res = evaluate(obs, CFG)
        self.assertTrue(res.profile)
        self.assertIn("missing", res.arms)
        self.assertTrue(res.up)

    def test_one_arm_side_on(self):
        """Side-on with the near arm up and the far arm down behind the body. A pose model usually
        still guesses a hidden arm, with low confidence: a guess that hangs down refuses the signal."""
        self.assertFalse(evaluate(see(at(0, 3.0, "left", pose="left_up"), "fisheye", zoom=True), CFG).up)
        # KNOWN LIMIT: if the model returns nothing at all for the far arm, one raised arm seen
        # side-on reads as the signal (the same picture as two arms up, one behind the other).
        gone = at(0, 3.0, "left", pose="left_up", hidden=("r_elbow", "r_wrist"))
        self.assertTrue(evaluate(see(gone, "fisheye", zoom=True), CFG).up)

    def test_other_poses_are_not_the_signal(self):
        """One arm, arms out, hands on head, arms down: never, at any bearing or facing."""
        wrong = []
        for pose in ("down", "left_up", "right_up", "t", "on_head", "forward"):
            for bearing in (0, 40, 90, 140, -90):
                for facing in FACINGS:
                    obs = see(at(bearing, 3.0, facing, pose=pose), camera_for(bearing), zoom=True, noise=1.0)
                    if obs is not None and evaluate(obs, CFG).up:
                        wrong.append((pose, bearing, facing))
        self.assertEqual(wrong, [])

    def test_wide_v_and_goalpost(self):
        self.assertTrue(evaluate(see(at(0, 3.0, v_deg=35), "fisheye", zoom=True), CFG).up)       # a wide V still counts
        self.assertFalse(evaluate(see(at(0, 3.0, v_deg=60), "fisheye", zoom=True), CFG).up)      # arms mostly sideways do not
        self.assertFalse(evaluate(see(at(0, 3.0, pose="t"), "fisheye", zoom=True), CFG).up)

    def test_reaching_toward_the_robot_up_close_is_refused_by_range(self):
        """Plan, "easy to miss" 2: inside 2 m a reach looks like the signal, so nothing closer than 2 m can lock.
        (From a camera 13 cm off the floor a hand held out toward the robot sits as high in the
        picture as a raised one. Arms held out level, at the robot, still read as the signal out to
        about 2.3 m: README, "What it gets wrong".)"""
        res = evaluate(see(at(0, 1.4, pose="up"), "fisheye", zoom=True), CFG)
        self.assertFalse(res.up)
        self.assertIn("range", res.why)
        self.assertFalse(evaluate(see(at(0, 1.4, pose="forward"), "fisheye", zoom=True), CFG).up)
        self.assertFalse(evaluate(see(at(0, 2.5, pose="forward"), "fisheye", zoom=True), CFG).up)

    def test_cut_off_at_the_top_is_not_the_signal_but_is_worth_a_look(self):
        res = evaluate(see(at(140, 1.3), "elp_left"), CFG)
        self.assertFalse(res.up)
        self.assertTrue(res.undecided)                # no arm in the picture: cannot say they are down
        self.assertFalse(evaluate(see(at(140, 1.3, pose="down"), "elp_left"), CFG).undecided)

    def test_level_frame_copes_with_a_tilted_robot(self):
        scene = Scene()
        pitch = math.radians(15)
        scene.R_tilt = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])
        self.assertTrue(evaluate(see(at(90, 3.0), "elp_left", zoom=True, scene=scene), CFG).up)
        self.assertFalse(evaluate(see(at(90, 3.0, pose="t"), "elp_left", zoom=True, scene=scene), CFG).up)

    def test_filter_wants_a_held_pose(self):
        f = GestureFilter.from_config(CFG)
        for i in range(5):                            # five quick frames: not held long enough
            f.add(i * 0.05, True)
        self.assertFalse(f.confirmed(0.2))
        f = GestureFilter.from_config(CFG)
        for i in range(9):                            # 0.8 s at 10 Hz with one miss
            f.add(i * 0.1, i != 3)
        self.assertTrue(f.confirmed(0.8))
        f = GestureFilter.from_config(CFG)
        for i in range(9):                            # a throw: the arm passes through for 0.3 s
            f.add(i * 0.1, 3 <= i <= 5)
        self.assertFalse(f.confirmed(0.8))


class AppearanceTests(unittest.TestCase):
    def picture(self, shirt, trousers):
        img = np.full((400, 300, 3), 120, np.uint8)
        img[100:220, 100:200] = shirt
        img[220:330, 100:200] = trousers
        k = np.zeros((17, 3))
        K = pose_decode.K
        for name, xy in (("l_shoulder", (190, 110)), ("r_shoulder", (110, 110)), ("l_hip", (185, 215)), ("r_hip", (115, 215)),
                         ("l_knee", (185, 320)), ("r_knee", (115, 320))):
            k[K[name]] = (*xy, 0.9)
        return img, k

    def test_same_clothes_match_and_different_clothes_do_not(self):
        red = appearance.describe(*self.picture((40, 40, 200), (90, 60, 30)), 0.4)
        red2 = appearance.describe(*self.picture((50, 45, 190), (95, 60, 35)), 0.4)
        blue = appearance.describe(*self.picture((200, 80, 30), (90, 60, 30)), 0.4)
        self.assertLess(appearance.distance(red, red2), 0.2)
        self.assertGreater(appearance.distance(red, blue), float(CFG["tracker"]["appearance_max"]))
        self.assertIsNone(appearance.distance(red, None))

    def test_no_torso_no_descriptor(self):
        img, k = self.picture((40, 40, 200), (90, 60, 30))
        k[pose_decode.K["l_hip"], 2] = 0.1
        self.assertIsNone(appearance.describe(img, k, 0.4))


class DeadReckoningTests(unittest.TestCase):
    def test_a_point_stays_put_while_the_robot_walks_and_turns(self):
        dr = DeadReckoner()
        spot = dr.to_local((3.0, 0.0))
        for i in range(40):                            # 0.5 m/s for 2 s, the command repeated as the mux does
            dr.set_cmd(i * 0.05, 0.5, 0.0, 0.0)
            dr.advance((i + 1) * 0.05)
        np.testing.assert_allclose(dr.to_body(spot), (2.0, 0.0), atol=1e-6)
        for i in range(20):                            # quarter turn left in 1 s (no IMU: uses the command)
            dr.set_cmd(2.0 + i * 0.05, 0.0, 0.0, math.pi / 2)
            dr.advance(2.0 + (i + 1) * 0.05)
        np.testing.assert_allclose(dr.to_body(spot), (0.0, -2.0), atol=1e-6)
        self.assertAlmostEqual(dr.travelled, 1.0)

    def test_a_command_that_stops_arriving_stops_counting(self):
        dr = DeadReckoner()
        dr.set_cmd(0.0, 0.5, 0.0, 0.0)                 # one message, then silence (the mux has gone quiet)
        dr.advance(5.0)
        self.assertAlmostEqual(dr.x, 0.3, places=6)    # counted for 0.6 s, not 5

    def test_imu_heading_wins_over_the_commanded_turn(self):
        dr = DeadReckoner()
        dr.set_imu_yaw(0.0, 3.1)
        dr.set_cmd(0.0, 0.0, 0.0, 5.0)                 # commanded spin, but the IMU says it turned 0.1 rad (through +-pi)
        dr.set_imu_yaw(0.1, -3.0832)
        dr.advance(0.2)
        self.assertAlmostEqual(dr.yaw, 0.1, places=3)


def run(p: SurvivorPerception, scene: Scene, det, seconds: float, t0: float = 0.0, hz: float = 10.0, moving=None):
    """Feed frames and step the pipeline for a while. moving(t) may change the scene."""
    fr = frames()
    t = t0
    for _ in range(int(round(seconds * hz))):
        t += 1.0 / hz
        if moving:
            moving(t)
        p.set_imu(t, scene.quat())
        for cam, img in fr.items():
            p.set_frame(cam, t, img)
        p.step(t, det)
    return t


class PerceptionTests(unittest.TestCase):
    def lock_time(self, person):
        scene = Scene([person])
        p = SurvivorPerception(CFG)
        det = FakeDetector(scene, noise_px=1.0)
        t = run(p, scene, det, 1.0)                    # arms down for a second
        self.assertEqual(p.phase(t), SCAN)
        person.pose = "up"
        t_up = t
        for _ in range(40):
            t = run(p, scene, det, 0.1, t)
            if p.phase(t) == LOCKED:
                return t - t_up, p
        return None, p

    def test_locks_within_two_seconds_all_round(self):
        """Success criterion: locked within 2 s, every sector a camera sees whole, out to 4 m."""
        slow = []
        for name, bearing in SECTORS.items():
            for dist in (nearest(bearing), 4.0):
                for facing in ("toward", "left"):
                    dt, p = self.lock_time(at(bearing, dist, facing, pose="down"))
                    if dt is None or dt > 2.0:
                        slow.append((name, dist, facing, dt))
                    else:
                        tgt = p.target(1e9)
                        self.assertLess(abs((tgt["bearing_deg"] - bearing + 180) % 360 - 180), 5.0)
                        self.assertLess(abs(tgt["range_m"] - dist) / dist, 0.15)
        self.assertEqual(slow, [])

    def test_events_and_no_false_lock(self):
        dt, p = self.lock_time(at(30, 3.0, pose="down"))
        self.assertIsNotNone(dt)
        self.assertGreaterEqual(dt, 0.6)               # it waited for a held pose
        names = [e["event"] for e in p.pop_events()]
        self.assertEqual(names, ["candidate", "locked"])
        # A bystander waving one arm, someone stretching sideways, and a person reaching for the robot: nothing.
        scene = Scene([at(-40, 3.0, pose="left_up"), at(90, 2.5, pose="t"), at(150, 3.5, pose="forward")])
        q = SurvivorPerception(CFG)
        t = run(q, scene, FakeDetector(scene, noise_px=1.0), 6.0)
        self.assertEqual(q.phase(t), SCAN)
        self.assertEqual(q.pop_events(), [])
        self.assertEqual(q.status(t)["people"], 3)

    def test_a_throw_is_not_the_signal(self):
        """Plan, "easy to miss" 4: both arms up for a third of a second does not lock."""
        thrower = at(90, 3.5, pose="down")
        scene = Scene([thrower])
        p = SurvivorPerception(CFG)
        det = FakeDetector(scene)
        t = run(p, scene, det, 1.0)
        thrower.pose = "up"
        t = run(p, scene, det, 0.3, t)
        thrower.pose = "down"
        t = run(p, scene, det, 2.0, t)
        self.assertIsNone(p.tracker.survivor)

    def test_lock_holds_after_the_arms_come_down_and_among_other_people(self):
        survivor, other = at(20, 3.0), at(-30, 3.0, pose="down")
        scene = Scene([survivor, other])
        p = SurvivorPerception(CFG)
        det = FakeDetector(scene, noise_px=1.0)
        t = run(p, scene, det, 2.0)
        self.assertEqual(p.phase(t), LOCKED)
        sid = p.target(t)["id"]
        survivor.pose = "down"

        def walk(now):                                 # the bystander walks across in front of the survivor
            other.y = -1.7 + 0.5 * (now - t)
        t2 = run(p, scene, det, 6.0, t, moving=walk)
        tgt = p.target(t2)
        self.assertEqual(tgt["id"], sid)
        self.assertLess(abs(tgt["bearing_deg"] - 20.0), 5.0)
        self.assertEqual(p.phase(t2), LOCKED)

    def test_someone_walking_between_hides_the_survivor_without_taking_the_pin(self):
        """The bystander wears the same clothes (grey test frames) and blocks the view on the way past."""
        survivor, other = at(20, 3.0), Person(2.6 * math.cos(math.radians(20)), -1.7)
        scene = Scene([survivor, other])
        p = SurvivorPerception(CFG)
        det = FakeDetector(scene, noise_px=1.0, occlusion=True)
        t = run(p, scene, det, 2.0)
        sid = p.target(t)["id"]
        survivor.pose = "down"

        def walk(now):
            other.y = -1.7 + 0.8 * (now - t)
        t2 = run(p, scene, det, 5.5, t, moving=walk)
        tgt = p.target(t2)
        self.assertEqual(tgt["id"], sid)
        self.assertLess(abs(tgt["bearing_deg"] - 20.0), 5.0)
        self.assertTrue(tgt["visible"])

    def test_second_signaller_is_queued_not_chased(self):
        first, second = at(20, 3.0), at(100, 3.0, pose="down")
        scene = Scene([first, second])
        p = SurvivorPerception(CFG)
        det = FakeDetector(scene)
        t = run(p, scene, det, 2.0)
        sid = p.target(t)["id"]
        second.pose = "up"
        t = run(p, scene, det, 2.5, t)
        self.assertEqual(p.target(t)["id"], sid)
        self.assertEqual(len(p.status(t)["queued"]), 1)

    def test_lost_then_reacquired_and_dead_reckoned_in_between(self):
        survivor = at(0, 3.0)
        scene = Scene([survivor])
        p = SurvivorPerception(CFG)
        det = FakeDetector(scene)
        t = run(p, scene, det, 2.0)
        p.pop_events()
        scene.people = []                              # they step behind something
        p.set_cmd_vel(t, 0.5, 0.0, 0.0)                # and the robot walks 1 m toward where they were

        def advance(now):
            scene.x = 0.5 * (now - t)
            p.set_cmd_vel(now, 0.5, 0.0, 0.0)
        t2 = run(p, scene, det, 2.0, t, moving=advance)
        self.assertEqual(p.phase(t2), LOST)
        tgt = p.target(t2)
        self.assertFalse(tgt["visible"])
        self.assertAlmostEqual(tgt["range_m"], 2.0, delta=0.25)       # carried forward by dead reckoning
        self.assertGreater(tgt["sigma_m"], 0.3)
        p.set_cmd_vel(t2, 0.0, 0.0, 0.0)
        survivor.pose = "down"
        scene.people = [survivor]                      # back in view, arms down: still them
        t3 = run(p, scene, det, 1.0, t2)
        self.assertEqual(p.phase(t3), LOCKED)
        self.assertEqual([e["event"] for e in p.pop_events()], ["lost", "reacquired"])

    def test_stranger_where_the_survivor_was_is_not_taken_for_them(self):
        """Appearance: after a gap, a person in different clothes near the pin is not the survivor."""
        p = SurvivorPerception(CFG)
        survivor = at(0, 3.0)
        scene = Scene([survivor])
        det = FakeDetector(scene)
        t = run(p, scene, det, 2.0)
        tr = p.tracker.survivor
        tr.appearance = {cam: np.concatenate([np.eye(72)[3], np.eye(72)[5]]).astype(np.float32) for cam in CFG["cameras"]}
        scene.people = []
        t = run(p, scene, det, 2.0, t)
        self.assertEqual(p.phase(t), LOST)
        p._describe = lambda image, d: np.concatenate([np.eye(72)[40], np.eye(72)[41]]).astype(np.float32)
        scene.people = [at(4, 3.1, pose="down")]
        t = run(p, scene, det, 1.5, t)
        self.assertEqual(p.phase(t), LOST)
        self.assertEqual(p.status(t)["people"], 2)     # the stranger is tracked as somebody else

    def test_turn_to_look_is_requested_for_a_cut_off_person_behind(self):
        # Too close to a rear camera for the raised hands to fit, and straight behind on the seam.
        for bearing, dist in ((150, 1.4), (135, 2.0), (-120, 1.8), (180, 2.5), (180, 4.0)):
            scene = Scene([at(bearing, dist)])
            p = SurvivorPerception(CFG)
            t = run(p, scene, FakeDetector(scene), 1.5)
            st = p.status(t)
            self.assertEqual(st["phase"], SCAN, (bearing, dist))
            self.assertIsNotNone(st["look"], (bearing, dist))
            err = (st["look"]["bearing_deg"] - bearing + 180) % 360 - 180
            self.assertLess(abs(err), 12.0, (bearing, dist))
        scene.people = [at(150, 1.4, pose="down")]     # close behind with arms down: no reason to turn
        q = SurvivorPerception(CFG)
        t = run(q, scene, FakeDetector(scene), 1.5)
        self.assertIsNone(q.status(t)["look"])

    def test_budget_wide_passes_rotate_and_zoom_goes_to_raised_arms(self):
        scene = Scene([at(0, 4.0), at(120, 4.0, pose="down")])
        p = SurvivorPerception(CFG)
        det = FakeDetector(scene)
        run(p, scene, det, 3.0)
        per_s = len(det.calls) / 3.0
        self.assertLessEqual(per_s, float(CFG["budget"]["inferences_per_s"]) + 1.0)
        wide = sum(1 for c in det.calls if c != "patch")
        self.assertAlmostEqual(wide / 3.0, 10.0, delta=1.0)           # one wide pass per cycle, three cameras in turn
        self.assertGreater(det.calls.count("patch"), 15)

    def test_cancel_and_disable(self):
        scene = Scene([at(0, 3.0)])
        p = SurvivorPerception(CFG)
        det = FakeDetector(scene)
        t = run(p, scene, det, 2.0)
        self.assertEqual(p.phase(t), LOCKED)
        p.pop_events()
        p.cancel(t)
        self.assertIsNone(p.target(t))
        self.assertEqual(p.pop_events()[0]["event"], "cancelled")
        p.enable(False)
        n = len(det.calls)
        run(p, scene, det, 1.0, t)
        self.assertEqual(len(det.calls), n)


if __name__ == "__main__":
    unittest.main(verbosity=2)
