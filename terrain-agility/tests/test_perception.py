"""Perception on synthetic fisheye images (ray-cast through the robot's lens model)."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
cv2 = pytest.importorskip("cv2")

from sim.render_fisheye import BROWN, Cuboid, Scene, render  # noqa: E402
from terrain import floor_model as FM  # noqa: E402
from terrain.camera_model import CameraPose, DoubleSphere, PolarView, height_from_angles, level_from_body  # noqa: E402
from terrain.gauge import BoxGauge, fine_view, measure_box, solve_camera_mount  # noqa: E402
from terrain.scan import ScanConfig, build_scan, find_obstacles  # noqa: E402

MODEL = DoubleSphere()
POSE = CameraPose(height=0.25, pitch_deg=5.0)
SIDE = FM.ColorRange(100, 125, 120, 60)          # blue
TOP = FM.ColorRange(0, 179, 0, 190, s_max=45)    # white
BALL = FM.ColorRange(32, 50, 120, 120)


def test_lens_round_trip_and_resolution():
    u, v = np.meshgrid(np.arange(380.0, 1000.0, 53.0), np.arange(200.0, 760.0, 47.0))
    rays, ok = MODEL.unproject(u, v)
    u2, v2, ok2 = MODEL.project(rays)
    assert ok.all() and ok2.all()
    assert np.abs(u2 - u).max() < 1e-6 and np.abs(v2 - v).max() < 1e-6
    a, _, _ = MODEL.project(np.array([[0.0, 0.0, 1.0]]))
    b, _, _ = MODEL.project(np.array([[math.sin(math.radians(1)), 0.0, math.cos(math.radians(1))]]))
    assert abs(float(b[0] - a[0]) - 4.0) < 0.1          # about 4 px per degree


def test_height_formula():
    hc, D, h = 0.25, 0.4, 0.04
    foot = math.degrees(math.atan2(hc, D))
    top = math.degrees(math.atan2(hc - h, D))
    assert abs(float(height_from_angles(hc, foot, top)) - h) < 1e-9
    assert np.isinf(height_from_angles(hc, foot, -1.0))
    # a pitch error of 1 degree barely changes the height (it is a ratio)
    assert abs(float(height_from_angles(hc, foot + 1, top + 1)) - h) < 0.0012


def _floor_model(scene=None):
    img = render(scene or Scene(), MODEL, POSE)
    view = PolarView(MODEL, POSE)
    pol, valid = view.sample(img)
    hsv = cv2.cvtColor(pol, cv2.COLOR_RGB2HSV)
    fm = FM.FloorModel()
    rows = (view.depressions > 35) & (view.depressions < 55)
    cols = np.abs(view.bearings) < 20
    fm.learn(hsv[np.ix_(rows, cols)])
    return fm, view


@pytest.mark.parametrize("h,d,dist", [(0.019, 0.064, 0.30), (0.025, 0.064, 0.40), (0.030, 0.076, 0.35),
                                      (0.038, 0.089, 0.40), (0.060, 0.064, 0.45)])
def test_gauge_box(h, d, dist):
    view = fine_view(MODEL, POSE)
    scene = Scene(boxes=[Cuboid(dist, 0.02, d, 0.5, h)], seed=1)
    img = render(scene, MODEL, POSE)
    m = measure_box(img, view, SIDE, TOP)
    assert m is not None
    assert abs(m.height - h) < 0.004, m
    assert abs(m.near_x - dist) < 0.008, m
    assert abs(m.depth - d) < 0.012, m
    assert abs(math.degrees(m.skew)) < 1.5, m
    if m.ends_seen:
        assert abs(m.width - 0.5) < 0.04 and abs(m.y_centre - 0.02) < 0.02, m


def test_gauge_skew_and_tilted_body():
    view = fine_view(MODEL, POSE)
    R = level_from_body(roll=math.radians(2.0), pitch=math.radians(-3.0))
    scene = Scene(boxes=[Cuboid(0.38, 0.0, 0.064, 0.5, 0.025, yaw_deg=7.0)], seed=2)
    img = render(scene, MODEL, POSE, R_level_body=R)
    m = measure_box(img, view, SIDE, TOP, R_level_body=R)
    assert m is not None
    assert abs(m.height - 0.025) < 0.004, m
    # the box is turned 7 deg anticlockwise seen from above: its left end is nearer, skew = -7 deg
    assert abs(math.degrees(m.skew) + 7.0) < 2.0, m
    # near face on the centreline: the box's near-face centre is at x = 0.38, y = 0
    assert abs(m.near_x - 0.38) < 0.012, m


def test_gauge_aggregates_frames():
    view = fine_view(MODEL, POSE)
    g = BoxGauge(frames_needed=8)
    for k in range(10):
        img = render(Scene(boxes=[Cuboid(0.40, 0.0, 0.064, 0.5, 0.030)], seed=10 + k, noise=6.0), MODEL, POSE)
        g.add(measure_box(img, view, SIDE, TOP))
    est = g.estimate()
    assert g.ready and est.frames == 10
    assert abs(est.height - 0.030) < 0.003 and est.height_sigma < 0.003
    assert abs(est.near_x - 0.40) < 0.006


def test_scan_finds_tall_and_low_obstacles_and_ignores_ball():
    fm, view = _floor_model()
    scene = Scene(boxes=[Cuboid(0.9, 0.25, 0.3, 0.35, 0.30, side=BROWN, top=BROWN),      # tall, left
                         Cuboid(0.5, -0.45, 0.064, 0.3, 0.025)],                           # low two-colour, right
                  balls=[(0.6, 0.0, 0.037)], seed=3)
    img = render(scene, MODEL, POSE)
    pol, valid = view.sample(img)
    labels = FM.classify(cv2.cvtColor(pol, cv2.COLOR_RGB2HSV), valid, fm, SIDE, TOP, BALL)
    scan = build_scan(labels, view, ScanConfig())
    obs = find_obstacles(scan)
    assert len(obs) == 2, obs
    low, tall = sorted(obs, key=lambda o: o.height)
    assert tall.tall and not low.tall
    assert low.is_box and not tall.is_box
    assert abs(tall.nearest - (0.9 - POSE.x_forward)) < 0.06
    assert abs(low.height - 0.025) < 0.02           # coarse view; the gauge does the fine job
    # nothing where the ball is (straight ahead)
    j0 = int(np.argmin(np.abs(scan.bearings_deg)))
    assert not np.isfinite(scan.ranges[j0])


def test_scan_empty_floor_is_clear():
    fm, view = _floor_model()
    img = render(Scene(seed=5), MODEL, POSE)
    pol, valid = view.sample(img)
    labels = FM.classify(cv2.cvtColor(pol, cv2.COLOR_RGB2HSV), valid, fm, SIDE, TOP, BALL)
    scan = build_scan(labels, view, ScanConfig())
    assert not find_obstacles(scan)
    assert np.isfinite(scan.ranges).sum() == 0


def test_camera_mount_calibration_from_three_distances():
    """tools/calibrate_camera.py's solver: box of known height at known distances -> camera
    height, pitch and fore-aft position, starting from a wrong guess."""
    true_pose = CameraPose(height=0.262, pitch_deg=8.0, x_forward=0.160)
    guess = CameraPose(height=0.25, pitch_deg=0.0, x_forward=0.145)
    view = fine_view(MODEL, guess)
    obs = []
    for dist in (0.28, 0.38, 0.52):
        img = render(Scene(boxes=[Cuboid(dist, 0.0, 0.064, 0.5, 0.025)], seed=7), MODEL, true_pose)
        m = measure_box(img, view, SIDE, TOP)
        obs.append((dist, m.foot_dep, m.top_dep))
    hc, corr, xf, rms = solve_camera_mount(obs, 0.025, guess.x_forward, solve_x_forward=True)
    assert abs(hc - 0.262) < 0.005 and abs(corr - 8.0) < 0.6 and abs(xf - 0.160) < 0.006 and rms < 0.3
    hc, corr, xf, _ = solve_camera_mount(obs[:2], 0.025, 0.160)            # two distances, lens position measured by hand
    assert abs(hc - 0.262) < 0.008 and abs(corr - 8.0) < 1.0 and xf == 0.160
    with pytest.raises(ValueError):
        solve_camera_mount(obs[:1], 0.025, 0.160)
    # with the calibrated pose the gauge is right
    good = CameraPose(height=hc, pitch_deg=corr, x_forward=xf)
    img = render(Scene(boxes=[Cuboid(0.36, 0.0, 0.064, 0.5, 0.025)], seed=8), MODEL, true_pose)
    m = measure_box(img, fine_view(MODEL, good), SIDE, TOP)
    assert abs(m.height - 0.025) < 0.004 and abs(m.near_x - 0.36) < 0.010


def test_landing_strip_check():
    fm, _ = _floor_model()
    view = fine_view(MODEL, POSE)
    box = Cuboid(0.36, 0.0, 0.064, 0.5, 0.025)
    for extra, clear in (([], True),
                         ([Cuboid(0.36 + 0.064 + 0.20, 0.0, 0.1, 0.8, 0.3, side=BROWN, top=BROWN)], False),     # a wall just beyond
                         ([Cuboid(0.36 + 0.064 + 0.15, 0.05, 0.05, 0.05, 0.2, side=BROWN, top=BROWN)], False),  # a post
                         ([Cuboid(0.36 + 0.064 + 0.50, 0.0, 0.1, 0.8, 0.3, side=BROWN, top=BROWN)], True)):     # far enough
        m = measure_box(render(Scene(boxes=[box] + extra, seed=1), MODEL, POSE), view, SIDE, TOP, floor=fm)
        assert (m.landing_floor >= 0.85) == clear, (extra, m.landing_floor)


def test_scan_at_half_resolution_through_jpeg():
    """What the node does: JPEG from the camera, decoded at half size, lens model scaled."""
    fm, _ = _floor_model()
    scene = Scene(boxes=[Cuboid(0.9, 0.25, 0.3, 0.35, 0.30, side=BROWN, top=BROWN), Cuboid(0.5, -0.45, 0.064, 0.3, 0.025)], seed=3)
    ok, buf = cv2.imencode(".jpg", cv2.cvtColor(render(scene, MODEL, POSE), cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 85])
    bgr = cv2.imdecode(buf, cv2.IMREAD_REDUCED_COLOR_2)
    assert bgr.shape[:2] == (525, 700)
    view = PolarView(MODEL.scaled(700, 525), POSE)
    pol, valid = view.sample(bgr)
    labels = FM.classify(cv2.cvtColor(pol, cv2.COLOR_BGR2HSV), valid, fm, SIDE, TOP, BALL)
    obs = find_obstacles(build_scan(labels, view, ScanConfig()))
    assert len(obs) == 2, obs
    low, tall = sorted(obs, key=lambda o: o.height)
    assert tall.tall and not tall.is_box and low.is_box and not low.tall
    assert abs(tall.nearest - (0.9 - POSE.x_forward)) < 0.07
