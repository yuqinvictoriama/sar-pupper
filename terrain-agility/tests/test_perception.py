"""Perception on synthetic fisheye images (ray-cast through the robot's lens model): the scan,
the close measurement of an obstacle, and which of the three kinds it is taken for.

The obstacles here are the ones the course uses - a white slab with a raw board edge, a
white round bar, a white carton - and others like them. Nothing about them is taught: the
only thing known beforehand is the floor's colour."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
cv2 = pytest.importorskip("cv2")

from sim.render_fisheye import BEIGE, BROWN, FLOOR, WHITE, Cuboid, Cylinder, Scene, render  # noqa: E402
from terrain import decision as D  # noqa: E402
from terrain import floor_model as FM  # noqa: E402
from terrain.camera_model import CameraPose, DoubleSphere, PolarView, height_from_angles, level_from_body  # noqa: E402
from terrain.gauge import BoxGauge, fine_view, measure_obstacle, solve_camera_mount  # noqa: E402
from terrain.scan import ScanConfig, build_scan, find_obstacles  # noqa: E402

MODEL = DoubleSphere()
POSE = CameraPose(height=0.25, pitch_deg=5.0)
BALL = FM.ColorRange(32, 50, 120, 120)
HARDWOOD = (168, 120, 72)
NEAR_DEP = 60.0


def slab(x=0.36, depth=0.38, width=0.565, height=0.015, yaw=0.0, y=0.0, side=BEIGE):
    """The scale-across obstacle: white on top, a raw board edge all round."""
    return Cuboid(x, y, depth, width, height, yaw_deg=yaw, side=side, top=WHITE)


def bar(x=0.36, depth=0.089, width=0.55, height=0.019):
    """A low bar to step over: white on top, a darker edge."""
    return Cuboid(x, 0.0, depth, width, height, side=BEIGE, top=WHITE)


def carton(x=0.36, depth=0.348, width=0.455, height=0.13):
    """The step-around obstacle: white all over, its sides a little darker than its top (light from above)."""
    return Cuboid(x, 0.0, depth, width, height, side=WHITE, top=WHITE, side_shade=0.8)


def gauge(scene, pose=POSE, floor=None, R=None, view=None, frames=1):
    floor = floor or _floor_model(scene.floor)[0]
    view = view or fine_view(MODEL, pose)
    if frames == 1:
        return measure_obstacle(render(scene, MODEL, pose, R_level_body=R), view, floor, R_level_body=R,
                                near_depression_deg=NEAR_DEP, ball=BALL)
    g = BoxGauge(frames_needed=frames)
    for k in range(frames):
        scene.seed = 100 + k
        g.add(measure_obstacle(render(scene, MODEL, pose, R_level_body=R), view, floor, R_level_body=R,
                               near_depression_deg=NEAR_DEP, ball=BALL))
    return g.estimate()


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


_FLOORS = {}


def _floor_model(floor=FLOOR):
    """The floor's colour as tools/learn_colors.py learns it: from a clear view."""
    if floor not in _FLOORS:
        view = PolarView(MODEL, POSE)
        pol, valid = view.sample(render(Scene(floor=floor), MODEL, POSE))
        hsv = cv2.cvtColor(pol, cv2.COLOR_RGB2HSV)
        fm = FM.FloorModel()
        fm.learn(hsv[np.ix_((view.depressions > 20) & (view.depressions < 58), np.abs(view.bearings) < 35)])
        _FLOORS[floor] = fm
    return _FLOORS[floor], PolarView(MODEL, POSE)


# ---------------------------------------------------------------- the close measurement
@pytest.mark.parametrize("floor", [FLOOR, HARDWOOD])
@pytest.mark.parametrize("h,d,dist", [(0.015, 0.38, 0.36), (0.012, 0.50, 0.33), (0.025, 0.25, 0.40), (0.019, 0.089, 0.36),
                                      (0.030, 0.076, 0.35), (0.060, 0.20, 0.40)])
def test_gauge_reads_height_depth_and_distance_of_a_flat_topped_obstacle(floor, h, d, dist):
    m = gauge(Scene(boxes=[slab(dist, d, 0.565, h, y=0.02)], floor=floor, seed=1))
    assert m is not None and m.side_seen > (0.9 if h <= 0.03 else 0.6) and m.far_seen
    assert abs(m.height - h) < 0.004, m
    assert abs(m.near_x - dist) < 0.008, m
    assert abs(m.depth - d) < 0.015, m
    assert abs(math.degrees(m.skew)) < 1.5, m
    assert m.height <= m.height_upper and m.depth <= m.extent + 0.005        # the limits hold
    if m.ends_seen:
        assert abs(m.width - 0.565) < 0.04 and abs(m.y_centre - 0.02) < 0.02, m


def test_gauge_needs_no_colour_contrast_only_a_side_face_that_differs_from_the_top():
    """All white, the side merely in shadow: read. Dark top, light edge: read."""
    m = gauge(Scene(boxes=[Cuboid(0.36, 0.0, 0.38, 0.565, 0.015, side=WHITE, top=WHITE, side_shade=0.8)], seed=1))
    assert abs(m.height - 0.015) < 0.004 and abs(m.depth - 0.38) < 0.015, m
    m = gauge(Scene(boxes=[Cuboid(0.36, 0.0, 0.21, 0.45, 0.025, side=(230, 225, 210), top=(40, 60, 120))], seed=1))
    assert abs(m.height - 0.025) < 0.004 and abs(m.depth - 0.21) < 0.015, m


def test_edge_in_a_floor_like_colour_is_still_found_against_the_floor_just_in_front():
    """The slab's raw edge on a wooden floor: the general floor model takes most of the edge
    for floor; the floor patch just in front of the slab tells them apart."""
    floor, _ = _floor_model(HARDWOOD)
    view = fine_view(MODEL, POSE)
    img = render(Scene(boxes=[slab()], floor=HARDWOOD, seed=1), MODEL, POSE)
    pol, valid = view.sample(img)
    edge_rows = (view.depressions > 47.6) & (view.depressions < 49.0)
    taken_for_floor = floor.is_floor(cv2.cvtColor(pol, cv2.COLOR_RGB2HSV))[np.ix_(edge_rows, np.abs(view.bearings) < 20)].mean()
    assert taken_for_floor > 0.3, "the test needs an edge the floor model is unsure about"
    m = gauge(Scene(boxes=[slab()], floor=HARDWOOD, seed=1))
    assert abs(m.height - 0.015) < 0.004 and abs(m.near_x - 0.36) < 0.008, m


def test_without_a_visible_side_face_the_height_is_not_guessed_but_limited():
    # edge exactly the floor's colour: nothing to see between floor and top
    m = gauge(Scene(boxes=[slab(side=FLOOR)], seed=1))
    assert m is not None and math.isnan(m.height) and math.isnan(m.depth) and m.side_seen < 0.3
    assert 0.38 <= m.extent < 0.42 and m.height_upper > 0.1          # could be a flat mark that deep, or a wall that tall
    # a sheet of paper: the same
    m = gauge(Scene(boxes=[Cuboid(0.36, 0.0, 0.30, 0.21, 0.0005, side=WHITE, top=WHITE)], seed=1))
    assert math.isnan(m.height) and 0.28 < m.extent < 0.31


@pytest.mark.parametrize("floor", [FLOOR, HARDWOOD])
@pytest.mark.parametrize("dia", [0.025, 0.05])
def test_round_bar_is_bounded_from_above(floor, dia):
    m = gauge(Scene(cylinders=[Cylinder(0.36, 0.0, dia, 0.70)], floor=floor, seed=1))
    assert m is not None and m.far_seen
    assert dia - 0.004 <= m.height_upper <= dia + 0.020, m            # no taller than this: true, and not far off
    assert dia - 0.004 <= m.extent <= dia + 0.035, m                  # no deeper than this
    assert 0.36 - 0.005 <= m.near_x <= 0.36 + dia, m                  # its foot is hidden under its own curve


def test_gauge_skew_and_tilted_body():
    R = level_from_body(roll=math.radians(2.0), pitch=math.radians(-3.0))
    m = gauge(Scene(boxes=[slab(0.38, yaw=7.0)], seed=2), R=R)
    assert m is not None
    assert abs(m.height - 0.015) < 0.004, m
    # the slab is turned 7 deg anticlockwise seen from above: its left end is nearer, skew = -7 deg
    assert abs(math.degrees(m.skew) + 7.0) < 2.0, m
    assert abs(m.near_x - 0.38) < 0.012, m


def test_gauge_aggregates_frames():
    scene = Scene(boxes=[slab(0.40)], noise=6.0)
    est = gauge(scene, frames=10)
    assert est.frames == 10
    assert abs(est.height - 0.015) < 0.003 and est.height_sigma < 0.003
    assert abs(est.near_x - 0.40) < 0.006 and abs(est.depth - 0.38) < 0.012
    assert est.far_seen and est.side_seen > 0.9


def test_gauge_measures_the_obstacle_ahead_not_one_beside_it():
    scene = Scene(boxes=[slab(0.36), Cuboid(0.30, 0.55, 0.2, 0.2, 0.3, side=BROWN, top=BROWN)], seed=4)
    m = gauge(scene)
    assert abs(m.near_x - 0.36) < 0.008 and abs(m.height - 0.015) < 0.004, m


def test_ball_is_not_an_obstacle_for_the_gauge():
    m = gauge(Scene(balls=[(0.45, 0.0, 0.037)], seed=1))
    assert m is None


# ---------------------------------------------------------------- the three kinds
def classify(scene, frames=10):
    return D.decide(gauge(scene, frames=frames))


@pytest.mark.parametrize("floor", [FLOOR, HARDWOOD])
def test_the_course_obstacles_are_told_apart(floor):
    """The slab is scaled across, the round bar and the carton are walked around (the bar is
    too big to step over and round, so no foothold), and a low flat bar is stepped over."""
    d = classify(Scene(boxes=[slab()], floor=floor))
    assert (d.action, d.method) == (D.SCALE_ACROSS, D.SLAB), d
    assert abs(d.height_used - 0.015) < 0.004
    d = classify(Scene(boxes=[bar()], floor=floor))
    assert (d.action, d.method) == (D.STEP_OVER, D.BOARD), d
    d = classify(Scene(cylinders=[Cylinder(0.36, 0.0, 0.05, 0.70)], floor=floor))
    assert d.action == D.GO_AROUND, d
    d = classify(Scene(boxes=[carton()], floor=floor))
    assert d.action == D.GO_AROUND, d
    d = classify(Scene(boxes=[carton(depth=0.13, height=0.348)], floor=floor))          # standing on its side
    assert d.action == D.GO_AROUND and any("taller than the camera" in r for r in d.reasons), d


@pytest.mark.parametrize("scene,word", [
    (Scene(boxes=[slab(height=0.040)]), "limit"),                               # a slab too tall to climb
    (Scene(boxes=[Cuboid(0.36, 0.0, 0.235, 0.5, 0.07, side=WHITE, top=WHITE, side_shade=0.8)]), "limit"),   # a block that covers as much floor as the slab
    (Scene(boxes=[slab(depth=0.565, width=0.38)]), "wide"),                      # the slab turned end-on: too narrow to walk on
    (Scene(boxes=[slab(side=FLOOR)]), "height could not be read"),              # nothing to read the height from
    (Scene(boxes=[slab(depth=0.15)]), "depth"),                                 # between a bar and a slab: no crossing built
    (Scene(boxes=[slab(yaw=9.0)]), "square"),
    (Scene(boxes=[slab(), Cuboid(0.36 + 0.38 + 0.15, 0.0, 0.1, 0.8, 0.3, side=BROWN, top=BROWN)]), "clear floor"),
])
def test_what_is_walked_around_and_why(scene, word):
    est = gauge(scene, frames=10)
    d = D.decide(est, landing_clear=not (est.landing_floor < 0.85))
    assert d.action == D.GO_AROUND and any(word in r for r in d.reasons), d


# ---------------------------------------------------------------- the scan (while walking)
def scan_of(scene, half=False):
    fm, view = _floor_model(scene.floor)
    img = render(scene, MODEL, POSE)
    if half:        # what the node does: JPEG from the camera, decoded at half size, lens model scaled
        ok, buf = cv2.imencode(".jpg", cv2.cvtColor(img, cv2.COLOR_RGB2BGR), [cv2.IMWRITE_JPEG_QUALITY, 85])
        bgr = cv2.imdecode(buf, cv2.IMREAD_REDUCED_COLOR_2)
        assert bgr.shape[:2] == (525, 700)
        view = PolarView(MODEL.scaled(700, 525), POSE)
        pol, valid = view.sample(bgr)
        hsv = cv2.cvtColor(pol, cv2.COLOR_BGR2HSV)
    else:
        pol, valid = view.sample(img)
        hsv = cv2.cvtColor(pol, cv2.COLOR_RGB2HSV)
    scan = build_scan(FM.classify(hsv, valid, fm, ball=BALL), view, ScanConfig())
    return scan, find_obstacles(scan)


@pytest.mark.parametrize("half", [False, True])
def test_scan_tells_certainly_tall_from_may_be_low(half):
    scan, obs = scan_of(Scene(boxes=[Cuboid(0.9, 0.45, 0.3, 0.35, 0.30, side=BROWN, top=BROWN),       # tall, left
                                     slab(0.6, 0.20, 0.35, 0.015, y=-0.45)],                           # low, right
                              balls=[(0.6, 0.0, 0.037)], seed=3), half)
    assert len(obs) == 2, obs
    low, tall = sorted(obs, key=lambda o: o.height)
    assert tall.tall and not tall.is_box and low.is_box and not low.tall
    assert abs(tall.nearest - (0.9 - POSE.x_forward)) < 0.07
    assert 0.015 <= low.height < 0.12, "a limit from above, never below the truth"
    # nothing where the ball is (straight ahead)
    j0 = int(np.argmin(np.abs(scan.bearings_deg)))
    assert not np.isfinite(scan.ranges[j0])


@pytest.mark.parametrize("dist", [1.2, 0.9])
def test_from_a_distance_the_carton_is_tall_and_the_slab_and_bar_are_not(dist):
    """What decides whether Pupper curves round from afar or walks up to measure."""
    for scene, tall in ((Scene(boxes=[carton(dist)], seed=3), True),
                        (Scene(boxes=[carton(dist, depth=0.13, height=0.348)], seed=3), True),
                        (Scene(boxes=[slab(dist)], seed=3), False),
                        (Scene(boxes=[slab(dist, depth=0.60)], seed=3), False),
                        (Scene(boxes=[bar(dist)], seed=3), False),
                        (Scene(cylinders=[Cylinder(dist, 0.0, 0.05, 0.70)], seed=3), False)):
        _, obs = scan_of(scene, half=True)
        ahead = [o for o in obs if abs(o.nearest_bearing_deg) < 40]
        assert ahead and all(o.tall == tall for o in ahead), (scene.boxes or scene.cylinders, ahead)


def test_scan_empty_floor_is_clear():
    scan, obs = scan_of(Scene(seed=5))
    assert not obs
    assert np.isfinite(scan.ranges).sum() == 0


# ---------------------------------------------------------------- calibration, landing strip
def test_camera_mount_calibration_from_three_distances():
    """tools/calibrate_camera.py's solver: an obstacle of known height at known distances ->
    camera height, pitch and fore-aft position, starting from a wrong guess."""
    true_pose = CameraPose(height=0.262, pitch_deg=8.0, x_forward=0.160)
    guess = CameraPose(height=0.25, pitch_deg=0.0, x_forward=0.145)
    floor, _ = _floor_model()
    view = fine_view(MODEL, guess)
    obs = []
    for dist in (0.28, 0.38, 0.52):
        img = render(Scene(boxes=[bar(dist, height=0.025)], seed=7), MODEL, true_pose)
        m = measure_obstacle(img, view, floor)
        obs.append((dist, m.foot_dep, m.top_dep))
    hc, corr, xf, rms = solve_camera_mount(obs, 0.025, guess.x_forward, solve_x_forward=True)
    assert abs(hc - 0.262) < 0.005 and abs(corr - 8.0) < 0.6 and abs(xf - 0.160) < 0.006 and rms < 0.3
    hc, corr, xf, _ = solve_camera_mount(obs[:2], 0.025, 0.160)            # two distances, lens position measured by hand
    assert abs(hc - 0.262) < 0.008 and abs(corr - 8.0) < 1.0 and xf == 0.160
    with pytest.raises(ValueError):
        solve_camera_mount(obs[:1], 0.025, 0.160)
    # with the calibrated pose the gauge is right
    good = CameraPose(height=hc, pitch_deg=corr, x_forward=xf)
    img = render(Scene(boxes=[slab()], seed=8), MODEL, true_pose)
    m = measure_obstacle(img, fine_view(MODEL, good), floor)
    assert abs(m.height - 0.015) < 0.004 and abs(m.near_x - 0.36) < 0.010


@pytest.mark.parametrize("floor", [FLOOR, HARDWOOD])
def test_camera_mount_calibration_far_from_the_first_guess_using_the_far_edge(floor):
    """As it was on the robot: the lens 9 cm lower than first guessed and looking 11 degrees
    up, the slab's raw edge on a wooden floor. The slab at three distances, its far edge used
    too -> the pose, and with it the slab reads right from where a crossing starts."""
    true_pose = CameraPose(height=0.164, pitch_deg=-11.5, x_forward=0.150)
    guess = CameraPose(height=0.25, pitch_deg=0.0, x_forward=0.145)
    fm, _ = _floor_model(floor)
    view = fine_view(MODEL, guess)
    obs = []
    for dist in (0.337, 0.437, 0.587):          # 25, 35 and 50 cm from the toes
        m = measure_obstacle(render(Scene(boxes=[slab(dist)], floor=floor, seed=7), MODEL, true_pose), view, fm,
                             near_depression_deg=NEAR_DEP)
        assert m is not None and all(math.isfinite(v) for v in (m.foot_dep, m.top_dep, m.far_dep)), (dist, m)
        obs.append((dist, m.foot_dep, m.top_dep, m.far_dep))
    details = {}
    hc, corr, xf, rms = solve_camera_mount(obs, 0.015, guess.x_forward, solve_x_forward=True, box_depth=0.38, details=details)
    assert abs(hc - 0.164) < 0.005 and abs(corr + 11.5) < 0.6 and abs(xf - 0.150) < 0.008 and rms < 0.3, (hc, corr, xf, rms)
    assert details["residuals"].shape == (3, 3) and np.abs(details["residuals"]).max() < 0.6
    # one distance measured 4 cm wrong shows as a poor fit, and as the worst row
    bad = [obs[0], (obs[1][0] + 0.04,) + obs[1][1:], obs[2]]
    *_, rms_bad = solve_camera_mount(bad, 0.015, guess.x_forward, solve_x_forward=True, box_depth=0.38, details=details)
    assert rms_bad > 0.35 and int(np.nanargmax(np.abs(details["residuals"]).max(axis=1))) == 1
    good = CameraPose(height=hc, pitch_deg=corr, x_forward=xf)
    m = measure_obstacle(render(Scene(boxes=[slab(0.36)], floor=floor, seed=8), MODEL, true_pose), fine_view(MODEL, good), fm,
                         near_depression_deg=NEAR_DEP)
    assert abs(m.height - 0.015) < 0.003 and abs(m.near_x - 0.36) < 0.010 and abs(m.depth - 0.38) < 0.015, m


def test_landing_strip_check():
    far = 0.36 + 0.38
    for extra, clear in (([], True),
                         ([Cuboid(far + 0.20, 0.0, 0.1, 0.8, 0.3, side=BROWN, top=BROWN)], False),     # a wall just beyond
                         ([Cuboid(far + 0.15, 0.05, 0.05, 0.05, 0.2, side=BROWN, top=BROWN)], False),  # a post
                         ([Cuboid(far + 0.50, 0.0, 0.1, 0.8, 0.3, side=BROWN, top=BROWN)], True)):     # far enough
        m = gauge(Scene(boxes=[slab()] + extra, seed=1))
        assert (m.landing_floor >= 0.85) == clear, (extra, m.landing_floor)
