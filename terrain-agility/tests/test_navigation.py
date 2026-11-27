"""Decision rule, tangent steering and the state machine (closed loop in sim/avoid_sim.py)."""
import math
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from sim.avoid_sim import AvoidSimConfig, Rect, run_avoid  # noqa: E402
from terrain import decision as D  # noqa: E402
from terrain.gauge import BoxEstimate  # noqa: E402
from terrain.steering import LocalMap, SteerConfig, move_points, steer  # noqa: E402
from terrain.state_machine import APPROACH, AVOID, CRUISE, GAUGE, HOLD, Inputs, TerrainFsm  # noqa: E402

ENV = {60: 0.030, 64: 0.030, 76: 0.030, 89: 0.025}


def est(**kw):
    """A low bar as the gauge reports it: 22 mm tall, 64 mm deep, seen square from 36 cm."""
    base = dict(near_x=0.36, skew=0.0, height=0.022, depth=0.064, width=0.50, y_left=0.25, y_right=-0.25,
                frames=12, height_sigma=0.001, near_sigma=0.002, ends_seen=True, height_upper=0.07, extent=0.09,
                side_seen=1.0, far_seen=True)
    base.update(kw)
    return BoxEstimate(**base)


def slab(**kw):
    """The scale-across slab as the gauge reports it."""
    return est(**{**dict(height=0.015, depth=0.38, width=0.565, y_left=0.28, y_right=-0.28, height_upper=0.16, extent=0.41), **kw})


def cfg(**kw):
    return D.DecisionConfig(envelope=dict(ENV), **kw)


def test_a_low_bar_is_stepped_over_and_a_slab_is_scaled_across():
    d = D.decide(est(), cfg())
    assert (d.action, d.method) == (D.STEP_OVER, D.BOARD) and abs(d.height_used - 0.022) < 1e-9
    d = D.decide(slab(), cfg())
    assert (d.action, d.method) == (D.SCALE_ACROSS, D.SLAB) and abs(d.height_used - 0.015) < 1e-9
    for depth in (0.20, 0.30, 0.62):
        assert D.decide(slab(depth=depth), cfg()).action == D.SCALE_ACROSS
    assert D.decide(slab(height=0.026), cfg()).action == D.SCALE_ACROSS         # 26 + 3 guard <= 30


@pytest.mark.parametrize("change,word", [
    (dict(height=0.034), "limit"),                  # too tall
    (dict(height=0.027, height_sigma=0.002), "limit"),   # tall enough that 2 sigma crosses the limit
    (dict(depth=0.120), "outside"),                 # too deep for a bar, too shallow for a slab
    (dict(depth=float("nan")), "depth"),            # far edge not seen
    (dict(height=float("nan"), depth=float("nan")), "height could not be read"),   # no side face: one colour, or round
    (dict(width=0.30, y_left=0.15, y_right=-0.15), "wide"),
    (dict(y_left=0.40, y_right=-0.10), "centred"),
    (dict(skew=math.radians(8)), "square"),
    (dict(frames=4), "frames"),
    (dict(height_sigma=0.006), "noisy"),
    (dict(near_x=0.25), "outside"),                 # too close for the counted approach
    (dict(near_x=0.50), "outside"),
    (dict(height=0.0285), "limit"),                 # 28.5 measured could be 31.5: over the 30 limit
    (dict(height=0.026, depth=0.062), "narrow"),    # what a 65 mm round bar reads as: not a foothold
    (dict(far_seen=False), "taller than the camera"),
])
def test_decision_goes_around_when_any_check_fails(change, word):
    d = D.decide(est(**change), cfg())
    assert d.action == D.GO_AROUND and any(word in r for r in d.reasons), d.reasons


@pytest.mark.parametrize("change,word", [
    (dict(height=0.0285), "limit"),
    (dict(depth=0.70), "outside"),
    (dict(width=0.38, y_left=0.19, y_right=-0.19), "wide"),      # the slab turned end-on
    (dict(height=float("nan"), depth=float("nan")), "height could not be read"),
])
def test_a_slab_is_walked_around_when_any_check_fails(change, word):
    d = D.decide(slab(**change), cfg())
    assert d.action == D.GO_AROUND and any(word in r for r in d.reasons), d.reasons


def test_envelope_check_carries_the_guard_or_the_scatter_but_the_plan_uses_the_reading():
    assert abs(D.decide(est(height=0.020), cfg()).height_used - 0.020) < 1e-9
    assert D.decide(est(height=0.0265, depth=0.076), cfg()).action == D.STEP_OVER                      # 26.5 + 3 guard <= 30
    assert D.decide(est(height=0.0265, depth=0.076, height_sigma=0.0025), cfg()).action == D.GO_AROUND   # 26.5 + 2 x 2.5 > 30


def test_decision_other_inputs():
    assert D.decide(None, cfg()).action == D.GO_AROUND
    assert D.decide(est(), cfg(), ball_tracked=True).action == D.GO_AROUND
    assert D.decide(est(), cfg(), landing_clear=False).action == D.GO_AROUND
    assert D.decide(est(), cfg(prefer_step_over=False)).action == D.GO_AROUND
    assert D.decide(est(), cfg(step_over=False)).action == D.GO_AROUND
    assert D.decide(slab(), cfg(step_over=False)).action == D.SCALE_ACROSS
    assert D.decide(slab(), cfg(scale_across=False)).action == D.GO_AROUND
    assert D.decide(est(height=0.004), cfg()).action == D.IGNORE
    # no side face, but nothing in view could be taller than 5 mm: a mark on the floor
    assert D.decide(est(height=float("nan"), depth=float("nan"), height_upper=0.005), cfg()).action == D.IGNORE
    assert D.decide(est(height=float("nan"), depth=float("nan"), height_upper=0.02), cfg()).action == D.GO_AROUND


def test_envelope_lookup_is_conservative():
    c = cfg()
    assert c.max_height(0.064) == 0.030
    assert c.max_height(0.080) == 0.025         # between 76 (30) and 89 (25): the lower
    assert c.max_height(0.050) == 0.0 and c.max_height(0.100) == 0.0
    assert cfg(height_cap=0.022).max_height(0.064) == 0.022
    assert c.crossing_for(0.089)[:2] == (D.STEP_OVER, D.BOARD) and c.crossing_for(0.38)[:2] == (D.SCALE_ACROSS, D.SLAB)
    assert c.crossing_for(0.15) is None and c.crossing_for(0.80) is None and c.crossing_for(float("nan")) is None


def test_steer_straight_when_clear_and_tangent_when_blocked():
    c = SteerConfig()
    r = steer(np.zeros((0, 2)), 0.0, c)
    assert not r.blocked and r.wz == 0.0 and abs(r.vx - c.speed) < 1e-9
    # a wall segment 0.8 m ahead, 0.4 m wide, slightly to the right: go left
    pts = np.array([[0.8, y] for y in np.arange(-0.25, 0.151, 0.02)])
    r = steer(pts, 0.0, c)
    assert r.blocked and r.side == 1 and r.heading > 0
    # the chosen heading clears the corner by the clearance, and not by much more
    corner = np.array([0.8, 0.15])
    u = np.array([math.cos(r.heading), math.sin(r.heading)])
    miss = abs(corner[0] * u[1] - corner[1] * u[0])
    assert c.clearance <= miss < c.clearance + 0.06
    # once committed to the left it stays left even if the right looks as good
    pts_sym = np.array([[0.8, y] for y in np.arange(-0.2, 0.201, 0.02)])
    assert steer(pts_sym, 0.0, c, side=-1).side == -1 and steer(pts_sym, 0.0, c, side=1).side == 1


def test_steer_reports_stuck_when_walled_in():
    ang = np.radians(np.arange(-120, 121, 3))
    pts = 0.4 * np.stack([np.cos(ang), np.sin(ang)], axis=1)
    assert steer(pts, 0.0, SteerConfig()).stuck


def test_local_map_moves_points_with_the_robot():
    m = LocalMap()
    m.add_scan(0.0, np.array([[1.0, 0.0]]), -1.7, 1.7, 2.5)
    m.move(0.5, 0.0, 0.0)
    assert np.allclose(m.points, [[0.5, 0.0]])
    m.move(0.0, 0.0, math.pi / 2)                  # turn left: the point is now on the right
    assert np.allclose(m.points, [[0.0, -0.5]], atol=1e-9)
    # out of view (behind the camera's sector) it is remembered, then forgotten
    m.add_scan(1.0, np.zeros((0, 2)), -1.0, 1.0, 2.5)
    assert len(m.points) == 1
    m.add_scan(100.0, np.zeros((0, 2)), -1.0, 1.0, 2.5)
    assert len(m.points) == 0


@pytest.mark.parametrize("offset,width,depth", [(0.0, 0.40, 0.30), (0.12, 0.40, 0.30), (-0.15, 0.60, 0.30), (0.05, 0.30, 0.60)])
def test_walk_around_is_short_and_keeps_clear(offset, width, depth):
    res = run_avoid(AvoidSimConfig(obstacles=[Rect(1.5, offset, depth, width)], seed=1))
    assert res.reached, res.line()
    assert res.min_clearance > 0.17, res.line()            # body centre to obstacle (robot half-width 0.13)
    assert res.path_length < 1.25 * res.shortest, res.line()
    assert res.longest_stop < 1.0, res.line()
    assert AVOID in res.states and res.states[-1] == CRUISE


def test_walk_around_picks_the_short_side():
    left = run_avoid(AvoidSimConfig(obstacles=[Rect(1.5, -0.15, 0.3, 0.5)], seed=2))     # obstacle mostly to the right
    right = run_avoid(AvoidSimConfig(obstacles=[Rect(1.5, 0.15, 0.3, 0.5)], seed=2))
    assert left.end_offset > 0.1 and right.end_offset < -0.1


def test_clear_floor_never_takes_over():
    res = run_avoid(AvoidSimConfig(obstacles=[Rect(1.5, 1.2, 0.3, 0.4)], seed=3))        # well off to the side
    assert res.reached and res.states == [CRUISE]


def test_close_in_front_of_a_wide_obstacle_it_turns_before_it_walks():
    """Where Pupper stands after measuring something and refusing it: 0.36 m from a face
    0.7 m wide. Walking on while turning would close in on it; it turns on the spot first."""
    ys = np.linspace(-0.35, 0.35, 36)
    face = np.stack([np.full_like(ys, 0.36), ys], axis=1)
    res = steer(face, 0.0)
    assert res.blocked and not res.stuck and res.vx == 0.0 and abs(res.wz) > 0.5
    # once turned so the nose is clear of it, it walks
    turned = move_points(face, 0.0, 0.0, math.copysign(math.radians(75.0), res.wz))
    res2 = steer(turned, -math.copysign(math.radians(75.0), res.wz), side=res.side)
    assert res2.vx > 0.0 and not res2.stuck


@pytest.mark.parametrize("width", [0.40, 0.55, 0.70, 0.90])
@pytest.mark.parametrize("offset", [-0.10, 0.0, 0.10])
def test_refused_low_obstacle_is_walked_around_from_where_it_was_measured(width, offset):
    """Something that may be low is walked up to, measured, refused (here: it cannot be
    measured), and walked around from 0.36 m away, however wide it is. It is measured once:
    it is not walked up to and measured again."""
    for seed in range(2):
        res = run_avoid(AvoidSimConfig(obstacles=[Rect(1.6 + 0.025, offset, 0.05, width, tall=False, is_box=True)],
                                       seed=seed, goal_distance=3.1, max_time=60, stop_on_hold=False))
        assert res.reached and HOLD not in res.states, (res.line(), res.states, res.events)
        assert res.min_clearance > 0.17, res.line()
        assert sum("decision:" in e for _, e in res.events) == 1, res.events
        assert res.states[:4] == [CRUISE, APPROACH, GAUGE, AVOID], res.states


def test_refused_obstacle_is_not_measured_again_after_a_hold():
    """After a HOLD (here: pretend one), the obstacle just refused is walked around, not
    approached and measured a second time; a different low obstacle elsewhere still is."""
    from terrain.scan import Obstacle
    fsm = TerrainFsm()

    def ob(x, y0, y1):
        ys = np.linspace(y0, y1, 15)
        pts = np.stack([np.full_like(ys, x), ys], axis=1)
        return Obstacle(left_deg=20.0, right_deg=-20.0, nearest=x - 0.145, nearest_bearing_deg=0.0, left_xy=tuple(pts[-1]),
                        right_xy=tuple(pts[0]), width=y1 - y0, height=0.03, tall=False, is_box=True, columns=15, points=pts)

    bar = ob(0.36, -0.35, 0.35)
    fsm.state, fsm.t_last, fsm.yaw_last = GAUGE, 0.0, 0.0
    fsm.estimate = None
    fsm._after_gauge(None, Inputs(t=0.0, yaw=0.0, obstacles=[bar], points=bar.points, intent=(0.3, 0.0, 0.0)), 0.0)
    assert fsm.state == AVOID
    fsm.state = CRUISE                                     # as after a HOLD
    out = fsm.tick(Inputs(t=0.05, yaw=0.0, obstacles=[bar], points=bar.points, intent=(0.3, 0.0, 0.0)))
    assert out.state == AVOID and "measured already" in out.event, out
    other = ob(0.80, -0.3, 0.3)                            # 0.45 m further on: a different one
    fsm2 = TerrainFsm()
    fsm2.refused, fsm2.refused_until = bar.points.copy(), 100.0
    out = fsm2.tick(Inputs(t=0.05, yaw=0.0, obstacles=[other], points=other.points, intent=(0.3, 0.0, 0.0)))
    assert out.state == APPROACH, out


def test_boxed_in_holds_instead_of_wandering():
    pocket = [Rect(0.15, 0.62, 2.4, 0.1), Rect(0.15, -0.62, 2.4, 0.1), Rect(1.3, 0.0, 0.1, 1.34)]
    res = run_avoid(AvoidSimConfig(obstacles=pocket, seed=4, max_time=40))
    assert not res.reached and HOLD in res.states
    assert res.time < 15 and res.min_clearance > 0.15
