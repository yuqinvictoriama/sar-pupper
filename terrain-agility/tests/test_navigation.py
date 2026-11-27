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
from terrain.steering import LocalMap, SteerConfig, steer  # noqa: E402
from terrain.state_machine import AVOID, CRUISE, HOLD  # noqa: E402

ENV = {60: 0.030, 64: 0.030, 76: 0.030, 89: 0.025}


def est(**kw):
    base = dict(near_x=0.36, skew=0.0, height=0.022, depth=0.064, width=0.50, y_left=0.25, y_right=-0.25,
                frames=12, height_sigma=0.001, near_sigma=0.002, ends_seen=True)
    base.update(kw)
    return BoxEstimate(**base)


def cfg(**kw):
    return D.DecisionConfig(envelope=dict(ENV), **kw)


def test_decision_steps_over_a_good_box():
    d = D.decide(est(), cfg())
    assert d.action == D.STEP_OVER and abs(d.height_used - 0.022) < 1e-9


@pytest.mark.parametrize("change,word", [
    (dict(height=0.034), "limit"),                  # too tall
    (dict(height=0.028, height_sigma=0.002), "limit"),   # tall enough that 2 sigma crosses the limit
    (dict(depth=0.120), "outside"),                 # too deep
    (dict(depth=float("nan")), "depth"),            # one-colour box: depth unknown
    (dict(width=0.30, y_left=0.15, y_right=-0.15), "wide"),
    (dict(y_left=0.40, y_right=-0.10), "centred"),
    (dict(skew=math.radians(8)), "square"),
    (dict(frames=4), "frames"),
    (dict(height_sigma=0.006), "noisy"),
    (dict(near_x=0.25), "outside"),                 # too close for the counted approach
    (dict(near_x=0.50), "outside"),
    (dict(height=0.0285), "limit"),                 # 28.5 measured could be 31.5: over the 30 limit
])
def test_decision_goes_around_when_any_check_fails(change, word):
    d = D.decide(est(**change), cfg())
    assert d.action == D.GO_AROUND and any(word in r for r in d.reasons), d.reasons


def test_envelope_check_carries_the_guard_or_the_scatter_but_the_plan_uses_the_reading():
    assert abs(D.decide(est(height=0.020), cfg()).height_used - 0.020) < 1e-9
    assert D.decide(est(height=0.0265), cfg()).action == D.STEP_OVER                      # 26.5 + 3 guard <= 30
    assert D.decide(est(height=0.0265, height_sigma=0.0025), cfg()).action == D.GO_AROUND   # 26.5 + 2 x 2.5 > 30


def test_decision_other_inputs():
    assert D.decide(None, cfg()).action == D.GO_AROUND
    assert D.decide(est(), cfg(), ball_tracked=True).action == D.GO_AROUND
    assert D.decide(est(), cfg(), landing_clear=False).action == D.GO_AROUND
    assert D.decide(est(), cfg(prefer_step_over=False)).action == D.GO_AROUND
    assert D.decide(est(height=0.004), cfg()).action == D.IGNORE


def test_envelope_lookup_is_conservative():
    c = cfg()
    assert c.max_height(0.064) == 0.030
    assert c.max_height(0.080) == 0.025         # between 76 (30) and 89 (25): the lower
    assert c.max_height(0.050) == 0.0 and c.max_height(0.100) == 0.0
    assert cfg(height_cap=0.022).max_height(0.064) == 0.022


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


def test_boxed_in_holds_instead_of_wandering():
    pocket = [Rect(0.15, 0.62, 2.4, 0.1), Rect(0.15, -0.62, 2.4, 0.1), Rect(1.3, 0.0, 0.1, 1.34)]
    res = run_avoid(AvoidSimConfig(obstacles=pocket, seed=4, max_time=40))
    assert not res.reached and HOLD in res.states
    assert res.time < 15 and res.min_clearance > 0.15
