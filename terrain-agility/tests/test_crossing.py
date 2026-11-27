"""The step-over crawl: planner, trajectory player, hand-over, and whole crossings in physics.

The physics tests use the MuJoCo model from pupper_v3_description with the legs' measured
envelope added (sim/crossing_sim.py) and are skipped where MuJoCo or the model is missing.
"""
import math
import sys
from dataclasses import replace
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from terrain import handover as H  # noqa: E402
from terrain import kinematics as K  # noqa: E402
from terrain import player as P  # noqa: E402
from terrain.crawl_planner import (BoxSpec, CrawlConfig, load_templates, plan_crawl, plan_crossing,  # noqa: E402
                                   plan_stand_test)
from terrain.decision import DecisionConfig  # noqa: E402

TEMPLATES = load_templates()
needs_templates = pytest.mark.skipif(not TEMPLATES, reason="config/crawl_templates.json not built")
GAUGE = 0.36          # where the robot stops to measure (body centre to the near face)


def check_plan(plan, cfg=None):
    """Everything a plan promises, checked sample by sample."""
    cfg = cfg or plan.config
    assert plan.ok, plan.reason
    lim = np.concatenate([K.leg_limits(leg) for leg in K.LEGS], axis=0)
    for q in (plan.q_des, plan.q_cmd):
        assert np.all(q >= lim[:, 0] - 1e-6) and np.all(q <= lim[:, 1] + 1e-6), "inside the hardware joint limits"
    for i, leg in enumerate(K.LEGS):
        assert K.mic_bar_safe(leg, plan.q_des[:, 3 * i:3 * i + 3]).all(), f"{leg} clear of the mic bars"
    assert np.abs(plan.q_des[0] - K.DEFAULT_JOINT_POS).max() < 0.02, "starts in the standing pose"
    assert np.abs(plan.q_des[-1] - K.DEFAULT_JOINT_POS).max() < 0.08, "ends in the standing pose"
    assert np.all(np.diff(plan.t) > 0)
    speed = np.abs(np.diff(plan.q_des, axis=0)).max(axis=1) / np.diff(plan.t)
    assert speed.max() <= cfg.max_joint_speed + 1e-6, f"joint speed {speed.max():.1f} rad/s"
    assert (plan.weights.sum(axis=1) >= 3).all(), "never fewer than three feet down"
    # (the footstep search checks the margin at each step's key poses; the finished trajectory,
    #  with the feet's rolling and the transitions in it, may come out a little under that)
    assert plan.margin.min() >= CrawlConfig().min_margin - 0.003, f"support margin {plan.margin.min() * 1000:.1f} mm"
    assert np.abs(plan.q_cmd - plan.q_des).max() < 0.45, "load compensation stays modest"
    for s in plan.steps:
        assert s.torque <= cfg.torque_limit + 1e-6


# ---------------------------------------------------------------- planner
def test_stand_and_crawl_plans():
    stand = plan_stand_test(hold=1.0)
    check_plan(stand)
    assert not stand.steps and 2.0 < stand.duration < 6.0
    for dist in (0.05, 0.08, 0.24):
        crawl = plan_crawl(dist)
        check_plan(crawl)
        moved = crawl.feet[-1, :, 0] - crawl.feet[0, :, 0]
        assert np.abs(moved - dist).max() < 0.012, moved
    assert not plan_crawl(0.005).ok


@needs_templates
@pytest.mark.parametrize("h,d", [(0.019, 0.089), (0.025, 0.089), (0.022, 0.076), (0.028, 0.095), (0.019, 0.064)])
@pytest.mark.parametrize("near", [0.33, GAUGE, 0.39])
def test_crossing_plans_from_the_gauge_distance(h, d, near):
    plan = plan_crossing(BoxSpec(h, d, near), search=False)
    check_plan(plan)
    cfg = plan.config
    box_far = near + d
    assert plan.feet[-1, :, 0].min() > box_far + K.FOOT_RADIUS, "every foot ends beyond the box"
    assert all(s.clearance >= min(cfg.clearance_top, cfg.clearance_support) - 1e-4 for s in plan.steps)
    assert plan.duration < 75.0
    # feet only ever rest on the floor or on the box top
    planted = plan.weights > 0.5
    z = plan.feet[..., 2][planted]
    on_floor = np.abs(z - K.FOOT_RADIUS) < 0.004
    on_top = np.abs(z - (h + K.FOOT_RADIUS)) < 0.004
    assert (on_floor | on_top).all()


@needs_templates
def test_envelope_file_matches_what_can_be_planned():
    env = DecisionConfig().envelope
    assert env, "planning envelope missing from crawl_templates.json"
    for depth_mm, h_max in env.items():
        if h_max <= 0:
            continue
        for near in (0.34, GAUGE, 0.38):
            plan = plan_crossing(BoxSpec(h_max, depth_mm / 1000.0, near), search=False)
            assert plan.ok, f"{h_max * 1000:.0f} x {depth_mm} mm from {near}: {plan.reason}"


def test_planner_refuses_what_it_cannot_do():
    assert not plan_crossing(BoxSpec(0.060, 0.064, GAUGE), search=False).ok           # too tall
    assert not plan_crossing(BoxSpec(0.025, 0.089, GAUGE, math.radians(20)), search=False).ok   # too skewed
    assert not plan_crossing(BoxSpec(-0.01, 0.064, GAUGE), search=False).ok
    assert not plan_crossing(BoxSpec(0.025, 0.089, GAUGE), templates=[], search=False).ok       # no pattern, no search


# ---------------------------------------------------------------- player and hand-over
def run_player(plan, cfg=None, tilt_at=None, block=None, dt=0.01):
    """Play a plan against a 'robot' that follows commands perfectly, except where told."""
    player = P.TrajectoryPlayer(plan, cfg)
    t, q = 0.0, K.DEFAULT_JOINT_POS.copy()
    player.start(t, q)
    states = []
    while not player.finished and t < plan.duration + 20:
        tilt = math.radians(30) if tilt_at is not None and t > tilt_at else 0.0
        cmd = player.update(t, q, tilt)
        states.append(cmd.state)
        i = min(len(plan.t) - 1, int(np.searchsorted(plan.t, player._tp)))
        q = plan.q_des[i].copy() if cmd.state in (P.RUN, P.REVERSE, P.DONE) else cmd.q.copy()
        if block is not None:
            q = block(t, q, cmd)
        t += dt
    return player, states


def test_player_follows_a_plan_to_the_end():
    player, states = run_player(plan_crawl(0.08))
    assert player.state == P.DONE and player.result == "done"
    assert states[0] == P.BLEND and P.RUN in states


def test_player_aborts_softly_on_tilt():
    plan = plan_crawl(0.08)
    player, _ = run_player(plan, tilt_at=3.0)
    assert player.state == P.STOPPED and "tilt" in player.result
    cmd = player.update(100.0, K.DEFAULT_JOINT_POS, 0.0)
    assert cmd.kp == player.cfg.settle_kp and cmd.kd == player.cfg.settle_kd     # soft joints, not a hold


def test_player_backs_a_blocked_swing_out():
    plan = plan_crawl(0.08)
    held = {}

    def block(t, q, cmd):
        # the first swinging leg stops dead half-way through its swing (it has hit something)
        if cmd.swing_leg >= 0 and cmd.phase.startswith("swing") and cmd.state == P.RUN:
            sl = slice(3 * cmd.swing_leg, 3 * cmd.swing_leg + 3)
            if "leg" not in held and cmd.progress > 0.0:
                held["leg"], held["q"], held["t"] = cmd.swing_leg, q[sl].copy(), t
            if held.get("leg") == cmd.swing_leg and t - held["t"] > 0.25:
                q[sl] = held["q"] if "frozen" in held else q[sl]
                held.setdefault("frozen", q[sl].copy())
                q[sl] = held["frozen"]
        elif "frozen" in held and cmd.state == P.REVERSE:
            pass                                    # free again on the way back
        return q

    player, states = run_player(plan, block=block)
    assert P.REVERSE in states, "it backs out along the way it came"
    assert player.state == P.STOPPED and "blocked" in player.result


def test_handover_sequence_and_estop():
    plan = plan_stand_test(hold=0.5)
    hand = H.Handover(P.TrajectoryPlayer(plan))
    q = K.DEFAULT_JOINT_POS.copy()
    c = hand.update(0.00, q, 0.0, switched=False)
    assert c.stage == H.PRE and c.kp == 0.0 and c.kd > 0.3                 # damping only until the switch is through
    c = hand.update(0.02, q, 0.0, switched=True)
    assert c.stage == H.RAMP and c.kp < 1.0
    c = hand.update(0.05, q, 0.0, switched=True)
    assert 0.0 < c.kp < 7.5
    c = hand.update(0.10, q, 0.0, switched=True)
    assert c.kp == pytest.approx(7.5)
    c = hand.update(0.12, q, 0.0, switched=True)
    assert hand.stage == H.PLAY
    c = hand.update(0.50, q, 0.0, switched=True, estop=True)
    assert c.stage == H.LIMP and c.kp == 0.0 and c.kd == pytest.approx(0.1)
    c = hand.update(0.60, q, 0.0, switched=True)                           # and it stays limp
    assert c.stage == H.LIMP and hand.finished and "e-stop" in hand.result


def test_handover_gives_up_if_the_switch_never_happens():
    hand = H.Handover(P.TrajectoryPlayer(plan_stand_test()))
    q = K.DEFAULT_JOINT_POS.copy()
    for k in range(300):
        c = hand.update(0.01 * k, q, 0.0, switched=False)
        assert c.kp == 0.0
    assert hand.stage == H.LIMP


# ---------------------------------------------------------------- physics
mujoco = pytest.importorskip("mujoco")
from sim import crossing_sim as CS  # noqa: E402

try:
    CS.find_model()
    HAVE_MODEL = True
except FileNotFoundError:
    HAVE_MODEL = False
physics = pytest.mark.skipif(not (HAVE_MODEL and TEMPLATES), reason="needs the MuJoCo model and the footstep templates")
HEAD = ((0.13, [0.055, 0.0, 0.13]), (0.065, [-0.014, 0.0, 0.078]))      # camera plate + mics, as in terrain.yaml


def crossed_cleanly(res, cfg):
    assert res.planned, res.plan_reason
    assert res.crossed, res.line()
    assert res.max_tilt_deg < 8.0, res.line()
    assert res.box_moved < 0.004, res.line()
    assert res.unplanned_contact_force < 2.0, (res.line(), res.contacts)
    assert res.max_torque < 3.0
    assert res.handover_sag < 0.006


@physics
@pytest.mark.parametrize("h,d", [(0.019, 0.089), (0.025, 0.089), (0.022, 0.076), (0.019, 0.064)])
def test_crossing_in_physics_from_the_gauge_distance(h, d):
    cfg = CS.SimConfig(box_height=h, box_depth=d, near=GAUGE, extra_masses=HEAD)
    crossed_cleanly(CS.run_crossing(cfg), cfg)


@physics
@pytest.mark.parametrize("kw", [dict(err_near=0.008), dict(err_near=-0.008), dict(err_height=0.003), dict(err_height=-0.002),
                                dict(skew_deg=3.0), dict(planner_extra_masses=()), dict(joint_noise=0.003)])
def test_crossing_survives_measurement_and_model_errors(kw):
    """Errors of the size the gauge and the mass estimate can have. The crossing must finish,
    or stop itself; it must never fall over or shove the box."""
    cfg = CS.SimConfig(box_height=0.022, box_depth=0.089, near=GAUGE, extra_masses=HEAD, **kw)
    res = CS.run_crossing(cfg)
    assert res.planned, res.plan_reason
    assert res.max_tilt_deg < 12.0, res.line()
    assert res.crossed or res.result != "done", res.line()
    assert res.box_moved < 0.015, res.line()
    if kw.get("err_near", 0.0) == 0.0 and "skew_deg" not in kw:
        assert res.crossed, res.line()


@physics
def test_fixed_and_light_boxes():
    for mass in (None, 0.25):
        cfg = CS.SimConfig(box_height=0.025, box_depth=0.089, near=GAUGE, box_mass=mass, extra_masses=HEAD)
        res = CS.run_crossing(cfg)
        assert res.crossed and res.max_tilt_deg < 8.0, (mass, res.line())
