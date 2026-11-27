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
                                   plan_scale_across, plan_stand_test, plan_step_over)
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


# ---------------------------------------------------------------- scale-across: a slab deep enough to walk on
def counted(box, cfg=None):
    """The slab in the plan's own measure (the crawl covers a little less ground than it counts)."""
    k = 1.0 / (cfg or CrawlConfig()).advance_scale
    return BoxSpec(box.height, box.depth * k, box.near_x * k, box.skew)


def check_slab_plan(plan, box):
    """What a scale-across plan promises beyond check_plan: where every planted foot is."""
    check_plan(plan)
    cfg = plan.config
    c = counted(box, cfg)
    planted = plan.weights > 0.5
    x, y, z = plan.feet[..., 0], plan.feet[..., 1], plan.feet[..., 2]
    near, far = c.faces(y)
    on_floor = np.abs(z - K.FOOT_RADIUS) < 0.004
    on_top = np.abs(z - (box.height + K.FOOT_RADIUS)) < 0.004
    assert (on_floor | on_top)[planted].all(), "feet only ever rest on the floor or on the slab's top"
    gap = K.FOOT_ENVELOPE_RADIUS + cfg.clearance - 0.002
    assert ((x < near - gap) | (x > far + gap))[planted & on_floor].all(), "a foot on the floor is clear of the slab's faces"
    edge = cfg.top_edge_margin - 0.002
    assert ((x > near + edge) & (x < far - edge))[planted & on_top].all(), "a foot on the top is clear of its edges, rolling included"
    assert plan.feet[-1, :, 0].min() > float(np.max(far[-1])) + K.FOOT_ENVELOPE_RADIUS, "every foot ends beyond the slab"
    # every foot goes up onto the top and comes down again (two edges each)
    for leg in range(4):
        tops = [st.on_top for st in plan.steps if st.leg == leg]
        assert any(tops) and not tops[-1], f"leg {leg} uses the top and ends on the floor"
    assert all(st.clearance >= min(cfg.clearance_top, cfg.clearance_support) - 1e-4 for st in plan.steps)
    return planted, on_top


@pytest.mark.parametrize("h,d", [(0.015, 0.38), (0.010, 0.25), (0.025, 0.50), (0.030, 0.20), (0.020, 0.60)])
@pytest.mark.parametrize("near,skew", [(0.33, 0.0), (GAUGE, 3.0), (0.39, -4.0)])
def test_scale_across_plans_for_slabs_of_many_sizes(h, d, near, skew):
    box = BoxSpec(h, d, near, math.radians(skew))
    plan = plan_scale_across(box)
    planted, on_top = check_slab_plan(plan, box)
    assert plan.duration < 110.0
    advance = plan.body[-1, 0] - plan.body[0, 0]
    assert near + d < advance < near + d + 0.25, "the body ends just beyond the slab"
    if d >= 0.30:
        assert (planted & on_top).all(axis=1).any(), "at some point all four feet stand on the slab"


def test_scale_across_plan_for_the_course_slab_is_what_the_docs_say():
    plan = plan_scale_across(BoxSpec(0.015, 0.38, GAUGE))
    check_slab_plan(plan, BoxSpec(0.015, 0.38, GAUGE))
    assert 34 <= len(plan.steps) <= 50 and 55.0 < plan.duration < 90.0
    assert sum(st.on_top for st in plan.steps) >= 16
    assert max(st.torque for st in plan.steps) < 1.5 and min(st.margin for st in plan.steps) >= 0.015
    # the same slab again plans from what worked: quickly, and the same way
    again = plan_scale_across(BoxSpec(0.015, 0.38, GAUGE))
    assert len(again.steps) == len(plan.steps)


def test_scale_across_refuses_what_it_cannot_do():
    assert not plan_scale_across(BoxSpec(0.050, 0.38, GAUGE)).ok                       # too tall to climb
    assert not plan_scale_across(BoxSpec(0.015, 0.38, GAUGE, math.radians(20))).ok     # too skewed
    assert not plan_scale_across(BoxSpec(0.015, 0.09, GAUGE)).ok                       # no room on top between its edges
    assert not plan_scale_across(BoxSpec(0.015, 0.38, 0.10)).ok                        # already at it
    assert not plan_scale_across(BoxSpec(-0.01, 0.38, GAUGE)).ok


def test_stepping_over_without_touching_is_only_for_very_small_things():
    """Why a 5 cm bar is walked around: with every foot carried clear over it, a plan exists
    only for obstacles about a centimetre tall. (The shin of a foot set down beyond an
    obstacle is still over it; the taller it is, the farther the foot has to reach.)"""
    assert plan_step_over(BoxSpec(0.010, 0.010, GAUGE)).ok
    cfg = CrawlConfig(slab_max_evaluations=150)
    assert not plan_step_over(BoxSpec(0.025, 0.025, GAUGE), cfg, hip_heights=(0.160,)).ok
    assert not plan_step_over(BoxSpec(0.050, 0.050, GAUGE), cfg, hip_heights=(0.160,)).ok


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


def test_player_is_not_fooled_by_joints_that_answer_late():
    """The joints follow the plan 40 ms late (the course's robot model says about 15 ms).
    Compared with where the plan is now, a swinging joint is then well off its target; the
    player must not take that for a blocked swing, or for a foot touching down early."""
    plan = plan_crawl(0.08)
    late = []

    def lag(t, q, cmd):
        late.append(q.copy())
        return late[max(0, len(late) - 5)].copy()          # 4 ticks of 10 ms behind

    player, states = run_player(plan, block=lag)
    assert player.state == P.DONE and player.result == "done", player.events
    assert P.REVERSE not in states and not any("early" in e for _, e in player.events), player.events


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


@physics
@pytest.mark.parametrize("kw", [dict(backlash=True), dict(command_latency=0.030), dict(backlash=True, command_latency=0.015)])
def test_crossing_with_the_real_robots_play_and_delay(kw):
    """The course's own real-robot model (+-0.01 rad of play in every joint, ten times the
    joint friction) and commands that arrive late. Before 2026-11-27 a 25 ms delay already
    made a free swing read as blocked."""
    cfg = CS.SimConfig(box_height=0.019, box_depth=0.089, box_width=0.6, box_mass=0.6, near=GAUGE, extra_masses=HEAD, **kw)
    res = CS.run_crossing(cfg)
    assert res.crossed and res.max_tilt_deg < 12.0, res.line()
    assert res.box_moved < 0.004, res.line()


# ---------------------------------------------------------------- scale-across in physics
slab_physics = pytest.mark.skipif(not HAVE_MODEL, reason="needs the MuJoCo model")
SLAB = dict(box_width=0.565, scale_across=True, extra_masses=HEAD)


@slab_physics
@pytest.mark.parametrize("h,d,mass", [(0.015, 0.38, 2.0), (0.015, 0.38, None), (0.012, 0.55, 2.5), (0.025, 0.25, 1.0),
                                      (0.027, 0.45, 2.0)])
def test_scale_across_in_physics_from_the_gauge_distance(h, d, mass):
    cfg = CS.SimConfig(box_height=h, box_depth=d, near=GAUGE, box_mass=mass, **SLAB)
    res = CS.run_crossing(cfg)
    crossed_cleanly(res, cfg)
    assert abs(res.final_x - CrawlConfig().advance_scale * res.planned_x) < 0.03, "it ends where the plan, as counted, says"
    assert abs(res.final_y) < 0.05 and abs(res.final_yaw_deg) < 5.0


@slab_physics
@pytest.mark.parametrize("kw", [dict(err_near=0.010), dict(err_near=-0.010), dict(err_depth=0.015), dict(err_depth=-0.015),
                                dict(err_height=0.003), dict(err_height=-0.003), dict(skew_deg=3.0, err_skew_deg=1.5),
                                dict(planner_extra_masses=()), dict(joint_noise=0.003)])
def test_scale_across_survives_measurement_and_model_errors(kw):
    """Errors of the size the gauge and the mass estimate can have. The crossing must finish,
    or stop itself; it must never fall over or shove the slab."""
    cfg = CS.SimConfig(box_height=0.015, box_depth=0.38, near=GAUGE, box_mass=2.0, **{**SLAB, **kw})
    res = CS.run_crossing(cfg)
    assert res.planned, res.plan_reason
    assert res.max_tilt_deg < 10.0, res.line()
    assert res.crossed, res.line()
    assert res.box_moved < 0.010, res.line()
    assert res.unplanned_contact_force < 12.0, (res.line(), res.contacts)


@slab_physics
def test_a_light_slab_is_not_pushed_away():
    cfg = CS.SimConfig(box_height=0.015, box_depth=0.38, near=GAUGE, box_mass=0.4, **SLAB)
    res = CS.run_crossing(cfg)
    assert res.crossed and res.max_tilt_deg < 8.0 and res.box_moved < 0.010, res.line()



@slab_physics
@pytest.mark.parametrize("kw", [dict(backlash=True), dict(command_latency=0.030), dict(backlash=True, command_latency=0.015)])
def test_scale_across_with_the_real_robots_play_and_delay(kw):
    cfg = CS.SimConfig(box_height=0.015, box_depth=0.38, near=GAUGE, box_mass=2.0, **{**SLAB, **kw})
    res = CS.run_crossing(cfg)
    assert res.crossed and res.max_tilt_deg < 10.0, res.line()
    assert res.box_moved < 0.010, res.line()
