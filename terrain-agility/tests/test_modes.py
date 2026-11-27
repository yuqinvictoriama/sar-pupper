"""The mode the mission layer sets on the state machine (full | avoid | report).

The demo's path navigation switches it by zone: full while the low obstacle is still ahead,
avoid for the tall one, report in the gauntlet (where the one tall thing ahead is the
survivor's cover and must not be walked around).
"""
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from terrain import state_machine as SM  # noqa: E402
from terrain.scan import Obstacle  # noqa: E402


def obstacle(x, half_width=0.25, is_box=False, tall=True):
    ys = np.linspace(-half_width, half_width, 11)
    pts = np.stack([np.full_like(ys, x), ys], axis=1)
    return Obstacle(left_deg=10.0, right_deg=-10.0, nearest=x - 0.145, nearest_bearing_deg=0.0,
                    left_xy=(x, half_width), right_xy=(x, -half_width), width=2 * half_width,
                    height=float("inf") if tall else 0.02, tall=tall, is_box=is_box, columns=len(pts), points=pts)


def drive(fsm, obstacles, seconds=0.5, t0=0.0):
    out = None
    for k in range(int(seconds * 20)):
        out = fsm.tick(SM.Inputs(t=t0 + 0.05 * k, yaw=0.0, obstacles=obstacles,
                                 points=np.concatenate([o.points for o in obstacles]) if obstacles else None,
                                 intent=(0.25, 0.0, 0.0)))
    return out


def test_mode_names_and_default():
    fsm = SM.TerrainFsm()
    assert fsm.mode == SM.FULL
    assert fsm.set_mode("avoid") and fsm.mode == SM.AVOID_ONLY
    assert not fsm.set_mode("sideways") and fsm.mode == SM.AVOID_ONLY


def test_full_approaches_the_box_to_measure_it():
    fsm = SM.TerrainFsm()
    out = drive(fsm, [obstacle(0.8, is_box=True, tall=False)])
    assert out.state == SM.APPROACH


def test_avoid_walks_around_the_box_like_any_obstacle():
    fsm = SM.TerrainFsm()
    fsm.set_mode(SM.AVOID_ONLY)
    out = drive(fsm, [obstacle(0.8, is_box=True, tall=False)])
    assert out.state == SM.AVOID and out.cmd is not None


def test_switching_to_avoid_drops_an_approach():
    fsm = SM.TerrainFsm()
    drive(fsm, [obstacle(0.8, is_box=True, tall=False)])
    assert fsm.state == SM.APPROACH
    fsm.set_mode(SM.AVOID_ONLY)
    assert fsm.state == SM.CRUISE


def test_report_never_steers_and_says_so():
    fsm = SM.TerrainFsm()
    fsm.set_mode(SM.REPORT_ONLY)
    out = drive(fsm, [obstacle(0.6)])
    assert out.state == SM.REPORT and out.cmd is None


def test_report_lets_go_of_an_avoid_in_progress():
    fsm = SM.TerrainFsm()
    out = drive(fsm, [obstacle(0.8)])
    assert out.state == SM.AVOID
    fsm.set_mode(SM.REPORT_ONLY)
    out = drive(fsm, [obstacle(0.8)], seconds=0.1, t0=1.0)
    assert out.state == SM.REPORT and out.cmd is None and fsm.state == SM.CRUISE


def test_report_does_not_abandon_a_crossing():
    fsm = SM.TerrainFsm()
    fsm.state = SM.STEP_OVER
    fsm.set_mode(SM.REPORT_ONLY)
    out = drive(fsm, [], seconds=0.1)
    assert out.state == SM.STEP_OVER
    fsm.crossing_result(True)
    assert drive(fsm, [], seconds=0.1, t0=1.0).state == SM.REPORT
