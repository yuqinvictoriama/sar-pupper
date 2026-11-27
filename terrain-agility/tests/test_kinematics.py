"""Leg kinematics: FK against MuJoCo (when available), IK by round trip, CoM against MuJoCo."""
import os
import sys
from pathlib import Path

import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from terrain import kinematics as K  # noqa: E402

MODEL_XML = os.environ.get(
    "PUPPER_XML", "/home/claude/ext/pupper_v3_description/description/mujoco_xml/pupper_v3_complete.position.xml")


def test_default_stance_numbers():
    h, feet = K.default_stance()
    assert abs(h - 0.1614) < 5e-4
    assert np.allclose(feet[0], [0.0657, -0.1015, -0.1414], atol=5e-4)
    assert np.allclose(feet[3], [-0.0843, 0.0905, -0.1414], atol=5e-4)
    assert abs(K.TOTAL_MASS - 3.17) < 0.01
    assert abs(K.LEG_MAX_REACH - 0.1734) < 5e-4


def test_ik_round_trip_all_legs():
    """On the standing branch (knee behind the hip-foot line, abduction near zero) the
    closed-form IK must return the joint angles that produced the foot position."""
    rng = np.random.default_rng(0)
    for leg in K.LEGS:
        lim = K.leg_limits(leg)
        side = K.leg_model(leg).side
        n = 4000
        q = np.zeros((n, 3))
        q[:, 0] = rng.uniform(-0.45, 1.2, n) * (-side)
        q[:, 1] = rng.uniform(-0.3, 0.6, n) * (-side)
        # knee: from just short of straight (K._PHI, right-leg sign) to nearly folded
        q[:, 2] = rng.uniform(K._PHI + 0.05, 0.55, n) * (-side)
        p = K.foot_position(leg, q)
        q2, ok = K.leg_ik(leg, p)
        err = np.linalg.norm(K.foot_position(leg, q2) - p, axis=1)
        assert ok.mean() > 0.999, (leg, ok.mean())
        assert err[ok].max() < 5e-6, (leg, err[ok].max())
        assert np.abs(q2 - q)[ok].max() < 1e-3, leg


def test_ik_default_pose():
    feet = K.all_feet(K.DEFAULT_JOINT_POS)
    for i, leg in enumerate(K.LEGS):
        q, ok = K.leg_ik(leg, feet[i])
        assert ok
        assert np.allclose(q, K.DEFAULT_JOINT_POS[3 * i:3 * i + 3], atol=1e-4)


def test_ik_unreachable():
    q, ok = K.leg_ik("front_r", K.hip_position("front_r") + np.array([0.0, -0.02, -0.25]))
    assert not ok


@pytest.mark.skipif(not Path(MODEL_XML).exists(), reason="MuJoCo model not available")
def test_fk_and_com_match_mujoco():
    mujoco = pytest.importorskip("mujoco")
    m = mujoco.MjModel.from_xml_path(MODEL_XML)
    d = mujoco.MjData(m)
    rng = np.random.default_rng(1)
    for _ in range(50):
        q = np.array([rng.uniform(*m.jnt_range[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]) for n in K.JOINT_NAMES])
        d.qpos[:] = 0
        d.qpos[3] = 1
        for n, v in zip(K.JOINT_NAMES, q):
            d.qpos[m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)]] = v
        mujoco.mj_kinematics(m, d)
        mujoco.mj_comPos(m, d)
        for i, leg in enumerate(K.LEGS):
            pts = K.leg_points(leg, q[3 * i:3 * i + 3])
            foot = d.site_xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, f"leg_{leg}_3_foot_site")]
            knee = d.xpos[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, f"leg_{leg}_3")]
            assert np.allclose(pts["foot"], foot, atol=1e-6)
            assert np.allclose(pts["knee"], knee, atol=1e-6)
        assert np.allclose(K.center_of_mass(q), d.subtree_com[1], atol=1e-6)
