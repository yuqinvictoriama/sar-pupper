"""Static balance and load: support margin, contact forces, joint torques, sag compensation,
and leg-to-box clearance. Pure numpy; everything broadcasts over leading dimensions so the
crawl planner can evaluate a whole grid of candidate body positions at once.
"""

from __future__ import annotations

import numpy as np

from . import kinematics as K

# Stance feet in counter-clockwise order seen from above (x forward, y left):
# FL(1) -> BL(3) -> BR(2) -> FR(0)
_CCW = (1, 3, 2, 0)


def support_margin(com_xy, feet_xy, stance):
    """Distance from the centre of mass (ground projection) to the nearest edge of the
    support polygon; positive inside, negative outside.

    com_xy: (..., 2).  feet_xy: (4, 2) or (..., 4, 2).  stance: 4 booleans (which feet carry).
    """
    com_xy = np.asarray(com_xy, dtype=float)
    feet_xy = np.asarray(feet_xy, dtype=float)
    order = [i for i in _CCW if stance[i]]
    if len(order) < 3:
        return np.full(com_xy.shape[:-1], -np.inf)
    margin = np.full(com_xy.shape[:-1], np.inf)
    for a, b in zip(order, order[1:] + order[:1]):
        pa, pb = feet_xy[..., a, :], feet_xy[..., b, :]
        e = pb - pa
        n = np.linalg.norm(e, axis=-1)
        cross = e[..., 0] * (com_xy[..., 1] - pa[..., 1]) - e[..., 1] * (com_xy[..., 0] - pa[..., 0])
        margin = np.minimum(margin, cross / np.maximum(n, 1e-9))
    return margin


def contact_forces(com_xy, feet_xy, weights, mass):
    """Vertical ground force on each foot (..., 4) for static equilibrium.

    weights (..., 4) in [0, 1]: how much a foot is allowed to carry (1 = planted, 0 = in the
    air). With three planted feet the answer is unique; with four it is the weighted
    least-norm solution, which changes smoothly as a weight ramps to zero.
    """
    com_xy = np.asarray(com_xy, dtype=float)
    feet_xy = np.asarray(feet_xy, dtype=float)
    w = np.asarray(weights, dtype=float) + 1e-6
    shape = np.broadcast_shapes(com_xy.shape[:-1], feet_xy.shape[:-2], w.shape[:-1])
    A = np.empty(shape + (3, 4))
    A[..., 0, :] = 1.0
    A[..., 1, :] = feet_xy[..., :, 0] - com_xy[..., None, 0]
    A[..., 2, :] = feet_xy[..., :, 1] - com_xy[..., None, 1]
    W = np.broadcast_to(w, shape + (4,))
    AW = A * W[..., None, :]
    M = AW @ np.swapaxes(A, -1, -2)
    b = np.zeros(shape + (3,))
    b[..., 0] = mass * K.GRAVITY
    lam = np.linalg.solve(M, b[..., None])[..., 0]
    return np.einsum("...ij,...i->...j", AW, lam)


def _leg_jacobians(leg, q, eps=1e-6):
    """Foot Jacobian (..., 3, 3) and the gravity torque of the leg's own links (..., 3)."""
    lm = K.leg_model(leg)
    q = np.asarray(q, dtype=float)
    J = np.empty(q.shape[:-1] + (3, 3))
    tau_g = np.zeros(q.shape)
    for j in range(3):
        dq = np.zeros(3)
        dq[j] = eps
        hi = K.leg_points(leg, q + dq)
        lo = K.leg_points(leg, q - dq)
        J[..., :, j] = (hi["foot"] - lo["foot"]) / (2 * eps)
        for mass, key in zip(lm.mass, ("com1", "com2", "com3")):
            tau_g[..., j] += mass * K.GRAVITY * (hi[key][..., 2] - lo[key][..., 2]) / (2 * eps)
    return J, tau_g


def joint_torques(q12, forces):
    """Motor torques (..., 12) that hold the pose: each leg's own weight plus the ground
    force on its foot (vertical, ``forces`` (..., 4) in newtons)."""
    q12 = np.asarray(q12, dtype=float)
    forces = np.asarray(forces, dtype=float)
    tau = np.empty(q12.shape)
    for i, leg in enumerate(K.LEGS):
        J, tau_g = _leg_jacobians(leg, q12[..., 3 * i:3 * i + 3])
        # ground pushes the foot up with f: the motors supply -J^T [0, 0, f]
        tau[..., 3 * i:3 * i + 3] = tau_g - J[..., 2, :] * forces[..., i, None]
    return tau


def sag_compensation(q12, forces, kp):
    """Offset to add to the desired joint angles so that, with a position gain ``kp`` and the
    static load, the joints settle at the desired angles: q_cmd = q_des + tau / kp."""
    return joint_torques(q12, forces) / kp


# ---------------------------------------------------------------------------- clearance
def _point_box_distance(x, z, x_near, x_far, h):
    dx = np.maximum(np.maximum(x_near - x, x - x_far), 0.0)
    dz = np.maximum(z - h, 0.0)
    return np.hypot(dx, dz)


def _segment_clearance(a, b, radius, x_near, x_far, h, n):
    t = np.linspace(0.0, 1.0, n).reshape((n,) + (1,) * (a.ndim - 1))
    x = a[..., 0][None] * (1 - t) + b[..., 0][None] * t
    z = a[..., 2][None] * (1 - t) + b[..., 2][None] * t
    return (_point_box_distance(x, z, x_near, x_far, h) - radius).min(axis=0)


PARTS = ("thigh", "knee", "shin_upper", "shin_lower", "foot")


def leg_box_gaps(hip, knee, foot, x_near, x_far, h):
    """Gap between each part of one leg's envelope and the box, in the fore-aft plane.

    hip, knee, foot: (..., 3) world positions (z = 0 on the floor). The box spans
    x_near..x_far, floor to h, and is treated as wide (no gap to either side). Returns
    (..., 5) in the order of ``PARTS``: thigh capsule, knee housing, upper shin, lower shin,
    foot (kinematics.*_RADIUS). Negative means overlap.
    """
    split = knee + (foot - knee) * K.SHANK_SPLIT
    return np.stack([
        _segment_clearance(hip, knee, K.THIGH_RADIUS, x_near, x_far, h, 7),
        _point_box_distance(knee[..., 0], knee[..., 2], x_near, x_far, h) - K.KNEE_RADIUS,
        _segment_clearance(knee, split, K.SHANK_RADIUS_UPPER, x_near, x_far, h, 6),
        _segment_clearance(split, foot, K.SHANK_RADIUS_LOWER, x_near, x_far, h, 6),
        _point_box_distance(foot[..., 0], foot[..., 2], x_near, x_far, h) - K.FOOT_ENVELOPE_RADIUS,
    ], axis=-1)


def leg_box_clearance(hip, knee, foot, x_near, x_far, h):
    """Smallest gap between any part of the leg and the box (see ``leg_box_gaps``)."""
    return leg_box_gaps(hip, knee, foot, x_near, x_far, h).min(axis=-1)


def center_of_mass_batch(q12, extra_masses=()):
    """Centre of mass in the body frame, (..., 3), for joint angles (..., 12)."""
    q12 = np.asarray(q12, dtype=float)
    m = K.MODEL["body"]["mass"]
    acc = np.zeros(q12.shape[:-1] + (3,)) + m * np.array(K.MODEL["body"]["com"])
    for i, leg in enumerate(K.LEGS):
        lm = K.leg_model(leg)
        pts = K.leg_points(leg, q12[..., 3 * i:3 * i + 3])
        for mass, key in zip(lm.mass, ("com1", "com2", "com3")):
            acc = acc + mass * pts[key]
            m += mass
    for mass, pos in extra_masses:
        acc = acc + mass * np.asarray(pos, dtype=float)
        m += mass
    return acc / m
