"""Pupper v3 leg kinematics, joint limits and mass model. Pure numpy, no ROS, no MuJoCo.

Frames and conventions
----------------------
Body frame: origin at ``base_link`` (on the hip-pitch axis plane), x forward, y left, z up.
Joint order everywhere is the robot's: FR(1,2,3), FL(1,2,3), BR(1,2,3), BL(1,2,3), i.e.
``leg_front_r_1 ... leg_back_l_3``. Joint 1 is hip pitch, joint 2 abduction, joint 3 knee.

Forward kinematics is built directly from the model's transforms (``pupper_model.MODEL``,
generated from pupper_v3_description), so it is correct by construction and is checked
against MuJoCo in tests/test_kinematics.py. Inverse kinematics is closed-form for a right
leg and mirrored for a left leg; it is checked by round trip against the forward kinematics.
"""

from __future__ import annotations

import math

import numpy as np

from .pupper_model import MODEL

LEGS = ("front_r", "front_l", "back_r", "back_l")
LEG_INDEX = {name: i for i, name in enumerate(LEGS)}
JOINT_NAMES = tuple(f"leg_{leg}_{j}" for leg in LEGS for j in (1, 2, 3))
DEFAULT_JOINT_POS = np.array([0.26, 0.0, -0.52, -0.26, 0.0, 0.52, 0.26, 0.0, -0.52, -0.26, 0.0, 0.52])

FOOT_RADIUS = MODEL["legs"]["front_r"]["foot_radius"]  # collision sphere in the model
TOTAL_MASS = MODEL["body"]["mass"] + sum(link["mass"] for leg in LEGS for link in MODEL["legs"][leg]["links"])
GRAVITY = 9.81

# The robot's hardware interface clamps position commands 0.1 rad inside the model's joint
# ranges (pupper_v3_description components.xacro: position_min / position_max).
HARDWARE_LIMIT_INSET = 0.1

# Envelope of the real leg links around their kinematic segments, measured from the visual
# meshes in the sagittal plane (sim/extract_model.py prints these; see plan "Step-over
# envelope"). Used for clearance checks, so they are deliberately the outer envelope.
THIGH_RADIUS = 0.030      # motor housings along hip -> knee (29.6 mm measured)
KNEE_RADIUS = 0.030       # around the knee axis (29.1 mm measured)
SHANK_RADIUS_UPPER = 0.0185   # shin, knee end (first 55 % of its length)
SHANK_RADIUS_LOWER = 0.013    # shin, foot end
SHANK_SPLIT = 0.55
FOOT_ENVELOPE_RADIUS = 0.0215  # the moulded foot is slightly larger than the model's sphere

# Joint ranges (right-leg signs) in which the legs clear the design-2 mic bars, from the
# design notes ("Leg check"). A pose is allowed if it is inside EITHER set:
#   WALK: the swept walking / lying-down envelope (front bars 18 mm clear, rear bars 9 mm);
#   ANY_KNEE: hip-pitch ranges that are clear at every knee angle (>= 3 mm).
MIC_BAR_WALK_ENVELOPE = ((-0.75, 1.25), (-0.42, 0.60), (-1.60, 0.71))
MIC_BAR_ANY_KNEE = {
    "front": ((-0.49, 1.64), (-0.42, math.radians(120.0)), (-9.0, 9.0)),
    "back": ((-0.94, 2.45), (-0.42, math.radians(90.0)), (-9.0, 9.0)),
}


def _quat_to_mat(q):
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


class _LegModel:
    def __init__(self, name):
        spec = MODEL["legs"][name]
        self.name = name
        self.side = -1.0 if name.endswith("_r") else 1.0   # sign of y
        self.is_front = name.startswith("front")
        links = spec["links"]
        self.p = [np.array(link["pos"]) for link in links]
        self.R = [_quat_to_mat(link["quat"]) for link in links]
        self.mass = [link["mass"] for link in links]
        self.com = [np.array(link["com"]) for link in links]
        self.range = np.array([link["range"] for link in links])
        self.foot = np.array(spec["foot"])
        self.hip = self.p[0].copy()   # hip origin in the body frame
        lo = self.range[:, 0] + HARDWARE_LIMIT_INSET
        hi = self.range[:, 1] - HARDWARE_LIMIT_INSET
        self.hw_limits = np.stack([lo, hi], axis=1)


_LEG_MODELS = {name: _LegModel(name) for name in LEGS}


def leg_model(leg: str) -> _LegModel:
    return _LEG_MODELS[leg]


def hip_position(leg: str) -> np.ndarray:
    """Hip (joint 1/2 origin) in the body frame."""
    return _LEG_MODELS[leg].hip.copy()


def _rotz(q):
    """Rotation matrices about z for an array of angles: shape (..., 3, 3)."""
    q = np.asarray(q, dtype=float)
    c, s = np.cos(q), np.sin(q)
    out = np.zeros(q.shape + (3, 3))
    out[..., 0, 0] = c
    out[..., 0, 1] = -s
    out[..., 1, 0] = s
    out[..., 1, 1] = c
    out[..., 2, 2] = 1.0
    return out


def leg_points(leg: str, q) -> dict:
    """Forward kinematics of one leg.

    q: (..., 3) joint angles. Returns points in the body frame, each (..., 3):
    ``hip``, ``knee`` (knee axis), ``foot`` (foot centre), and link centres of mass
    ``com1``, ``com2``, ``com3``.
    """
    lm = _LEG_MODELS[leg]
    q = np.asarray(q, dtype=float)
    A1 = lm.R[0] @ _rotz(q[..., 0])                     # body <- link1
    A2 = A1 @ lm.R[1] @ _rotz(q[..., 1])                # body <- link2
    o2 = lm.p[0] + A1 @ lm.p[1]                         # link2 origin (== hip)
    o3 = o2 + A2 @ lm.p[2]                              # knee
    A3 = A2 @ lm.R[2] @ _rotz(q[..., 2])                # body <- link3
    return {
        "hip": np.broadcast_to(lm.p[0], o3.shape).copy(),
        "knee": o3,
        "foot": o3 + A3 @ lm.foot,
        "com1": lm.p[0] + A1 @ lm.com[0],
        "com2": o2 + A2 @ lm.com[1],
        "com3": o3 + A3 @ lm.com[2],
    }


def foot_position(leg: str, q) -> np.ndarray:
    return leg_points(leg, q)["foot"]


def shin_rotation(leg: str, q) -> np.ndarray:
    """Orientation of the shin link in the body frame, (..., 3, 3)."""
    lm = _LEG_MODELS[leg]
    q = np.asarray(q, dtype=float)
    return lm.R[0] @ _rotz(q[..., 0]) @ lm.R[1] @ _rotz(q[..., 1]) @ lm.R[2] @ _rotz(q[..., 2])


def rolling_shift(R_from, R_to, radius: float = None) -> np.ndarray:
    """How far a ball foot's centre travels over the floor when its shin turns from
    orientation R_from to R_to while the foot rolls without slipping: (..., 3), z = 0.

    A planted ball foot does not pivot about its centre; it rolls. The centre moves by
    radius x (rotation about y) forward and radius x (rotation about x) to the right.
    """
    radius = FOOT_RADIUS if radius is None else radius
    dR = R_to @ np.swapaxes(R_from, -1, -2)
    w = np.stack([dR[..., 2, 1] - dR[..., 1, 2], dR[..., 0, 2] - dR[..., 2, 0], dR[..., 1, 0] - dR[..., 0, 1]], axis=-1) / 2.0
    s = np.linalg.norm(w, axis=-1)
    angle = np.arctan2(s, (np.trace(dR, axis1=-2, axis2=-1) - 1.0) / 2.0)
    scale = np.where(s > 1e-9, angle / np.maximum(s, 1e-9), 1.0)
    w = w * scale[..., None]
    out = np.zeros(w.shape)
    out[..., 0] = radius * w[..., 1]
    out[..., 1] = -radius * w[..., 0]
    return out


def all_feet(q12) -> np.ndarray:
    """Foot centres (4, 3) in the body frame for a 12-vector of joint angles."""
    q12 = np.asarray(q12, dtype=float)
    return np.stack([foot_position(leg, q12[3 * i:3 * i + 3]) for i, leg in enumerate(LEGS)])


# ---------------------------------------------------------------------------- inverse kinematics
# Closed form for a right leg. With the model's fixed link rotations, a foot position
# (X, Y, Z) relative to the hip satisfies (derivation in tests/test_kinematics.py docstring):
#   distance law:   D^2 - fz^2 = |k + rot(q3) s|^2         -> q3 (knee)
#   lateral:        fz cos q2 - yL(q3) sin q2 = -Y           -> q2 (abduction)
#   sagittal angle: q1 = atan2(Z, X) - atan2(gy, gx)         -> q1 (hip pitch)
_R = _LEG_MODELS["front_r"]
_FX, _FY, _FZ = _R.foot
_KY, _KZ = _R.p[2][1], _R.p[2][2]
_AS = _KY * _FX + _KZ * _FY
_AC = _KY * _FY - _KZ * _FX
_K2 = _KY * _KY + _KZ * _KZ
_S2 = _FX * _FX + _FY * _FY
_AMP = math.hypot(_AS, _AC)
_PHI = math.atan2(_AS, _AC)
LEG_MAX_REACH = math.sqrt(_K2 + _S2 + 2 * _AMP + _FZ * _FZ)   # hip to foot centre, leg straight


def _wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def _branch_signs():
    """Signs of the knee and abduction solutions that reproduce the default standing pose."""
    q0 = DEFAULT_JOINT_POS[:3]
    p = foot_position("front_r", q0) - _R.hip
    D2 = float(p @ p) - _FZ * _FZ
    acw = math.acos(max(-1.0, min(1.0, (D2 - _K2 - _S2) / 2.0 / _AMP)))
    knee = min((-1.0, 1.0), key=lambda sgn: abs(_wrap(_PHI + sgn * acw - q0[2])))
    yL = _KY + math.sin(q0[2]) * _FX + math.cos(q0[2]) * _FY
    delta = math.atan2(yL, _FZ)
    acy = math.acos(max(-1.0, min(1.0, -p[1] / math.hypot(_FZ, yL))))
    abd = min((-1.0, 1.0), key=lambda sgn: abs(_wrap(-delta + sgn * acy - q0[1])))
    return knee, abd


_KNEE_SIGN, _ABD_SIGN = _branch_signs()


def _ik_right(p_rel, limits):
    """p_rel (..., 3): foot centre relative to the hip, right-leg geometry.
    Returns q (..., 3) and ok (...) for the standing branch inside ``limits`` (3, 2)."""
    p_rel = np.asarray(p_rel, dtype=float)
    X, Y, Z = p_rel[..., 0], p_rel[..., 1], p_rel[..., 2]
    D2 = X * X + Y * Y + Z * Z - _FZ * _FZ
    W = (D2 - _K2 - _S2) / 2.0
    cw = W / _AMP
    ok = np.abs(cw) <= 1.0
    acw = np.arccos(np.clip(cw, -1.0, 1.0))
    # One branch only: the knee-backward, leg-under-the-body configuration the robot stands
    # in (the branch signs are fixed at import from the default pose). Other branches are
    # separated from it by a singular pose, so they are never mixed in.
    q3 = _wrap(_PHI + _KNEE_SIGN * acw)
    s3, c3 = np.sin(q3), np.cos(q3)
    yL = _KY + s3 * _FX + c3 * _FY
    zL = _KZ - c3 * _FX + s3 * _FY
    Rm = np.sqrt(_FZ * _FZ + yL * yL)
    delta = np.arctan2(yL, _FZ)
    cy = -Y / np.maximum(Rm, 1e-12)
    ok = ok & (np.abs(cy) <= 1.0)
    acy = np.arccos(np.clip(cy, -1.0, 1.0))
    q2 = _wrap(-delta + _ABD_SIGN * acy)
    gx = -zL
    gy = np.sin(q2) * _FZ + np.cos(q2) * yL
    q1 = _wrap(np.arctan2(Z, X) - np.arctan2(gy, gx))
    q = np.stack([q1, q2, q3], axis=-1)
    inside = np.all((q >= limits[:, 0] - 1e-9) & (q <= limits[:, 1] + 1e-9), axis=-1)
    return q, ok & inside


def leg_limits(leg: str) -> np.ndarray:
    """Hardware command limits (3, 2) of one leg."""
    return _LEG_MODELS[leg].hw_limits.copy()


def _in_box(q, box):
    out = np.ones(q.shape[:-1], dtype=bool)
    for j, (lo, hi) in enumerate(box):
        out &= (q[..., j] >= lo - 1e-9) & (q[..., j] <= hi + 1e-9)
    return out


def _shrink(box, margin):
    return tuple((lo + margin, hi - margin) for lo, hi in box)


def mic_bar_safe(leg: str, q, margin: float = 0.0) -> np.ndarray:
    """True where joint angles q (..., 3) of ``leg`` keep it clear of the mic bars (with
    ``margin`` radians to spare inside each range)."""
    lm = _LEG_MODELS[leg]
    qr = np.asarray(q, dtype=float) * (-lm.side)      # to right-leg signs
    return (_in_box(qr, _shrink(MIC_BAR_WALK_ENVELOPE, margin))
            | _in_box(qr, _shrink(MIC_BAR_ANY_KNEE["front" if lm.is_front else "back"], margin)))


def leg_ik(leg: str, foot_body, mic_bars: bool = True, margin: float = 0.0):
    """Inverse kinematics of one leg.

    foot_body: (..., 3) foot centre in the body frame. Returns (q (..., 3), ok (...)).
    ``ok`` is False where the point is out of reach, outside the hardware joint limits, or
    (with ``mic_bars``) in a pose that could touch the design-2 mic bars. ``margin`` (rad)
    keeps that far inside every joint range; planning uses it so that a pose found by the
    search is still valid when the feet turn out a few millimetres from where it assumed.
    """
    lm = _LEG_MODELS[leg]
    p = np.asarray(foot_body, dtype=float) - lm.hip
    lim = lm.hw_limits + np.array([margin, -margin])
    if lm.side < 0:
        q, ok = _ik_right(p, lim)
    else:
        # Left leg: mirror image of a right leg with all joint angles negated.
        pm = p * np.array([1.0, -1.0, 1.0])
        lim_r = np.stack([-lim[:, 1], -lim[:, 0]], axis=1)
        q, ok = _ik_right(pm, lim_r)
        q = -q
    if mic_bars:
        ok = ok & mic_bar_safe(leg, q, margin)
    return q, ok


# ---------------------------------------------------------------------------- mass model
def center_of_mass(q12, extra_masses=()) -> np.ndarray:
    """Centre of mass in the body frame for joint angles q12 (12,).

    extra_masses: iterable of (mass_kg, [x, y, z]) for hardware not in the model (the camera
    plate, cameras and mics on the head).
    """
    q12 = np.asarray(q12, dtype=float)
    m = MODEL["body"]["mass"]
    acc = m * np.array(MODEL["body"]["com"])
    for i, leg in enumerate(LEGS):
        lm = _LEG_MODELS[leg]
        pts = leg_points(leg, q12[3 * i:3 * i + 3])
        for mass, key in zip(lm.mass, ("com1", "com2", "com3")):
            acc = acc + mass * pts[key]
            m += mass
    for mass, pos in extra_masses:
        acc = acc + mass * np.asarray(pos, dtype=float)
        m += mass
    return acc / m


def total_mass(extra_masses=()) -> float:
    return TOTAL_MASS + sum(mass for mass, _ in extra_masses)


def foot_jacobian(leg: str, q, eps: float = 1e-6) -> np.ndarray:
    """d(foot position in body frame)/dq, (3, 3), by central differences."""
    q = np.asarray(q, dtype=float)
    J = np.zeros((3, 3))
    for j in range(3):
        dq = np.zeros(3)
        dq[j] = eps
        J[:, j] = (foot_position(leg, q + dq) - foot_position(leg, q - dq)) / (2 * eps)
    return J


def default_stance():
    """Default standing pose: (hip height above the floor, feet (4, 3) in the body frame)."""
    feet = all_feet(DEFAULT_JOINT_POS)
    return float(-(feet[:, 2].mean()) + FOOT_RADIUS), feet
