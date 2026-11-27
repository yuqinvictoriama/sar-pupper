"""The front fisheye: double-sphere camera model, its mounting on the robot, and a "polar
view" that resamples the image onto a grid of bearing x depression angle.

Camera frame (same as follow_me_pupper/robot/fisheye_converter.py): z out of the lens, x to
the right of the image, y down. Level frame: x forward, y left, z up, gravity-aligned, origin
on the floor under the body centre. The lens parameters are the course's (camera_params.yaml,
calibrated for 1400 x 1050).

Why a polar view: in a fisheye image "straight up from the floor" is a curve. After
resampling, each column of the polar view is one vertical plane through the camera (one
bearing), and each row one depression angle below the horizon, so "walk outward along the
floor until something is not floor, then up to its top" is plain array indexing, and the
ground-plane geometry of the plan is one line: range = camera_height / tan(depression).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass
class DoubleSphere:
    fx: float = 228.81030
    fy: float = 228.81030
    cx: float = 680.0
    cy: float = 479.0
    xi: float = -0.00092
    alpha: float = 0.62007
    width: int = 1400
    height: int = 1050

    @classmethod
    def from_yaml(cls, path, width=1400, height=1050):
        import yaml
        with open(path) as f:
            p = yaml.safe_load(f)["camera_params"]
        return cls(float(p["fx"]), float(p["fy"]), float(p["cx"]), float(p["cy"]), float(p["xi"]),
                   float(p["alpha"]), int(width), int(height))

    def scaled(self, width: int, height: int) -> "DoubleSphere":
        """The same lens for an image resized to width x height."""
        sx, sy = width / self.width, height / self.height
        return DoubleSphere(self.fx * sx, self.fy * sy, (self.cx + 0.5) * sx - 0.5, (self.cy + 0.5) * sy - 0.5,
                            self.xi, self.alpha, width, height)

    def project(self, rays):
        """rays (..., 3) in the camera frame -> u, v (pixels), valid."""
        x, y, z = rays[..., 0], rays[..., 1], rays[..., 2]
        r2 = x * x + y * y
        d1 = np.sqrt(r2 + z * z)
        k2 = self.xi * d1 + z
        d2 = np.sqrt(r2 + k2 * k2)
        denom = self.alpha * d2 + (1.0 - self.alpha) * k2
        valid = denom > 1e-9
        denom = np.where(valid, denom, 1.0)
        u = self.fx * x / denom + self.cx
        v = self.fy * y / denom + self.cy
        # behind the model's limit the projection folds over; reject it
        w1 = (1.0 - self.alpha) / self.alpha if self.alpha > 0.5 else self.alpha / (1.0 - self.alpha)
        w2 = (w1 + self.xi) / math.sqrt(2.0 * w1 * self.xi + self.xi * self.xi + 1.0)
        valid &= z > -w2 * d1
        valid &= (u >= 0) & (u <= self.width - 1) & (v >= 0) & (v <= self.height - 1)
        return u, v, valid

    def unproject(self, u, v):
        """Pixels -> unit rays (..., 3) in the camera frame, valid."""
        mx = (np.asarray(u, dtype=float) - self.cx) / self.fx
        my = (np.asarray(v, dtype=float) - self.cy) / self.fy
        r2 = mx * mx + my * my
        valid = np.ones(r2.shape, dtype=bool)
        if self.alpha > 0.5:
            valid &= r2 <= 1.0 / (2.0 * self.alpha - 1.0)
        s = np.sqrt(np.clip(1.0 - (2.0 * self.alpha - 1.0) * r2, 0.0, None))
        mz = (1.0 - self.alpha * self.alpha * r2) / (self.alpha * s + 1.0 - self.alpha)
        k = (mz * self.xi + np.sqrt(mz * mz + (1.0 - self.xi * self.xi) * r2)) / (mz * mz + r2)
        rays = np.stack([k * mx, k * my, k * mz - self.xi], axis=-1)
        rays /= np.linalg.norm(rays, axis=-1, keepdims=True)
        return rays, valid


@dataclass
class CameraPose:
    """Where the fisheye sits on the robot, in the default standing pose. Calibrate with
    tools/calibrate_camera.py; the defaults are estimates from the body CAD."""

    height: float = 0.250      # lens above the floor in the default stance (m)
    pitch_deg: float = 0.0     # lens axis below horizontal (+ = looking down)
    roll_deg: float = 0.0      # image rotated clockwise seen from behind the camera
    yaw_deg: float = 0.0       # lens axis to the left of straight ahead
    x_forward: float = 0.145   # lens ahead of the body centre (base_link) (m)
    y_left: float = 0.0

    def rotation(self) -> np.ndarray:
        """R such that ray_body = R @ ray_camera, for a level body."""
        # camera axes in the body frame for zero pitch/roll/yaw: z_cam -> +x, x_cam -> -y, y_cam -> -z
        R0 = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
        return _rz(math.radians(self.yaw_deg)) @ _ry(math.radians(self.pitch_deg)) @ _rx(math.radians(self.roll_deg)) @ R0


def _rx(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], dtype=float)


def _ry(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], dtype=float)


def _rz(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], dtype=float)


def level_from_body(quat_xyzw=None, roll: float = 0.0, pitch: float = 0.0) -> np.ndarray:
    """R such that v_level = R @ v_body: the body's tilt with its heading taken out.

    Give either the IMU orientation quaternion (x, y, z, w), as in sensor_msgs/Imu, or roll
    (about x) and pitch (about y, + = nose down) in radians.
    """
    if quat_xyzw is None:
        return _ry(pitch) @ _rx(roll)
    x, y, z, w = quat_xyzw
    R = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return _rz(-yaw) @ R


def tilt_of(R_level_body) -> float:
    """Angle between the body's up axis and vertical, radians."""
    return float(math.acos(max(-1.0, min(1.0, R_level_body[2, 2]))))


class PolarView:
    """Resampling of the fisheye image onto bearing x depression.

    bearings: degrees, + = left of straight ahead. depressions: degrees below the horizon
    (negative = above it). ``maps(R_level_body)`` gives the cv2.remap tables for the current
    body attitude; ``sample`` applies them.
    """

    def __init__(self, model: DoubleSphere, pose: CameraPose, bearing_deg=(-100.0, 100.0, 1.0),
                 depression_deg=(-20.0, 70.0, 0.5)):
        self.model, self.pose = model, pose
        self.bearings = np.arange(bearing_deg[0], bearing_deg[1] + 1e-9, bearing_deg[2])
        self.depressions = np.arange(depression_deg[0], depression_deg[1] + 1e-9, depression_deg[2])
        az = np.radians(self.bearings)[None, :]
        dep = np.radians(self.depressions)[:, None]
        # rows = depression (top row = highest above the horizon), columns = bearing
        self.rays_level = np.stack([np.cos(dep) * np.cos(az), np.cos(dep) * np.sin(az),
                                    -np.sin(dep) * np.ones_like(az)], axis=-1)
        self._R_cam = pose.rotation()
        self._cache = {}

    def maps(self, R_level_body=None):
        """(map_u, map_v, valid) float32 arrays, shape (n_depression, n_bearing)."""
        R_lb = np.eye(3) if R_level_body is None else np.asarray(R_level_body, dtype=float)
        key = tuple(np.round(R_lb, 3).ravel())
        if key not in self._cache:
            if len(self._cache) > 64:
                self._cache.clear()
            R = R_lb @ self._R_cam                              # level <- camera
            rays_cam = self.rays_level @ R                      # = R^T @ ray_level, row-wise
            u, v, valid = self.model.project(rays_cam)
            self._cache[key] = (u.astype(np.float32), v.astype(np.float32), valid)
        return self._cache[key]

    def sample(self, image, R_level_body=None, nearest: bool = False):
        """Polar image (n_depression, n_bearing[, channels]) and its valid mask."""
        import cv2
        mu, mv, valid = self.maps(R_level_body)
        out = cv2.remap(image, mu, mv, cv2.INTER_NEAREST if nearest else cv2.INTER_LINEAR,
                        borderMode=cv2.BORDER_CONSTANT, borderValue=0)
        return out, valid

    def ground_range(self, depression_deg, camera_height=None):
        """Horizontal distance from the lens to where a ray at this depression meets the floor."""
        hc = self.pose.height if camera_height is None else camera_height
        dep = np.radians(np.asarray(depression_deg, dtype=float))
        return np.where(dep > 1e-3, hc / np.tan(np.clip(dep, 1e-3, None)), np.inf)

    def to_body_xy(self, bearing_deg, rng):
        """Floor point (x forward, y left) relative to the body centre for a bearing and a
        horizontal range from the lens."""
        az = np.radians(np.asarray(bearing_deg, dtype=float))
        r = np.where(np.isfinite(rng), rng, np.nan)
        return (self.pose.x_forward + r * np.cos(az), self.pose.y_left + r * np.sin(az))


def height_from_angles(camera_height, dep_base_deg, dep_top_deg):
    """Height of a vertical face from the depression of its foot and of its top edge:
    h = hc * (1 - tan(top) / tan(base)). At or above the horizon the top is at least camera
    height; returns inf there."""
    b = np.radians(np.asarray(dep_base_deg, dtype=float))
    t = np.radians(np.asarray(dep_top_deg, dtype=float))
    with np.errstate(divide="ignore", invalid="ignore"):
        h = camera_height * (1.0 - np.tan(t) / np.tan(b))
    return np.where(t <= 0.0, np.inf, h)
