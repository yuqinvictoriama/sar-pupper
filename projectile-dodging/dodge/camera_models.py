"""Lens models and camera mounting poses.

Two frames are used:

  camera frame   z along the optical axis, x to the right in the image, y down
                 (the OpenCV convention)
  base_link      x forward, y left, z up (the robot's URDF frame)

A lens model turns pixels into unit rays in the camera frame and back. A Camera
adds where the lens sits on the robot and which way it points.
"""
from __future__ import annotations

import math

import numpy as np


# --------------------------------------------------------------------------- lens models
class CameraModel:
    width: int
    height: int

    def project(self, pts):  # (N,3) camera frame -> (N,2) pixels, (N,) valid
        raise NotImplementedError

    def unproject(self, uv):  # (N,2) pixels -> (N,3) unit rays, (N,) valid
        raise NotImplementedError

    def scaled(self, width: int, height: int) -> "CameraModel":
        raise NotImplementedError

    def solid_angle_per_px(self, u: float, v: float, h: float = 1.0) -> float:
        """Steradians covered by one pixel at (u, v)."""
        uv = np.array([[u + h, v], [u - h, v], [u, v + h], [u, v - h]], dtype=np.float64)
        rays, ok = self.unproject(uv)
        if not ok.all():
            return float("nan")
        a = (rays[0] - rays[1]) / (2.0 * h)
        b = (rays[2] - rays[3]) / (2.0 * h)
        return float(np.linalg.norm(np.cross(a, b)))


class DoubleSphere(CameraModel):
    """Double-sphere fisheye model (Usenko et al. 2018), as used by the course's fisheye_converter.py."""

    def __init__(self, fx, fy, cx, cy, xi, alpha, width, height):
        self.fx, self.fy, self.cx, self.cy = float(fx), float(fy), float(cx), float(cy)
        self.xi, self.alpha = float(xi), float(alpha)
        self.width, self.height = int(width), int(height)

    def project(self, pts):
        pts = np.atleast_2d(np.asarray(pts, dtype=np.float64))
        x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
        d1 = np.sqrt(x * x + y * y + z * z)
        k = self.xi * d1 + z
        d2 = np.sqrt(x * x + y * y + k * k)
        denom = self.alpha * d2 + (1.0 - self.alpha) * k
        valid = denom > 1e-9
        denom = np.where(valid, denom, 1.0)
        u = self.fx * x / denom + self.cx
        v = self.fy * y / denom + self.cy
        valid &= (u >= 0) & (u <= self.width - 1) & (v >= 0) & (v <= self.height - 1)
        return np.stack([u, v], axis=1), valid

    def unproject(self, uv):
        uv = np.atleast_2d(np.asarray(uv, dtype=np.float64))
        mx = (uv[:, 0] - self.cx) / self.fx
        my = (uv[:, 1] - self.cy) / self.fy
        r2 = mx * mx + my * my
        a, xi = self.alpha, self.xi
        valid = np.ones(len(uv), dtype=bool)
        if a > 0.5:
            valid &= r2 <= 1.0 / (2.0 * a - 1.0)
        s = np.sqrt(np.clip(1.0 - (2.0 * a - 1.0) * r2, 0.0, None))
        mz = (1.0 - a * a * r2) / (a * s + 1.0 - a)
        k = (mz * xi + np.sqrt(np.clip(mz * mz + (1.0 - xi * xi) * r2, 0.0, None))) / (mz * mz + r2)
        rays = np.stack([k * mx, k * my, k * mz - xi], axis=1)
        norm = np.linalg.norm(rays, axis=1, keepdims=True)
        rays = rays / np.where(norm > 0, norm, 1.0)
        return rays, valid

    def scaled(self, width, height):
        sx, sy = width / self.width, height / self.height
        return DoubleSphere(self.fx * sx, self.fy * sy, self.cx * sx, self.cy * sy, self.xi, self.alpha, width, height)


class Pinhole(CameraModel):
    """Rectilinear lens with OpenCV radial-tangential distortion (k1, k2, p1, p2, k3)."""

    def __init__(self, fx, fy, cx, cy, width, height, dist=None):
        self.fx, self.fy, self.cx, self.cy = float(fx), float(fy), float(cx), float(cy)
        self.width, self.height = int(width), int(height)
        d = list(dist or [])
        d = d + [0.0] * (5 - len(d))
        self.dist = [float(v) for v in d[:5]]
        self.r_lim, self.rd_lim = self._fold()

    @classmethod
    def from_hfov(cls, hfov_deg, width, height):
        f = (width / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
        return cls(f, f, (width - 1) / 2.0, (height - 1) / 2.0, width, height)

    def _radial(self, r2):
        k1, k2, _p1, _p2, k3 = self.dist
        return 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))

    def _fold(self, r_max: float = 6.0):
        """How far from the axis (undistorted, and as distorted) the distortion polynomial still maps
        farther-out rays to farther-out pixels. A wide lens's fitted polynomial bends back beyond the
        part of the picture the calibration saw; past that point it means nothing."""
        k1, k2, _p1, _p2, k3 = self.dist
        if not (k1 or k2 or k3):
            return float("inf"), float("inf")
        r = np.linspace(0.0, r_max, 6001)
        rd = r * self._radial(r * r)
        bad = np.nonzero(np.diff(rd) <= 0.0)[0]
        i = int(bad[0]) if len(bad) else len(r) - 1
        return float(r[i]), float(rd[i])

    def _distort(self, x, y):
        k1, k2, p1, p2, k3 = self.dist
        r2 = x * x + y * y
        radial = 1.0 + r2 * (k1 + r2 * (k2 + r2 * k3))
        xd = x * radial + 2.0 * p1 * x * y + p2 * (r2 + 2.0 * x * x)
        yd = y * radial + p1 * (r2 + 2.0 * y * y) + 2.0 * p2 * x * y
        return xd, yd

    def project(self, pts):
        pts = np.atleast_2d(np.asarray(pts, dtype=np.float64))
        z = pts[:, 2]
        valid = z > 1e-3
        zs = np.where(valid, z, 1.0)
        x, y = pts[:, 0] / zs, pts[:, 1] / zs
        valid &= np.hypot(x, y) <= self.r_lim          # beyond the fold the polynomial means nothing
        x, y = np.where(valid, x, 0.0), np.where(valid, y, 0.0)
        xd, yd = self._distort(x, y)
        u = self.fx * xd + self.cx
        v = self.fy * yd + self.cy
        valid &= (u >= 0) & (u <= self.width - 1) & (v >= 0) & (v <= self.height - 1)
        return np.stack([u, v], axis=1), valid

    def unproject(self, uv):
        """Pixels to rays. The radial part is inverted by bisection on the stretch where the polynomial
        rises, then the small tangential part by a few Newton steps. (A plain fixed-point iteration,
        used before 2026-11-27, ran off to infinity near the edges of a 120 degree lens with ordinary
        barrel distortion: that is where the rear cameras see a ball straight behind.)"""
        uv = np.atleast_2d(np.asarray(uv, dtype=np.float64))
        xd = (uv[:, 0] - self.cx) / self.fx
        yd = (uv[:, 1] - self.cy) / self.fy
        ok = np.ones(len(uv), dtype=bool)
        x, y = xd.copy(), yd.copy()
        if any(self.dist):
            rd = np.hypot(xd, yd)
            ok = rd <= self.rd_lim * (1.0 + 1e-9)
            target = np.minimum(rd, self.rd_lim)
            lo = np.zeros_like(rd)
            hi = np.full_like(rd, min(self.r_lim, 6.0))
            for _ in range(48):
                mid = 0.5 * (lo + hi)
                above = mid * self._radial(mid * mid) > target
                hi = np.where(above, mid, hi)
                lo = np.where(above, lo, mid)
            r = 0.5 * (lo + hi)
            scale = np.where(rd > 1e-12, r / np.maximum(rd, 1e-12), 1.0)
            x, y = xd * scale, yd * scale
            if self.dist[2] or self.dist[3]:
                eps = 1e-6
                for _ in range(5):
                    fx_, fy_ = self._distort(x, y)
                    ex, ey = fx_ - xd, fy_ - yd
                    ax, ay = self._distort(x + eps, y)
                    bx, by = self._distort(x, y + eps)
                    j11, j21 = (ax - fx_) / eps, (ay - fy_) / eps
                    j12, j22 = (bx - fx_) / eps, (by - fy_) / eps
                    det = j11 * j22 - j12 * j21
                    det = np.where(np.abs(det) > 1e-12, det, 1e-12)
                    x = x - (j22 * ex - j12 * ey) / det
                    y = y - (-j21 * ex + j11 * ey) / det
            # a pixel the polynomial cannot reach from inside its rising stretch is not trusted
            fx_, fy_ = self._distort(x, y)
            ok &= (np.hypot(x, y) <= self.r_lim) & (np.hypot(fx_ - xd, fy_ - yd) < 1e-6)
        rays = np.stack([x, y, np.ones_like(x)], axis=1)
        rays /= np.linalg.norm(rays, axis=1, keepdims=True)
        return rays, ok

    def scaled(self, width, height):
        sx, sy = width / self.width, height / self.height
        return Pinhole(self.fx * sx, self.fy * sy, (self.cx + 0.5) * sx - 0.5, (self.cy + 0.5) * sy - 0.5,
                       width, height, self.dist)


def make_model(kind: str, intr: dict) -> CameraModel:
    if kind == "double_sphere":
        return DoubleSphere(intr["fx"], intr["fy"], intr["cx"], intr["cy"], intr["xi"], intr["alpha"],
                            intr["width"], intr["height"])
    if kind == "pinhole":
        if "fx" in intr:
            return Pinhole(intr["fx"], intr.get("fy", intr["fx"]), intr["cx"], intr["cy"],
                           intr["width"], intr["height"], intr.get("dist"))
        return Pinhole.from_hfov(intr["hfov_deg"], intr["width"], intr["height"])
    raise ValueError("unknown camera model %r" % kind)


# --------------------------------------------------------------------------- mounting pose
def rotation_base_from_camera(yaw_deg: float, pitch_deg: float = 0.0, roll_deg: float = 0.0) -> np.ndarray:
    """Rotation taking camera-frame vectors to base_link.

    yaw    direction the lens points, counter-clockwise from straight ahead (0 = forward, 90 = left)
    pitch  lens tilted up by this much
    roll   image rotated about the optical axis (use 180 for a camera mounted upside down)
    """
    psi, th, ro = (math.radians(a) for a in (yaw_deg, pitch_deg, roll_deg))
    c, s = math.cos(psi), math.sin(psi)
    level = np.array([[s, 0.0, c],        # columns: camera x (image right), y (image down), z (optical axis)
                      [-c, 0.0, s],
                      [0.0, -1.0, 0.0]])
    ct, st = math.cos(th), math.sin(th)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, ct, -st], [0.0, st, ct]])   # tilts the optical axis up (image y is down)
    cr, sr = math.cos(ro), math.sin(ro)
    rz = np.array([[cr, -sr, 0.0], [sr, cr, 0.0], [0.0, 0.0, 1.0]])
    return level @ rx @ rz


class Camera:
    """A lens model plus where it sits on the robot."""

    def __init__(self, name: str, model: CameraModel, extr: dict, cfg: dict | None = None):
        self.name = name
        self.model = model
        self.extr = dict(extr)
        self.R = rotation_base_from_camera(extr.get("yaw_deg", 0.0), extr.get("pitch_deg", 0.0), extr.get("roll_deg", 0.0))
        self.t = np.array([extr.get("x", 0.0), extr.get("y", 0.0), extr.get("z", 0.0)], dtype=np.float64)
        self.cfg = cfg or {}

    @classmethod
    def from_config(cls, name: str, cfg: dict) -> "Camera":
        return cls(name, make_model(cfg["model"], cfg["intrinsics"]), cfg["extrinsics"], cfg)

    def with_resolution(self, width: int, height: int) -> "Camera":
        if (width, height) == (self.model.width, self.model.height):
            return self
        return Camera(self.name, self.model.scaled(width, height), self.extr, self.cfg)

    def rays_base(self, uv):
        rays, ok = self.model.unproject(uv)
        return rays @ self.R.T, rays, ok

    def project_base(self, pts_base):
        pts = (np.atleast_2d(np.asarray(pts_base, dtype=np.float64)) - self.t) @ self.R
        uv, ok = self.model.project(pts)
        return uv, ok, pts


def build_cameras(cfg: dict) -> dict:
    return {name: Camera.from_config(name, c) for name, c in cfg["cameras"].items() if c.get("enabled", True)}
