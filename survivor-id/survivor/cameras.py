"""Views: the images handed to the pose model, and where their pixels point.

Lens models and mounting poses come from the dodging component (dodge.camera_models),
so both components use one calibration. Three kinds of view:

  EquirectView  the whole fisheye, rectified the way Lab 5 does it (700 x 572 for 220 degrees)
  FrameView     a rear camera's frame as it is
  PatchView     a perspective "zoom" patch around one person, rendered from the full-resolution
                frame through the lens model (it also removes the stretch a 120-degree lens
                gives people near the image edge)

Frames:  camera  x right, y down, z along the lens axis
         base    x forward, y left, z up (base_link)
         level   base with roll and pitch removed (z = against gravity, x = the nose's heading)
"""
from __future__ import annotations

import math
from collections import OrderedDict

import cv2
import numpy as np

from .config import use_dodge_code

use_dodge_code()
from dodge.camera_models import Camera  # noqa: E402,F401


def quat_to_rot(w: float, x: float, y: float, z: float) -> np.ndarray:
    """Rotation matrix of a unit quaternion (body -> world)."""
    n = math.sqrt(w * w + x * x + y * y + z * z)
    if n < 1e-9:
        return np.eye(3)
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_of(R_wb: np.ndarray) -> float:
    return math.atan2(R_wb[1, 0], R_wb[0, 0])


def level_from_base(R_wb: np.ndarray) -> np.ndarray:
    """Rotation taking base_link vectors to the level frame (roll and pitch undone, heading kept)."""
    c, s = math.cos(-yaw_of(R_wb)), math.sin(-yaw_of(R_wb))
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ R_wb


def az_el(rays: np.ndarray):
    """Azimuth (counter-clockwise from straight ahead) and elevation of unit rays, radians."""
    rays = np.atleast_2d(rays)
    return np.arctan2(rays[:, 1], rays[:, 0]), np.arcsin(np.clip(rays[:, 2], -1.0, 1.0))


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class View:
    """An image for the pose model. Subclasses say how its pixels map to camera-frame rays."""

    kind = "view"

    def __init__(self, camera: Camera, width: int, height: int):
        self.camera, self.width, self.height = camera, int(width), int(height)

    def rays_cam(self, uv):                       # (N,2) view pixels -> (N,3) unit rays, (N,) valid
        raise NotImplementedError

    def render(self, bgr: np.ndarray) -> np.ndarray:
        raise NotImplementedError

    def rays_base(self, uv):
        rays, ok = self.rays_cam(uv)
        return rays @ self.camera.R.T, ok

    def source_uv(self, uv):
        """Where view pixels come from in the camera's own frame: (N,2), (N,) inside-the-frame."""
        rays, ok = self.rays_cam(uv)
        src, inside = self.camera.model.project(rays)
        return src, ok & inside

    def project_base(self, pts_base):
        """base_link points -> view pixels (used by the tests and the simulated detector)."""
        raise NotImplementedError


class FrameView(View):
    kind = "frame"

    def __init__(self, camera: Camera):
        super().__init__(camera, camera.model.width, camera.model.height)

    def rays_cam(self, uv):
        return self.camera.model.unproject(uv)

    def render(self, bgr):
        return bgr

    def project_base(self, pts_base):
        uv, ok, _ = self.camera.project_base(pts_base)
        return uv, ok


class _Remapped(View):
    """A view rendered by looking every one of its pixels up in the camera frame."""

    def _maps(self):
        if getattr(self, "_map", None) is None:
            u, v = np.meshgrid(np.arange(self.width, dtype=np.float64), np.arange(self.height, dtype=np.float64))
            rays, ok = self.rays_cam(np.stack([u.ravel(), v.ravel()], axis=1))
            src, inside = self.camera.model.project(rays)
            ok &= inside
            src[~ok] = -1.0
            self._map = (src[:, 0].reshape(self.height, self.width).astype(np.float32),
                         src[:, 1].reshape(self.height, self.width).astype(np.float32))
        return self._map

    def render(self, bgr):
        mx, my = self._maps()
        return cv2.remap(bgr, mx, my, interpolation=cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=0)


class EquirectView(_Remapped):
    """Longitude across, latitude down, equal degrees per pixel both ways (Lab 5's detection frame)."""
    kind = "equirect"

    def __init__(self, camera: Camera, width: int = 700, hfov_deg: float = 220.0):
        height = max(2, int(round(width * 180.0 / hfov_deg)) // 2 * 2)
        super().__init__(camera, width, height)
        self.hfov = math.radians(hfov_deg)

    def rays_cam(self, uv):
        uv = np.atleast_2d(np.asarray(uv, dtype=np.float64))
        lon = uv[:, 0] / (self.width - 1) * self.hfov - self.hfov / 2.0
        lat = uv[:, 1] / (self.height - 1) * math.pi - math.pi / 2.0
        rays = np.stack([np.cos(lat) * np.sin(lon), np.sin(lat), np.cos(lat) * np.cos(lon)], axis=1)
        ok = (uv[:, 0] >= 0) & (uv[:, 0] <= self.width - 1) & (uv[:, 1] >= 0) & (uv[:, 1] <= self.height - 1)
        return rays, ok

    def project_base(self, pts_base):
        pts = (np.atleast_2d(np.asarray(pts_base, dtype=np.float64)) - self.camera.t) @ self.camera.R
        d = pts / np.linalg.norm(pts, axis=1, keepdims=True)
        lon = np.arctan2(d[:, 0], d[:, 2])
        lat = np.arcsin(np.clip(d[:, 1], -1.0, 1.0))
        u = (lon + self.hfov / 2.0) / self.hfov * (self.width - 1)
        v = (lat + math.pi / 2.0) / math.pi * (self.height - 1)
        _, inside = self.camera.model.project(pts)
        return np.stack([u, v], axis=1), inside & (np.abs(lon) <= self.hfov / 2.0)


class PatchView(_Remapped):
    """A square perspective view looking along `axis_cam`, upright with respect to `down_cam`."""
    kind = "patch"

    def __init__(self, camera: Camera, axis_cam, fov_deg: float, size: int, down_cam=(0.0, 1.0, 0.0)):
        super().__init__(camera, size, size)
        z = np.asarray(axis_cam, dtype=np.float64)
        z = z / np.linalg.norm(z)
        down = np.asarray(down_cam, dtype=np.float64)
        x = np.cross(down, z)
        if np.linalg.norm(x) < 1e-6:              # looking straight up or down: any upright will do
            x = np.cross(np.array([0.0, 0.0, 1.0]), z)
        x = x / np.linalg.norm(x)
        self.R = np.stack([x, np.cross(z, x), z], axis=1)      # camera <- patch
        self.fov_deg = float(fov_deg)
        self.f = (size / 2.0) / math.tan(math.radians(fov_deg) / 2.0)
        self.c = (size - 1) / 2.0

    def rays_cam(self, uv):
        uv = np.atleast_2d(np.asarray(uv, dtype=np.float64))
        p = np.stack([(uv[:, 0] - self.c) / self.f, (uv[:, 1] - self.c) / self.f, np.ones(len(uv))], axis=1)
        p /= np.linalg.norm(p, axis=1, keepdims=True)
        return p @ self.R.T, np.ones(len(uv), dtype=bool)

    def project_base(self, pts_base):
        pts = (np.atleast_2d(np.asarray(pts_base, dtype=np.float64)) - self.camera.t) @ self.camera.R @ self.R
        ok = pts[:, 2] > 1e-6
        z = np.where(ok, pts[:, 2], 1.0)
        uv = np.stack([self.f * pts[:, 0] / z + self.c, self.f * pts[:, 1] / z + self.c], axis=1)
        _, inside = self.camera.model.project((np.atleast_2d(pts_base) - self.camera.t) @ self.camera.R)
        ok &= inside & (uv[:, 0] >= 0) & (uv[:, 0] <= self.width - 1) & (uv[:, 1] >= 0) & (uv[:, 1] <= self.height - 1)
        return uv, ok


class Rig:
    """The robot's cameras, and the views made from them (cached: building a view's lookup table is the slow part)."""

    def __init__(self, cfg: dict, max_patches: int = 24):
        self.cfg = cfg
        self.cam_cfg = cfg["cameras"]
        self.cameras = {name: Camera.from_config(name, c) for name, c in self.cam_cfg.items()}
        self._sized: dict = {}
        self._wide: dict = {}
        self._patches: OrderedDict = OrderedDict()
        self.max_patches = max_patches

    def camera(self, name: str, size=None) -> Camera:
        """The camera at the resolution its frames actually arrive in."""
        cam = self.cameras[name]
        if size is None or tuple(size) == (cam.model.width, cam.model.height):
            return cam
        key = (name, int(size[0]), int(size[1]))
        if key not in self._sized:
            self._sized[key] = cam.with_resolution(int(size[0]), int(size[1]))
        return self._sized[key]

    def is_rear(self, name: str) -> bool:
        return bool(self.cam_cfg[name].get("rear", False))

    def wide_view(self, name: str, size=None) -> View:
        cam = self.camera(name, size)
        key = (name, cam.model.width, cam.model.height)
        if key not in self._wide:
            c = self.cam_cfg[name]
            if c.get("wide", "frame") == "equirect":
                self._wide[key] = EquirectView(cam, int(c.get("equirect_width", 700)), float(c.get("equirect_hfov_deg", 220.0)))
            else:
                self._wide[key] = FrameView(cam)
        return self._wide[key]

    def patch_view(self, name: str, size, ray_base, fov_deg: float, patch_px: int, R_level_base=None) -> PatchView:
        """A zoom patch looking along ray_base (base_link). Directions and field of view are rounded
        so nearby requests reuse one lookup table."""
        cam = self.camera(name, size)
        ray_cam = cam.R.T @ np.asarray(ray_base, dtype=np.float64)
        ray_cam = ray_cam / np.linalg.norm(ray_cam)
        az = round(math.degrees(math.atan2(ray_cam[0], ray_cam[2])) / 2.0) * 2.0
        el = round(math.degrees(math.asin(max(-1.0, min(1.0, -ray_cam[1])))) / 2.0) * 2.0
        fov = min(150.0, max(8.0, math.ceil(fov_deg / 4.0) * 4.0))
        down_level = np.array([0.0, 0.0, -1.0])
        down_base = down_level if R_level_base is None else R_level_base.T @ down_level
        down_cam = cam.R.T @ down_base
        dq = tuple(np.round(down_cam, 1))
        key = (name, cam.model.width, cam.model.height, az, el, fov, int(patch_px), dq)
        view = self._patches.get(key)
        if view is None:
            a, e = math.radians(az), math.radians(el)
            axis = np.array([math.cos(e) * math.sin(a), -math.sin(e), math.cos(e) * math.cos(a)])
            view = PatchView(cam, axis, fov, int(patch_px), down_cam)
            self._patches[key] = view
            while len(self._patches) > self.max_patches:
                self._patches.popitem(last=False)
        else:
            self._patches.move_to_end(key)
        return view
