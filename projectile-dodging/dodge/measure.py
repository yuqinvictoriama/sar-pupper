"""Blob -> where the ball is, as seen from the robot.

A blob gives a direction (through the lens model and the camera's mounting
pose) and an angular size. The ball's real diameter is known, so the angular
size gives a range: Z = (D / 2) / sin(theta / 2).
"""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .camera_models import Camera
from .detect import circle_params


@dataclass
class Measurement:
    cam: str
    origin: np.ndarray      # lens position, base_link (m)
    ray: np.ndarray         # unit direction to the ball, base_link
    theta: float            # angular diameter (rad)
    range_m: float          # from the angular size
    sigma_t: float          # 1-sigma position error across the ray (m)
    sigma_r: float          # 1-sigma position error along the ray (m)
    truncated: bool         # blob cut by the frame edge: range is only a rough guess
    u: float = 0.0
    v: float = 0.0
    area: float = 0.0

    @property
    def point(self) -> np.ndarray:
        return self.origin + self.range_m * self.ray

    @property
    def azimuth_deg(self) -> float:
        p = self.point
        return math.degrees(math.atan2(p[1], p[0]))

    @property
    def elevation_deg(self) -> float:
        p = self.point
        return math.degrees(math.atan2(p[2], math.hypot(p[0], p[1])))


def angular_diameter(camera: Camera, u: float, v: float, area_px: float) -> float:
    """Angular diameter of a blob of area_px pixels centred at (u, v).

    Uses the solid angle one pixel covers at that spot, so it is right in the
    stretched corners of a rectilinear lens and on a fisheye alike.
    """
    h = max(1.0, 0.5 * math.sqrt(area_px / math.pi))
    sr_per_px = camera.model.solid_angle_per_px(u, v, h)
    if not (sr_per_px > 0.0):
        return float("nan")
    omega = area_px * sr_per_px
    # disc of angular radius a covers 2*pi*(1 - cos a)
    c = 1.0 - omega / (2.0 * math.pi)
    return 2.0 * math.acos(max(-1.0, min(1.0, c)))


def measure_blob(camera: Camera, blob, ball_diameter: float) -> Measurement | None:
    cfg = camera.cfg
    circle = cfg.get("image_circle")
    if circle:
        intr = cfg.get("intrinsics", {})
        c = circle_params(circle, (intr.get("width", 0), intr.get("height", 0)), camera.model.width, camera.model.height)
        if c is not None and math.hypot(blob.u - c[0], blob.v - c[1]) > c[2]:
            return None                      # in the fringe at the rim of the lens
    rays_b, rays_c, ok = camera.rays_base(np.array([[blob.u, blob.v]]))
    if not ok[0]:
        return None
    ray_c, ray_b = rays_c[0], rays_b[0]
    off_axis = math.degrees(math.acos(max(-1.0, min(1.0, float(ray_c[2])))))
    if off_axis > float(cfg.get("max_angle_deg", 180.0)):
        return None
    elev = math.degrees(math.asin(max(-1.0, min(1.0, float(ray_b[2])))))
    if elev < float(cfg.get("min_elevation_deg", -90.0)):
        return None
    # Thresholded blobs come out a little too big (edge pixels that are only
    # partly ball still pass): take a fixed number of pixels off the diameter,
    # then apply the scale. Both come from tools/calibrate_color.py --distance.
    d_px = blob.diameter_px
    d_fix = max(d_px - float(cfg.get("size_offset_px", 0.0)), 0.5 * d_px)
    theta = angular_diameter(camera, blob.u, blob.v, blob.area * (d_fix / d_px) ** 2)
    if not (theta > 0.0):
        return None
    theta *= float(cfg.get("size_scale", 1.0))
    rng = (ball_diameter / 2.0) / math.sin(min(theta, math.pi * 0.999) / 2.0)
    d_px = max(blob.diameter_px, 1.0)
    rel = float(cfg.get("size_sigma_px", 0.5)) / d_px
    sigma_r = rng * rel
    truncated = bool(blob.touches_edge)
    if truncated:
        sigma_r = max(sigma_r, 0.6 * rng)   # the ball is at most this far; could be much closer
    sigma_t = rng * math.radians(float(cfg.get("bearing_sigma_deg", 0.3)))
    sigma_t = max(sigma_t, rng * math.tan(math.sqrt(camera.model.solid_angle_per_px(blob.u, blob.v)) * 0.4))
    return Measurement(camera.name, camera.t.copy(), ray_b, theta, rng, sigma_t, sigma_r, truncated,
                       blob.u, blob.v, blob.area)


def measure_blobs(camera: Camera, blobs, ball_diameter: float) -> list:
    out = []
    for b in blobs:
        m = measure_blob(camera, b, ball_diameter)
        if m is not None:
            out.append(m)
    return out
