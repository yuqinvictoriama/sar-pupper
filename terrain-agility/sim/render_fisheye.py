"""Synthetic fisheye images: ray-cast a floor, coloured boxes and a ball through the same
double-sphere lens model the robot uses. For testing perception without the robot.

    from sim.render_fisheye import Scene, Cuboid, render
    img = render(Scene(boxes=[Cuboid(x=0.45, y=0.0, depth=0.064, width=0.5, height=0.025)]), model, pose)
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from terrain.camera_model import CameraPose, DoubleSphere  # noqa: E402

FLOOR = (150, 140, 125)        # RGB
WALL = (200, 200, 195)
BLUE = (30, 70, 200)
WHITE = (240, 240, 235)
ORANGE = (235, 120, 30)
BROWN = (150, 110, 70)
BALL_GREEN = (170, 240, 40)


@dataclass
class Cuboid:
    """A box standing on the floor. x, y: the centre of its NEAR face when yaw = 0 (m, body
    frame at the moment of the picture: x forward from the body centre, y left)."""

    x: float
    y: float
    depth: float
    width: float
    height: float
    yaw_deg: float = 0.0
    side: tuple = BLUE
    top: tuple = WHITE

    def centre(self):
        c, s = math.cos(math.radians(self.yaw_deg)), math.sin(math.radians(self.yaw_deg))
        return np.array([self.x + 0.5 * self.depth * c, self.y + 0.5 * self.depth * s])


@dataclass
class Scene:
    boxes: list = field(default_factory=list)
    balls: list = field(default_factory=list)       # (x, y, radius)
    floor: tuple = FLOOR
    wall: tuple = WALL
    wall_distance: float = 4.0                      # walls of a square room centred on the robot
    floor_texture: float = 6.0                      # std of per-pixel floor colour noise
    noise: float = 3.0                              # std of sensor noise on every pixel
    tint: tuple = (1.0, 1.0, 1.0)                   # per-channel gain (the real camera is reddish)
    seed: int = 0


def render(scene: Scene, model: DoubleSphere, pose: CameraPose, R_level_body=None, body_height_offset: float = 0.0):
    """RGB uint8 image (model.height, model.width, 3)."""
    rng = np.random.default_rng(scene.seed)
    v, u = np.mgrid[0:model.height, 0:model.width]
    rays_cam, valid = model.unproject(u, v)
    R = (np.eye(3) if R_level_body is None else np.asarray(R_level_body)) @ pose.rotation()
    d = rays_cam @ R.T                                   # level frame
    o = np.array([pose.x_forward, pose.y_left, pose.height + body_height_offset])
    if R_level_body is not None:                         # the lens moves with the body's tilt
        offset_body = np.array([pose.x_forward, pose.y_left, 0.0])
        o = np.asarray(R_level_body) @ offset_body + np.array([0, 0, pose.height + body_height_offset])
    img = np.zeros((model.height, model.width, 3), dtype=float)
    best = np.full(d.shape[:2], np.inf)

    # floor
    with np.errstate(divide="ignore", invalid="ignore"):
        t = np.where(d[..., 2] < -1e-6, -o[2] / d[..., 2], np.inf)
        px, py = o[0] + t * d[..., 0], o[1] + t * d[..., 1]
    on_floor = np.isfinite(t) & (np.abs(px) < scene.wall_distance) & (np.abs(py) < scene.wall_distance)
    img[on_floor] = np.array(scene.floor) + rng.normal(0, scene.floor_texture, (int(on_floor.sum()), 1))
    best[on_floor] = t[on_floor]
    img[~on_floor] = scene.wall

    # boxes (slab test in each box's own frame)
    for b in scene.boxes:
        c, s = math.cos(math.radians(b.yaw_deg)), math.sin(math.radians(b.yaw_deg))
        ctr = b.centre()
        rel = o[:2] - ctr
        ob = np.array([c * rel[0] + s * rel[1], -s * rel[0] + c * rel[1], o[2]])
        db = np.stack([c * d[..., 0] + s * d[..., 1], -s * d[..., 0] + c * d[..., 1], d[..., 2]], axis=-1)
        lo = np.array([-b.depth / 2, -b.width / 2, 0.0])
        hi = np.array([b.depth / 2, b.width / 2, b.height])
        with np.errstate(divide="ignore", invalid="ignore"):
            t1 = (lo - ob) / db
            t2 = (hi - ob) / db
        tn = np.minimum(t1, t2)
        tf = np.maximum(t1, t2)
        t_in = tn.max(axis=-1)
        t_out = tf.min(axis=-1)
        hit = (t_in < t_out) & (t_in > 0) & (t_in < best)
        axis = tn.argmax(axis=-1)
        is_top = hit & (axis == 2)
        is_side = hit & (axis != 2)
        shade = np.where(axis == 0, 1.0, 0.82)           # faces to the left/right a little darker
        img[is_top] = b.top
        img[is_side] = np.array(b.side)[None, :] * shade[is_side][:, None]
        best[hit] = t_in[hit]

    for (bx, by, r) in scene.balls:
        ctr = np.array([bx, by, r])
        oc = o - ctr
        bq = d @ oc
        cq = oc @ oc - r * r
        disc = bq * bq - cq
        tb = -bq - np.sqrt(np.clip(disc, 0, None))
        hit = (disc > 0) & (tb > 0) & (tb < best)
        img[hit] = BALL_GREEN
        best[hit] = tb[hit]

    img = img * np.array(scene.tint) + rng.normal(0, scene.noise, img.shape)
    img[~valid] = 0
    return np.clip(img, 0, 255).astype(np.uint8)
