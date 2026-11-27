"""One detected person, as the robot sees them: where each joint points (in the level frame),
how big their torso looks, and from that how far away they are.

    unit   the body scale: the shoulder line's height above the hip line, measured as a
           difference of tangents of elevation. For someone standing upright that is their
           torso length divided by their distance, whatever height the camera looks from, so
           every gesture threshold is a multiple of it ("0.4 torso lengths above the head").
    range  torso length / unit. Body size, not where the feet meet the floor: from cameras
           this close to the ground, one degree of body pitch would change a floor-based
           range by metres (plan, "What each camera sees of a person").
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .cameras import View, az_el
from .pose_decode import INPUT_SIZE, K, PoseDet


@dataclass
class PersonObs:
    cam: str
    t: float
    score: float
    box: np.ndarray               # in view pixels
    kpts: np.ndarray              # (17, 3) in view pixels
    conf: np.ndarray              # (17,) keypoint confidence; 0 where the pixel is outside the view
    rays: np.ndarray              # (17, 3) unit rays, level frame
    az: np.ndarray                # (17,) radians
    el: np.ndarray
    tan_el: np.ndarray            # (17,) tangent of each joint's elevation: height / distance
    unit: float | None            # body scale: torso length / distance, in the same tangent units
    origin: np.ndarray            # lens position, level frame
    s: float | None               # the torso's angular length, radians (for reference)
    s_from: str                   # where the scale came from: "torso", "head", "box" or "none"
    range_m: float | None         # lens to person
    position: np.ndarray | None   # level frame, about the body origin
    centre_ray: np.ndarray        # level frame, toward the torso (or the box centre)
    height_px: float              # box height as the pose model saw it (in its 640 px input)
    box_frac: float               # box height / view height
    clipped_top: bool             # the top of the box is at the edge of what the camera sees
    clipped_side: bool            # ... or a side of it is (a person half out of the picture)
    box_rays: np.ndarray          # (3, 3) level-frame rays to the box's top-centre, bottom-centre and centre
    zoom: bool = False
    view: View | None = field(default=None, repr=False)
    gesture: object | None = field(default=None, repr=False)     # the gesture rule's reading of this view

    @property
    def clipped(self) -> bool:
        return self.clipped_top or self.clipped_side

    @property
    def bearing(self) -> float:
        p = self.position if self.position is not None else self.centre_ray
        return math.atan2(p[1], p[0])

    @property
    def ground_range(self) -> float | None:
        return None if self.position is None else float(math.hypot(self.position[0], self.position[1]))

    def found(self, name: str, thr: float) -> bool:
        return bool(self.conf[K[name]] >= thr)


def angle(a: np.ndarray, b: np.ndarray) -> float:
    return float(math.acos(max(-1.0, min(1.0, float(np.dot(a, b))))))


def _mid(rays: np.ndarray, conf: np.ndarray, names, thr: float):
    idx = [K[n] for n in names if conf[K[n]] >= thr]
    if not idx:
        return None
    v = rays[idx].mean(axis=0)
    return v / np.linalg.norm(v)


def observe(det: PoseDet, view: View, R_level_base: np.ndarray, t: float, cfg: dict, zoom: bool = False) -> PersonObs:
    """Turn one detection in one view into a PersonObs."""
    thr = float(cfg["gesture"]["kpt_conf"])
    pc = cfg["person"]
    rays_b, ok = view.rays_base(det.kpts[:, :2])
    rays = rays_b @ R_level_base.T
    conf = np.where(ok, det.kpts[:, 2], 0.0)
    az, el = az_el(rays)
    origin = R_level_base @ view.camera.t

    x1, y1, x2, y2 = det.box
    edge = np.array([[(x1 + x2) / 2, y1], [(x1 + x2) / 2, y2], [(x1 + x2) / 2, (y1 + y2) / 2]])
    edge_rays = view.rays_base(edge)[0] @ R_level_base.T
    # Cut off = the box runs into the edge of what the camera itself sees (whatever view the
    # detection was made in): at the top (raised hands out of the picture, close to a rear camera)
    # or at a side (a person half in the picture, as at the seam between the two rear cameras).
    W, H = view.camera.model.width, view.camera.model.height
    margin_x, margin_y = 0.01 * W, 0.015 * H
    src, inside = view.source_uv(edge[:1])
    u, v = float(src[0, 0]), float(src[0, 1])
    top_in_columns = -margin_x <= u <= W - 1 + margin_x
    clipped = bool(v <= margin_y and top_in_columns) if np.isfinite(v) else False
    sides = np.array([[x1, (y1 + y2) / 2], [x2, (y1 + y2) / 2]])
    src_s, inside_s = view.source_uv(sides)
    clipped_side = bool((not inside_s.all()) or src_s[:, 0].min() <= margin_x or src_s[:, 0].max() >= W - 1 - margin_x
                        or (not inside[0] and not clipped))

    shoulders = _mid(rays, conf, ("l_shoulder", "r_shoulder"), thr)
    hips = _mid(rays, conf, ("l_hip", "r_hip"), thr)
    head = _mid(rays, conf, ("l_ear", "r_ear"), thr)
    if head is None:
        head = _mid(rays, conf, ("l_eye", "r_eye"), thr)
    # Range: people stand upright, so two points a known height apart give the horizontal distance
    # from the difference of the tangents of their elevations. (Dividing the length by the angle
    # between them would over-read from cameras this low, which look up at the torso.)
    def el_of(ray) -> float:
        return math.asin(max(-1.0, min(1.0, float(ray[2]))))

    def flat_range(upper, lower, metres: float) -> float | None:
        spread = math.tan(el_of(upper)) - math.tan(el_of(lower))
        return metres / spread if spread > 1e-4 else None

    s, s_from, flat, centre, unit = None, "none", None, edge_rays[2], None
    ratio = float(pc["torso_m"]) / float(pc["head_above_shoulders_m"])
    if shoulders is not None and hips is not None:
        s, s_from, centre = angle(shoulders, hips), "torso", shoulders + hips
        flat = flat_range(shoulders, hips, float(pc["torso_m"]))
        unit = math.tan(el_of(shoulders)) - math.tan(el_of(hips))
    elif shoulders is not None and head is not None:
        s, s_from, centre = (el_of(head) - el_of(shoulders)) * ratio, "head", shoulders
        flat = flat_range(head, shoulders, float(pc["head_above_shoulders_m"]))
        unit = (math.tan(el_of(head)) - math.tan(el_of(shoulders))) * ratio
    if s is None or flat is None or unit is None or unit < 1e-4:
        s, s_from, centre, unit = None, "none", edge_rays[2], None
        flat = flat_range(edge_rays[0], edge_rays[1], float(pc["height_m"]))
        if flat is not None:
            s_from = "box"
    centre = centre / np.linalg.norm(centre)
    rng, position = None, None
    horiz = math.hypot(centre[0], centre[1])
    if flat is not None and horiz > 1e-3:
        rng = flat / horiz                         # along the ray to the torso
        position = origin + rng * centre
    model_scale = INPUT_SIZE / max(view.width, view.height)
    tan_el = np.clip(rays[:, 2] / np.maximum(np.hypot(rays[:, 0], rays[:, 1]), 1e-3), -50.0, 50.0)
    return PersonObs(view.camera.name, t, det.score, np.asarray(det.box, dtype=np.float64), det.kpts, conf, rays, az, el,
                     tan_el, unit, origin, s, s_from, rng, position, centre, float((y2 - y1) * model_scale),
                     float((y2 - y1) / view.height), clipped, clipped_side, edge_rays, zoom, view)
