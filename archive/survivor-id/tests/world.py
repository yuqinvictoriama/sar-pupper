"""A stand-in world for the tests: stick-figure people, a pose "detector" that projects them
through the real lens models, and a robot that moves the way it is told.

Nothing here runs on the robot. The detector answers the same call the Hailo model does
(detector(image, view) -> [PoseDet]) by projecting 3D joints into whatever view it is handed,
so everything downstream of the neural network is exercised with the real geometry.
"""
from __future__ import annotations

import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from survivor.config import load_config  # noqa: E402
from survivor.pose_decode import KPT_NAMES, K, PoseDet  # noqa: E402

CFG = load_config(use_calib=False)
HIP_HEIGHT = float(CFG["dodge"]["robot"]["hip_height_m"])      # base_link origin above the floor

BODY = {  # metres, person frame: x = the way they face, y = their left, z = up from the floor (1.7 m adult)
    "nose": (0.10, 0.0, 1.60), "l_eye": (0.08, 0.03, 1.63), "r_eye": (0.08, -0.03, 1.63),
    "l_ear": (0.0, 0.075, 1.61), "r_ear": (0.0, -0.075, 1.61),
    "l_shoulder": (0.0, 0.18, 1.40), "r_shoulder": (0.0, -0.18, 1.40),
    "l_hip": (0.0, 0.12, 0.92), "r_hip": (0.0, -0.12, 0.92),
    "l_knee": (0.0, 0.12, 0.50), "r_knee": (0.0, -0.12, 0.50),
    "l_ankle": (0.0, 0.12, 0.08), "r_ankle": (0.0, -0.12, 0.08),
}
UPPER, FORE = 0.30, 0.30


def _unit(v):
    v = np.asarray(v, dtype=float)
    return v / np.linalg.norm(v)


def arm(pose: str, side: float, v_deg: float = 20.0):
    """(upper-arm direction, forearm direction) for one arm; side = +1 left, -1 right."""
    v = math.radians(v_deg)
    up = _unit((0.0, side * math.sin(v), math.cos(v)))
    down = _unit((0.0, side * 0.1, -1.0))
    if pose == "up":
        return up, up
    if pose == "down":
        return down, down
    if pose == "forward":                 # reaching toward whatever they face (e.g. to pet the robot)
        d = _unit((1.0, 0.0, -0.17))
        return d, d
    if pose == "t":
        return _unit((0, side, 0)), _unit((0, side, 0))
    if pose == "goalpost":
        return _unit((0, side, 0)), _unit((0, 0, 1))
    if pose == "on_head":
        return _unit((0.0, side * 0.8, 0.6)), _unit((0.0, -side * 0.75, 0.25))
    if pose == "left_up":
        return (up, up) if side > 0 else (down, down)
    if pose == "right_up":
        return (up, up) if side < 0 else (down, down)
    raise ValueError(pose)


class Person:
    def __init__(self, x: float, y: float, facing_deg: float = 0.0, pose: str = "down", v_deg: float = 20.0,
                 hidden=(), scale: float = 1.0):
        self.x, self.y, self.facing, self.pose, self.v_deg = float(x), float(y), math.radians(facing_deg), pose, v_deg
        self.hidden = set(hidden)         # joint names the "detector" cannot find
        self.scale = scale

    def face(self, x: float, y: float) -> "Person":
        self.facing = math.atan2(y - self.y, x - self.x)
        return self

    def joints(self) -> np.ndarray:
        """(17, 3) world coordinates, z up from the floor."""
        pts = {k: np.array(v, dtype=float) for k, v in BODY.items()}
        for side, s in (("l", 1.0), ("r", -1.0)):
            u, f = arm(self.pose, s, self.v_deg)
            pts[side + "_elbow"] = pts[side + "_shoulder"] + UPPER * u
            pts[side + "_wrist"] = pts[side + "_elbow"] + FORE * f
        c, s_ = math.cos(self.facing), math.sin(self.facing)
        R = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
        local = np.stack([pts[n] for n in KPT_NAMES]) * self.scale
        return local @ R.T + np.array([self.x, self.y, 0.0])


class Scene:
    """People, and where the robot is (world frame: z up from the floor)."""

    def __init__(self, people=()):
        self.people = list(people)
        self.x = self.y = self.yaw = 0.0
        self.R_tilt = np.eye(3)           # extra roll/pitch of the body

    def R_wb(self) -> np.ndarray:
        c, s = math.cos(self.yaw), math.sin(self.yaw)
        return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]) @ self.R_tilt

    def quat(self) -> tuple:
        R = self.R_wb()
        w = math.sqrt(max(0.0, 1.0 + R[0, 0] + R[1, 1] + R[2, 2])) / 2.0
        if w < 1e-6:
            return (0.0, 0.0, 0.0, 1.0)
        return (w, (R[2, 1] - R[1, 2]) / (4 * w), (R[0, 2] - R[2, 0]) / (4 * w), (R[1, 0] - R[0, 1]) / (4 * w))

    def to_base(self, pts_world: np.ndarray) -> np.ndarray:
        return (pts_world - np.array([self.x, self.y, HIP_HEIGHT])) @ self.R_wb()


class FakeDetector:
    """Projects the scene's people into the view it is given."""

    def __init__(self, scene: Scene, noise_px: float = 0.0, seed: int = 0, occlusion: bool = False):
        self.scene, self.noise, self.rng = scene, noise_px, np.random.default_rng(seed)
        self.occlusion = occlusion        # a person standing behind another is not found
        self.calls = []

    def hidden_behind(self, person, cam_t) -> bool:
        me = self.scene.to_base(np.array([[person.x, person.y, 1.2]]))[0] - cam_t
        for other in self.scene.people:
            if other is person:
                continue
            o = self.scene.to_base(np.array([[other.x, other.y, 1.2]]))[0] - cam_t
            gap = abs((math.atan2(me[1], me[0]) - math.atan2(o[1], o[0]) + math.pi) % (2 * math.pi) - math.pi)
            if np.hypot(*o[:2]) < np.hypot(*me[:2]) and gap < math.atan2(0.25, np.hypot(*o[:2])):
                return True
        return False

    def __call__(self, image, view):
        self.calls.append(view.kind)
        dets = []
        for person in self.scene.people:
            if self.occlusion and self.hidden_behind(person, view.camera.t):
                continue
            pts = self.scene.to_base(person.joints())
            uv, ok = view.project_base(pts)
            conf = np.where(ok, 0.9, 0.0)
            # Side-on, the arm on the far side is hidden behind the body.
            cam = view.camera.t
            to_cam = _unit((cam - pts[[K["l_shoulder"], K["r_shoulder"]]].mean(axis=0))[:2])
            across = pts[K["l_shoulder"]][:2] - pts[K["r_shoulder"]][:2]
            if abs(float(np.dot(_unit(across), to_cam))) > 0.9:
                far = "l" if np.linalg.norm(pts[K["l_shoulder"]] - cam) > np.linalg.norm(pts[K["r_shoulder"]] - cam) else "r"
                for j in ("_elbow", "_wrist"):
                    conf[K[far + j]] = min(conf[K[far + j]], 0.15)
            for name in person.hidden:
                conf[K[name]] = 0.05
            if int((conf >= 0.5).sum()) < 6:
                continue
            if self.noise:
                uv = uv + self.rng.normal(0.0, self.noise, uv.shape)
            vis = uv[ok]
            x1, y1, x2, y2 = vis[:, 0].min(), vis[:, 1].min(), vis[:, 0].max(), vis[:, 1].max()
            pad_x, pad_y = 0.12 * (x2 - x1) + 2, 0.06 * (y2 - y1) + 2
            box = np.array([x1 - pad_x, y1 - pad_y, x2 + pad_x, y2 + pad_y])
            if not ok[[K["l_wrist"], K["r_wrist"], K["nose"]]].all() and y1 < 0.25 * view.height:
                box[1] = 0.0              # part of them is above the top of the picture
            box = np.clip(box, 0, [view.width - 1, view.height - 1, view.width - 1, view.height - 1])
            dets.append(PoseDet(box, 0.9, np.concatenate([uv, conf[:, None]], axis=1)))
        return dets


def frames(cfg=CFG) -> dict:
    """A plain grey frame per camera, at the size its lens model expects."""
    out = {}
    for name, c in cfg["cameras"].items():
        intr = c["intrinsics"]
        out[name] = np.full((int(intr["height"]), int(intr["width"]), 3), 110, np.uint8)
    return out


def at(bearing_deg: float, dist: float, facing: str = "toward", pose: str = "up", **kw) -> Person:
    """A person `dist` metres from the robot at `bearing_deg` (0 = ahead, 90 = left), facing
    toward / away / left / right as seen from the robot."""
    b = math.radians(bearing_deg)
    p = Person(dist * math.cos(b), dist * math.sin(b), pose=pose, **kw)
    turn = {"toward": 180.0, "away": 0.0, "left": 90.0, "right": -90.0}[facing]
    p.facing = b + math.radians(turn)
    return p


class Sim:
    """Perception + mission + a robot that moves as commanded: the whole loop, without ROS.

    The robot is a point that follows /cmd_vel exactly (scaled by `slip`), cannot enter an
    obstacle (x, y, radius), and takes time over stand-ups and tricks the way the real one does.
    """

    def __init__(self, people, motion=True, drive=None, obstacles=(), slip=1.0, cfg=CFG, noise_px=1.0,
                 occlusion=False, tricks=("push_up",), hands_back=False):
        from survivor.mission import Inputs, Mission
        from survivor.perception import SurvivorPerception
        self.Inputs = Inputs
        self.cfg = cfg
        self.scene = Scene(people)
        self.det = FakeDetector(self.scene, noise_px=noise_px, occlusion=occlusion)
        self.perception = SurvivorPerception(cfg)
        self.mission = Mission(cfg, motion=motion, drive=drive)
        self.frames = frames(cfg)
        self.obstacles, self.slip, self.tricks = list(obstacles), slip, set(tricks)
        self.hands_back = hands_back      # the server returns to walking by itself after the trick
        self.t = 0.0
        self.walking, self.walking_at, self.busy_until = True, None, {}
        self.mux = "none"
        self.override = {}                # Inputs fields a test wants to force
        self.cmd = (0.0, 0.0, 0.0)
        self.states, self.events, self.actions, self.cmds = [], [], [], []
        self._n = 0

    # the robot server's side of things
    def _do(self, name: str) -> None:
        self.actions.append((round(self.t, 2), name))
        if name == "celebrate":
            trick = next((t for t in self.cfg["arrival"]["tricks"] if t in self.tricks), None)
            if trick is None:
                self.mission.action_done("celebrate", False)
                return
            self.walking = False                              # the trick controller takes the legs...
            self.busy_until["celebrate"] = self.t + 4.0
            if self.hands_back:
                self.walking_at = self.t + 4.0
        elif name in ("walk", "stand"):
            self.walking_at = self.t + 0.3                    # ... until gemini_walk is switched back in

    def _move(self, dt: float) -> None:
        vx, vy, wz = self.cmd
        s = self.scene
        c, sn = math.cos(s.yaw), math.sin(s.yaw)
        nx = s.x + (vx * c - vy * sn) * dt * self.slip
        ny = s.y + (vx * sn + vy * c) * dt * self.slip
        s.yaw += wz * dt
        for ox, oy, rad in self.obstacles:
            if math.hypot(nx - ox, ny - oy) < rad <= math.hypot(s.x - ox, s.y - oy):
                return                                        # walked into it: no progress
        s.x, s.y = nx, ny

    def step(self, dt: float = 0.05) -> None:
        self.t += dt
        t, p = self.t, self.perception
        if self.walking_at is not None and t >= self.walking_at:
            self.walking, self.walking_at = True, None
        for name, until in list(self.busy_until.items()):
            if t >= until:
                del self.busy_until[name]
                self.mission.action_done(name, True)
        self._move(dt)
        p.set_imu(t, self.scene.quat())
        p.set_cmd_vel(t, *(self.cmd if self.walking else (0.0, 0.0, 0.0)))
        self._n += 1
        if self._n % 2 == 0:                                  # cameras and the pose model at 10 Hz
            for cam, img in self.frames.items():
                p.set_frame(cam, t, img)
            p.step(t, self.det)
            self.events += p.pop_events()
        fields = dict(now=t, status=p.status(t), status_age=0.0, mux_active=self.mux, walking=self.walking)
        fields.update(self.override)
        out = self.mission.tick(self.Inputs(**fields))
        self.events += out.events
        for a in out.actions:
            self._do(a)
        joystick = fields["mux_active"] in self.cfg["mission"]["joystick_topics"]
        drives = out.cmd is not None and self.mission.motion and self.walking and not joystick
        self.cmd = tuple(out.cmd) if drives else (0.0, 0.0, 0.0)
        if not joystick:
            self.mux = self.cfg["mission"]["cmd_topic"] if drives else "none"
        self.cmds.append(out.cmd)
        if not self.states or self.states[-1] != out.state:
            self.states.append(out.state)
        self.out = out

    def run(self, seconds: float, until=None) -> bool:
        for _ in range(int(round(seconds / 0.05))):
            self.step()
            if until is not None and until(self):
                return True
        return until is None

    def walk(self, person, dx: float, dy: float, speed: float = 1.0) -> None:
        """Run the loop while a person walks (dx, dy) metres."""
        steps = max(1, int(round(math.hypot(dx, dy) / speed / 0.05)))
        for _ in range(steps):
            person.x += dx / steps
            person.y += dy / steps
            self.step()

    def range_to(self, person) -> float:
        return math.hypot(person.x - self.scene.x, person.y - self.scene.y)

    def names(self) -> list:
        return [e["event"] for e in self.events]
