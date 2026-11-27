"""Shared by the tools: get camera frames (live from the robot, or saved ones) with the
body attitude that goes with each, and edit values in terrain.yaml without losing comments."""

from __future__ import annotations

import json
import re
import sys
import time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from terrain.camera_model import level_from_body  # noqa: E402


def grab(n: int = 10, topic: str = "/camera/image_raw/compressed", timeout: float = 15.0, every: int = 2):
    """n live frames as (bgr, R_level_body, quaternion-or-None); needs the robot stack running."""
    import rclpy
    from sensor_msgs.msg import CompressedImage, Imu

    rclpy.init()
    node = rclpy.create_node("terrain_tool_frames")
    state = {"quat": None, "count": 0}
    frames = []

    def on_imu(msg):
        o = msg.orientation
        if abs(o.x) + abs(o.y) + abs(o.z) + abs(o.w) > 1e-6:
            state["quat"] = (o.x, o.y, o.z, o.w)

    def on_image(msg):
        state["count"] += 1
        if state["count"] % every or len(frames) >= n:
            return
        bgr = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        if bgr is not None:
            q = state["quat"]
            frames.append((bgr, level_from_body(q) if q else np.eye(3), q))

    node.create_subscription(Imu, "/imu_sensor_broadcaster/imu", on_imu, 10)
    node.create_subscription(CompressedImage, topic, on_image, 1)
    t_end = time.monotonic() + timeout
    while len(frames) < n and time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.1)
    had_imu = state["quat"] is not None
    node.destroy_node()
    rclpy.shutdown()
    if not frames:
        raise SystemExit(f"no frames on {topic} in {timeout:.0f} s: is the robot stack (camera node) running?")
    if not had_imu:
        print("warning: no IMU orientation received; assuming the robot is level")
    return frames


def save(frames, folder) -> None:
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    start = len(list(folder.glob("frame_*.jpg")))
    for i, (bgr, _R, q) in enumerate(frames):
        stem = folder / f"frame_{start + i:04d}"
        cv2.imwrite(str(stem) + ".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 95])
        Path(str(stem) + ".json").write_text(json.dumps({"quat_xyzw": q}))


def load(path):
    """Saved frames from a folder (or one image file): same tuples as grab()."""
    path = Path(path)
    files = sorted(path.glob("frame_*.jpg")) if path.is_dir() else [path]
    if path.is_dir() and not files:
        files = sorted(p for p in path.iterdir() if p.suffix.lower() in (".jpg", ".jpeg", ".png"))
    out = []
    for f in files:
        bgr = cv2.imread(str(f))
        if bgr is None:
            continue
        meta = f.with_suffix(".json")
        q = json.loads(meta.read_text()).get("quat_xyzw") if meta.exists() else None
        out.append((bgr, level_from_body(tuple(q)) if q else np.eye(3), tuple(q) if q else None))
    if not out:
        raise SystemExit(f"no images found in {path}")
    return out


def frames_from_args(args, n=10):
    return load(args.frames) if getattr(args, "frames", None) else grab(n, topic=args.topic)


def add_frame_args(ap, cfg=None):
    ap.add_argument("--config", default=None, help="terrain.yaml (default: config/terrain.yaml)")
    ap.add_argument("--frames", default=None, help="use saved frames (a folder from record_frames.py, or one image) "
                                                   "instead of the live camera")
    ap.add_argument("--topic", default="/camera/image_raw/compressed")


def set_yaml_value(path, section: str, key: str, value_text: str) -> bool:
    """Replace ``key: ...`` inside top-level ``section:`` of a YAML file, keeping the rest of
    the file (comments included) as it is. Returns False if the key was not found."""
    lines = Path(path).read_text().splitlines(keepends=True)
    inside = False
    for i, line in enumerate(lines):
        if re.match(r"^\S", line):
            inside = line.startswith(section + ":")
            continue
        m = re.match(rf"^(\s+){re.escape(key)}:\s*([^#\n]*?)(\s*#.*)?\n?$", line) if inside else None
        if m:
            comment = m.group(3) or ""
            lines[i] = f"{m.group(1)}{key}: {value_text}{comment}\n"
            Path(path).write_text("".join(lines))
            return True
    return False


def classify_frame(bgr, R, cfg, floor):
    """(view, polar bgr, hsv, labels, scan, obstacles) for one frame, as the node computes them."""
    from terrain import floor_model as FM
    from terrain.camera_model import PolarView
    from terrain.scan import build_scan, find_obstacles

    view = PolarView(cfg.model.scaled(bgr.shape[1], bgr.shape[0]), cfg.pose)
    pol, valid = view.sample(bgr, R)
    hsv = cv2.cvtColor(pol, cv2.COLOR_BGR2HSV)
    labels = FM.classify(hsv, valid, floor, ball=cfg.ball)
    scan = build_scan(labels, view, cfg.scan)
    return view, pol, hsv, labels, scan, find_obstacles(scan, cfg.scan)


def overlay(view, pol, labels, obstacles, near_depression_deg, scale=4):
    """Picture for a person to check: the levelled polar view (left of the robot on the left,
    horizon near the top) above the same view coloured by class, with a grid."""
    from terrain import floor_model as FM

    table = np.zeros((256, 3), dtype=np.uint8)
    table[FM.FLOOR] = (70, 70, 70)
    table[FM.OBSTACLE] = (0, 0, 255)
    table[FM.BOX_SIDE] = (255, 120, 0)
    table[FM.BOX_TOP] = (255, 255, 255)
    table[FM.BALL] = (0, 255, 0)
    both = np.concatenate([pol, table[labels]], axis=0)[:, ::-1]
    both = cv2.resize(both, None, fx=scale, fy=scale, interpolation=cv2.INTER_NEAREST)
    n_dep, n_az = labels.shape
    dep0, dres = view.depressions[0], view.depressions[1] - view.depressions[0]
    az_max, ares = view.bearings[-1], view.bearings[1] - view.bearings[0]
    for half in (0, 1):
        off = half * n_dep * scale
        for d in range(-10, 71, 10):
            y = off + int(round((d - dep0) / dres * scale))
            colour = (0, 255, 255) if d == 0 else (120, 120, 120)
            cv2.line(both, (0, y), (both.shape[1], y), colour, 1)
            cv2.putText(both, f"{d}", (2, y - 2), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
        y = off + int(round((near_depression_deg - dep0) / dres * scale))
        cv2.line(both, (0, y), (both.shape[1], y), (255, 0, 255), 1)
        for a in range(-90, 91, 30):
            x = int(round((az_max - a) / ares * scale))
            cv2.line(both, (x, off), (x, off + n_dep * scale), (120, 120, 120), 1)
            cv2.putText(both, f"{a:+d}", (x + 2, off + 12), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    for ob in obstacles:
        x = int(round((az_max - 0.5 * (ob.left_deg + ob.right_deg)) / ares * scale))
        tag = ("TALL" if ob.tall else "LOW?") + f" {ob.nearest:.2f}m"
        cv2.putText(both, tag, (max(0, x - 40), n_dep * scale + 28), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
    return both
