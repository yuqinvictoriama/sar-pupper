"""Loads config/terrain.yaml into the objects the rest of the package uses."""

from __future__ import annotations

import json
from dataclasses import fields, replace
from pathlib import Path

from .camera_model import CameraPose, DoubleSphere
from .crawl_planner import CrawlConfig
from .decision import DecisionConfig
from .floor_model import ColorRange, FloorModel
from .player import PlayerConfig
from .scan import ScanConfig
from .state_machine import FsmConfig
from .steering import SteerConfig

DEFAULT_PATH = Path(__file__).resolve().parent.parent / "config" / "terrain.yaml"


def _pick(cls, d):
    """Only the keys of d that are fields of dataclass cls."""
    names = {f.name for f in fields(cls)}
    return {k: v for k, v in (d or {}).items() if k in names}


class TerrainConfig:
    def __init__(self, path=None):
        import yaml
        self.path = Path(path) if path else DEFAULT_PATH
        raw = yaml.safe_load(self.path.read_text()) if self.path.exists() else {}
        self.raw = raw or {}
        cam = self.raw.get("camera", {})
        self.model = DoubleSphere(
            fx=float(cam.get("fx", 228.8103)), fy=float(cam.get("fy", 228.8103)), cx=float(cam.get("cx", 680.0)),
            cy=float(cam.get("cy", 479.0)), xi=float(cam.get("xi", -0.00092)), alpha=float(cam.get("alpha", 0.62007)),
            width=int(cam.get("image_width", 1400)), height=int(cam.get("image_height", 1050)))
        self.pose = CameraPose(**_pick(CameraPose, cam))
        self.near_depression_deg = float(cam.get("near_depression_deg", 60.0))
        self.camera_topic = cam.get("topic", "/camera/image_raw/compressed")

        col = self.raw.get("colors", {})
        self.box_side = ColorRange.from_dict(col.get("box_side", {"h_lo": 100, "h_hi": 125}))
        self.box_top = ColorRange.from_dict(col.get("box_top", {"h_lo": 0, "h_hi": 179, "s_min": 0, "s_max": 50, "v_min": 185}))
        self.ball = ColorRange.from_dict(col.get("ball", {"h_lo": 30, "h_hi": 52}))
        self.floor_file = self.path.parent / col.get("floor_file", "floor_model.json")
        self.adapt_floor = bool(col.get("adapt_floor", True))

        sc = self.raw.get("scan", {})
        self.scan_rate = float(sc.get("rate_hz", 8.0))
        self.scan = ScanConfig(**_pick(ScanConfig, sc), near_depression_deg=self.near_depression_deg)

        self.decision = DecisionConfig(**_pick(DecisionConfig, self.raw.get("decision", {})))
        self.steer = SteerConfig(**_pick(SteerConfig, self.raw.get("steer", {})))
        fsm = self.raw.get("fsm", {})
        self.fsm = FsmConfig(**_pick(FsmConfig, fsm), steer=self.steer, decision=self.decision)
        self.intent_topics = list(fsm.get("intent_topics", ["/llm_cmd_vel", "/person_following_cmd_vel"]))
        self.cmd_topic = fsm.get("cmd_topic", "/terrain_cmd_vel")
        self.level_tilt_deg = float(fsm.get("level_tilt_deg", 8.0))

        cr = dict(self.raw.get("crawl", {}))
        self.allow_search = bool(cr.pop("allow_search", False))
        masses = tuple((float(m[0]), [float(m[1]), float(m[2]), float(m[3])]) for m in cr.pop("extra_masses", []) or [])
        self.crawl = replace(CrawlConfig(**_pick(CrawlConfig, cr)), extra_masses=masses)
        pl = self.raw.get("player", {})
        self.player = PlayerConfig(**_pick(PlayerConfig, pl), kp=self.crawl.kp, kd=self.crawl.kd)
        self.stepover = dict({"controllers": ["terrain_position_controller", "terrain_kp_controller", "terrain_kd_controller"],
                              "walking_controller_types": ["neural_controller/NeuralController"], "rate_hz": 120.0,
                              "estop_kd": 0.1, "request_topic": "/terrain/step_over",
                              "status_topic": "/terrain/crossing_status"}, **self.raw.get("stepover", {}))

    def load_floor(self) -> FloorModel:
        if self.floor_file.exists():
            return FloorModel.from_dict(json.loads(self.floor_file.read_text()))
        return FloorModel()

    def save_floor(self, model: FloorModel) -> None:
        self.floor_file.write_text(json.dumps(model.to_dict()))
