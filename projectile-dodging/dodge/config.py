"""Configuration loading: config/dodge.yaml with config/calib/*.yaml merged on top."""
from __future__ import annotations

import copy
import glob
import os
import sys

import yaml

COLOUR_FIT = 2      # how colour limits are fitted; a calibration file made for an earlier way no longer applies
_told = set()

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT_CONFIG = os.path.join(ROOT, "config", "dodge.yaml")
CALIB_DIR = os.path.join(ROOT, "config", "calib")


def deep_merge(base: dict, extra: dict) -> dict:
    """Return base with extra merged in (dicts merge key by key, everything else replaces)."""
    out = copy.deepcopy(base)
    for key, value in (extra or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def load_config(path: str | None = None, calib_dir: str | None = None, use_calib: bool = True) -> dict:
    path = path or DEFAULT_CONFIG
    with open(path) as f:
        cfg = yaml.safe_load(f)
    if use_calib:
        calib_dir = calib_dir or os.path.join(os.path.dirname(os.path.abspath(path)), "calib")
        for name in sorted(glob.glob(os.path.join(calib_dir, "*.yaml"))):
            with open(name) as f:
                data = yaml.safe_load(f) or {}
            for cam, entry in (data.get("cameras") or {}).items():
                color = (entry or {}).get("color")
                if isinstance(color, dict) and color.get("fit") != COLOUR_FIT:
                    # Fitted before the detector took the colour cast out of its pictures:
                    # those limits describe a differently coloured ball. Use the starting values.
                    del entry["color"]
                    if name not in _told:
                        _told.add(name)
                        print("note: the colour calibration in %s is from an earlier version and is being ignored. "
                              "Redo it:  python3 tools/calibrate_color.py %s" % (name, cam), file=sys.stderr)
            cfg = deep_merge(cfg, data)
    cfg["_path"] = os.path.abspath(path)
    return cfg


def save_calib(name: str, data: dict, calib_dir: str | None = None) -> str:
    """Merge data into config/calib/<name>.yaml and return the file path."""
    calib_dir = calib_dir or CALIB_DIR
    os.makedirs(calib_dir, exist_ok=True)
    path = os.path.join(calib_dir, name + ".yaml")
    current = {}
    if os.path.exists(path):
        with open(path) as f:
            current = yaml.safe_load(f) or {}
    with open(path, "w") as f:
        yaml.safe_dump(deep_merge(current, data), f, sort_keys=False)
    return path


def enabled_cameras(cfg: dict) -> dict:
    return {name: c for name, c in cfg["cameras"].items() if c.get("enabled", True)}
