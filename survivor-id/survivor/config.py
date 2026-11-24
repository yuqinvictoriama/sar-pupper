"""Configuration: config/survivor.yaml with config/calib/*.yaml merged on top, plus the
camera lens models and mounting poses from the projectile-dodging component."""
from __future__ import annotations

import copy
import glob
import os
import sys

import yaml

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))          # .../survivor-id
REPO = os.path.dirname(ROOT)
DODGE_ROOT = os.environ.get("SAR_DODGE_DIR") or os.path.join(REPO, "projectile-dodging")
DEFAULT_CONFIG = os.path.join(ROOT, "config", "survivor.yaml")


def use_dodge_code() -> str:
    """Make the dodging component's packages importable (lens models, recordings, USB capture)."""
    if not os.path.isdir(os.path.join(DODGE_ROOT, "dodge")):
        raise SystemExit("projectile-dodging not found at %s: survivor ID shares its camera models and calibration. "
                         "Set SAR_DODGE_DIR if it is somewhere else." % DODGE_ROOT)
    for p in (os.path.join(DODGE_ROOT, "robot"), DODGE_ROOT):
        if p not in sys.path:
            sys.path.insert(0, p)
    return DODGE_ROOT


def deep_merge(base: dict, extra: dict) -> dict:
    out = copy.deepcopy(base)
    for key, value in (extra or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


def load_config(path: str | None = None, use_calib: bool = True, dodge_config: str | None = None) -> dict:
    """The survivor configuration. cfg["dodge"] holds the dodging configuration it was built on;
    each cfg["cameras"][name] gains that camera's "model", "intrinsics" and "extrinsics" from it."""
    path = path or DEFAULT_CONFIG
    with open(path) as f:
        cfg = yaml.safe_load(f)
    if use_calib:
        for name in sorted(glob.glob(os.path.join(os.path.dirname(os.path.abspath(path)), "calib", "*.yaml"))):
            with open(name) as f:
                cfg = deep_merge(cfg, yaml.safe_load(f) or {})
    use_dodge_code()
    from dodge.config import load_config as load_dodge
    dodge = load_dodge(dodge_config, use_calib=use_calib)
    cams = {}
    for name, ours in cfg.get("cameras", {}).items():
        theirs = dodge.get("cameras", {}).get(name)
        if theirs is None or not theirs.get("enabled", True) or not ours.get("enabled", True):
            continue
        cams[name] = deep_merge(theirs, ours)
    cfg["cameras"] = cams
    cfg["dodge"] = dodge
    cfg["_path"] = os.path.abspath(path)
    return cfg


def resolve(path: str) -> str:
    """A path from the config: ~ expanded, relative paths taken from the survivor-id folder."""
    path = os.path.expanduser(str(path))
    return path if os.path.isabs(path) else os.path.join(ROOT, path)
