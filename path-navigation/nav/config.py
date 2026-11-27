"""Every number of the demo, in one place.

The defaults below are the demo. config/demo.yaml overrides them, section by
section, so a number changed for the room lives in that file and nowhere else.
Unknown keys in the file are an error: a mistyped name must not be ignored
silently on demo day.

Angles are degrees in the file and in these classes (people read them);
nav/ converts to radians where it does the maths.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass, field, fields, is_dataclass
from pathlib import Path

DEFAULT_PATH = Path(__file__).resolve().parent.parent / "config" / "demo.yaml"


@dataclass
class Course:
    """The course, measured along the line from the start mark (metres)."""
    length: float = 6.0             # Pupper's body centre when it stands in front of the cover
    # The obstacles of the first stretch, in the order Pupper meets them. near: the near edge, metres from
    # the start mark. depth: the way Pupper walks. width: across the line. shape: flat (a board or slab,
    # which terrain agility can cross) or round (a cylinder: it measures it, refuses, and walks around).
    # Navigation uses `near` (each crossing is a landmark) and how many there are; the rest is for the
    # simulator and Mission Control's map. What terrain agility does with each is its own decision.
    obstacles: list = field(default_factory=lambda: [
        {"name": "low bar", "shape": "flat", "near": 0.45, "depth": 0.089, "width": 0.60, "height": 0.019},
        {"name": "slab", "shape": "flat", "near": 1.15, "depth": 0.38, "width": 0.565, "height": 0.015},
    ])
    refix_at: float = 2.00          # second listen: not before this
    throw_from: float = 3.00        # throw window, by Pupper's body centre
    throw_to: float = 5.20
    cover_near: float = 6.40        # cover, near face
    cover_width: float = 1.20
    thrower_along: float = 4.00     # throwers stand level with this
    thrower_side: float = 3.00      # and this far either side of the line


@dataclass
class Sound:
    """What the sound module is taken to give (the partner's module; see robot/sound_adapter.py)."""
    moe_deg: float = 5.0            # its margin of error, as given: about +-5 degrees
    min_bearings: int = 3           # bearings that must agree before a fix is believed
    spread_deg: float = 8.0         # they agree if they are within this of their median
    window_s: float = 20.0          # bearings older than this are forgotten (one bearing per call: SOUND_INTEGRATION.md)
    min_confidence: float = 0.5     # bearings reported with less confidence are logged, not used (dji checks)
    still_s: float = 0.6            # Pupper must have stood still this long for a bearing to count
    no_sound_after_s: float = 20.0  # say "I can't hear anyone yet"
    turn_after_s: float = 60.0      # then turn a quarter and listen again


@dataclass
class Dji:
    """How robot/sound_node.py starts the partner's script, sound-localization/localize_angle.py.

    Every value is passed to it on its command line; the defaults are his script's own defaults.
    """
    script: str = "sound-localization/localize_angle.py"   # relative to path-navigation/
    python: str = "python3"         # the interpreter that has numpy and sounddevice (on the Pupper: system python3)
    device: str = ""                # "" = his auto-select (a name with "DJI" or "Wireless Mic Rx"); or an index or name part
    rate: int = 48000
    width_mm: float = 120.0         # left/right spacing of the transmitters, CH1-CH2 and CH3-CH4
    height_mm: float = 234.0        # front/back spacing, CH1-CH3 and CH2-CH4
    threshold: float = 0.01         # RMS (0..1, all four channels) a sound must reach to be located
    frame_seconds: float = 0.35
    deadband_samples: float = 1.5
    # Checks made on his numbers (robot/dji_bearing.py): a bearing failing one gets a lower confidence
    fit_low: float = 0.5            # fit magnitude is cos(elevation); his script warns below 0.15, the course never goes below 0.8
    fit_high: float = 1.25          # his script's own limit
    caution_confidence: float = 0.3  # confidence of a bearing whose fit is outside fit_low..fit_high
    max_residual_samples: float = 3.0  # |CH4 - (CH2 + CH3)| above this: a pair is wrong, confidence 0
    restart_s: float = 3.0          # if his script exits (receiver unplugged), start it again after this


@dataclass
class Orient:
    turn_rate: float = 0.8          # rad/s, in place
    min_rate: float = 0.25          # rad/s: slower than this the legs do not turn
    gain: float = 2.0               # 1/s
    done_deg: float = 5.0
    settle_s: float = 0.5
    timeout_s: float = 14.0
    confirm_ok_deg: float = 10.0    # second listen within this of straight ahead: correct the line and go
    confirm_redo_deg: float = 25.0  # beyond this: the first fix was wrong, listen again from scratch
    confirm_timeout_s: float = 6.0  # nothing heard: go with the first fix
    confirm_min_bearings: int = 1
    confirm_blend: float = 0.5      # weight of the first fix when the two agree (0 = trust only the second listen)
    max_reorients: int = 2


@dataclass
class Walk:
    # Commanded speeds. Both are 0.30 because that is the one speed the legs' real speed has been measured
    # at (see reckon.forward_factor): on the floor Pupper then moves at about 0.16 m/s.
    cruise: float = 0.30            # m/s commanded in zone 1
    gauntlet: float = 0.30          # m/s commanded in zone 2
    look_ahead: float = 1.0         # m: aim at the point this far ahead on the line
    max_off_line_deg: float = 25.0  # never point further than this from the line to get back to it
    yaw_gain: float = 1.5           # 1/s
    max_yaw_rate: float = 0.6       # rad/s
    turn_in_place_deg: float = 35.0  # heading error beyond which it stops walking and turns
    min_forward: float = 0.10       # m/s while walking and turning at once
    brake_s: float = 0.5            # s of zero commands when stopping, before going silent
    back_off_tries: int = 2         # times it may step back when terrain agility cannot find a way round
    back_off_m: float = 0.45
    back_off_speed: float = 0.15    # m/s, backwards


@dataclass
class Reckon:
    """Position from the commands the legs were sent. Calibrate the factors on the demo floor."""
    # MEASURED ONCE, 2026-11-26, on the owner's floor: 0.80 m in 5 s from standing with 0.30 m/s commanded.
    # Re-measure over 4 m at walk.cruise before trusting any distance (README, "First time on the robot").
    forward_factor: float = 0.53    # actual / commanded, forwards, at the speeds above
    sideways_factor: float = 0.53   # not measured: taken to be the same
    lag_s: float = 0.2              # the legs follow a command about this late
    restart_s: float = 4.0          # after a crossing the walking controller takes this long to come back
    crossing_advance: float = 0.87  # m the body moves in a crossing, if the step-over node does not say (slab 0.87, bar 0.59)
    gauge_distance: float = 0.36    # m from the box where terrain agility stops to measure (its fsm.gauge_distance)


@dataclass
class Refix:
    enabled: bool = True
    min_bearings: int = 1           # the survivor calls twice; one good bearing is enough
    gather_s: float = 2.5           # after the first bearing, wait this long for the second
    accept_deg: float = 20.0        # a bearing further than this from where the survivor should be is an echo
    timeout_s: float = 6.0          # nothing heard: keep the old line
    need_walk_around: bool = True   # wait for the tall obstacle to be passed, not only for the distance
    fallback_extra: float = 0.8     # m past refix_at at which it stops for the second listen regardless


@dataclass
class Gauntlet:
    arm_dodge: bool = True
    obstacle_stop_range: float = 0.60   # m: something tall this close in the lane, before the cover is expected
    obstacle_wait_s: float = 3.0        # stand this long; if it is still there, walk around it
    lane_half: float = 0.20


@dataclass
class Arrive:
    keyword_from: float = 4.0       # the phrase is ignored (and logged) before this distance
    cover_from: float = 4.8         # a tall thing ahead past this distance is the cover
    cover_cone_deg: float = 35.0
    stop_range: float = 0.45        # m from the lens to the cover at which to stop
    home_on_cover: bool = True      # steer at the middle of the cover once it is seen
    home_max_deg: float = 20.0
    hard_stop: float = 7.2          # m: stop here whatever else is true
    blind_stop: float = 6.0         # m: stop here if terrain agility's camera has gone quiet
    repeat_s: float = 10.0          # say "I'm here" again this often while waiting for the phrase
    give_up_s: float = 30.0         # then tell the team and wait for the operator


@dataclass
class Safety:
    stale_s: float = 1.0            # a component not heard from for this long is not alive
    tilt_deg: float = 25.0          # more than this: fallen or being carried, pause
    joystick_release_s: float = 1.0  # carry on this long after the joystick lets go
    require: list = field(default_factory=lambda: ["imu", "walking", "sound", "keyword", "terrain", "dodge"])


@dataclass
class Topics:
    """ROS names. /sound/bearing and /sound/status are published by robot/sound_node.py, which runs the
    partner's localize_angle.py and passes on its numbers unchanged: degrees, positive to the RIGHT."""
    cmd: str = "/nav_cmd_vel"
    sound: str = "/sound/bearing"
    sound_type: str = "json"        # json | float32 | float64 | vector3
    sound_units: str = "deg"        # deg | rad
    sound_positive: str = "right"   # left (ROS) | right (compass). localize_angle.py prints positive to the right
    sound_zero_deg: float = 0.0     # where the module's zero points, measured from the nose, left positive
    sound_json_key: str = "bearing"
    sound_status: str = "/sound/status"
    walking_controllers: list = field(default_factory=lambda: [
        "gemini_walk", "neural_controller", "neural_controller_default", "neural_controller_default_alt",
        "neural_controller_student"])
    robot_server: str = "http://127.0.0.1:8000"


@dataclass
class DemoConfig:
    course: Course = field(default_factory=Course)
    sound: Sound = field(default_factory=Sound)
    dji: Dji = field(default_factory=Dji)
    orient: Orient = field(default_factory=Orient)
    walk: Walk = field(default_factory=Walk)
    reckon: Reckon = field(default_factory=Reckon)
    refix: Refix = field(default_factory=Refix)
    gauntlet: Gauntlet = field(default_factory=Gauntlet)
    arrive: Arrive = field(default_factory=Arrive)
    safety: Safety = field(default_factory=Safety)
    topics: Topics = field(default_factory=Topics)
    source: str = "built-in defaults"

    def to_dict(self) -> dict:
        return asdict(self)


def apply(cfg: DemoConfig, raw: dict) -> DemoConfig:
    """Lay a {section: {key: value}} dictionary over the defaults. Unknown names raise."""
    sections = {f.name: f for f in fields(cfg) if is_dataclass(getattr(cfg, f.name))}
    for name, values in (raw or {}).items():
        if name not in sections:
            raise KeyError(f"demo config: no section called '{name}' (have {sorted(sections)})")
        target = getattr(cfg, name)
        known = {f.name: f for f in fields(target)}
        for key, value in (values or {}).items():
            if key not in known:
                raise KeyError(f"demo config: '{name}.{key}' is not a setting (have {sorted(known)})")
            current = getattr(target, key)
            if isinstance(current, bool):
                value = bool(value)
            elif isinstance(current, float):
                value = float(value)
            elif isinstance(current, int):
                value = int(value)
            setattr(target, key, value)
    return cfg


def load_config(path=None, overrides: dict | None = None) -> DemoConfig:
    """config/demo.yaml over the defaults. Without PyYAML the defaults are used and `source` says so."""
    cfg = DemoConfig()
    p = Path(path) if path else DEFAULT_PATH
    if p.exists():
        try:
            import yaml
        except ImportError:
            cfg.source = f"built-in defaults (PyYAML is not installed, so {p.name} was NOT read)"
        else:
            apply(cfg, yaml.safe_load(p.read_text()) or {})
            cfg.source = str(p)
    if overrides:
        apply(cfg, overrides)
    return cfg
