"""The one file that knows how the sound-direction module talks.

The module is the project partner's. Its topic name, message type and sign
convention had not been seen when this was written, so they are settings
(config/demo.yaml, section `topics`) and this file turns whatever arrives into
the one thing navigation uses:

    bearing in radians from Pupper's nose, positive to its LEFT, and a confidence 0..1

Check the sign with one clap before anything else (tools/fake_sound.py and
Mission Control's compass both show which way a bearing points): if it is
wrong, Pupper turns confidently the wrong way.

    sound_type      what the topic carries
    ----------      ----------------------
    json            std_msgs/String: {"bearing": 12.5, "confidence": 0.8}, or just "12.5"
    float32         std_msgs/Float32: the bearing
    float64         std_msgs/Float64: the bearing
    vector3         geometry_msgs/Vector3Stamped: a direction vector in the body frame (x forward, y left)

    sound_units     deg | rad
    sound_positive  left (the ROS convention) | right (a compass)
    sound_zero_deg  where the module's zero points, measured from the nose, left positive
                    (0 = its zero is the nose; 90 = its zero is Pupper's left side)

No ROS import at module level: the conversion is tested on a laptop.
"""
from __future__ import annotations

import json
import math

TYPES = ("json", "float32", "float64", "vector3")


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


class SoundAdapter:
    def __init__(self, topics):
        """topics: nav.config.Topics (or anything with the same sound_* attributes)."""
        self.kind = str(topics.sound_type).lower()
        if self.kind not in TYPES:
            raise ValueError(f"topics.sound_type '{topics.sound_type}' is not one of {TYPES}")
        self.scale = math.pi / 180.0 if str(topics.sound_units).lower().startswith("deg") else 1.0
        self.sign = -1.0 if str(topics.sound_positive).lower().startswith("r") else 1.0
        self.zero = math.radians(float(topics.sound_zero_deg))
        self.key = str(topics.sound_json_key)

    def convert(self, value: float) -> float:
        """The module's number -> radians from the nose, left positive."""
        return wrap(self.sign * float(value) * self.scale + self.zero)

    def from_text(self, text: str):
        """(bearing rad, confidence) from a String message, or None if it holds no bearing."""
        text = text.strip()
        try:
            data = json.loads(text)
        except ValueError:
            return None
        if isinstance(data, bool):
            return None
        if isinstance(data, (int, float)):
            return self.convert(data), 1.0
        if not isinstance(data, dict):
            return None
        value = data.get(self.key)
        if value is None:
            for alt in ("bearing", "bearing_deg", "azimuth", "azimuth_deg", "angle", "doa", "direction"):
                if isinstance(data.get(alt), (int, float)):
                    value = data[alt]
                    break
        if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(value):
            return None
        conf = data.get("confidence", data.get("conf", 1.0))
        conf = float(conf) if isinstance(conf, (int, float)) and not isinstance(conf, bool) else 1.0
        return self.convert(value), max(0.0, min(1.0, conf))

    def from_vector(self, x: float, y: float):
        if abs(x) + abs(y) < 1e-9:
            return None
        # a vector is already an angle in the body frame; only the sign and zero settings apply
        return wrap(self.sign * math.atan2(y, x) + self.zero), 1.0

    def parse(self, msg):
        """(bearing rad, confidence) from a ROS message of the configured type, or None."""
        if self.kind == "json":
            return self.from_text(str(msg.data))
        if self.kind in ("float32", "float64"):
            value = float(msg.data)
            return (self.convert(value), 1.0) if math.isfinite(value) else None
        v = msg.vector
        return self.from_vector(float(v.x), float(v.y))

    def message_type(self):
        """The ROS message class for the configured type (imported here, on the robot)."""
        if self.kind == "json":
            from std_msgs.msg import String
            return String
        if self.kind == "float32":
            from std_msgs.msg import Float32
            return Float32
        if self.kind == "float64":
            from std_msgs.msg import Float64
            return Float64
        from geometry_msgs.msg import Vector3Stamped
        return Vector3Stamped
