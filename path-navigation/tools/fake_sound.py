#!/usr/bin/env python3
"""Send a bearing by hand, as if the sound module had heard a cry.

    python3 tools/fake_sound.py 30            a cry 30 degrees to Pupper's LEFT, three times a second apart
    python3 tools/fake_sound.py -45 --n 1     one bearing, 45 degrees to the right
    python3 tools/fake_sound.py 0 --noise 5   straight ahead, with +-5 degrees of scatter

It publishes on the sound module's own topic, in the form config/demo.yaml says that module
uses (section `topics`), so the navigation node cannot tell the difference. Use it to test
seek, orient and the second call before the real module is wired in, to check the sign
convention (a positive number here must turn Pupper to its left), and as the last resort on
demo day if the module is not ready. Run it on the Pupper after `source robot/env.sh`.

The angle you type is always "degrees from the nose, left positive"; this tool converts it
to the module's convention before sending.
"""
import argparse
import json
import math
import random
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "robot"))

from nav.config import load_config  # noqa: E402
from sound_adapter import SoundAdapter  # noqa: E402


def to_module(adapter: SoundAdapter, left_deg: float) -> float:
    """Degrees from the nose (left positive) -> the number the module would report."""
    rad = adapter.sign * (math.radians(left_deg) - adapter.zero)
    return rad / adapter.scale


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bearing", type=float, help="degrees from Pupper's nose, left positive")
    ap.add_argument("--n", type=int, default=3, help="how many to send (a fix needs 3 that agree)")
    ap.add_argument("--gap", type=float, default=1.0, help="seconds between them")
    ap.add_argument("--noise", type=float, default=0.0, help="scatter, degrees")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()
    cfg = load_config(args.config)
    adapter = SoundAdapter(cfg.topics)

    import rclpy
    rclpy.init()
    node = rclpy.create_node("fake_sound")
    pub = node.create_publisher(adapter.message_type(), cfg.topics.sound, 10)
    time.sleep(0.5)                                 # let the subscriber find us
    for k in range(args.n):
        left = args.bearing + random.uniform(-args.noise, args.noise)
        value = to_module(adapter, left)
        msg = adapter.message_type()()
        if adapter.kind == "json":
            msg.data = json.dumps({adapter.key: round(value, 3), "confidence": 1.0})
        elif adapter.kind == "vector3":
            msg.vector.x, msg.vector.y = math.cos(math.radians(left)), math.sin(math.radians(left))
        else:
            msg.data = float(value)
        pub.publish(msg)
        print("sent %s on %s: %.1f deg to the %s" % (
            adapter.kind, cfg.topics.sound, abs(left), "left" if left >= 0 else "right"))
        rclpy.spin_once(node, timeout_sec=0.05)
        if k + 1 < args.n:
            time.sleep(args.gap)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
