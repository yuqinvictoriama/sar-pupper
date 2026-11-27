#!/usr/bin/env python3
"""ROS 2 node: the partner's sound localisation, on the topics navigation listens to.

It starts sound-localization/localize_angle.py (Max's four-microphone DJI script) exactly as
it is, with the settings in config/demo.yaml section `dji`, and publishes each bearing it prints.
robot/dji_bearing.py does the work; this file is only the ROS side.

Publishes
  /sound/bearing  std_msgs/String  JSON, one per sound event his script locates:
                  {"bearing": deg (0 = nose, POSITIVE RIGHT, his convention), "confidence": 0..1,
                   "check": ok|fit|inconsistent, "fit", "rms", "delays": [CH2, CH3, CH4] samples,
                   "residual", "stamp"}
                  topics.sound_positive = right in demo.yaml turns it into navigation's left-positive
  /sound/status   std_msgs/String  JSON once a second: "alive" is true only while his script is running
                  and has opened a four-channel device; also the device name, counts, the last reading
                  and the last line it printed (an error, if it stopped)

It never commands motion. Run by robot/demo.launch.py (sound:=dji, the default), or by hand:
  source robot/env.sh && python3 robot/sound_node.py
"""
import json
import sys
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(HERE))
from dji_bearing import Localizer  # noqa: E402
from nav.config import load_config  # noqa: E402


class SoundNode(Node):
    def __init__(self):
        super().__init__("sound_node")
        path = self.declare_parameter("config", "").value or None
        self.cfg = load_config(path)
        topics = self.cfg.topics
        self.pub_bearing = self.create_publisher(String, topics.sound, 20)
        self.pub_status = self.create_publisher(String, topics.sound_status, 5)
        self.localizer = Localizer(self.cfg.dji, on_reading=self._on_reading)
        self.get_logger().info("starting the sound module: %s" % " ".join(self.localizer.cmd))
        self.localizer.start()
        self.create_timer(1.0, self._each_second)

    def _on_reading(self, reading, _t) -> None:
        """Called on the reader thread for every estimate line his script prints."""
        if reading["bearing"] is None:
            self.get_logger().info("sound with no direction (%s), not published" % reading["text"])
            return
        stamp = self.get_clock().now().nanoseconds / 1e9
        self.pub_bearing.publish(String(data=json.dumps({
            "bearing": reading["bearing"], "confidence": reading["confidence"], "check": reading["check"],
            "fit": reading["fit"], "rms": reading["rms"], "delays": reading["delays"],
            "residual": reading["residual"], "stamp": stamp})))
        self.get_logger().info("bearing %+.1f deg (right positive), %s, fit %.2f, residual %+.2f, rms %.4f" % (
            reading["bearing"], reading["check"], reading["fit"], reading["residual"], reading["rms"]))

    def _each_second(self) -> None:
        if self.localizer.poll():
            self.get_logger().warning("the sound script had stopped (%s); started it again" %
                                      (self.localizer.status()["last_message"] or "no message"))
        self.pub_status.publish(String(data=json.dumps(self.localizer.status())))


def main():
    rclpy.init()
    node = SoundNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.localizer.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
