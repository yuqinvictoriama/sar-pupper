#!/usr/bin/env python3
"""ROS 2 node: listens on the microphones and says when it hears "You saved me, Pupper!".

Publishes
  /keyword/detected  std_msgs/String  JSON {"phrase", "channel", "stamp"}   once per time it is heard
  /keyword/status    std_msgs/String  JSON {"alive", "device", "level_db"}  once a second (preflight heartbeat)

It only reports. Whether a detection ends the search (far enough along the
line, in the right state) is the navigation node's decision.

The operator can stand in for it at any time:
  ros2 topic pub -1 /keyword/detected std_msgs/String "data: '{\"phrase\": \"OPERATOR\", \"channel\": -1}'"
"""
import json
import sys
import threading
import time
from pathlib import Path

import rclpy
import yaml
from rclpy.node import Node
from std_msgs.msg import String

HERE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(HERE))
from keyphrase import PhraseSpotter  # noqa: E402
from keyphrase.audio import mic_blocks  # noqa: E402


class KeywordNode(Node):
    def __init__(self):
        super().__init__("keyword_node")
        cfg = yaml.safe_load((HERE / "config" / "keyword.yaml").read_text())["audio"]
        self.device = self.declare_parameter("device", cfg["device"] or "").value or None
        self.channels = self.declare_parameter("channels", cfg["channels"]).value
        self.rate = self.declare_parameter("sample_rate", cfg["sample_rate"]).value
        self.detected = self.create_publisher(String, "/keyword/detected", 10)
        self.status = self.create_publisher(String, "/keyword/status", 10)
        self.spotter = PhraseSpotter(channels=self.channels)
        self.last_audio = 0.0
        self.stop = threading.Event()
        self.create_timer(1.0, self.publish_status)
        self.thread = threading.Thread(target=self.listen, daemon=True)
        self.thread.start()

    def listen(self):
        while not self.stop.is_set():
            try:
                for block, rate in mic_blocks(self.device, self.channels, self.rate, stop=self.stop):
                    self.last_audio = time.monotonic()
                    for hit in self.spotter.feed(block, rate):
                        stamp = self.get_clock().now().nanoseconds / 1e9
                        self.detected.publish(String(data=json.dumps(
                            {"phrase": hit.phrase, "channel": hit.channel, "stamp": stamp})))
                        self.get_logger().info(f"heard the phrase ({hit.phrase}, mic {hit.channel})")
            except Exception as e:  # device unplugged or busy: say so, retry
                self.get_logger().error(f"microphone problem: {e}; retrying in 2 s")
                self.stop.wait(2.0)

    def publish_status(self):
        self.status.publish(String(data=json.dumps({
            "alive": time.monotonic() - self.last_audio < 1.5,  # audio arrived in the last 1.5 s
            "device": self.device or "default",
            "level_db": [round(float(x), 1) for x in self.spotter.level_db]})))


def main():
    rclpy.init()
    node = KeywordNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop.set()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
