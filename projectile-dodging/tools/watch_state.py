#!/usr/bin/env python3
"""Print the dodge node's state as one readable line per update (used by robot/watch.sh)."""
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def line(s: dict) -> str:
    cams = "  ".join("%s %s %2.0ffps %3.0fms lag %2.0fms cpu" % (
        name, "ok " if c["alive"] else "---", c["fps"], c["latency_ms"], c["proc_ms"])
        for name, c in s.get("cameras", {}).items())
    tracks = [t for t in s.get("tracks", []) if t["confirmed"]]
    ball = ""
    if tracks:
        t = max(tracks, key=lambda k: k["closing_speed"])
        ball = " | ball %.1f m at %+4.0f deg, up %+3.0f deg, closing %.1f m/s" % (
            t["range_m"], t["azimuth_deg"], t["elevation_deg"], t["closing_speed"])
    flags = ("ARMED" if s.get("armed") else "dry-run") + ("" if s.get("can_dodge") else " CANNOT DODGE: " + s.get("cannot_reason", ""))
    if not s.get("imu_alive", True):
        flags += " NO IMU"
    return "%-7s %s | %s%s" % (s.get("state"), flags, cams, ball)


def main() -> None:
    rclpy.init()
    node = Node("dodge_watch")
    node.create_subscription(String, "/dodge/state", lambda m: print(line(json.loads(m.data)), flush=True), 10)
    node.create_subscription(String, "/dodge/event", lambda m: print("EVENT " + m.data, flush=True), 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
