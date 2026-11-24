#!/usr/bin/env python3
"""Print survivor identification's state as one readable line per update (used by robot/watch.sh)."""
from __future__ import annotations

import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

latest = {"perception": {}}


def line(s: dict) -> str:
    p = latest["perception"]
    cams = " ".join("%s:%s" % (name, "ok" if age < 1.0 else "---") for name, age in p.get("cameras", {}).items()) or "no frames"
    flags = ("MOTION ON" if s.get("motion") else "dry-run") + ("" if s.get("walking") else " not-walking") \
        + (" FALLEN" if s.get("fallen") else "") + ("" if s.get("robot_server") else " no-robot-server") \
        + ("" if p.get("pose_model", True) else " NO POSE MODEL") + ("" if p.get("imu_alive", True) else " NO IMU")
    who = ""
    t = s.get("target")
    if t:
        who = " | survivor #%d %+4.0f deg %.1f m %s" % (
            t["id"], t["bearing_deg"], t["range_m"], "in view" if t["visible"] else "unseen %.0f s" % t["since_seen_s"])
    cmd = s.get("cmd")
    drive = "" if not cmd else " | cmd %+.2f %+.2f %+.2f" % tuple(cmd)
    note = (" | " + s["note"]) if s.get("note") else ""
    return "%-9s %s | %d people, %.0f inf/s, %s%s%s%s" % (
        s.get("state"), flags, s.get("people", 0), p.get("infer_hz", 0.0), cams, who, drive, note)


def main() -> None:
    rclpy.init()
    node = Node("survivor_watch")
    node.create_subscription(String, "/survivor/perception", lambda m: latest.__setitem__("perception", json.loads(m.data)), 10)
    node.create_subscription(String, "/survivor/state", lambda m: print(line(json.loads(m.data)), flush=True), 10)
    node.create_subscription(String, "/survivor/event", lambda m: print("EVENT " + m.data, flush=True), 10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
