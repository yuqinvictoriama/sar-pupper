#!/usr/bin/env python3
"""ROS 2 node: tell Gemini what Pupper's reflexes just did.

    /survivor/event   locked, blocked, lost, reacquired, arrived, cancelled   ┐
    /dodge/event      dodge                                                   ├─> POST /mission/event
    /survivor/state   (only to know whether motion is on)                     ┘    on the robot server

The robot server (started through personality/run.py, which adds the feed) passes each one to
the web app as "MISSION EVENT: arrived range=0.7", the app types it into the Live session, and
the mission prompt has a line for Gemini to say. If the server or Gemini is not there, nothing
else is affected: the bark and the bounce do not come through here.

What is not passed on (config/survivor.yaml, "events"): candidate (the prompt stays silent
until the lock), dodges the dodge node only reported in dry run, repeats of the same event
within a few seconds, and - while the approach node's motion is off - everything from the
survivor nodes, because "I'm coming to you" would not be true.
"""
from __future__ import annotations

import json
import os
import queue
import sys
import threading
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)

from robot_client import RobotClient  # noqa: E402
from survivor.config import load_config  # noqa: E402


def details_for(event: dict) -> dict:
    """The few numbers worth saying. The prompt allows the range, in whole metres."""
    out = {}
    rng = event.get("range_m")
    if isinstance(rng, (int, float)) and event.get("event") in ("locked", "reacquired", "arrived"):
        out["range"] = round(float(rng), 1) if rng < 1.0 else int(round(float(rng)))
    return out


class Filter:
    """Which events go through (pure logic, tested off the robot)."""

    def __init__(self, cfg: dict):
        e = cfg.get("events", {})
        self.skip = set(e.get("skip", ["candidate"]))
        self.gaps = {k: float(v) for k, v in (e.get("min_gap_s") or {}).items()}
        self.default_gap = float(e.get("default_gap_s", 2.0))
        self.announce_dry_run = bool(e.get("announce_dry_run", False))
        self.last: dict = {}

    def accept(self, source: str, event: dict, now: float, motion: bool) -> str:
        """'' to pass it on, else why not."""
        name = str(event.get("event", ""))
        if not name:
            return "no event name"
        if name in self.skip:
            return "not announced"
        if event.get("dry_run"):
            return "dry run"
        if source == "dodge" and name != "dodge":
            return "not announced"
        if source == "survivor" and not motion and not self.announce_dry_run:
            return "motion is off"
        if now - self.last.get(name, -1e9) < self.gaps.get(name, self.default_gap):
            return "said a moment ago"
        self.last[name] = now
        return ""


class EventBridge(Node):
    def __init__(self, client=None):
        super().__init__("survivor_event_bridge")
        self.declare_parameter("config", "")
        self.cfg = load_config(self.get_parameter("config").value or None)
        rs = self.cfg["robot_server"]
        self.client = client or RobotClient(rs["url"], float(rs["timeout_s"]))
        self.filter = Filter(self.cfg)
        self.motion = False
        self.complained = 0.0
        self.outbox: queue.Queue = queue.Queue()
        self.running = True
        self.create_subscription(String, "/survivor/event", lambda m: self.on_event("survivor", m), 20)
        self.create_subscription(String, "/dodge/event", lambda m: self.on_event("dodge", m), 20)
        self.create_subscription(String, "/survivor/state", self.on_state, 10)
        threading.Thread(target=self.sender, daemon=True, name="mission-events").start()
        self.get_logger().info("event bridge up: mission events go to %s/mission/event" % rs["url"])

    def on_state(self, msg: String) -> None:
        try:
            self.motion = bool(json.loads(msg.data).get("motion", False))
        except ValueError:
            pass

    def on_event(self, source: str, msg: String) -> None:
        try:
            event = json.loads(msg.data)
        except ValueError:
            event = {"event": msg.data.strip()}
        if not isinstance(event, dict):
            return
        why = self.filter.accept(source, event, time.time(), self.motion)
        if why:
            self.get_logger().info("%s event %s not passed to Gemini: %s" % (source, event.get("event"), why))
            return
        self.outbox.put((event["event"], details_for(event)))

    def sender(self) -> None:
        while self.running:
            try:
                name, details = self.outbox.get(timeout=0.5)
            except queue.Empty:
                continue
            ok, text = self.client.mission_event(name, **details)
            if ok:
                self.get_logger().info("told Gemini: %s" % text)
            elif time.time() - self.complained > 30.0:
                self.complained = time.time()
                self.get_logger().warning(
                    "could not pass '%s' to Gemini (%s). Is the robot server the mission one "
                    "(personality/start.sh, not the lab's start.sh)?" % (name, text))

    def shutdown(self) -> None:
        self.running = False


def main() -> None:
    rclpy.init()
    node = EventBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
