#!/usr/bin/env python3
"""ROS 2 node: tell Pupper's voice what its reflexes just did.

    /nav/event     start, heard, no_sound, stepping_over, going_around, refix,   ┐
                   arrived, saved, hold, cancelled                               ├─> POST /mission/event
    /dodge/event   dodge                                                         ┘    on the robot server

The robot server (started through personality/run.py, which adds the feed) passes each one to
Gemini, or says it itself if Gemini is not there. This node only listens and posts: it
publishes nothing and cannot move the robot. start.sh runs it; stop.sh stops it.

It lives here, not in survivor-id (archived 2026-11-26), and needs nothing from that folder.
Which events exist is callouts.py's business, and repeats are the feed's, so this stays small:

    passed on   an event whose name is a call-out
    dropped     any other name (the dodge node also reports throw_seen, recovered, abort...),
                anything marked dry_run (Pupper is not really doing it, so it does not say so),
                and anything that could not be delivered within a few seconds: a stale
                "Something flew past me" is worse than none

A message is JSON, {"event": "heard", ...}, or just the name as plain text.
Topics: -p topics:="[/nav/event, /dodge/event]"   Server: -p url:=http://127.0.0.1:8000
"""
from __future__ import annotations

import json
import os
import queue
import sys
import threading
import time
import urllib.error
import urllib.request

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from callouts import CALLOUTS  # noqa: E402

TOPICS = ["/nav/event", "/dodge/event"]
URL = "http://127.0.0.1:8000"
STALE_S = 4.0          # an event older than this when its turn comes is not said
SAYS_DISTANCE = ("heard", "arrived", "saved")
DISTANCE_KEYS = ("distance_m", "range_m", "distance", "range")


# ------------------------------------------------------------------ pure logic (desk-tested)
def parse(data: str) -> dict:
    """A message's text -> {"event": name, ...}. Plain text is taken as the name."""
    try:
        event = json.loads(data)
    except ValueError:
        event = {"event": data.strip()}
    if isinstance(event, str):
        event = {"event": event.strip()}
    return event if isinstance(event, dict) else {}


def details_for(event: dict) -> dict:
    """The one number worth saying: how far, in whole metres (to 0.1 m under a metre)."""
    if event.get("event") not in SAYS_DISTANCE:
        return {}
    for key in DISTANCE_KEYS:
        v = event.get(key)
        if isinstance(v, (int, float)) and not isinstance(v, bool) and 0 <= v < 100:
            return {"distance": round(float(v), 1) if v < 1.0 else int(round(float(v)))}
    return {}


def decide(event: dict, announce_dry_run: bool = False) -> tuple[str, dict, str]:
    """(name, details, '') to pass it on, or ('', {}, why not)."""
    name = str(event.get("event", "")).strip()
    if not name:
        return "", {}, "no event name"
    callout = CALLOUTS.get(name)
    if callout is None or callout.internal:
        return "", {}, "not a call-out"
    if event.get("dry_run") and not announce_dry_run:
        return "", {}, "dry run"
    return name, details_for(event), ""


def post(url: str, name: str, details: dict, timeout: float = 1.5) -> tuple[bool, str]:
    """(ok, what the server said). Never raises: the mission carries on without a voice."""
    req = urllib.request.Request(url.rstrip("/") + "/mission/event", method="POST",
                                 data=json.dumps({"event": name, "details": details}).encode(),
                                 headers={"Content-Type": "application/json"})
    try:
        with urllib.request.urlopen(req, timeout=timeout) as r:
            out = json.loads(r.read().decode("utf-8", "replace"))
    except urllib.error.HTTPError as exc:
        return False, "HTTP %d %s" % (exc.code, exc.read().decode("utf-8", "replace")[:200])
    except Exception as exc:                                  # refused, timed out, not JSON...
        return False, repr(exc)
    return bool(out.get("success", True)), str(out.get("message", ""))


class Outbox:
    """Events waiting to be posted. Bounded, and nothing stale leaves it."""

    def __init__(self, size: int = 20, stale_s: float = STALE_S, clock=time.monotonic):
        self.q: queue.Queue = queue.Queue(maxsize=size)
        self.stale_s, self.clock = stale_s, clock

    def put(self, name: str, details: dict) -> None:
        while True:
            try:
                self.q.put_nowait((self.clock(), name, details))
                return
            except queue.Full:                                # drop the oldest, keep the newest
                try:
                    self.q.get_nowait()
                except queue.Empty:
                    pass

    def get(self, timeout: float = 0.5):
        """(name, details), or None if nothing fresh is waiting."""
        try:
            t, name, details = self.q.get(timeout=timeout)
        except queue.Empty:
            return None
        return (name, details) if self.clock() - t <= self.stale_s else None


# ------------------------------------------------------------------ the node
def main() -> None:
    import rclpy
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from std_msgs.msg import String

    class EventBridge(Node):
        def __init__(self):
            super().__init__("mission_event_bridge")
            self.declare_parameter("topics", TOPICS)
            self.declare_parameter("url", URL)
            self.declare_parameter("announce_dry_run", False)
            self.url = str(self.get_parameter("url").value)
            self.announce_dry_run = bool(self.get_parameter("announce_dry_run").value)
            self.outbox, self.running, self.complained = Outbox(), True, 0.0
            topics = [str(t) for t in self.get_parameter("topics").value]
            for topic in topics:
                self.create_subscription(String, topic, lambda m, topic=topic: self.on_event(topic, m), 20)
            threading.Thread(target=self.sender, daemon=True, name="mission-events").start()
            self.get_logger().info("event bridge up: %s -> %s/mission/event" % (", ".join(topics), self.url))

        def on_event(self, topic: str, msg) -> None:
            event = parse(msg.data)
            name, details, why = decide(event, self.announce_dry_run)
            if why:
                self.get_logger().debug("%s %s not passed on: %s" % (topic, event.get("event"), why))
                return
            self.outbox.put(name, details)

        def sender(self) -> None:
            while self.running:
                item = self.outbox.get()
                if item is None:
                    continue
                ok, text = post(self.url, *item)
                if ok:
                    self.get_logger().info("%s: %s" % (item[0], text))
                elif time.time() - self.complained > 30.0:
                    self.complained = time.time()
                    self.get_logger().warning(
                        "'%s' was not said (%s). Is the robot server the mission one (personality/start.sh)? "
                        "python3 personality/doctor.py says what is missing." % (item[0], text))

    rclpy.init()
    node = EventBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
