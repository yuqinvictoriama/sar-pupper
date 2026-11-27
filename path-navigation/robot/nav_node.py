#!/usr/bin/env python3
"""Path navigation node: runs the demo mission on the robot.

    python3 robot/nav_node.py                          dry run: listens, reports, publishes no velocity
    python3 robot/nav_node.py --ros-args -p motion:=true     may walk (someone at the e-stop!)

The thinking is in nav/ (plain Python, tested off the robot, the same code the
simulator runs). This file is the ROS plumbing around it.

In
  the sound module's topic        a bearing to the cry (config/demo.yaml, `topics`; robot/sound_adapter.py)
  /keyword/detected, /keyword/status   "You saved me, Pupper!" (robot/keyword_node.py)
  /imu_sensor_broadcaster/imu     heading and tilt
  /cmd_vel, /cmd_vel_mux/active_source   what the legs were really told, and by whom
  /terrain/state, /terrain/obstacles, /terrain/crossing_status
  /dodge/state, /dodge/event
  /emergency_stop
  /nav/command (std_msgs/String)  start | start! | pause | resume | end | abort | phrase | motion_on | motion_off |
                                  goto:<STATE> | bearing:<degrees, left positive>
                                  ("start!" starts even if a preflight check other than the IMU fails;
                                   "phrase" stands in for the phrase detector; "bearing:" for the sound module)

Out
  /nav_cmd_vel      Twist into the velocity mux. Nothing at all while it listens or waits
  /nav/event        JSON {"event": name, ...}. personality/event_bridge.py passes the spoken ones to Pupper's voice
  /nav/state        JSON, 5 Hz: state, distance along the line, offset, what it is waiting for
  /nav/telemetry    JSON, 5 Hz, only while something subscribes: everything Mission Control draws
  /terrain/mode     full | avoid | report | off, by zone
  /dodge/arm        (service call) armed in the gauntlet only

Every run is logged to data/runs/<date-time>.jsonl: after a bad run that file is the
only way to tell a bad bearing from a bad distance estimate.

Motion is off unless motion:=true (or Mission Control switches it on). With motion off
its events carry "dry_run": true, so Pupper does not say it is coming when it is not.
"""
from __future__ import annotations

import json
import math
import sys
import threading
import time
import urllib.request
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "robot"))

import rclpy  # noqa: E402
from controller_manager_msgs.srv import ListControllers  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402
from std_msgs.msg import Empty, String  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402

from nav.board import Board  # noqa: E402
from nav.config import load_config  # noqa: E402
from nav.mission import IDLE, Mission  # noqa: E402
from sound_adapter import SoundAdapter  # noqa: E402

TICK_HZ = 20.0


def yaw_tilt(o) -> tuple[float, float] | None:
    """(yaw rad, tilt deg) from an orientation quaternion, or None if it is all zeros."""
    if abs(o.x) + abs(o.y) + abs(o.z) + abs(o.w) < 1e-6:
        return None
    yaw = math.atan2(2.0 * (o.w * o.z + o.x * o.y), 1.0 - 2.0 * (o.y * o.y + o.z * o.z))
    up = 1.0 - 2.0 * (o.x * o.x + o.y * o.y)             # the body's up axis against gravity
    return yaw, math.degrees(math.acos(max(-1.0, min(1.0, up))))


class NavNode(Node):
    now = staticmethod(time.monotonic)                    # replaceable clock (tests)

    def __init__(self):
        super().__init__("path_navigation")
        self.declare_parameter("config", "")
        self.declare_parameter("motion", False)
        self.declare_parameter("log", True)
        self.cfg = load_config(self.get_parameter("config").value or None)
        topics = self.cfg.topics
        self.board = Board(self.cfg)
        self.mission = Mission(self.cfg, motion=bool(self.get_parameter("motion").value))
        self.adapter = SoundAdapter(topics)
        self.yaw = 0.0
        self._imu_t = None
        self.mux_source = None
        self.mode_sent, self.mode_sent_t = None, -1e9
        self.arm_asked_t = -1e9
        self.list_pending = False
        self.last_state_pub = self.last_telemetry = self.last_second = -1e9
        self.log_file = None
        self.log_wanted = bool(self.get_parameter("log").value)
        self.running = True

        self.pub_cmd = self.create_publisher(Twist, topics.cmd, 10)
        self.pub_event = self.create_publisher(String, "/nav/event", 20)
        self.pub_state = self.create_publisher(String, "/nav/state", 10)
        self.pub_telemetry = self.create_publisher(String, "/nav/telemetry", 5)
        self.pub_mode = self.create_publisher(String, "/terrain/mode", 10)

        self.create_subscription(self.adapter.message_type(), topics.sound, self._on_sound, 20)
        self.create_subscription(String, topics.sound_status, lambda m: self.board.sound_alive(self.now(), self._json(m)), 5)
        self.create_subscription(String, "/keyword/detected", self._on_keyword, 10)
        self.create_subscription(String, "/keyword/status", lambda m: self.board.keyword_alive(self.now(), self._json(m)), 5)
        self.create_subscription(Imu, "/imu_sensor_broadcaster/imu", self._on_imu, 20)
        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 20)
        self.create_subscription(String, "/cmd_vel_mux/active_source", self._on_mux, 10)
        self.create_subscription(String, "/terrain/state", lambda m: self.board.terrain_state(self.now(), self._json(m)), 10)
        self.create_subscription(String, "/terrain/obstacles", self._on_obstacles, 5)
        self.create_subscription(String, "/terrain/crossing_status", lambda m: self.board.crossing(self.now(), self._json(m)), 20)
        self.create_subscription(String, "/dodge/state", lambda m: self.board.dodge_state(self.now(), self._json(m)), 5)
        self.create_subscription(String, "/dodge/event", lambda m: self.board.dodge_event(self.now(), self._json(m)), 20)
        self.create_subscription(Empty, "/emergency_stop", lambda _m: self.board.estop(self.now()), 10)
        self.create_subscription(String, "/nav/command", self._on_command, 20)
        self.arm_client = self.create_client(SetBool, "/dodge/arm")
        self.list_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.walkers = set(topics.walking_controllers)
        self.sound_topic = topics.sound
        self.create_timer(1.0 / TICK_HZ, self._tick)
        threading.Thread(target=self._voice_watch, daemon=True, name="voice-watch").start()
        self.get_logger().warning(
            "path navigation up: %s; sound bearings on %s (%s, %s, positive %s); config %s" % (
                "MOTION ON - the robot will walk when a run starts" if self.mission.motion else "dry run (motion off)",
                topics.sound, topics.sound_type, topics.sound_units, topics.sound_positive, self.cfg.source))

    # ------------------------------------------------------------------ inputs
    @staticmethod
    def _json(msg) -> dict:
        try:
            data = json.loads(msg.data)
        except ValueError:
            return {"text": str(msg.data)}
        return data if isinstance(data, dict) else {"value": data}

    def _on_sound(self, msg) -> None:
        got = self.adapter.parse(msg)
        if got is not None:
            self.board.bearing(self.now(), got[0], got[1])

    def _on_keyword(self, msg) -> None:
        self.board.keyword_heard(self.now(), str(self._json(msg).get("phrase", "HEARD")))

    def _on_imu(self, msg) -> None:
        now = self.now()
        got = yaw_tilt(msg.orientation)
        if got is not None:
            self.yaw, tilt = got
        else:                                           # no orientation estimate: integrate the yaw rate
            if self._imu_t is not None:
                self.yaw += msg.angular_velocity.z * min(0.1, now - self._imu_t)
            tilt = 0.0
        self._imu_t = now
        self.board.imu(now, self.yaw, tilt)

    def _on_cmd_vel(self, msg) -> None:
        self.board.cmd_vel(self.now(), msg.linear.x, msg.linear.y, msg.angular.z)

    def _on_mux(self, msg) -> None:
        self.mux_source = str(msg.data)
        self.board.mux(self.now(), self.mux_source)

    def _on_obstacles(self, msg) -> None:
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        if isinstance(data, list):
            self.board.terrain_obstacles(self.now(), data)

    def _on_command(self, msg) -> None:
        text = str(msg.data).strip()
        self.get_logger().warning("operator: %s" % text)
        self.board.command(text)

    def _voice_watch(self) -> None:
        """Is anything connected to speak the call-outs? (personality's mission feed)"""
        url = self.cfg.topics.robot_server.rstrip("/") + "/mission/recent"
        while self.running:
            try:
                with urllib.request.urlopen(url, timeout=1.0) as r:
                    info = json.loads(r.read().decode("utf-8", "replace"))
                own = str(info.get("local_voice", "none"))
                self.board.voice(self.now(), int(info.get("listeners", 0) or 0) or (0 if own.startswith("none") else 1))
            except Exception:
                self.board.voice(self.now(), 0)
            time.sleep(2.0)

    # ------------------------------------------------------------------ the loop
    def _tick(self) -> None:
        now = self.now()
        if self.mux_source is not None:
            self.board.mux(now, self.mux_source)        # the mux only reports changes: its last word still stands
        if now - self.last_second >= 1.0:
            self.last_second = now
            self._each_second(now)
        inp = self.board.inputs(now)
        out = self.mission.tick(inp)
        self.board.record(now, out, self.mission)

        if out.cmd is not None:
            t = Twist()
            t.linear.x, t.linear.y, t.angular.z = float(out.cmd[0]), float(out.cmd[1]), float(out.cmd[2])
            self.pub_cmd.publish(t)
        for name, details in out.events:
            event = dict(details, event=name, state=self.mission.state)
            if not self.mission.motion:
                event["dry_run"] = True
            self.pub_event.publish(String(data=json.dumps(event)))
            self.get_logger().info("event: %s %s" % (name, details or ""))
            self._log({"k": "event", "t": round(now, 3), **event})

        # the two reflexes, by zone
        if out.terrain_mode != self.mode_sent or now - self.mode_sent_t > 2.0:
            self.mode_sent, self.mode_sent_t = out.terrain_mode, now
            self.pub_mode.publish(String(data=out.terrain_mode))
        armed = inp.dodge.get("armed")
        if inp.dodge.get("alive") and armed is not None and bool(armed) != out.arm_dodge and now - self.arm_asked_t > 1.0:
            self.arm_asked_t = now
            if self.arm_client.service_is_ready():
                self.arm_client.call_async(SetBool.Request(data=bool(out.arm_dodge)))
                self.get_logger().warning("asked the dodge node to %s" % ("ARM" if out.arm_dodge else "disarm"))

        if now - self.last_state_pub >= 0.2:
            self.last_state_pub = now
            status = self.mission.status()
            self.pub_state.publish(String(data=json.dumps(status)))
            if self.mission.state != IDLE:
                self._log({"k": "state", "t": round(now, 3), "yaw": round(inp.yaw, 4), "cmd": inp.cmd_vel,
                           "mux": inp.mux_source, "terrain": inp.terrain.get("state"), "dodge": inp.dodge.get("state"),
                           **{k: status[k] for k in ("state", "s", "d", "heading_error_deg", "pose", "bearings", "would_cmd")}})
        # Only while Mission Control is listening, and five times a second: the Pi has little to spare
        # (about 80 % busy with the dodge stack alone), and every bit taken here costs camera frames.
        if now - self.last_telemetry >= 0.2 and self.pub_telemetry.get_subscription_count() > 0:
            self.last_telemetry = now
            self.pub_telemetry.publish(String(data=json.dumps(self.board.snapshot(now, self.mission, mode="robot"))))

    def _each_second(self, now: float) -> None:
        if self.count_publishers(self.sound_topic) > 0:
            self.board.sound_alive(now, dict(self.board.sound_status, publishers=True))
        if self.list_client.service_is_ready() and not self.list_pending:
            self.list_pending = True
            self.list_client.call_async(ListControllers.Request()).add_done_callback(self._on_controllers)

    def _on_controllers(self, future) -> None:
        self.list_pending = False
        try:
            result = future.result()
        except Exception:
            return
        active = any(c.state == "active" and (c.name in self.walkers or "neural" in c.type.lower())
                     for c in result.controller)
        self.board.walking_active(self.now(), active)

    # ------------------------------------------------------------------ the run log
    def _log(self, record: dict) -> None:
        if not self.log_wanted:
            return
        try:
            if self.log_file is None:
                folder = ROOT / "data" / "runs"
                folder.mkdir(parents=True, exist_ok=True)
                self.log_file = open(folder / (time.strftime("%Y%m%d-%H%M%S") + ".jsonl"), "a", buffering=1)
                self.log_file.write(json.dumps({"k": "config", "config": self.cfg.to_dict()}) + "\n")
            self.log_file.write(json.dumps(record, default=str) + "\n")
        except OSError as exc:
            self.log_wanted = False
            self.get_logger().error("run log switched off: %r" % (exc,))

    def shutdown(self) -> None:
        self.running = False
        if self.log_file is not None:
            self.log_file.close()


def main() -> None:
    rclpy.init()
    node = NavNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
