#!/usr/bin/env python3
"""ROS 2 node: walk up to the pinned survivor, stop, bark, bounce, stay.

    python3 robot/approach_node.py --ros-args -p motion:=false

It turns what survivor_node.py sees into velocity commands for the mux (survivor/mission.py
holds the logic):

    /survivor/perception          who is pinned and where (survivor_node)            ┐
    /cmd_vel_mux/active_source    who is driving (joystick? dodge? us?)              │
    /dodge/state, /dodge/event    is a ball in the air                               ├─> Mission ─> /survivor_cmd_vel
    /terrain/state, /terrain/event    step-over in progress, no way past             │              (into cmd_vel_mux, below
    /imu_sensor_broadcaster/imu   standing level?                                    │               dodge and terrain)
    controller manager            is a walking controller holding the legs           │
    robot server (port 8000)      fallen? clear Gemini's queue, stand up, play a trick ┘

    /survivor/state     JSON, 5 Hz: SCAN, CANDIDATE, LOOK, LOCKED, APPROACH, BLOCKED, REACQUIRE,
                        ARRIVED, CELEBRATE, STAY or PAUSED, with the pin and why
    /survivor/event     JSON: blocked, arrived (survivor_node publishes the others)
    /survivor/motion    std_srvs/SetBool: let it move the robot (true) or not (false)

With motion:=false (the default) it goes through every state and reports what it would do,
but publishes no velocity and asks the robot server for nothing. Turn motion on only with
someone at the e-stop.

Parameters:
    config:=''        another survivor.yaml
    motion:=false     let it move the robot
    drive:=''         direct or route (default from the config: direct)
"""
from __future__ import annotations

import json
import math
import os
import queue
import shutil
import subprocess
import sys
import threading
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import SetBool

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)

from robot_client import RobotClient  # noqa: E402
from survivor.cameras import quat_to_rot  # noqa: E402
from survivor.config import load_config, resolve  # noqa: E402
from survivor.mission import Inputs, Mission  # noqa: E402


def word(text: str, key: str) -> str:
    """A topic that may carry JSON ({"state": "STEP_OVER"}) or just the word."""
    text = (text or "").strip()
    if text.startswith("{"):
        try:
            return str(json.loads(text).get(key, ""))
        except ValueError:
            return ""
    return text


class ApproachNode(Node):
    def __init__(self, client=None):
        super().__init__("survivor_approach")
        self.declare_parameter("config", "")
        self.declare_parameter("motion", False)
        self.declare_parameter("drive", "")
        self.cfg = load_config(self.get_parameter("config").value or None)
        self.m, self.a = self.cfg["mission"], self.cfg["arrival"]
        self.mission = Mission(self.cfg, motion=bool(self.get_parameter("motion").value),
                               drive=str(self.get_parameter("drive").value or "") or None)
        rs = self.cfg["robot_server"]
        self.client = client or RobotClient(rs["url"], float(rs["timeout_s"]))
        self.lock = threading.Lock()

        self.status, self.status_at = None, 0.0
        self.mux_active = "none"
        self.walking = False
        self.walking_known = 0.0
        self.fallen = False
        self.server_ok = False
        self.tilt_deg, self.imu_at = 0.0, 0.0
        self.terrain_state, self.terrain_at, self.terrain_blocked = None, 0.0, False
        self.ball_at = 0.0                 # last time the dodge node had a ball or dodged
        self.last_out = None

        self.cmd_pub = self.create_publisher(Twist, self.m["cmd_topic"], 10)
        self.state_pub = self.create_publisher(String, "/survivor/state", 10)
        self.event_pub = self.create_publisher(String, "/survivor/event", 10)
        self.create_service(SetBool, "/survivor/motion", self.on_motion)

        self.create_subscription(String, "/survivor/perception", self.on_status, 10)
        self.create_subscription(String, "/cmd_vel_mux/active_source", self.on_mux, 10)
        self.create_subscription(String, "/dodge/state", self.on_dodge_state, 10)
        self.create_subscription(String, "/dodge/event", self.on_dodge_event, 10)
        self.create_subscription(String, "/terrain/state", self.on_terrain_state, 10)
        self.create_subscription(String, "/terrain/event", self.on_terrain_event, 10)
        self.create_subscription(Imu, self.cfg["node"].get("imu_topic", "/imu_sensor_broadcaster/imu"), self.on_imu, 10)

        self.walking_names = set(self.cfg["dodge"].get("node", {}).get("walking_controllers", ["gemini_walk"]))
        self.list_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.list_pending = False

        self.jobs: queue.Queue = queue.Queue()
        self.running = True
        threading.Thread(target=self.worker, daemon=True, name="survivor-actions").start()
        threading.Thread(target=self.server_watch, daemon=True, name="survivor-server-watch").start()

        self.create_timer(1.0 / float(self.m["rate_hz"]), self.on_tick)
        self.create_timer(1.0 / float(self.cfg["node"].get("status_hz", 5.0)), self.on_state_timer)
        self.create_timer(1.0, self.on_controller_timer)
        self.get_logger().warning("approach node up: %s, drive=%s" % (
            "MOTION ON - the robot will walk to a pinned survivor" if self.mission.motion else "motion off (dry run)",
            self.mission.drive))

    # ------------------------------------------------------------------ inputs
    def on_status(self, msg: String) -> None:
        try:
            status = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.status, self.status_at = status, time.time()

    def on_mux(self, msg: String) -> None:
        self.mux_active = msg.data.strip() or "none"

    def on_dodge_state(self, msg: String) -> None:
        try:
            s = json.loads(msg.data)
        except ValueError:
            return
        # SEARCH = nothing tracked. Anything else (TRACK, THREAT, DODGE, RECOVER) = a ball is about.
        if str(s.get("state", "SEARCH")).upper() != "SEARCH" or any(t.get("confirmed") for t in s.get("tracks", [])):
            self.ball_at = time.time()

    def on_dodge_event(self, msg: String) -> None:
        self.ball_at = time.time()

    def on_terrain_state(self, msg: String) -> None:
        self.terrain_state, self.terrain_at = word(msg.data, "state").upper() or None, time.time()

    def on_terrain_event(self, msg: String) -> None:
        if word(msg.data, "event").lower() in ("blocked", "no_way", "no_route", "blocked_both_sides"):
            self.terrain_blocked = True

    def on_imu(self, msg: Imu) -> None:
        q = msg.orientation
        up_z = float(quat_to_rot(q.w, q.x, q.y, q.z)[2, 2])       # how much of the body's "up" is really up
        self.tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, up_z))))
        self.imu_at = time.time()

    def on_motion(self, request, response):
        self.mission.set_motion(bool(request.data))
        response.success = True
        response.message = "motion on: the robot will walk to a pinned survivor" if request.data else "motion off: dry run"
        self.get_logger().warning(response.message)
        return response

    def on_controller_timer(self) -> None:
        if not self.list_client.service_is_ready():
            self.walking = False
            return
        if self.list_pending:
            return
        self.list_pending = True
        self.list_client.call_async(ListControllers.Request()).add_done_callback(self.on_controllers)

    def on_controllers(self, future) -> None:
        self.list_pending = False
        try:
            result = future.result()
        except Exception:
            self.walking = False
            return
        self.walking = any(c.state == "active" and c.name in self.walking_names for c in result.controller)
        self.walking_known = time.time()

    def server_watch(self) -> None:
        """Once a second: is the robot server there, and does it say Pupper has fallen over?"""
        while self.running:
            self.server_ok = self.client.reachable()
            if self.server_ok:
                ok, text = self.client.mode()
                self.fallen = bool(ok and "fallen" in text.lower())
            time.sleep(1.0)

    # ------------------------------------------------------------------ the tick
    def on_tick(self) -> None:
        now = time.time()
        with self.lock:
            status, age = self.status, now - self.status_at
        terrain = self.terrain_state if now - self.terrain_at <= float(self.m["terrain_stale_s"]) else None
        blocked, self.terrain_blocked = self.terrain_blocked, False
        inp = Inputs(now=now, status=status, status_age=age, mux_active=self.mux_active, walking=self.walking,
                     fallen=self.fallen, level=self.tilt_deg <= float(self.a["level_tilt_deg"]) and now - self.imu_at < 1.0,
                     terrain_state=terrain, terrain_blocked=blocked, dodge_quiet_s=now - self.ball_at, server=self.server_ok)
        out = self.mission.tick(inp)
        self.last_out = out
        if out.cmd is not None and self.mission.motion:
            msg = Twist()
            msg.linear.x, msg.linear.y, msg.angular.z = float(out.cmd[0]), float(out.cmd[1]), float(out.cmd[2])
            self.cmd_pub.publish(msg)
        for event in out.events:
            if not self.mission.motion:
                event = dict(event, dry_run=True)
            text = json.dumps(event)
            self.event_pub.publish(String(data=text))
            self.get_logger().warning("survivor event: " + text)
        for action in out.actions:
            self.jobs.put(action)

    def on_state_timer(self) -> None:
        out = self.last_out
        if out is None:
            return
        with self.lock:
            status = self.status or {}
        tgt = status.get("target")
        state = {"t": round(time.time(), 3), "state": out.state, "note": out.note, "motion": self.mission.motion,
                 "drive": self.mission.drive, "target": tgt, "look": status.get("look"),
                 "people": status.get("people", 0), "queued": status.get("queued", []),
                 "walking": self.walking, "fallen": self.fallen, "robot_server": self.server_ok,
                 "mux": self.mux_active, "cmd": None if out.cmd is None else [round(float(v), 2) for v in out.cmd],
                 "can_dodge": out.state != "CELEBRATE"}
        self.state_pub.publish(String(data=json.dumps(state)))

    # ------------------------------------------------------------------ things that take time
    def worker(self) -> None:
        while self.running:
            try:
                action = self.jobs.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                getattr(self, "do_" + action)()
            except Exception as exc:
                self.get_logger().error("%s failed: %r" % (action, exc))
                self.mission.action_done(action, False)

    def do_bark(self) -> None:
        path = resolve(self.a.get("bark", ""))
        player = next((p for p in ("pw-play", "paplay", "aplay") if shutil.which(p)), None)
        if not os.path.exists(path) or player is None:
            self.get_logger().info("no bark: %s" % ("sound file %s missing" % path if player else "no audio player found"))
            return
        subprocess.Popen([player, path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def do_clear_queue(self) -> None:
        ok, text = self.client.stop_now()
        self.get_logger().info("cleared Gemini's queue and follow mode: %s" % text if ok else "could not clear the queue: %s" % text)

    def do_stand(self) -> None:
        ok, text = self.client.stand_up()
        self.get_logger().info("stand up: %s" % text)

    do_walk = do_stand

    def do_celebrate(self) -> None:
        """Queue the bounce on the robot server and wait until it has played."""
        reached, tricks = self.client.tricks()
        name = next((t for t in self.a["tricks"] if t in tricks), None)
        if not reached or name is None:
            self.get_logger().warning("no bounce: %s" % ("robot server not reachable" if not reached else
                                                         "none of %s is installed (have: %s)" % (self.a["tricks"], sorted(tricks))))
            self.mission.action_done("celebrate", False)
            return
        ok, text = self.client.play(name)
        self.get_logger().warning("bounce (%s): %s" % (name, text))
        if not ok:
            self.mission.action_done("celebrate", False)
            return
        deadline = time.time() + tricks[name] + 20.0
        time.sleep(1.0)
        while self.running and time.time() < deadline:
            reached, busy = self.client.busy()
            if reached and not busy:
                break
            time.sleep(0.25)
        self.mission.action_done("celebrate", True)

    def shutdown(self) -> None:
        self.running = False
        if self.mission.motion:
            self.cmd_pub.publish(Twist())          # a last zero, so the mux drops us standing still


def main() -> None:
    rclpy.init()
    node = ApproachNode()
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
