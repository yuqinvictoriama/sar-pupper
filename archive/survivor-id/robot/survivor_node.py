#!/usr/bin/env python3
"""ROS 2 node: find people, read the arms-up signal, pin the survivor.

    python3 robot/survivor_node.py --ros-args -p enabled:=true

Cameras in, "who and where" out. It never publishes a velocity; walking up to the survivor is
approach_node.py's job.

    /camera/image_raw/compressed   fisheye (course camera node)            ┐
    /elp_left/image/compressed     rear cameras, republished at 10 Hz      ├─> SurvivorPerception
    /elp_right/image/compressed    by the dodge node, which owns them      │   (survivor/perception.py)
    /imu_sensor_broadcaster/imu    which way is up, heading                │
    /cmd_vel                       dead reckoning while nobody is in view  ┘

    /people                 JSON, every tracked person: id, bearing, range, arms-up score
    /survivor/perception    JSON, 10 Hz: phase, the pin, turn-to-look request (approach_node reads this)
    /survivor/goal          geometry_msgs/PoseStamped in base_link: where the survivor is
    /survivor/event         JSON: candidate, locked, lost, reacquired, cancelled
    /survivor/debug/compressed   the three views with skeletons drawn, 2 Hz, only while someone listens
    ZMQ port 5556           where people are, for the eyes of Gemini's face

    /survivor/enable, /survivor/disable   std_srvs/Trigger: start / stop watching
    /survivor/cancel                      std_srvs/Trigger: drop the current pin

This node holds the Hailo accelerator, which serves one program at a time: the person
follower and hailo_detection must not be running (robot/start.sh sees to it).

Parameters:
    config:=''        another survivor.yaml
    enabled:=true     start watching straight away
    elp:=topic        where rear frames come from: topic (the dodge node republishes them),
                      v4l2 (open the cameras here; only when the dodge node is NOT running), off
    record:=''        folder to save every frame it looks at, with the IMU (tools/replay.py reads it)
"""
from __future__ import annotations

import json
import math
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(ROOT, "tools"))

from survivor.config import load_config, resolve  # noqa: E402
from survivor.perception import SurvivorPerception  # noqa: E402


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class Jpeg:
    """A camera frame as it arrived; decoded the first time somebody looks at it."""

    def __init__(self, data):
        self.data, self.bgr, self.done = data, None, False
        self.lock = threading.Lock()

    def __call__(self):
        with self.lock:
            if not self.done:
                self.bgr = cv2.imdecode(np.frombuffer(self.data, dtype=np.uint8), cv2.IMREAD_COLOR)
                self.done = True
            return self.bgr


class SurvivorNode(Node):
    def __init__(self, make_pose=None):
        super().__init__("survivor_node")
        self.declare_parameter("config", "")
        self.declare_parameter("enabled", True)
        self.declare_parameter("elp", "topic")
        self.declare_parameter("record", "")
        self.cfg = load_config(self.get_parameter("config").value or None)
        self.ncfg = self.cfg["node"]
        self.core = SurvivorPerception(self.cfg, enabled=bool(self.get_parameter("enabled").value))
        self.make_pose = make_pose or self.hailo_pose
        self.pose = None
        self.pose_broken = False
        self.running = True
        self.imu_seen = 0.0
        self.frame_seen: dict = {}
        self.last_error = ""

        self.people_pub = self.create_publisher(String, "/people", 10)
        self.status_pub = self.create_publisher(String, "/survivor/perception", 10)
        self.event_pub = self.create_publisher(String, "/survivor/event", 10)
        self.goal_pub = self.create_publisher(PoseStamped, "/survivor/goal", 10)
        self.debug_pub = self.create_publisher(CompressedImage, "/survivor/debug/compressed", 1)
        self.debug_last = 0.0
        self.create_service(Trigger, "/survivor/enable", lambda _q, r: self.on_enable(True, r))
        self.create_service(Trigger, "/survivor/disable", lambda _q, r: self.on_enable(False, r))
        self.create_service(Trigger, "/survivor/cancel", self.on_cancel)

        self.create_subscription(Imu, self.ncfg.get("imu_topic", "/imu_sensor_broadcaster/imu"), self.on_imu, 50)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)

        self.recorder = None
        record = str(self.get_parameter("record").value or "")
        if record:
            from dodge.recording import RecordingWriter
            path = os.path.join(resolve(record), time.strftime("%Y%m%d-%H%M%S"))
            self.recorder = RecordingWriter(path, meta={"kind": "survivor-id", "config": self.cfg})
            self.get_logger().warning("recording every frame it looks at to %s" % path)

        elp = str(self.get_parameter("elp").value).strip().lower()
        self.usb = []
        for name, cam in self.cfg["cameras"].items():
            if cam.get("source") == "v4l2" and elp == "off":
                continue
            if cam.get("source") == "v4l2" and elp == "v4l2":
                self.open_usb(name, cam)
                continue
            topic = cam.get("topic")
            if not topic:
                self.get_logger().error("%s: no topic in the config; skipping it" % name)
                continue
            self.create_subscription(CompressedImage, topic, lambda msg, n=name: self.on_image(n, msg), 1)

        self.eyes = None
        port = int(self.ncfg.get("eyes_port", 5556) or 0)
        if port:
            sock = None
            try:
                import zmq
                sock = zmq.Context.instance().socket(zmq.PUB)
                sock.bind("tcp://127.0.0.1:%d" % port)
                self.eyes = sock
            except Exception as exc:          # the eyes are a nicety (and another detector may hold the port)
                if sock is not None:
                    sock.close(0)
                self.get_logger().warning("no eye feed on port %d: %r" % (port, exc))

        self.thread = threading.Thread(target=self.pipeline, daemon=True, name="survivor-pipeline")
        self.thread.start()
        self.get_logger().warning("survivor node up: cameras %s, %s" % (
            ", ".join(self.cfg["cameras"]), "watching" if self.core.enabled else "not watching (call /survivor/enable)"))

    # ------------------------------------------------------------------ the pose model
    def hailo_pose(self):
        from hailo_pose import HailoYoloPose
        d = self.cfg["detector"]
        return HailoYoloPose(resolve(d["model"]), float(d["score_threshold"]), float(d["iou_threshold"]), int(d["max_people"]))

    def open_pose(self) -> bool:
        if self.pose is not None:
            return True
        try:
            self.pose = self.make_pose()
            self.last_error = ""
            self.get_logger().info("pose model loaded")
            return True
        except Exception as exc:
            msg = "cannot start the pose model: %r. Is another program using the Hailo (person follower, " \
                  "hailo_detection)? Is the model there (tools/download_model.sh)?" % (exc,)
            if msg != self.last_error:
                self.get_logger().error(msg)
            self.last_error = msg
            return False

    def close_pose(self) -> None:
        pose, self.pose = self.pose, None
        if pose is not None:
            try:
                pose.close()
            except Exception as exc:
                self.get_logger().warning("closing the pose model: %r" % (exc,))

    # ------------------------------------------------------------------ inputs
    def open_usb(self, name: str, cam: dict) -> None:
        if not cam.get("device"):
            self.get_logger().error("%s has no device set (projectile-dodging/tools/find_cameras.py). Skipping it." % name)
            return
        from usb_camera import UsbCamera
        mine = dict(cam)
        mine["max_process_fps"] = float(self.cfg["budget"]["cycle_hz"])
        thread = UsbCamera(name, mine, self.on_usb_frame, log=lambda m: self.get_logger().info(m))
        thread.start()
        self.usb.append(thread)

    def on_usb_frame(self, name: str, t_capture: float, jpeg, bgr) -> None:
        self.take_frame(name, t_capture, bgr, jpeg)

    def on_image(self, name: str, msg: CompressedImage) -> None:
        now = time.time()
        t = stamp_to_sec(msg.header.stamp)
        if abs(now - t) > 2.0:                 # stamp missing or on another clock
            t = now
        t += float(self.cfg["cameras"][name].get("time_offset_s", 0.0))
        self.take_frame(name, t, Jpeg(msg.data), msg.data)

    def take_frame(self, name: str, t: float, frame, jpeg) -> None:
        self.frame_seen[name] = time.time()
        self.core.set_frame(name, t, frame)
        if self.recorder is not None and jpeg is not None:
            self.recorder.add_frame(name, t, bytes(jpeg))

    def on_imu(self, msg: Imu) -> None:
        now = time.time()
        t = stamp_to_sec(msg.header.stamp)
        if abs(now - t) > 2.0:
            t = now
        q, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
        self.core.set_imu(t, (q.w, q.x, q.y, q.z))
        self.imu_seen = now
        if self.recorder is not None:
            self.recorder.add_imu(t, (q.w, q.x, q.y, q.z), (w.x, w.y, w.z), (a.x, a.y, a.z))

    def on_cmd_vel(self, msg: Twist) -> None:
        self.core.set_cmd_vel(time.time(), msg.linear.x, msg.linear.y, msg.angular.z)

    def on_enable(self, on: bool, response):
        self.core.enable(on)
        response.success = True
        response.message = "watching for the signal" if on else "not watching (the Hailo is released)"
        self.get_logger().warning(response.message)
        return response

    def on_cancel(self, _request, response):
        had = self.core.tracker.survivor is not None
        self.core.cancel(time.time())
        response.success = True
        response.message = "pin dropped" if had else "nobody was pinned"
        self.get_logger().warning(response.message)
        return response

    # ------------------------------------------------------------------ the loop
    def detector(self, image, view):
        try:
            return self.pose(image)
        except Exception:
            self.pose_broken = True            # the accelerator went away: reopen it
            raise

    def pipeline(self) -> None:
        period = 1.0 / float(self.cfg["budget"]["cycle_hz"])
        next_people = 0.0
        while self.running:
            t0 = time.time()
            try:
                if not self.core.enabled:
                    self.close_pose()          # leave the accelerator free for whoever else wants it
                elif self.open_pose():
                    self.core.step(t0, self.detector)
                    self.publish_events()
                now = time.time()
                status = self.core.status(now)
                status["imu_alive"] = (now - self.imu_seen) < 0.5
                status["pose_model"] = self.pose is not None
                status["error"] = self.last_error
                self.status_pub.publish(String(data=json.dumps(status)))
                self.publish_goal(status, now)
                if now >= next_people:
                    next_people = now + 1.0 / float(self.ncfg.get("status_hz", 5.0))
                    people = self.core.people(now)
                    self.people_pub.publish(String(data=json.dumps({"t": round(now, 3), "people": people})))
                    self.send_eyes(people, now)
                self.publish_debug(now)
            except Exception as exc:
                self.get_logger().error("pipeline: %r" % (exc,), throttle_duration_sec=2.0)
                if self.pose_broken:
                    self.pose_broken = False
                    self.close_pose()
            spare = period - (time.time() - t0)
            if self.core.enabled and self.pose is None:
                spare = max(spare, 1.0)        # no pose model: try again once a second, not flat out
            if spare > 0:
                time.sleep(spare)

    def publish_events(self) -> None:
        for event in self.core.pop_events():
            text = json.dumps(event)
            self.event_pub.publish(String(data=text))
            self.get_logger().warning("survivor event: " + text)
            if self.recorder is not None:
                self.recorder.add_event(event)

    def publish_goal(self, status: dict, now: float) -> None:
        tgt = status.get("target")
        if tgt is None:
            return
        b, r = math.radians(tgt["bearing_deg"]), float(tgt["range_m"])
        msg = PoseStamped()
        msg.header.frame_id = "base_link"
        msg.header.stamp.sec = int(now)
        msg.header.stamp.nanosec = int((now - int(now)) * 1e9)
        msg.pose.position.x, msg.pose.position.y = r * math.cos(b), r * math.sin(b)
        msg.pose.orientation.z, msg.pose.orientation.w = math.sin(b / 2.0), math.cos(b / 2.0)   # facing along the bearing
        self.goal_pub.publish(msg)

    def send_eyes(self, people: list, now: float) -> None:
        """Where people are, in the JSON the Gemini face's eyes already read (follow_service.py's format)."""
        if self.eyes is None:
            return
        seen = [p for p in people if p["visible"] and abs(p["bearing_deg"]) <= 110.0]
        pinned = [p for p in seen if p["survivor"]]
        out = []
        for p in (pinned or seen):                      # once someone is pinned, the eyes stay on them
            heading = -p["bearing_deg"]                 # the face counts right as positive
            size = max(0.05, min(1.0, 1.7 / max(p["range_m"], 0.5) / math.pi))
            elevation = math.degrees(math.atan2(1.45, max(p["range_m"], 0.3)))
            out.append({"x": 0.5 + heading / 220.0, "y": 0.5 - elevation / 180.0, "width": 0.35 * size, "height": size,
                        "heading": heading, "elevation": elevation, "id": p["id"]})
        try:
            self.eyes.send_string(json.dumps({"people": out, "timestamp": now}))
        except Exception:
            pass

    def publish_debug(self, now: float) -> None:
        hz = float(self.ncfg.get("debug_image_hz", 2.0))
        if hz <= 0 or now - self.debug_last < 1.0 / hz or self.debug_pub.get_subscription_count() == 0:
            return
        self.debug_last = now
        import overlay
        image = overlay.mosaic(self.core, now)
        if image is None:
            return
        ok, enc = cv2.imencode(".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 70])
        if ok:
            msg = CompressedImage()
            msg.format = "jpeg"
            msg.data = enc.tobytes()
            self.debug_pub.publish(msg)

    def shutdown(self) -> None:
        self.running = False
        self.thread.join(timeout=3.0)
        for cam in self.usb:
            cam.stop()
        self.close_pose()
        if self.eyes is not None:
            self.eyes.close(0)
        if self.recorder is not None:
            self.recorder.close()


def main() -> None:
    rclpy.init()
    node = SurvivorNode()
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
