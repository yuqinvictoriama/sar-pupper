#!/usr/bin/env python3
"""ROS 2 node for the projectile-dodging reflex.

    python3 robot/dodge_node.py --ros-args -p armed:=false

Cameras in, velocity commands out:

    /camera/image_raw/compressed  (fisheye, from the course camera node)  ┐
    two ELP USB cameras, opened directly                                  ├─> DodgeCore ─> /dodge_cmd_vel
    /imu_sensor_broadcaster/imu                                           ┘                (into cmd_vel_mux)

It also publishes /dodge/state (JSON, 5 Hz) and /dodge/event (JSON, one per
dodge), offers /dodge/arm (std_srvs/SetBool), and saves a short recording
around every dodge to data/events/ so each one can be replayed afterwards.

This node holds the two USB cameras, so nothing else can open them. For
survivor identification it passes their frames on, as the camera's own JPEGs,
on /elp_left/image/compressed and /elp_right/image/compressed: at most
node.share_frames_hz a second each, and only while something subscribes.

With armed:=false (the default) it does everything except move the robot: it
reports the dodge it would have made. Arm it only with someone at the e-stop.
"""
from __future__ import annotations

import json
import os
import sys
import threading
import time

import cv2
import numpy as np
import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Imu
from std_msgs.msg import Empty, Float32MultiArray, String
from std_srvs.srv import SetBool

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)

from dodge.config import enabled_cameras, load_config  # noqa: E402
from dodge.core import DodgeCore  # noqa: E402
from dodge.detect import BallDetector  # noqa: E402
from dodge.recording import RingRecorder  # noqa: E402
from usb_camera import UsbCamera  # noqa: E402


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class DodgeNode(Node):
    def __init__(self):
        super().__init__("dodge_node")
        self.declare_parameter("config", "")
        self.declare_parameter("armed", False)
        self.declare_parameter("record_events", True)
        self.declare_parameter("cameras", "")        # comma-separated subset, empty = all enabled in the config

        self.cfg = load_config(self.get_parameter("config").value or None)
        ncfg = self.cfg["node"]
        # Each camera already has a thread of its own. Left alone, OpenCV also splits every
        # operation across all the cores, and three cameras' worth of that fight each other
        # and the walking controller for the Pi's four cores.
        cv2.setNumThreads(int(ncfg.get("opencv_threads", 1)))
        only = [c.strip() for c in str(self.get_parameter("cameras").value).split(",") if c.strip()]
        if only:
            for name, cam in self.cfg["cameras"].items():
                cam["enabled"] = cam.get("enabled", True) and name in only
        self.cam_cfgs = enabled_cameras(self.cfg)
        self.core = DodgeCore(self.cfg, armed=bool(self.get_parameter("armed").value))
        self.det_cfg = self.cfg["detector"]
        self.detectors = {}
        for name, cam in self.cam_cfgs.items():
            mask = None
            mask_file = cam.get("mask_file")
            if mask_file:
                path = mask_file if os.path.isabs(mask_file) else os.path.join(ROOT, mask_file)
                mask = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
                if mask is None:
                    self.get_logger().warning("%s: mask file %s could not be read" % (name, path))
            self.detectors[name] = BallDetector.for_camera(cam, self.det_cfg, mask)
        self.roi_tracking = bool(self.det_cfg.get("roi_tracking", True))
        self.full_every = max(1, int(self.det_cfg.get("full_frame_every", 3)))
        self.frame_no = {name: 0 for name in self.cam_cfgs}
        self.cam_stats = {name: {"frames": 0, "blobs": 0, "latency": 0.0, "proc_ms": 0.0, "last": 0.0}
                          for name in self.cam_cfgs}
        self.stats_lock = threading.Lock()

        self.record_events = bool(self.get_parameter("record_events").value)
        self.ring = RingRecorder(float(ncfg.get("event_buffer_s", 1.5)) + float(ncfg.get("event_post_s", 1.0)))
        self.event_post = float(ncfg.get("event_post_s", 1.0))
        self.event_dir = ncfg.get("event_dir", "data/events")
        if not os.path.isabs(self.event_dir):
            self.event_dir = os.path.join(ROOT, self.event_dir)

        # ---- outputs
        self.cmd_pub = self.create_publisher(Twist, ncfg.get("cmd_topic", "/dodge_cmd_vel"), 10)
        self.state_pub = self.create_publisher(String, "/dodge/state", 10)
        self.event_pub = self.create_publisher(String, "/dodge/event", 10)
        self.pub_lock = threading.Lock()
        self.create_service(SetBool, "/dodge/arm", self.on_arm)

        # ---- inputs
        self.create_subscription(Imu, ncfg.get("imu_topic", "/imu_sensor_broadcaster/imu"), self.on_imu, 50)
        self.create_subscription(Empty, ncfg.get("estop_topic", "/emergency_stop"), self.on_estop, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(Float32MultiArray, ncfg.get("free_space_topic", "/free_space"), self.on_free_space, 5)
        self.free_space_time = 0.0
        self.imu_seen = 0.0

        self.walking = set(ncfg.get("walking_controllers", []))
        self.require_controller = bool(ncfg.get("require_active_controller", True))
        self.list_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.list_pending = False
        if self.require_controller:
            self.core.set_can_dodge(False, "controller state not known yet")

        # ---- cameras
        self.usb = []
        self.usb_by_name = {}
        self.fisheye_name = None
        self.fisheye_msg = None
        self.fisheye_event = threading.Event()
        self.running = True
        for name, cam in self.cam_cfgs.items():
            src = cam.get("source")
            if src == "v4l2":
                if not cam.get("device"):
                    self.get_logger().error(
                        "%s has no device set: run tools/find_cameras.py and put the path in config/calib. Skipping it." % name)
                    continue
                cam_thread = UsbCamera(name, cam, self.on_usb_frame, log=lambda m: self.get_logger().info(m))
                cam_thread.start()
                self.usb.append(cam_thread)
                self.usb_by_name[name] = cam_thread
            elif src == "ros_compressed":
                self.fisheye_name = name
                self.create_subscription(CompressedImage, cam.get("topic", "/camera/image_raw/compressed"),
                                         self.on_fisheye_msg, 1)
                threading.Thread(target=self.fisheye_worker, daemon=True, name="fisheye").start()
            else:
                self.get_logger().error("%s: unknown source %r" % (name, src))

        # ---- rear-camera frames for other nodes (survivor identification)
        self.share_hz = float(ncfg.get("share_frames_hz", 10.0))
        self.share_pubs = {}
        self.share_last = {}
        if self.share_hz > 0:
            for name, cam in self.cam_cfgs.items():
                if cam.get("source") == "v4l2":
                    self.share_pubs[name] = self.create_publisher(CompressedImage, "/%s/image/compressed" % name, 1)

        self.create_timer(1.0 / float(ncfg.get("cmd_rate_hz", 100.0)), self.on_cmd_timer)
        self.create_timer(0.2, self.on_status_timer)
        self.create_timer(1.0, self.on_controller_timer)
        self.get_logger().warning("dodge node up: cameras %s, %s" % (
            ", ".join(self.cam_cfgs), "ARMED - the robot will move" if self.core.armed else "not armed (dry run)"))

    # ------------------------------------------------------------------ frames
    def process_frame(self, name: str, t_frame: float, bgr, jpeg) -> None:
        t0 = time.time()
        t_raw = t_frame                      # recordings keep the camera's own stamp
        t_frame += float(self.cam_cfgs[name].get("time_offset_s", 0.0))
        size = (bgr.shape[1], bgr.shape[0])
        det = self.detectors[name]
        self.frame_no[name] += 1
        blobs = None
        if self.roi_tracking and self.frame_no[name] % self.full_every != 0:
            pred = self.core.predicted_pixel(name, t_frame, size)
            if pred is not None:
                u, v, half = pred
                blobs = det.detect(bgr, roi=(u - half, v - half, u + half, v + half))
                if not blobs:
                    blobs = None          # not where expected: look everywhere
        if blobs is None:
            blobs = det.detect(bgr)
        started = self.core.process_blobs(name, t_frame, blobs, time.time(), size)
        if started:
            self.publish_command()        # do not wait for the timer
        now = time.time()
        with self.stats_lock:
            s = self.cam_stats[name]
            s["frames"] += 1
            s["blobs"] += len(blobs)
            s["latency"] += 0.1 * ((now - t_frame) - s["latency"])
            s["proc_ms"] += 0.1 * ((now - t0) * 1000.0 - s["proc_ms"])
            s["last"] = now
        if self.record_events and jpeg is not None:
            self.ring.add_frame(name, t_raw, jpeg)

    def on_usb_frame(self, name: str, t_capture: float, jpeg, bgr) -> None:
        try:
            self.process_frame(name, t_capture, bgr, jpeg)
        except Exception as exc:
            self.get_logger().error("%s: frame processing failed: %r" % (name, exc), throttle_duration_sec=2.0)
        try:
            self.share_frame(name, t_capture, jpeg, bgr)          # after the ball: dodging comes first
        except Exception as exc:
            self.get_logger().error("%s: could not pass the frame on: %r" % (name, exc), throttle_duration_sec=5.0)

    def share_frame(self, name: str, t_capture: float, jpeg, bgr) -> None:
        """Pass a rear-camera frame on to whoever subscribes, a few times a second."""
        pub = self.share_pubs.get(name)
        if pub is None or t_capture - self.share_last.get(name, 0.0) < 0.95 / self.share_hz:
            return
        if pub.get_subscription_count() == 0:
            return
        self.share_last[name] = t_capture
        if jpeg is None:                   # camera not in pass-through mode: encode this one frame
            ok, jpeg = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                return
        msg = CompressedImage()
        msg.header.stamp.sec = int(t_capture)
        msg.header.stamp.nanosec = int((t_capture - int(t_capture)) * 1e9)
        msg.header.frame_id = name
        msg.format = "jpeg"
        msg.data = jpeg.tobytes()
        with self.pub_lock:
            pub.publish(msg)

    def on_fisheye_msg(self, msg: CompressedImage) -> None:
        self.fisheye_msg = (msg, time.time())      # keep only the newest; decoding happens in the worker
        self.fisheye_event.set()

    def fisheye_worker(self) -> None:
        while self.running:
            if not self.fisheye_event.wait(timeout=0.5):
                continue
            self.fisheye_event.clear()
            item, self.fisheye_msg = self.fisheye_msg, None
            if item is None:
                continue
            msg, t_rx = item
            try:
                data = np.frombuffer(msg.data, dtype=np.uint8)
                bgr = cv2.imdecode(data, cv2.IMREAD_COLOR)
                if bgr is None:
                    continue
                t = stamp_to_sec(msg.header.stamp)
                if abs(t_rx - t) > 2.0:            # stamp missing or on another clock
                    t = t_rx
                self.process_frame(self.fisheye_name, t, bgr, data if self.record_events else None)
            except Exception as exc:
                self.get_logger().error("fisheye: frame processing failed: %r" % (exc,), throttle_duration_sec=2.0)

    # ------------------------------------------------------------------ other inputs
    def on_imu(self, msg: Imu) -> None:
        t = stamp_to_sec(msg.header.stamp)
        now = time.time()
        if abs(now - t) > 2.0:
            t = now
        q, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
        quat, gyro = (q.w, q.x, q.y, q.z), (w.x, w.y, w.z)
        self.core.set_imu(t, quat, gyro)
        self.imu_seen = now
        if self.record_events:
            self.ring.add_imu(t, quat, gyro, (a.x, a.y, a.z))

    def on_cmd_vel(self, msg: Twist) -> None:
        self.core.set_robot_velocity(time.time(), msg.linear.x, msg.linear.y)

    def on_estop(self, _msg: Empty) -> None:
        self.core.emergency_stop(time.time())
        self.get_logger().warning("emergency stop seen: dodge cancelled")

    def on_free_space(self, msg: Float32MultiArray) -> None:
        self.core.set_free_space(list(msg.data))
        self.free_space_time = time.time()

    def on_arm(self, request, response):
        self.core.set_armed(bool(request.data))
        response.success = True
        response.message = "armed: the robot will move to dodge" if request.data else "disarmed: dry run only"
        self.get_logger().warning(response.message)
        return response

    def on_controller_timer(self) -> None:
        if not self.require_controller:
            self.core.set_can_dodge(True)
            return
        if not self.list_client.service_is_ready():
            self.core.set_can_dodge(False, "controller manager not reachable")
            return
        if self.list_pending:
            return
        self.list_pending = True
        future = self.list_client.call_async(ListControllers.Request())
        future.add_done_callback(self.on_controllers)

    def on_controllers(self, future) -> None:
        self.list_pending = False
        try:
            result = future.result()
        except Exception as exc:
            self.core.set_can_dodge(False, "controller query failed: %r" % (exc,))
            return
        active = [c.name for c in result.controller if c.state == "active" and c.name in self.walking]
        if active:
            self.core.set_can_dodge(True, "walking with %s" % active[0])
        else:
            self.core.set_can_dodge(False, "no walking controller active (mid-trick, relaxed or fallen)")

    # ------------------------------------------------------------------ outputs
    def publish_command(self) -> None:
        cmd = self.core.command(time.time())
        if cmd is None:
            return
        msg = Twist()
        msg.linear.x, msg.linear.y, msg.angular.z = float(cmd[0]), float(cmd[1]), float(cmd[2])
        with self.pub_lock:
            self.cmd_pub.publish(msg)

    def on_cmd_timer(self) -> None:
        self.publish_command()
        for event in self.core.pop_events():
            text = json.dumps(event, default=str)
            with self.pub_lock:
                self.event_pub.publish(String(data=text))
            self.get_logger().warning("dodge event: " + text)
            if event["event"] == "dodge" and self.record_events:
                threading.Timer(self.event_post, self.dump_event, args=(event,)).start()

    def dump_event(self, event: dict) -> None:
        try:
            name = time.strftime("%Y%m%d-%H%M%S", time.localtime(event["t"])) + ("-dry" if event.get("dry_run") else "")
            path = self.ring.dump(os.path.join(self.event_dir, name),
                                  meta={"kind": "dodge event", "event": event, "config": self.cfg}, events=[event])
            self.get_logger().info("saved %s" % path)
        except Exception as exc:
            self.get_logger().error("could not save the event recording: %r" % (exc,))

    def on_status_timer(self) -> None:
        now = time.time()
        if self.free_space_time and now - self.free_space_time > 1.0:
            self.core.set_free_space(None)          # stale: do not trust it
            self.free_space_time = 0.0
        status = self.core.snapshot(now)
        cams = {}
        with self.stats_lock:
            for name, s in self.cam_stats.items():
                s["fps"] = s.get("fps", 0.0) + 0.3 * (s["frames"] / 0.2 - s.get("fps", 0.0))
                cams[name] = {"fps": round(s["fps"], 1), "latency_ms": round(s["latency"] * 1000.0, 1),
                              "proc_ms": round(s["proc_ms"], 1), "alive": (now - s["last"]) < 1.0,
                              "blobs": s["blobs"]}
                usb = self.usb_by_name.get(name)
                if usb is not None:
                    cams[name]["brightness"] = round(usb.brightness, 0)
                    if usb.problem:
                        cams[name]["problem"] = usb.problem
                s["frames"] = 0
                s["blobs"] = 0
        status["cameras"] = cams
        status["imu_alive"] = (now - self.imu_seen) < 0.5
        with self.pub_lock:
            self.state_pub.publish(String(data=json.dumps(status, default=str)))

    def shutdown(self) -> None:
        self.running = False
        for cam in self.usb:
            cam.stop()
        for cam in self.usb:
            cam.close(2.0)        # released before Python exits; exiting mid-capture aborts the process


def main() -> None:
    rclpy.init()
    node = DodgeNode()
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
