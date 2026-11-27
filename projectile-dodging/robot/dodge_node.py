#!/usr/bin/env python3
"""ROS 2 node for the projectile-dodging reflex.

    python3 robot/dodge_node.py --ros-args -p armed:=false

Cameras in, velocity commands out:

    /camera/image_raw/compressed  (fisheye, from the course camera node)  ┐
    two ELP USB cameras, opened directly                                  ├─> DodgeCore ─> /dodge_cmd_vel
    /imu_sensor_broadcaster/imu                                           ┘                (into cmd_vel_mux)

Each camera is captured, decoded and searched for the ball in a process of its
own (robot/camera_worker.py); this process gets only what they found, keeps the
track and decides. So no camera waits for another, or for anything here.

It also publishes /dodge/state (JSON, 5 Hz) and /dodge/event (JSON, one per
dodge), offers /dodge/arm (std_srvs/SetBool), and keeps the throw recorder:
every dodge, every throw it saw but did not dodge, and every manual mark (a
message on /dodge/mark) is saved to data/throws/ with the camera frames, the
IMU and a trace of what the node detected and did, frame by frame.
tools/dodging_view.py (Pupper Dodging View) shows them in a browser; tools/replay.py re-runs them.

With armed:=false (the default) it does everything except move the robot: it
reports the dodge it would have made. Arm it only with someone at the e-stop.
"""
from __future__ import annotations

import os

os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")    # before numpy loads: its maths library otherwise keeps a
os.environ.setdefault("OMP_NUM_THREADS", "1")         # thread per core spinning, for 3x3 matrices (40 % of a core each)

import json  # noqa: E402
import shutil
import sys
import threading
import time

import rclpy
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Empty, Float32MultiArray, String
from std_srvs.srv import SetBool

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)

from dodge.config import enabled_cameras, load_config  # noqa: E402
from dodge.core import DodgeCore  # noqa: E402
from dodge.recording import RingRecorder  # noqa: E402
from camera_worker import CameraFarm  # noqa: E402


def stamp_to_sec(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


class DodgeNode(Node):
    def __init__(self):
        super().__init__("dodge_node")
        self.declare_parameter("config", "")
        self.declare_parameter("armed", False)
        self.declare_parameter("record_events", True)
        self.declare_parameter("cameras", "")        # comma-separated subset, empty = all enabled in the config
        self.declare_parameter("imu_topic", "")      # empty = the config's (the launch file gives a thinned-out copy)

        self.cfg = load_config(self.get_parameter("config").value or None)
        ncfg = self.cfg["node"]
        only = [c.strip() for c in str(self.get_parameter("cameras").value).split(",") if c.strip()]
        if only:
            for name, cam in self.cfg["cameras"].items():
                cam["enabled"] = cam.get("enabled", True) and name in only
        self.cam_cfgs = enabled_cameras(self.cfg)
        self.core = DodgeCore(self.cfg, armed=bool(self.get_parameter("armed").value))
        self.cam_stats = {name: {"frames": 0, "blobs": 0, "latency": 0.0, "proc_ms": 0.0, "decode_ms": 0.0,
                                 "detect_ms": 0.0, "wait_ms": 0.0, "last": 0.0}
                          for name in self.cam_cfgs}
        self.stats_lock = threading.Lock()

        self.record_events = bool(self.get_parameter("record_events").value)
        self.ring = RingRecorder(float(ncfg.get("event_buffer_s", 1.5)) + float(ncfg.get("event_post_s", 1.0)))
        self.event_post = float(ncfg.get("event_post_s", 1.0))
        self.event_dir = ncfg.get("event_dir", "data/throws")
        self.record_undodged = bool(ncfg.get("record_undodged", True))
        self.record_gap = float(ncfg.get("min_record_gap_s", 4.0))
        self.min_free = float(ncfg.get("min_free_gb", 1.5)) * 1e9
        self.last_dump = 0.0
        self.last_auto = 0.0               # time of the last dodge or undodged throw that is being saved
        self.disk_warned = False
        if not os.path.isabs(self.event_dir):
            self.event_dir = os.path.join(ROOT, self.event_dir)

        # ---- outputs
        self.cmd_pub = self.create_publisher(Twist, ncfg.get("cmd_topic", "/dodge_cmd_vel"), 10)
        self.state_pub = self.create_publisher(String, "/dodge/state", 10)
        self.event_pub = self.create_publisher(String, "/dodge/event", 10)
        self.pub_lock = threading.Lock()
        self.create_service(SetBool, "/dodge/arm", self.on_arm)

        # ---- inputs
        imu_topic = str(self.get_parameter("imu_topic").value or "") or ncfg.get("imu_topic", "/imu_sensor_broadcaster/imu")
        self.create_subscription(Imu, imu_topic, self.on_imu, 10)
        self.create_subscription(Empty, ncfg.get("estop_topic", "/emergency_stop"), self.on_estop, 10)
        self.create_subscription(Twist, "/cmd_vel", self.on_cmd_vel, 10)
        self.create_subscription(String, "/dodge/mark", self.on_mark, 5)
        self.create_subscription(Float32MultiArray, ncfg.get("free_space_topic", "/free_space"), self.on_free_space, 5)
        self.free_space_time = 0.0
        self.imu_seen = 0.0
        # The IMU can arrive several hundred times a second; the tracker needs the body's attitude at
        # camera rate. Samples in between are let go before they cost anything.
        imu_rate = float(ncfg.get("imu_rate_hz", 100.0) or 0.0)
        self.imu_gap = 0.9 / imu_rate if imu_rate > 0 else 0.0
        self.imu_used = 0.0

        self.walking = set(ncfg.get("walking_controllers", []))
        self.require_controller = bool(ncfg.get("require_active_controller", True))
        self.list_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.list_pending = False
        if self.require_controller:
            self.core.set_can_dodge(False, "controller state not known yet")

        # ---- cameras
        self.running = True
        self.farm = CameraFarm(self.cfg, list(self.cam_cfgs), self.on_frame_result,
                               log=lambda m: self.get_logger().info(str(m)), keep_jpeg=self.record_events,
                               processes=bool(ncfg.get("camera_processes", True)), ros_node=self,
                               opencv_threads=int(ncfg.get("opencv_threads", 1)), on_jpeg=self.on_frame_jpeg)
        self.farm.start()

        self.create_timer(1.0 / float(ncfg.get("cmd_rate_hz", 100.0)), self.on_cmd_timer)
        self.create_timer(0.2, self.on_status_timer)
        self.create_timer(1.0, self.on_controller_timer)
        self.get_logger().warning("dodge node up: cameras %s, %s" % (
            ", ".join(self.cam_cfgs), "ARMED - the robot will move" if self.core.armed else "not armed (dry run)"))

    # ------------------------------------------------------------------ frames
    def on_frame_result(self, res, jpeg) -> None:
        """One picture has been searched (by its camera's process): feed what was found to the tracker.

        Called from one thread per camera. res is a dodge.pipeline.FrameResult.
        """
        name = res.cam
        t_in = time.time()
        started = self.core.process_blobs(name, res.t, res.blobs, t_in, res.size)
        if started:
            self.publish_command()        # do not wait for the timer
        self.farm.set_hint(self.core.track_hint())
        now = time.time()
        core_ms = (now - t_in) * 1000.0
        if self.record_events:
            rec = {"k": "f", "cam": name, "t": round(res.t, 4), "raw": round(res.raw, 4), "done": round(now, 4),
                   "ms": round(res.detect_ms + core_ms, 1), "dec": round(res.decode_ms, 1),
                   "start": round(res.start, 4), "size": [res.size[0], res.size[1]]}
            if res.roi:
                rec["roi"] = 1
            if res.still:
                rec["still"] = res.still
            if res.blobs:
                rec["blobs"] = [[round(b.u, 1), round(b.v, 1), b.w, b.h] for b in res.blobs]
            state = self.core.trace_state()
            if state is not None:
                rec["core"] = state
            self.ring.add_trace(now, rec)
        with self.stats_lock:
            s = self.cam_stats[name]
            s["frames"] += 1
            s["blobs"] += len(res.blobs)
            s["still"] = res.still
            s["latency"] += 0.1 * ((now - res.t) - s["latency"])
            s["proc_ms"] += 0.1 * (res.decode_ms + res.detect_ms + core_ms - s["proc_ms"])
            s["decode_ms"] += 0.1 * (res.decode_ms - s["decode_ms"])
            s["detect_ms"] += 0.1 * (res.detect_ms - s["detect_ms"])
            s["wait_ms"] += 0.1 * ((t_in - res.done) * 1000.0 - s["wait_ms"])     # on its way from the camera's process
            s["last"] = now
        if self.record_events and jpeg is not None:
            self.ring.add_frame(name, res.raw, jpeg)

    def on_frame_jpeg(self, name: str, t_raw: float, jpeg) -> None:
        """The picture a result was about, sent after it by the camera's process: kept for the throw recorder."""
        if self.record_events:
            self.ring.add_frame(name, t_raw, jpeg)

    # ------------------------------------------------------------------ other inputs
    def on_imu(self, msg: Imu) -> None:
        t = stamp_to_sec(msg.header.stamp)
        now = time.time()
        if abs(now - t) > 2.0:
            t = now
        if now - self.imu_used < self.imu_gap:
            self.imu_seen = now
            return
        self.imu_used = now
        q, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
        quat, gyro = (q.w, q.x, q.y, q.z), (w.x, w.y, w.z)
        self.core.set_imu(t, quat, gyro)
        self.imu_seen = now
        if self.record_events:
            self.ring.add_imu(t, quat, gyro, (a.x, a.y, a.z))

    def on_cmd_vel(self, msg: Twist) -> None:
        now = time.time()
        self.core.set_robot_velocity(now, msg.linear.x, msg.linear.y)
        if self.record_events:          # what the mux passed on to the walking controller, and when
            self.ring.add_trace(now, {"k": "m", "t": round(now, 4), "vx": round(msg.linear.x, 3),
                                      "vy": round(msg.linear.y, 3)})

    def on_mark(self, msg: String) -> None:
        """Someone asked for the last few seconds to be kept (the save buttons in Pupper Dodging View)."""
        text, wait = str(msg.data)[:200], 0.0
        if text.startswith("wait="):                 # "wait=3.0;note": save what happens in the next few seconds instead
            head, _, text = text.partition(";")
            try:
                wait = max(0.0, min(self.ring.seconds - 0.2, float(head[5:])))
            except ValueError:
                wait = 0.0
        event = {"t": round(time.time() + wait, 4), "event": "mark", "note": text}
        self.get_logger().warning("marked: saving %s" % ("the next %.1f s" % wait if wait else "the last few seconds"))
        if self.record_events:
            threading.Timer(wait + 0.3, self.dump_mark, args=(event, time.time())).start()

    def dump_mark(self, event: dict, asked: float) -> None:
        if self.last_auto >= asked:
            self.get_logger().info("the throw recorded on request was already saved as a dodge or a seen throw")
            return
        self.dump_event(event)

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
        if self.record_events:
            now = time.time()
            self.ring.add_trace(now, {"k": "c", "t": round(now, 4), "vx": round(float(cmd[0]), 3),
                                      "vy": round(float(cmd[1]), 3)})

    def on_cmd_timer(self) -> None:
        self.publish_command()
        for event in self.core.pop_events():
            text = json.dumps(event, default=str)
            with self.pub_lock:
                self.event_pub.publish(String(data=text))
            self.get_logger().warning("dodge event: " + text)
            if not self.record_events:
                continue
            self.ring.add_trace(event["t"], dict(event, k="e"))
            if event["event"] == "dodge":
                self.last_auto = event["t"]
                threading.Timer(self.event_post, self.dump_event, args=(event,)).start()
            elif event["event"] == "throw_seen" and self.record_undodged:
                if event["t"] - self.last_dump >= self.record_gap:
                    self.last_auto = event["t"]
                    threading.Timer(0.5, self.dump_event, args=(event,)).start()

    def dump_event(self, event: dict) -> None:
        """Save the last few seconds as one throw: frames, IMU, the live trace and the event itself."""
        try:
            os.makedirs(self.event_dir, exist_ok=True)
            free = shutil.disk_usage(self.event_dir).free
            if free < self.min_free:
                if not self.disk_warned:
                    self.disk_warned = True
                    self.get_logger().error("throws are no longer being saved: only %.1f GB free on the disk. Delete old "
                                            "folders in %s." % (free / 1e9, self.event_dir))
                return
            self.disk_warned = False
            kind = {"dodge": "dry" if event.get("dry_run") else "dodge", "throw_seen": "seen"}.get(event["event"], "mark")
            self.last_dump = max(self.last_dump, float(event["t"]))
            name = time.strftime("%Y%m%d-%H%M%S", time.localtime(event["t"])) + "-" + kind
            path = os.path.join(self.event_dir, name)
            n = 1
            while os.path.exists(path):                  # two in the same second
                n += 1
                path = os.path.join(self.event_dir, "%s-%d" % (name, n))
            with self.stats_lock:
                cams = {c: {"fps": round(s.get("fps", 0.0), 1), "latency_ms": round(s["latency"] * 1000.0, 1),
                            "proc_ms": round(s["proc_ms"], 1)} for c, s in self.cam_stats.items()}
            meta = {"kind": kind, "event": event, "armed": self.core.armed, "can_dodge": self.core.can_dodge,
                    "cannot_reason": self.core.cannot_reason, "cameras": cams, "config": self.cfg}
            self.ring.dump(path, meta=meta, events=[event])
            self.get_logger().info("saved %s" % path)
        except Exception as exc:
            self.get_logger().error("could not save the throw recording: %r" % (exc,))

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
                              "proc_ms": round(s["proc_ms"], 1), "decode_ms": round(s["decode_ms"], 1),
                              "detect_ms": round(s["detect_ms"], 1), "wait_ms": round(s["wait_ms"], 1),
                              "alive": (now - s["last"]) < 1.0, "blobs": s["blobs"], "still": s.get("still", 0)}
                extra = self.farm.status(name)
                if "brightness" in extra:
                    cams[name]["brightness"] = extra["brightness"]
                if "cast" in extra:
                    cams[name]["cast"] = extra["cast"]
                if extra.get("fps_in"):
                    cams[name]["fps_in"] = extra["fps_in"]
                if extra.get("problem"):
                    cams[name]["problem"] = extra["problem"]
                s["frames"] = 0
                s["blobs"] = 0
        status["cameras"] = cams
        status["imu_alive"] = (now - self.imu_seen) < 0.5
        with self.pub_lock:
            self.state_pub.publish(String(data=json.dumps(status, default=str)))

    def shutdown(self) -> None:
        self.running = False
        self.farm.stop()          # cameras released before Python exits; exiting mid-capture aborts the process


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
