#!/usr/bin/env python3
"""Terrain agility node: sees obstacles in the front fisheye, decides step over / go around,
and steers. The thinking is in terrain/ (pure Python, tested off the robot); this file is
the ROS plumbing around it.

    python3 robot/terrain_node.py [--config config/terrain.yaml]

In
  /camera/image_raw/compressed     the front fisheye (the course camera node)
  /imu_sensor_broadcaster/imu      body attitude (to level the picture) and heading
  the "intent" topics              whoever is driving: /llm_cmd_vel, /survivor_cmd_vel, ... (terrain.yaml: fsm.intent_topics)
  /terrain/crossing_status         answers from robot/stepover_node.py
  /dodge/event, /dodge/state       optional, from the projectile-dodging node
  /terrain/enable (Bool), /terrain/reset (Empty)
  /terrain/mode (String)           full | avoid | report | off   (the demo's path navigation sets this by zone)
                                     full    step over the box, walk around everything else
                                     avoid   walk around everything, the box included
                                     report  publish obstacles and the scan, never steer
                                     off     the same as /terrain/enable false

Out
  /terrain_cmd_vel                 Twist into cmd_vel_mux - ONLY while it is dealing with an obstacle;
                                   silent otherwise, so the mux gives control back to the driver
  /terrain/step_over               crossing requests to robot/stepover_node.py
  /terrain/state, /terrain/event   JSON / text, for the mission layer, Gemini and logs. The state carries the
                                   mode and, from the moment the box has been measured, the measurement
                                   ("box": near_x, height, depth, skew, width; metres and radians)
  /terrain/obstacles               JSON list of the obstacles in view
  /terrain/scan                    JSON: range to the nearest obstacle at each bearing (free space)
  /terrain/debug/compressed        the classified polar view (only rendered while someone subscribes)
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import CompressedImage, Imu  # noqa: E402
from std_msgs.msg import Bool, Empty, String  # noqa: E402

from terrain import floor_model as FM  # noqa: E402
from terrain import state_machine as SM  # noqa: E402
from terrain.camera_model import PolarView, level_from_body, tilt_of  # noqa: E402
from terrain.config import TerrainConfig  # noqa: E402
from terrain.gauge import fine_view, measure_box  # noqa: E402
from terrain.scan import build_scan, find_obstacles, scan_points  # noqa: E402

TICK_HZ = 20.0
INTENT_FRESH = 0.6           # s: an intent message older than this no longer counts
PLAN_TIMEOUT = 180.0         # s to wait for the step-over node to start a crossing
ADAPT_REGION = (40.0, 12.0)  # floor patch used to keep the floor colour up to date: depression from 40 deg
#                              down to the robot's chin, bearings within +-12 deg
ADAPT_MIN_FLOOR = 0.92       # only adapt when the patch already looks like floor
ADAPT_RATE = 0.03

def label_colours():
    table = np.zeros((256, 3), dtype=np.uint8)
    table[FM.FLOOR] = (70, 70, 70)
    table[FM.OBSTACLE] = (0, 0, 255)
    table[FM.BOX_SIDE] = (255, 120, 0)
    table[FM.BOX_TOP] = (255, 255, 255)
    table[FM.BALL] = (0, 255, 0)
    table[FM.INVALID] = (0, 0, 0)
    return table


class Perception(threading.Thread):
    """Turns camera frames into a scan (always) and a box measurement (when asked)."""

    def __init__(self, cfg: TerrainConfig, log, clock=time.monotonic):
        super().__init__(daemon=True)
        self.cfg, self.log, self.clock = cfg, log, clock
        half = cfg.model.scaled(cfg.model.width // 2, cfg.model.height // 2)
        self.scan_view = PolarView(half, cfg.pose)
        self.gauge_view = fine_view(cfg.model, cfg.pose)
        self.floor = cfg.load_floor()
        self.lock = threading.Lock()
        self.new_frame = threading.Event()
        self._frame = None            # (jpeg bytes, R_level_body, arrival time)
        self.want_gauge = False
        self.want_debug = False
        self.scan_result = None       # (id, time, scan, obstacles)
        self.box_result = None        # (id, time, BoxMeasurement | None)
        self.debug_jpeg = None
        self.frame_age = float("inf")
        self.scan_ms = 0.0
        self._count = 0
        self._last_scan = 0.0
        self._colours = label_colours()
        self.running = True
        if not self.floor.trained:
            log.warning("no floor colour model yet: learning it from the floor just ahead of the robot. "
                        "Start on clear floor, or run tools/learn_colors.py first.")

    def push(self, data: bytes, R) -> None:
        with self.lock:
            self._frame = (data, R, self.clock())
        self.new_frame.set()

    def run(self) -> None:
        while self.running:
            if not self.new_frame.wait(0.5):
                continue
            self.new_frame.clear()
            with self.lock:
                frame, self._frame = self._frame, None
            if frame is None:
                continue
            data, R, t_arr = frame
            try:
                if self.want_gauge:
                    self._gauge(data, R, t_arr)
                elif t_arr - self._last_scan >= 1.0 / self.cfg.scan_rate - 0.01:
                    self._last_scan = t_arr
                    self._scan(data, R, t_arr)
            except Exception as e:      # a bad frame must not kill the thread
                self.log.error(f"perception: {e!r}")

    def _scan(self, data, R, t_arr) -> None:
        t0 = time.monotonic()
        bgr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_REDUCED_COLOR_2)
        if bgr is None:
            return
        view = self.scan_view
        if bgr.shape[1] != view.model.width or bgr.shape[0] != view.model.height:
            # the camera is not running at the size the lens model was given for
            view = self.scan_view = PolarView(self.cfg.model.scaled(bgr.shape[1], bgr.shape[0]), self.cfg.pose)
        pol, valid = view.sample(bgr, R)
        hsv = cv2.cvtColor(pol, cv2.COLOR_BGR2HSV)
        rows = (view.depressions > ADAPT_REGION[0]) & (view.depressions < self.cfg.near_depression_deg - 2.0)
        cols = np.abs(view.bearings) < ADAPT_REGION[1]
        patch = hsv[np.ix_(rows, cols)]
        if not self.floor.trained:
            self.floor.learn(patch)
        labels = FM.classify(hsv, valid, self.floor, self.cfg.box_side, self.cfg.box_top, self.cfg.ball)
        if self.cfg.adapt_floor:
            if float(np.mean(labels[np.ix_(rows, cols)] == FM.FLOOR)) > ADAPT_MIN_FLOOR:
                self.floor.learn(patch, rate=ADAPT_RATE)
        scan = build_scan(labels, view, self.cfg.scan)
        obstacles = find_obstacles(scan, self.cfg.scan)
        self._count += 1
        self.scan_result = (self._count, t_arr, scan, obstacles)
        self.scan_ms = (time.monotonic() - t0) * 1000.0
        if self.want_debug:
            img = self._colours[labels]
            img = cv2.resize(img, None, fx=3, fy=3, interpolation=cv2.INTER_NEAREST)
            for ob in obstacles:
                j0 = int(np.argmin(np.abs(view.bearings - ob.right_deg)))
                j1 = int(np.argmin(np.abs(view.bearings - ob.left_deg)))
                tag = ("BOX" if ob.is_box else ("TALL" if ob.tall else "LOW")) + f" {ob.nearest:.2f}m"
                cv2.putText(img, tag, (3 * min(j0, j1), 14), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
            # bearings increase to the left; show left on the left
            ok, jpg = cv2.imencode(".jpg", img[:, ::-1], [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                self.debug_jpeg = jpg.tobytes()

    def _gauge(self, data, R, t_arr) -> None:
        bgr = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            return
        if bgr.shape[1] != self.gauge_view.model.width:
            self.gauge_view = fine_view(self.cfg.model.scaled(bgr.shape[1], bgr.shape[0]), self.cfg.pose)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        m = measure_box(rgb, self.gauge_view, self.cfg.box_side, self.cfg.box_top, R_level_body=R,
                        floor=self.floor if self.floor.trained else None, landing=self.cfg.decision.landing_clear)
        self._count += 1
        self.box_result = (self._count, t_arr, m)


class TerrainNode(Node):
    now = staticmethod(time.monotonic)      # replaceable clock (tests)

    def __init__(self, cfg: TerrainConfig):
        super().__init__("terrain_agility")
        self.cfg = cfg
        self.fsm = SM.TerrainFsm(cfg.fsm)
        self.enabled = bool(cfg.raw.get("fsm", {}).get("enabled", True))
        self.fsm.set_mode(str(cfg.raw.get("fsm", {}).get("mode", SM.FULL)))
        self.perception = Perception(cfg, self.get_logger(), self.now)
        self.perception.start()

        self.R = np.eye(3)
        self.yaw = 0.0
        self.tilt = 0.0
        self.imu_time = 0.0
        self._imu_t_last = None
        self.intents = {}            # topic -> (time, (vx, vy, wz))
        self.dodge_event = False
        self.ball_time = -1e9
        self.cross_id = 0
        self.cross_sent = None
        self.last_scan_id = 0
        self.last_box_id = 0
        self.last_state = ""
        self.last_state_pub = 0.0
        self.last_debug = 0.0
        self.last_warn = 0.0

        self.pub_cmd = self.create_publisher(Twist, cfg.cmd_topic, 10)
        self.pub_req = self.create_publisher(String, cfg.stepover["request_topic"], 10)
        self.pub_state = self.create_publisher(String, "/terrain/state", 10)
        self.pub_event = self.create_publisher(String, "/terrain/event", 10)
        self.pub_scan = self.create_publisher(String, "/terrain/scan", 10)
        self.pub_obstacles = self.create_publisher(String, "/terrain/obstacles", 10)
        self.pub_debug = self.create_publisher(CompressedImage, "/terrain/debug/compressed", 1)

        self.create_subscription(CompressedImage, cfg.camera_topic, self._on_image, 1)
        self.create_subscription(Imu, "/imu_sensor_broadcaster/imu", self._on_imu, 10)
        for topic in cfg.intent_topics:
            self.create_subscription(Twist, topic, lambda m, topic=topic: self._on_intent(topic, m), 10)
        self.create_subscription(String, cfg.stepover["status_topic"], self._on_crossing_status, 10)
        self.create_subscription(String, "/dodge/event", self._on_dodge_event, 10)
        self.create_subscription(String, "/dodge/state", self._on_dodge_state, 10)
        self.create_subscription(Bool, "/terrain/enable", self._on_enable, 10)
        self.create_subscription(String, "/terrain/mode", self._on_mode, 10)
        self.create_subscription(Empty, "/terrain/reset", self._on_reset, 10)
        self.create_timer(1.0 / TICK_HZ, self._tick)
        self.get_logger().info(
            f"terrain agility ready ({'enabled' if self.enabled else 'DISABLED'}); intent from {cfg.intent_topics}, "
            f"commands on {cfg.cmd_topic}; step-over limit {cfg.decision.height_cap * 1000:.0f} mm")

    # ------------------------------------------------------------------ inputs
    def _on_image(self, msg: CompressedImage) -> None:
        self.perception.push(bytes(msg.data), self.R)

    def _on_imu(self, msg: Imu) -> None:
        now = self.now()
        o = msg.orientation
        if abs(o.x) + abs(o.y) + abs(o.z) + abs(o.w) > 1e-6:
            q = (o.x, o.y, o.z, o.w)
            self.R = level_from_body(q)
            self.tilt = tilt_of(self.R)
            self.yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
        elif self._imu_t_last is not None:
            # no orientation estimate: integrate the yaw rate (drifts slowly, fine over one detour)
            self.yaw += msg.angular_velocity.z * min(0.1, now - self._imu_t_last)
        self._imu_t_last = now
        self.imu_time = now

    def _on_intent(self, topic: str, msg: Twist) -> None:
        self.intents[topic] = (self.now(), (msg.linear.x, msg.linear.y, msg.angular.z))

    def _on_dodge_event(self, _msg: String) -> None:
        self.dodge_event = True

    def _on_dodge_state(self, msg: String) -> None:
        text = msg.data.upper()
        if any(k in text for k in ("TRACK", "THREAT", "DODGE")):
            self.ball_time = self.now()

    def _new_fsm(self) -> None:
        mode = self.fsm.mode
        self.fsm = SM.TerrainFsm(self.cfg.fsm)
        self.fsm.set_mode(mode)

    def _on_enable(self, msg: Bool) -> None:
        if msg.data != self.enabled:
            self.enabled = bool(msg.data)
            self._new_fsm()
            self.event(f"terrain agility {'enabled' if self.enabled else 'disabled'}")

    def _on_mode(self, msg: String) -> None:
        mode = msg.data.strip().lower()
        if mode == "off":
            self._on_enable(Bool(data=False))
            return
        if mode not in SM.MODES:
            self.get_logger().warning(f"/terrain/mode: '{msg.data}' is not one of full, avoid, report, off")
            return
        if not self.enabled:
            self._on_enable(Bool(data=True))
        if mode != self.fsm.mode:
            self.fsm.set_mode(mode)
            self.event(f"mode {mode}")

    def _on_reset(self, _msg: Empty) -> None:
        self._new_fsm()
        self.cross_sent = None
        self.event("reset")

    def _on_crossing_status(self, msg: String) -> None:
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        if st.get("request") != f"terrain-{self.cross_id}":
            return
        state, why = st.get("state"), st.get("why", "")
        if state == "started":
            self.fsm.plan_result(True)
            self.event(f"stepping over: {st.get('plan', '')}")
        elif state in ("no_plan", "refused"):
            self.cross_sent = None
            self.fsm.plan_result(False, why)
            self.event(f"not stepping over ({why}); going around")
        elif state == "done":
            self.cross_sent = None
            self.fsm.crossing_result(True)
            self.event("crossing finished")
        elif state == "failed":
            self.cross_sent = None
            if self.fsm.state == SM.PLAN:
                self.fsm.plan_result(False, why)
            else:
                self.fsm.crossing_result(False, why)
            self.event(f"crossing failed: {why}")

    # ------------------------------------------------------------------ outputs
    def event(self, text: str) -> None:
        m = String()
        m.data = text
        self.pub_event.publish(m)
        self.get_logger().info(text)

    def _intent(self, now):
        for topic in self.cfg.intent_topics:        # in the mux's priority order
            hit = self.intents.get(topic)
            if hit is not None and now - hit[0] < INTENT_FRESH:
                return hit[1]
        return None

    def _tick(self) -> None:
        now = self.now()
        p = self.perception
        imu_ok = now - self.imu_time < 0.5
        if not self.enabled:
            self._publish_state(now, "DISABLED")
            return

        inp = SM.Inputs(t=now, yaw=self.yaw, level=imu_ok and math.degrees(self.tilt) < self.cfg.level_tilt_deg,
                        intent=self._intent(now), ball_tracked=(now - self.ball_time) < 1.0,
                        dodge_event=self.dodge_event)
        self.dodge_event = False
        res = p.scan_result
        scan_fresh = res is not None and now - res[1] < 1.0
        if res is not None and res[0] != self.last_scan_id:
            self.last_scan_id = res[0]
            _, _, scan, obstacles = res
            inp.points = scan_points(scan)
            seen = np.radians(scan.bearings_deg[scan.seen]) if scan.seen.any() else np.array([0.0])
            inp.seen_sector = (float(seen.min()), float(seen.max()))
            inp.scan_range = self.cfg.scan.max_range
            self._publish_scan(scan, obstacles)
        if scan_fresh:
            inp.obstacles = res[3]
        box = p.box_result
        if box is not None and box[0] != self.last_box_id and now - box[1] < 0.5:
            self.last_box_id = box[0]
            inp.box = box[2]

        if (not scan_fresh and self.fsm.mode != SM.REPORT_ONLY
                and self.fsm.state not in (SM.CRUISE, SM.STEP_OVER, SM.PLAN, SM.GAUGE, SM.ALIGN, SM.HOLD)):
            # blind while steering round something: stop rather than walk on old data
            if now - self.last_warn > 2.0:
                self.last_warn = now
                self.get_logger().warning("no fresh camera scan: stopping until frames come back")
            self._send((0.0, 0.0, 0.0))
            self._publish_state(now, self.fsm.state, "no camera frames")
            return

        out = self.fsm.tick(inp)
        p.want_gauge = out.want_gauge
        if out.event:
            self.event(out.event)
        if out.plan_box is not None:
            self.cross_id += 1
            self.cross_sent = now
            b = out.plan_box
            m = String()
            m.data = json.dumps({"cmd": "cross", "id": f"terrain-{self.cross_id}", "height": round(b.height, 4),
                                 "depth": round(b.depth, 4), "near_x": round(b.near_x, 4), "skew": round(b.skew, 4)})
            self.pub_req.publish(m)
        if self.fsm.state == SM.PLAN and self.cross_sent is not None and now - self.cross_sent > PLAN_TIMEOUT:
            self.cross_sent = None
            self.fsm.plan_result(False, "the step-over node did not answer")
            self.event("no answer from the step-over node; going around")

        crossing_failed = self.fsm.state == SM.HOLD and self.fsm.hold_reason.startswith("crossing failed")
        if out.cmd is not None and not crossing_failed:
            self._send(out.cmd)
        self._publish_state(now, out.state, self.fsm.hold_reason if self.fsm.state == SM.HOLD else "")

        if self.pub_debug.get_subscription_count() > 0:
            p.want_debug = True
            if p.debug_jpeg is not None and now - self.last_debug > 0.4:
                self.last_debug = now
                m = CompressedImage()
                m.header.stamp = self.get_clock().now().to_msg()
                m.format = "jpeg"
                m.data = p.debug_jpeg
                self.pub_debug.publish(m)
        else:
            p.want_debug = False

    def _send(self, cmd) -> None:
        t = Twist()
        t.linear.x, t.linear.y, t.angular.z = float(cmd[0]), float(cmd[1]), float(cmd[2])
        self.pub_cmd.publish(t)

    def _publish_state(self, now, state, reason="") -> None:
        if state != self.last_state or now - self.last_state_pub > 1.0:
            self.last_state, self.last_state_pub = state, now
            m = String()
            status = {"state": state, "reason": reason, "scan_ms": round(self.perception.scan_ms, 1),
                      "mode": self.fsm.mode if self.enabled else "off"}
            e = self.fsm.estimate
            if e is not None and state in (SM.ALIGN, SM.PLAN, SM.STEP_OVER):
                # the box as measured, for whoever keeps track of where the robot is (it is a landmark)
                num = lambda v: round(float(v), 4) if math.isfinite(v) else None      # noqa: E731
                status["box"] = {"near_x": num(e.near_x), "height": num(e.height), "depth": num(e.depth),
                                 "skew": num(e.skew), "width": num(e.width)}
            m.data = json.dumps(status)
            self.pub_state.publish(m)

    def _publish_scan(self, scan, obstacles) -> None:
        """/terrain/obstacles: what is in view. /terrain/scan: range to the nearest obstacle at
        each bearing (m from the body centre's frame origin is not implied: ranges are from the
        lens; null = clear), which is the free space the dodge node and the planner can use."""
        m = String()
        m.data = json.dumps([{"nearest": round(o.nearest, 3), "bearing_deg": round(o.nearest_bearing_deg, 1),
                              "left_deg": round(o.left_deg, 1), "right_deg": round(o.right_deg, 1),
                              "left_xy": [round(v, 3) for v in o.left_xy], "right_xy": [round(v, 3) for v in o.right_xy],
                              "width": round(o.width, 3), "height": None if not math.isfinite(o.height) else round(o.height, 3),
                              "tall": o.tall, "is_box": o.is_box} for o in obstacles])
        self.pub_obstacles.publish(m)
        if self.pub_scan.get_subscription_count() > 0:
            step = 5                                  # every 5 degrees is plenty for free-space queries
            b = scan.bearings_deg[::step]
            r = [float(np.nanmin(np.where(np.isfinite(scan.ranges[i:i + step]), scan.ranges[i:i + step], np.nan)))
                 if np.isfinite(scan.ranges[i:i + step]).any() else None for i in range(0, len(scan.ranges), step)]
            seen = [bool(scan.seen[i:i + step].any()) for i in range(0, len(scan.ranges), step)]
            m = String()
            m.data = json.dumps({"bearings_deg": [round(float(v), 1) for v in b],
                                 "ranges": [None if v is None else round(v, 3) for v in r], "seen": seen,
                                 "lens_x_forward": self.cfg.pose.x_forward})
            self.pub_scan.publish(m)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None, help="path of terrain.yaml")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = TerrainNode(TerrainConfig(args.config))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.perception.running = False
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
