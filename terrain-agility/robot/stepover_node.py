#!/usr/bin/env python3
"""Step-over node: plans a crossing and plays it on the legs.

    python3 robot/stepover_node.py [--config config/terrain.yaml]

Requests arrive as JSON on /terrain/step_over (std_msgs/String):

    {"cmd": "cross", "height": 0.025, "depth": 0.064, "near_x": 0.36, "skew": 0.0}
    {"cmd": "stand", "hold": 3.0}          rise to the crossing height, stand, lower (no foot leaves the floor)
    {"cmd": "crawl", "distance": 0.10}     slow crawl straight ahead on clear floor
    {"cmd": "stop"}                        abort whatever is running (joints go soft)
    add "dry_run": true to plan and report without moving

and it reports on /terrain/crossing_status (JSON): state = planning | no_plan | refused |
started | progress | done | failed, with "why" and details.

What it does for a motion
  1. checks the robot is standing level under a walking controller, joints near the default pose;
  2. plans (terrain/crawl_planner.py) - nothing moves if there is no plan;
  3. switches the legs from the active walking controller to the three terrain forward
     command controllers (robot/forward_controllers.yaml), streams position / kp / kd at
     120 Hz through terrain/handover.py + terrain/player.py, and switches back when done;
  4. on a fault (tilt, tracking error, blocked foot) softens the joints and lets go;
     on the gamepad e-stop (/emergency_stop) goes limp immediately.

It publishes /terrain/stepover_heartbeat while it holds the legs; robot/estop_guard.py
relaxes the legs if that heartbeat stops (this process died or hung).
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import threading
import time
from dataclasses import replace
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import rclpy  # noqa: E402
from controller_manager_msgs.srv import ListControllers, SwitchController  # noqa: E402
from rclpy.executors import MultiThreadedExecutor  # noqa: E402
from rclpy.node import Node  # noqa: E402
from sensor_msgs.msg import Imu, JointState  # noqa: E402
from std_msgs.msg import Empty, Float64MultiArray, String  # noqa: E402

from terrain import handover as H  # noqa: E402
from terrain.camera_model import level_from_body, tilt_of  # noqa: E402
from terrain.config import TerrainConfig  # noqa: E402
from terrain.crawl_planner import BoxSpec, plan_crawl, plan_crossing, plan_stand_test  # noqa: E402
from terrain.kinematics import DEFAULT_JOINT_POS, JOINT_NAMES, all_feet, default_stance  # noqa: E402
from terrain.player import TrajectoryPlayer  # noqa: E402

READY_JOINT_ERROR = 0.30     # rad: joints must be this close to the default pose to start
READY_JOINT_SPEED = 1.0      # rad/s: and this still
READY_TILT_DEG = 8.0
STALE_AFTER = 0.15           # s without joint states or IMU while holding the legs -> limp
SOFT_REST_TIME = 2.0         # s resting on soft joints after a fault before letting go
LIMP_TIME = 0.4              # s of limp commands before the controllers are released


class StepOverNode(Node):
    now = staticmethod(time.monotonic)      # the clock and sleep, replaceable so tests can run it
    sleep = staticmethod(time.sleep)        # against a simulator in simulated time

    def __init__(self, cfg: TerrainConfig):
        super().__init__("terrain_stepover")
        self.cfg = cfg
        so = cfg.stepover
        self.ctl_pos, self.ctl_kp, self.ctl_kd = so["controllers"]
        self.walk_types = set(so["walking_controller_types"])
        self.rate = float(so["rate_hz"])
        self.estop_kd = float(so["estop_kd"])

        self.pub_pos = self.create_publisher(Float64MultiArray, f"/{self.ctl_pos}/commands", 10)
        self.pub_kp = self.create_publisher(Float64MultiArray, f"/{self.ctl_kp}/commands", 10)
        self.pub_kd = self.create_publisher(Float64MultiArray, f"/{self.ctl_kd}/commands", 10)
        self.pub_status = self.create_publisher(String, so["status_topic"], 10)
        self.pub_beat = self.create_publisher(String, "/terrain/stepover_heartbeat", 10)
        self.switch_client = self.create_client(SwitchController, "/controller_manager/switch_controller")
        self.list_client = self.create_client(ListControllers, "/controller_manager/list_controllers")

        self.q = None                # 12 joint angles in the robot's joint order
        self.dq = np.zeros(12)
        self.q_time = 0.0
        self._index = None
        self.tilt = 0.0
        self.imu_time = 0.0
        self.estop_time = -1e9
        self.stop_requested = False
        self.job = None              # the worker thread of the motion in progress
        self.job_id = 0

        self.create_subscription(JointState, "/joint_states", self._on_joints, 10)
        self.create_subscription(Imu, "/imu_sensor_broadcaster/imu", self._on_imu, 10)
        self.create_subscription(Empty, "/emergency_stop", self._on_estop, 10)
        self.create_subscription(String, so["request_topic"], self._on_request, 10)
        self.get_logger().info(f"step-over node ready; requests on {so['request_topic']}")

    # ------------------------------------------------------------------ inputs
    def _on_joints(self, msg: JointState) -> None:
        if self._index is None:
            try:
                self._index = [list(msg.name).index(n) for n in JOINT_NAMES]
            except ValueError:
                return
        self.q = np.array([msg.position[i] for i in self._index])
        if len(msg.velocity) >= len(msg.name):
            self.dq = np.array([msg.velocity[i] for i in self._index])
        self.q_time = self.now()

    def _on_imu(self, msg: Imu) -> None:
        o = msg.orientation
        if abs(o.x) + abs(o.y) + abs(o.z) + abs(o.w) > 1e-6:
            self.tilt = tilt_of(level_from_body((o.x, o.y, o.z, o.w)))
            self.imu_time = self.now()

    def _on_estop(self, _msg: Empty) -> None:
        self.estop_time = self.now()
        self.get_logger().warning("e-stop received")

    def _on_request(self, msg: String) -> None:
        try:
            req = json.loads(msg.data)
            cmd = req["cmd"]
        except (ValueError, KeyError, TypeError):
            self.status("refused", why="request is not JSON with a 'cmd'")
            return
        if cmd == "stop":
            self.stop_requested = True
            return
        if self.job is not None and self.job.is_alive():
            self.status("refused", why="a motion is already running", request=req.get("id"))
            return
        self.stop_requested = False
        self.job_id += 1
        self.job = threading.Thread(target=self._run_job, args=(req,), daemon=True)
        self.job.start()

    # ------------------------------------------------------------------ helpers
    def status(self, state: str, **fields) -> None:
        msg = String()
        msg.data = json.dumps({"state": state, **fields})
        self.pub_status.publish(msg)
        if state != "progress":
            self.get_logger().info(f"status: {msg.data}")

    def _call(self, client, request, timeout: float = 3.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        future = client.call_async(request)
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        return future.result() if done.wait(timeout) else None

    def controllers(self) -> dict:
        """name -> (type, state), or {} if the controller manager does not answer."""
        res = self._call(self.list_client, ListControllers.Request())
        return {c.name: (c.type, c.state) for c in res.controller} if res is not None else {}

    def switch(self, activate, deactivate, strict=True) -> bool:
        req = SwitchController.Request()
        req.activate_controllers = list(activate)
        req.deactivate_controllers = list(deactivate)
        req.strictness = SwitchController.Request.STRICT if strict else SwitchController.Request.BEST_EFFORT
        res = self._call(self.switch_client, req, timeout=5.0)
        return bool(res is not None and res.ok)

    def send(self, q, kp: float, kd: float) -> None:
        """Position first, gains after: a gain never arrives ahead of the position it belongs to."""
        m = Float64MultiArray()
        m.data = [float(v) for v in q]
        self.pub_pos.publish(m)
        m = Float64MultiArray()
        m.data = [float(kd)] * 12
        self.pub_kd.publish(m)
        m = Float64MultiArray()
        m.data = [float(kp)] * 12
        self.pub_kp.publish(m)

    def beat(self, text: str) -> None:
        m = String()
        m.data = text
        self.pub_beat.publish(m)

    def ready(self) -> str:
        """'' if a motion may start, else why not."""
        now = self.now()
        if self.q is None or now - self.q_time > 0.5:
            return "no joint states (is the robot stack running?)"
        if now - self.imu_time > 0.5:
            return "no IMU data"
        if now - self.estop_time < 3.0:
            return "e-stop was just pressed"
        if math.degrees(self.tilt) > READY_TILT_DEG:
            return f"robot is not level ({math.degrees(self.tilt):.0f} deg)"
        err = float(np.abs(self.q - DEFAULT_JOINT_POS).max())
        if err > READY_JOINT_ERROR:
            return f"legs are not in the standing pose (a joint is {err:.2f} rad off): stand still under the walking controller first"
        if float(np.abs(self.dq).max()) > READY_JOINT_SPEED:
            return "legs are still moving"
        return ""

    # ------------------------------------------------------------------ a motion
    def _plan(self, req):
        crawl = self.cfg.crawl
        cmd = req["cmd"]
        if cmd == "cross":
            # The box was measured from where the body is now. The plan starts from the default
            # stance; easing the legs into it moves the body over the planted feet by however
            # far the feet currently are from their default places, so allow for that.
            shift = 0.0
            if self.q is not None:
                shift = float(np.mean(all_feet(self.q)[:, 0] - default_stance()[1][:, 0]))
                shift = max(-0.03, min(0.03, shift))
            box = BoxSpec(height=float(req["height"]), depth=float(req["depth"]), near_x=float(req["near_x"]) - shift,
                          skew=float(req.get("skew", 0.0)))
            if abs(shift) > 0.003:
                self.get_logger().info(f"feet are {shift * 1000:+.0f} mm from their default places: box distance corrected")
            cap = self.cfg.decision.height_cap + self.cfg.decision.height_guard
            if box.height > cap + 1e-9:
                return None, f"box height {box.height * 1000:.0f} mm is over the {cap * 1000:.0f} mm limit"
            plan = plan_crossing(box, crawl, search=self.cfg.allow_search)
        elif cmd == "stand":
            plan = plan_stand_test(crawl, hold=float(req.get("hold", 3.0)))
        elif cmd == "crawl":
            plan = plan_crawl(float(req.get("distance", 0.08)), crawl)
        else:
            return None, f"unknown cmd {cmd!r}"
        return (plan, "") if plan.ok else (None, plan.reason)

    def _run_job(self, req) -> None:
        rid = req.get("id", self.job_id)
        try:
            self._job(req, rid)
        except Exception as e:  # never leave the legs held by a crashed job
            self.get_logger().error(f"step-over job crashed: {e!r}")
            self._release(limp=True)
            self.status("failed", why=f"internal error: {e}", request=rid)

    def _job(self, req, rid) -> None:
        dry = bool(req.get("dry_run", False))
        why = "" if dry else self.ready()
        if why:
            self.status("refused", why=why, request=rid)
            return
        states = self.controllers()
        if not dry:
            if not states:
                self.status("refused", why="controller_manager does not answer", request=rid)
                return
            missing = [c for c in (self.ctl_pos, self.ctl_kp, self.ctl_kd) if c not in states]
            if missing:
                self.status("refused", why=f"controllers not loaded: {missing} (run robot/spawn_controllers.py)", request=rid)
                return
            walking = [n for n, (typ, st) in states.items() if typ in self.walk_types and st == "active"]
            if len(walking) != 1:
                self.status("refused", why=f"need exactly one active walking controller, found {walking}", request=rid)
                return

        self.status("planning", request=rid)
        t0 = self.now()
        plan, why = self._plan(req)
        if plan is None:
            self.status("no_plan", why=why, request=rid)
            return
        summary = plan.summary().splitlines()[0]
        self.get_logger().info(plan.summary())
        if dry:
            self.status("done", request=rid, dry_run=True, plan=summary, duration=round(plan.duration, 1),
                        planning_s=round(self.now() - t0, 2))
            return
        why = self.ready()          # planning took a moment: the robot must still be standing
        if why or self.stop_requested:
            self.status("refused", why=why or "stopped", request=rid)
            return

        player_cfg = replace(self.cfg.player, kp=self.cfg.crawl.kp, kd=self.cfg.crawl.kd)
        hand = H.Handover(TrajectoryPlayer(plan, player_cfg), H.HandoverConfig(estop_kd=self.estop_kd))
        terrain = [self.ctl_pos, self.ctl_kp, self.ctl_kd]
        switched = threading.Event()
        switch_failed = threading.Event()

        def do_switch():
            (switched if self.switch(terrain, walking) else switch_failed).set()

        period = 1.0 / self.rate
        t_start = self.now()
        next_tick = t_start
        last_progress = 0.0
        last_stage = ""
        t_final = None
        back_thread, back_ok, back_failed = None, threading.Event(), threading.Event()
        threading.Thread(target=do_switch, daemon=True).start()
        started = False
        while True:
            now = self.now()
            estop = (now - self.estop_time) < 1.0 or switch_failed.is_set()
            stale = (now - self.q_time) > STALE_AFTER or (now - self.imu_time) > STALE_AFTER
            if self.stop_requested and hand.stage in (H.PLAY, H.FINAL):
                hand.player.stop(now, "stopped by request")
            cmd = hand.update(now, self.q, self.tilt, switched.is_set(), estop=estop, stale=stale)
            self.send(cmd.q, cmd.kp, cmd.kd)
            self.beat("holding" if hand.stage != H.LIMP else "released")
            if cmd.event:
                self.get_logger().info(f"[{now - t_start:6.2f} s] {cmd.event}")
            if cmd.stage != last_stage:
                last_stage = cmd.stage
                t_stage = now
            if not started and hand.stage in (H.RAMP, H.PLAY):
                started = True                  # the legs are ours: only now is the crossing under way
                self.status("started", request=rid, plan=summary, duration=round(plan.duration, 1),
                            walking_controller=walking[0])
            if now - last_progress > 0.5:
                last_progress = now
                self.status("progress", request=rid, stage=cmd.stage, phase=cmd.phase, progress=round(cmd.progress, 3))

            if hand.stage == H.FINAL:
                # Plan finished, standing in the default pose: give the legs back. The switch
                # runs in its own thread so the legs stay held (and the heartbeat keeps
                # going) until the walking controller really has them.
                if t_final is None:
                    t_final = now
                if back_thread is None and now - t_final > 0.3:
                    back_thread = threading.Thread(target=lambda: (back_ok if self.switch(walking, terrain) else back_failed).set(),
                                                   daemon=True)
                    back_thread.start()
                if back_ok.is_set():
                    self.beat("released")
                    self.status("done", request=rid, seconds=round(now - t_start, 1),
                                max_tilt_deg=round(hand.player.max_tilt_deg, 1))
                    return
                if back_failed.is_set():
                    self._release(limp=True)
                    self.status("failed", request=rid, why="could not switch back to the walking controller; legs relaxed")
                    return
            elif hand.stage == H.SOFT and hand.player.finished and now - t_stage > SOFT_REST_TIME:
                self._release(limp=True)
                self.status("failed", request=rid, why=hand.result, events=[e for _, e in hand.player.events][-6:])
                return
            elif hand.stage == H.LIMP and now - t_stage > LIMP_TIME:
                self._release(limp=False)
                if not started:
                    # the walking controller never let go: nothing happened to the robot
                    self.status("refused", request=rid, why="controller switch failed" if switch_failed.is_set() else hand.result)
                else:
                    self.status("failed", request=rid, why=hand.result)
                return

            next_tick += period
            delay = next_tick - self.now()
            if delay > 0:
                self.sleep(delay)
            else:
                next_tick = self.now()     # fell behind: do not try to catch up

    def _release(self, limp: bool) -> None:
        """Let go of the legs: (optionally) command limp joints first, then deactivate the
        terrain controllers. The walking controller is NOT re-activated: after a fault a
        person decides when the robot stands up again (joystick, or the Gemini app)."""
        if limp and self.q is not None:
            t_end = self.now() + LIMP_TIME
            while self.now() < t_end:
                self.send(self.q, 0.0, self.estop_kd)
                self.sleep(1.0 / self.rate)
        self.switch([], [self.ctl_pos, self.ctl_kp, self.ctl_kd], strict=False)
        self.beat("released")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None, help="path of terrain.yaml")
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = StepOverNode(TerrainConfig(args.config))
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.job is not None and node.job.is_alive():
            node.stop_requested = True
            node.job.join(timeout=4.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
