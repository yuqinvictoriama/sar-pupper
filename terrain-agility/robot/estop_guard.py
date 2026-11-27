#!/usr/bin/env python3
"""E-stop guard for the terrain forward controllers. Run it as its own process.

    python3 robot/estop_guard.py [--config config/terrain.yaml]

The gamepad e-stop (right-stick press) relaxes only the controllers on the joystick node's
own list and publishes /emergency_stop. The step-over controllers are not on that list, and
unlike the walking controllers they have no tilt cut-out of their own: they hold whatever
they were last sent. So this small independent process makes the legs limp (kp = 0, light
damping) and deactivates the three terrain controllers when

  * /emergency_stop is published, or
  * the terrain controllers are active but robot/stepover_node.py has stopped sending its
    heartbeat (it crashed or hung while holding the legs).

It does not re-activate anything: after an e-stop a person stands the robot up again.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

import rclpy  # noqa: E402
from controller_manager_msgs.srv import ListControllers, SwitchController  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Empty, Float64MultiArray, String  # noqa: E402

from terrain.config import TerrainConfig  # noqa: E402

HEARTBEAT_TIMEOUT = 0.35     # s
POLL_PERIOD = 0.5            # s between checks of which controllers are active
RELAX_TIME = 1.0             # s of limp commands around a release


class EstopGuard(Node):
    def __init__(self, cfg: TerrainConfig):
        super().__init__("terrain_estop_guard")
        so = cfg.stepover
        self.controllers = list(so["controllers"])
        self.estop_kd = float(so["estop_kd"])
        _, kp_ctl, kd_ctl = self.controllers
        self.pub_kp = self.create_publisher(Float64MultiArray, f"/{kp_ctl}/commands", 10)
        self.pub_kd = self.create_publisher(Float64MultiArray, f"/{kd_ctl}/commands", 10)
        self.switch_client = self.create_client(SwitchController, "/controller_manager/switch_controller")
        self.list_client = self.create_client(ListControllers, "/controller_manager/list_controllers")
        self.beat_time = 0.0
        self.beat_text = "released"
        self.active = False          # terrain controllers active at the last poll
        self.active_since = 0.0
        self.relax_until = 0.0
        self.pending = None
        self.create_subscription(Empty, "/emergency_stop", self._on_estop, 10)
        self.create_subscription(String, "/terrain/stepover_heartbeat", self._on_beat, 10)
        self.create_timer(0.02, self._tick)
        self.create_timer(POLL_PERIOD, self._poll)
        self.get_logger().info("e-stop guard watching " + ", ".join(self.controllers))

    def _on_beat(self, msg: String) -> None:
        self.beat_time = time.monotonic()
        self.beat_text = msg.data

    def _on_estop(self, _msg: Empty) -> None:
        self.release("e-stop")

    def _poll(self) -> None:
        if self.pending is not None and not self.pending.done():
            return
        if not self.list_client.service_is_ready():
            return
        self.pending = self.list_client.call_async(ListControllers.Request())
        self.pending.add_done_callback(self._on_list)

    def _on_list(self, future) -> None:
        res = future.result()
        if res is None:
            return
        active = any(c.name in self.controllers and c.state == "active" for c in res.controller)
        if active and not self.active:
            self.active_since = time.monotonic()
        self.active = active

    def release(self, why: str) -> None:
        self.get_logger().warning(f"relaxing the terrain controllers: {why}")
        self.relax_until = time.monotonic() + RELAX_TIME
        self._limp()
        req = SwitchController.Request()
        req.deactivate_controllers = self.controllers
        req.strictness = SwitchController.Request.BEST_EFFORT
        if self.switch_client.service_is_ready():
            self.switch_client.call_async(req)
        self.active = False

    def _limp(self) -> None:
        m = Float64MultiArray()
        m.data = [0.0] * 12
        self.pub_kp.publish(m)
        m = Float64MultiArray()
        m.data = [self.estop_kd] * 12
        self.pub_kd.publish(m)

    def _tick(self) -> None:
        now = time.monotonic()
        if now < self.relax_until:
            self._limp()
            return
        holding = self.beat_text == "holding" and now - self.beat_time < 5.0
        silent = now - self.beat_time > HEARTBEAT_TIMEOUT
        if holding and silent:
            self.beat_text = "lost"
            self.release("the step-over node stopped sending its heartbeat")
        elif self.active and silent and now - self.active_since > 1.0:
            self.release("terrain controllers active with no step-over node driving them")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    args, ros_args = ap.parse_known_args()
    rclpy.init(args=ros_args)
    node = EstopGuard(TerrainConfig(args.config))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
