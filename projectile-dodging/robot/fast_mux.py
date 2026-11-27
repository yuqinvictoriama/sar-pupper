#!/usr/bin/env python3
"""Replacement for the monorepo's cmd_vel_mux (plan decision D2). The default since 2026-11-26.

Same rules as the original: inputs in priority order, the joystick counts only
outside its deadband, a source that goes quiet for timeout_ms is dropped, the
highest-priority live source drives /cmd_vel. The one difference: a message
from the active source is forwarded the moment it arrives instead of on the
next 50 ms tick, which takes up to 50 ms off the dodge reaction time.

Measured on the Pupper (2026-11-26, two armed throws): the monorepo mux passed
the dodge command on 40 and 43 ms after it was sent, a third to a half of the
whole delay between deciding and the body moving. To go back to the monorepo mux:

    ros2 launch robot/sar_dodge.launch.py fast_mux:=false      (robot/start.sh --slow-mux)
"""
from __future__ import annotations

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

JOYSTICK_TOPICS = ("/teleop_cmd_vel", "/joystick_cmd_vel")


class FastMux(Node):
    def __init__(self):
        super().__init__("cmd_vel_mux")
        self.declare_parameter("deadband", 0.05)
        self.declare_parameter("timeout_ms", 500)
        self.declare_parameter("inputs", ["/teleop_cmd_vel", "/dodge_cmd_vel", "/llm_cmd_vel",
                                          "/person_following_cmd_vel"])
        self.deadband = float(self.get_parameter("deadband").value)
        self.timeout = float(self.get_parameter("timeout_ms").value) / 1000.0
        self.inputs = list(self.get_parameter("inputs").value)
        self.last_activity = {t: 0.0 for t in self.inputs}
        self.latest = {t: Twist() for t in self.inputs}
        self.received = {t: False for t in self.inputs}
        self.active = "none"
        self.pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.status = self.create_publisher(String, "/cmd_vel_mux/active_source", 10)
        for topic in self.inputs:
            self.create_subscription(Twist, topic, lambda msg, t=topic: self.on_cmd(t, msg), 10)
        self.create_timer(0.05, self.on_timer)
        self.get_logger().info("fast mux inputs (highest priority first): %s" % ", ".join(self.inputs))

    def best(self, now: float) -> str:
        for topic in self.inputs:                 # list order is priority order
            if self.received[topic] and now - self.last_activity[topic] <= self.timeout:
                return topic
        return "none"

    def on_cmd(self, topic: str, msg: Twist) -> None:
        now = time.monotonic()
        self.latest[topic] = msg
        self.received[topic] = True
        if topic in JOYSTICK_TOPICS:
            if max(abs(msg.linear.x), abs(msg.linear.y), abs(msg.angular.z)) > self.deadband:
                self.last_activity[topic] = now
        else:
            self.last_activity[topic] = now
        if self.best(now) == topic:
            self.pub.publish(msg)                 # forward at once
            self.set_active(topic)

    def on_timer(self) -> None:
        now = time.monotonic()
        best = self.best(now)
        if best != "none":
            self.pub.publish(self.latest[best])   # keep-alive at 20 Hz, as the original does
        elif self.active != "none":
            self.pub.publish(Twist())             # everyone went quiet: stop once
        self.set_active(best)

    def set_active(self, topic: str) -> None:
        if topic != self.active:
            self.active = topic
            self.status.publish(String(data=topic))
            self.get_logger().info("active source: %s" % topic)


def main() -> None:
    rclpy.init()
    node = FastMux()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
