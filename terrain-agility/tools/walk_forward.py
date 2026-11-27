#!/usr/bin/env python3
"""A stand-in driver for testing terrain agility without Gemini or a planner: walks the
robot straight ahead for a while on /terrain_test_cmd_vel, the lowest-priority mux input.
Terrain agility treats it like any other driver: it steers round or steps over what is in
the way and hands control back.

    python3 tools/walk_forward.py                 # 0.3 m/s for 12 s
    python3 tools/walk_forward.py --speed 0.25 --seconds 20

The joystick overrides it at any time; Ctrl-C stops it. Someone must be at the e-stop.
"""
import argparse
import time

import rclpy
from geometry_msgs.msg import Twist


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--speed", type=float, default=0.30, help="m/s forward")
    ap.add_argument("--seconds", type=float, default=12.0)
    ap.add_argument("--topic", default="/terrain_test_cmd_vel")
    ap.add_argument("--yes", action="store_true", help="skip the e-stop question")
    args = ap.parse_args()
    if not 0.0 < args.speed <= 0.5:
        raise SystemExit("speed must be between 0 and 0.5 m/s")
    if not args.yes and input("The robot will walk forward. Is someone at the e-stop? [yes/no] ").strip().lower() != "yes":
        raise SystemExit("not started")
    rclpy.init()
    node = rclpy.create_node("terrain_test_driver")
    pub = node.create_publisher(Twist, args.topic, 10)
    msg = Twist()
    msg.linear.x = args.speed
    t_end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < t_end:
            pub.publish(msg)
            rclpy.spin_once(node, timeout_sec=0.0)
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    for _ in range(5):
        pub.publish(Twist())
        time.sleep(0.05)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
