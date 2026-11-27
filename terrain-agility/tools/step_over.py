#!/usr/bin/env python3
"""Ask the step-over node for a motion and show what it reports. For bring-up, in this order:

    python3 tools/step_over.py cross --height 0.015 --depth 0.38 --near 0.36 --dry-run
        plan only: nothing moves. Prints the plan and how long planning took on this computer.
        A depth of 0.20 m or more is scaled across (onto the top, along it, off the far side);
        a depth of 0.06 to 0.10 m is stepped over (each foot uses the top once).
    python3 tools/step_over.py stand
        rise to the crossing height, stand 3 s, lower. No foot leaves the floor.
    python3 tools/step_over.py crawl --distance 0.08
        one slow crawl cycle straight ahead on clear floor.
    python3 tools/step_over.py cross --height 0.015 --depth 0.38 --near 0.36
        the real thing, with the obstacle where you say it is: --near is the distance from the
        body centre to its near face (front of the front feet + 0.087 m).
    python3 tools/step_over.py stop
        abort the motion in progress (joints go soft).

The robot must be standing still under its walking controller, with robot/terrain.launch.py
running. Someone must be at the e-stop for anything that moves.
"""
import argparse
import json
import time

import rclpy
from std_msgs.msg import String

END_STATES = ("done", "failed", "refused", "no_plan")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("cross")
    c.add_argument("--height", type=float, required=True, help="obstacle height (m)")
    c.add_argument("--depth", type=float, required=True, help="obstacle depth, fore-aft (m)")
    c.add_argument("--method", choices=("slab", "board"), default=None,
                   help="slab: scale across; board: step over. Default: by the depth")
    c.add_argument("--near", type=float, default=0.36, help="body centre to the box's near face (m)")
    c.add_argument("--skew", type=float, default=0.0, help="degrees; + = near face farther away on the left")
    s = sub.add_parser("stand")
    s.add_argument("--hold", type=float, default=3.0)
    w = sub.add_parser("crawl")
    w.add_argument("--distance", type=float, default=0.08)
    sub.add_parser("stop")
    for p in (c, s, w):
        p.add_argument("--dry-run", action="store_true", help="plan and report; do not move")
        p.add_argument("--yes", action="store_true", help="skip the e-stop question")
    ap.add_argument("--request-topic", default="/terrain/step_over")
    ap.add_argument("--status-topic", default="/terrain/crossing_status")
    args = ap.parse_args()

    req = {"cmd": args.cmd, "id": f"cli-{int(time.time())}"}
    if args.cmd == "cross":
        import math
        req.update(height=args.height, depth=args.depth, near_x=args.near, skew=math.radians(args.skew))
        if args.method:
            req["method"] = args.method
    elif args.cmd == "stand":
        req.update(hold=args.hold)
    elif args.cmd == "crawl":
        req.update(distance=args.distance)
    moving = args.cmd != "stop" and not args.dry_run
    if args.cmd != "stop":
        req["dry_run"] = bool(args.dry_run)
    if moving and not args.yes:
        if input("The legs will move under position control. Is someone at the e-stop? [yes/no] ").strip().lower() != "yes":
            raise SystemExit("not started")

    rclpy.init()
    node = rclpy.create_node("terrain_step_over_cli")
    result = {}

    def on_status(msg):
        try:
            st = json.loads(msg.data)
        except ValueError:
            return
        if st.get("request") not in (req["id"], None):
            return
        if st["state"] == "progress":
            print(f"\r  {st.get('stage', ''):6s} {st.get('phase', ''):14s} {100 * st.get('progress', 0):5.1f} %", end="", flush=True)
            return
        print("\n" + json.dumps(st, indent=1))
        if st["state"] in END_STATES:
            result.update(st)

    node.create_subscription(String, args.status_topic, on_status, 10)
    pub = node.create_publisher(String, args.request_topic, 10)
    t0 = time.monotonic()
    while pub.get_subscription_count() == 0 and time.monotonic() - t0 < 5.0:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        raise SystemExit("the step-over node is not running (ros2 launch robot/terrain.launch.py)")
    m = String()
    m.data = json.dumps(req)
    pub.publish(m)
    if args.cmd == "stop":
        time.sleep(0.3)
    else:
        try:
            while not result and time.monotonic() - t0 < 420.0:
                rclpy.spin_once(node, timeout_sec=0.1)
        except KeyboardInterrupt:
            m.data = json.dumps({"cmd": "stop"})
            pub.publish(m)
            print("\nstop sent")
            time.sleep(0.3)
    node.destroy_node()
    rclpy.shutdown()
    raise SystemExit(0 if result.get("state") == "done" or args.cmd == "stop" else 1)


if __name__ == "__main__":
    main()
