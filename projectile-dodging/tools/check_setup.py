#!/usr/bin/env python3
"""Pre-flight check. Moves nothing.

    python3 tools/check_setup.py           # with the robot stack running (robot/start.sh)
    python3 tools/check_setup.py --no-ros  # just the files, config and USB cameras

Goes through everything the dodge node depends on and prints OK / PROBLEM for
each, with what to do about a problem.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time

import yaml

from _common import cv2, load_config, np, run_tool

results = []


def report(ok, what: str, fix: str = "") -> None:
    results.append(bool(ok))
    print("%-8s %s%s" % ("OK" if ok else "PROBLEM", what, "" if ok or not fix else "\n         -> " + fix))


def ros2(*args, timeout=8):
    try:
        return subprocess.run(["ros2", *args], capture_output=True, text=True, timeout=timeout).stdout
    except (subprocess.TimeoutExpired, OSError):
        return ""


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-ros", action="store_true")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()

    print("python %s, numpy %s, opencv %s" % (sys.version.split()[0], np.__version__, cv2.__version__))
    cfg = load_config(args.config)
    report(True, "config loaded from %s" % cfg["_path"])
    report(shutil.which("v4l2-ctl"), "v4l2-ctl available (fixes camera exposure and white balance)",
           "sudo apt install v4l-utils")
    for name, cam in cfg["cameras"].items():
        if not cam.get("enabled", True):
            print("skipped  %s (disabled in the config)" % name)
            continue
        calibrated = os.path.exists(os.path.join(os.path.dirname(cfg["_path"]), "calib", name + "_color.yaml"))
        report(calibrated, "%s: colour calibrated" % name, "python3 tools/calibrate_color.py %s" % name)
        if cam.get("source") == "v4l2":
            dev = cam.get("device", "")
            report(bool(dev) and os.path.exists(dev), "%s: camera present at %s" % (name, dev or "(not set)"),
                   "python3 tools/find_cameras.py   (it says what is wrong and what to do)")
            tuned = False
            try:
                with open(os.path.join(os.path.dirname(cfg["_path"]), "calib", "exposure.yaml")) as f:
                    tuned = name in ((yaml.safe_load(f) or {}).get("cameras") or {})
            except OSError:
                pass
            report(tuned, "%s: exposure and gain set for this room" % name, "python3 tools/tune_exposure.py %s" % name)

    if args.no_ros:
        print("\n%d problem(s)" % results.count(False))
        return
    report(os.environ.get("ROS_LOCALHOST_ONLY") == "1", "ROS_LOCALHOST_ONLY=1 (the robot stack runs with it)",
           "export ROS_LOCALHOST_ONLY=1 in this terminal, or nothing below will be found")
    try:
        import rclpy  # noqa: F401
    except ImportError:
        report(False, "rclpy importable", "source /opt/ros/jazzy/setup.bash and the monorepo's install/local_setup.bash")
        print("\n%d problem(s)" % results.count(False))
        return
    from sensor_msgs.msg import CompressedImage, Imu
    from std_msgs.msg import String

    from _common import ros_node
    node = ros_node("dodge_check")
    seen = {"imu": 0, "img": 0, "state": None, "img_t": []}
    node.create_subscription(Imu, cfg["node"]["imu_topic"], lambda m: seen.__setitem__("imu", seen["imu"] + 1), 100)

    def on_img(_m):
        seen["img"] += 1
        seen["img_t"].append(time.time())
    fisheye = cfg["cameras"].get("fisheye", {})
    node.create_subscription(CompressedImage, fisheye.get("topic", "/camera/image_raw/compressed"), on_img, 5)
    node.create_subscription(String, "/dodge/state", lambda m: seen.__setitem__("state", json.loads(m.data)), 5)
    time.sleep(4.0)
    report(seen["imu"] > 200, "IMU publishing (%.0f Hz)" % (seen["imu"] / 4.0), "start the robot stack: robot/start.sh")
    fps = (len(seen["img_t"]) - 1) / (seen["img_t"][-1] - seen["img_t"][0]) if len(seen["img_t"]) > 2 else 0.0
    report(seen["img"] > 5, "fisheye frames arriving (%.0f fps)" % fps, "start the robot stack: robot/start.sh")
    if 0 < fps < 25:
        print("         note: under 25 fps the front is slow to react; try robot/start.sh --fisheye-fps 60")

    mux_inputs = ros2("param", "get", "/cmd_vel_mux", "inputs")
    cmd_topic = cfg["node"]["cmd_topic"]
    report(cmd_topic in mux_inputs, "velocity mux listens to %s" % cmd_topic,
           "the stack was not started by robot/start.sh (or sar_dodge.launch.py); stop it and start it that way")
    listing = ros2("control", "list_controllers", timeout=15)
    walking = [n for n in cfg["node"]["walking_controllers"] if any(
        line.split() and line.split()[0] == n and "active" in line and "inactive" not in line for line in listing.splitlines())]
    report(bool(walking), "a walking controller is active (%s)" % (", ".join(walking) or "none"),
           "wake the robot: joystick X, or Gemini's wake-up, or: ros2 control switch_controllers --activate gemini_walk")

    st = seen["state"]
    report(st is not None, "dodge node running and reporting", "see /tmp/sar-dodge/robot.log")
    if st is not None:
        for name, c in st.get("cameras", {}).items():
            report(c["alive"], "dodge node gets frames from %s (%.0f fps, %.0f ms behind, %.0f ms to process)" % (
                name, c["fps"], c["latency_ms"], c["proc_ms"]),
                c.get("problem") or "stop the stack (robot/stop.sh) and run  python3 tools/find_cameras.py")
            if c["alive"] and 0 <= c.get("brightness", -1) < 25:
                report(False, "%s: picture bright enough (average %.0f of 255)" % (name, c["brightness"]),
                       "stop the stack, then  python3 tools/tune_exposure.py %s" % name)
        report(st.get("imu_alive"), "dodge node gets the IMU")
        report(st.get("can_dodge"), "dodge node says it can dodge" + ("" if st.get("can_dodge") else ": " + st.get("cannot_reason", "")))
        print("         dodge node is %s" % ("ARMED" if st.get("armed") else "in dry run (not armed)"))
    print("\n%d problem(s)" % results.count(False))


if __name__ == "__main__":
    run_tool(main)
