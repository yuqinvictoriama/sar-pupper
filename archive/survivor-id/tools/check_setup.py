#!/usr/bin/env python3
"""Pre-flight check for survivor identification. Moves nothing.

    python3 tools/check_setup.py           # with the stack running (robot/start.sh)
    python3 tools/check_setup.py --no-ros  # just files, the model, the config

Goes through what the survivor nodes depend on and prints OK / PROBLEM for each, with what
to do about a problem.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys

import _common  # noqa: F401
from survivor.config import DODGE_ROOT, load_config, resolve

results = []


def report(ok, what: str, fix: str = "") -> None:
    results.append(bool(ok))
    print("%-8s %s%s" % ("OK" if ok else "PROBLEM", what, "" if ok or not fix else "\n         -> " + fix))


def ros2(*args, timeout=8) -> str:
    try:
        return subprocess.run(["ros2", *args], capture_output=True, text=True, timeout=timeout).stdout
    except (subprocess.TimeoutExpired, OSError):
        return ""


def once(topic: str, timeout=6):
    out = ros2("topic", "echo", "--once", "--field", "data", topic, timeout=timeout).strip().split("\n---")[0].strip()
    try:
        return json.loads(out.strip("'\""))
    except ValueError:
        return None


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-ros", action="store_true")
    args = ap.parse_args()

    cfg = load_config()
    report(True, "config loaded from %s (cameras: %s)" % (cfg["_path"], ", ".join(cfg["cameras"])))
    report(len(cfg["cameras"]) == 3, "all three cameras are in the dodging configuration and enabled",
           "see %s/config/dodge.yaml and its calib/ folder" % DODGE_ROOT)
    model = resolve(cfg["detector"]["model"])
    report(os.path.exists(model), "pose model %s" % model, "bash tools/download_model.sh")
    try:
        import hailo_platform  # noqa: F401
        report(True, "HailoRT's Python module is installed")
    except ImportError:
        report(False, "HailoRT's Python module (hailo_platform)", "this is the robot's own install; on a laptop use --backend torch in the tools")
    with open(os.path.join(DODGE_ROOT, "robot", "mux.yaml")) as f:
        mux = f.read()
    order = [mux.find(t) for t in ("/dodge_cmd_vel", cfg["mission"]["cmd_topic"], "/llm_cmd_vel")]
    report(-1 not in order and order == sorted(order), "the mux takes %s, below dodging and above Gemini" % cfg["mission"]["cmd_topic"],
           "update projectile-dodging/robot/mux.yaml")
    for name, cam in cfg["cameras"].items():
        if cam.get("source") == "v4l2":
            report(bool(cam.get("device")), "%s has a device set" % name,
                   "python3 %s/tools/find_cameras.py --assign" % DODGE_ROOT)
    bark = resolve(cfg["arrival"]["bark"])
    report(os.path.exists(bark), "bark sound %s" % bark, "set arrival.bark in config/survivor.yaml (the mission works without it)")
    report(any(shutil.which(p) for p in ("pw-play", "paplay", "aplay")), "an audio player for the bark (pw-play, paplay or aplay)")
    try:
        import zmq  # noqa: F401
        report(True, "pyzmq (the eye feed for Gemini's face)")
    except ImportError:
        report(False, "pyzmq", "the face's eyes will not follow people; everything else works")

    if not args.no_ros:
        nodes = ros2("node", "list")
        if not nodes:
            report(False, "ROS answers", "source robot/env.sh, and start the stack with robot/start.sh")
        else:
            for n in ("/survivor_node", "/survivor_approach", "/survivor_event_bridge", "/cmd_vel_mux"):
                report(n in nodes, "node %s is running" % n, "see /tmp/sar-survivor/robot.log")
            report("/dodge_node" in nodes, "the dodge node is running (it passes the rear cameras' frames on)",
                   "without it start with robot/start.sh --no-dodge so the survivor node opens the rear cameras itself")
            report("person_follower" not in nodes and "hailo_detection" not in nodes and "follow_service" not in nodes,
                   "nothing else is holding the Hailo", "stop the person follower")
            p = once("/survivor/perception")
            report(p is not None, "the survivor node is publishing /survivor/perception")
            if p:
                report(p.get("pose_model"), "the pose model is loaded on the Hailo", p.get("error") or "see the log")
                report(p.get("imu_alive"), "IMU data is arriving")
                for name in cfg["cameras"]:
                    age = p.get("cameras", {}).get(name)
                    report(age is not None and age < 1.0, "frames from %s" % name,
                           "rear cameras come through the dodge node; the fisheye from the course camera node")
                hz = p.get("infer_hz", 0.0)
                report(hz >= 0.6 * float(cfg["budget"]["inferences_per_s"]) or not p.get("enabled"),
                       "pose model runs at %.0f inferences a second (budget %s)" % (hz, cfg["budget"]["inferences_per_s"]),
                       "python3 tools/bench_hailo.py with the stack stopped; lower budget.inferences_per_s to what it can do")
            s = once("/survivor/state")
            report(s is not None, "the approach node is publishing /survivor/state")
            if s:
                print("         motion is %s, state %s, walking controller %s, robot server %s" % (
                    "ON" if s.get("motion") else "off (dry run)", s.get("state"),
                    "active" if s.get("walking") else "NOT active", "reachable" if s.get("robot_server") else "not reachable"))
        from robot_client import RobotClient
        client = RobotClient(cfg["robot_server"]["url"])
        up = client.reachable()
        report(up, "robot server at %s (needed for the bounce, for standing up, and for Gemini)" % cfg["robot_server"]["url"],
               "robot/start.sh --gemini (or personality/start.sh)")
        if up:
            _, tricks = client.tricks()
            have = [t for t in cfg["arrival"]["tricks"] if t in tricks]
            report(bool(have), "an arrival trick is installed: %s" % (have[0] if have else "none of %s" % cfg["arrival"]["tricks"]),
                   "push_up comes with gemini_pupper; bounce needs training first (bounce/README.md)")
            ok, _ = client._call("GET", "/mission/recent", timeout=1.5)
            report(ok, "the server has the mission-event feed (Gemini can be told about events)",
                   "start the server through personality/start.sh, not the lab's start.sh")
            web = os.path.expanduser("~/gemini_pupper/web/dist/assets")
            listening = os.path.isdir(web) and any("ws/mission" in open(os.path.join(web, f), errors="ignore").read()
                                                   for f in os.listdir(web) if f.endswith(".js"))
            report(listening, "the built web app listens for mission events",
                   "cd ~/gemini_pupper && git pull && bash robot/setup.sh   (rebuilds web/dist)")

    bad = results.count(False)
    print("\n%d checks, %s" % (len(results), "all OK" if not bad else "%d PROBLEM%s" % (bad, "" if bad == 1 else "S")))
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
