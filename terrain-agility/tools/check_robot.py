#!/usr/bin/env python3
"""Pre-flight check on the robot: is everything terrain agility needs present and sane?
Nothing moves. Run it with the robot stack and robot/terrain.launch.py running and the
robot standing under its walking controller.

    python3 tools/check_robot.py            # everything
    python3 tools/check_robot.py --offline  # only what needs no ROS (also works on a laptop)

Each line is OK, WARN or FAIL, with what to do about it.
"""
import argparse
import math
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from terrain.config import TerrainConfig  # noqa: E402
from terrain.kinematics import DEFAULT_JOINT_POS, JOINT_NAMES  # noqa: E402

COUNT = {"OK": 0, "WARN": 0, "FAIL": 0}


def say(level, what, hint=""):
    COUNT[level] += 1
    print(f"[{level:4s}] {what}" + (f"\n         -> {hint}" if hint and level != "OK" else ""))


def offline(cfg):
    import cv2
    import yaml  # noqa: F401
    say("OK", f"python {sys.version.split()[0]}, numpy {np.__version__}, opencv {cv2.__version__}")
    from terrain.crawl_planner import BoxSpec, load_templates, plan_crossing
    n = len(load_templates())
    say("OK" if n else "FAIL", f"{n} stored footstep patterns (config/crawl_templates.json)",
        "the file is missing: copy it from the repo, or build it with sim/build_templates.py on a laptop")
    floor = cfg.load_floor()
    say("OK" if floor.trained else "WARN", "floor colour model " + ("present" if floor.trained else "missing"),
        "python3 tools/learn_colors.py floor   (until then it learns the floor from whatever is ahead at start-up)")
    p = cfg.pose
    untouched = abs(p.height - 0.250) < 1e-9 and abs(p.pitch_deg) < 1e-9 and abs(p.x_forward - 0.145) < 1e-9
    say("WARN" if untouched else "OK", f"camera pose: height {p.height * 1000:.0f} mm, pitch {p.pitch_deg:+.1f} deg, "
        f"{p.x_forward * 1000:.0f} mm ahead of the body centre" + (" (still the uncalibrated defaults)" if untouched else ""),
        "python3 tools/calibrate_camera.py   (heights and distances are not trustworthy until then)")
    masses = sum(m for m, _ in cfg.crawl.extra_masses)
    say("OK", f"extra hardware in the balance model: {masses * 1000:.0f} g "
              "(crawl.extra_masses in terrain.yaml; weigh the camera plate, cameras and mics)")
    if n:
        t0 = time.time()
        plan = plan_crossing(BoxSpec(0.025, 0.064, cfg.fsm.gauge_distance), cfg.crawl, search=False)
        dt = time.time() - t0
        if plan.ok:
            say("OK" if dt < 20 else "WARN", f"planning a 25 x 64 mm box from {cfg.fsm.gauge_distance * 100:.0f} cm: "
                f"{dt:.1f} s on this computer, {len(plan.steps)} foot swings, {plan.duration:.0f} s to walk",
                "slow, but it only delays the start of a crossing")
        else:
            say("FAIL", f"no plan for a 25 x 64 mm box: {plan.reason}", "check crawl.* in terrain.yaml (extra_masses, mic_bars)")
    from terrain.crawl_planner import plan_scale_across
    t0 = time.time()
    plan = plan_scale_across(BoxSpec(0.015, 0.38, cfg.fsm.gauge_distance), cfg.crawl)
    dt = time.time() - t0
    if plan.ok:
        say("OK" if dt < 40 else "WARN", f"planning to scale across a 15 x 380 mm slab from {cfg.fsm.gauge_distance * 100:.0f} cm: "
            f"{dt:.1f} s on this computer, {len(plan.steps)} foot swings, {plan.duration:.0f} s to crawl",
            "slow, but it only delays the start of a crossing")
    else:
        say("FAIL", f"no plan for a 15 x 380 mm slab: {plan.reason}", "check crawl.* in terrain.yaml (extra_masses, mic_bars)")


def online(cfg):
    import rclpy
    from controller_manager_msgs.srv import ListControllers, ListControllerTypes, ListHardwareInterfaces
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import CompressedImage, Imu, JointState

    rclpy.init()
    node = rclpy.create_node("terrain_check")

    def call(srv_type, name, request, timeout=4.0):
        client = node.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(request)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        return fut.result()

    res = call(ListControllers, "/controller_manager/list_controllers", ListControllers.Request())
    if res is None:
        say("FAIL", "controller_manager does not answer", "start the robot stack first (robot/standalone.launch.py, or Gemini's start.sh)")
        node.destroy_node()
        rclpy.shutdown()
        return
    states = {c.name: (c.type, c.state) for c in res.controller}
    types = call(ListControllerTypes, "/controller_manager/list_controller_types", ListControllerTypes.Request())
    have = types is not None and "forward_command_controller/ForwardCommandController" in types.types
    say("OK" if have else "FAIL", "forward_command_controller is installed",
        "sudo apt install ros-jazzy-forward-command-controller   (the step-over needs it)")
    so = cfg.stepover
    missing = [c for c in so["controllers"] if c not in states]
    active = [c for c in so["controllers"] if states.get(c, ("", ""))[1] == "active"]
    say("OK" if not missing else "FAIL", "terrain controllers loaded: " + (", ".join(so["controllers"]) if not missing else f"MISSING {missing}"),
        "ros2 launch robot/terrain.launch.py   (or python3 robot/spawn_controllers.py)")
    if active:
        say("WARN", f"terrain controllers are ACTIVE right now: {active}", "fine during a crossing; otherwise robot/estop_guard.py should release them")
    walking = [n for n, (t, s) in states.items() if t in so["walking_controller_types"] and s == "active"]
    say("OK" if len(walking) == 1 else "WARN", f"active walking controller: {walking or 'none'}",
        "exactly one walking controller must hold the legs before a crossing (joystick X, or Gemini's stand-up)")
    hw = call(ListHardwareInterfaces, "/controller_manager/list_hardware_interfaces", ListHardwareInterfaces.Request())
    if hw is not None:
        names = {i.name for i in hw.command_interfaces}
        need = [f"{JOINT_NAMES[0]}/{k}" for k in ("position", "kp", "kd")]
        say("OK" if all(n in names for n in need) else "FAIL", "hardware exports position / kp / kd command interfaces",
            f"expected {need}; this robot's hardware interface is different from the one this was written for")

    mux = call(GetParameters, "/cmd_vel_mux/get_parameters", GetParameters.Request(names=["inputs"]), timeout=2.0)
    if mux is None or not mux.values:
        say("WARN", "cmd_vel_mux is not running", "the launch you use must start cmd_vel_mux with robot/terrain_mux.yaml")
    else:
        inputs = list(mux.values[0].string_array_value)
        if cfg.cmd_topic not in inputs:
            say("FAIL", f"cmd_vel_mux does not listen to {cfg.cmd_topic} (inputs: {inputs})",
                "give the mux the inputs list in robot/terrain_mux.yaml (README: 'Wiring into the mux')")
        else:
            mine = inputs.index(cfg.cmd_topic)
            above = [t for t in cfg.intent_topics if t in inputs and inputs.index(t) < mine]
            unheard = [t for t in cfg.intent_topics if t not in inputs]
            say("OK" if not above else "FAIL", f"mux priority: {' > '.join(inputs)}",
                f"{above} outrank {cfg.cmd_topic}: terrain agility could not steer round what they walk into")
            if unheard:
                say("WARN", f"intent topics the mux does not carry: {unheard}", "harmless; they just cannot drive the robot")

    # sensor streams
    got = {"joints": [], "imu": [], "image": [], "size": None, "q": [], "dq": [], "tilt": []}

    def on_joints(msg):
        got["joints"].append(time.monotonic())
        try:
            idx = [list(msg.name).index(n) for n in JOINT_NAMES]
        except ValueError:
            return
        got["q"].append([msg.position[i] for i in idx])
        if len(msg.velocity) >= len(msg.name):
            got["dq"].append([msg.velocity[i] for i in idx])

    def on_imu(msg):
        got["imu"].append(time.monotonic())
        o = msg.orientation
        if abs(o.x) + abs(o.y) + abs(o.z) + abs(o.w) > 1e-6:
            from terrain.camera_model import level_from_body, tilt_of
            got["tilt"].append(math.degrees(tilt_of(level_from_body((o.x, o.y, o.z, o.w)))))

    def on_image(msg):
        got["image"].append(time.monotonic())
        if got["size"] is None:
            import cv2
            img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
            got["size"] = None if img is None else (img.shape[1], img.shape[0])

    node.create_subscription(JointState, "/joint_states", on_joints, 50)
    node.create_subscription(Imu, "/imu_sensor_broadcaster/imu", on_imu, 50)
    node.create_subscription(CompressedImage, cfg.camera_topic, on_image, 5)
    t_end = time.monotonic() + 4.0
    while time.monotonic() < t_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    def rate(stamps):
        return 0.0 if len(stamps) < 2 else (len(stamps) - 1) / (stamps[-1] - stamps[0])

    r = rate(got["joints"])
    say("OK" if r > 100 else "FAIL", f"/joint_states at {r:.0f} Hz", "need 100 Hz or more (the course config gives 260)")
    r = rate(got["imu"])
    say("OK" if r > 50 and got["tilt"] else "FAIL", f"IMU at {r:.0f} Hz" + ("" if got["tilt"] else ", but with no orientation"),
        "the imu_sensor_broadcaster must be active and publishing an orientation")
    r = rate(got["image"])
    say("OK" if r >= 8 else ("WARN" if r > 0 else "FAIL"), f"camera at {r:.1f} frames/s on {cfg.camera_topic}",
        "need about 8 or more; is the camera node running, and is another process hogging the CPU?")
    if got["size"]:
        match = got["size"] == (cfg.model.width, cfg.model.height)
        ratio_ok = abs(got["size"][0] / got["size"][1] - cfg.model.width / cfg.model.height) < 0.01
        say("OK" if match else ("WARN" if ratio_ok else "FAIL"),
            f"camera frames are {got['size'][0]} x {got['size'][1]}" + ("" if match else f" (lens model: {cfg.model.width} x {cfg.model.height})"),
            "same shape: the lens model is scaled, with less resolution for the height gauge" if ratio_ok else
            "different shape: the lens model does not apply; run the camera at 1400 x 1050")
    if got["tilt"]:
        tilt = float(np.median(got["tilt"]))
        say("OK" if tilt < 5 else "WARN", f"body tilt {tilt:.1f} deg", "stand the robot on level floor (over 8 deg nothing will start)")
    if got["q"]:
        q = np.array(got["q"])
        dev = float(np.abs(q - DEFAULT_JOINT_POS).max())
        sway = float(np.ptp(q, axis=0).max())
        speed = float(np.abs(np.array(got["dq"])).max()) if got["dq"] else 0.0
        if dev > 1.0:
            say("WARN", f"legs are far from the standing pose (a joint is {dev:.2f} rad off): is the robot lying down?",
                "stand it up under the walking controller and run this again")
        else:
            ok = dev < 0.30 and speed < 1.0
            say("OK" if ok and sway < 0.10 else ("WARN" if ok else "FAIL"),
                f"standing pose: joints within {dev:.2f} rad of the default pose, moving {sway:.2f} rad over 4 s, "
                f"peak speed {speed:.1f} rad/s",
                "a crossing only starts with every joint within 0.30 rad of the default pose and slower than 1 rad/s. "
                "If the walking policy shuffles its feet while standing, use a policy that stands still at zero command")
    node.destroy_node()
    rclpy.shutdown()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--config", default=None)
    ap.add_argument("--offline", action="store_true")
    args = ap.parse_args()
    cfg = TerrainConfig(args.config)
    offline(cfg)
    if not args.offline:
        try:
            online(cfg)
        except ImportError as e:
            say("FAIL", f"ROS 2 Python packages not found ({e})", "source /opt/ros/jazzy/setup.bash and the monorepo workspace first")
    print(f"\n{COUNT['OK']} ok, {COUNT['WARN']} warnings, {COUNT['FAIL']} failures")
    raise SystemExit(1 if COUNT["FAIL"] else 0)


if __name__ == "__main__":
    main()
