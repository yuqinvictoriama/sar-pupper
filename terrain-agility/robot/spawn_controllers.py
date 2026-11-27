#!/usr/bin/env python3
"""Load the three terrain forward command controllers, inactive.

    python3 robot/spawn_controllers.py

Run by robot/terrain.launch.py (and safe to run by hand, any number of times) once the
robot stack is up. Same pattern as gemini_pupper/robot/spawn_controllers.py: wait for the
course's own controllers first, because loading several controllers at once makes the
controller manager's service calls time out.
"""

from __future__ import annotations

import re
import subprocess
import sys
import time
from pathlib import Path

PARAM_FILE = Path(__file__).resolve().parent / "forward_controllers.yaml"
NAMES = ["terrain_position_controller", "terrain_kp_controller", "terrain_kd_controller"]
LOADED = {"inactive", "active"}


def controller_states() -> dict:
    try:
        out = subprocess.run(["ros2", "control", "list_controllers"], capture_output=True, text=True, timeout=20).stdout
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return {}
    states = {}
    for line in re.sub(r"\x1b\[[0-9;]*m", "", out).splitlines():
        words = line.split()
        if len(words) >= 3:
            states[words[0]] = words[2]
    return states


def spawn(names) -> bool:
    cmd = ["ros2", "run", "controller_manager", "spawner", *names, "--controller-manager", "/controller_manager",
           "--controller-manager-timeout", "60", "--service-call-timeout", "60", "--inactive",
           "--param-file", str(PARAM_FILE)]
    return subprocess.run(cmd).returncode == 0


def main() -> int:
    deadline = time.monotonic() + 120.0
    states = {}
    while time.monotonic() < deadline:
        states = controller_states()
        if states.get("imu_sensor_broadcaster") == "active" and states.get("joint_state_broadcaster") == "active":
            break
        time.sleep(1.0)
    else:
        print("spawn_controllers: the robot stack's controllers did not come up; is it running?", flush=True)
        return 1
    for attempt in range(3):
        missing = [n for n in NAMES if states.get(n) not in LOADED]
        if not missing:
            break
        for n in missing:
            spawn([n])
        states = controller_states()
    failed = [n for n in NAMES if states.get(n) not in LOADED]
    if failed:
        print("spawn_controllers: FAILED to load " + ", ".join(failed) +
              ". Check `ros2 control list_controller_types | grep forward_command_controller`.", flush=True)
        return 1
    active = [n for n in NAMES if states.get(n) == "active"]
    if active:
        print("spawn_controllers: WARNING " + ", ".join(active) + " are ACTIVE (left over from a crashed run?); "
              "robot/estop_guard.py will release them.", flush=True)
    print("spawn_controllers: terrain controllers loaded (inactive)", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
