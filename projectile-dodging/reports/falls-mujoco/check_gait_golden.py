"""Check the Python gait reference against pupper_gait_deploy/test/gait_golden.json
(offsets generated from the trained mjlab env itself)."""
import json, sys
from pathlib import Path
import numpy as np
from pupper_policy_sim import GaitReference, POLICY_JSON
T = Path.home() / "Desktop/CS123/pi/pupper_gait_deploy/test"
for name in ("gait_golden.json", "mixed_jump_golden.json"):
    g = json.load(open(T / name))
    ref = GaitReference(g["gait_reference"], g["default_joint_pos"])
    worst = 0
    for c in g["cases"]:
        out = ref.offset(c["t"], c["vx"], c["vy"], c["yaw"])
        worst = max(worst, float(np.abs(out - np.asarray(c["expected"])).max()))
    print(name, len(g["cases"]), "cases, worst abs err", worst)
d = json.load(open(POLICY_JSON))["gait_reference"]
same = all(np.allclose(d[k], json.load(open(T / "mixed_jump_golden.json"))["gait_reference"][k]) for k in ("trot_table", "gallop_table", "lift_table", "gallop_back_table"))
print("default_policy.json tables equal mixed_jump_golden tables:", same)
