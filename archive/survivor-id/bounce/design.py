"""The arrival bounce: Pupper bobs up and down on the spot, feet planted.

    cd pupper-mjlab && uv run python <this file> <out.npz>

Played when Pupper reaches the survivor (survivor-id/plan.md, "Arrival"). A quick
dip, then BOUNCES pops up to nearly straight legs with the nose a little up, and
a soft settle back to the stand. All four feet stay where they stand the whole
time: the body moves, the feet do not. Designed the way gemini_pupper's sneeze
was (tricks/sneeze/design.py), and trained with Mjlab-Tracking-Flat-Pupper-v3.

It must never hop. Every stroke is slow enough that gravity alone can stop the
body at the top: the report's lowest_ground_load is the smallest share of the
robot's weight the feet carry during the motion (1.0 = standing still, 0 = the
feet are about to leave the floor). Keep it above 0.5.
"""

import sys

import mujoco
import numpy as np

from mjlab.tasks.tracking.config.pupper import motion_design as md

BOUNCES = 3
DIP = 0.045  # how far the body drops below the stand at the bottom of a bounce (m)
RISE = 0.015  # how far it rises above the stand at the top (m); legs run out at about +0.02
NOSE_UP = 4.0  # nose lifts this much at the top of each bounce (deg): a happy bounce, not a push-up
WIND_UP_S = 0.34  # stand -> first dip
STROKE_S = 0.30  # dip -> top, and top -> dip; shorter unloads the feet (see lowest_ground_load)
SETTLE_S = 0.45  # last top -> stand
HOLD_S = 0.30  # standing still at the start, and again at the end
MIN_GROUND_LOAD = 0.5


def keyframes() -> list[md.Keyframe]:
  feet = md.stand_feet()  # all four feet stay here from the first frame to the last
  h = md.STAND_HEIGHT
  low = (0.0, 0.0, h - DIP)
  high = (0.0, 0.0, h + RISE)
  t = HOLD_S
  kfs = [md.stand(0.0), md.stand(t)]
  t += WIND_UP_S
  kfs.append(md.Keyframe(t, low, feet=feet))  # the wind-up dip
  for i in range(BOUNCES):
    t += STROKE_S
    kfs.append(md.Keyframe(t, high, (0, -NOSE_UP, 0), feet=feet))  # up!
    if i < BOUNCES - 1:
      t += STROKE_S
      kfs.append(md.Keyframe(t, low, feet=feet))  # and down again
  t += SETTLE_S
  kfs.append(md.stand(t))
  kfs.append(md.stand(t + HOLD_S))
  return kfs


def ground_load(motion: md.Motion) -> np.ndarray:
  """Share of the robot's weight on the feet at each frame: 1 + (centre-of-mass
  vertical acceleration) / g. Below about 0.5 a tracking error could lift the feet."""
  k = md.kinematics()
  z = np.zeros(len(motion.root_pos))
  for i in range(len(z)):
    k.set_state(motion.root_pos[i], motion.root_quat[i], motion.joint_pos[i])
    mujoco.mj_comPos(k.model, k.data)
    z[i] = k.data.subtree_com[k.base_body][2]
  acc = np.gradient(np.gradient(z, 1.0 / motion.fps), 1.0 / motion.fps)
  return 1.0 + acc / 9.81


def build() -> md.Motion:
  m = md.build_motion(keyframes())  # its report already has pupper-mjlab's own checks
  load = ground_load(m)
  m.report["lowest_ground_load"] = round(float(load.min()), 2)
  m.report["highest_ground_load"] = round(float(load.max()), 2)
  feet = md.stand_feet()
  k = md.kinematics()
  slip = 0.0
  for i in range(len(m.root_pos)):
    now = k.feet(m.root_pos[i], m.root_quat[i], m.joint_pos[i])
    slip = max(slip, max(float(np.linalg.norm(now[leg] - np.array(feet[leg]))) for leg in md.LEGS))
  m.report["max_foot_movement_m"] = round(slip, 4)  # 0: the feet never leave their spots
  return m


if __name__ == "__main__":
  m = build()
  for key, value in m.report.items():
    print(f"{key:36} {value}")
  if m.report["lowest_ground_load"] < MIN_GROUND_LOAD:
    sys.exit(f"the feet carry under {MIN_GROUND_LOAD} of the weight at some point: lengthen STROKE_S or shrink DIP + RISE")
  print("saved", m.save_npz(sys.argv[1] if len(sys.argv) > 1 else "bounce.npz"))
