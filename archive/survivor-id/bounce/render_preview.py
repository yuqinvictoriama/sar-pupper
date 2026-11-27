"""Render the reference motion (no physics) to a GIF, to judge the look before training.

    cd pupper-mjlab && MUJOCO_GL=egl uv run python <this file> bounce.npz bounce_preview.gif
"""
import sys

import mujoco
import numpy as np
from PIL import Image

from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import JOINT_NAMES
from mjlab.tasks.tracking.config.pupper.env_cfgs import get_spec_with_hulls

W, H, EVERY = 400, 300, 2  # pixels; keep every 2nd frame of the 50 Hz motion (25 fps)


def main(npz: str, out: str) -> None:
  data = np.load(npz)
  spec = get_spec_with_hulls()
  spec.worldbody.add_geom(type=mujoco.mjtGeom.mjGEOM_PLANE, size=[2, 2, 0.01], rgba=[0.82, 0.82, 0.8, 1])
  spec.worldbody.add_light(pos=[0.3, -0.6, 1.2], dir=[-0.2, 0.4, -1], diffuse=[0.9, 0.9, 0.9])
  for g in spec.geoms:
    if g.name.endswith("_hull"):
      g.rgba = [0, 0, 0, 0]  # collision hulls: not part of the picture
  model = spec.compile()
  model.vis.headlight.ambient[:] = 0.45
  d = mujoco.MjData(model)
  qadr = [model.jnt_qposadr[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)] for n in JOINT_NAMES]
  names = [str(n) for n in data["body_names"]]
  base = names.index("base_link")
  cam = mujoco.MjvCamera()
  cam.lookat[:] = (0.03, 0.0, 0.10)
  cam.distance, cam.azimuth, cam.elevation = 0.7, 120, -12
  frames = []
  with mujoco.Renderer(model, H, W) as r:
    for i in range(0, len(data["joint_pos"]), EVERY):
      d.qpos[:3] = data["body_pos_w"][i, base]
      d.qpos[3:7] = data["body_quat_w"][i, base]
      d.qpos[qadr] = data["joint_pos"][i]
      mujoco.mj_forward(model, d)
      r.update_scene(d, cam)
      frames.append(Image.fromarray(r.render()))
  frames[0].save(out, save_all=True, append_images=frames[1:], duration=int(1000 * EVERY / float(data["fps"][0])), loop=0)
  print("saved", out, f"({len(frames)} frames)")


if __name__ == "__main__":
  main(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else "bounce_preview.gif")
