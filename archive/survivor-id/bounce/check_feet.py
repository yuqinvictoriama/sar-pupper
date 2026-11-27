"""Does any foot leave the floor? Replay the bounce with physics and measure.

    cd pupper-mjlab && uv run python <this file> bounce.npz
    ... bounce.npz --checkpoint logs/rsl_rl/pupper_tracking/<run>/model_3000.pt   # a trained policy
    ... bounce.npz --mass-scale 1.15 --com-x 0.02     # the cameras and mics on the head

pupper-mjlab's play_motion says whether the body follows the reference. This asks
the bounce's own question: does a foot ever leave the floor, and do the feet end
where they started? (Pupper's feet are balls, so they roll a centimetre or so as
the legs bend and roll back; that is not sliding.)
Without --checkpoint the joints are driven straight to the reference (open loop),
as play_motion does. Same robot options as play_motion.
"""

from __future__ import annotations

import math
from pathlib import Path

import torch
import tyro

import mjlab
from mjlab.actuator import BuiltinPositionActuatorCfg
from mjlab.asset_zoo.robots.pupper_v3.pupper_constants import PUPPER_ACTION_SCALE
from mjlab.tasks.tracking.config.pupper import motion_design as md
from mjlab.tasks.tracking.config.pupper.play_motion import REPLAY_KD, REPLAY_KP

MAX_LIFT_M = 0.003  # a foot more than this above the floor counts as leaving it
MAX_CREEP_M = 0.015  # how far a foot may end from where it started
SETTLE_STEPS = 10  # 0.2 s at 50 Hz


def main(
  motion_file: tyro.conf.Positional[Path],
  checkpoint: Path | None = None,
  mass_scale: float = 1.0,
  com_x: float = 0.0,
  com_y: float = 0.0,
  com_z: float = 0.0,
  show_timeline: bool = False,
) -> None:
  from mjlab.envs import ManagerBasedRlEnv
  from mjlab.rl import RslRlVecEnvWrapper
  from mjlab.tasks.tracking.config.pupper.env_cfgs import pupper_tracking_env_cfg
  from mjlab.tasks.tracking.mdp import MotionCommand

  device = "cuda:0" if torch.cuda.is_available() else "cpu"
  cfg = pupper_tracking_env_cfg(motion_file=str(motion_file.resolve()), play=True)
  cfg.scene.num_envs = 1
  cfg.terminations = {}
  for name in ("encoder_bias", "foot_friction", "pd_gains"):  # the nominal robot, as play_motion
    cfg.events.pop(name, None)
  cfg.commands["motion"].joint_position_range = (0.0, 0.0)  # start exactly on the first frame
  cfg.events["base_com"].params["ranges"] = {i: (d, d) for i, d in enumerate((com_x, com_y, com_z))}
  alpha = math.log(mass_scale) / 2
  cfg.events["inertial"].params["alpha_range"] = (alpha, alpha)
  articulation = cfg.scene.entities["robot"].articulation
  assert articulation is not None
  actuator = articulation.actuators[0]
  assert isinstance(actuator, BuiltinPositionActuatorCfg)
  if checkpoint is None:
    actuator.stiffness, actuator.damping = REPLAY_KP, REPLAY_KD
  env = ManagerBasedRlEnv(cfg=cfg, device=device)
  robot = env.scene["robot"]
  cmd = env.command_manager.get_term("motion")
  assert isinstance(cmd, MotionCommand)
  default = robot.data.default_joint_pos
  n_frames = int(cmd.motion.time_step_total)
  wrapped = RslRlVecEnvWrapper(env)

  def replay(obs) -> torch.Tensor:
    del obs
    nxt = torch.clamp(cmd.time_steps + 1, max=n_frames - 1)
    return (cmd.motion.joint_pos[nxt] - default) / PUPPER_ACTION_SCALE

  policy = replay
  if checkpoint is not None:
    from dataclasses import asdict

    from mjlab.tasks.registry import load_rl_cfg
    from mjlab.tasks.tracking.rl import MotionTrackingOnPolicyRunner

    runner = MotionTrackingOnPolicyRunner(wrapped, asdict(load_rl_cfg("Mjlab-Tracking-Flat-Pupper-v3")), device=device)
    runner.load(str(checkpoint), map_location=device)
    policy = runner.get_inference_policy(device=device)

  feet = [robot.site_names.index(f"leg_{leg}_3_foot_site") for leg in md.LEGS]
  wrapped.reset()
  obs = wrapped.get_observations()
  # The simulator places the robot a few millimetres above the floor and lets it
  # land, so the first SETTLE_STEPS steps are not counted (the motion stands still
  # for longer than that).
  start = None
  lift = torch.zeros(4, device=device)
  roll = torch.zeros(4, device=device)
  p = robot.data.site_pos_w[0, feet]
  worst_pos = worst_tilt = 0.0
  timeline = []
  for step in range(n_frames - 1):
    with torch.inference_mode():
      obs, *_ = wrapped.step(policy(obs))
    p = robot.data.site_pos_w[0, feet]
    if (step + 1) % 5 == 0:
      timeline.append(f"{(step + 1) * env.step_dt:4.1f}s body {float(robot.data.root_link_pos_w[0, 2]) * 1000:5.0f} mm "
                      f"(reference {float(cmd.anchor_pos_w[0, 2]) * 1000:5.0f}), highest foot {float(p[:, 2].max() - md.FOOT_RADIUS) * 1000:4.1f} mm")
    if step < SETTLE_STEPS:
      start = p.clone()
      continue
    assert start is not None
    lift = torch.maximum(lift, p[:, 2] - md.FOOT_RADIUS)
    roll = torch.maximum(roll, torch.linalg.norm(p[:, :2] - start[:, :2], dim=1))
    worst_pos = max(worst_pos, float(torch.linalg.norm(cmd.robot_anchor_pos_w - cmd.anchor_pos_w)))
    dot = float(torch.abs((cmd.robot_anchor_quat_w * cmd.anchor_quat_w).sum()))
    worst_tilt = max(worst_tilt, math.degrees(2 * math.acos(min(1.0, dot))))
  if show_timeline:
    print("\n".join("  " + line for line in timeline))
  who = f"trained policy {checkpoint.name}" if checkpoint else "open-loop joint replay"
  print(f"motion        {motion_file.name}: {n_frames} frames ({who}; mass x{mass_scale:g}, "
        f"centre of mass moved {com_x:+g}, {com_y:+g}, {com_z:+g} m)")
  assert start is not None
  creep = torch.linalg.norm(p[:, :2] - start[:, :2], dim=1)
  for leg, up, out, end in zip(md.LEGS, lift.tolist(), roll.tolist(), creep.tolist(), strict=True):
    print(f"  {leg:8} highest {up * 1000:4.1f} mm above the floor; rolled up to {out * 1000:4.1f} mm, ended {end * 1000:4.1f} mm from its spot")
  print(f"body          worst {worst_pos * 100:.1f} cm and {worst_tilt:.1f} deg from the reference")
  hopped, walked = float(lift.max()) > MAX_LIFT_M, float(creep.max()) > MAX_CREEP_M
  if hopped:
    print(f"verdict       LEAVES THE FLOOR: a foot rose more than {MAX_LIFT_M * 1000:.0f} mm. Slow the strokes or shrink the bounce.")
  elif walked:
    print(f"verdict       CREEPS: a foot ended more than {MAX_CREEP_M * 1000:.0f} mm from where it started.")
  else:
    print("verdict       FEET STAY DOWN: no foot left the floor, and all four ended where they started.")
  env.close()


if __name__ == "__main__":
  tyro.cli(main, config=mjlab.TYRO_FLAGS)
