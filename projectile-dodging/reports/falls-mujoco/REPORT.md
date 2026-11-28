# Which dodge commands tip Pupper over: a MuJoCo study of the default walking policy

Date: 2026-11-27. Everything here ran in simulation on the Mac. Nothing touched the robot, and the monorepo was not edited. The scripts and the summary tables are in this folder. It is the companion to [../testing-2026-11-27.md](../testing-2026-11-27.md).

**Not included here:**
- `model/`, 14 MB. Before rerunning, make `model/` in this folder and copy in `default_policy.json` (md5 f43c95fa45970f4a978ad2801429a7e0) plus `pupper_v3_complete.position.xml` and its meshes from `pupperv3-monorepo`. Copy them; do not edit the monorepo.
- `ref_mjlab/`: the pupper-mjlab training files read for checking (`tasks/pupper/mdp/observations.py`, `tasks/pupper_gait/mdp/mixed_gaits.py`, `pupper_constants.py`, `pupper_env_cfg.py` and others). Read them in the pupper-mjlab repository on GitHub.
- The `results_*.json` raw runs and `venv/`. The venv needs `mujoco` and `numpy`.

## Short answer

- **Forward speed of 0.5 m/s or more is the main cause of tilt.** At |vx| ≥ 0.5 the controller switches the policy's gait reference from the trot to the fast gait (`gallop_speed` 0.5, `gallop_freq_mult` 2.5, so 3.3 steps a second). In sim, a forward dodge at vx 0.49 peaks at about 7°, and the same dodge at 0.51 peaks at 13–18°, with 70–95% of runs reaching 12°. The fast gait also does not travel further in the first 0.5 s: it covers 0.09–0.10 m, against 0.12 m at vx 0.45. Every shipped forward and forward-diagonal command (0.8 or 1.0) is in the fast gait.
- **Pure sideways dodges are the mildest, and the diagonals give the least lateral distance.** Sideways dodges at 0.5 m/s peak at about 6°. The shipped diagonals peak at about 12° and move only 0.04 m sideways by 0.5 s, against 0.09 m for pure sideways.
- **A longer ramp-out makes things worse, not better.** It keeps the fast gait running longer. A ramp-in barely helps under realistic conditions and costs distance. Capping at |vy| 0.4 and |vx| 0.8 changes almost nothing, because 0.8 is still in the fast gait.
- **No profile reaches 0.2 m sideways within 0.5 s.** The best is about 0.09–0.11 m.
- **The sim reproduces the real tilt pattern only once the actuation latency is raised to 40–50 ms** (or the contacts are made firm). At the 20–40 ms asked for, it under-predicts the tilt. It never reproduces the sideways fall.

## 1. How the harness works

Files (all in this folder):
- `pupper_policy_sim.py`: the policy, the controller and the physics.
- `dodge_sweep.py`: the dodge profiles, the domain randomisation and the sweep.
- `sanity.py`: standing and steady-walk checks.
- `vx_scan.py`: a speed scan across the fast-gait threshold.
- `trace_one.py`: prints one dodge's timeline.
- `check_gait_golden.py`: checks the gait reference against the golden data.
- `analyze.py` and `report_tables.py`: the tables.
- `results_*.json`: one row per run, 26,060 runs in all (not committed; regenerate with `dodge_sweep.py`).

### Policy file: `default_policy.json`, not either of the two candidates

`gemini_pupper.launch.py` builds `gemini_walk` from `config.yaml`'s `neural_controller_default.ros__parameters`. Its `model_path` is `$(find-pkg-share neural_controller)/launch/default_policy.json`.

Neither candidate file is the one that runs:
- **`policy_latest.json`** is byte-identical to `legacy_walk_policy.json` (the triangle button). It has a 720-wide input (20 × 36), action_scale 1.0 and kp 5.
- **`policy.json`** has a 66-wide input.

`default_policy.json` (md5 f43c95fa…) is identical to `pupper_gait_deploy/policies/default_policy.json`. It is a pupper-mjlab MixedGaitsJump export:
- contract v2, a 48-wide frame × 20 frames = 960 inputs;
- an MLP of 960-256-128-128-128-12 with ELU, linear output;
- action_scale 0.75, kp 5.5 (not 5), kd 0.25;
- command_clip vx ±1.5, vy ±0.5, wz ±2;
- heading_hold kp 1.5, clip 1.0;
- gait tables (trot, fast forward, fast backward, lift-in-place) and a jump slot, which is unused here.

I could not check which copy is on the robot without SSH. The harness uses an md5-checked local copy under `model/`.

### Observation layout, and where I checked it

The JSON's `obs_spec` gives the layout, newest frame first:

| Offset | Component | Size |
|---|---|---|
| 0 | `base_ang_vel` | 3 |
| 3 | `projected_gravity` | 3 |
| 6 | `command` | 3 |
| 9 | `desired_world_z` | 3 |
| 12 | `joint_pos_rel` | 12 |
| 24 | `last_action` | 12 |
| 36 | `gait_reference` | 12 |

I checked this against three sources:
- **The controller:** `neural_controller.cpp` `update()` and `obs_spec.hpp`. This covers how each frame is filled, the frame rotation by `std::rotate`, and that `last_action` is written into the new frame after the rotation.
- **The training code:** pupper-mjlab `tasks/pupper/mdp/observations.py` and `tasks/pupper_gait/mdp/mixed_gaits.py` (read-only from GitHub).
- **The gait reference itself:** a Python port of `compute_gait_reference_offset()` matches the mjlab-generated `pupper_gait_deploy/test/{gait,mixed_jump}_golden.json` to 6e-8 on all 48 cases.

`pupper_mjx_rl_training.py` and `_use_this.py` belong to the older MJX pipeline, the one that produced `policy_latest.json`. They do not describe this policy.

### The command path, ported from the controller

On each policy step, the raw (vx, vy, wz) goes through these stages in order:
1. Clip to `command_clip`.
2. Apply `max_vx_forward` and `max_vx_backward` (1.5).
3. Apply gemini_walk's `speed_cap` of 1.5 to vx and vy.
4. Apply the heading hold. While walking with no turn commanded, the yaw command becomes clip(1.5 · heading error, ±1.0), using the IMU yaw.

The result goes into the observation and into the gait reference. The phase clock is the time since fade-in.

The action becomes the motor target: target = clip(0.75 · a + default pose, joint limits). The emergency stop at `max_body_angle` 1.0 rad zeroes the commands and sets kd to 0.1, as the robot does.

### Physics: matched to how the policy was trained

The settings come from pupper-mjlab's `pupper_constants.py` and `pupper_env_cfg.py`, applied to the monorepo's `pupper_v3_complete.position.xml`:
- **Solver:** 4 ms Euler steps, pyramidal cone, impratio 10, 2 solver iterations and 5 line-search iterations, eulerdamp off.
- **Control rate:** decimation 5, so **the policy runs at its trained rate of 50 Hz**. The robot runs it at 520 Hz / 10 = 52 Hz. A 52 Hz check changed nothing.
- **Motors:** kp 5.5, kd 0.25, ±3 N m. Joint frictionloss 0.125.
- **Contacts:** solimp (0.015, 1, 0.031), condim 3. The feet have contact priority, so the foot's friction is the contact's friction.

Training-style corruption stays on, as it does in mjlab play: uniform observation noise, and a 50% chance that the IMU reading is one control step stale.

### Sanity checks (`sanity_train.txt`)

Standing for 8 s:
- tilt stays within 0.9°;
- the body sits 151 mm high and drifts 1 mm.

Steady commands, 3 s window:

| Command (vx, vy) | Measured body speed (m/s) |
|---|---|
| (0.3, 0) | 0.27 |
| (0.5, 0) | 0.51 |
| (1.0, 0) | 1.00 |
| (1.5, 0) | 1.53 |
| (−0.8, 0) | −0.79 |
| (0, ±0.5) | 0.36–0.44 sideways |
| (0.8, −0.4) | (0.84, −0.40) |

The policy walks sensibly. The firm-contact physics gives the same picture (`sanity_stiff.txt`).

### Dodge profiles and randomisation

**Profile.** The command steps to (vx, vy) 0.06 s after the decision. It holds for the burst, eases linearly to zero over the ramp-out, then stays at zero. Each run stands for a random 1.5–3 s first, so the gait phase differs from run to run.

**Default randomisation, per seed:**

| Parameter | Range |
|---|---|
| Floor friction | 0.4–0.8 |
| Actuation latency | 20–40 ms |
| kp, per motor | ×0.85–1.05 |
| Torque limit, per motor | 2.6–3.0 N m |
| kd | ×0.9–1.2 |
| Mass | ×0.97–1.12 |
| Centre of mass, along x | ±1 cm |

There are 20 seeds per case, and every variant uses the same 20 seeds.

**Metrics:**
- the peak tilt from vertical;
- the peak body roll, pitch and yaw rates;
- when the peak tilt comes, relative to the end of the burst;
- the share of runs over 30° or fallen (emergency stop, tilt over 60°, or body below 7 cm);
- the displacement 0.35 s and 0.5 s after the decision, measured sideways (positive in the dodge direction) and along the commanded direction.

## 2. How well the sim matches the real robot

| | Real, 50 armed dodges | Sim A: 20–40 ms (as asked) | Sim B: 40–50 ms | Sim C: firm contacts, 20–40 ms |
|---|---|---|---|---|
| Diagonals: median peak tilt / share ≥ 12° | 11° / 41% | 8.6° / 25% | 12.5° / 51% | 11.7° / 49% |
| Pure sideways: median / share ≥ 12° | 6.5° / 10% | 5.4° / 0% | 6.1° / 4% | 6.3° / 0% |
| When the peak comes | 0–0.3 s after the burst | +0.04 s (median) | +0.06 s | +0.03 s |
| Forward-diagonal pitch rate | 214 °/s (peak 25°) | 185–218 °/s | 221–274 °/s | about 190 °/s |
| Falls | 2 of 50 (one also hit by the ball) | 0 of 180 | 0 of 360 | 0 of 180 |

**What the sim reproduces:**
- diagonals tilt more than sideways dodges;
- the peak comes just after the burst ends, while the command eases to zero;
- the forward-diagonal pitch rate.

**What does not match:**
- The tilt levels match only in condition B or C. At the requested 20–40 ms the sim is about 3° too gentle on diagonals.
- The sim under-predicts the sideways tail: 0–5% of runs reach 12°, against 10% on the robot.
- **The sideways fall does not reproduce at all.** None of 5,080 simulated sideways dodges fell. That includes runs at 70 ms latency, friction 0.2, weak motors, 0.35 kg on the head, and second dodges. Their worst yaw rate was 170 °/s, and a typical one was 45–60 °/s, against the real −127 °/s mid-burst. The heading hold cannot explain that yaw rate: it never commands more than 1 rad/s (57 °/s), and turning it off changed nothing. Whatever caused the real sideways fall is outside the model. Possibilities include a foot catching, a motor dropping out, residual motion, or a bad IMU yaw. The recorded commanded wz and joint data for that dodge would tell.

**Latency scan, shipped profile** (`lat_table.txt`). This is the knob that moves the sim the most:

| Latency | Diagonals: median / share ≥ 12° / share over 30° | Sideways: share ≥ 12° |
|---|---|---|
| 16 ms (trained) | 5.4° / 2% / 0% | – |
| 30 ms | 8.4° / 24% / 0% | – |
| 40 ms | 11.3° / 44% / 0% | 2% |
| 50 ms | 14.0° / 59% / 0% | 5% |
| 60 ms | 16.8° / 78% / 5% | 10% |
| 70 ms | 19.7° / 89% / 14% | – |

From 60 ms up, falls appear 0–0.5 s after the command reaches zero, which is the real timing. With a second dodge at those latencies they reach 18–35%. The real robot sits between the 40 and 50 ms rows. That suggests its effective control delay is longer than the 15–16 ms the policy was trained with, or that its contacts are firmer than the training contacts. The sim cannot tell which.

## 3. Results

The full tables are in `tables.md` (generated by `report_tables.py`); the raw summaries are in `out_main.txt`, `cal_table.txt` and `vx_scan.txt`. Below, B is the condition that matches the robot: 40–50 ms, 160 runs per row (fewer where the profile applies to fewer commands). Rates are the median of each run's peak, in °/s. "Lateral" is the sideways displacement at 0.5 s; "along" is the displacement along the commanded direction at 0.5 s.

| Profile | Kind | Tilt (B) | ≥ 12° (B) | > 30° or fell (B) | Roll / pitch / yaw (B) | Lateral, m | Along, m | ≥ 12° in A / C |
|---|---|---|---|---|---|---|---|---|
| shipped (ramp-out 0.2) | diagonal | 12.5° | 51% | 0% | 160 / 199 / 60 | 0.04 | 0.13 | 25% / 49% |
| shipped | sideways | 6.1° | 4% | 0% | 93 / 92 / 47 | 0.09 | 0.09 | 0% / 0% |
| shipped | fore-aft | 14.0° | 62% | 0% | 130 / 249 / 57 | – | 0.13 | 27% / 57% |
| ramp-out 0.4 | diagonal | 13.8° | 59% | 0% | 163 / 218 / 71 | 0.04 | 0.13 | 30% / 49% |
| ramp-out 0.6 | diagonal | 14.1° | 62% | 1% | 168 / 225 / 73 | 0.04 | 0.13 | 30% / 50% |
| ramp-out 0.6 | fore-aft | 16.7° | 72% | 0% | 146 / 271 / 64 | – | 0.13 | 37% / 62% |
| ramp-in 0.08 | diagonal | 11.8° | 49% | 1% | 159 / 189 / 61 | 0.04 | 0.11 | 18% / 44% |
| ramp-in 0.10 | diagonal | 11.0° | 42% | 1% | 161 / 181 / 64 | 0.03 | 0.10 | 10% / 36% |
| ramp-in 0.10 | fore-aft | 12.6° | 55% | 0% | 150 / 219 / 62 | – | 0.10 | 12% / 38% |
| ramp-in 0.10 | sideways | 5.9° | 1% | 0% | 90 / 90 / 47 | 0.07 | 0.07 | 0% / 2% |
| diagonal → y only (0, ±0.5) | diagonal | **6.5°** | **4%** | 0% | 99 / 93 / 51 | **0.09** | 0.09 | 0% / 0% |
| diagonal → x only (1.0 / −0.8) | diagonal | 13.1° | 54% | 1% | 156 / 217 / 58 | 0 | 0.14 | 25% / 52% |
| diagonal scale 0.6 | diagonal | 10.7° | 39% | 0% | 131 / 165 / 54 | 0.04 | 0.11 | 15% / 31% |
| caps vy 0.4, vx 0.8 | diagonal | 12.5° | 51% | 0% | unchanged | 0.04 | 0.13 | 25% / 49% |
| caps vy 0.4, vx 0.8 | sideways | 5.7° | 5% | 0% | 71 / 76 / 43 | 0.07 | 0.07 | 0% / 0% |
| caps vy 0.4, vx 0.8 | fore-aft | 13.9° | 62% | 0% | 135 / 249 / 58 | – | 0.13 | 20% / 53% |
| **vx cap 0.45** (trot only; extra) | diagonal | **7.0°** | **8%** | 0% | 117 / 109 / 53 | 0.06 | 0.12 | – / 1% |
| **vx cap 0.45** (extra) | fore-aft | **6.9°** | **5%** | 0% | 101 / 96 / 48 | – | 0.11 | – / 0% |
| second dodge, opposite, 1.5–2.5 s later | diagonal | 11.9° | 49% | 1% | 163 / 199 / 64 | 0.05 | 0.13 | 19% / 50% |
| second dodge, opposite, 1.5–2.5 s later | fore-aft | 14.2° | 62% | 1% | 147 / 255 / 57 | – | 0.12 | 38% / 62% |
| second dodge, opposite, 0.6–1.2 s later (extra) | diagonal | 11.9° | 49% | 4% | – | – | – | 19% / 49% |
| second dodge, ramp-out 0.4 | fore-aft | 14.7° | 62% | 3% | – | – | – | 38% / 60% |

In condition A (20–40 ms), no profile went over 30° or fell in any run.

**Shipped profile, per real command, condition B:**

| Command (vx, vy, burst) | Tilt | ≥ 12° | Along at 0.5 s |
|---|---|---|---|
| (0.8, −0.4) 0.34 s | 18.7° | 98% | 0.11 m |
| (0.8, +0.4) 0.34 s | 13.4° | 57% | 0.10 m |
| (−0.64, ∓0.4) 0.47 s | 8–11° | 10–40% | 0.15 m |
| (0, ±0.5) 0.36–0.40 s | 5.5–7.3° | 0–8% | 0.09 m |
| (0.8, 0) 0.40 s | 16.1° | 85% | 0.10 m |
| (1.0, 0) 0.37 s | 15.5° | 85% | 0.10 m |
| (−0.8, 0) 0.31 s | 8.9° | 18% | 0.17 m |

The forward-right diagonal is the worst command. The real 214 °/s pitch event was that command.

**Speed scan across the fast-gait threshold** (`vx_scan.txt`; burst 0.37 s, 20 seeds, 40 ms, 50 ms and firm-contact 45 ms):

| Command | Median tilt | ≥ 12° | Along at 0.5 s |
|---|---|---|---|
| vx +0.45 | 6.0–7.6° | 5% | 0.11–0.12 m |
| vx +0.49 | 6.6–7.3° | 5% | 0.12 m |
| vx +0.51 | 12.9–17.7° | 70–95% | 0.09–0.10 m |
| vx +1.0 | 14.5–20.8° | 80–90% | 0.07–0.10 m |
| vx −0.45 | 6.9–8.8° | 0–10% | 0.10–0.12 m |
| vx −0.8 | 7.9–11.6° | 10–40% | 0.16–0.20 m |
| vy 0.3 / 0.4 / 0.5 | 4–5° / 6° / 7° | – | 0.06 / 0.08 / 0.09 m sideways |

The backward fast gait is far gentler than the forward one, and it does go further.

## 4. Recommendations

These are for the dodge node, in order of confidence.

1. **Cap forward vx below 0.5 m/s, for example at 0.45, in every dodge, diagonals included.** This stays out of the policy's forward fast gait. It is the largest effect found, it appears in every condition, and it does not cost distance in the first 0.5 s. Note that the robot's `gallop_speed` comes from the policy JSON, so re-check this threshold if the policy changes.
2. **Replace diagonals with pure sideways (0, ±0.5).** This roughly halves the tilt (6.5° against 12.5°) and the 12° share drops from about 50% to 4%. It also doubles the lateral distance at 0.5 s (0.09 m against 0.04 m). If some forward or backward component is needed, use |vx| ≤ 0.45: (±0.45, ±0.4) gave 6–8° in the scan.
3. **Keep ramp-out at 0.2 s.** 0.4 s and 0.6 s were worse in every condition. Skip the ramp-in: it helps a little at low latency, nothing at realistic latency, and costs 20–30% of the distance at 0.35 s.
4. **The vy 0.4 / vx 0.8 caps are not worth it as specified.** They do not touch the diagonals, and vx 0.8 is still the fast gait. vy 0.4 makes sideways dodges a little gentler but loses 20% of the distance.
5. **Backward (−0.8, 0) is the most effective dodge per tilt.** It moved 0.17 m along its direction by 0.5 s, at 9° median tilt. If the ball geometry allows backing away, prefer it. −0.45 is the conservative version at 0.10–0.12 m.
6. **Leave at least 1.5 s between dodges.** Opposite second dodges 1.5–2.5 s apart added little. At 0.6–1.2 s, falls appeared in the realistic conditions (2–12% of fore-aft and diagonal runs).
7. **Expect no more than about 0.1 m sideways by 0.5 s after the decision.** Steady sideways speed at the 0.5 m/s limit is 0.36–0.44 m/s, and getting going takes about 0.15 s. The 0.2 m requirement needs either an earlier decision (about 0.7 s) or backward motion. No command profile fixes this with this policy.
8. **Worth measuring:** the real command-to-motor latency on the robot. The sim matches the robot only at 40–50 ms, and falls start in sim at 60 ms. If the real figure is near that, cutting it would add margin: the `ros2_control_node` runs at normal, not real-time, priority.

## 5. How much to trust this

**Trust the relative ordering more than the absolute numbers.** The ordering is consistent across seven physics and latency conditions:
- the fast gait is bad;
- pure sideways is the gentlest;
- a longer ramp-out does not help.

The jump in tilt at vx = 0.5 is a property of the deployed controller and policy, not of the contact model. It appears in every condition, and the gait tables and switch logic are verified against the golden data.

**The absolute tilt levels depend on latency and contact stiffness, which I calibrated against the real medians.** With the default randomisation the sim is too gentle. Treat the condition B and C numbers as the estimate, give or take a few degrees.

**Falls are not predicted.** The sim produced no falls in the plausible conditions, and none at all for sideways dodges, while the robot fell twice in 50. So a 0% fall rate here does not mean a profile is safe. The two real falls involve things the model lacks: a ball hit, and an unexplained yaw spin.

**What is not modelled:**
- the real floor and the foot rubber;
- backlash and motor heating;
- the robot's camera mass. A 0.35 kg head-mass test lowered tilt slightly, so it is not the missing factor.
- IMU and encoder error beyond the training noise;
- the gemini_walk activation and fade-in history;
- per-robot `kp_scale` trims, which were not available on the Mac.

**The displacements** come from a body that is still accelerating. They are relative, consistent across conditions, and roughly right in steady state (speeds tracked to within about 10%).

## Rerunning

```bash
cd projectile-dodging/reports/falls-mujoco   # after copying model/ in and making venv/
venv/bin/python sanity.py                       # rate, standing, steady walks
venv/bin/python check_gait_golden.py            # gait reference vs mjlab goldens
venv/bin/python dodge_sweep.py --cond dr lat40 lat50 stiff --out my.json   # about 6 min on 8 cores
venv/bin/python analyze.py my.json
venv/bin/python trace_one.py 0.8 -0.4 0.34 0.2 3   # one dodge's timeline (vx vy burst ramp seed)
```
