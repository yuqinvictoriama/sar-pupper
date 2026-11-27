# Terrain agility

Pupper sees an obstacle in its front fisheye camera, measures it, and does one of three things:
**steps over** a low flat bar, **scales across** a low flat slab (onto it, along its top, off
the far side), or **walks around** it along the shortest clear line. `plan.md` is the design;
this file is the reference. **To put it on the robot, follow `deploy.md`**, every step in order.

**Status.** Everything here runs and passes its tests off the robot, including whole crossings
in a physics simulation of the Pupper v3 model with the real leg shapes, driven by the same
node code that runs on the robot. **None of it has been on the real robot yet.** The bring-up
ladder in `deploy.md` is how to find out safely what carries over; expect to tune numbers in
`config/terrain.yaml` on the way. Section 10 lists exactly which claims rest on simulation.

---

## 1. The three behaviours

| | Step over | Scale across | Walk around |
|---|---|---|---|
| Obstacle | low flat bar | low flat slab | everything else |
| Height read as | 10 to 27 mm (less at some depths) | 10 to 27 mm | – |
| Depth (the way Pupper walks) | 60 to 102 mm | 200 to 620 mm | – |
| Width (across the path) | 400 mm or more | 400 mm or more | – |
| How | each foot uses the top once as a stepping stone; stored footstep patterns | footsteps planned on the measured slab: up, along the top, down | tangent steering, no stop |
| Planner | `plan_crossing` | `plan_scale_across` | `terrain/steering.py` |

- **No colours are taught.** The floor's colour is learned; obstacles are told apart by shape
  (`terrain/scan.py`, `terrain/gauge.py`, `terrain/decision.py`).
- **Both crossings put feet on the obstacle**, so its height must be read to a few millimetres.
  That needs a side face at its foot that looks different from its top and from the floor in
  front of it (the raw edge of a white-topped board does). With no such edge the height is
  "not read" and Pupper walks around.
- **A no-touch step-over is only possible for very small things**: about 10 mm tall and 20 mm
  deep (`plan_step_over`, kept for that purpose and not used by the nodes). A 50 mm round bar
  is walked around. `deploy.md` 1c has the reasoning.
- Read under 10 mm: a mark on the floor, walked over with the normal gait.

Compared with v2 of the plan: v2 knew one crossable obstacle, by its colours (blue sides,
white top), and walked around everything else low. Now the measurement needs no particular
colours, and a second crossing (the slab) has been added.

---

## 2. What is in this folder

```
terrain/      the method, pure Python (no ROS): kinematics, balance, crawl planner, trajectory
              player, lens model, floor colour, scan, obstacle gauge, decision, steering, state machine
robot/        ROS 2 nodes and launch files (the only code that imports rclpy)
tools/        calibration, checks and test drivers to run on the robot
config/       terrain.yaml (every parameter), crawl_templates.json (stored footstep patterns)
sim/          physics simulation of crossings, a fisheye renderer, an avoidance simulator
tests/        pytest suite (runs on a laptop)
plan.md       the design
```

Three processes run on the robot:

| Process | Does |
|---|---|
| `robot/terrain_node.py` | Camera → obstacles → decides → steers on `/terrain_cmd_vel`. Asks for a crossing when the obstacle qualifies. Silent when nothing is in the way |
| `robot/stepover_node.py` | Plans a crossing, takes the legs from the walking controller, plays the crawl at 120 Hz, gives the legs back |
| `robot/estop_guard.py` | Makes the legs limp on the gamepad e-stop, or if the step-over node dies while holding them |

---

## 3. Install on the robot

The robot needs what the course already installed (ROS 2 Jazzy, the pupperv3 monorepo built,
`pupper_gait_deploy` installed, OpenCV, NumPy, PyYAML). Nothing is compiled; there is no
`colcon build`. MuJoCo is **not** needed on the robot.

1. Put this folder on the Pi inside your repo, e.g. `~/sar-pupper/terrain-agility`
   (`git pull` on the Pi, or from your Mac: `rsync -av terrain-agility/ pi@<pupper>:~/sar-pupper/terrain-agility/`).
2. In every terminal on the Pi:
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/pupperv3-monorepo/ros2_ws/install/local_setup.bash
   export ROS_LOCALHOST_ONLY=1          # the course scripts set this; all terminals must match
   cd ~/sar-pupper/terrain-agility
   ```
3. `python3 tools/check_robot.py --offline` should print no FAIL (two WARNs about calibration are expected).

### Wiring into the mux

Terrain agility steers by publishing on `/terrain_cmd_vel`, which `cmd_vel_mux` must list
**below the joystick and the dodge reflex, above every "driver"** (Gemini, survivor approach,
planner, person follower). `robot/terrain_mux.yaml` has the list.

- **Without Gemini** (bring-up): `ros2 launch robot/standalone.launch.py` starts the course
  stack, the joystick through the mux with that list, the fixed white balance, and terrain agility.
- **With Gemini**: copy the `inputs:` list from `robot/terrain_mux.yaml` into the `cmd_vel_mux`
  section of `~/gemini_pupper/robot/launch/gemini_pupper.yaml`, start Gemini as usual
  (`robot/start.sh`), then `ros2 launch robot/terrain.launch.py`.
- **From your own `sar.launch.py`**: start the mux with `robot/terrain_mux.yaml` and include
  `robot/terrain.launch.py` (the snippet is in that file's docstring).

`python3 tools/check_robot.py` (with everything running, robot standing) then checks the mux
order, the controllers, the sensor rates and that the legs stand still enough to start a crossing.

---

## 4. Calibrate, and 5. Bring-up

Both are in `deploy.md` (parts 4 and 5), step by step with what to expect and when to stop.
In outline:

- **Calibrate:** floor colour (`tools/learn_colors.py floor`); a look at what Pupper makes of
  each obstacle, nothing moving (`tools/learn_colors.py check`); camera position from the slab
  at three ruler distances (`tools/calibrate_camera.py`); payload masses; walking speed.
- **Bring-up:** check → plan without moving → stand at crossing height (tests the hand-over) →
  e-stop test → crawl on clear floor → scale across with the slab placed by ruler → walk around
  the carton → the whole chain from the camera → the round bar (measured, refused, walked around).

Each tool explains itself with `-h`, and the camera tools take `--frames <folder>` to work from
saved frames (`tools/record_frames.py`) instead of the live camera.

---

## 6. How it behaves

- **Clear floor:** it publishes nothing. Whoever is driving drives.
- **Something certainly tall in the lane within 1.2 m** (taller than the camera, or hiding so
  much floor that it must be over 60 mm tall even if it were 0.7 m deep): takes over, steers
  along the tangent past the nearer end (walking and turning together, never stopping),
  returns to the heading the driver had, lets go. Simulated detours are 0 to 4 % longer than
  the shortest clear path.
- **Something that may be low, within 1 m:** walks to 36 cm, stops, squares up and centres with
  short turn/sidestep pulses, measures height, depth, distance, skew and width over ten frames,
  checks the strip beyond it is clear floor, and decides (`terrain/decision.py`): step over,
  scale across, or walk around. `/terrain/event` gives the measurement and, when it walks
  around, which check failed.
- **Crossing:** planning (a few seconds; the same obstacle again is quicker), then the crawl:
  about 57 s for a 19 × 89 mm bar, about 70 s for a 15 × 380 mm slab, then the walking
  controller's own 4 s start-up. The dodge reflex cannot act during a crossing: a crossing is
  not started while a ball is tracked.
- **Boxed in, robot tilted, camera frames stop:** it stands still and reports (`HOLD`).
- **Driver stops asking for motion:** it lets go within a second.

Switch it off and on: `ros2 topic pub -1 /terrain/enable std_msgs/Bool "data: false"`.
Modes: `ros2 topic pub -1 /terrain/mode std_msgs/String "data: avoid"` (`full` crosses what can
be crossed, `avoid` walks around everything, `report` only looks, `off`).
Clear a `HOLD` after a failed crossing: `ros2 topic pub -1 /terrain/reset std_msgs/Empty`.

### Safety behaviour of the crossing

- Nothing moves unless: one walking controller is active, the robot is level (under 8°), every
  joint is within 0.30 rad of the standing pose and still, and a complete plan exists in which
  every sample is inside the joint limits, clear of the mic bars, balanced and under 2.2 N·m.
- Gains are the walking controllers' own start-up gains (kp 7.5, kd 0.25): nothing is raised.
- During the crossing: tilt over 20°, a planted leg more than 0.15 rad off, a swinging foot
  blocked (it is first backed out the way it came), stale joint or IMU data → joints go soft
  and the robot settles; the walking controller is **not** restarted automatically.
- The gamepad e-stop makes the legs limp immediately (step-over node and, independently, the guard).
- A foot stepping down stops when it feels the surface early.
- No foot is set down within 30 mm of a slab's edge, or on the floor within 30 mm of its faces.

---

## 7. Topics

| Topic | Type | Dir | |
|---|---|---|---|
| `/terrain_cmd_vel` | `Twist` | out | into `cmd_vel_mux`; only while handling an obstacle |
| `/terrain/state` | `String` (JSON) | out | `state`: `CRUISE, AVOID, APPROACH, GAUGE, ALIGN, PLAN, STEP_OVER, SCALE_ACROSS, HOLD, DISABLED, REPORT`; `mode`; `kind` (what was decided); `box` (the measurement) |
| `/terrain/event` | `String` | out | decisions and reasons, in words (feed these to Gemini as mission events) |
| `/terrain/obstacles` | `String` (JSON) | out | obstacles in view: bearing, range, width, `height` (the most it could be), `min_height` (the least), `extent`, `tall` (certainly), `is_box` (may be low: worth measuring) |
| `/terrain/scan` | `String` (JSON) | out | range to the nearest obstacle every 5° (free space, for the dodge node and the planner) |
| `/terrain/debug/compressed` | `CompressedImage` | out | the classified view, rendered only while subscribed |
| `/terrain/step_over`, `/terrain/crossing_status` | `String` (JSON) | | requests to / reports from the step-over node (`method`: `board` steps over, `slab` scales across; `advance`: metres the body moved) |
| `/terrain/enable`, `/terrain/reset`, `/terrain/mode` | `Bool`, `Empty`, `String` | in | |
| `/dodge/event`, `/dodge/state` | `String` | in | a dodge drops all measurements; no crossing starts while a ball is tracked |
| intent topics (`fsm.intent_topics`) | `Twist` | in | the drivers' own commands: where the robot was going |

---

## 8. Troubleshooting

| Symptom | Likely cause | Do |
|---|---|---|
| `refused: legs are not in the standing pose` | the walking policy shuffles or leans at zero command | `check_robot.py` prints how far and how much it moves; use a policy that stands still, or raise `READY_JOINT_ERROR` in `robot/stepover_node.py` a little |
| `no_plan` | read taller or deeper than can be crossed, or too far askew | `/terrain/event` shows the reading; `tools/plan_box.py --height .. --depth ..` reproduces it off the robot |
| "its height could not be read" | no side face seen at the obstacle's foot: its edge is the colour of the floor or of its top | `tools/learn_colors.py check`; a strip of tape in another colour along the edge facing Pupper |
| A foot comes down on a slab's far edge | Pupper covers less ground than it counts, or the depth read short | `deploy.md` rung 6 (`crawl.advance_scale`); redo the camera position |
| Lurch or sag as the legs change hands | controller switch slower than expected | the legs are only damped during the switch; if the body drops more than about 5 mm raise `HandoverConfig.pre_kd` (max 0.8) |
| Body low, feet drag in the crawl | real joints sag more than modelled | re-weigh `extra_masses`; then raise `crawl.sag_gain` to 1.2 to 1.5 |
| `swing blocked ... backing out` on clear floor | joints lag the plan more than in simulation | `crawl.time_scale: 1.3`, or `player.swing_error_limit: 0.16` |
| A rear foot brushes the box's far face on the way down | the robot ends a little short after 30 swings, or distance/depth read short | redo step 4 of the calibration; then add `final_extra: 0.025` under `crawl:` in `terrain.yaml` (the rear feet finish 10 mm farther on) and check `tools/plan_box.py` still plans your box |
| Turns back and forth while squaring up to the box, then goes around | the turn pulses overshoot on this walking policy | lower `fsm.align_min_pulse` (0.2) or `fsm.align_yaw_rate` (0.3) |
| Goes around something it should cross | read the reason in `/terrain/event` | height near the limit: a lower obstacle; "no clear floor beyond": clear it; "depth ... outside": between a bar (60 to 102 mm) and a slab (200 to 620 mm) nothing is built; "wide": 400 mm across the path at least |
| Walks into / stops for things that are not there | floor colour model | `learn_colors.py check`, then `floor --add` on the patch that reads red |
| Steers but the robot does not follow | mux order | `check_robot.py`; `ros2 topic echo /cmd_vel_mux/active_source` |
| Terrain controllers left ACTIVE after a crash | | the guard releases them within a second; `python3 robot/spawn_controllers.py` reports it |

---

## 9. Working on it off the robot

```bash
pip install numpy opencv-python pyyaml pytest mujoco     # laptop only
python3 -m pytest tests -q                               # about 12 minutes
python3 sim/crossing_sim.py --height 0.015 --depth 0.38 --near 0.36 --view     # watch the slab being scaled
python3 sim/crossing_sim.py --height 0.019 --depth 0.089 --near 0.36 --view    # watch a bar being stepped over
python3 sim/avoid_sim.py                                 # a walk-around, with the path length
python3 tools/plan_box.py --height 0.015 --depth 0.38    # print the footsteps for an obstacle
python3 sim/build_templates.py                           # rebuild the stored footstep patterns (30 min)
```

The physics tests need the Pupper model: clone `G-Levine/pupper_v3_description` next to this
repo or set `PUPPER_XML` to `.../description/mujoco_xml/pupper_v3_complete.position.xml`.
Rebuild the patterns after changing anything in `CrawlConfig` that alters the gait (clearances,
stance, margins, payload by more than about 100 g). The patterns are for the low bar only; a
slab's footsteps are worked out each time.

---

## 10. What is verified and what is not

Verified in simulation (MuJoCo, Pupper v3 model with measured leg envelopes, stiff foot
contacts, ball feet that roll, camera plate and mics as extra mass):

- **Stepping over a low bar** (unchanged from v2). Planning: every bar from 19 to 31 mm tall and
  70 to 102 mm deep plans from every stopping distance between 30 and 42 cm (shallower ones
  only when low). 30 of 30 crossings with exact measurements and 40 of 40 with errors of ±3 mm
  in height, ±8 mm in distance and depth, ±1.5° in skew: no abort, no fall, tilt at most 6°;
  in a few runs a rear foot brushed the bar's far face (under 8 N).
- **Scaling across a slab.** Planning: all 225 slabs of 10 to 30 mm × 200 to 600 mm, met from
  32, 36 and 40 cm, square and 4° askew either way, in 3 s median and 31 s worst on a laptop.
  40 of 40 crossings with exact measurements (10 to 27 mm × 250 to 600 mm, up to 4° askew, slab
  fixed or 1 to 2.5 kg, payload 0.6 to 1.4 times what the planner assumes): tilt at most 3.3°,
  no leg touched the slab except a foot on its top, the slab never moved. 60 of 60 with errors
  of ±10 mm in distance, ±3 mm in height, ±15 mm in depth and ±1.5° in skew: tilt at most 4.7°,
  one light touch of a foot on an edge (4.9 N), ending within 15 mm of where the plan said.
- **A no-touch step-over** plans only for obstacles about 10 mm tall (10 × 10 and 10 × 20 mm);
  none at 15 mm or above with the standard clearances. Planned only; not run in physics.
- **Walking around:** paths 0 to 4 % longer than the shortest clear one, never closer than
  0.23 m, the short side chosen; 420 of 420 runs past a 0.40 × 0.25 m obstacle without a stop
  (13 of them used to stop beside it; fixed in `terrain/steering.py`).
- Hand-over from the walking controller and back: the body drops 2 to 3 mm.
- E-stop, stale data, failed controller switch, refusals: tested through the real node code.
- **Perception on rendered fisheye images through JPEG**, on a grey and a wood-coloured floor:
  for flat-topped obstacles with a visible edge, height ±4 mm, distance ±8 mm, depth ±15 mm;
  the course slab scaled across, a low bar stepped over, and the round bar, the carton (lying
  and on edge), a 40 mm slab, a 70 mm block that hides as much floor as the slab, the slab
  turned end-on and a slab with a floor-coloured edge all walked around, each for the stated
  reason. Camera calibration recovers height within 5 mm and pitch within 0.6°.

Not verified, because it needs the robot:

- That the real motors track the crawl as the simulated ones do (joint sag, friction, backlash),
  and how much ground a slab crossing really covers (`crawl.advance_scale`).
- That the walking policy stands still enough at zero command for a crossing to start.
- Foot grip on the real floor and on the obstacle's top; whether a slab stays put.
- The controller switch timing on the Pi, and planning time on the Pi.
- Whether an obstacle's edge shows on the real floor under the real lights; the lens model;
  the IMU's levelling of the picture while walking. **The rendered images have flat colours
  and even light. Real frames will be harder, and this is the part I trust least.**
- The real walking policy's response to the steering commands (the avoidance simulator uses a
  simple model of it).

Known limits: one obstacle at a time, on flat floor; nothing may stand within 35 cm beyond a
crossing; an obstacle whose height cannot be read is walked around, as is a flat mark the size
of a slab; a round bar 55 to 75 mm across can look like a low flat bar (a depth-to-height check
refuses it; tested on the 50 mm bar only); depths between 102 and 200 mm have no crossing;
something tall directly behind a slab is only seen once the slab has been measured.
