# Terrain agility

Pupper sees an obstacle in its front fisheye camera and either **steps over it** with a slow,
planned, statically balanced crawl (the low two-colour box) or **walks around it** along the
shortest clear line (anything else). `plan.md` is the design; this file is the reference.
**To put it on the robot, follow `deploy.md`**: materials with their sizes, then every step in order.

**Status.** Everything here runs and passes its tests off the robot: 96 tests, including whole
crossings in a physics simulation of the Pupper v3 model with the real leg shapes, driven by the
same node code that runs on the robot. **None of it has been on the real robot yet.** The
bring-up ladder below is how to find out safely what carries over; expect to tune numbers in
`config/terrain.yaml` on the way. Section "What is verified and what is not" lists exactly which
claims rest on simulation.

---

## 1. The box to get

| | Value | Why |
|---|---|---|
| Height | **19 mm** (¾ in). Anything from 19 to 25 mm works | The gait is verified for boxes read as up to 27 mm; it refuses taller ones and walks around |
| Depth (the way Pupper walks) | **89 mm** (3½ in). Anything from 76 to 102 mm works | Pupper steps *onto* the box top. A foot set down there rolls forward 15 to 25 mm as the body passes over it and must stay 12 mm clear of both edges, so a deeper top forgives more error in where the box is |
| Width (across the path) | **500 to 600 mm** | Room for both feet with margin, and too wide to be worth walking around |
| Weight / grip | 250 g or more, or taped down; non-slip underside | A foot stands on it |
| Colours | Sides one strong colour (blue painter's tape is ideal), top a different one (white). Matte, both unlike the floor | This is how the camera tells a low box from a mark on the floor, and reads its height and depth |

**A length of "1×4" timber (really 19 × 89 mm) cut to 55 cm**, with blue tape round the sides and
white tape or paint on top, is exactly right.

- A "1×3" (19 × 64 mm) is too shallow to rely on: at that depth only boxes read at 19 mm or
  less are accepted, so a reading of 20 mm sends Pupper around it.
- A "2×3" or "2×4" (38 mm tall) is too tall.

The tall obstacle can be anything at least 80 mm tall that is not the floor's colour: a
cardboard box is fine.

Compared with the first version of the plan: the box is lower (was 30 to 38 mm) and deeper
(was 40 to 65 mm). Measuring the real leg shapes showed the thigh and knee housings are too
bulky for a foot to clear a taller box cleanly, and that a front foot cannot reach past the box
while the robot stays balanced, so each foot steps up onto the box and down again.

---

## 2. What is in this folder

```
terrain/      the method, pure Python (no ROS): kinematics, balance, crawl planner, trajectory
              player, lens model, floor/box colours, scan, box gauge, decision, steering, state machine
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
| `robot/terrain_node.py` | Camera → obstacles → decides → steers on `/terrain_cmd_vel`. Asks for a crossing when the box qualifies. Silent when nothing is in the way |
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

## 4. Calibrate (once, about 20 minutes)

Robot **standing under its walking controller** on the floor you will run on, in the lighting
you will run in. The white balance must be the fixed one (both launches set it). Each tool
explains itself with `-h`, and each takes `--frames <folder>` to work from saved frames
(`tools/record_frames.py`) instead of the live camera.

1. **Floor colour.** Clear floor for 1 m ahead and to the sides.
   `python3 tools/learn_colors.py floor` (add `--add` on other patches if the floor varies).
2. **Box colours.** Box square across the path, about 40 cm ahead of the front feet, nothing else within 1 m.
   `python3 tools/learn_colors.py box`
3. **Look.** `python3 tools/learn_colors.py check` writes `look.jpg`: the camera's view levelled
   and unrolled, and below it the same view by class (grey floor, red obstacle, blue box side,
   white box top). Copy it to your Mac and look at it. Floor must be grey, the box blue under
   white. Find where the robot's own chin/feet enter the picture at the bottom and set
   `camera.near_depression_deg` in `terrain.yaml` a few degrees above that (the magenta line
   must be clear of the robot).
4. **Camera position.** Put the box at three or four distances between 20 and 55 cm, measured
   with a ruler from the **front of the front feet** to the box's near face:
   ```bash
   python3 tools/calibrate_camera.py add 0.25
   python3 tools/calibrate_camera.py add 0.35
   python3 tools/calibrate_camera.py add 0.50
   python3 tools/calibrate_camera.py solve --box-height 0.019 --write
   ```
   A fit residual under about 0.35° is good. Check with the box somewhere new:
   `learn_colors.py check` should read its height within 3 mm and its distance within 1 cm.
   **The crossing depends on that distance**: errors near 1 cm make a rear foot brush the box.
5. **Payload.** Weigh the camera plate with cameras, and a mic, and correct `crawl.extra_masses`
   in `terrain.yaml` (mass in kg and position in metres from the body centre). The balance
   calculation uses it.
6. **Walking speed** (for going around): `python3 tools/walk_forward.py --seconds 5` on clear
   floor, measure the distance, set `steer.speed_scale` to distance ÷ 1.5 m.

---

## 5. Bring-up, one rung at a time

**Someone is at the e-stop (gamepad right-stick press) for every step that moves.** For the
first leg motions, hold the robot by its handle just clear of the floor, or stand it on foam.
Set `crawl.time_scale: 1.5` in `terrain.yaml` for the first runs (everything half as fast again).

Start with `ros2 launch robot/standalone.launch.py avoid:=false` (step-over side only), stand
the robot up (joystick X), then in a second terminal:

| # | Command | Expect | If not |
|---|---|---|---|
| 1 | `python3 tools/check_robot.py` | no FAIL | it says what to do |
| 2 | `python3 tools/step_over.py cross --height 0.019 --depth 0.089 --near 0.36 --dry-run` | a plan in a few seconds, nothing moves | note the planning time on the Pi |
| 3 | `python3 tools/step_over.py stand` | body settles a millimetre or two to the crossing height, stands 3 s, returns, walking controller takes over again. No lurch at the start | a lurch or sag at the start: see "hand-over" in Troubleshooting |
| 4 | Press the e-stop during a `stand --hold 8` | legs go limp at once, robot sits down | **do not go on until this works** |
| 5 | `python3 tools/step_over.py crawl --distance 0.08` | four slow steps, 8 cm forward, body level, no foot dragging | feet drag or it tips: payload masses, or `crawl.sag_gain` (Troubleshooting) |
| 6 | Box square, near face 27 cm ahead of the front feet (= 0.36 m from the body centre): `python3 tools/step_over.py cross --height 0.019 --depth 0.089 --near 0.36` | about a minute: crawls up to the box, each foot onto the top and down the far side, stands, walking controller back | watch which foot touches what; see Troubleshooting |
| 7 | Relaunch with `avoid:=true`. Tall box 1.5 m ahead: `python3 tools/walk_forward.py` | curves round it without stopping, about 20 cm clear, straightens onto the old heading | `ros2 topic echo /terrain/event` says what it decided and why |
| 8 | The step-over box 1.5 m ahead: `python3 tools/walk_forward.py --seconds 20` | walks up, stops about 36 cm away, squares up, measures (`/terrain/event` prints height × depth), crosses | if it goes around instead, the event gives the reason (height, depth, skew, width, landing strip...) |
| 9 | Both launched with Gemini; ask it to walk forward into each obstacle | same as 7 and 8 | |

Return `time_scale` to 1.0 when 6 and 8 are reliable.

---

## 6. How it behaves

- **Clear floor:** it publishes nothing. Whoever is driving drives.
- **Tall or unknown obstacle in the lane within 1.2 m:** takes over, steers along the tangent
  past the nearer end (walking and turning together, never stopping), returns to the heading
  the driver had, lets go. Simulated detours are 1 to 4 % longer than the shortest clear path.
- **The two-colour box within 1 m:** walks to 36 cm, stops, squares up and centres with short
  turn/sidestep pulses, measures height, depth, distance and skew over ten frames, checks the
  strip beyond the box is clear floor, and decides. If every check passes it asks the step-over
  node for a crossing; otherwise it walks around and `/terrain/event` says which check failed.
- **Crossing:** planning (2 to 20 s on a desktop core; the pattern that worked last time is
  tried first, so repeats are quick; expect about double on the Pi), then 45 to 60 s of crawl
  (about 20 s of counted steps up to the box, the rest over it), then the walking controller's
  own 4 s start-up. The dodge reflex cannot act during a crossing: a crossing is not started
  while a ball is tracked.
- **Boxed in, robot tilted, camera frames stop:** it stands still and reports (`HOLD`).
- **Driver stops asking for motion:** it lets go within a second.

Switch it off and on: `ros2 topic pub -1 /terrain/enable std_msgs/Bool "data: false"`.
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

---

## 7. Topics

| Topic | Type | Dir | |
|---|---|---|---|
| `/terrain_cmd_vel` | `Twist` | out | into `cmd_vel_mux`; only while handling an obstacle |
| `/terrain/state` | `String` (JSON) | out | `CRUISE, AVOID, APPROACH, GAUGE, ALIGN, PLAN, STEP_OVER, HOLD, DISABLED` |
| `/terrain/event` | `String` | out | decisions and reasons, in words (feed these to Gemini as mission events) |
| `/terrain/obstacles` | `String` (JSON) | out | obstacles in view: bearing, range, width, height, tall, is_box |
| `/terrain/scan` | `String` (JSON) | out | range to the nearest obstacle every 5° (free space, for the dodge node and the planner) |
| `/terrain/debug/compressed` | `CompressedImage` | out | the classified view, rendered only while subscribed |
| `/terrain/step_over`, `/terrain/crossing_status` | `String` (JSON) | | requests to / reports from the step-over node |
| `/terrain/enable`, `/terrain/reset` | `Bool`, `Empty` | in | |
| `/dodge/event`, `/dodge/state` | `String` | in | a dodge drops all measurements; no crossing starts while a ball is tracked |
| intent topics (`fsm.intent_topics`) | `Twist` | in | the drivers' own commands: where the robot was going |

---

## 8. Troubleshooting

| Symptom | Likely cause | Do |
|---|---|---|
| `refused: legs are not in the standing pose` | the walking policy shuffles or leans at zero command | `check_robot.py` prints how far and how much it moves; use a policy that stands still, or raise `READY_JOINT_ERROR` in `robot/stepover_node.py` a little |
| `no_plan` | box read taller/deeper than the patterns cover, or more than 5° skew | `/terrain/event` shows the reading; `tools/plan_box.py --height .. --depth ..` reproduces it off the robot |
| Lurch or sag as the legs change hands | controller switch slower than expected | the legs are only damped during the switch; if the body drops more than about 5 mm raise `HandoverConfig.pre_kd` (max 0.8) |
| Body low, feet drag in the crawl | real joints sag more than modelled | re-weigh `extra_masses`; then raise `crawl.sag_gain` to 1.2 to 1.5 |
| `swing blocked ... backing out` on clear floor | joints lag the plan more than in simulation | `crawl.time_scale: 1.3`, or `player.swing_error_limit: 0.16` |
| A rear foot brushes the box's far face on the way down | the robot ends a little short after 30 swings, or distance/depth read short | redo step 4 of the calibration; then add `final_extra: 0.025` under `crawl:` in `terrain.yaml` (the rear feet finish 10 mm farther on) and check `tools/plan_box.py` still plans your box |
| Turns back and forth while squaring up to the box, then goes around | the turn pulses overshoot on this walking policy | lower `fsm.align_min_pulse` (0.2) or `fsm.align_yaw_rate` (0.3) |
| Goes around a box it should cross | read the reason in `/terrain/event` | height near the limit: use a lower box; "landing strip": clear the floor beyond; "depth not measured": the top colour is not recognised |
| Walks into / stops for things that are not there | floor colour model | `learn_colors.py check`, then `floor --add` on the patch that reads red |
| Steers but the robot does not follow | mux order | `check_robot.py`; `ros2 topic echo /cmd_vel_mux/active_source` |
| Terrain controllers left ACTIVE after a crash | | the guard releases them within a second; `python3 robot/spawn_controllers.py` reports it |

---

## 9. Working on it off the robot

```bash
pip install numpy opencv-python pyyaml pytest mujoco     # laptop only
python3 -m pytest tests -q                               # about 7 minutes
python3 sim/crossing_sim.py --height 0.019 --depth 0.089 --near 0.36 --view    # watch a crossing
python3 sim/avoid_sim.py                                 # a walk-around, with the path length
python3 tools/plan_box.py --height 0.022 --depth 0.08    # print the footsteps for a box
python3 sim/build_templates.py                           # rebuild the stored footstep patterns (30 min)
```

The physics tests need the Pupper model: clone `G-Levine/pupper_v3_description` next to this
repo or set `PUPPER_XML` to `.../description/mujoco_xml/pupper_v3_complete.position.xml`.
Rebuild the patterns after changing anything in `CrawlConfig` that alters the gait (clearances,
stance, margins, payload by more than about 100 g).

---

## 10. What is verified and what is not

Verified in simulation (MuJoCo, Pupper v3 model with measured leg envelopes, stiff foot
contacts, ball feet that roll, camera plate and mics as extra mass):

- Planning: every box from 19 to 31 mm tall and 70 to 102 mm deep plans from every stopping
  distance between 30 and 42 cm (shallower boxes only when low: up to 28 mm at 60 mm deep, 22 mm
  at 64 mm). 40 of 40 random boxes of 19–27 × 76–100 mm with up to 4° of skew planned, in 2.4 s
  median (19 s worst) on a desktop core.
- Crossing with exact measurements, 30 random boxes, distances, skews and payloads: 30 crossed,
  the box never moved, body tilt at most 6°, peak torque under 2 N·m (limit 3). In 3 runs the
  last rear foot brushed the far face of the box on its way down (under 6 N).
- Crossing with errors of ±3 mm in height, ±8 mm in distance and depth, ±1.5° in skew and
  ±40 % in the payload mass, 40 runs: 40 crossed, no abort, no fall, box moved at most 0.2 mm,
  tilt at most 6°; in 5 runs a rear foot or shin brushed the far face (under 8 N).
- Hand-over from the walking controller and back: the body drops 2 to 3 mm.
- E-stop, stale data, failed controller switch, refusals: tested through the real node code.
- Perception on rendered fisheye images through JPEG: height ±4 mm, distance ±10 mm, depth
  ±12 mm; calibration recovers camera height within 5 mm and pitch within 0.6°.

Not verified, because it needs the robot:

- That the real motors track the crawl as the simulated ones do (joint sag, friction, backlash).
- That the walking policy stands still enough at zero command for a crossing to start.
- Foot grip on the real floor and on the box top.
- The controller switch timing on the Pi, and planning time on the Pi.
- Colours and the lens model on real images; the IMU's levelling of the picture while walking.
- The real walking policy's response to the steering commands (the avoidance simulator uses a
  simple model of it).

Known limits: one box at a time, square to the path within 5°, on flat floor; nothing may
stand within 35 cm beyond it; a low obstacle that is *not* the two-colour box is always walked
around (a single camera cannot tell a low step from a flat mark without the colour trick);
something tall directly behind a low box is only seen once the box has been measured.
