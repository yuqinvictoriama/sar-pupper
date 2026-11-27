# Deploying terrain agility on Pupper

Every step from an empty Pi folder to Pupper telling three kinds of obstacle apart and
dealing with each: **stepping over** a low bar, **scaling across** a low slab, and
**walking around** everything else.

**Read this first.** All of it passes its tests off the robot, including whole crossings in a
physics simulation of Pupper v3 driven by the same node code that runs on the robot. **None of
it has run on the real robot yet.** The steps below are ordered so that each one tests one new
thing, with the least that can go wrong first. Do them in order and do not skip the stop
points. Someone holds the gamepad, ready on the e-stop (right-stick press), for every step
marked **MOVES**.

| Part | Time |
|---|---|
| 1. What Pupper does with each obstacle | read |
| 2. Put the code on the Pi | 5 min |
| 3. Start it and check it | 10 min |
| 4. Calibrate | 20 min |
| 5. Bring-up, rung by rung | 1 to 2 hours |
| 6. Run it with the rest of the mission | – |

---

## 1. What Pupper does with each obstacle

Pupper is not told which obstacle is which, and no colours are taught. It knows what the floor
looks like, measures whatever is in its way, and decides from the shape.

| | Step over | Scale across | Walk around |
|---|---|---|---|
| **What it is** | A low flat bar, shallow front to back | A low flat slab, deep enough to walk on | Anything else |
| **What Pupper does** | Each foot crosses in one stride, touching the bar's top once | Climbs onto it, crawls along the top with all four feet on it, climbs off the far side | Curves round the nearer end without stopping |
| **Height it accepts** | read as 10 to 27 mm (less for some depths) | read as 10 to 27 mm | – |
| **Depth it accepts** (the way Pupper walks) | 60 to 102 mm | 200 to 620 mm | – |
| **Width it needs** (across the path) | 400 mm or more | 400 mm or more | – |
| **How square** | within 5° after Pupper has squared itself up | the same | – |
| **Clear floor beyond it** | 350 mm | 350 mm | 0.5 m beside it |
| **Time** | about 1 min | about 1¼ min for a 380 mm slab, 1½ min for 600 mm | a few seconds |
| **Body moves forward** | about 0.6 m | slab depth + about 0.5 m (0.87 m for 380 mm) | – |
| **Simulated** | yes (30 of 30 and 40 of 40 crossings) | yes (40 of 40, and 60 of 60 with measurement errors) | yes |

- Read **under 10 mm**: a mark on the floor. Pupper walks on with its normal gait.
- **Anything that fails a check is walked around**, and `/terrain/event` says which check:
  too tall, too narrow, a depth between a bar and a slab (102 to 200 mm), not square, no clear
  floor beyond, or **its height could not be read** (see 1b).

### 1a. The course obstacles

| Obstacle | What Pupper does with it | Why |
|---|---|---|
| **Flat bar**, about 19 mm tall × 89 mm deep × 600 mm long (a length of "1×4" timber), flat side down, with an edge that shows (1b) | **Steps over**, when the 600 mm length is across the path | Low and shallow: each foot crosses in one stride, touching the top once |
| **Slab**, 15 mm tall × 565 mm wide × 380 mm deep, white top, raw board edge | **Scales across**, when the 565 mm side is across the path | Height read from the edge, depth from the top |
| The same slab **turned** so the 380 mm side is across the path | Walks around | 380 mm is under the 400 mm it needs to keep all four feet clear of the sides |
| **Round bar**, 50 mm diameter × 700 mm (not on the course any more: the flat bar took its place) | **Walks around** | See 1c |
| **Carton**, 455 × 348 × 130 mm, white | **Walks around**, from about a metre away, on any side | Plainly too tall from a distance; standing on edge it is taller than the camera |

Place the bar and the slab square to Pupper's path (within about 20°: Pupper squares itself up
to 5°), centred on it, lying flat on the floor without rocking, each with at least 350 mm of
bare floor beyond it. If one slides when pushed with a finger, put two strips of tape under it.

**The bar.** Any flat-topped, rigid bar 15 to 25 mm tall, 76 to 102 mm deep and at least
400 mm long will do; 19 × 89 mm is the middle of that range and the size everything was
tested with. Like the slab it needs an edge that shows (1b): a white top with the raw wood
left on the sides, or a strip of tape in another colour along the side facing Pupper. Measure
its real thickness and depth with a ruler: `check` (step 4.2) should agree within 3 mm and
12 mm.

**Bar first, then slab.** With the bar's near edge 0.70 m before the slab's, Pupper measures
the bar with the slab in view behind it, steps over, walks on, and measures the slab from
its own stopping distance. Leave at least 0.5 m between the bar's far edge and the slab.

### 1b. The one thing Pupper needs from a low obstacle: a visible edge

Pupper puts its feet on the slab, so it must know the slab's height to a few millimetres. One
camera can only read that height from the **side face at the obstacle's foot**: the band
between the floor and the top. That band has to look different from the top (another colour,
or the same colour in shadow) and different from the floor just in front of it.

- The slab's raw board edge under its white top is exactly this. In simulation it is read on a
  grey floor and on a wood-coloured floor (15 ± 1 mm). On the robot's own camera it has been
  found on a glossy wood floor from one position (section 9).
- A glossy floor makes this harder: it shows glare from the lights and a dim mirror image of
  the edge right under it. Pupper allows for both, but if `check` shows the side face on
  less than 90 % of the width, move the slab out of the worst glare.
- If the edge is the same colour as the floor, or the obstacle is one colour with no shading,
  the height reads as "not read" and Pupper walks around. Step 4.2 checks this on your floor
  before anything moves. The fix, if it is needed, is a strip of tape in any other colour
  along the slab's edge facing Pupper.

### 1c. The round bar cannot be stepped over

A 50 mm bar is too big, by a wide margin, and being round makes it worse.

- **Carrying a foot clear over it** needs the foot set down far enough beyond the bar for the
  shin behind it to clear the bar, and Pupper's knees point backward. With three feet on the
  ground and the body balanced, the planner finds a way only for obstacles up to about
  **10 to 12 mm tall and 20 mm deep**. At 15 mm it already finds none. 50 mm would need a
  stride of about 160 mm; Pupper's longest balanced stride is about 110 mm.
- **Using its top as a stepping stone**, which is how the low bar and the slab are crossed,
  needs a flat, rigid top at most 27 mm high. A round bar offers no foothold, and a light one
  rolls.
- Pupper measures it, reports why, and walks around it. It never tries.

What does work as a step-over obstacle, and is what the course now uses (1a): **a flat-topped
bar 15 to 25 mm tall and 76 to 102 mm deep**, at least 400 mm long, with an edge that shows
(1b). A length of "1×4" timber, which is really 19 × 89 mm, is in the middle of that range.

---

## 2. Put the code on the Pi

The Pi already has what it needs from the course (ROS 2 Jazzy, the pupperv3 monorepo,
`pupper_gait_deploy`, OpenCV, NumPy). Nothing is compiled.

1. On the Pi:
   ```bash
   cd ~/sar-pupper && git pull
   ```
   Your calibration lives in `config/terrain.yaml` on the Pi, as local changes that are not
   committed; `git pull` keeps them. If `git pull` ever refuses because of local changes to
   that file, run `git stash && git pull && git stash pop`.
2. **In every terminal you open on the Pi**, run these four lines first:
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/pupperv3-monorepo/ros2_ws/install/local_setup.bash
   export ROS_LOCALHOST_ONLY=1
   cd ~/sar-pupper/terrain-agility
   ```
3. Check the files, with nothing running:
   ```bash
   python3 tools/check_robot.py --offline
   ```
   **Expect:** no `FAIL`. Two `WARN` lines about calibration are normal at this point. It also
   plans a scale-across of a 15 × 380 mm slab and prints how long that took on the Pi: note it.

---

## 3. Start it and check it

Use the stand-alone launch for all of bring-up. It starts the course's robot stack, the
joystick, the velocity mux in the right order, the camera's fixed white balance, and terrain
agility. **Do not run Gemini's `start.sh` at the same time.**

1. Pupper on the floor, gamepad on. Terminal 1:
   ```bash
   ros2 launch robot/standalone.launch.py avoid:=false
   ```
   (`avoid:=false` starts only the crossing side. The camera side comes in at rung 7.)
2. Stand Pupper up with the gamepad (X), as usual.
3. Terminal 2:
   ```bash
   python3 tools/check_robot.py
   ```
   **Expect:** no `FAIL`. Every `FAIL` and `WARN` line says what to do. The ones that matter most:

   | Line | Must say |
   |---|---|
   | forward_command_controller is installed | OK. If not: `sudo apt install ros-jazzy-forward-command-controller` |
   | terrain controllers loaded | OK |
   | active walking controller | exactly one |
   | mux priority | `/terrain_cmd_vel` present, above `/llm_cmd_vel` |
   | standing pose | OK. If it says the legs shuffle while standing, a crossing will be refused: tell me what it prints |

---

## 4. Calibrate (once per room and lighting)

Pupper **standing** under its walking controller, on the floor you will run on. Each command
prints what it found.

1. **Floor.** Nothing but floor for 1 m ahead and to the sides.
   ```bash
   python3 tools/learn_colors.py floor
   ```
   If the floor varies (planks, tiles), turn Pupper to face two or three other patches and run
   `python3 tools/learn_colors.py floor --add` on each.
   **Expect:** "85 % or more of the view reads as floor".

2. **Look at what Pupper makes of each obstacle.** Nothing moves. Put the slab square across
   Pupper's path, its near face 27 cm ahead of the front of the front feet, nothing else within
   1 m.
   ```bash
   python3 tools/learn_colors.py check
   ```
   **Expect, for the slab:** a `gauge` line with a `height` (not "not read") and `side face
   seen on` 90 % or more. The numbers themselves, and `would do`, can be well off until
   step 3: before it the tool assumes a camera 250 mm up and level, and this Pupper's is
   much lower and looks up. After step 3: `height` near 15 mm, `depth` near 380 mm,
   `would do: scale_across`.
   - **"height not read"**: the edge does not show against this floor or against the top (1b).
     Put a strip of tape in another colour along the edge and run it again.
   - Repeat with the **flat bar** (after step 3 expect `height` and `depth` within 3 mm and
     12 mm of what your ruler says, and `would do: step_over`) and the **carton** at 1 m
     (expect a `scan` line ending "certainly tall").
   - Copy `look.jpg` to your Mac
     (`scp <user>@<pupper>:~/sar-pupper/terrain-agility/look.jpg .`). Top half: the camera's
     view, unrolled. Bottom half: the same view by class. Floor must be **grey**, obstacles
     **red**. Red patches on bare floor mean the floor model needs another `floor --add`
     there. Find where Pupper's own chin or feet enter at the bottom of the picture: the
     magenta line must be above them. If it is not, lower `camera.near_depression_deg` in
     `config/terrain.yaml` by a few degrees and run `check` again.

3. **Camera position.** Use the slab. Move the **slab**, not Pupper. Measure with the ruler on
   the floor from the **front of the front feet** to the slab's near face, slab square to
   Pupper.
   ```bash
   python3 tools/calibrate_camera.py clear        # forget any earlier attempt
   python3 tools/calibrate_camera.py add 0.25     # slab 25 cm from the toes
   python3 tools/calibrate_camera.py add 0.35     # move it to 35 cm
   python3 tools/calibrate_camera.py add 0.50     # move it to 50 cm
   python3 tools/calibrate_camera.py solve --box-height 0.015 --box-depth 0.38 --write
   ```
   `--box-height` is the slab's **measured** thickness and `--box-depth` its measured size
   fore-aft (near face to far edge, as it lies), both in metres.
   - **Each `add` prints three angles**: the foot of the near face, its top edge, the far
     edge. All three must be numbers. "not seen" for the top edge means the edge does not
     show (1b): fix that before going on. Adding a distance again replaces the earlier
     reading for it; `python3 tools/calibrate_camera.py list` shows what is recorded.
   - **Not closer than 25 cm**: nearer than that the foot of the slab drops out of the
     bottom of the camera's view, and `add` refuses.
   - **Expect from `solve`:** "fit residual ... (good)", and a table of how far each angle
     is from the fit, all within about 0.4 deg. If it says HIGH it names the distance that
     fits worst: measure that one again, `add` it again, `solve` again. It will not write a
     poor fit.
   - For this Pupper expect a camera height near 160 to 170 mm and a pitch near -11 deg
     (looking a little up). That is my estimate from one picture, not a measurement.

   **Check it:** put the slab at 27 cm from the toes and run `python3 tools/learn_colors.py check`.
   The gauge line must give the height within 3 mm, the depth within 15 mm, and the distance
   within 1 cm of 360 mm (27 cm from the toes is 36 cm from the body centre).
   **Stop point: do not go on until it does.** The crossing counts its steps blind from this
   distance.

4. **Payload (optional: no scale needed).** The crossings allow for the hardware added to the
   stock Pupper, using my estimates: 130 g for the camera plate and cameras on the head, 65 g
   for the mics. Getting these exact does not matter much. In simulation, with the planner
   kept on my estimates and the real hardware made lighter or heavier at the same places:

   | Real added hardware | Low bar | Slab |
   |---|---|---|
   | none at all | crossed, tilt 3.4° | crossed, tilt 1.9° |
   | 0.39 kg (twice my estimate) | crossed, tilt 4.7° | crossed, tilt 2.8° |
   | 0.58 kg (3×) | crossed, tilt 6.1° | crossed, tilt 3.3° |
   | 0.78 kg (4×) | crossed, tilt 8.1° | crossed, tilt 4.0° |
   | 0.98 kg (5×) | **tips over** | crossed, tilt 4.7° |

   - If everything added to the stock Pupper together is clearly lighter than a full 500 ml
     water bottle (0.5 kg) held in the other hand, leave the estimates as they are.
   - If something heavier has been added (a power bank, another board), weigh it on any
     kitchen or postal scale, to the nearest 10 g, and add a line for it under `crawl:` →
     `extra_masses` in `config/terrain.yaml`: `[mass in kg, x, y, z]`, metres from the point
     between the hips (x forward, z up).
   - A badly wrong payload shows before any obstacle: the body lurches in rung 3 or a foot
     drags or Pupper leans in rung 5.

5. **Walking speed** (**MOVES**). 2 m of clear floor ahead.
   ```bash
   python3 tools/walk_forward.py --seconds 5
   ```
   Measure how far Pupper went. Set `steer.speed_scale` in `config/terrain.yaml` to
   that distance in metres ÷ 1.5.
   - Measured on this Pupper (26 November): 0.80 m in 5 s, so `speed_scale: 0.533`. It walks
     at about half the speed it is told to. The figure includes the moment it takes to get
     going, so its cruising speed is a little higher; a value on the low side is the safe
     one (when walking round something, Pupper then assumes it has not yet passed it).

After editing `terrain.yaml`, stop the launch (Ctrl-C) and start it again.

---

## 5. Bring-up, rung by rung

For rungs 3 to 6: set `crawl.time_scale: 1.5` in `config/terrain.yaml` (everything runs half
as slow again), and put a foam mat or folded blanket under Pupper for the first try of rungs
3 to 5.

Launch as in part 3 (`avoid:=false`), Pupper standing. All commands in terminal 2.

| Rung | Do | Expect | Stop if |
|---|---|---|---|
| **1** | `python3 tools/check_robot.py` | no FAIL | any FAIL |
| **2** | `python3 tools/step_over.py cross --height 0.015 --depth 0.38 --near 0.36 --dry-run` | A plan is printed: about 40 foot swings, 70 s. **Nothing moves.** Note the planning time | "no_plan" |
| **3 MOVES** | `python3 tools/step_over.py stand` | Body shifts a millimetre or two, stands 3 s, shifts back; Pupper is walking-ready again after 4 s. No lurch | the body drops or jerks at the start |
| **4 MOVES** | `python3 tools/step_over.py stand --hold 8`, and **press the e-stop** during the hold | Legs go limp at once, Pupper sits down | the legs stay stiff. **Do not go on until the e-stop works.** Stand Pupper up again afterwards |
| **5 MOVES** | Mark the floor at the front of a front foot. `python3 tools/step_over.py crawl --distance 0.24` on clear floor. Measure how far that foot went | Twelve slow steps, body level, no foot dragging, **240 mm, give or take 1 cm** | a foot drags, Pupper tips, or it is more than 1 cm out |
| **6 MOVES** | Slab square to Pupper, near face **27 cm ahead of the front of the front feet**, 1 m clear beyond. Mark the floor at the front of a front foot. `python3 tools/step_over.py cross --height 0.015 --depth 0.38 --near 0.36` | About two minutes at this speed. Slow steps up to the slab; front feet onto it, then rear feet; along the top; front feet down beyond the far edge, then rear feet; stands; walking-ready again. The last report prints `advance` (about 0.87): that foot should be that far from its mark, within 1.5 cm | a foot lands on an edge, or Pupper tips. Note which foot and when |
| **7 MOVES** | Ctrl-C the launch, start it again **without** `avoid:=false`. Carton 1.5 m straight ahead. `python3 tools/walk_forward.py` | Pupper curves round it without stopping, about 20 cm clear, then straightens onto its old direction | it walks into it, or stops and stays |
| **8 MOVES** | Slab 1.5 m ahead, square, 1 m clear beyond. `python3 tools/walk_forward.py --seconds 20` | Walks up, stops about 36 cm away, squares up, measures (`/terrain/event` prints height, depth and "scale across"), then crosses as in rung 6 | it walks around instead (see below) |
| **9 MOVES** | Launch with `avoid:=false` again. Flat bar square to Pupper, near face **27 cm ahead of the front of the front feet**, 1 m clear beyond. `python3 tools/step_over.py cross --height 0.019 --depth 0.089 --near 0.36` (your bar's measured thickness and depth) | About 30 slow steps, under a minute at normal speed: each front foot onto the bar's top and off beyond it, then each rear foot; stands; walking-ready again. `advance` about 0.58 | a foot lands on an edge, the bar slides, or Pupper tips |
| **10 MOVES** | Launch without `avoid:=false`. Bar 1.2 m ahead, slab's near edge 0.70 m beyond the bar's, 1 m clear beyond the slab. `python3 tools/walk_forward.py --seconds 30`, and run it again once Pupper is past the bar if it has stopped | Walks up to the bar, measures it ("step over"), crosses; walks on, measures the slab ("scale across"), crosses | it walks around either one (the event says why) |

- **Rung 6 sets one number.** Over a slab, Pupper covers a little less ground than its steps
  add up to: 2 % less in simulation (on bare floor, rung 5, nothing is lost), which is what
  `crawl.advance_scale: 0.98` allows for. If the foot ended more than 1.5 cm from where
  `advance` says, multiply `crawl.advance_scale` in `config/terrain.yaml` by
  (distance measured ÷ `advance`) and run rung 6 again. This is what puts the feet down on the
  right side of the slab's far edge, which Pupper reaches after about 0.75 m of counted steps.
- Use your own measured thickness in rungs 2 and 6 if it is not 0.015.
- To see what Pupper decided and why, at any time: `ros2 topic echo /terrain/event`.
  If it walks around something it should cross, this prints the reason.
- `python3 tools/step_over.py stop` aborts a crossing: the joints go soft and Pupper settles.
- When rungs 6 and 8 work three times in a row, set `crawl.time_scale` back to `1.0`.
- **A low bar** (1c), if you add one: the same rungs 2, 6 and 8 with its own height and depth,
  for example `--height 0.019 --depth 0.089`. A depth under 0.20 m is stepped over; from
  0.20 m it is scaled across.

---

## 6. Running it with the rest of the mission

- **With Gemini:** copy the `inputs:` list from `robot/terrain_mux.yaml` into the `cmd_vel_mux`
  section of `~/gemini_pupper/robot/launch/gemini_pupper.yaml`. Start Gemini as usual, then in
  another terminal `ros2 launch robot/terrain.launch.py`. Repeat rungs 7 to 9 by asking
  Gemini to walk forward.
- **From your own launch file:** start the mux with `robot/terrain_mux.yaml` and include
  `robot/terrain.launch.py` (the snippet is at the top of that file).
- **What the mission layer sees** on `/terrain/state` (JSON): `state` is one of `CRUISE`,
  `AVOID`, `APPROACH`, `GAUGE`, `ALIGN`, `PLAN`, `STEP_OVER`, `SCALE_ACROSS`, `HOLD`,
  `DISABLED`, `REPORT`; `kind` is what was decided (`step_over`, `scale_across`, `go_around`,
  `ignore`); `box` is the measurement. While the state is `STEP_OVER` or `SCALE_ACROSS` the
  crawl has the legs and no velocity command does anything.
- **How far a crossing took Pupper** is in `/terrain/crossing_status`: `started` and `done`
  carry `advance`, in metres on the floor.
- **What Pupper says.** The demo's navigation has one call-out for a low obstacle ("Low
  obstacle. Stepping over. Still coming."), and says it for the slab as well as for a bar. To
  know which crossing it is, read `kind` or the state, not the call-out.
- **On the demo course** the first stretch is the flat bar, then the slab. Where each goes
  is set in `path-navigation/config/demo.yaml` (`course.obstacles`) and described in
  `path-navigation/README.md`, "Demo instructions"; that file is the one to trust for the
  distances. In simulation Pupper stops 0.36 m short of each, the bar takes about 52 s and
  moves the body 0.58 m, the slab about 80 s and 0.87 m.
- **Modes** (`/terrain/mode`, a `std_msgs/String`): `full` crosses what can be crossed,
  `avoid` walks around everything, `report` only looks, `off` switches it off.
- **Before a demo:** do one crossing as a warm-up. Planning the same slab again is quicker.

---

## 7. What it does when things go wrong

| Situation | What Pupper does | What you do |
|---|---|---|
| Gamepad e-stop | Legs limp immediately | Stand it up again with the gamepad |
| Body tilts over 20°, a foot is blocked, or a leg is far from where it should be | Joints go soft, Pupper settles. The walking controller is **not** restarted | Lift Pupper clear, stand it up, `ros2 topic pub -1 /terrain/reset std_msgs/Empty` |
| The obstacle cannot be crossed (too tall, wrong depth, too narrow, skewed, height not read, no clear floor beyond) | Walks around it, says why on `/terrain/event` | Fix what the event names |
| Blocked on both sides | Stands still, reports `HOLD`, retries every 2 s | Clear a side |
| The camera stops sending frames | Stands still | Check the camera node |
| A foot comes down on the slab's far edge instead of beyond it | Carries on; in simulation this is a light touch (under 11 N) and never a fall | Redo the measurement in rung 6 (`crawl.advance_scale`) and the check in step 4.3: the edge was farther than Pupper thought |

More symptoms and fixes are in `README.md`, section 8.

---

## 8. Numbers worth knowing

| | |
|---|---|
| Where it stops to measure | 36 cm from the body centre to the obstacle's near face (27 cm from the toes) |
| Scale across, 15 × 380 mm slab | about 40 foot swings, 70 s, body 0.87 m farther on |
| Scale across, planning | 1 to 4 s on a laptop for most slabs; up to 30 s for a 30 mm slab met 4° askew. I have not timed it on the Pi: expect several times longer, and rung 2 tells you |
| Step over, 19 × 89 mm bar | about 33 foot swings, 57 s |
| Pupper cannot dodge during a crossing | no throws until it is walking again |
| Walk-around starts | when something certainly tall is within 1.2 m in its path |
| A closer look starts | when something that may be low is within 1.0 m in its path |
| Gains used for the crawl | kp 7.5, kd 0.25: the walking controllers' own start-up values, nothing raised |

## 9. What is verified, and where

Verified in simulation (MuJoCo, Pupper v3 model with measured leg envelopes, camera plate and
mics as extra mass, the node's own hand-over):

- **Scale across, exact measurements:** 40 random slabs (10 to 27 mm tall, 250 to 600 mm deep,
  up to 4° askew, met from 32 to 40 cm, fixed to the floor or 1 to 2.5 kg, payload 0.6 to 1.4
  times what the planner assumes). 40 planned, 40 crossed. Body tilt at most 3.3°. No leg
  touched the slab anywhere but a foot on its top. The slab never moved.
- **Scale across, with measurement errors** on top (distance ±10 mm, height ±3 mm, depth
  ±15 mm, skew ±1.5°): 60 of 60 crossed, tilt at most 4.7°, slab never moved, one light touch
  of a foot on an edge (4.9 N). Pupper ended within 15 mm of where the plan said.
- **Planning:** every slab of 10, 15, 20, 25 and 30 mm × 200 to 600 mm, met from 32, 36 and
  40 cm, square and 4° askew either way, has a plan (225 cases).
- **Telling the obstacles apart**, on rendered fisheye frames through JPEG, on a grey floor and
  a wood-coloured one: the slab is scaled across; the slab turned end-on, the round bar, the
  carton lying and standing, a 40 mm slab, a 70 mm block that hides as much floor as the slab,
  and a slab whose edge is the floor's colour are all walked around, each for the right reason;
  a low bar is stepped over.
- **Through the real nodes** with a stand-in for ROS: camera frames of the slab → approach →
  measure → request → plan → hand-over → crossing in physics → hand-back.
- **Bar, then slab, one after the other:** in physics, through the same crossing node with
  nothing restarted: the bar stepped over (52 s, 0.58 m, tilt at most 3.8°), then the slab
  0.36 m beyond where that left Pupper scaled across (80 s, 0.87 m, tilt at most 2.0°);
  neither was pushed. And from camera frames with both in view at once: the bar is measured
  and crossed first, then the slab. One run of each, not a spread of cases.

Seen on the robot so far (26 November, evening; nothing moved):

- **Six camera frames of the slab on a glossy wood floor**, one slab position. On them the
  first version of the gauge failed: it took the slab's raw edge for floor, lost half the
  slab to glare patches, and read no height. The gauge was reworked against those frames (the
  edge is now found by comparing it with the floor just beyond it; its mirror image in the
  floor and patches of glare are left out) and now finds the edge on 98 % of the slab's
  width, with the same angles on all six frames. That is one position on one floor.
- **The camera is not where I had assumed**: lower (about 165 mm, not 250 mm) and looking
  about 11° up. Step 4.3 measures it; until it has been run and checked, no distance or
  height from the camera is right.
- The first try at step 4.3 failed ("fit too poor") because of the gauge fault above, not
  because of how it was run.

Not verified, because it needs the robot:

- That the real motors track the crawl as the simulated ones do, and how far it really
  advances (rungs 5 and 6).
- That the slab's edge shows on your floor under your lights from every distance (step 4.2;
  seen from one), and the lens model and camera position on real frames (step 4.3, not yet
  completed).
- The round bar and the carton on real frames.
- Foot grip on the slab's top, and whether the slab stays put.
- That the walking policy stands still enough at zero command for a crossing to start.
- Planning time and controller-switch time on the Pi.

Known limits:

- One obstacle at a time, on flat floor, with 350 mm of clear floor beyond it.
- A slab or bar whose height cannot be read is walked around, never guessed at. So is a flat
  mark the size of a slab (a sheet of paper): one camera cannot tell it from a low step.
- A round bar between about 55 and 75 mm across can look like a low flat bar. The check that
  the top is at least 2.6 times as deep as it is tall is there to refuse it; I have tested
  that check on the 50 mm bar only.
- Something tall directly behind a slab is only seen once the slab has been measured.
