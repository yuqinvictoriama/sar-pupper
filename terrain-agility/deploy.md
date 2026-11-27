# Deploying terrain agility on Pupper

What to buy, how to prepare it, and every step from an empty Pi folder to Pupper stepping over
a box and walking around an obstacle on its own.

**Read this first.** The code passes its 96 tests and crosses boxes in a physics simulation of
Pupper v3. It has **not run on the real robot yet**. The steps below are ordered so that each
one tests one new thing, with the least that can go wrong first. Do them in order and do not
skip the stop points. Someone holds the gamepad, ready on the e-stop (right-stick press), for
every step marked **MOVES**.

| Part | Time |
|---|---|
| 1. Get the materials | a shop visit |
| 2. Prepare the box and the obstacle | 20 min |
| 3. Put the code on the Pi | 5 min |
| 4. Start it and check it | 10 min |
| 5. Calibrate | 20 min |
| 6. Bring-up, rung by rung | 1 to 2 hours |
| 7. Run it with the rest of the mission | – |

---

## 1. Materials

### 1a. The step-over box (the one Pupper crosses)

One straight wooden board. Pupper steps **onto** it and off the other side, one foot at a time.

| Dimension | Minimum | **Suggested** | Maximum | What happens outside the range |
|---|---|---|---|---|
| **Height** (floor to top) | 15 mm (⅝ in) | **19 mm (¾ in)** | 25 mm (1 in) | Read under 10 mm: treated as a mark on the floor and walked over normally. Read over 27 mm: refused, Pupper walks around it |
| **Depth** (the direction Pupper walks) | 76 mm (3 in) | **89 mm (3½ in)** | 102 mm (4 in) | Shallower: only very low boxes are accepted (at 64 mm, nothing read over 19 mm). Deeper: refused |
| **Width** (across Pupper's path) | 500 mm (19¾ in) | **550 mm (22 in)** | 600 mm (23½ in) | Narrower: fails the "room to spare on both sides" check, Pupper walks around. Wider still works, but the camera can no longer see both ends, so the centring check is skipped |
| **Weight** | 250 g | **about 400 g** (what the suggested board weighs) | none | Lighter: can shift when a foot lands on it. Tape it to the floor instead |

**What to buy: a "1×4" softwood board.** Its real size is 19 × 89 mm (¾ × 3½ in), which is the
suggested height and depth exactly. Have a 22 in (550 mm) piece cut from it.

- Pick a **straight, flat** board. Lay it on the shop floor: if it rocks, take another.
- Square edges, not rounded or bevelled.
- **Do not buy:** a "1×3" (64 mm deep: too shallow), a "2×3" or "2×4" (38 mm tall: too tall),
  or foam/cardboard (it gives under a foot).

Why these limits. Height: with three feet on the ground and the body balanced, Pupper's legs
cannot clear more. Depth: a foot on the top rolls forward 15 to 25 mm as the body passes over
it and must stay 12 mm from both edges. The suggested values sit in the middle of what was
tested, so a small camera error in either direction still gives a crossing.

### 1b. The tall obstacle (the one Pupper walks around)

Any rigid box. A small shipping carton with a book inside is fine.

| Dimension | Minimum | **Suggested** | Maximum | Notes |
|---|---|---|---|---|
| **Height** | 80 mm | **250 mm or more** | none | From 250 mm it crosses the camera's horizon and is recognised as tall from far away, so the turn starts early. Under 80 mm it is still walked around, but only once Pupper is close |
| **Width** (across the path) | 200 mm | **300 to 400 mm** | 600 mm | Simulated from 200 to 600 mm. For the demo course keep it at 400 mm or less (`path-navigation/plan.md`) |
| **Depth** (along the path) | 100 mm | **200 to 300 mm** | 400 mm | Simulated at 300 mm; the other figures are my judgement, not tested |
| **Weight** | enough not to slide if brushed | **1 kg** (a book inside) | none | |
| **Clear floor beside it** | 0.5 m on the side Pupper will pass | **1 m on both sides** | – | Pupper keeps 22 cm between the obstacle and its body centre. With less than about 0.45 m free on both sides it stops and reports "blocked" |

### 1c. Colours (this matters as much as the sizes)

Your floor is hardwood, so:

| Item | Use | Avoid |
|---|---|---|
| Step-over box, **four side faces** | **Blue painter's tape**, matte | – |
| Step-over box, **top face** | **Matte white** tape, paint or paper | Glossy white tape (it reflects the lights) |
| Tall obstacle | One solid matte colour unlike wood: **white, black, purple or pink**. Wrap it in paper if needed | **Plain brown cardboard, tan, orange, red** (too close to the floor: Pupper may not see it). **Blue** (it would be mistaken for the step-over box at first). **Neon green or yellow-green** (the ball's colour) |

The two colours on the box are how the camera tells a low box from a mark on the floor, and
how it reads height (blue band) and depth (white band) separately. A box in one colour reads
about three times too tall and is walked around.

### 1d. Supplies and tools

| Item | Minimum | Suggested | Used for |
|---|---|---|---|
| Blue painter's tape | 19 mm (¾ in) wide | 24 mm (1 in) wide, one roll | Box sides |
| White matte tape, paint or paper | covers 89 × 550 mm | white gaffer tape, 48 mm wide | Box top |
| Thin non-slip liner or double-sided tape | – | drawer/shelf liner, 1 mm or thinner | Under the box: wood slides on hardwood |
| Ruler or tape measure, mm markings | 60 cm | 1 m | Calibration distances, box placement |
| Kitchen scale | 1 g steps, 500 g range | – | Weighing the camera plate and mics |
| Masking tape | – | one roll | Marks on the floor |
| Foam mat, cushion or folded blanket | – | – | Under Pupper for the first leg tests |
| Calipers | optional | – | Measuring the finished box height |

### 1e. Space

| | Minimum | Suggested |
|---|---|---|
| Clear floor in front of the obstacle | 1.2 m | 1.5 m |
| Clear floor **beyond the step-over box** | 0.35 m (a crossing is refused with less) | 1 m |
| Clear floor around the obstacles | 0.5 m | 1 m |
| Lighting | steady, the same as on demo day | no direct sun patches on the floor |

Bags, chair legs and feet within about 0.5 m of Pupper's path count as obstacles.

---

## 2. Prepare the box and the obstacle

1. **Tape the sides blue.** All four vertical faces. Let the extra tape width fold **under**
   the board, never over the top edge.
2. **Make the top white**, edge to edge. No blue showing on top, no white showing on the sides.
3. **Add the non-slip liner** underneath (or have tape ready to fix the box to the floor).
4. **Measure the finished height** in three places, floor to top, with the liner on. Write it
   down in metres (for example `0.020`). You need this number in step 5.4.
5. **Put a book in the tall box**, close it, and wrap it if it is brown.

---

## 3. Put the code on the Pi

The Pi already has what it needs from the course (ROS 2 Jazzy, the pupperv3 monorepo,
`pupper_gait_deploy`, OpenCV, NumPy). Nothing is compiled.

1. Unzip `terrain-agility.zip` on your Mac into your repo, so you have
   `~/Desktop/CS123/pi/sar-pupper/terrain-agility/`.
2. Copy it to the Pi (or commit, push, and `git pull` on the Pi):
   ```bash
   rsync -av ~/Desktop/CS123/pi/sar-pupper/terrain-agility/ <user>@<pupper>:~/sar-pupper/terrain-agility/
   ```
3. **In every terminal you open on the Pi**, run these four lines first:
   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/pupperv3-monorepo/ros2_ws/install/local_setup.bash
   export ROS_LOCALHOST_ONLY=1
   cd ~/sar-pupper/terrain-agility
   ```
4. Check the files, with nothing running:
   ```bash
   python3 tools/check_robot.py --offline
   ```
   **Expect:** no `FAIL`. Two `WARN` lines about calibration are normal at this point.

---

## 4. Start it and check it

Use the stand-alone launch for all of bring-up. It starts the course's robot stack, the
joystick, the velocity mux in the right order, the camera's fixed white balance, and terrain
agility. **Do not run Gemini's `start.sh` at the same time.**

1. Pupper on the floor, gamepad on. Terminal 1:
   ```bash
   ros2 launch robot/standalone.launch.py avoid:=false
   ```
   (`avoid:=false` starts only the step-over side. The camera side comes in at rung 7.)
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

## 5. Calibrate (once per room and lighting)

Pupper **standing** under its walking controller, on the floor you will run on. Each command
prints what it found.

1. **Floor.** Nothing but floor for 1 m ahead and to the sides.
   ```bash
   python3 tools/learn_colors.py floor
   ```
   Hardwood varies plank to plank: turn Pupper to face two or three other patches and run
   `python3 tools/learn_colors.py floor --add` on each.
   **Expect:** "85 % or more of the view reads as floor".

2. **Box colours.** Box square across Pupper's path, about 40 cm ahead of the front feet.
   Nothing else within 1 m.
   ```bash
   python3 tools/learn_colors.py box
   ```
   **Expect:** two colour ranges printed, no `WARNING`.

3. **Look at what Pupper sees.**
   ```bash
   python3 tools/learn_colors.py check
   ```
   Copy `look.jpg` to your Mac (`scp <user>@<pupper>:~/sar-pupper/terrain-agility/look.jpg .`).
   Top half: the camera's view, unrolled. Bottom half: the same view by class.
   - Floor must be **grey**. The box must be a **blue band with a white band above it**.
   - Red patches on bare floor mean the floor model needs another `floor --add` there.
   - Find where Pupper's own chin or feet enter at the bottom of the picture. The magenta line
     must be above them. If it is not, lower `camera.near_depression_deg` in
     `config/terrain.yaml` by a few degrees and run `check` again.

4. **Camera position.** Move the **box**, not Pupper. Measure with the ruler on the floor from
   the **front of the front feet** to the box's near face.
   ```bash
   python3 tools/calibrate_camera.py add 0.25     # box 25 cm from the toes
   python3 tools/calibrate_camera.py add 0.35     # move it to 35 cm
   python3 tools/calibrate_camera.py add 0.50     # move it to 50 cm
   python3 tools/calibrate_camera.py solve --box-height 0.020 --write
   ```
   Use **your** measured box height from step 2.4 in place of `0.020`.
   **Expect:** "fit residual ... (good)".
   **Check it:** put the box at a new distance, run `python3 tools/learn_colors.py check`. The
   gauge line must give the height within 3 mm and the distance within 1 cm of your ruler.
   **Stop point: do not go on until it does.** The crossing counts its last steps blind from
   this distance.

5. **Payload.** Weigh the camera plate with both cameras on it, and one mic. In
   `config/terrain.yaml`, under `crawl:` → `extra_masses`, replace my estimates (130 g for the
   plate and cameras, 65 g for the mics together) with the real masses in kg.

6. **Walking speed** (**MOVES**). 2 m of clear floor ahead.
   ```bash
   python3 tools/walk_forward.py --seconds 5
   ```
   Measure how far Pupper went. Set `steer.speed_scale` in `config/terrain.yaml` to
   that distance in metres ÷ 1.5.

After editing `terrain.yaml`, stop the launch (Ctrl-C) and start it again.

---

## 6. Bring-up, rung by rung

For rungs 3 to 6: set `crawl.time_scale: 1.5` in `config/terrain.yaml` (everything runs half
as slow again), and put the foam mat under Pupper for the first try of each.

Launch as in part 4 (`avoid:=false`), Pupper standing. All commands in terminal 2.

| Rung | Do | Expect | Stop if |
|---|---|---|---|
| **1** | `python3 tools/check_robot.py` | no FAIL | any FAIL |
| **2** | `python3 tools/step_over.py cross --height 0.019 --depth 0.089 --near 0.36 --dry-run` | A plan is printed within a minute. **Nothing moves.** Note the planning time | "no_plan" |
| **3 MOVES** | `python3 tools/step_over.py stand` | Body shifts a millimetre or two, stands 3 s, shifts back; Pupper is walking-ready again after 4 s. No lurch | the body drops or jerks at the start |
| **4 MOVES** | `python3 tools/step_over.py stand --hold 8`, and **press the e-stop** during the hold | Legs go limp at once, Pupper sits down | the legs stay stiff. **Do not go on until the e-stop works.** Stand Pupper up again afterwards |
| **5 MOVES** | `python3 tools/step_over.py crawl --distance 0.08` on clear floor | Four slow steps, about 8 cm forward, body level, no foot dragging | a foot drags, or Pupper tips |
| **6 MOVES** | Box square to Pupper, near face **27 cm ahead of the front of the front feet**, 1 m clear beyond. `python3 tools/step_over.py cross --height 0.019 --depth 0.089 --near 0.36` | About a minute and a half at this speed. Slow steps up to the box; each foot onto the top and down the far side; stands; walking-ready again | a foot lands on an edge, or Pupper tips. Note which foot and when |
| **7 MOVES** | Ctrl-C the launch, start it again **without** `avoid:=false`. Tall obstacle 1.5 m straight ahead. `python3 tools/walk_forward.py` | Pupper curves round it without stopping, about 20 cm clear, then straightens onto its old direction | it walks into it, or stops and stays |
| **8 MOVES** | Step-over box 1.5 m ahead, square, 1 m clear beyond. `python3 tools/walk_forward.py --seconds 20` | Walks up, stops about 36 cm away, squares up, measures, then crosses as in rung 6 | it walks around instead (see below) |

- Use your own measured height in rungs 2 and 6 if it is not 0.019.
- To see what Pupper decided and why, at any time: `ros2 topic echo /terrain/event`.
  If it walks around a box it should cross, this prints the reason (height, depth, skew,
  width, landing strip).
- `python3 tools/step_over.py stop` aborts a crossing: the joints go soft and Pupper settles.
- When rungs 6 and 8 work three times in a row, set `crawl.time_scale` back to `1.0`.

---

## 7. Running it with the rest of the mission

- **With Gemini:** copy the `inputs:` list from `robot/terrain_mux.yaml` into the `cmd_vel_mux`
  section of `~/gemini_pupper/robot/launch/gemini_pupper.yaml`. Start Gemini as usual, then in
  another terminal `ros2 launch robot/terrain.launch.py`. Repeat rungs 7 and 8 by asking
  Gemini to walk forward.
- **From your own launch file:** start the mux with `robot/terrain_mux.yaml` and include
  `robot/terrain.launch.py` (the snippet is at the top of that file).
- **On the demo course** (`path-navigation/plan.md`): step-over box with its near face 0.60 m
  from the start mark, square to the line and centred on it; tall obstacle with its near face
  at 1.70 m. Keep at least 35 cm of bare floor beyond the box.
- **Not built yet:** the path-navigation plan asks terrain agility for a three-way mode switch
  (full / walk-around only / report only), the distance covered in the crossing report, and
  navigation ranked above Gemini in the mux. Today there is only on and off:
  `ros2 topic pub -1 /terrain/enable std_msgs/Bool "data: false"`.
- **Before a demo:** do one crossing as a warm-up. The first plan of a session is the slow one;
  the same box plans in a second or two afterwards.

---

## 8. What it does when things go wrong

| Situation | What Pupper does | What you do |
|---|---|---|
| Gamepad e-stop | Legs limp immediately | Stand it up again with the gamepad |
| Body tilts over 20°, a foot is blocked, or a leg is far from where it should be | Joints go soft, Pupper settles. The walking controller is **not** restarted | Lift Pupper clear, stand it up, `ros2 topic pub -1 /terrain/reset std_msgs/Empty` |
| The box cannot be crossed (too tall, too shallow, skewed, no clear floor beyond) | Walks around it, says why on `/terrain/event` | Fix what the event names |
| Blocked on both sides | Stands still, reports `HOLD`, retries every 2 s | Clear a side |
| The camera stops sending frames | Stands still | Check the camera node |

More symptoms and fixes are in `README.md`, section 8.

---

## 9. Numbers worth knowing

| | |
|---|---|
| Crossing time | 45 to 60 s, plus planning (2 to 20 s on a laptop; expect about double on the Pi) |
| Pupper cannot dodge during a crossing | no throws until it is walking again |
| Where it stops to measure the box | 36 cm from body centre to the near face |
| Walk-around starts | when a tall obstacle is within 1.2 m in its path |
| Gains used for the crawl | kp 7.5, kd 0.25: the walking controllers' own start-up values, nothing raised |
