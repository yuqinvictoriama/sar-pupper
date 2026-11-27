# Terrain agility – plan and as-built design (v3)

v1 written 2026-11-23 as the plan to code against. v2, same day: implemented and tested in simulation. **v3, 2026-11-26: three behaviours instead of two (step over, scale across, walk around), and obstacles told apart by shape instead of by taught colours. Implemented and tested in simulation; nothing here has been on the robot yet.** `README.md` is the reference, `deploy.md` the step-by-step; this file is the why.

The sections from "What changed from v1" onward are v2 as written. They still describe the low-bar gait, the hand-over, the abort rules and the walk-around exactly. Where they speak of "the two-colour box", read "a low bar whose edge shows" (T6 below).

## What changed in v3

The course obstacles changed: a white slab 15 mm tall × 565 mm wide × 380 mm deep with a raw board edge, a white round bar 50 mm across and 700 mm long, and a white carton 455 × 348 × 130 mm. v2 could cross one thing, a board with blue sides and a white top, and walked around everything else low. None of the new obstacles is blue, and the slab is four times deeper than anything v2 would cross.

| v2 | v3 | Why |
|---|---|---|
| The crossable obstacle is known by its colours (blue sides, white top), taught with `learn_colors.py box` | **Nothing is taught but the floor.** The side face is whatever band at the obstacle's foot does not look like its top; the foot is found again against the floor just in front of the robot | The obstacles are white. A colour rule also fails the first time someone puts down an obstacle it was not taught |
| Low and not the box → walked around without a closer look | **Anything not certainly tall is walked up to and measured** | A low slab and a tall thin thing hide the same floor from one viewpoint (below) |
| One crossing: onto a 60 to 102 mm deep top and off, from stored footstep patterns | **A second crossing, scale across**, for tops 200 to 620 mm deep: footsteps worked out on the measured slab | A stored pattern is written relative to one box; a slab's footsteps depend on its depth |
| `STEP_OVER` | `STEP_OVER` (low bar) and `SCALE_ACROSS` (slab) | The mission layer and Gemini should say which |
| Walk-around could stall beside the obstacle (13 of 420 simulated runs) | Fixed | One scan point read 1 to 2 cm inside the clearance, beside the robot, blocked every heading |

### T6 – obstacles are told apart by shape

**What one look can and cannot tell.** From camera height `hc`, an obstacle hides a stretch of floor from its foot (range `r_b`) to where the ray over its far top edge would land (`r_t`). A box of height `h` and depth `d` gives `r_t = (r_b + d) × hc / (hc − h)`. One equation, two unknowns: the 15 × 380 mm slab and a 70 × 235 mm block hide the same floor from 36 cm. Two limits always hold: it is no taller than `hc × (1 − r_b / r_t)` and no deeper than `r_t − r_b`.

- **While walking** (`terrain/scan.py`): if the obstacle would be over 60 mm tall even if it were 0.7 m deep (deeper than anything crossed), it is certainly tall and is walked around from a distance. The carton is, from 1.2 m. The slab, the bar and the round bar are not, so they get a closer look.
- **Standing at 36 cm** (`terrain/gauge.py`): the height is read from the side face, the band at the obstacle's foot that differs from the top's colour (another colour, or the same in shadow) and from the floor. Then the depth follows from the far edge. No side face seen → height "not read" → walked around. The two limits are still reported.
- **The foot is found twice.** The floor model is general (any floor it was taught); a raw wooden edge on a wooden floor can pass for floor. Cells at the foot are therefore also compared with the strip of floor right at the robot's feet. In the rendered test this is what reads the slab's edge on a wood-coloured floor.
- **Round things** have no side face and no flat top. Their shaded underside can read as a low side face: a 50 mm round bar comes out as "20 to 25 mm tall, 45 to 55 mm deep". The decision refuses a top less than 2.6 times as deep as it is tall, and that depth is under the 60 mm a bar needs anyway.

### T7 – the scale-across gait (`crawl_planner.plan_scale_across`)

The same quasi-static crawl and the same checks as the low bar (balance, reach, joint ranges, whole-leg clearance, torque, ball feet that roll), with footholds on the floor before the slab, on its top, and on the floor beyond.

- **Cycles on a lattice.** The rear pair and the front pair each stand at one x (so the stance is a rectangle after every cycle), at the default width or the widened one. A foot may stand anywhere except across an edge: on the floor at least 51 mm from a face (foot centre), on the top at least 30 mm from an edge. Which x and width each pair takes in each cycle is a shortest-path search (A* over a 10 mm lattice, fewest swings). The path is a proposal: every swing is then put through the full checks, and one that fails is remembered and the path found again from where the checks had got to.
- **The hard step is a front foot coming down past the far edge.** The shin of the foot just set down is still over the edge, so the body has to be well forward; it stays balanced there only with the rear feet about 105 mm behind the front feet and set wide. The search is told so (a cost), which is why it usually finds the way within 40 to 250 swing checks.
- **The taller the slab, the farther out the foot lands** beyond the far edge (1 mm per mm of height above 15 mm), for the same reason.
- **The body stays at one height above the floor.** On the slab the legs are that much more bent, which is what gives the reach to step down. Raising the body with the feet was tried and loses it.
- **Mirror image.** On a skewed slab the side that meets the edge first leads; if that fails the other side leads; then other crossing heights (160, 155, 165 mm).

### T8 – a no-touch step-over is not practical; the round bar is walked around

Asked: can Pupper step over a 50 mm round bar? No. With the same lattice search and the top forbidden as a foothold (`plan_step_over`), a plan exists only for obstacles about 10 mm tall and up to 20 mm deep; none at 15 mm with the standard clearances, and only up to 20 × 20 mm with clearances cut to 10 mm fore-aft and 6 mm above. A foot set down beyond the obstacle has its shin over it (knees point backward), so it must land far out; with the obstacle's depth and the take-off distance that is a stride of about 160 mm for a 50 mm bar, against about 110 mm that can be balanced. v2 found the same for a 30 to 38 mm box and took to using the top as a stepping stone, which a round bar does not offer.

So "step over" means the low flat bar, crossed with one touch of its top per foot, as in v2.

### T9 – the plan counts more ground than a slab crossing covers

In simulation a slab crossing ends 1.9 % short of the plan (0.4 mm a swing; a crawl of 0.24 or 0.48 m on bare floor ends within 2 mm). Uncorrected, with a depth read 15 mm short as well, the feet came down on the far edge in 5 of 40 runs (3 to 10 N, no fall). The slab is therefore planned in the crawl's own measure, everything `1 / advance_scale` farther away (0.98). With that, 60 of 60 runs with measurement errors crossed with one light touch. The number has to be measured on the robot (`deploy.md`, rung 6).

### Still open

- **T3 – touch check.** More useful now: a front foot feeling for the top before the first step would give the height of a slab whose edge does not show, instead of walking around it.
- **A second look from farther off** would tell a round bar from a flat one, and a flat mark from a low slab, by how the hidden stretch of floor changes with distance. Not built.
- **Depths of 102 to 200 mm** have no crossing: too deep for the stored patterns, too shallow for four feet.
- T1 (range sensor) and T5 (prefer crossing) as before.

## What changed from v1

| v1 said | Built | Why |
|---|---|---|
| Box 30–38 mm tall, 40–65 mm deep, stepped over cleanly | **Box 19–25 mm tall, 76–102 mm deep (a "1×4": 19 × 89 mm); each foot steps onto the box top and off again** | v1's limits used thin stick legs. Measured from the meshes, the thigh and knee housings are 30 mm in radius and the shin 13–18.5 mm. With those, a front foot cannot be carried beyond even a narrow box while the centre of mass stays inside the other three feet (knees point backward, so the shin sweeps the box). Using the top as a foothold makes it possible, and a deep top is then an advantage |
| Hip height 170 mm | 155–165 mm, chosen per footstep pattern | Higher costs reach; the stored patterns were found at these heights |
| 12 swings, 20–30 s | **26–35 swings, 45–62 s** including about 20 s of counted approach steps | Stepping on and off doubles the footsteps; the approach from where the camera can still see the box (36 cm) is four crawl cycles |
| Footsteps generated by a fixed sequence | Footstep patterns found offline by search, stored relative to the box, replayed and fully re-checked on the measured box | A greedy sequence dead-ends; a search from scratch takes 10–60 s, too slow on the robot |
| Feet stay where planted | **Ball feet roll**: the foot centre moves as the shin rotates over it; modelled in search and trajectory | Found in simulation: without it the robot drifted 30 mm and pitched 5° |
| Reuse the monorepo's forward controllers and the joystick's e-stop list | **Own three forward command controllers** (`robot/forward_controllers.yaml`), own e-stop guard | The course's installed config loads no forward controllers, and the joystick e-stop only relaxes its own list |
| Raise kp against sag | Gains left at the walking controllers' start-up values (kp 7.5, kd 0.25); sag handled by feed-forward (`q_cmd = q_des + τ/kp`) plus a velocity lead | Course rule: do not raise gains |
| Walk to 0.25 m and re-measure | Measure once at 36 cm (30–42), then count crawl steps | At 25 cm the box's foot is below what the camera sees past the chin |
| Touch check, VERIFY state, tilt-rate abort, time-of-flight sensor | Not built | Not needed in simulation; candidates if the robot shows a need |
| Camera calibration with a height card | Box of known height at 3–4 ruler distances; solves height, pitch and fore-aft position | One prop, already on hand |

Everything else in v1 (ground-plane geometry, two-colour box, floor colour model, tangent steering, decision by thresholds with a guard, quasi-static crawl with shift-then-lift, guarded touchdown, abort rules, mux position, dodge interactions) was built as planned.

## Goal

**When Pupper meets an obstacle in its path, it measures it and picks one of two responses:**

1. **Short enough → step over it** with a planned, statically stable gait. No lunging, no jumping.
2. **Too tall → walk around it** by the shortest safe path, without stopping and without a boxy detour.

If it cannot tell, it treats the obstacle as too tall. Going around is always the safe default.

## The box to procure

### Step-over box

| Dimension | Get | Verified range | Why |
|---|---|---|---|
| **Height** | **19 mm** (¾ in) | read as up to 27 mm (the reading + 3 mm must be ≤ 30) | Leg clearance while a foot is carried over the edges |
| **Depth** (along travel) | **89 mm** (3½ in) | 76–102 mm at any height up to the limit; 60–70 mm only for the lowest boxes (see "Simulation results") | A foot on the top rolls forward 15–25 mm as the body passes over it and must stay 12 mm from both edges; a deep top leaves room for error in where the box is |
| **Width** (across the path) | **500–600 mm** | at least 400 mm | Feet are 200–260 mm apart while crossing; this leaves margin and makes stepping over the sensible choice |
| Weight | 250 g or more, or taped down; non-slip underside | 250 g tested | A foot stands on it |
| Edges and top | square edges, flat rigid top | – | The measurement assumes a vertical front face |
| Colour | matte; **sides one colour, top another** (blue tape sides, white top), both unlike the floor | not neon green | Height from the side band, depth from the top band |

- **Easiest thing that fits:** 55 cm of "1×4" timber (19 × 89 mm), blue painter's tape round the sides, white on top.
- A "1×3" (19 × 64 mm) is too shallow to rely on: at 64 mm only boxes read at 19 mm or less are accepted. A "2×3" or "2×4" (38 mm) is too tall.

### Tall box

At least 80 mm tall (250 mm or more is unmistakable: it crosses the camera's horizon), any colour unlike the floor, heavy enough not to be pushed.

### Grey zone

Two-colour boxes read between 27 and about 80 mm, or shallower than 70 mm unless very low, are refused and walked around. Keep demo obstacles out of this band.

## Fixed facts

### Legs and body (public model, checked against MuJoCo in `tests/test_kinematics.py`)

| Quantity | Value |
|---|---|
| Thigh (hip to knee) | 84.5 mm |
| Shin (knee to foot centre) | 88 mm in the fore-aft plane |
| Foot | ball, 20 mm radius (21.5 mm moulded) |
| Leg envelope (from the meshes) | thigh and knee housing 30 mm radius; shin 18.5 mm near the knee, 13 mm near the foot |
| Hip-to-foot distance, straight leg | 173.4 mm |
| Knee direction | backward on all four legs |
| Default stance (`0.26, 0, −0.52`) | hip axis 161 mm above the floor; front feet +66 mm, ±101 mm; rear −84 mm, ±90 mm |
| Mass | 3.17 kg, plus camera plate, cameras and mics (estimated 195 g; **weigh them** and set `crawl.extra_masses`) |
| Hardware limits | 3 N·m, kp ≤ 10, kd ≤ 1, position commands 0.1 rad inside the joint ranges |

- **Lifting a front foot is the unstable case**, and head hardware makes it worse. The gait widens the rear stance by 40 mm and steps the rear feet up first.
- **Mic bars limit leg swing.** Every planned pose is inside the union of the walking envelope and the any-knee ranges from the design-2 notes.

### Sensing

Front fisheye (double-sphere model from the course's `camera_params.yaml`, about 4 px per degree, fixed white balance), IMU for levelling and heading, joint positions. No depth sensor, no odometry.

### Controllers (as installed by the course)

- Walking: `neural_controller/NeuralController` instances (2 s pose hold + 2 s fade-in on activation; tilt cut-out).
- **No forward command controllers are loaded.** `robot/spawn_controllers.py` loads three (`terrain_position_controller`, `terrain_kp_controller`, `terrain_kd_controller`), inactive. A forward controller writes nothing until its first message and keeps its last values, so the hand-over is sequenced (below) and an independent guard handles the e-stop.
- `cmd_vel_mux`: priority list, 50 ms timer, a source silent for 500 ms is dropped, **any message (even zeros) makes a non-joystick source active** – so the terrain node publishes nothing while it has nothing to do.

## How the height is gauged

### Geometry

With the camera at height `hc` and angles measured below the horizon in a view levelled with the IMU:

- **Distance**: `D = hc / tan(θ_foot)`, where the near face meets the floor.
- **Height**: `h = hc × (1 − tan(θ_top) / tan(θ_foot))`. A ratio, so a pitch error nearly cancels.
- **Horizon shortcut**: a top at or above the horizon is taller than the camera.

### Why the box is two colours

The camera looks down on a low box and sees its top. With one colour the highest visible edge is the far top edge, which reads a 19 mm board as 55 mm; a flat mark on the floor reads the same. Side colour → height; top colour → depth; slope of the foot line → skew. Anything low that is not the two-colour box is walked around.

### Pipeline (`terrain/`, run by `robot/terrain_node.py`)

1. Frame decoded at half size, resampled onto bearing × depression with the IMU's tilt taken out (`camera_model.PolarView`), 8 Hz.
2. Each cell labelled floor / obstacle / box side / box top / ball / invalid (`floor_model`): floor by a hue–saturation histogram plus a grey brightness histogram, adapted slowly from the patch ahead while it reads as floor.
3. Per bearing, the nearest run of non-floor cells is the obstacle's foot; the run above gives apparent height (`scan`). Neighbouring bearings are grouped into obstacles: tall (≥ 80 mm or to the horizon), the box (side colour at its foot), or low unknown.
4. For the box, standing still at 36 cm: a fine view (0.5° × 0.2°) of the full-size frame, per-bearing foot, side/top boundary and far edge → height, depth, distance, skew, width, ends; plus the fraction of the 35 cm strip beyond the box that reads as floor (`gauge`). Medians over 10 frames.

### Calibration

`tools/learn_colors.py` (floor, box colours from the non-floor pixels split in two, a check picture) and `tools/calibrate_camera.py` (box of known height at 3–4 ruler distances → camera height, pitch, fore-aft position by least squares on the foot and top angles).

## Decision rule (`terrain/decision.py`)

Standing still, squared up, 29–43 cm from the box. **Step over only if every line holds; otherwise go around, and say which line failed on `/terrain/event`.**

| Check | Threshold |
|---|---|
| Height | reading + max(3 mm, 2σ) ≤ envelope(depth), capped at 30 mm (`height_cap`) |
| Depth | measured (top colour seen) and inside the envelope table, 60–102 mm ± 3 |
| Width, centring | ≥ 400 mm; ends at least 80 mm beyond the feet line on both sides |
| Squareness | within 5° after aligning (up to 5 turn/sidestep pulses) |
| Landing strip | ≥ 85 % floor at the worst bearing, and no remembered obstacle point in it |
| Readings | ≥ 10 frames, height σ ≤ 4 mm |
| Robot | level; no ball tracked by the dodge node |

- Reads under 10 mm: walked over with the normal gait, and not stopped for again while passing.
- `prefer_step_over` (default true): a steppable box is stepped over although going round is faster.
- The crossing is planned for the height as read; the 3 mm guard is only in the envelope check. The gait's 8 mm clearance above the top covers an under-read, and a foot stepping onto the top stops when it feels it.

## Step-over gait (`terrain/crawl_planner.py`, `player.py`, `handover.py`)

### Principles

- **Quasi-static crawl**: one foot in the air at a time; body moved over the other three before each lift.
- **Balance**: centre of mass (with payload, per pose) at least 15 mm inside the support triangle, 35 mm aimed for; static torque ≤ 2.2 N·m per joint.
- **No nose-dives**: while a rear foot is in the air the centre of mass stays at least 26 mm behind the line through the two front feet. Tipping forward over that line is the one fall nothing catches, and front-leg sag feeds it. (Found in simulation: a pattern that crossed the rear feet with the front feet tucked under the body tipped when the box was read 2 mm low.)
- **Feet on the box top**: set down 42 % of the depth in from the near edge, and kept at least 12 mm from either edge through all the rolling that follows. (Found in simulation: landing in the middle let feet roll to the far edge; landing in the near third left no room for a distance error.)
- **Clearance**: no part of any leg's envelope within 15 mm of the box fore-aft or 8 mm above it; a foot on the top may have its shin 5 mm from the edge.
- **Swing path**: found by flooding the foot's free space (reachable, inside joint and mic-bar ranges, whole leg clear) from take-off to landing, then straightened. Lift at least 18 mm.
- **Rolling feet**: every body shift moves the planted feet's centres; search and trajectory carry that along.
- **Speed**: body 0.10 m/s, foot 0.25 m/s, final descent 0.12 m/s; the time base is stretched wherever a joint would exceed 4 rad/s (a nearly straight leg lifting its foot).
- **Sag**: commanded angle = planned angle + load torque / kp + (kd/kp) × planned velocity.
- **Touchdown**: in the final descent a position error over 0.035 rad for 30 ms means the foot has arrived; it is held there.
- **Stored patterns**: each is tried as stored and mirrored left-right, nearest box first, the one that worked last time before the others; every step of the candidate is re-checked on the measured box.
- **No plan, no motion**: a box outside what the stored patterns cover yields no plan (`allow_search` can enable a slow search).

### Sequence (typical, 19 × 89 mm box from 36 cm)

| Swings | What |
|---|---|
| 1–12 | Counted approach: three crawl cycles (rear-left, front-left, rear-right, front-right), 70–80 mm each, at the default stance width, ending with the front feet 20 mm before the take-off line |
| 13–14 | A rear foot up and out to the wide stance, then a front foot **onto the box top** |
| 15–16 | The same on the other side |
| 17–22 | Rear feet onto the top, front feet down beyond the box, in the order the pattern found |
| 23–30 | Front feet forward to their final places (before the rear feet leave the top), rear feet down beyond the box (15 mm past the landing line), all four back to the default stance |
| – | Body centred and lowered to the standing height; walking controller re-activated |

### Taking and returning the legs (`terrain/handover.py`)

1. Stream position = measured angles, kp = 0, kd = 0.6 and request the switch (walking controller off, three terrain controllers on, STRICT). Messages before activation are dropped by the controllers; whatever lands first after activation carries no stiffness.
2. Switch confirmed: hold the angles measured then, ramp kp to 7.5 over 60 ms.
3. Player blends into the plan (0.6 s) and plays it.
4. Plan done: keep holding while the walking controller is switched back in (its own 2 s hold + 2 s fade-in follow).

Body drop through the hand-over in simulation: 2–3 mm.

### Abort rules (`terrain/player.py`, `robot/stepover_node.py`, `robot/estop_guard.py`)

- Tilt over 20°, or a planted leg's joint more than 0.15 rad off for 0.2 s → joints soft (kp 1.5, kd 0.6 over 1.5 s), robot settles, controllers released. The walking controller is not restarted automatically.
- Swinging leg more than 0.12 rad off for 0.1 s (it has met something) → backed out along its path, foot put back, then as above.
- Joint states or IMU older than 0.15 s, controller switch failed or timed out → limp (kp 0, kd 0.1).
- Gamepad e-stop (`/emergency_stop`) → limp at once; the guard process does the same independently and also if the step-over node's heartbeat stops.

## Walking around (`terrain/steering.py`, `state_machine.py`)

1. **Trigger**: a tall or unknown obstacle in the lane (±18 cm about the driver's direction of travel) within 1.2 m.
2. **Memory**: scan points are kept in a small body-frame map moved by the commanded speed and the IMU's yaw, so the obstacle is still known when it leaves view alongside.
3. **Steer**: the clear heading nearest the goal heading, with 22 cm between obstacle points and the body centre over a 1 m look-ahead; walk and turn together at up to 0.35 m/s and 1 rad/s. The side is chosen by detour cost and then kept.
4. **Leave** when the original heading is clear and the robot is facing it; go silent.
5. **Blocked both sides, or turning on the spot for 6 s without a way** → stand still, report (`HOLD`), retry after 2 s.

Simulated (`sim/avoid_sim.py`, obstacles 0.2–0.6 m wide met centred and off-centre): path 1.00–1.04 × the shortest clear path, never closer than 0.23 m, no stops, the short side chosen.

## State machine

```
CRUISE → (tall/unknown in lane) AVOID ─────────────────────────────→ CRUISE
       → (two-colour box)       APPROACH → GAUGE → (no)  AVOID
                                          ↑   ↓ → (ignore) CRUISE
                                        ALIGN ←┘ → (yes) PLAN → STEP_OVER → CRUISE
any state → HOLD (stand still, report)
```

- It acts only while a driver is commanding motion (the "intent" topics), and lets go within a second when the driver stops.
- A dodge (`/dodge/event`) during AVOID/APPROACH/GAUGE/ALIGN drops every measurement and starts over.

## Interfaces

| Item | Use |
|---|---|
| `cmd_vel_mux` | `inputs`: joystick > `/dodge_cmd_vel` > **`/terrain_cmd_vel`** > Gemini > survivor > nav > person follower > test driver (`robot/terrain_mux.yaml`) |
| Intent | the drivers' own topics (`fsm.intent_topics`); the heading at take-over is the heading returned to |
| `/terrain/state`, `/terrain/event` | for the mission layer and Gemini; `STEP_OVER` means "cannot dodge" |
| `/terrain/obstacles`, `/terrain/scan` | obstacles in view; range per 5° of bearing – the free-space input the dodge plan asks for |
| `/terrain/step_over`, `/terrain/crossing_status` | JSON requests to and reports from the step-over node |
| Gemini's `robot_api.py` | the crossing returns the legs to the same walking controller, so its state stays valid; a trick requested mid-crossing is refused by the controller manager (interfaces claimed) |

**Interactions with dodging:** no dodging mid-crossing (not in a walking controller), and no crossing starts while a ball is tracked; balls are labelled by colour and never treated as obstacles; one more image pipeline on the Pi at 8 Hz and half resolution.

### Folder layout

```
terrain-agility/
  plan.md, README.md
  terrain/   kinematics, pupper_model (generated), stability, crawl_planner, player, handover,
             camera_model, floor_model, scan, gauge, decision, steering, state_machine, config
  robot/     terrain_node, stepover_node, estop_guard, spawn_controllers, forward_controllers.yaml,
             terrain.launch.py, standalone.launch.py, terrain_mux.yaml
  tools/     check_robot, learn_colors, calibrate_camera, record_frames, step_over, walk_forward, plan_box
  sim/       crossing_sim (MuJoCo), build_templates, render_fisheye, avoid_sim, extract_model
  config/    terrain.yaml, crawl_templates.json
  tests/     the test suite; fake_ros.py runs the real nodes off the robot
```

## Simulation results

MuJoCo, Pupper v3 model with the measured leg envelope as collision shapes, stiff foot contacts, free 0.6 kg box, camera plate and mics as extra mass, the node's own hand-over sequence, joint noise 2 mrad.

- **Planning envelope** (5 heights × 8 depths × 5 starting distances of 30–42 cm = 200 cases, 18 stored patterns):

  | Box depth (mm) | 60 | 64 | 70 | 76 | 82 | 89 | 95 | 102 |
  |---|---|---|---|---|---|---|---|---|
  | Tallest box that plans from every distance, with every lower one (mm) | 28 | 22 | 31 | 31 | 31 | 31 | 31 | 31 |

  The decision caps this at 30 mm and requires reading + 3 mm to be inside it, so boxes read at up to 27 mm are crossed when 70–102 mm deep. Random boxes in 19–27 × 76–100 mm with up to 4° of skew: 40 of 40 planned, in 2.4 s median and 19 s worst on a desktop core (the first time; the pattern that worked last is tried first afterwards).
- **Exact measurements**, 30 random boxes (19–27 × 76–100 mm), distances (31–41 cm), skews (±4°), payload ×0.6–1.4 of what the planner assumes: 30 of 30 crossed; body tilt at most 6.2°; the box never moved; peak torque 1.9 N·m; 50–62 s. In 3 runs the last rear foot brushed the box's far face on its way down (4–6 N).
- **With measurement errors** on top (height ±3 mm, distance ±8 mm, depth ±8 mm, skew ±1.5°), 40 runs: 40 of 40 crossed; tilt at most 6.3°; box moved at most 0.2 mm; 47–60 s. In 5 runs a rear foot or shin brushed the far face (4–8 N). No abort, no fall.
- Distance error alone, −8 to +8 mm, on 22 × 76 and 22 × 89 mm boxes: all crossed, tilt ≤ 4°.
- Box fixed to the floor, and a 250 g box (25 × 89 mm): crossed, tilt under 8°.
- Hand-over: 2–3 mm body drop; e-stop, stale data and a failed switch all end limp with nothing re-activated (tested through the node code).
- Perception on rendered fisheye frames through JPEG: height ±4 mm, distance ±10 mm, depth ±12 mm, skew ±2°; camera calibration within 5 mm and 0.6°.

What simulation cannot tell: real joint sag, friction and backlash; foot grip; how still the walking policy stands; switch and planning time on the Pi; colours and lens model on real frames. The bring-up ladder in the README is ordered to find these out with the least risk.

## Operating envelope

**Demo spec:** flat, matte, uniform floor in steady light; one obstacle at a time with 1 m of clear floor around it and 35 cm clear beyond the step-over box; boxes as specified, square to the path within about 20° on approach (the robot squares up to 5°); approach at up to 0.4 m/s; no throws during a crossing.

**Out of scope:** stairs, slopes, gaps, soft or moving obstacles; boxes read over 27 mm as step-over targets; obstacles the colour of the floor; glass and mirrors.

**Success criteria on the robot** (unchanged in spirit from v1, times updated): gauge within ±4 mm at 36 cm and the right call 20 of 20 for boxes ≤ 25 mm and ≥ 60 mm; 9 of 10 crossings with no fall and the box moved under 10 mm, under 70 s each from the stop; 9 of 10 walk-arounds with no contact, path ≤ 1.25 × the tangent path, no pause over 1 s; under one false stop per 5 minutes on empty floor.

## Bring-up order on the robot

README section 5 is the step-by-step. In outline: check → plan without moving → stand at crossing height (tests the hand-over) → e-stop test → crawl on clear floor (tests tracking and sag) → cross with the box placed by ruler → walk around the tall box → the whole chain from the camera → with Gemini and the dodge node.

**If time runs short:** walking around (camera + walking policy only) and stepping over from a ruler-placed start are each a demo on their own; the camera-triggered crossing joins them.

## Things that are easy to miss

1. **The legs are bulky.** Clearance is set by the thigh and knee housings, not the foot.
2. **Front-foot lifts are unstable by default**, and the cameras on the head make it worse: weigh the payload.
3. **Ball feet roll.** Feet "planted" on the floor still move millimetres per body shift; over 27 swings it adds up, which is why the last feet get extra room.
4. **Motor sag is as big as the clearance margin** at stock gains; `crawl.sag_gain` scales the compensation.
5. **The camera sees the top of a low box**, so a one-colour box over-reads.
6. **Blind zone under the chin**: the last 27 cm is counted steps, so the distance reading must be good to under 1 cm.
7. **The forward controllers have no tilt cut-out and hold their last command.** Hence the guard process.
8. **Any message on a mux input takes control.** Stay silent in CRUISE.
9. **The walking policy must stand still at zero command** for a crossing to start.
10. **A skewed box is a different box for each foot**: lines are computed at each foot's own lateral position (a bug here made every skewed box unplannable until a random test caught it).
11. **No dodging mid-crossing**, for almost a minute.
12. **Going around is faster** than stepping over; the step-over box must be wide for the behaviour to make sense.
13. **Shadows and reflections read as "not floor"**; teach the floor model the patches that vary.

## Open decisions

- **T1 – a range sensor** (VL53L5CX) low on the chest would see into the blind zone and make the distance independent of colour calibration. Decide after the first camera-triggered crossings.
- **T3 – touch check** before the first step onto the box, if the distance reading proves the weak point.
- **T5 – prefer stepping over, or take the faster option** (`prefer_step_over`).
- **Crawl speed**: `crawl.time_scale` below 1 once the robot has shown it tracks the plan.

## Sources

- Pupper v3 model (leg geometry, joint limits, mass, meshes): https://github.com/G-Levine/pupper_v3_description
- Pupper v3 monorepo (controllers, velocity mux, joystick e-stop): https://github.com/Nate711/pupperv3-monorepo
- Course repos read for the installed configuration: cs123-stanford/pupper_gait_deploy, gemini_pupper, follow_me_pupper
- Kamon, Rimon, Rivlin, TangentBug: A Range-Sensor-Based Navigation Algorithm (IJRR 1998): https://www.cs.technion.ac.il/~ehudr/publications/tangentbug-rangesensorbased-navigation-algorithm
- Ulrich, Nourbakhsh, Appearance-Based Obstacle Detection with Monocular Color Vision (AAAI 2000): https://ri.cmu.edu/?p=16753
- Kolter, Rodgers, Ng, A control architecture for quadruped locomotion over rough terrain (LittleDog, ICRA 2008): https://www-cs.stanford.edu/group/littledog/pubs/kolter-icra08.pdf
- Usenko, Demmel, Cremers, The Double Sphere Camera Model (3DV 2018): https://arxiv.org/abs/1807.08957
