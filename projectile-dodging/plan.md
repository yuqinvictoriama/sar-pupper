# Projectile dodging – implementation plan (v2)

Updated 2026-11-26. Supersedes the 2026-11-16 plan. **Implemented in this folder** (see `README.md`; deployment steps in `DEPLOY.md`). On the Pupper it has been through camera setup, calibration and the first armed throws; what those showed, and what has not yet been re-measured on the robot, is under Implementation status.

## Goal

**Pupper dodges a thrown ball arriving from any direction, 360° around it.** Every design choice below is checked against that: coverage, tracking, and the escape move all have to work at every bearing, not only in front.

Dodging is one component of the search-and-rescue robot (alongside obstacle avoidance and route planning). It is a reflex: it runs on the robot, interrupts whatever else is driving, moves the robot out of the way, then hands control back.

## Fixed facts

### Projectile
- Neon green foam pickleball, solid foam inside, shallow dimples, one visible seam.
- **74 mm across.** The request said "74 mm circumference", but the spec image and the photo show 74 mm as the diameter (circumference would be about 232 mm). This plan uses 74 mm diameter – measure one ball with calipers to confirm.
- 0.917 oz = **26.0 g**.
- Drag (sphere, Cd about 0.47): about 1.7 m/s² deceleration at 6 m/s, 0.75 m/s² at 4 m/s. It makes the ball arrive slightly later than a no-drag estimate, which is the safe direction.

### Cameras (360° coverage)
| Camera | Faces | Covers (bearing from nose) | Ball size at Z metres | Notes |
|---|---|---|---|---|
| Native fisheye (IMX296 global shutter, CSI) | front | 0° ± 110° nominal (code uses 220°) | about 17/Z px at 1400×1050 | Red-tinted; runs fixed white-balance gains [1.4, 3.3]. 60 fps is the sensor's full-frame limit. Owned by `camera_ros`. |
| ELP global shutter USB, 120° × 2 | rear, yawed ±57.5° | 180° ± 117.5° | about 27/Z px at 1280 wide, 41/Z at 1920 | 5° overlap at the rear seam. Vertical FOV about ±44–47°. No capture code exists for these yet. |

#### Camera positions (from the CAD)

Calculated from `Whole pupper.step` (the Arducam IMX296 part) and `design_option2_cadquery.py` (the printed angled plate, mounted rear-facing). Given in the robot's `base_link` frame used by the URDF: x forward, y left, z up, origin on the centre plane at hip-axis height. Positions are the front of each lens.

| Camera | x (mm) | y (mm) | z (mm) | Optical axis |
|---|---|---|---|---|
| Fisheye | +157.9 | 0.0 | −1.5 | straight ahead, level |
| ELP left | +45.0 | +31.5 | +152.1 | 122.5° to the left of straight ahead (57.5° off straight back), level |
| ELP right | +45.0 | −31.5 | +152.1 | 122.5° to the right of straight ahead, level |

- **Relative to the fisheye, each ELP is 153.5 mm higher, 112.9 mm further back, and 31.5 mm to its own side** – 193 mm apart in a straight line. The two ELPs are 62.9 mm apart.
- The fisheye sits at the chin, level with the hip axes. The ELPs sit on top of the head (56 mm above the head top) and look backward over the body.
- All three optical axes are horizontal: the ELP plates are vertical, as printed. ELP vertical view is therefore about ±44–47° around the horizon.
- CAD frame for reference (`Whole pupper.step`, mm, X right, Y up, front at −Z): fisheye lens front (0.44, −69.53, −157.89); ELP lens fronts (0.44 ∓ 31.46, 84.0, −44.97). `base_link` = CAD point (0.44, −68.07, 0).
- Accuracy: the ELP figure assumes a 20.6 mm deep camera module (the CAD stand-in) and the fisheye lens is a plain cylinder in the CAD, so expect ±5 mm and a few degrees against the built robot. These are starting values; Phase 2 calibration replaces them.

What the offsets mean for dodging:
- **Parallax.** A ball 1 m away is seen at bearings up to 11° apart by the fisheye and an ELP (2.7° at 4 m), and up to 8.7° apart in elevation. Detections cannot be merged as raw bearings; each is first placed in 3D using its camera's position and the size-based range, then tracked about one body origin.
- **Stereo where they overlap.** In the side overlap zones the fisheye and an ELP are a real 193 mm baseline, so range can be triangulated there (roughly ±0.1 m at 2 m). Optional, but it is free information in the easiest sectors.
- **Self-view.** The ELPs look back over the body, so the body, top tray, ears and rear mics fill the lower middle of both images (mics about 20° below the horizon). Balls arrive from above the horizon, so this costs little, but those pixels need a static mask.

- Fisheye and ELPs overlap by about 47° on each side (bearings 62.5°–110°). Total coverage is 360° with no gap, provided the fisheye is usable to at least ±65°.
- The fisheye is only *needed* for the front ±62.5°. Beyond that an ELP sees the same ball with more pixels, so the rim of the fisheye (where quality is worst) is never relied on.
- Fisheye resolution is roughly uniform at 4 px per degree; ELP is 6.5 px per degree at centre (1280 wide). **The front is the weaker sensor**: fewer pixels on the ball and a lower frame rate.

### Robot motion (from the codebase)
- Default walking policy command limits: **vx ±1.5 m/s, vy ±0.5 m/s**, yaw ±2 rad/s (`pupper_gait_deploy/policies/default_policy.json`, `command_clip`). Sideways is three times slower than forward/back.
- The policy caps vx/vy at 0.49 m/s unless `speed_cap` is raised (as `follow_me_pupper/robot/speed_cap.yaml` and Gemini's `gemini_walk` controller do).
- Policy step is 52 Hz. Heading hold keeps the nose pointing the same way during a pure translation.
- Tilt e-stop: `max_body_angle` 1.0 rad on the default controller.
- Body about 22 cm wide, 25–30 cm long, about 20–25 cm tall.
- **Walking controller: `gemini_walk`** (decided). It is the same default policy, but with the speed cap already raised to 1.5 m/s forward *and* backward (the course default allows 0.49 m/s forward and 0.3 m/s backward). The Gemini launch that spawns it also already routes the joystick through `cmd_vel_mux`, and `robot_api.py` treats it as the locomotion state for fall and e-stop recovery. The mission uses Gemini Live anyway, so dodging and the mission share one controller with no switching. Bench tests without Gemini spawn the same controller from our launch file.

## Approach (unchanged in principle from v1)

- **Perception = physics, not a learned model.** Known ball size and colour → bearing, angular size, growth rate → time-to-contact and whether it is on a collision course. (Idea 3.)
- **Action = state machine with preprogrammed escapes.** (Idea 2, but binned on the predicted approach direction, not on where the ball is in the image now.)
- **Neural network = optional upgrade** for rejecting green clutter, not the core. (Idea 1.) The Hailo stays free for person detection.

What changes in v2: everything is expressed in a single body-frame bearing space so three different cameras feed one tracker, and the escape move is chosen per bearing because Pupper's speed is very different along its two axes.

## Operating envelope

These are the limits we design and test to. The numbers marked *est.* are placeholders until Phase 0 measures them; the envelope is then tightened or relaxed.

### Why these limits
- **Flight time is what matters, not speed.** A ball released at about 1 m and thrown flat takes about 0.43 s to drop to robot height, from any distance. An underhand lob that rises 0.5–1.0 m above the release point takes 0.84–1.05 s. Asking for a lob is the simplest way to guarantee time.
- **Time needed = react + clear.**
  - React (ball visible → robot starts moving): *est.* 0.10–0.15 s on the ELPs, 0.15–0.25 s on the fisheye.
  - Clear (move far enough that the ball misses): steady-state 0.16–0.39 s depending on bearing, plus acceleration lag; *est.* 0.35–0.45 s for most bearings, 0.55–0.65 s when the ball comes from dead ahead or dead astern (only the 0.5 m/s sidestep helps there).
- **Detection range.** The ball needs to be about 6 px across for a reliable blob: inside 2.8 m on the fisheye, 4.5 m on an ELP at 1280 wide.

| Sector (bearing from nose) | Camera | Est. time needed | Time the lob is in view (3–4 m throw) | Outlook |
|---|---|---|---|---|
| Front cone, 0° ± 15° | fisheye | 0.70–0.90 s | 0.65–0.84 s from 3 m; 0.49–0.60 s from 4 m | Marginal – weakest sector |
| Front diagonals, 15°–60° | fisheye | 0.50–0.70 s | same | OK from about 3 m |
| Sides, 60°–120° | fisheye + ELP | 0.45–0.55 s | 0.84–1.05 s | Comfortable |
| Rear diagonals, 120°–165° | ELP | 0.45–0.60 s | 0.84–1.05 s | Comfortable |
| Rear cone, 180° ± 15° | ELP (seam) | 0.65–0.80 s | 0.84–1.05 s | OK |

### Tier A – the demo spec (must work)
- **Distance:** thrower stands 3–4 m from the robot. Tape two rings on the floor.
- **Throw:** underhand lob, released at waist-to-chest height (about 1 m).
- **Height:** the ball peaks between the thrower's shoulder and about 0.3 m above their head (1.5–2.0 m above the floor). Not lower (too little time), not higher (leaves the rear cameras' view too early).
- **Speed:** follows from the above – about 5–6 m/s launch speed. No overhand or flat throws.
- **One ball in the air at a time**, at least 3 s between throws.
- **Robot:** standing or walking at up to 0.3 m/s, in a walking controller (not mid-trick, not relaxed).
- **Arena:** flat open floor, at least 1 m clear around the robot, steady indoor lighting, no other neon green or yellow-green objects in view, thrower not hidden behind anything.
- **Any bearing, 360°.**

### Tier B – stretch
- Flat or overhand throws up to about 7 m/s from at least 4 m, side and rear-diagonal sectors only.
- Robot moving faster than 0.3 m/s.
- A second ball within 1–3 s of the first.

### Out of scope
- Throws from closer than 2 m, or hard direct throws (flight under about 0.45 s).
- Two balls in the air at once.
- Balls dropped from directly overhead, bounced off the floor or a wall, or rolled.
- Darkness, strongly coloured lighting, or a thrower the cameras cannot see.

### Success criteria
- 20 on-target Tier A throws per sector (front, front-diagonal, side, rear-diagonal, rear). A baseline run with dodging disabled records how many of the same thrower's throws actually hit.
- Target: at least 80 % avoided in side, rear and diagonal sectors; at least 60 % in the front cone.
- False dodges: fewer than 1 per 5 minutes with no ball thrown, including while walking.

## Pipeline

Everything after detection works in **body-frame angles**, so the tracker does not care which camera saw the ball.

1. **Capture**
   - ELPs: V4L2, MJPG, one capture thread each, at the fastest mode with at least 1280 px width. Manual short exposure, fixed gain and white balance. Timestamp at capture.
   - Fisheye: 1400×1050 from the camera stack (see decision D1). Do not undistort the whole frame – too slow and not needed.
2. **Detect (per camera)**
   - **Colour cast removed first.** Each picture's blue and red channels are scaled so that its bright, nearly colourless parts (walls, ceiling, anything white) come out neutral; the scaling follows the room slowly and ignores strongly coloured pixels, so a passing ball does not pull it. Without this the ball's hue depends on the camera's white balance: within one evening the same ball read as hue 51 under a greenish cast and hue 33 under a yellow one, and under the yellow cast the white walls themselves passed for ball colour.
   - Colour test with a **separate calibrated profile per camera** (the fisheye's red tint and fixed gains shift neon green toward yellow). A pixel is ball-coloured when its hue is in range and it is saturated, bright and *colourful in absolute terms* (saturation × brightness). Saturation alone is a ratio and is large by accident in the dark, noisy parts of a picture; in the first real ELP picture 4 % of the room passed a saturation test that the ball only just passed, against 0.01 % for the absolute measure.
   - Morphological open → connected patches. Every patch is judged, largest first (a far ball must not be crowded out by bigger patches). A patch must hold some strongly coloured pixels, be at least as big as the ball would look at the greatest throwing range, and, once big enough to judge, be solid (area ÷ area of its convex outline) and oval (outline ÷ best-fitting ellipse). These shape tests replace an outline-length test that failed on the real ball: noise, dimples and the seam make the edge ragged, which lengthens the outline without changing the shape.
   - Size and centre come from the filled convex outline, so dimples and glints do not shrink the ball.
   - Static mask per camera for robot parts in view (mics, ears, body edge).
   - Once tracking, search only a region around the predicted position.
3. **Measure → ray**
   - Sub-pixel centroid and blob extent, converted through the camera model (double-sphere for the fisheye using the existing `camera_params.yaml`; pinhole + distortion for the ELPs) into a ray and an **angular diameter θ**, then through the camera's mounting pose (table above) into a 3D point in `base_link`: camera position + (D / θ) × ray direction. Azimuth and elevation are taken from that point, so the 193 mm between cameras is accounted for.
   - Angular diameter is used instead of pixel diameter because a rectilinear 120° lens stretches the ball near the image edge and the fisheye compresses it.
4. **Track (one tracker, body-centred)**
   - As built: a Kalman filter on the ball's 3D position and velocity relative to the robot, assuming free flight (gravity plus drag). Each measurement is the 3D point from step 3 with a small error across the line of sight and a large one along it. Azimuth, elevation, angular size and their rates are read off this state; filtering them directly was the original idea, but a lobbed ball's elevation is not constant on a collision course, and the 3D form handles that and the camera offsets in one go.
   - Three detections to confirm a track. Detections from two cameras merge into one track; a track hands over between cameras without resetting. A track that goes unseen is dropped after 0.25 s, except a ball seen in flight (fast, and moving across the line of sight), which is followed on its arc for 0.6 s: a lob leaves the top of a rear camera's picture in mid-flight.
   - The filter runs in a gravity-aligned frame built from the IMU orientation (`/imu_sensor_broadcaster/imu`), so body pitch and roll while trotting do not look like ball motion.
5. **Predict**
   - The track is flown forward on its arc until it lands. Time-to-contact τ = when it first comes within the margin of the robot's outline (a box in `base_link`); miss distance = the closest the ball's surface gets. For a ball coming straight in this is the same as τ = θ / θ̇.
   - Threat = predicted miss within margin (0.05 m) plus 1.5 σ of the prediction's own uncertainty, and the ball really is a moving ball: closing at over 1 m/s after removing the robot's own walking speed, and moving over the ground at over 1.5 m/s on a straight-line fit (a still object hanging above the robot would otherwise look like it is falling onto it). A ball in flight that is still climbing is exempt from the closing test: early in a lob it mostly goes up, and waiting for it to close in cost about 0.3 s.
   - Approach direction and descent angle come from the predicted velocity at contact.
6. **Decide (state machine)**
   - SEARCH → TRACK → THREAT → DODGE → RECOVER → SEARCH.
   - The dodge fires when: a threat is predicted, and τ is below (time needed for that bearing + 0.6 s; 0.3 s until 2026-11-26). With time in hand it waits for a second consistent verdict and a sideways prediction good to 0.2 m; once τ is down to the time needed it goes at once. Trigger early rather than timing it finely – τ is only accurate late.
   - Ignore anything not growing: a ball lying on the floor stays green and in view forever.
   - Refractory period of about 1 s after a dodge so the bouncing ball does not retrigger.
7. **Escape selection (new in v2)**
   - Move perpendicular to the ball's ground track. Command vx and vy together, signs chosen so both carry the robot away from the track: perpendicular speed is about 1.5·|sin α| + 0.5·|cos α| m/s for a ball from bearing α.
   - Of the two perpendicular directions, take the one the ball is already going to miss toward (sign of the predicted miss); if it is dead centre, take the side with more free space.
   - Burst: held until the needed travel plus 0.12 m is covered, capped at 0.4 m of travel and 0.8 s, and never more than 0.15 s past the ball's predicted arrival; then zero velocity.
   - Shipped speeds are 1.0 m/s forward, 0.8 m/s backward, 0.5 m/s sideways. Raise the fore-aft ones toward 1.5 once the step-response test shows the robot stays upright.
   - Option for the weak front/rear cones (`escape.along_track`, off by default): when the ball is descending steeply (over 45°), run forward or back along the track instead of sidestepping. In simulation it only pays off at the full 1.5 m/s.
8. **Act**
   - Publish on `/dodge_cmd_vel`. Add it to `cmd_vel_mux`'s `inputs` list **below the joystick and above Gemini, navigation and the person follower**.
   - RECOVER: zero velocity until the IMU shows the body is level and still, publish a `/dodge/event` so the planner can re-plan, then go quiet so the mux releases control.

Gemini Live stays out of this loop (cloud round trip is far longer than the 0.5–0.9 s available). It can be told a dodge happened.

## How this fits the existing codebase

| What exists | Where | How the dodge component uses it |
|---|---|---|
| Velocity arbitration | `pupperv3-monorepo/ros2_ws/src/cmd_vel_mux` | Add `/dodge_cmd_vel` via the `inputs` parameter in our own launch file. No monorepo edit. Note: the mux forwards on a 50 ms timer (up to 50 ms added latency) and drops a source after 500 ms of silence. |
| Walking policy and limits | `pupper_gait_deploy`, `neural_controller` | Commands via `/cmd_vel`; needs `speed_cap` raised to use more than 0.49 m/s. |
| Fisheye capture, white balance, lens model | `follow_me_pupper/robot/` (`camera_params.yaml`, `fisheye_converter.py`, `follow_me.launch.py`) | Reuse the double-sphere model and fixed gains. Only one process can open the sensor. |
| Launch pattern that wraps the course launch | `gemini_pupper/robot/launch/gemini_pupper.launch.py` | Model for `sar-pupper`'s launch: include the course launch, add mux, cameras, dodge node. |
| State machine + pure-function + off-robot tests pattern | `follow_me_pupper/lab/` | Same structure: logic in plain Python with tests that run on recorded clips, ROS only at the edges. |
| Robot states and job queue | `gemini_pupper/robot/robot_api.py` | Dodging only works in locomotion state. A dodge invalidates any timed move in the queue; the mission layer must be told. |
| Hailo YOLO | `follow_me_pupper/robot/yolo_seg.py` | Not used for dodging. One process at a time may hold the Hailo, so keeping dodging off it leaves it for finding people. |
| Bag recorder | `monorepo/.../bag_recorder` | Excludes raw images, so it cannot record training/tuning footage. We need our own recorder. |

Rules carried over from the course repos: do not edit the monorepo in place (`install.py` overwrites it – keep overrides in our launch and parameter files), never write keys into the repo, and never move the robot from code without someone at the e-stop.

### Repo layout (`sar-pupper/projectile-dodging/`)
Everything for this component lives in this one folder, so other components of the robot can sit beside it.
```
projectile-dodging/
  plan.md, README.md, DEPLOY.md
  dodge/             pure Python, no ROS: lens models, detect, colorfit, measure, track, threat, escape, core (state machine), recording, sim
  robot/             on the Pupper: USB capture, dodge_node.py, sar_dodge.launch.py, mux.yaml, fast_mux.py, start/stop/watch/walk scripts
  tools/             find_cameras, tune_exposure, fit_image_circle, view, calibrate_color / _camera / _extrinsics, record, replay, measure_latency, step_response, check_setup, sim_report, dodging_view (+ dodging_view.html)
  config/            dodge.yaml (every number) and calib/ (written by the calibration tools, merged on top)
  tests/             off-robot tests
  data/              recordings and saved throws (git-ignored)
```

## Implementation status (2026-11-26)

**Built:** every pipeline step above, the ROS node, the launch file that wraps the Gemini stack and adds `/dodge_cmd_vel` to the mux, and the Phase 0–3 tools. The dodge node starts unarmed: it reports the dodge it would make and saves a recording of each, but sends no command until armed.

**Checked off the robot (Python 3.11, numpy 1.26, OpenCV 4.11, as on the Pi):** 70 automated tests, including rendered ball images through detection, tracking and replay; the node driven in real time through stand-in ROS libraries and a fake USB camera.

**Simulated dodge rate, Tier A lobs, shipped config** (fisheye assumed to see the ball from 6 px at 30 fps; robot lag 0.2 s; 30–40 throws per sector):

| Sector | Dodged |
|---|---|
| Front cone | about 57 % |
| Front diagonals | about 93 % |
| Sides, rear diagonals, rear cone | 100 % |
| Throws aimed 0.7–1.5 m wide | 0 false dodges |
| Ball lying on the floor | 0 false dodges |

The front cone moves most with assumptions: 73 % with the fisheye at 60 fps, 97 % if it sees the ball from 5 px, 30 % if the robot's lag is 0.3 s, 93 % with the along-track option at 1.5 m/s. Flat throws (Tier B) are mostly not dodged, as expected. Re-run with `tools/sim_report.py` once the real lag and delay are measured.

**First run on the Pupper (2026-11-23, deployment stages 1–4).** The stack starts, the fisheye arrives through ROS and the ball is found in it, and one ELP opens at 1280×720, 90 fps. Three things were wrong and are fixed:
- *ELP picture near-black.* A 3 ms exposure at the camera's default gain is too dark indoors. `tools/tune_exposure.py` now sets gain (then exposure, if the gain runs out) from the picture itself and locks the white balance; the node and the tools warn when a picture is too dark to use.
- *False blobs around the fisheye's edge.* The rim of the lens splits colours and part of the fringe is ball-green. The detector now ignores everything beyond 89 % of the lit disc's radius (`cameras.fisheye.image_circle`; measured per robot with `tools/fit_image_circle.py`). Simulated dodge rates are unchanged by it.
- *Tools aborted on Ctrl+C.* They now release the cameras and ROS before exiting.

**Detection reworked after the first well-exposed ELP pictures (2026-11-23).** The ball in plain view was not detected. Two causes, both fixed as described in pipeline step 2: the roundness test rejected it for its ragged edge, and the saturation-based colour test let the dim floor through. Colour calibration now fits the limits per room: as loose as possible while the empty room shows nothing ball-coloured, and it reports how much dimmer the ball can get before it is lost. Checked on the two real pictures available (one ELP, one fisheye): the ball is the only thing detected in each, with the starting settings and with fitted ones, and the real ball shrunk and pasted around the real room is found down to about 9 px across. In that dim, lamp-lit room a ball 40 % darker than the hand-held one is not separable from picture noise; light is the remedy. Not yet run live on the robot.

**Colour cast (2026-11-23, later).** The next ELP pictures were strongly yellow (white objects read red 2.5 × blue): the fixed white balance set earlier no longer matched what the cameras were showing. On one camera the calibrated hue range missed all but the ball's shaded edge; on the other the walls and dozens of objects were boxed. The detector now takes the cast out itself (pipeline step 2), calibration is done on the corrected pictures, and older calibrations are ignored. The cameras' settings are applied again once they are streaming and read back, and the exposure tool checks that a fixed white balance survives reopening the camera, falling back to automatic if it does not. With the out-of-the-box settings, on all six pictures taken on the robot so far: the ball is found whole in each of the three that show it, and the two pictures that showed dozens of false boxes show none.

The second ELP could not be opened in that run and the log could not say why. `tools/find_cameras.py` now reports the cause (not connected, in use, no video, USB power limit) and tests both rear cameras streaming together. Also seen: with the default colour profile the fisheye's blob on a ball lying on a beige carpet included the carpet around it, so the range read short; per-room colour calibration (Phase 0) is not optional.

**First armed throws: the Pi could not keep up (2026-11-26).** With all three cameras running in the full stack, `watch.sh` showed every camera at 3–8 frames a second, 100–320 ms of processing per frame and 0.3–0.55 s of lag; `top` showed the dodge node as the largest consumer and a load of 14 on the Pi's four cores. A saved armed dodge (`data/events/20261126-122155`) fired with the ball 0.65 m away and 0.09 s from contact, against 0.48 s needed, and its recording held 10–16 frames per camera over 2.7 s, too few for `replay.py` to reproduce the decision. Still green objects in the room were picked up and dropped as tracks over and over, because at that rate one frame gap is as long as a track is allowed to go unseen. Three changes, not yet measured on the robot:
- *Whole-frame searches are coarse first.* The detector looks at every second pixel across and down, then measures at full resolution only the places that showed ball colour (`detector.coarse_step`). It returns the same blobs as before (checked on 120 synthetic rooms and in the tests) for about a third of the work; a picture with colour in more than 150 places is searched directly, as before.
- *OpenCV runs single-threaded in the node* (`node.opencv_threads`). Each camera already has its own thread; OpenCV splitting every operation across all cores on top of that made the threads fight each other.
- *The rear cameras are processed at 30 frames a second, not 60* (`max_process_fps`). Decoding and searching 60 frames a second from two cameras plus the fisheye asks for more than the Pi has left beside the walking controller. In simulation 30 costs nothing (`sim_report.py --elp-fps 30`: same dodge rates in every sector). Raise it only while `watch.sh` shows the rate being reached with processing time well under the frame interval.

Also fixed: a dodge event's `late` flag was saved as the text "True" rather than a true/false value.

**Armed throws after those three changes: still too slow (2026-11-26, later).** Pupper Dodging View showed the ball detected in only 5–9 pictures a second, pictures from the rear cameras "more than 80 ms old when processed", 402–705 ms from first sight to the decision, and the decision made with 200–410 ms left against about 450 ms needed. One throw was dodged, most were not. The mux passed the command on 40–43 ms after it was sent, and the body began to move 90–131 ms after the decision. Simulation with those camera numbers (`sim_report.py --dist 2 3 --rise 0.4 0.9 --elp-fps 7 --elp-latency 0.15 --fisheye-fps 7 --fisheye-latency 0.2`) gives the same picture: no throw dodged in any sector, decisions with 0.26–0.45 s left. At 15 pictures a second it dodges about half; at 30, 73–100 %. The decision logic was not the problem; the pictures were too few and too old. Two causes in the code, both fixed, **neither yet measured on the robot:**
- *Stale pictures.* One thread per rear camera both took frames off the camera and processed them. While it processed one, the camera's queue filled, and the next frame it took was the oldest one waiting. Now a capture thread empties the queue as frames arrive and the work thread always gets the newest (`robot/usb_camera.py`). Against a simulated camera with a 60 ms job per frame, frames were about 100 ms old when handed over before and about 3 ms after.
- *Everything in one Python process.* Three cameras' capture, decoding and search, the IMU callback and the command timer were threads of one process, and Python runs one thread's Python code at a time, so they queued behind each other. Each camera now runs in a process of its own (`robot/camera_worker.py`) and sends the node only the blobs it found and the camera's JPEG for the recorder; the node tracks and decides. `node.camera_processes: false` goes back to threads.

**Measured on the Pupper in dry run, same evening (nobody throwing, warm lamp light, robot stack plus another session's voice server running).** Before any of this: 5–15 pictures a second per camera, 130–390 ms from a picture being taken to the node acting on it, the node using more than a whole core, ten "balls" in every picture of every camera, and a false "throw seen" every few seconds. After: 20–26 pictures a second, 3–35 ms, the node at about 20 % of a core, no false "throw seen". Simulation at the measured rates (`sim_report.py --dist 2 3 --rise 0.4 0.9 --elp-fps 21 --elp-latency 0.04 --fisheye-fps 20 --fisheye-latency 0.07 --lag 0.25 --delay 0.07`): front cone 67 %, front diagonals and sides 90 %, rear diagonals 100 %, rear cone 77 %. **No throw has been made at the robot since these changes, armed or not.** What the measurements turned up, beyond the two causes above:
- *The tracker ignored new blobs while its 8 slots were full,* and clutter kept them full, so a thrown ball could get no track at all (the throws with no reaction). Now 24 slots; when full, the weakest track gives way, never a ball seen in flight.
- *The rear cameras were not showing the white balance they reported.* They read "fixed at 3858 K" while red was three to four times blue, so walls and floor passed for ball colour. A camera that keeps a strong cast has its settings sent again (`node.cast_limit`; `watch.sh` shows `COLOUR CAST`). What makes the camera act was got wrong at first (a changed temperature) and corrected after the next boot: see the cold-boot note below. The exposure was retuned for evening light. **The colour calibration of both rear cameras was made with the cast in the picture and must be redone** (`calibrate_color.py`); the rear-left one accepts noise on a dark floor.
- *The fisheye's colour calibration was for hue 9–28* (skin, wood), not green. A calibration whose hue range is not green is now ignored with a note; redo it.
- *After a power-up the rear cameras ignore a fixed white balance (found after a reboot, 2026-11-26 night).* Both came up reading "fixed at 3858 K / 4034 K" with one colour channel 2.9 times another, and the node's three re-sends changed nothing. Tried on the live cameras with `v4l2-ctl` while the node watched the cast: a different temperature, even held for a second, did nothing; automatic white balance on for 0.3–2 s and off again brought the cast to 1.25, and from then on 2800, 4034 and 6000 K each changed the picture as they should. `usb_camera.wake_white_balance` now does that whenever a camera is opened and whenever the cast check fires. The mechanism was established by hand; the code doing it by itself is first tested at the next power-up. With the cast in, the Pi was also back to 10–15 pictures a second, because every picture was full of false patches.
- *The colour step was learning the hand, not the ball.* With the cast removed the ball's hue is 35–56 in all three cameras and a hand's or the wooden floor's is 11–25 (measured on that night's recordings). The tool accepted hue from 15 and took the biggest changed patch, which in the fisheye is the hand and arm: its two fisheye calibrations came out at hue 9–28 and 8–27. It now accepts hue from 28, takes the biggest patch that is round, and refuses to write a result that is not green.
- *Balls being handled beside the robot, and balls lying still, set off dry-run "dodges".* Ten in four minutes that night, with nobody throwing. Five were pulled off the robot and replayed; four causes, each fixed and tested:
  - *The cameras disagree about a ball's range.* Range comes from how big the ball looks, and each camera has its own error (size calibration, a hand over half the ball). One still ball read 4.5 m in the rear-right camera and 1.5 m in the fisheye a moment later: put together, a ball coming in at 10 m/s. Now an approach must also show within one camera's own pictures, three good looks of it, at 0.4 of the speed all cameras together give (`threat.camera_support`), and "in flight" is judged from one camera's pictures too.
  - *A track with a vague range matched a blob on the other side of the robot.* All rays start at the robot, so the two together put the "ball" at the robot. A blob now only matches a track within 45° of where that camera expects it (`tracker.gate_angle_deg`).
  - *The fisheye smears a moving ball* (about 20 ms exposure in lamp light), and the streak's area read as a bigger, nearer ball. That camera now sizes the ball by the streak's width (`size_by_width`). In the rear cameras (3 ms) a blob that is not round is a ball partly hidden or in shadow: its range is marked unreliable and does not count as a good look.
  - *A still ball's blob grew while the white balance was being changed* (my own testing that night, and it would happen for a second whenever a camera is opened). A rear camera now reports nothing for 1.5 s after it is opened or sent its settings (`node.settle_s`), and the still-object filter only counts growth beyond the biggest a place has shown before.

  Replayed after the fixes: none of the five dodges, the recorded real throw still does. Simulated at the measured rates, dodge rates are within one throw in thirty of before in every direction, and throws aimed 0.7–1.5 m wide are dodged 1 time in 30 instead of 3 (high lobs from 2 m: 1 instead of 9). Cost: a ball first seen very late needs a third look from one camera, about 45 ms at 21 pictures a second. `tools/replay.py` now runs the same per-camera code as the robot, still-object filter included. Not yet run on the robot.
- *numpy's maths library kept two idle threads spinning* at 35–39 % of a core each inside the node, and *the IMU arrives 520 times a second*, each message costing Python about a millisecond. One thread now, and a C++ throttle hands the node 100 a second.
- *Still things* (balls already thrown, a bowl of them, a patterned mat, a drying rack) flicker, and flickering blobs were taken for motion. Each camera now leaves out blobs that have not moved for a second and squares of the picture that always show some colour (`detector.still`).
- *Decoding.* Each JPEG is decoded at half size first and in full only when something that is not still shows in it (`detector.decode_small_first`); a speckled picture no longer makes the search give up and take its slowest path.
- *The rear cameras deliver about 43 frames a second, not the 90 they are opened at,* probably because both are on one USB controller (`xhci-hcd.0`). With a picture taking more than 23 ms to process, the node gets every second one, about 21 a second. Moving one camera to a socket of the other controller is the next thing to try; `max_process_fps` is 45 so that every frame delivered can be used.
- *The fisheye delivers about 20 a second* in this light (its exposure is automatic).

Smaller changes with them: IMU samples beyond 100 a second are let go on arrival (`node.imu_rate_hz`); the fisheye search stays inside the lens's lit disc, a third of the frame; the fisheye's JPEG quality is set to 85 instead of the camera node's 95 (`--fisheye-quality`); the person follower is no longer started by default (`--follower`), since it reads and decodes the fisheye pictures too; `start.sh` says so if the monorepo's `llm-agent` service is running. `watch.sh` now splits "cpu" into decoding and searching.

**Small room, high lobs, and the throw recorder (2026-11-26).** The practice space is about 4 × 6 m, so throws come from 2–3 m, not the 3–4 m of Tier A, and throwing higher had not helped. Apart from the overload above, simulation showed a second reason for the lobs: thrown from 2 m, the ball leaves the top of a rear camera's picture in mid-flight, and the tracker dropped it after 0.25 s unseen, so high lobs from the sides and rear diagonals were almost never dodged. Three changes to the logic: a ball seen in flight is followed on its arc for 0.6 s while unseen (`tracker.flight_coast_s`); a lob is judged while it is still climbing, by where its arc comes down, instead of only once it is closing in (`threat.judge_rising_balls`; "in flight" means fast and moving across the line of sight, since range read from the blob's size jitters too much to prove motion); and the node commits a little earlier (`threat.trigger_margin_s` 0.3 → 0.6). Simulated high lobs dodged, of 40, with the rear cameras at 30 frames a second: from the sides 4 before and 31 after, rear diagonals 10 and 40, behind 25 and 40; they are decided about 0.3 s sooner; shoulder-height lobs are unchanged (sides about 95 %). The high lobs still missed from the sides come from the front-side (60–90°), where the ball is at the edge of a rear camera's picture and leaves it within three frames, before it can be tracked: from the sides, a shoulder-to-head-height lob is the better throw; about 1 in 20 throws passing 0.7–1.5 m wide is now dodged anyway. Straight from the front stays the weak direction (about 80 % for a shoulder-height lob from 3 m, far less if the robot is slower than assumed), because the sidestep is slow: the room layout in `DEPLOY.md` gives the front and back the long 3 m and the sides the short 2 m, and asks for high lobs. The diagonal move is 20 % slower (`escape.diagonal_scale`) because it trips the robot on hardwood.

To see where the time goes on the real robot, the node now records every throw (`data/throws/`): dodges, dry-run dodges, throws it tracked but did not dodge (with its reasons), and seconds saved on request. Each has the cameras' frames, the IMU and a frame-by-frame trace. `tools/dodging_view.py`, the page called Pupper Dodging View (started by `robot/start.sh`, port 8077), turns each into timings (first sight of the ball, decision, command, the mux passing it on, the body starting to turn, expected contact, each camera's rate and picture age) and a `throws.csv`, and serves a page that plays the cameras with the detections drawn on. Video files (plain and annotated) are made on request, not automatically, so that encoding does not take processor time from the dodge. Run so far only against a stand-in robot.

The step-response tool no longer reports distance or lag: Pupper's IMU driver leaves the accelerometer off, so the tool had been integrating zeros. It reports the reaction delay (from the gyro) and tilt, and fits the lag to timings from a slow-motion video if given.

**Not verified until it runs on the Pupper:** the second ELP and both together, the exposure tuner on the real cameras, colour thresholds, timestamps, the dodge node in the real stack, CPU load, and the robot's real acceleration. `DEPLOY.md` walks through these in order.

**Where the build differs from the earlier text of this plan:**
- Tracker is 3D-ballistic rather than a filter on angles (pipeline step 4).
- Decision D1: the fisheye is read from the course camera node's compressed topic. Owning the sensor directly is not implemented.
- Decision D2: `robot/fast_mux.py` is the default since 2026-11-26 (the monorepo mux cost 40–43 ms of a 90–131 ms reaction in two recorded throws); `--slow-mux` goes back.
- Decision D3: ELP mode and processing rate are config values (1280×720 at 120 fps requested, 30 fps processed since the first armed throws) to be set from Phase 0.
- Latency is measured by tapping the robot (IMU jolt vs image jolt) rather than with a light flash; step response uses the IMU.
- Added: a roundness check and a still-object check against false dodges; compensation for the robot's own walking speed; a size-offset correction for blob width.

## Build order

**Phase 0 – measure (decides the envelope)**
1. Step response: time from command to 0.25 m displacement for +vx, −vx, ±vy, and combined vx+vy, from standing and from a slow walk. Check the tilt e-stop does not trip.
2. Latency per camera: light flash → frame timestamp → `/cmd_vel` → first joint motion. Includes the mux's 50 ms tick.
3. Fisheye: actual frame rate and delay of `/camera/image_raw` vs `/camera/image_raw/compressed` vs direct capture.
4. ELPs: mode list, real frame rate with both streaming, CPU cost of MJPG decode, real horizontal and vertical FOV.
5. CPU headroom with the full stack running (control loop, Gemini kiosk, cameras).
6. Ball: caliper diameter; ball colour sampled in each camera at 1, 2, 3, 4 m.

**Phase 1 – record.** A recorder that saves raw frames with timestamps from all three cameras plus IMU. 20+ Tier A throws per sector, plus misses, plus no-ball footage of the room and of the robot walking.

**Phase 2 – perception offline.** Camera calibration (ELP intrinsics by checkerboard; verify the fisheye parameters at 1400×1050; mounting poses start from the CAD table above and are refined by showing one ball to two cameras at once in the overlap zones). Per-camera colour profiles. Detector, ray conversion, tracker, cross-camera handover – all tuned on the recordings.

**Phase 3 – threat logic offline.** Label each recorded throw hit or miss. Tune trigger thresholds for hit recall first, false-dodge rate second.

**Phase 4 – live, standing.** Dodge node + mux priority + escape selection. Sector-by-sector tests against the success criteria.

**Phase 5 – live, walking and integrated.** Gyro de-rotation, free-space input from obstacle avoidance, `/dodge/event` to the planner, behaviour during Gemini moves.

**Phase 6 – stretch.** Tier B throws; steep-lob escape for the front/rear cones; small classifier for green clutter; a trained dodge policy in pupper-mjlab with a wider sideways range.

## Things that were easy to miss

1. **Sideways is the slow axis.** 0.5 m/s sideways vs 1.5 m/s fore-aft means a ball from dead ahead or dead astern is the hardest to dodge, and a ball from the side is the easiest. Escape choice has to depend on bearing.
2. **The front is the weak sensor.** Fewer pixels and fewer frames than the rear cameras, in the direction the robot usually faces a person.
3. **Three different lenses, one tracker.** Without converting to body-frame angles, pixel sizes and positions from the fisheye and the ELPs are not comparable and tracks cannot hand over.
4. **Per-camera colour.** One green threshold will not work on both the red-tinted fisheye and the ELPs.
5. **Rear cameras' vertical view.** The ELP plates are vertical (level axes) and already printed. A high lob from close behind leaves the top of the frame before it lands (0.25–0.65 s before impact for a 0.5–1.0 m rise from 3 m). The trigger normally fires before that, and the tracker coasts through the gap, so the plan accepts this rather than reprinting. It is the reason Tier A caps the peak height.
6. **Balls on the floor.** After the first throw there is a neon green ball lying in view. The detector must ignore blobs that are not growing.
7. **Who owns the fisheye.** `camera_ros` holds the sensor; Gemini and follow-me read its compressed topic. The dodge node needs low-latency frames from the same sensor without breaking them.
8. **Mux latency and priority.** Up to 50 ms from the mux timer; dodge must outrank Gemini and navigation but never the joystick.
9. **Robot must be in a walking controller.** No dodging mid-trick, relaxed, or fallen. The dodge node should report "cannot dodge" rather than fail silently.
10. **Dodging into something.** Until obstacle avoidance supplies free space, tests need an open floor; afterwards the escape direction must respect it.
11. **CPU budget.** Three camera pipelines beside a 520 Hz control loop and a browser on a Pi 5. Latency measured on an idle robot will not hold under full load.
12. **USB.** Two cameras plus mic receivers on four ports; bandwidth and power with everything streaming.
13. **Lighting flicker.** At 90–120 fps with short exposure, mains-powered lights can make frame brightness pulse; thresholds should rely on hue and saturation, not brightness.
14. **Scoring needs ground truth.** "Would it have hit?" needs a baseline run with dodging off and ideally a phone slow-motion video per session.
15. **Recovery.** A hard dodge can tip the robot toward the tilt e-stop; the burst must be capped and followed by a settle phase.

## Decisions to make

- **D1 – fisheye access:** currently the compressed topic. If its delay or frame rate proves too poor (Phase 0 item 3), have the dodge node own the sensor and republish for Gemini/follow-me.
- **D2 – mux:** live with the 50 ms tick (default), or switch to `robot/fast_mux.py`. Decide after Phase 0 item 2.
- **D3 – ELP mode:** resolution vs frame rate vs CPU. Decide after Phase 0 item 4.
- **D4 – free-space interface** from the obstacle-avoidance component.

Already decided: cameras stay level on the printed plate; walking controller is `gemini_walk`.

## Sources

- Forrai et al., Event-based Agile Object Catching with a Quadrupedal Robot (2023): https://arxiv.org/pdf/2303.17479
- Falanga, Kleber, Scaramuzza, Dynamic obstacle avoidance for quadrotors with event cameras (Science Robotics 2020): https://www.zora.uzh.ch/entities/publication/a37f45b7-b5cc-45ea-b2c3-6c3191fb4928
- Falanga, Kim, Scaramuzza, How Fast is Too Fast? The Role of Perception Latency in High-Speed Sense and Avoid: https://nccr-robotics.ch/?p=10357
- Xu et al., REBot: Reflexive Evasion Robot for Instantaneous Dynamic Obstacle Avoidance (2025): https://arxiv.org/abs/2508.06229
- Ma et al., Learning coordinated badminton skills for legged manipulators (Science Robotics 2025), coverage: https://cosmosmagazine.com/technology/robotics/robot-plays-badminton
- Tau / time-to-contact (Lee 1976), summary: https://macsphere.mcmaster.ca/handle/11375/11240
- KTH ball-catching robot (HSV segmentation + EKF): https://www.csc.kth.se/~ccs/Robot_arm/ballcatching.html
- Pupper v3 technical specifications: https://pupper-v3-documentation.readthedocs.io/en/latest/learn_more/tech_specs.html
