# Projectile dodging – implementation plan (v2)

Updated 2026-11-23. Supersedes the 2026-11-16 plan. **Implemented in this folder** (see `README.md`; deployment steps in `DEPLOY.md`). The code has been tested off the robot only; it has not yet run on the Pupper.

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
   - Colour threshold with a **separate calibrated profile per camera** (the fisheye's red tint and fixed gains shift neon green toward yellow).
   - Morphological open → connected components → gate on area and roundness.
   - Static mask per camera for robot parts in view (mics, ears, body edge).
   - Once tracking, search only a region around the predicted position.
3. **Measure → ray**
   - Sub-pixel centroid and blob extent, converted through the camera model (double-sphere for the fisheye using the existing `camera_params.yaml`; pinhole + distortion for the ELPs) into a ray and an **angular diameter θ**, then through the camera's mounting pose (table above) into a 3D point in `base_link`: camera position + (D / θ) × ray direction. Azimuth and elevation are taken from that point, so the 193 mm between cameras is accounted for.
   - Angular diameter is used instead of pixel diameter because a rectilinear 120° lens stretches the ball near the image edge and the fisheye compresses it.
4. **Track (one tracker, body-centred)**
   - As built: a Kalman filter on the ball's 3D position and velocity relative to the robot, assuming free flight (gravity plus drag). Each measurement is the 3D point from step 3 with a small error across the line of sight and a large one along it. Azimuth, elevation, angular size and their rates are read off this state; filtering them directly was the original idea, but a lobbed ball's elevation is not constant on a collision course, and the 3D form handles that and the camera offsets in one go.
   - Three detections to confirm a track. Detections from two cameras merge into one track; a track hands over between cameras without resetting.
   - The filter runs in a gravity-aligned frame built from the IMU orientation (`/imu_sensor_broadcaster/imu`), so body pitch and roll while trotting do not look like ball motion.
5. **Predict**
   - The track is flown forward on its arc until it lands. Time-to-contact τ = when it first comes within the margin of the robot's outline (a box in `base_link`); miss distance = the closest the ball's surface gets. For a ball coming straight in this is the same as τ = θ / θ̇.
   - Threat = predicted miss within margin (0.05 m) plus 1.5 σ of the prediction's own uncertainty, and the ball really is a moving ball: closing at over 1 m/s after removing the robot's own walking speed, and moving over the ground at over 1.5 m/s on a straight-line fit (a still object hanging above the robot would otherwise look like it is falling onto it).
   - Approach direction and descent angle come from the predicted velocity at contact.
6. **Decide (state machine)**
   - SEARCH → TRACK → THREAT → DODGE → RECOVER → SEARCH.
   - The dodge fires when: a threat is predicted, and τ is below (time needed for that bearing + 0.3 s). With time in hand it waits for a second consistent verdict and a sideways prediction good to 0.2 m; once τ is down to the time needed it goes at once. Trigger early rather than timing it finely – τ is only accurate late.
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
  dodge/             pure Python, no ROS: lens models, detect, measure, track, threat, escape, core (state machine), recording, sim
  robot/             on the Pupper: USB capture, dodge_node.py, sar_dodge.launch.py, mux.yaml, fast_mux.py, start/stop/watch/walk scripts
  tools/             find_cameras, view, calibrate_color / _camera / _extrinsics, record, replay, measure_latency, step_response, check_setup, sim_report
  config/            dodge.yaml (every number) and calib/ (written by the calibration tools, merged on top)
  tests/             off-robot tests
  data/              recordings and saved dodge events (git-ignored)
```

## Implementation status (2026-11-23)

**Built:** every pipeline step above, the ROS node, the launch file that wraps the Gemini stack and adds `/dodge_cmd_vel` to the mux, and the Phase 0–3 tools. The dodge node starts unarmed: it reports the dodge it would make and saves a recording of each, but sends no command until armed.

**Checked off the robot (Python 3.11, numpy 1.26, OpenCV 4.11, as on the Pi):** 32 automated tests, including rendered ball images through detection, tracking and replay; the node driven in real time through stand-in ROS libraries and a fake USB camera.

**Simulated dodge rate, Tier A lobs, shipped config** (fisheye assumed to see the ball from 6 px at 30 fps; robot lag 0.2 s; 30–40 throws per sector):

| Sector | Dodged |
|---|---|
| Front cone | about 57 % |
| Front diagonals | about 93 % |
| Sides, rear diagonals, rear cone | 100 % |
| Throws aimed 0.7–1.5 m wide | 0 false dodges |
| Ball lying on the floor | 0 false dodges |

The front cone moves most with assumptions: 73 % with the fisheye at 60 fps, 97 % if it sees the ball from 5 px, 30 % if the robot's lag is 0.3 s, 93 % with the along-track option at 1.5 m/s. Flat throws (Tier B) are mostly not dodged, as expected. Re-run with `tools/sim_report.py` once the real lag and delay are measured.

**Not verified until it runs on the Pupper:** USB capture modes and exposure control, colour thresholds, timestamps, the launch file, CPU load, and the robot's real acceleration. `DEPLOY.md` walks through these in order.

**Where the build differs from the earlier text of this plan:**
- Tracker is 3D-ballistic rather than a filter on angles (pipeline step 4).
- Decision D1: the fisheye is read from the course camera node's compressed topic. Owning the sensor directly is not implemented.
- Decision D2: the monorepo mux is the default; `robot/fast_mux.py` is there as the alternative (`--fast-mux`).
- Decision D3: ELP mode and processing rate are config values (1280×720 at 120 fps requested, 60 fps processed) to be set from Phase 0.
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
