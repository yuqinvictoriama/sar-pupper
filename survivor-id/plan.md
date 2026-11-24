# Survivor identification – implementation plan (v1)

Written 2026-11-23; updated the same day with your answers (readings confirmed, obstacles go to route mapping, arrival behaviour chosen – see "Decisions"). **The plan is now implemented** in this folder (`README.md` for what is where, `DEPLOY.md` for getting it onto the robot), tested off the robot only; "What was built" at the end lists where the code departs from the plan and what is still missing. The arrival bounce is designed in `survivor-id/bounce/` and not yet trained. It sits beside `projectile-dodging/plan.md` and reuses that plan's camera table, mounting poses, walking controller (`gemini_walk`) and `cmd_vel_mux` notes instead of repeating them.

## Goal

Two things, in order:

1. **Identify.** A person who throws both arms high in the air is recognised as the survivor. This has to work wherever the person is around Pupper (all three cameras, 360°) and whichever way the person is facing (toward the camera, away from it, or side-on).
2. **Pin and reach.** That person becomes *the* survivor. Pupper remembers who they are and where they are – after the arms come down, and while they are out of view – and keeps working its way to them until it arrives. Obstacles, dodges and losing sight of them delay it; they do not cancel it.

### How this plan reads the request

All four were confirmed on 2026-11-23.

- **"From any direction"** means both: any bearing around the robot, and any body orientation relative to the camera.
- **The signal** is both arms raised above the head, roughly straight, and held. The reference photo shows a standing person with the arms in a narrow V. One raised arm is not the signal.
- **"Regardless of obstacles"** means persistent: Pupper goes around things and never gives up on its own. It does not mean pushing through objects or into people, and it stops short of the survivor rather than walking into them.
- **One survivor at a time.** The first confirmed signal wins. Later signals from other people are recorded but not chased until the first survivor is reached (decision S1).

## Fixed facts

### The signal
- Reference: full-body silhouette, standing, feet together, both arms straight and raised about 20° either side of vertical, hands open.
- It is the same shape as the recognised rescue signal: both arms raised in a "Y" tells a rescue helicopter "yes, I need help". People will do it without being taught.
- Body numbers used for the estimates below (1.7 m adult): eyes 1.6 m, shoulders 1.4 m, hips 0.92 m, wrists with arms up 2.0 m, fingertips 2.15 m. Shoulder-to-hip ("torso") length 0.48 m. Shoulder to wrist 0.6 m.

### What each camera sees of a person

Camera positions are from the dodging plan (CAD). Lens heights above the floor assume the hip axes are about 0.18 m up when standing – measure this.

| | Fisheye (front) | ELP ×2 (rear, 1280 wide) |
|---|---|---|
| Lens height above floor | about 0.18 m | about 0.33 m |
| Resolution on a person | about 4 px per degree, everywhere | 6.5 px per degree at centre (9.7 at 1920 wide) |
| Person with arms up, feet to fingertips, at 2 / 4 / 6 / 8 m | 199 / 115 / 80 / 60 px | 397 / 199 / 132 / 99 px |
| Same, in the 700×572 rectified frame Lab 5 uses | 158 / 92 / 63 / 48 px | – |
| Upper arm + forearm (shoulder to wrist) at 4 m | about 30 px | about 50 px |
| Vertical view | whole upper hemisphere – raised hands are never cut off | ±44–47° – raised hands are cut off when the person is close |
| Closest distance at which raised hands still fit in the frame | no limit | 1.7–1.9 m at image centre; 2.3–2.6 m at 42° off-axis; **3.2–3.5 m directly behind the robot** (the seam, 57.5° off both axes) |

The ELP limits are pinhole estimates; the real lens's barrel distortion should help a little. Phase 0 measures them.

What follows from this:
- **The front is the weaker sensor again** (as for dodging): about half the pixels on a person. The fisheye should read the signal to about 4 m, perhaps 5; the rear cameras to 6–8 m.
- **The rear cameras cannot see the raised hands of a close person**, and directly behind is worst. This is what the turn-to-look behaviour is for.
- **Everything is seen from knee height or lower.** Pose models are trained mostly on eye-level photos, and from the floor a close person is strongly foreshortened. Closer than about 2 m the signal cannot be told apart from arms reaching toward the robot (see "easy to miss" 2).
- **Feet on the floor are a poor range sensor here.** At 4 m the feet are only 2.6° below the fisheye's horizon (4.7° for an ELP), so 1° of body pitch while trotting turns 4 m into anything from 2.9 to 6.5 m (3.3 to 5.1 m on an ELP). Range comes from body size instead.

### Compute and state
- One Hailo-8L, and one process at a time may hold it.
- Hailo's model zoo has ready-compiled pose models for the 8L: `yolov8s_pose` (640×640, 82 FPS on the chip at batch 1, 59.2 / 55.6 mAP float / quantised) and `yolov8m_pose` (36 FPS, 64.3 / 61.7). A published benchmark on a Raspberry Pi compute-module board with the Hailo-8L got 27 FPS end to end for YOLOv8 pose. **Plan on 25–30 inferences per second in total** until Phase 0 measures ours.
- These models return person boxes and 17 body keypoints (nose, eyes, ears, shoulders, elbows, wrists, hips, knees, ankles), each with a confidence.
- **There is no odometry anywhere in the stack** (nothing in the monorepo publishes it). The only motion sensing is the IMU (`/imu_sensor_broadcaster/imu`) and the velocity commands themselves.
- Fisheye frames already exist on `/camera/image_raw/compressed` (1400×1050). ELP capture is being written for dodging.

## Approach

- **Perception = an off-the-shelf pose model plus a geometric rule. No training for v1.** YOLOv8s-pose on the Hailo finds people and their joints in one pass. The signal is then a few inequalities on shoulder, elbow, wrist and head positions. One model serves person detection, the gesture, tracking and re-identification, and it replaces `yolov8n_seg` as the mission's people detector.
- **Alternatives considered.**
  - A custom "hands-up" detector class: needs a labelled dataset and the Hailo compile toolchain. Not for v1.
  - Silhouette matching against the photo: needs clean segmentation and breaks on clutter and side views.
  - Asking Gemini Live: seconds of delay, off-robot, and no bearing to steer by. Gemini narrates; it does not detect.
  - **Kept as the fallback:** a tiny classifier (logistic regression or a small MLP) on the normalised keypoints, trained on our own recordings, if the hand-written rule gives too many false locks. It runs on the CPU in microseconds and needs no recompiled model.
- **Prior art with the same shape:** Liu & Szirányi's rescue drone runs a pose skeleton into a gesture classifier, and its "Attention" gesture is what makes the drone approach the person.
- **Two passes per person: wide to find, zoom to read.** A whole-camera view shrunk to 640 px leaves a 4 m person about 85–100 px tall – enough to find them, not enough to trust their wrists. The gesture is read from a second pass on a patch around the person at the camera's full resolution. Pose accuracy is known to fall quickly as people get small, so the zoom pass is what sets the range.
- **Everything after detection is in body-frame angles**, exactly as in the dodging plan: the same camera models and mounting poses, so one tracker serves three lenses. "Up" means against gravity (from the IMU), not up in the image.
- **Action = a state machine.** Its velocity commands enter `cmd_vel_mux` below dodging, so a dodge always interrupts and the approach then resumes.

## Operating envelope

Numbers marked *est.* are placeholders until Phase 0.

### Tier A – the demo spec (must work)
- **Person:** an adult standing (or kneeling or sitting upright). Head, both shoulders and both arms visible; legs may be hidden behind something.
- **Signal:** both arms straight up or in a V, held for at least 2 s. Pupper locks within about 1.5 s.
- **Distance: 2–4 m from Pupper, at any bearing, facing any way.** Directly behind and closer than about 3.5 m (*est.*), Pupper first turns to look, which adds 1–2 s.
- **Robot:** standing or walking at up to 0.3 m/s, in the walking controller.
- **Scene:** steady indoor lighting, person not standing in front of a bright window, up to three people in view (survivor, ball thrower, a bystander).
- **Approach:** flat floor; obstacles that terrain agility can step over or walk around; the survivor stays within about 1 m of where they signalled. Pupper stops 0.7 m from them, announces through Gemini Live, and bounces on the spot.

### Tier B – stretch
- 4–6 m in front, up to 8 m to the sides and rear.
- Survivor walks away after signalling; robot moving faster than 0.3 m/s when the signal is given.
- One arm hidden behind an obstacle; a second survivor queued; the one-arm-up-one-arm-down "N" signal to cancel.

### Out of scope
- One raised arm, waving, or a person lying down.
- Crowds, darkness, a person visible only on a screen or in a mirror.
- Stairs, climbing over things, obstacles that move (other than thrown balls, which dodging handles).

### Success criteria
- **Detection:** 20 trials per cell of 5 sectors (front, front-diagonal, side, rear-diagonal, rear) × 3 distances (2, 3, 4 m), facing directions mixed evenly (toward, away, left side, right side). At least 90 % locked within 2 s; rear cone inside 3.5 m at least 80 % within 4 s (includes the turn).
- **False locks: none in 10 minutes** of a person doing everything except the signal – walking about, waving one arm, throwing underhand and overhand, stretching, hands on head, hands on hips, bending down to pet the robot, pointing, clapping at chest height.
- **Identity:** with two other people crossing in front, the lock stays on the right person in at least 9 of 10 trials, and is recovered after 5 s fully out of view.
- **Approach:** stops within 0.5–1.0 m of the survivor in at least 8 of 10 runs on each obstacle layout, never touches them, and resumes after a dodge in 10 of 10.
- **Arrival:** in 10 of 10 arrivals the announcement starts within 3 s of stopping, the bounce plays with no foot leaving the floor, Pupper ends within 0.1 m of where it stopped, and it is back in the walking controller afterwards.

## Pipeline

1. **Frames**
   - Fisheye: `/camera/image_raw/compressed`, as today.
   - ELPs: the dodge capture threads republish the camera's own MJPG frames, untouched, at up to 10 Hz on `/elp_left/image/compressed` and `/elp_right/image/compressed`. No re-encoding, so it costs dodging almost nothing.
   - Survivor ID needs 5–10 frames per second, not 60–120, and keeps only the newest frame per camera.
2. **Wide pass – find people.** Cameras in rotation, one 640×640 inference each, every camera at least 3 times a second.
   - Fisheye → the 700×572 rectified (equirectangular) frame Lab 5 already produces with `fisheye_converter.py`.
   - ELP → whole frame, shrunk and letterboxed.
   - Output: person boxes, keypoints, scores. If far people are missed in front, split the fisheye into left and right halves (two passes, more pixels each).
3. **To the body frame.** Box centre and keypoints → rays through the camera model and mounting pose → azimuth and elevation about the body origin, levelled with IMU roll and pitch. The robot's own parts in the ELP views (mics, ears, body) are masked as in the dodging plan.
4. **People tracker.** One tracker for all cameras, working in azimuth, elevation and range.
   - Association by nearest position inside a gate, then appearance. Bearings are de-rotated with the gyro so tracks do not jump when Pupper turns.
   - The same person seen by two cameras in an overlap zone becomes one track; a track hands over between cameras without a new id.
   - Each track carries: id, bearing, range, last-seen time, appearance, and its recent gesture results.
5. **Zoom pass – read the gesture.** For each track that needs it, cut a patch around the person from the full-resolution frame and run the same pose model on it.
   - Fisheye: a perspective patch rendered through the double-sphere model, centred on the person, at about 8 px per degree. ELP: the same re-projection, which also removes the stretching a 120° lens gives people near the image edge.
   - Patch = the person's box plus 25 % margin, more above the head.
   - Priority when the budget is short: locked survivor → people whose arms may be up → new people → everyone else.
   - Skipped when the person is already more than about 200 px tall in the wide pass.
   - Budget: whatever the wide passes leave, *est.* 15–20 per second, i.e. about 10 Hz on one or two people.
6. **Measure.** Scale `s` = angular length from mid-shoulders to mid-hips. If the hips are hidden, `s` ≈ 2.4 × the height of the ears above the shoulders.
   - **Range ≈ 0.48 m / s** (s in radians), good to roughly ±10–15 % across adults.
   - Cross-checks: eye-to-ankle height when the legs are visible; triangulation between the fisheye and an ELP in the side overlap zones (193 mm baseline).
7. **Gesture rule, per person per frame.** Positions are compared in gravity-up angles and in units of `s`, so the rule does not depend on distance or on which camera saw the person. Start values, to be tuned on recordings:
   - a. Shoulder, elbow and wrist of both arms found with confidence ≥ 0.4.
   - b. Each wrist is above eye/ear level by ≥ 0.4 s. (Arms straight up give 0.6–0.75 s; a 45° V gives about 0.45 s.)
   - c. Each elbow is above its shoulder by ≥ 0.15 s.
   - d. Upper arm and forearm each point within 45° of straight up.
   - e. Shoulder-to-wrist length is ≥ 0.75 s. This rejects arms pointing at the camera, which look short.
   - f. Estimated range is between 2 m and the tier limit, and the patch is not cut off at the top of the frame.
   - **Side-on exception:** when the shoulders appear closer together than 0.35 s the person is in profile and the far arm is hidden. Then one arm passing a–e is enough, provided the other arm is either also passing or simply not found. An arm that is confidently found pointing *down* always fails the test.
   - Left/right labels are never used, so facing toward or away makes no difference.
8. **Confirm over time.** Signal seen in at least 70 % of the zoom frames in a 0.8 s window, minimum 5 frames → **LOCK**. This is what rejects a throw, a stretch or a wave passing through the pose.
9. **Pin.** At lock, store the track id, the appearance descriptor, bearing, range and time, and announce it (`/survivor/event: locked`, a bark).
   - Appearance = colour histograms (hue and saturation) of the torso region between the shoulder and hip keypoints, and of the thighs between hips and knees. The pose gives these regions directly, so no segmentation mask is needed.
   - Stored per camera, because the fisheye is red-tinted and the ELPs are not.
10. **Hold the lock.** The lock is on the person, not on the pose – the arms will come down within seconds.
    - Normal case: the track simply continues.
    - After a gap: the pin is carried forward by dead reckoning (gyro yaw, plus the velocity actually sent on `/cmd_vel` scaled by the measured tracking factor), and a person reappearing near the predicted place with matching appearance is the survivor.
    - If two candidates match, prefer the nearer to the prediction; if still unsure, keep heading for the pin and wait for a clearer view. Signalling again always re-locks.
11. **Approach** – next section.

### States

```
SCAN ──person with arms possibly up──> CANDIDATE ──0.8 s confirmed──> LOCKED ──> APPROACH ──0.7 m──> ARRIVED ──> CELEBRATE ──> STAY
  ^                                        |                                       |   ^
  |                                   (hands cut off                            BLOCKED |
  |                                    in a rear camera)                           |   |
  └──────── not the signal ───────────── LOOK                                  REACQUIRE
```

Any state can be paused by a dodge, the joystick, or a fall, and resumes where it was.

- **SCAN:** watching all three cameras. Publishes no velocity, so whatever else is driving keeps driving.
- **CANDIDATE:** zoom passes concentrated on this person. Still no motion.
- **LOOK (turn-to-look):** a person in a rear camera is cut off at the top of the frame, and either their elbows are above their shoulders or their shoulders are out of frame too. Pupper turns in place to bring them into the fisheye, which sees all of them, and judges there. At 2 rad/s a half turn takes about 1.6 s. Limited to once per person per 10 s so a close bystander does not make Pupper spin.
- **LOCKED → APPROACH → ARRIVED → CELEBRATE → STAY:** next section. CELEBRATE is the bounce; STAY is standing beside the survivor in the walking controller.

## Reaching the survivor

- **Command path.** Publishes `/survivor_cmd_vel`. Mux order becomes: joystick → dodge → terrain → **survivor** → Gemini → person follower. The mux treats any message, even zeros, as "active", so SCAN and CANDIDATE publish nothing, and ARRIVED publishes zeros to hold position.
- **On lock, clear the way.** Call the robot server's `/stop/immediate` (empties Gemini's job queue and ends follow mode), check `/system/mode` is locomotion, and if Pupper is lying down or mid-trick, get it standing first. Otherwise the approach commands go to a controller that is not listening.
- **Face, then go.** If the survivor is more than 30° off the nose, turn in place toward them; then walk forward, steering on the bearing.
  - Why facing matters: forward is the fast axis (1.5 m/s against 0.5 m/s sideways), and with the survivor in the fisheye the rear cameras are left watching for balls.
  - Speed: 0.6 m/s beyond 2 m (raise later), 0.3 m/s inside 2 m, stop at 0.7 m. Back away at up to 0.2 m/s if the survivor steps closer than 0.5 m.
  - Stop distance comes from the size-based range, with a hard stop if the person's box grows past a set fraction of the fisheye frame – a second, independent check.

### Who steers around obstacles (decided: route mapping)

Obstacles are route mapping's job. Route mapping is not built yet; it will take the survivor's position from this component and the obstacle picture from terrain agility, and choose the way there. Survivor ID's part is to say where the survivor is, keep the lock, and do the last stretch and the arrival.

**Until route mapping exists** (`drive:=direct`):
- Survivor ID walks straight at the survivor.
- Terrain agility deals with whatever is in the way – steps over it if it is low, walks around it if it is tall – on its own mux input, which sits above this one.
- `/survivor/goal` is the "where was the robot heading" input the terrain plan asks for (its decision T4), so terrain rejoins the line to the survivor after a detour.
- If terrain agility is not running either, the only fallback is blind: when stalled, back up 0.3 m, sidestep, try again, alternating sides and going wider. That is enough to test the approach on its own; it is not a demo behaviour.

**Once route mapping exists** (`drive:=route`):
- Route mapping drives. Survivor ID publishes the goal and stays quiet.
- When Pupper is within about 1.5 m with the survivor in view, survivor ID takes over for the final approach, the stop distance and the arrival. It is the component that can see the person.

**What route mapping can rely on from this component:**
- `/survivor/goal` – the survivor's position relative to Pupper, refreshed at least 5 times a second while they are visible and dead-reckoned while they are not, with an uncertainty and the seconds since they were last seen.
- `/people` – everyone else in view, to keep clear of.
- `/survivor/event` and `/survivor/state` – locked, lost, reacquired, arrived, cancelled.
- **The survivor is not an obstacle.** Terrain agility's scan marks anything that is not floor as an obstacle, and a person 0.7 m ahead is exactly that. The goal and `/people` tell route mapping and terrain which "obstacle" is the destination.

**What this component needs back:** a signal when the route is blocked or no route exists (so the ladder below can move on), and to be told when a step-over is in progress.

### Persistence ladder

Each rung is tried when the one above fails, and none of them ends the mission:

1. Straight line to the survivor (later: route mapping's route).
2. **Something in the way** – terrain agility steps over or goes around. Survivor ID waits and keeps tracking. A step-over takes 20–30 s with Pupper barely moving, so it must not be mistaken for a stall.
3. **Blocked** – terrain reports no way past on either side, or (with no terrain running) Pupper has commanded at least 0.3 m/s forward for 1.5 s without getting closer. Back up, try the other side, then wider; route mapping re-plans once it exists.
4. **Lost sight** – keep heading for the dead-reckoned pin, watching all three cameras for the survivor's appearance.
5. **At the pin but no survivor** – the three cameras already cover 360°, so first just look; then turn slowly once (for anyone cut off in the rear views); then try spots 1 m around the pin.
6. **Still nothing** – go back to rung 4 with the last sighting and keep cycling. Only the joystick, the e-stop, `survivor/cancel` or a new mission command ends the rescue.

Every rung change is published on `/survivor/event` so Gemini can say what Pupper is doing.

### Interruptions

- Dodge: the mux gives dodging control; on `/dodge/event` the approach re-reads the survivor's bearing and carries on.
- Step-over: paused while `/terrain/state` is STEP_OVER; the lock is kept from the cameras.
- Joystick: pauses the approach for as long as the stick is held.
- Fall: Pupper cannot right itself. The state is kept, the event is published, and the approach resumes after a person stands it up.

### Arrival (decided: announce through Gemini Live, then bounce)

1. **Stop** 0.7 m from the survivor, facing them, holding zero velocity.
2. **Announce.** Bark at once (local, no network), and publish `/survivor/event: arrived`. The event reaches Gemini Live as `MISSION EVENT: arrived`, and Gemini says the mission prompt's arrival line ("I'm here. Look down. …"). Gemini's voice follows the stop by a second or two.
3. **Bounce** up and down on the spot to celebrate: about three bounces in three seconds, all four feet staying on the floor, no walking. It overlaps the announcement.
4. **Stay.** Back to the walking controller, holding position beside the survivor, while Gemini carries on with the prompt's arrival questions and the report to the team.

"Not leave the group" is read here as "not leave the ground": feet planted, body moving up and down. Either way it stays on the spot, next to the survivor.

- **The bounce is started by the survivor node, not by Gemini.** It goes through the robot server's trick path (`POST /animation`), so it happens every time, and still happens if Gemini is offline. In the mission prompt it is one of Pupper's reflexes; Gemini's own "no tricks on duty" rule is unchanged.
- **Pupper cannot dodge while bouncing.** A trick hands the legs to a trick controller, and dodging only works in the walking controller. So:
  - `/survivor/state` reads CELEBRATE for the whole time, and the dodge node reports "cannot dodge".
  - The bounce does not start while a ball is being tracked; it waits up to 3 s and is skipped if the air is not clear.
  - Demo rule, as for a step-over: no throws during the celebration.
  - A trick keeps the legs until the next motion, so the node switches back to walking itself afterwards. Getting back takes about 4 s on top of the bounce.
- **Skipped if** the survivor is closer than 0.5 m, Pupper is not standing level, or it has already bounced for this survivor.

**Which bounce (decided: a new `bounce` trick).** It is designed and checked in simulation in `survivor-id/bounce/` (see its README); it still has to be trained.

| Option | Exists today | Time without dodging | Notes |
|---|---|---|---|
| `push_up` trick | yes | about 16 s (1.5 s hand-over, 10.8 s of push-ups, 4.2 s back to walking) | Feet planted, body lowers about 10 cm and comes back, slowly, several times. Not much of a celebration, but it runs today. **The stand-in** for building and testing the arrival sequence until `bounce` is trained and OK'd. |
| New `bounce` trick, designed the way `sneeze` was (`design.py`, then trained in pupper-mjlab) | designed, not trained | about 9 s (2.9 s of bouncing) | **Chosen.** Three bounces, 0.6 s each, body between 116 and 176 mm, feet planted. In simulation no foot leaves the floor, including with a heavier, nose-heavy robot. Needs a training run and, by the course rules, a TA's OK before its first run on the robot. |
| Joint targets through the forward position controllers (the route terrain agility's step-over uses) | plumbing is planned there | 3 s plus two hand-overs | No training. Those controllers have no tilt e-stop, so it needs its own. Worth it only if the step-over node already exists. |
| Scaled-down jump reference, played inside the walking controller | no | about 2 s per bounce, no hand-over | The walking policy was trained on the full jump only, so a smaller one may still hop, or do nothing. Simulation only until shown otherwise. |

### Getting the announcement to Gemini

Mission events do not reach Gemini yet: the web app sends it only the microphone and the camera. The arrival announcement needs this bridge, so it is built with the approach, not left to the end.

- **Robot side:** a small events feed on the robot server, fed from `/survivor/event` and `/dodge/event`. It can be added from sar-pupper's wrapper around the lab server (`personality/run.py` already wraps it), so nothing in `gemini_pupper/robot` changes.
- **App side:** listen to that feed and pass each event to the Live session as text, `MISSION EVENT: arrived range=0.7`. The Live API takes text mid-conversation through the same `sendRealtimeInput` call the app already uses for audio and video. This is about 15 lines in the web app and a rebuild of `web/dist`, which the robot serves itself. It is the one place the lab's web app has to change, and the change is made in `gemini_pupper/web` itself (decided).
- **If Gemini is offline:** the bark and the bounce still happen.

## Interfaces

| Name | Type | Direction | Purpose |
|---|---|---|---|
| `/people` | JSON string (custom message later) | out | Every tracked person: id, bearing, range, arms-up score. Also useful to dodging (where the thrower is), terrain agility and route mapping. |
| `/survivor/state` | JSON string | out | Current state, target id, bearing, range, confidence, seconds since last seen. CELEBRATE tells the dodge node it cannot dodge. |
| `/survivor/goal` | `geometry_msgs/PoseStamped` in `base_link` | out | Where the survivor is: for route mapping, and for terrain agility to rejoin its heading after a detour. |
| `/survivor/event` | string | out | `candidate`, `locked`, `blocked`, `lost`, `reacquired`, `arrived`, `cancelled`. |
| `/survivor_cmd_vel` | `geometry_msgs/Twist` | out | Into `cmd_vel_mux`. |
| `survivor/enable`, `survivor/cancel` | `std_srvs/Trigger` | in | Start/stop watching; drop the current pin. |
| Camera topics, IMU, `/cmd_vel`, `/dodge/event`, `/terrain/state`, `/terrain/event`, `/cmd_vel_mux/active_source` | – | in | Frames, levelling and de-rotation, dead reckoning, resume after a dodge, wait through a step-over, knowing who is driving. |
| Mission-event feed on the robot server | text lines, `MISSION EVENT: <name> [key=value]` | out | What the web app passes to Gemini Live so it can speak (arrival, lock, blocked, …). |
| ZMQ port 5556 | JSON | out | The "where people are" feed the Gemini face's eyes already read, so the eyes keep working without the old detectors. |

## How this fits the existing codebase

| What exists | Where | How survivor ID uses it |
|---|---|---|
| Hailo wrapper for YOLOv8n-seg | `follow_me_pupper/robot/yolo_seg.py` | Pattern for a `HailoYoloPose` class: same device setup, letterbox, box decoding and NMS. The pose model's extra head is 51 numbers per cell = 17 keypoints × (x, y, confidence). |
| Fisheye lens model and rectification | `follow_me_pupper/robot/fisheye_converter.py`, `camera_params.yaml` | Wide pass reuses the equirectangular view as is. Add cached perspective-patch maps for the zoom pass. |
| On-demand detector as a ROS node | `follow_me_pupper/robot/follow_service.py` | Pattern for the node: newest-frame-only subscription, a pipeline thread, Trigger services, the port-5556 eye feed. |
| Target lock and colour re-identification | `follow_me_pupper/lab/target_lock.py` | Same ideas (position gate, then colour), redone in body-frame angles with torso histograms. Not imported: note that `lab/boxes.py`'s `box_bottom` currently returns centre − half-height, which is the top edge of the box. |
| Which way round an obstacle | `follow_me_pupper/lab/avoidance.py` (`measure_crowding`, `choose_side`) | Not usable as is: it works on YOLO object boxes, and the pose model reports people only. Obstacles come from terrain agility's scan. |
| Velocity arbitration | `pupperv3-monorepo/ros2_ws/src/cmd_vel_mux` | Add `/survivor_cmd_vel` to the `inputs` list in our launch file, below terrain's input. 50 ms tick and 500 ms timeout are fine here. |
| Robot state, job queue, falls | `gemini_pupper/robot/robot_api.py`, `server.py` (port 8000) | `/stop/immediate` and `/system/mode` on lock; `/animation` for the bounce and `/walking/activate` to return to walking after it; fall state respected. Later: Gemini tools `survivor_status` and `cancel_rescue`. |
| Person followers | `follow_service.py`; monorepo `hailo_detection` + `person_follower` | **Not started in the mission launch** – each would take the Hailo. `gemini_pupper.launch.py` starts one or the other, so `sar.launch.py` builds its own list instead of including that file. Gemini's "follow" tool is disabled in the mission prompt or re-pointed at our tracker. |
| Sounds | `follow_me_pupper/robot/sounds/` | Bark on lock and on arrival. |
| Tricks | `gemini_pupper/tricks/` (`push_up`, and `sneeze/design.py` as the pattern for a designed trick) | The arrival bounce: `push_up` as the stand-in; the new `bounce` trick (`survivor-id/bounce/`) is installed here by `get_trick.py` once trained. |
| Mission personality | `sar-pupper/personality/` (`prompt.md`, `run.py`) | Already has the `arrived` call-out and the `MISSION EVENT:` format; its server wrapper is where the event feed is added. |
| Terrain agility | `terrain-agility/plan.md` (project doc) | Handles obstacles on the way (`/terrain/scan`, `/terrain/state`, its own mux input above ours). |
| Camera models, mounting poses, ELP capture, recorder | planned in `projectile-dodging/plan.md` | Shared, not duplicated. The camera-model code moves to a module both components import. |

Course rules carried over: do not edit the monorepo in place, no keys in the repo, never raise speed or tilt limits to make something work, and **never move the robot from code without someone at the e-stop**.

### Proposed layout

Plans live in their own folders; code follows the layout the dodging plan proposed.

```
sar-pupper/
  projectile-dodging/plan.md
  survivor-id/plan.md        this file
  survivor-id/bounce/        (exists) the arrival bounce: design, reference motion, sim checks, how to train it
  cameras/                   shared: lens models, mounting poses, pixel → body-frame angles
  dodge/                     (dodging plan)
  personality/               (exists) mission prompt and the server wrapper that will carry mission events
  survivor/                  pure Python, no ROS: pose decode, gesture rule, people tracker,
                             appearance, pin + dead reckoning, approach state machine
  robot/                     ROS: survivor_node.py (owns the Hailo), approach_node.py, sar.launch.py, parameters
  tools/                     recorder, replay, gesture labeller, live overlay viewer, range and timing tests
  config/                    thresholds, envelope numbers, per-camera colour correction
  models/                    yolov8s_pose.hef – fetched by a script, git-ignored
  tests/                     off-robot tests: synthetic keypoints and recorded clips
  data/                      recordings (git-ignored)
```

## Build order

**Phase 0 – measure (decides the envelope)**
1. Fetch `yolov8s_pose.hef` from the same model-zoo release as `yolov8n_seg.hef` (v2.14.0, `hailo8l`) – confirm it is there and loads with the robot's HailoRT. Measure inferences per second end to end with Python decoding.
2. Stills: one person does the signal at 2, 3, 4 and 6 m, at 8 bearings, facing 4 ways. Save frames from all cameras, run wide and zoom passes, and plot wrist/elbow confidence against distance per camera. Sets the Tier A range and settles `s` vs `m` model (S3).
3. ELPs: real vertical view, and the closest distance at which raised hands stay in frame at image centre, 45° off-axis and directly behind.
4. Exposure: run the pose model on frames taken at the short exposure dodging wants. If people come out too dark, the two components need a compromise setting.
5. Low viewpoint: fisheye pose quality at 1, 1.5 and 2 m.
6. CPU cost of JPEG decode, rectification and patch rendering with the dodge pipeline running. Standing lens heights.

**Phase 1 – record.** Using the dodging recorder: the signal in every sector/distance/facing cell; the full list of negatives from the success criteria; two and three people together; the survivor lowering their arms and walking; the robot walking past people.

**Phase 2 – gesture, offline.** Pose decoding, body-frame conversion, the rule and the time filter. Unit tests on synthetic keypoints for every branch of the rule; then tuning on the recordings, false locks first, misses second. Thresholds are tuned on outputs from the robot's quantised model, not a laptop's float model.

**Phase 3 – identify live, robot not moving.** `survivor_node` with wide/zoom scheduling, an overlay viewer, bark on lock. Test against the detection and false-lock criteria.

**Phase 4 – pin and hold, robot not walking.** Tracker, cross-camera handover, appearance, dead reckoning, re-acquire. Tested with people crossing and hiding, and by turning the robot with the joystick.

**Phase 5 – approach and arrival on open floor.** Mux input, queue clearing, face-then-go, stop distance. Then the arrival sequence: the mission-event bridge to Gemini, the announcement, the bounce with `push_up` as the stand-in, and the switch back to walking. Then turn-to-look. Someone at the e-stop for every run.

**Phase 6 – obstacles and persistence.** With terrain agility running: sharing the goal, waiting through a step-over, the survivor not treated as an obstacle, blocked handling, re-acquire after a detour, the search at the pin. The blind fallback for runs without terrain. In parallel: train the `bounce` trick, score the checkpoint (`play_motion`, `check_feet.py`), and get the TA's OK.

**Phase 7 – integrated.** Throws during the approach, the remaining Gemini call-outs and tools, the full demo script. Hand-over to route mapping (`drive:=route`) whenever it is ready; nothing earlier waits for it.

**Phase 8 – stretch.** Tier B ranges, a moving survivor, the keypoint classifier, the "N" cancel signal, a queue of survivors.

## Things that are easy to miss

1. **The rear cameras cut off raised hands at close range** – inside about 1.8 m at best, and inside about 3.3 m directly behind. Without turn-to-look, the place a person is most likely to signal from when Pupper has walked past them is the place it cannot see the signal.
2. **From floor level, reaching toward the robot looks like arms up.** Height of the wrists above the eyes, in torso lengths, arms up vs arms stretched toward Pupper: 0.55 vs 0.61 at 1.5 m (no difference), 0.63 vs 0.30 at 2 m, 0.71 vs −0.04 at 3 m. Hence no new locks inside 2 m, the arm-length test, and a "bend down to pet it" clip in the negatives.
3. **The arms come down.** A detector that only knows the pose loses the survivor two seconds after finding them.
4. **The ball thrower is a person in view, with an arm going up.** Two arms plus the 0.8 s hold is what separates a throw from the signal; it needs its own test.
5. **"Up" is not image-up.** At the fisheye's edges, and whenever the body rolls or pitches while trotting, image-up and real up differ. The rule works in levelled angles.
6. **Side-on, one arm hides the other.** Without the profile exception the signal fails from two of the four facing directions.
7. **One Hailo.** The Lab 5 follow service and the monorepo detector each grab it. If either is running, the survivor node fails to start (or they do).
8. **No odometry, so the pin drifts** whenever the survivor is out of view – expect 15–25 % of the distance walked (*est.*). Keep detours short and trust eyes over memory.
9. **Dodging and survivor ID share the cameras' exposure.** Dodging wants short and fixed; people need to be bright enough to find.
10. **Colours differ between cameras.** A shirt's histogram from the red-tinted fisheye will not match the same shirt in an ELP unless they are stored per camera or colour-corrected first.
11. **Size-based range assumes an adult torso.** A child, or someone crouching, reads as further away than they are. Bearing is unaffected, and the independent box-size stop covers the last metre.
12. **A queued Gemini move or a trick in progress** means the walking controller may not be the active one when the lock happens.
13. **The mux counts zeros as a command.** A node that publishes zero velocity while idle silently blocks everything below it.
14. **Speed near people.** 1.5 m/s is available; inside 2 m the cap is 0.3 m/s.
15. **Pictures of people.** A poster, a screen or a window reflection of someone with their arms up will pass the rule. Out of scope, but worth knowing on demo day.
16. **Heat.** The Hailo running all mission long makes the robot run warmer than Lab 5's on-demand use did.
17. **Scoring needs ground truth.** Each trial needs the true bearing, distance and facing written down, and for approach runs a floor tape measure of where Pupper stopped.
18. **Terrain agility will see the survivor as an obstacle.** Its scan marks anything that is not floor, so without being told otherwise it will try to walk around the person Pupper is walking to.
19. **The pose model only reports people.** With `yolov8n_seg` gone there are no boxes for chairs or crates, so nothing in this component can see an obstacle. That is terrain agility's scan, and later route mapping.
20. **No dodging during the bounce**, and the legs stay with the trick controller afterwards until something switches back to walking.
21. **A bounce 0.7 m from someone's feet** has to stay on the spot. The `push_up` stand-in keeps its feet planted; a new trick must be trained to.
22. **Gemini cannot be told anything yet.** Until the event bridge exists, an arrival is a bark and a bounce with no words.

## Decisions

### Made
- **Readings of the request** – confirmed 2026-11-23: any bearing and any facing direction; both arms, not one; persistent but not pushing through.
- **S1 – more than one person signalling:** first come, first served; later signals are queued.
- **S2 – who steers around obstacles:** route mapping (not built yet), using terrain agility and this component. Until then, direct approach with terrain agility handling what is in the way.
- **S4 – on arrival:** announce through Gemini Live and bounce up and down on the spot, feet on the floor.
- **S8 – which bounce:** a new, purpose-made `bounce` trick (`survivor-id/bounce/`), with `push_up` standing in until it is trained and OK'd.
- **S9 – where the web-app change for mission events lives:** in `gemini_pupper/web` itself.

### Open
- **S3 – `yolov8s_pose` or `yolov8m_pose`.** After Phase 0 item 2.
- **S5 – always watching, or only when the mission says "search"?** Plan assumes always on in the mission launch, with `survivor/enable` to switch it.
- **S6 – how a pin is cancelled:** joystick button, a Gemini voice command, or the "N" signal.
- **S7 – may a turn-to-look interrupt whatever is driving** (Gemini or route mapping) for up to 3 s? Plan assumes yes.
- **S10 – hand-over distance to route mapping** (plan assumes survivor ID takes the last 1.5 m), and the "blocked / no route" signal it sends back. Settle when route mapping is designed.

## What was built, and where it departs from this plan (2026-11-23)

Phases 2–7 are written and pass their off-robot tests (74 of them, plus the dodging and personality checks). Phases 0 and 1 – measuring and recording on the robot – could not be done from a desk, so every number that was marked *est.* still is. `DEPLOY.md` is those phases, in order.

**Departures, each for a reason found while building:**

1. **Layout.** The repo's real convention turned out to be one folder per component (`projectile-dodging/{dodge,robot,tools,config,tests}`), so the code is in `survivor-id/{survivor,robot,tools,config,tests}`, not at the top level. The lens models stay in `projectile-dodging/dodge/camera_models.py` and are imported from there; no shared `cameras/` module was split out.
2. **Launch.** `robot/sar_survivor.launch.py` includes the dodging launch file with `follower:=false` instead of building its own controller list. One mux file (`projectile-dodging/robot/mux.yaml`) serves both, now with `/terrain_cmd_vel` and `/survivor_cmd_vel`.
3. **Step 6–7, units.** Heights are compared as differences of the *tangent* of elevation, divided by the same measure of the torso, not as angles in units of `s`. From cameras this low, which look steeply up at a close person, angle-based range over-read by 13 % at 2 m and the thresholds shifted with distance; tangents give metres over metres at any range. Range = 0.48 m ÷ (tan elevation of shoulders − tan elevation of hips).
4. **Step 7, thresholds.** The start values did not pass the reference photo itself. YOLOv8s-pose reads that photo as: wrists 0.37–0.40 torso lengths above the ears (the plan assumed 0.6–0.75 for straight arms; the photo's arms are a relaxed V), arm segments 35–39° from vertical, shoulder-to-wrist 0.80–0.83. So: (b) 0.40 → **0.25**; (d) 45° per segment → **50° for the whole arm, 65° per segment** (at 3 m the elbow moves a few pixels between frames and a short segment swings 10–15°); (e) 0.75 → **0.5**. The photo's joints are kept in `tests/test_survivor.py` so a later change cannot silently exclude it.
5. **"Easy to miss" 2 is worse than estimated.** With (b) at 0.25, two arms held out level toward the robot read as the signal out to about 2.3 m, not 2 m. The 2 m gate and the refusal of a downward reach stand. This cannot be fixed with one camera at this height.
6. **Side-on exception.** Added: a far arm the model half-finds (confidence between 0.1 and 0.4) with its wrist below its shoulder refuses the signal. Without it, one raised arm seen side-on passed whenever the other was hidden. If the model returns nothing at all for the far arm, it still passes (tested as a known limit).
7. **Turn-to-look** also fires for a person cut by the *side* of a rear frame within 25° of straight behind (the seam: the two rear cameras overlap by 5°, less than a person's width), and its test is "no arm was seen to be down" rather than "elbows above shoulders". It turns at 1.5 rad/s, not 2.
8. **Rear cameras lock directly from about 2.8–3 m**, not 2 m: closer, raised hands reach the top of the frame and it turns to look instead (in simulation; stage 3 of `DEPLOY.md` measures it).
9. **Tracker.** Tracks live as positions in a dead-reckoned floor frame (bearing, range from body size) rather than azimuth/elevation/range, with a sideways-speed estimate so two people crossing keep their own tracks. A camera that sees only part of a person has no say while another sees all of them.
10. **On lock** the queue is cleared with `/stop/immediate` as planned. Note what that server does: a trick that is still *playing* is stopped by relaxing the motors, so Pupper lies down and is then stood up again.
11. **The trick endpoint queues and returns at once**, so the approach node polls `/queue/status` to know when the bounce is over, and the server itself returns to walking after a trick that ends standing.
12. **Interfaces.** `/survivor/event` carries JSON (`{"event": "locked", ...}`) like `/dodge/event`. Added: `/survivor/perception` (survivor node → approach node), `/survivor/debug/compressed`, `/survivor/disable`, and `/survivor/motion` (`std_srvs/SetBool`, the dry-run switch). The goal's uncertainty and seconds-since-seen are in `/survivor/state`, since a `PoseStamped` cannot carry them.
13. **Dry run is the default**, as for dodging: without `motion:=true` the approach node reports and sends nothing, and mission events are not passed to Gemini.

**In the plan, not built:**
- Range cross-checks (eye-to-ankle height, triangulation in the side overlap zones), and splitting the fisheye into halves for far people.
- Masking the robot's own parts in the rear views (the pose model has not been seen to mistake them for people; untested on the robot).
- Visiting queued signallers after the first arrival (their ids are reported), the "N" cancel signal, the keypoint classifier.
- Gemini tools `survivor_status` and `cancel_rescue`; the follow tool is still registered (the mission prompt tells Gemini not to use it).
- A gesture labelling tool. `tools/replay.py` with `--frames` is what there is.
- A trained `bounce`. `push_up` stands in.

## Sources

- Hailo Model Zoo, Hailo-8L pose estimation models: https://github.com/hailo-ai/hailo_model_zoo/blob/master/docs/public_models/HAILO8L/HAILO8l_pose_estimation.rst
- Seeed Studio, YOLOv8 pose estimation on Raspberry Pi hardware with Hailo-8L (27 FPS): https://wiki.seeedstudio.com/yolov8_pose_estimation_on_recomputer_r1000_with_hailo_8l/
- Hailo community, two ROS 2 nodes sharing one Hailo on a Pi 5 (scheduler and multi-process service): https://community.hailo.ai/t/switching-between-two-ros-2-nodes-using-hailo-on-rpi5-device-still-reported-as-busy/18740
- Liu & Szirányi, Real-Time Human Detection and Gesture Recognition for On-Board UAV Rescue (Sensors 2021): https://pmc.ncbi.nlm.nih.gov/articles/PMC8003912
- Distress hand signals, including the "Y" signal to a rescue helicopter: https://en.wikipedia.org/wiki/Distress_hand_signal
- Hardy, Dasmahapatra & Kim, Can Super Resolution Improve Human Pose Estimation in Low Resolution Scenarios? (2021): https://arxiv.org/abs/2107.02108
- Algabri & Choi, Deep-Learning-Based Indoor Human Following of Mobile Robot Using Color Feature (Sensors 2020): https://pmc.ncbi.nlm.nih.gov/articles/PMC7273221
- Person Re-Identification for Robot Person Following with Online Continual Learning (2023): https://arxiv.org/abs/2309.11727
- Google, Live API capabilities guide (sending text during a session): https://ai.google.dev/gemini-api/docs/live-guide
- Local code read for this plan: `follow_me_pupper/`, `gemini_pupper/`, `pupperv3-monorepo/ros2_ws/src/{cmd_vel_mux,hailo,person_follower}`, `projectile-dodging/plan.md`, `personality/`, and the terrain-agility plan in the project docs.
