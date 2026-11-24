# Survivor identification

Pupper watches all three cameras for a person holding both arms up, pins that person as the
survivor, walks to them, stops 0.7 m away, barks, tells Gemini it has arrived, and bounces on
the spot.

- **`plan.md`** – the design, the operating limits and why. Its last section lists where this code departs from it.
- **`DEPLOY.md`** – how to put this on the robot, step by step. Nothing moves until its stage 7.
- **`bounce/`** – the arrival bounce trick: designed and checked in simulation, not trained yet.

**Status (2026-11-23): written and tested off the robot only.** Nothing here has run on the
Pupper, on ROS, or on the Hailo. See "What has and has not been checked" below before trusting any of it.

## What is where

```
survivor/   the logic, plain Python (no ROS): camera views, pose decoding, the arms-up rule,
            people tracker and pin, dead reckoning, the approach / arrival state machine
robot/      what runs on the Pupper: the Hailo pose model, three ROS nodes, the launch file,
            start / stop / watch scripts
tools/      try photos, replay recordings, live view in a browser, speed test, pre-flight check
config/     survivor.yaml (every number) and calib/ (your measured overrides)
tests/      off-robot tests, with a stand-in world and a stand-in ROS
bounce/     the arrival trick
models/     the pose model goes here (tools/download_model.sh; not in git)
```

It shares code with `../projectile-dodging`: the lens models and mounting poses, the camera
calibration, the recording format, the USB capture, and the launch file and velocity mux.
Two files there changed for this: `robot/mux.yaml` (two more inputs) and `robot/dodge_node.py`
(passes the rear cameras' frames on). `../personality` gained `mission_feed.py`, and
`gemini_pupper/web/services/geminiLive.ts` gained a listener for it.

## Try it without a robot

```bash
cd sar-pupper/survivor-id
python3 -m unittest discover -s tests          # 74 tests, under a minute
```

Needs Python 3.10+, numpy, opencv-python, pyyaml (scipy if you have it) – all on the Pupper already.

On a laptop with `pip install ultralytics` you can run the real pose model, slowly, on photos:

```bash
python3 tools/try_photos.py some_photos/ --out /tmp/out     # who is found, who shows the signal, annotated copies
```

## How it works

1. **Find people.** One camera per cycle (10 cycles a second, cameras in turn) is shrunk to 640 px
   and given to YOLOv8s-pose on the Hailo, which returns each person's box and 17 joints. The
   fisheye is first unrolled to the same 700 × 572 view Lab 5 uses.
2. **Look closer.** A 4 m person is about 60–100 px tall in that view: enough to find, not to
   trust the wrists. So each person who matters gets a second pass on a patch cut around them
   at the camera's full resolution, rendered through the lens model so they stand upright.
3. **Where are they.** Every joint becomes a direction relative to the robot's body, levelled
   with the IMU. The shoulder-to-hip length (0.48 m on an adult) gives the distance.
4. **Is it the signal.** Per arm: wrist above the ears, elbow above the shoulder, arm pointing
   up, all in torso lengths so distance does not matter. Both arms, seen in at least 70 % of the
   readings over 0.8 s. Left and right are never told apart, so facing toward or away is the same.
5. **Pin.** The first person to hold the signal is the survivor. The pin is on the person, not
   the pose: a tracker follows them when the arms come down, across cameras, and past other
   people, using position, sideways motion and the colours of their clothes. Out of view, the
   pin is carried along by dead reckoning (IMU heading and the commanded velocity).
6. **Go.** A state machine turns the pin into velocity commands for the mux, below the joystick,
   dodging and terrain agility: face them, walk, slow down inside 2 m, stop at 0.7 m. Blocked →
   back up and go round. Lost → walk to where they were, look round, search, repeat. It never
   gives up by itself.
7. **Arrive.** Bark, tell Gemini (`MISSION EVENT: arrived range=0.7`), play the bounce trick
   through the robot server, go back to walking, stay.

`robot/start.sh` starts everything in **dry run**: it watches, locks and reports, and moves
nothing. `robot/watch.sh motion on` (or `start.sh --motion`) lets it walk.

## What has and has not been checked

| | Checked how | Not checked |
|---|---|---|
| Geometry (lens models, levelling, range, bearing) | 74 unit tests through the real lens models; range within 12 %, bearing within 3° at 2–6 m, all round | The real cameras' calibration and mounting |
| Pose decoding | Matches Ultralytics' own decoder to the pixel on 61 people in 22 photos | The Hailo's quantised output (same tensor layout expected; `tools/bench_hailo.py` prints it) |
| The arms-up rule | The photo that defines the signal reads as the signal; none of 60 other people in 22 everyday photos does; synthetic poses from every direction | Real people seen from 13 cm off the floor. **Thresholds are start values.** |
| Whole perception chain with the real model | The reference figure pasted into synthetic fisheye and rear-camera recordings, 2.2–4 m, 0° to ±80° and behind: locked 0.8 s after appearing, every time | Live cameras, exposure, motion blur, clutter, real walking |
| Tracking and the pin | Tests: arms come down, a bystander walks in front (hiding the survivor), a second signaller, lost and found, a stranger near the pin | Real clothes, real crowds |
| Approach, detours, search, arrival | Closed-loop simulation with a robot that moves as told (and one that moves 30 % slower) | The robot. Stopping distance, turning, the detour are all untried on the floor |
| ROS nodes | Driven with a stand-in ROS: topics, JSON, services, calls to the robot server | ROS itself, timing, the launch file, the scripts (syntax-checked only) |
| Mission events to Gemini | The feed's routes on a FastAPI app like the lab's; the web app type-checks and builds | A live Gemini session speaking an event |
| The bounce | `bounce/README.md`: designed, feet stay down in simulation | Trained policy, TA's OK, the robot |

## What it gets wrong (known)

- **Arms held out at the robot look like arms up, inside about 2.3 m.** From a camera 13 cm off
  the floor a hand that is closer sits as high in the picture as a hand that is higher; one
  camera cannot tell which. Nothing closer than 2 m can lock, and a reach down toward the robot
  is refused at any range, but two arms held out level toward Pupper at 2–2.3 m read as the signal.
- **Side-on, one raised arm can pass** if the pose model returns nothing at all for the far arm.
  Usually it returns a low-confidence guess, and a guess that hangs down refuses the signal.
- **Close behind, the rear cameras cut off raised hands** (inside about 2.8 m; straight behind,
  a person straddles the two rear cameras at any range). Pupper then turns to look with the
  fisheye, which adds a second or two – and, in dry run, cannot.
- **Range assumes an adult torso** and is good to about ±10–15 %. Measure yours in DEPLOY stage 3.
  Until then stand 2.3 m or more away for a reliable lock.
- **A picture of a person with their arms up passes.** So does a reflection.
- **Dead reckoning has no odometry**: while the survivor is out of view the pin drifts with every
  step. Eyes are trusted over memory as soon as they are seen again.
- **It cannot see obstacles.** The pose model reports people only. Getting round things is
  terrain agility's and route mapping's job; without them the only fallback is the blind
  back-up-and-sidestep.
- **Later signallers are recorded, not visited.** Their ids are in `/survivor/state` (`queued`);
  going on to them after the first arrival is not built.

## Topics and services

| Name | Type | |
|---|---|---|
| `/people` | `std_msgs/String`, JSON | everyone tracked: id, bearing, range, arms-up score, which is the survivor |
| `/survivor/state` | JSON, 5 Hz | SCAN, CANDIDATE, LOOK, LOCKED, APPROACH, BLOCKED, REACQUIRE, ARRIVED, CELEBRATE, STAY or PAUSED; the pin (bearing, range, seconds since seen, uncertainty); `can_dodge` |
| `/survivor/goal` | `geometry_msgs/PoseStamped`, `base_link` | where the survivor is, 10 Hz, dead-reckoned while unseen (for route mapping and terrain agility) |
| `/survivor/event` | JSON `{"event": ...}` | candidate, locked, lost, reacquired, cancelled, blocked, arrived |
| `/survivor_cmd_vel` | `geometry_msgs/Twist` | into the mux; silent unless a survivor is pinned and motion is on |
| `/survivor/perception` | JSON, 10 Hz | what the survivor node hands the approach node |
| `/survivor/debug/compressed` | JPEG, 2 Hz | the annotated views (`tools/view.py`); only drawn while someone subscribes |
| `/survivor/enable`, `/survivor/disable`, `/survivor/cancel` | `std_srvs/Trigger` | watch / stop watching and free the Hailo / drop the pin |
| `/survivor/motion` | `std_srvs/SetBool` | let the approach node move the robot |

Reads: the three camera topics, the IMU, `/cmd_vel`, `/cmd_vel_mux/active_source`, `/dodge/state`,
`/dodge/event`, `/terrain/state`, `/terrain/event`, the controller manager, the robot server (port 8000).
