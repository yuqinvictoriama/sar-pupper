# Projectile dodging

Pupper sees a thrown neon-green foam pickleball with its three cameras and steps
out of the way, from any direction around it.

- **`plan.md`** – the design, the operating limits and why.
- **`DEPLOY.md`** – how to put this on the robot, step by step.

## What is where

```
dodge/      the reflex itself, plain Python (no ROS): lens models, colour detection,
            3D tracking, hit prediction, escape choice, state machine, simulator
robot/      what runs on the Pupper: USB camera capture, the ROS node, the launch
            file, start/stop scripts, mux configuration
tools/      setting up and tuning: find and check the cameras, tune exposure, fit the
            fisheye's rim, calibrate colour and lenses, record, replay, measure latency
            and step response, pre-flight check, simulation report
config/     dodge.yaml (all numbers) and calib/ (what the calibration tools write)
tests/      off-robot tests
data/       recordings made on the robot (not in git)
```

## Try it without a robot

```bash
python3 -m unittest discover -s tests     # 47 tests, a few seconds
python3 tools/sim_report.py               # simulated throws from every direction
```

Needs Python 3.11+, numpy, opencv-python, pyyaml, scipy – all already on the Pupper.

## How it works, in one paragraph

Each camera thresholds the frame for the ball's colour and measures the blob's
position and area. The lens model turns that into a direction and an angular
size; the ball's known diameter turns the size into a range. A Kalman filter
tracks the ball in 3D relative to the robot, assuming free flight. The track is
flown forward to see whether it will touch the robot's outline and when. If it
will, and there is only just enough time left to get clear, the robot is sent
sideways to the ball's path at the fastest command the walking policy accepts,
for a fraction of a second, then held still until steady, then control goes
back to whatever was driving. The dodge command enters the velocity mux below
the joystick and above everything else.
