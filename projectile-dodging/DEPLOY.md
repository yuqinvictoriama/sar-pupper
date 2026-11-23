# Deploying projectile dodging on the Pupper

Work through the stages in order. Nothing moves the robot until stage 7, and
the dodge node stays in **dry run** (it reports the dodge it would make, and
never moves) until you arm it in stage 8.

All commands run **on the Pupper** (over SSH), from `~/sar-pupper/projectile-dodging`,
unless a line says "on the Mac".

**Safety, throughout:** the joystick e-stop (right-stick press) always works and
always outranks the dodge reflex. Whenever the robot can move (stages 7 and 8),
have someone at the joystick, the robot on open floor with 1.5 m clear around
it, and nothing it could fall off.

---

## 1. Get the code onto the Pupper

On the Mac (the code is not committed yet):

```bash
cd ~/Desktop/CS123/pi/sar-pupper
git add projectile-dodging
git commit -m "Projectile dodging: core, robot node, tools, plan"
git push -u origin main
```

On the Pupper:

```bash
git clone https://github.com/yuqinvictoriama/sar-pupper.git ~/sar-pupper     # if it is already there: cd ~/sar-pupper && git pull
cd ~/sar-pupper/projectile-dodging
which v4l2-ctl || sudo apt install -y v4l-utils     # used to fix camera exposure and white balance
```

Everything else it needs (numpy, OpenCV, PyYAML, SciPy, ROS 2 Jazzy) is already
on the course image. It also expects `~/gemini_pupper` and `~/pupperv3-monorepo`
where the labs put them.

## 2. Check the code runs (no robot needed)

```bash
python3 -m unittest discover -s tests     # expect: Ran 32 tests ... OK
python3 tools/sim_report.py --n 20        # simulated throws; takes about a minute on the Pi
```

If the tests fail here, stop and fix that first: the rest will not work.

## 3. Start the robot stack without the dodge node

Calibration tools open the USB cameras themselves, so the dodge node must not
be holding them yet.

```bash
bash ~/gemini_pupper/robot/stop.sh        # make sure nothing else is running
bash robot/start.sh --no-dodge            # course stack + mux with the dodge input; about a minute
source robot/env.sh                       # in every terminal you use from here on
```

`start.sh` refuses to start if a robot stack is already running. If it says so
and `stop.sh` did not clear it, find what started it (`pgrep -af ros2_control_node`).

## 4. Tell it which USB camera is which

```bash
python3 tools/find_cameras.py             # lists the cameras and their modes, saves data/snapshots/usb*.jpg
python3 tools/find_cameras.py --assign    # cover the LEFT rear camera when asked
```

"Left" is the robot's left. The result goes to `config/calib/devices.yaml`.
Cameras are identified by USB port, so re-run this if you move a plug.

Look at the mode list it prints. The config asks for 1280×720 at 120 fps
(`cameras.elp_*.capture` in `config/dodge.yaml`). If your cameras do not offer
that, set the closest mode that is at least 1280 wide.

Then look through each camera:

```bash
python3 tools/view.py elp_left            # open http://<pupper-ip>:8090  (Ctrl+C when done)
python3 tools/view.py elp_right
python3 tools/view.py fisheye
```

Check three things in each:

| Check | If not |
|---|---|
| Picture is the right way up | Camera is mounted rotated: put `roll_deg: 180` (or 90 / -90) under that camera's `extrinsics` in a new file `config/calib/mounting.yaml` (same layout as `dodge.yaml`) |
| The ball looks clearly neon green, not dark or washed out | ELPs: raise or lower `exposure_100us` (units of 0.1 ms) or set `gain`, under that camera's `capture`. Fisheye: exposure is automatic unless you start with `bash robot/start.sh --fisheye-exposure 4000` (microseconds) |
| The ball is not smeared when you wave it | Shorten the exposure (and add light) |

## 5. Calibrate (each camera, in the room and lighting you will run in)

```bash
python3 tools/calibrate_color.py elp_left                     # follow the two prompts
python3 tools/calibrate_color.py elp_left --distance 1 2 3    # ball held at 1, 2, 3 m from that lens
```

Repeat for `elp_right` and `fisheye` (for the fisheye use `--distance 0.75 1.5 2.5`).
Then check the result:

```bash
python3 tools/view.py elp_left
```

Hold the ball at known places and read the label on its box. It should show:

| Ball held | Bearing | Elevation |
|---|---|---|
| straight behind the robot, at camera height | about ±180° | about 0° |
| behind and to the robot's left | about +135° | – |
| directly to the robot's left | about +90° | – |
| straight ahead (fisheye) | about 0° | – |
| above the camera's eye level | – | positive |

and a range within about 10 % of the tape measure. If left and right are
swapped, redo stage 4. If bearings are mirrored or upside down, fix `roll_deg`.
`view.py --mask` shows exactly which pixels count as ball; nothing else in the
room should light up.

Timing (the robot must be standing still or on its stand, looking at something with detail):

```bash
python3 tools/measure_latency.py elp_left --write     # tap the robot firmly 5 times
python3 tools/measure_latency.py elp_right --write
python3 tools/measure_latency.py fisheye --write
```

Optional, for better accuracy later: `tools/calibrate_camera.py` (lens, with a
printed checkerboard) and `tools/calibrate_extrinsics.py` (exact camera angles).
Each explains itself with `--help`. The defaults from the CAD are good enough to start.

## 6. Record throws and replay them (still nothing moves)

```bash
python3 tools/record.py --seconds 30 --name rear-lobs-01     # someone lobs balls at the robot meanwhile
python3 tools/replay.py data/recordings/rear-lobs-01
```

`replay.py` prints a `DODGE` line for every throw the reflex would have reacted
to: where the ball came from, time left, and the move chosen. Record at least a
few throws from behind, each side and the front, one or two that clearly miss,
and 30 s of the empty room. You want a dodge for every on-target throw and none
otherwise. This is also how to tune: change a number in `config/dodge.yaml`,
replay, compare. For many recordings, list them in a CSV and use
`python3 tools/replay.py --labels data/labels.csv`.

## 7. Dry run with the dodge node, then measure the robot's response

```bash
bash robot/stop.sh
bash robot/start.sh                 # now with the dodge node, NOT armed
source robot/env.sh
python3 tools/check_setup.py        # every line should say OK (the walking-controller line needs the robot awake)
bash robot/watch.sh                 # live: state, each camera's frame rate and lag, the tracked ball
```

Throw balls. `watch.sh` prints an `EVENT ... "dodge" ... "dry_run": true` line
for each one it would have dodged, and the node saves a short recording of every
such moment under `data/events/` (replay them the same way). In the `watch.sh`
line, "lag" is how old a frame is when it has been processed and "cpu" is the
processing time per frame. If lag climbs or `top` shows the Pi near 100 % on all
cores, lower `max_process_fps` for the ELPs.

Now the first step that **moves the robot**. Stand it up:

```bash
bash robot/walk.sh                  # stands up in gemini_walk (or press X on the joystick: slower controller)
```

and measure how it really accelerates, gently first:

```bash
python3 tools/step_response.py --vy 0.5
python3 tools/step_response.py --vx 0.6
python3 tools/step_response.py --vx 1.0
python3 tools/step_response.py --vx 1.0 --vy 0.5 --write     # the kind of move a dodge makes; saves the lag
python3 tools/step_response.py --vx -0.8
```

Each run prints the delay before the robot moves, the time to cover
0.15–0.30 m, and the peak body tilt. Stop raising the speed if the tilt passes
about 35° or the robot stumbles. Then put what you measured into the simulation
to see what to expect:

```bash
python3 tools/sim_report.py --lag <fitted lag> --delay <measured delay>
```

If the robot was steady at 1.0 m/s and you want more margin, try
`--vx 1.5` / `--vx -1.5`, and if those are steady too, raise `vx_forward` and
`vx_backward` under `escape:` (in a `config/calib/` file or `dodge.yaml`).

## 8. Arm it

With the robot standing, the floor clear and someone at the joystick:

```bash
bash robot/watch.sh arm             # from now on a predicted hit moves the robot
bash robot/watch.sh disarm          # back to dry run
```

Start with the easy directions and work toward the hard one:

1. underhand lobs from the **side**, 3–4 m away, peaking around head height;
2. from **behind** and the rear diagonals;
3. the front diagonals;
4. straight from the **front** – the weakest direction (see `plan.md`).

To start the stack already armed: `bash robot/start.sh --armed`. With Gemini
Pupper as well: `bash robot/start.sh --gemini` (add `--kiosk` for the on-robot
app, `--armed` to arm). Always stop with `bash robot/stop.sh`.

After a dodge the node holds the robot still until it is steady, then goes
quiet; the mux hands control back to Gemini or navigation half a second later.
Each dodge is announced on `/dodge/event` (JSON) so a planner can re-plan.

## 9. If something is off

| What you see | Likely cause | What to do |
|---|---|---|
| `check_setup.py`: mux does not listen to `/dodge_cmd_vel` | Stack was started by Gemini's own script | `bash robot/stop.sh`, then `bash robot/start.sh` |
| Tools find no topics / no IMU | Terminal lacks the ROS environment | `source robot/env.sh` |
| "CANNOT DODGE: no walking controller active" | Robot is relaxed, mid-trick or fell | `bash robot/walk.sh`, or wake it through Gemini |
| A camera shows `---` in `watch.sh` | Unplugged, wrong port, or another program has it | Stage 4; make sure `view.py`/`record.py` are not still running |
| No track although the ball is in view | Colour profile does not match this lighting | Redo `calibrate_color.py`; check with `view.py --mask` |
| Tracks appear with no ball | Something else in view is ball-coloured | Remove it, or tighten `s_min` / the hue range for that camera |
| Range reads consistently wrong | Size calibration | `calibrate_color.py <cam> --distance ...`; check `ball.diameter_m` |
| Dodges balls that would miss by a lot | Prediction too generous | Lower `threat.k_sigma` or `threat.sigma_cap_m` |
| Reacts too late from the front | Fisheye sees the ball late | `bash robot/start.sh --fisheye-fps 60`; lower `detector.min_area_px`; throw from 3 m rather than 4 m |
| Reacts in time but does not get clear | Robot slower than the config assumes | Stage 7 numbers into `escape.accel_lag_s` / `react_s`; raise `vx_*` if steady |
| Moves the wrong way | Left/right swapped or a camera is rotated | Stage 4 and the bearing table in stage 5 |
| Robot stumbles or trips the tilt e-stop when dodging | Command too aggressive | Lower `escape.vx_forward` / `vx_backward`, shorten `max_duration_s` |
| Robot twitches twice per throw | Ball bounces back into view | Raise `fsm.refractory_s` |

Logs: `/tmp/sar-dodge/robot.log`. Every dodge (real or dry) is in `data/events/`.

## What has and has not been verified

- **Verified off the robot:** the lens maths, detection on rendered images,
  tracking, hit prediction, escape choice, the state machine, the recording
  format and replay, the node's message handling (against stand-in ROS
  libraries) and the optional fast mux's priority logic. In simulation with the
  default numbers, lobs from the sides and rear are dodged essentially every
  time, front diagonals about 9 in 10, straight from the front about 6 in 10;
  throws that miss and balls on the floor are ignored.
- **Not verified – needs the stages above:** anything involving the real
  cameras (USB capture modes, exposure control, colours, timestamps), the real
  ROS stack (the launch file has not been run), CPU load on the Pi, and how the
  real robot accelerates. Expect to adjust numbers in `config/` during stages
  4–7. The launch file and the USB capture are the two pieces most likely to
  need a code fix on first contact with the robot; `/tmp/sar-dodge/robot.log`
  will say which.
