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

**Space: this guide assumes 2 m of clear floor around the robot.** Every
calibration distance and every throw below fits in that.

**The throw**, wherever a stage says to throw a ball:

- Stand **2 m** from the robot (tape a mark). Not closer: from 1.5 m the
  simulated dodge rate falls off sharply, worst from behind.
- Underhand lob, released at waist height (about 1 m), **peaking at about
  shoulder height** (1.4–1.7 m above the floor). Not above your head: the ball
  then climbs out of the top of the rear cameras' view.
- Straight from behind, peak a little lower (chest height); straight from the
  front, a little higher is fine.
- One ball in the air at a time, 3 s apart. No flat or hard throws: from 2 m
  they arrive in under half a second, which is not enough.

Simulated with the shipped numbers, 40 of these throws per direction (a
simulation, not a measurement on the robot):

| Throw comes from | Dodged |
|---|---|
| Rear diagonals | 100 % |
| Front diagonals | 100 % |
| Sides | 95 % |
| Straight ahead | about 80 % |
| Straight behind | about 75 % (about 90 % with the lower, chest-height peak) |

---

## 1. Get the code onto the Pupper

The code lives in `https://github.com/yuqinvictoriama/sar-pupper` on branch `main`.

First time, on the Pupper:

```bash
git clone https://github.com/yuqinvictoriama/sar-pupper.git ~/sar-pupper
cd ~/sar-pupper/projectile-dodging
which v4l2-ctl || sudo apt install -y v4l-utils     # needed: sets and tunes the cameras' exposure, gain and white balance
```

To bring a newer version over later, on the Mac:

```bash
cd ~/Desktop/CS123/pi/sar-pupper
git add projectile-dodging && git commit -m "Projectile dodging: <what changed>" && git push
```

then on the Pupper:

```bash
cd ~/sar-pupper && git pull && cd projectile-dodging
```

What the calibration tools write on the Pupper (`config/calib/*.yaml`) is not
touched by a pull. Everything else the code needs (numpy, OpenCV, PyYAML,
SciPy, ROS 2 Jazzy) is already on the course image. It also expects
`~/gemini_pupper` and `~/pupperv3-monorepo` where the labs put them.

## 2. Check the code runs (no robot needed)

```bash
python3 -m unittest discover -s tests     # expect: Ran 52 tests ... OK
python3 tools/sim_report.py --n 20 --dist 1.8 2.0 --rise 0.4 0.7    # simulated 2 m throws; takes about a minute on the Pi
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

## 4. Find the rear cameras, set their exposure, look through each

Four steps. Each tool says what is wrong and what to do when something is.

**4a. Are both cameras there and working?**

```bash
python3 tools/find_cameras.py
```

It lists each USB camera with its socket, takes a snapshot of each
(`data/snapshots/usb*.jpg`), and for a camera that will not deliver pictures it
prints why. What it can report, and the fix:

| It says | Meaning | Fix |
|---|---|---|
| `1 USB camera present` | The Pi does not see the other camera at all | Reseat its plug at both ends. Swap the two cameras' sockets: if the fault follows the camera it is the camera or its cable, if it stays with the socket it is power or the socket. `lsusb` shows what the Pi sees |
| `NOT WORKING: ... in use by another program: pid ...` | Something else has the camera | Stop that program. `bash robot/stop.sh` stops the dodge node; a `view.py` or `record.py` left running in another terminal counts too |
| `NOT WORKING: ... refuses to deliver video` | The Pi sees it but it will not stream | Unplug and replug it. Then the USB power row below; the tool also prints any recent kernel messages about USB, which usually name the cause |
| `The Pi is limiting all four USB sockets together to 0.6 A` | On the robot the Pi is not on a 5 A USB-C supply, so it caps USB power; two cameras can exceed the cap and one drops out | `echo usb_max_current_enable=1 \| sudo tee -a /boot/firmware/config.txt`, then `sudo reboot`. Check with `vcgencmd get_config usb_max_current_enable` (want `=1`) |
| `<-- no camera there now` next to `elp_left` or `elp_right` | The config points at a socket with nothing in it (a plug was moved, or that camera is the missing one) | Fix the camera first, then 4b |

**4b. Which is left, which is right?**

```bash
python3 tools/find_cameras.py --assign    # cover the LEFT rear camera when asked
```

"Left" is the robot's left. It needs exactly two working cameras and refuses
otherwise. The result goes to `config/calib/devices.yaml`. Cameras are
identified by USB socket, so run this again whenever a plug moves.

It finishes by running both cameras **at the same time**, which is what the
dodge node does, and prints the frame rate of each (`python3 tools/find_cameras.py --both`
repeats just this). If each camera works alone but one fails here, they are
starving each other: put them on different USB controllers (move one plug
until the list in 4a shows one camera on `xhci-hcd.0` and the other on
`xhci-hcd.1`, then `--assign` again), check the USB power note above, or lower
the mode for both by creating `config/calib/mode.yaml`:

```yaml
cameras:
  elp_left:  {capture: {fps: 60}}
  elp_right: {capture: {fps: 60}}
```

The config asks for 1280×720 at 120 fps; a camera that tops out lower in that
mode simply runs at its own top rate (90 fps on these ELPs), which is fine.

**4c. Set exposure, gain and white balance for the room**

```bash
python3 tools/tune_exposure.py all
```

Do this in the room and light you will throw in, robot where it will stand.
The rear cameras run a short, fixed exposure (3 ms) so a thrown ball does not
smear; at the cameras' default gain that is nearly black indoors. The tool
raises the gain until the picture is bright enough, lengthens the exposure
only if the gain runs out, then fixes the white balance: it tries every colour
temperature the camera offers and keeps the one that leaves the picture with
no yellow or blue cast. Everything goes to `config/calib/exposure.yaml`. Read
what it prints:

| It says | What to do |
|---|---|
| `exposure 3.0 ms, gain N ... picture averages 80-100 of 255` | Nothing. This is the good case |
| `the exposure is longer than the 3 ms asked for` | It works, but fast balls smear. More light in the room, then run it again |
| `STILL TOO DARK` | The room is too dim for these cameras. Add light (best), or allow more smear with the command it prints |
| `the room lights flicker with the mains` | The ball's brightness will pulse. Daylight or a different lamp is better; colour calibration copes if you leave it |
| `colour cast: ... before, none to speak of now` | Nothing. `slight` is fine too |
| `colour cast: ... clear` or `strong ... now` | The room is strongly coloured, so the picture's average is a poor guide. Run it with `--white-card` and hold a sheet of white paper in the middle of the picture when asked |
| `white balance is left on automatic` | The lamp is warmer or cooler than the camera's fixed settings reach. It works; colours can drift a little |

Two things back this up, so a white balance that is not quite right does not
break detection:

- Every program that opens a rear camera prints what the camera is really set
  to, for example `camera settings: exposure 3.0 ms, gain 15, white balance
  fixed at 2900 K`. If that line ends in `THE CAMERA DID NOT TAKE THE
  SETTINGS`, run `tune_exposure.py` for that camera again; it checks that the
  white balance survives reopening the camera and switches to automatic if not.
- The detector takes the remaining colour cast out of each picture itself
  before looking for the ball (it makes the bright, colourless parts of the
  room neutral). `python3 tools/view.py <camera> --balanced` shows the picture
  as the detector sees it. If `view.py` prints `strong colour cast`, detection
  still works but with less margin: fix the white balance here in 4c.

If a picture still looks tinted to you afterwards, set the temperature by hand
and look again: `python3 tools/tune_exposure.py elp_left --wb 3000` (lower
numbers make the picture bluer, higher ones yellower).

Run it again whenever the lighting changes a lot, and redo the colour
calibration (stage 5) afterwards.

**4d. Look through each camera**

```bash
python3 tools/fit_image_circle.py fisheye --write   # once per robot: finds the rim of the fisheye lens
python3 tools/view.py elp_left            # open http://<pupper-ip>:8090  (Ctrl+C when done)
python3 tools/view.py elp_right
python3 tools/view.py fisheye
```

The fisheye lights a disc in the middle of its frame, and the very edge of
that disc splits colours into a fringe that is partly ball-green. The detector
ignores everything outside the thin red circle `view.py` draws on the fisheye
picture. `fit_image_circle.py` measures where the disc is on your camera and
saves a preview (`data/snapshots/fisheye_circle.jpg`): the yellow circle should
sit on the edge of the lit disc. Point the robot at a normally lit room for it.

Check in each view:

| Check | If not |
|---|---|
| Picture is clearly visible, not near-black | `view.py` prints `picture very dark` when it is: go back to 4c |
| Picture is the right way up | Camera is mounted rotated: put `roll_deg: 180` (or 90 / -90) under that camera's `extrinsics` in a new file `config/calib/mounting.yaml` (same layout as `dodge.yaml`) |
| The ball looks clearly neon green, not washed out to white | ELPs: `python3 tools/tune_exposure.py <camera> --target 70`. Fisheye: exposure is automatic unless you start with `bash robot/start.sh --fisheye-exposure 4000` (microseconds) |
| The ball is not smeared when you wave it | More light, then 4c again |
| Fisheye: no red boxes along the edge of the lit disc | `fit_image_circle.py fisheye --write` again; if the red circle is right and boxes remain just inside it, add `--margin 0.14` |

Before stage 5 the cameras use generic colour settings. Hold the ball about
1 m from each camera: it should already get a red box. Its distance may read
wrong and a box may appear on something else yellow-green; stage 5 fits the
settings to this room and fixes both. If the ball gets no box at all,
`python3 tools/view.py <camera> --mask` shows what counts as ball-coloured
(white): a picture that is mostly white or has no white on the ball means the
exposure or white balance is off, so go back to 4c.

## 5. Calibrate (each camera, in the room and lighting you will run in)

Three things in a fixed order for each camera, because each depends on the one
before: exposure (4c), then colour, then size. If the lighting changes, redo
all three. A colour calibration made with an earlier version of the code is
ignored (the tools say so when they start): redo the colour and size steps for
all three cameras after updating.

```bash
python3 tools/calibrate_color.py elp_left                          # 1. colour: follow the two prompts
python3 tools/calibrate_color.py elp_left --distance 0.5 1 1.5 2   # 2. size: ball held at 0.5, 1, 1.5, 2 m from that lens
```

Repeat both for `elp_right` and `fisheye`, with the same distances.

- **Colour step.** First it looks at the room with no ball in view, then you
  move the ball around 0.5–1.5 m from that camera for ten seconds. Move it low
  and high: a thrown ball arrives from above the camera, and its underside is
  darker than a ball held in the light. The tool then sets the colour limits
  as loose as it can while the empty room still shows nothing ball-coloured,
  and prints how it went (table below).
- **Size step.** Hold the ball still at each distance. Distances are from the
  lens of the camera being calibrated to the centre of the ball, with the ball
  roughly in the middle of that camera's picture (for a rear camera that is
  behind the robot and out to its side, not straight back). Any two or more
  distances work if the farthest is at least 1.5 times the nearest.

What the colour step prints, and what to do:

| It says | What to do |
|---|---|
| `the ball is found in 9x-100% of the pictures ... the empty room shows nothing ball-coloured` | Nothing. This is the good case |
| `the ball can be about N% dimmer ... and still be found` | The margin for a ball in poorer light than your hand-held one. 40 % or more is comfortable. Under 30 % it adds a note: throws will be picked up late or missed when the ball's underside faces the camera. More light in the room raises it (then 4c and this step again) |
| `WARNING: something in view is the ball's colour` | Something yellow-green is in that camera's view. `view.py <camera> --mask` shows it in white. Take it out of the room and repeat the step |
| `WARNING: the ball is lost in some of its own sample pictures` | The light is too poor to tell the ball from the noise in the picture. Add light, then 4c and this step again |
| `Only saw the ball clearly in N frames` | Keep the ball in view and moving for the whole ten seconds, your hand behind it, nothing else yellow-green moving |

The detector also ignores anything too small to be the ball within 3 m
(`detector.max_range_m` in `config/dodge.yaml`), which removes most stray
specks. If you ever throw from further away, raise that number.

If the size step says the ball was not found:

| It says | What to do |
|---|---|
| `has no colour calibration yet` | Run the colour step for that camera first |
| `picture ... is too dark to work with` | `python3 tools/tune_exposure.py <camera>` (4c), then colour, then size |
| `nothing in the picture has the ball's calibrated colour` | The light or exposure changed since the colour step, or the ball is in shadow. Look at the saved `data/snapshots/<camera>_size_problem.jpg` (picture on the left, what counted as ball on the right), then redo the colour step |
| `none of them the size of a ball N m away` | Something else in view is ball-coloured, or the distance is off. Remove it, or check the tape measure |
| `cut off by the edge of the picture` | Move the ball toward the middle of that camera's picture |

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

Timing. This refines each camera's timestamps by a few hundredths of a second;
if it will not work, skip it and move on.

```bash
python3 tools/measure_latency.py elp_left --write     # then rap on the head 5 times, 2-3 s apart
python3 tools/measure_latency.py elp_right --write
python3 tools/measure_latency.py fisheye --write
```

It compares the moment the IMU feels a knock with the moment the picture
jolts, so the camera itself has to shake: rap with a knuckle on the head,
right beside the cameras, not on the body or legs. Have the robot standing or
on its stand (lying on the floor it cannot move), with the camera looking at
something lit and detailed. If the knocks reach the IMU but not the picture,
the tool prints how far the picture moved and how far it needed to.

Optional, for better accuracy later: `tools/calibrate_camera.py` (lens, with a
printed checkerboard) and `tools/calibrate_extrinsics.py` (exact camera angles).
Each explains itself with `--help`. The defaults from the CAD are good enough to start.

## 6. Record throws and replay them (still nothing moves)

```bash
python3 tools/record.py --seconds 30 --name rear-lobs-01     # someone lobs balls at the robot from 2 m meanwhile
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
python3 tools/sim_report.py --dist 1.8 2.0 --rise 0.4 0.7 --lag <fitted lag> --delay <measured delay>
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

Use the throw described at the top (2 m, peaking at shoulder height). Start
with the easy directions and work toward the hard ones:

1. the **rear diagonals** and the **sides**;
2. the front diagonals;
3. straight from the **front** (a slightly higher lob helps here);
4. straight from **behind**, the weakest direction at 2 m (peak at chest height).

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
| `No frames from fisheye` (the rear cameras work) | The fisheye comes through the robot stack, unlike the rear cameras. The stack is not running, or this terminal has not had `source robot/env.sh` | `source robot/env.sh`, then `python3 tools/check_setup.py`. If it says the stack is not running: `bash robot/start.sh --no-dodge` (from `~/sar-pupper/projectile-dodging`), wait a minute, try again |
| `camera settings: ... THE CAMERA DID NOT TAKE THE SETTINGS` | The camera ignored or dropped a setting | `python3 tools/tune_exposure.py <camera>` again; unplug and replug the camera if it repeats |
| A camera's picture is tinted and `view.py` says `strong colour cast` | White balance does not match the room's light (a lamp was switched, or it was never tuned here) | `python3 tools/tune_exposure.py <camera>`, then the colour step again |
| "CANNOT DODGE: no walking controller active" | Robot is relaxed, mid-trick or fell | `bash robot/walk.sh`, or wake it through Gemini |
| A camera shows `---` in `watch.sh` | Unplugged, wrong socket, no power, or another program has it | `bash robot/stop.sh`, then `python3 tools/find_cameras.py` (stage 4a); `/tmp/sar-dodge/robot.log` has the node's own explanation |
| `cannot open the camera: there is no camera on that USB socket` | That camera is not connected as far as the Pi can tell, or its plug moved | Stage 4a, then 4b |
| An ELP picture is near-black | Short exposure at the camera's default gain | `python3 tools/tune_exposure.py <camera>` (stage 4c) |
| An ELP picture is very yellow (or very blue) | White balance fixed at a colour temperature that does not match the room's light | `python3 tools/tune_exposure.py <camera>` (stage 4c), then the colour and size steps of stage 5 again |
| One ELP works alone but not with the other | Shared USB bandwidth or power | Stage 4b: different controllers, `usb_max_current_enable=1`, or `fps: 60` |
| Red boxes around the edge of the fisheye picture | Colour fringe at the lens rim | `python3 tools/fit_image_circle.py fisheye --write` (stage 4d) |
| Range to a ball on the floor reads far too short and its box flickers | Colour range too wide: the blob is the ball plus the floor it lights up | `calibrate_color.py` for that camera (stage 5) |
| No track although the ball is in view | Colour settings do not match this lighting, or the ball is in poor light | `view.py <camera> --mask`: the ball should be white. If not, 4c then the colour step again; add light if the colour step reports little margin |
| Tracks appear with no ball | Something else in view is ball-coloured | Remove it and redo the colour step for that camera |
| Range reads consistently wrong | Size calibration | `calibrate_color.py <cam> --distance ...`; check `ball.diameter_m` |
| Dodges balls that would miss by a lot | Prediction too generous | Lower `threat.k_sigma` or `threat.sigma_cap_m` |
| Reacts too late from the front | Too little time in the air, or the fisheye is slow | Lob a little higher; `bash robot/start.sh --fisheye-fps 60` |
| Misses throws from straight behind | The ball climbs out of the top of the rear cameras' view | Lower arc: peak at chest height. Do not throw from closer than 2 m |
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
  default numbers and the 2 m throw, the dodge rates are those in the table at
  the top; throws that miss and balls on the floor are ignored. (From 3–4 m, the
  distance the plan was first written for: sides and rear essentially every
  time, front diagonals about 9 in 10, straight from the front about 6 in 10.) The camera finder, the
  exposure tuner and the lens-rim fit have only run against stand-in cameras
  and one screenshot of the real fisheye.
- **Seen on the robot (2026-11-23, stages 1-4):** the code installs and the
  stack starts with `start.sh --no-dodge`; the fisheye arrives through ROS and
  the ball is found in it; one ELP opens at 1280×720, 90 fps, with its JPEG
  passed through. That run also showed three faults, fixed since: the ELP
  picture was near-black at the default gain (now stage 4c), the lens rim of
  the fisheye produced false blobs (now masked, stage 4d), and Ctrl+C aborted
  the tools instead of ending them. The second ELP could not be opened in that
  run; the log could not say why, which is what stage 4a now reports. Later the
  same day both ELPs ran, and their first well-exposed pictures showed the ball
  was not being picked out: the shape test rejected it for its noisy edge, and
  the colour test let the dim floor through. Both were reworked (see `plan.md`).
  The next pictures were strongly yellow, with white walls passing for ball
  colour on one camera and the ball's hue shifted out of its calibrated range
  on the other; the detector now removes the colour cast itself. All of this is
  checked against the six pictures taken on the robot so far; none of it has
  run live on the robot yet.
- **Not verified - needs the stages above:** the second ELP and both ELPs
  streaming together, the exposure tuner on the real cameras, colours and
  timestamps, the dodge node in the real ROS stack, CPU load on the Pi, and how
  the real robot accelerates. Expect to adjust numbers in `config/` during
  stages 4-7. `/tmp/sar-dodge/robot.log` says what the node could not do.
