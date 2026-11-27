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

**Space: this guide assumes a room of about 4 × 6 m.** Stand Pupper in the
middle, **facing along the long side**. That gives:

| Throw comes from | Thrower stands | Why |
|---|---|---|
| Front or back | 2.5–3 m away (the long way) | Pupper can only sidestep away from these, at 0.5 m/s, so it needs the longer flight |
| Either side | 2 m away (the short way) | Pupper runs forward or back away from these, at 1.0 m/s |
| Diagonals | about 2.5 m away | |

Every calibration distance below fits in 2 m.

**The throw**, wherever a stage says to throw a ball:

- Underhand lob, released at about waist height. One ball in the air at a
  time, 3 s apart.
- **From the front and the back, throw high.** A lob that peaks above head
  height (the ball climbs a metre or more above your hand) is in the air for
  1.0–1.2 s; one that peaks at shoulder height for about 0.7 s. Pupper can only
  sidestep away from these, and needs the time.
- **From the sides, shoulder to head height is enough**, and better than very
  high: from 2 m a very high lob climbs out of the cameras' view within a tenth
  of a second, sometimes before it has been seen often enough to track.
- A lob leaves the top of the rear cameras' picture on the way up. Pupper keeps
  following it on its arc while it is out of sight.
- No flat or hard throws: from these distances they arrive in under half a
  second, which is not enough.

Simulated with the shipped numbers, 40 throws per row and direction (a
simulation, not a measurement on the robot):

| Throw comes from | Shoulder-height lob | High lob |
|---|---|---|
| Sides, 2 m | about 95 % | about 80 % |
| Behind, 2.5–3.5 m | 100 % | about 99 % |
| Rear diagonals, 2–2.5 m | 100 % | about 99 % |
| Front diagonals, 2–2.5 m | about 98 % | 100 % |
| Straight ahead, 2.5–3.5 m | about 80 % | about 98 % |

These assume the robot gets up to speed as fast as the config says. With a
robot a third slower, the straight-ahead shoulder-height lob drops to about
40 % and the high one to about 85 %.

On a smooth floor (hardwood, tile) the robot's feet slip when it sets off
fast. If it trips while dodging, put it on a rug, a yoga mat or foam tiles
before lowering speeds: grip is worth more than any setting. The diagonal move
(forward and sideways at once) is already softened for this
(`escape.diagonal_scale`, stage 9).

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
python3 -m unittest discover -s tests     # expect: Ran 70 tests ... OK
python3 tools/sim_report.py --n 20 --dist 1.8 2.0 --rise 0.4 0.7 --elp-fps 30    # simulated 2 m throws, rear cameras at their 30 a second; about a minute on the Pi
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
python3 tools/record.py --seconds 30 --name rear-lobs-01     # someone lobs balls at the robot meanwhile, from the places described at the top
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
for each one it would have dodged. Every such moment, and every throw it saw
but would not have dodged, is saved under `data/throws/` and shown in the
**Pupper Dodging View** (the section after stage 8): open it now, in a browser, at the
address `start.sh` printed. In the `watch.sh` line, "lag" is how old a frame is
when it has been processed and "cpu" is the processing time per frame. A healthy
line shows the ELPs at about 30 fps (their `max_process_fps`), "cpu" under 20 ms
and "lag" under 100 ms. If the frame rates are far below that, lag climbs, or
`top` shows the Pi near 100 % on all cores, the Pi is overloaded and no dodge
will be in time: lower `max_process_fps` for the ELPs, or stop whatever else is
using the processor.

Now the first step that **moves the robot**. Stand it up:

```bash
bash robot/walk.sh                  # stands up in gemini_walk (or press X on the joystick: slower controller)
```

and check how it takes the kind of burst a dodge sends, gently first:

```bash
python3 tools/step_response.py --vy 0.5
python3 tools/step_response.py --vx 0.6
python3 tools/step_response.py --vx 1.0
python3 tools/step_response.py --vx 1.0 --vy 0.5      # the kind of move a dodge makes
python3 tools/step_response.py --vx -0.8
```

Each run asks you to type `go`, moves the robot for half a second, and prints
two measurements:

- **delay before the body reacts**, read from the IMU's turn-rate sensor. The
  config assumes 0.12 s (`escape.react_s`); the tool says so if yours is later.
- **peak body tilt.** Stop raising the speed if it passes about 35° or the
  robot stumbles.

It does **not** measure how far the robot got. Pupper's IMU sends orientation
and turn rate but no acceleration, so distance cannot be read from it, and the
tool says "not measured" rather than guess. The config's lag of 0.20 s
(`escape.accel_lag_s`, how quickly the robot gets up to speed) stays in use,
and stage 8 is the real test of it.

**Optional: measure the lag with a phone.** Put two tape marks on the floor,
0.15 m and 0.30 m ahead of the leading foot in the direction of the burst. Film
a `--vx 1.0 --vy 0.5` burst from the side in slow motion. In the video, count
the time from the robot's first movement until that foot reaches each mark
(frames divided by the frame rate, e.g. 74 frames at 240 fps = 0.31 s). Then:

```bash
python3 tools/step_response.py --vx 1.0 --vy 0.5 --timed 0.15:0.31 0.30:0.47 --write
python3 tools/sim_report.py --dist 1.8 2.0 --rise 0.4 0.7 --elp-fps 30 --lag <fitted lag>
```

The first line moves nothing: it fits the lag to your two timings and saves it.
The second shows the dodge rates to expect with that lag (the table at the top
of this guide assumed 0.20 s).

| Message | Meaning | What to do |
|---|---|---|
| `delay before the body reacts: could not tell` | The body never turned noticeably | If the robot visibly moved, ignore it. If it stood still, it is not in a walking controller: `bash robot/walk.sh`, then `check_setup.py` |
| `The body reacts later than the config assumes` | Reaction slower than 0.12 s | Put the suggested value under `escape:` as `react_s` in a `config/calib/` file |
| `These timings do not fit` | The video timings are impossible for that speed | Recount; time from the first movement, not from pressing Enter |
| `note: the response lag of 0.79 s ... is being ignored` | A file saved by an earlier version of this tool | `rm config/calib/step_response.yaml` |

If the robot was steady at 1.0 m/s and you want more margin, try
`--vx 1.5` / `--vx -1.5`, and if those are steady too, raise `vx_forward` and
`vx_backward` under `escape:` (in a `config/calib/` file or `dodge.yaml`).

## 8. Arm it

With the robot standing, the floor clear and someone at the joystick:

```bash
bash robot/watch.sh arm             # from now on a predicted hit moves the robot
bash robot/watch.sh disarm          # back to dry run
```

Use the throw and the positions described at the top. Start with the easy
directions and work toward the hard ones:

1. the **sides** (from 2 m) and the **rear diagonals**;
2. the front diagonals and straight from **behind** (from 2.5–3 m);
3. straight from the **front** (from 3 m, as high a lob as the ceiling allows):
   the hardest, because Pupper can only sidestep.

After each throw, mark in Pupper Dodging View whether it was really dodged or hit.

To start the stack already armed: `bash robot/start.sh --armed`. With Gemini
Pupper as well: `bash robot/start.sh --gemini` (add `--kiosk` for the on-robot
app, `--armed` to arm). Always stop with `bash robot/stop.sh`.

After a dodge the node holds the robot still until it is steady, then goes
quiet; the mux hands control back to Gemini or navigation half a second later.
Each dodge is announced on `/dodge/event` (JSON) so a planner can re-plan.

## Pupper Dodging View: every throw, recorded

The dodge node saves a few seconds around three kinds of moment, each in its
own folder under `data/throws/`:

| Folder name ends in | What it is |
|---|---|
| `-dodge` | A dodge it made (armed) |
| `-dry` | A dodge it would have made (dry run) |
| `-seen` | A ball it tracked in flight but did not dodge, with its reasons |
| `-mark` | Seconds you asked it to keep (the two buttons below) |

In each folder: the pictures from every camera exactly as they arrived
(`<camera>.bin` / `.idx`), the IMU (`imu.csv`), a frame-by-frame trace of what
the node detected and did (`trace.jsonl`), and, added by the page, the timings
(`throw.json`). `data/throws/throws.csv` has one line per throw, for a
spreadsheet.

**Video files** (`videos/<camera>.mp4`, plain, and
`videos/<camera>_detections.mp4` with what Pupper detected drawn on) are made
when you ask: the **Make video files** button in a throw's panel, or all at
once with `python3 tools/dodging_view.py --once` after a session. They are not
made automatically, because encoding video takes processor time the dodge
needs. The page does not need them: it plays the saved pictures directly.

**Open it.** `robot/start.sh` starts Pupper Dodging View and prints its address:

```
http://pupper6.local:8077
```

Open that in a browser on your laptop or phone (same Wi-Fi as the Pupper). If
the name does not resolve, use the numeric address in `/tmp/sar-dodge/dodging_view.log`,
or tunnel it and open `http://localhost:8077` on the Mac:

```bash
ssh -L 8077:localhost:8077 pi@pupper6        # on the Mac; leave it open
```

To run it by hand: `bash robot/dodging_view.sh`. It cannot move or arm the robot.

**Finding your way around.**

- **Top bar:** what the robot is doing now (armed or dry run, and each camera's
  pictures per second and how old they are), and the two buttons for keeping a
  throw.
- **Left: the list of throws**, newest first, always in view. Each line says
  what Pupper did, when, where the ball came from (the green dot on the small
  dial; the robot's nose points up) and how many milliseconds were left. The
  buttons above the list show one kind only. Click a throw, or walk the list
  with the up and down arrow keys.
- **Right: the chosen throw.** One sentence on what happened, then the detail:
  why it waited if it did, and whether the ball went out of sight.
- **The strip under that is the throw in time.** The top lane is the ball
  (pale green while tracked, a dark mark for every picture it was found in,
  amber where it was out of sight). The bottom lane is Pupper: making up its
  mind, the command on its way, moving. Red is the instant it decided. **Drag
  along the strip to move through the video**; the cameras under it follow.
- **The cameras** play in step, at quarter speed unless you change it, with a
  green ring on the ball it was tracking and yellow boxes on anything else it
  took for ball-coloured. Untick "Show what Pupper detected" for the plain
  picture. Space plays and pauses; the left and right arrow keys step one
  picture.
- **Below:** the timings in the table that follows, the planned move drawn from
  above, and how each camera was doing during the throw.

| Number | Meaning |
|---|---|
| Ball first in a picture, to the decision | How long Pupper took to make up its mind, from the first picture with the ball in it |
| Decision, to the command being sent | Should be a few ms |
| Command, to the mux passing it on | The velocity mux's delay. If it is tens of ms, try `bash robot/start.sh --fast-mux` |
| Command, to the body starting to move | **The robot's own latency**: from the command to the IMU showing the body turn |
| Decision, to the body starting to move | **From recognising the ball to starting to move** |
| Ball first seen, to the body moving | The whole reaction |
| Time left at the decision | How long until the ball was due when it decided |
| Time the move needs | Reaction plus getting clear, as the planner estimates it |

The robot-latency numbers exist only for armed dodges; a dry run sends no
command.

**Throws it does not react to at all** are not saved on their own, because
the node did not notice anything. Two buttons at the top right cover them:

- **Record the next throw**: press it, then throw within 3 seconds. Those
  seconds are saved whatever Pupper does.
- **Keep the last 3 s**: press it straight after a throw (a phone in your
  hand works).

Then play the video with detections on: either the ball was never ringed
(a detection problem: lighting, colour calibration, stage 5) or it was
ringed but too briefly or too late (look at where it first appears).

**Say what really happened.** Pupper cannot tell whether the ball hit it. In
each throw's panel press *dodged*, *hit*, *off target* or *not a throw*, and
add a note. The summary above the list and `throws.csv` use these.

**Disk space.** A throw takes roughly 30–60 MB, so a session of 30–50 throws
needs about 2 GB. `df -h ~` on the Pupper shows what is free; so does the page,
under the list, with how much the throws use; the node stops saving when less than 1.5 GB
is free. Copy what you want to keep to the Mac, then delete on the Pupper:

```bash
scp -r pi@pupper6:~/sar-pupper/projectile-dodging/data/throws ~/Desktop/CS123/throws    # on the Mac
rm -r data/throws/20261126-*                                                           # on the Pupper: one day's throws
```

Each throw's panel also has a link that downloads its numbers (and videos, once
made) as one .zip, without the raw pictures.

**Load on the Pi.** The page is drawn by your browser; on the Pupper it is a
small server that hands over saved files, at low priority. Idle, it should sit
near 0 % in `top` (look for `dodging_view.py`). Two things are heavy: making
video files (so do it between throws), and, once per recording, re-analysing
recordings made before the recorder existed (`data/events/`). To run without
it: `bash robot/start.sh --no-view`. To look at throws on the Mac instead, copy
`data/throws` over (the `scp` line above) and run, in this folder on the Mac,
`python3 tools/dodging_view.py --no-ros --dir ~/Desktop/CS123/throws`, then
open `http://localhost:8077`. That needs Python with NumPy, OpenCV and PyYAML,
and has no live status or record buttons.

**If "Seen, no dodge" throws appear with nobody throwing,** something green
moving in the room is being taken for a ball in flight, and each one costs
disk space. Remove the object, or stop saving that kind: put
`node: {record_undodged: false}` in a file such as `config/calib/recording.yaml`
and restart.

To get the numbers and videos without the page:
`python3 tools/dodging_view.py --once` analyses every saved throw, writes the
videos and `throws.csv`, and prints the summary sentences. Run it with the
robot idle.

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
| The colour step reports a hue range that is not green (well outside 30–75) | It sampled something other than the ball, or the picture has a strong cast | `view.py <camera> --mask`: only the ball should be white. If the floor or skin is, redo the colour step with the ball filling the box, after stage 4c |
| One camera's ranges are off and its `size_scale` in `config/calib/<camera>_color.yaml` is far from 1 (say above 1.2) | The size step measured at the wrong distances, or on a cut-off ball | Redo `calibrate_color.py <camera> --distance 0.5 1 1.5 2` with a tape measure |
| Tracks appear with no ball | Something else in view is ball-coloured | Remove it and redo the colour step for that camera |
| Range reads consistently wrong | Size calibration | `calibrate_color.py <cam> --distance ...`; check `ball.diameter_m` |
| Dodges balls that would miss by a lot | Prediction too generous | Lower `threat.k_sigma` or `threat.sigma_cap_m` |
| Reacts too late | Open the throw in Pupper Dodging View: it says where the time went | Few pictures per second or old pictures (age over 100 ms) in "Cameras during this throw": the Pi is overloaded, see stage 7. Ball first seen close to the robot: lighting or colour calibration (stage 5). A long time from command to body moving: `--fast-mux`. Otherwise more time in the air: throw higher, or from further back |
| Loses a high lob that left the picture | It is followed unseen for 0.6 s; a very high lob is out of sight longer | Raise `tracker.flight_coast_s` (0.8), or throw a little lower |
| Reacts in time but does not get clear | Robot slower than the config assumes | Raise `escape.accel_lag_s` in steps of 0.05 (or measure it with the stage 7 video), set `react_s` from stage 7; raise `vx_*` if steady |
| Moves the wrong way | Left/right swapped or a camera is rotated | Stage 4 and the bearing table in stage 5 |
| Robot stumbles or trips when dodging | Feet slipping on a smooth floor, or the command is too aggressive | A rug or mat first. Then lower `escape.diagonal_scale` (0.8 → 0.65) if it is the diagonal move, `escape.vx_forward` / `vx_backward` if it is the straight run |
| Robot twitches twice per throw | Ball bounces back into view | Raise `fsm.refractory_s` |

Logs: `/tmp/sar-dodge/robot.log` (the page's: `dodging_view.log`). Every throw is in `data/throws/` and in Pupper Dodging View.

## What has and has not been verified

- **Verified off the robot:** the lens maths, detection on rendered images,
  tracking, hit prediction, escape choice, the state machine, the recording
  format and replay, the node's message handling (against stand-in ROS
  libraries) and the optional fast mux's priority logic. In simulation with the
  default numbers and the throws described at the top, the dodge rates are those in the table at
  the top; a ball on the floor is ignored, and of throws that pass 0.7–1.5 m to
  the side about 1 in 20 is dodged anyway (none were before lobs were followed
  out of sight; that is the price of it). The camera finder, the
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
- **Small room and late reactions (2026-11-26).** In simulation, high lobs
  thrown from 2 m were almost never dodged from the sides or the rear
  diagonals: the ball left the top of a rear camera's picture and was dropped
  after a quarter of a second unseen. Two changes: a ball seen in flight is
  followed on its arc for 0.6 s while unseen, and a lob is judged while it is
  still climbing instead of only once it is closing in. Simulated high lobs
  dodged, of 40: from the sides 4 before, 31 after; rear diagonals 10, then 40;
  behind 25, then 40; and they are decided about 0.3 s sooner. (The lag
  measured on the robot the same day had a different cause, the Pi being
  overloaded, dealt with separately: see `plan.md`.) The diagonal move is 20 % slower, for smooth floors. The
  throw recorder and Pupper Dodging View have run against a stand-in robot (stand-in
  ROS, rendered cameras, a body that reacts 110 ms after a command), not on
  the Pupper; the timings they report for that stand-in match what was put in.
- **Not verified - needs the stages above:** the second ELP and both ELPs
  streaming together, the exposure tuner on the real cameras, colours and
  timestamps, the dodge node in the real ROS stack, CPU load on the Pi, and how
  the real robot accelerates. Expect to adjust numbers in `config/` during
  stages 4-7. `/tmp/sar-dodge/robot.log` says what the node could not do.
