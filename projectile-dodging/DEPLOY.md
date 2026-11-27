# Projectile dodging on the Pupper: what to do next, and how to run it

This guide has three parts:

1. **[What to do next](#what-to-do-next)**: the list to work through now, steps A to G.
2. **[Running it](#running-it-every-session)**: the short version for every session after that.
3. **[Reference](#reference)**: the detail behind each step, Pupper Dodging View, and what to do when something is off.

All commands run **on the Pupper** (`ssh pi@192.168.1.73`), from
`~/sar-pupper/projectile-dodging`, unless a line says "on the Mac". In every new
terminal on the Pupper, run `source robot/env.sh` first.

**Safety, throughout.** The joystick e-stop (right-stick press) always works.
Whenever the robot can move (steps E, F and G), have someone holding the
joystick, the robot on open floor with 1.5 m clear around it, and nothing it
could fall off. The dodge node starts in **dry run**: it reports the dodge it
would make and never moves the robot until you arm it in step G.

## Where things stand (26 November 2026)

| | |
|---|---|
| **Done on this Pupper** | Code installed and up to date; all tests passed there on 26 Nov (85 then, 100 now). Both rear cameras found and assigned left and right. Exposure and white balance tuned for evening light. Fisheye lens rim fitted. Camera timing measured (23 Nov). USB power cap lifted. |
| **Fixed on 26 Nov** | The lag and the missed throws. In dry run with nobody throwing, the node went from 5–15 pictures a second and 130–390 ms lag to 20–26 a second and 3–35 ms, with no false "throw seen". |
| **Not done yet** | **No ball has been thrown at the robot since those fixes**, armed or not. The colour calibration is from 23 Nov and was made with a yellow cast in the picture: `elp_left`'s lets dark floor through, `elp_right`'s size scale is off (1.38, should be near 1), and the fisheye's is not green at all, so it is ignored and that camera runs on generic settings. The joystick has not been tried against the new velocity mux on the robot. |
| **Found after the reboot that evening** | After a power-up the rear cameras ignored their white balance (strong yellow cast, rates back down to 10–15 a second). Cause found on the live cameras and fixed in the code; the fix running by itself is first tested at your next boot (step D checks it). The colour step had been learning the hand holding the ball instead of the ball, which is why the fisheye's result was thrown away twice; fixed. |
| **Also fixed that night** | Balls being handled beside the robot, and balls lying still, set off about ten dry-run "dodges" in four minutes. Five recordings were replayed on the Mac: none dodges any more, the recorded real throw still does. Not yet run on the robot. |
| **After the first armed lobs (21:36–21:40)** | Three faults read from the 22 recordings: dodges of still patches (one ended in the fall), real dodges 0.12–0.18 s before the ball, and straight behind weak. Cause of the first two: the ball's speed was read from how fast it grows in the picture, which at 2.5 m is a matter of one pixel. It is now read from how the ball's direction moves. In simulation every direction is as good or better (rear cone 70 → 80 %, front 67 → 77 %), and the false dodges in the recordings are gone. The dodge now eases to a stop instead of stopping dead. **Not yet run on the robot.** |
| **Straight behind** | The rear cameras see only 30° above the horizontal there, so a lob is out of sight 0.1 s after release, and they cannot be tilted. The node now pools both rear cameras' looks, judges a climbing ball on two looks when no third comes, and does not wait on a ball it can no longer see. Simulated straight behind: high lobs from 2–2.5 m 60 → 85 %, shoulder-height 62 → 72 %; all directions together, the rear cone is at 90 %. Not yet run on the robot. |
| **Open** | Both rear cameras share one USB controller and deliver about 43 pictures a second instead of 90 (step B). |

## What to do next

Work through A to G in order. Rough times are guesses. Nothing moves the robot
until step E.

### A. Get the latest code (2 min)

```bash
cd ~/sar-pupper && git pull && cd projectile-dodging
```

```bash
python3 -m unittest discover -s tests
```

- [ ] It ends with `OK` after 100 tests or more. If not, stop and send me the output.

The line `note: the colour calibration in ... fisheye_color.yaml is for hue 9-28 ... is being ignored`
is expected until step C.

### B. Put the rear cameras on different USB controllers (5 min, optional)

Both rear cameras are on `xhci-hcd.0`. Moving one may raise their rate from
about 43 pictures a second; this has not been tried. Skip it if you are short
of time: A to G work without it.

```bash
bash robot/stop.sh
```

```bash
python3 tools/find_cameras.py
```

Move **one** rear camera's plug to another socket on the Pi and run it again,
until the list shows one camera on `xhci-hcd.0` and the other on `xhci-hcd.1`. Then:

```bash
python3 tools/find_cameras.py --assign
```

- [ ] Cover the **robot's left** rear camera when asked. It ends by running both cameras together and printing each one's rate. Write the two rates down for me.

Detail: [stage 4a and 4b](#4-find-the-rear-cameras-set-their-exposure-look-through-each).

### C. Calibrate the three cameras with the ball (20–30 min)

Do this in the room and the light you will throw in, with the robot where it
will stand. **Do it again in the demo room.** The dodge node must not be
running: only one program can read a rear camera at a time, so with the dodge
node up the tools stop and tell you to run the two commands below. (That is
what "back cameras not working" was on 26 Nov: plain `start.sh` had been run.)

```bash
bash robot/stop.sh
```

```bash
bash robot/start.sh --no-dodge
```

```bash
source robot/env.sh
```

**C1. Exposure and white balance** (rear cameras; about 2 min):

```bash
python3 tools/tune_exposure.py all
```

- [ ] Each camera reports `picture averages 80-100 of 255` and a colour cast of `none to speak of` or `slight`. Otherwise see the table in [stage 4c](#4-find-the-rear-cameras-set-their-exposure-look-through-each).

**C2. Colour, then size, for each camera.** Three cameras, two commands each:

```bash
python3 tools/calibrate_color.py elp_left
```

```bash
python3 tools/calibrate_color.py elp_left --distance 0.5 1 1.5 2
```

```bash
python3 tools/calibrate_color.py elp_right
```

```bash
python3 tools/calibrate_color.py elp_right --distance 0.5 1 1.5 2
```

```bash
python3 tools/calibrate_color.py fisheye
```

```bash
python3 tools/calibrate_color.py fisheye --distance 0.5 1 1.5 2
```

- **Colour:** first keep every ball out of view, then move one around 0.5–1.5 m
  from that camera for ten seconds, low and high. Hold it by your fingertips
  from below, so the camera sees ball rather than hand.
- **Size:** hold the ball still at each distance, measured with a tape from
  that camera's lens to the middle of the ball, ball near the middle of that
  camera's picture.
- [ ] Each colour step says the ball is found in 90–100 % of the pictures and `the empty room shows nothing ball-coloured`.
- [ ] Each prints a hue range inside about 28–75. If it says `That is not the ball's green`, it sampled your hand: run it again.
- [ ] No `WARNING` lines. If there is one, the tables in [stage 5](#5-calibrate-each-camera-in-the-room-and-lighting-you-will-run-in) say what to do.

**C3. Look through each camera** (open `http://192.168.1.73:8090`, Ctrl+C when done):

```bash
python3 tools/view.py elp_left
```

- [ ] The ball gets a red box, its range reads within about 10 % of the tape, and nothing else in the room gets a box. Same for `elp_right` and `fisheye`.
- [ ] Ball held behind and to the robot's left reads a bearing near +135°; straight ahead (fisheye) near 0°.

**Then tell me "calibrated".** I will copy the new files in `config/calib/` off
the Pupper, check the limits and commit them, so they are not lost.

### D. Dry run, nobody throwing (5 min)

```bash
bash robot/stop.sh
```

```bash
bash robot/start.sh
```

```bash
python3 tools/check_setup.py
```

```bash
bash robot/watch.sh
```

- [ ] `check_setup.py`: every line says OK (the walking-controller line needs the robot standing; that comes in step E).
- [ ] `watch.sh` shows `dry-run`, each camera at about 20 pictures a second or more and a lag under 50 ms.
- [ ] No `COLOUR CAST` after a camera's name. After a power-up these cameras ignore their white balance until automatic has been switched on and off once; the node now does that when it opens them. If the words are still there after 30 seconds, tell me, and meanwhile go back to C1 and C2 for that camera.
- [ ] No `EVENT` lines while the room is still. If they appear, something in view is being taken for the ball: open Pupper Dodging View (`http://192.168.1.73:8077`) and play the saved throw to see what was ringed.

Paste me a few lines of `watch.sh` if any of these fail.

### E. Stand up, check the joystick wins, throw in dry run (15 min)

**The robot stands up here. Someone holds the joystick from now on.**

```bash
bash robot/walk.sh
```

- [ ] Walk it a little with the joystick. It follows the stick.
- [ ] Press the right stick: it goes limp. Stand it up again with `bash robot/walk.sh`.

Now the check that the joystick outranks a dodge command with the new mux. One
person holds the left stick gently forward, the other runs this and types `go`:

```bash
python3 tools/step_response.py --vy 0.5
```

- [ ] The robot keeps walking forward with the stick and does **not** step sideways. If it steps sideways, stop here, restart with `bash robot/start.sh --slow-mux` and tell me.

**Before throwing, handle a ball beside the robot for half a minute**: take
some out of a basket, pick one up from the floor, wave one about 1.5 m away.

- [ ] No `"dodge"` event appears while you do. On 26 Nov this set off about ten; the causes found in those recordings are fixed, but this is the first check on the robot. If one appears, tell me the time: it is saved under `data/throws/`.

It still helps to keep spare balls behind the thrower rather than in view.

Then, with the robot standing still, throw balls at it, using the throws in
[the room and the throw](#the-room-and-the-throw): sides first, then behind,
then the front. Still in dry run, so it does not move. Leave `bash robot/watch.sh`
running in a terminal, or keep Pupper Dodging View open.

- [ ] Each throw that would have hit prints an `EVENT` with `"dodge"` and `"dry_run": true`, and appears in Pupper Dodging View as "would have dodged".
- [ ] For a throw it ignores completely, press **Keep the last 3 s** in Pupper Dodging View straight afterwards.
- [ ] Throw at least three from each side, three from behind, three from the front, and two that clearly miss.

(With the robot lying relaxed instead of standing, the node reports
`cannot_dodge` for a predicted hit rather than the dodge it would make.)

**Then tell me "thrown".** I will read the saved throws off the Pupper and say
whether it is ready to arm, or what to change first.

### F. Measure how the robot takes a burst (10 min)

**The robot moves.** Each run asks for `go` and moves it for half a second.
Stop raising the speed if it stumbles or tilts past about 35°.

```bash
python3 tools/step_response.py --vy 0.5
```

```bash
python3 tools/step_response.py --vx 0.6
```

```bash
python3 tools/step_response.py --vx 1.0
```

```bash
python3 tools/step_response.py --vx 1.0 --vy 0.5
```

```bash
python3 tools/step_response.py --vx -0.8
```

- [ ] Note the `delay before the body reacts` each prints. The config assumes 0.12 s; the tool says so if yours is later.
- [ ] If the feet slip on the floor, put the robot on a rug or mat before changing any number.

Detail: [stage 7](#7-dry-run-with-the-dodge-node-then-measure-the-robots-response).

### G. Arm it and throw (as long as you like)

Robot standing, floor clear, someone on the joystick:

```bash
bash robot/watch.sh arm
```

Throw in this order: the **sides** from 2 m and the rear diagonals, then
straight from **behind** and the front diagonals from 2.5–3 m, then straight
from the **front** from 3 m with as high a lob as the ceiling allows.

- [ ] After each throw, press *dodged*, *hit* or *off target* for it in Pupper Dodging View. Pupper cannot tell by itself whether the ball hit.

```bash
bash robot/watch.sh disarm
```

```bash
bash robot/stop.sh
```

**Then tell me "thrown, armed".** I will read the throws and timings off the
Pupper and carry on from there.

## Running it, every session

Once A to G have been done in a room, a session is:

| When | Command | What happens |
|---|---|---|
| Start | `bash robot/stop.sh` then `bash robot/start.sh` | Robot stack and dodge node in dry run; about a minute. Prints the address of Pupper Dodging View |
| New terminal | `source robot/env.sh` | Needed before any tool |
| Check | `python3 tools/check_setup.py` | Every line OK |
| Watch | `bash robot/watch.sh` | State, each camera's rate and lag, events |
| Stand up | `bash robot/walk.sh` | **Robot stands.** `bash robot/walk.sh relax` lets it down |
| Arm | `bash robot/watch.sh arm` | **A predicted hit now moves the robot** |
| Disarm | `bash robot/watch.sh disarm` | Back to dry run |
| Stop | `bash robot/stop.sh` | Everything off |
| Look at throws | `http://192.168.1.73:8077` | Pupper Dodging View: every throw, its video and timings |

If the light has changed since the last calibration (daylight to lamps, or a
different room), do step C again first: `bash robot/start.sh --no-dodge`, then
`tune_exposure.py all` and the colour and size steps.

`bash robot/start.sh --armed` starts already armed. `--gemini` also starts
Gemini Pupper's server. For the whole demo with navigation, terrain and voice
together, use `path-navigation/README.md`; this guide is dodging by itself.

## The room and the throw

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
- **Stand at least 2 m away.** From 1–1.5 m a lob is in the air 0.4–0.5 s and
  the robot needs 0.6 s. Keep the basket of spare balls behind you, not in your
  hand in front of the cameras.
- **Straight behind is the weak direction, and the cameras cannot be tilted.**
  The rear cameras overlap there, but only up to 30° above the horizontal
  (38° at 15° to one side, 46° at the rear diagonals), and a lob from 2.5 m is
  above that 0.1 s after release. So the node makes the most of that tenth of
  a second: both rear cameras' looks are put together, a ball they saw only
  twice climbing like a lob is judged on those two looks, and a ball judged to
  be on its way is dodged without waiting for a better look that cannot come.
  Simulated, straight behind (within 15°), 40 throws each, at the measured
  rates:

  | From behind | 2–2.5 m | 2.5–3.5 m |
  |---|---|---|
  | High lob (peaks 0.8–1.2 m above the hand) | 85 % | 92 % |
  | Shoulder-height lob | 72 % | 72 % |
  | Flat, chest-high | – | 0 % |

  **From behind, throw high.** A flat throw from behind arrives in about 0.55 s
  and Pupper can only sidestep from it, at 0.5 m/s, which takes longer. Of the
  shoulder-height ones that fail, most were reacted to but not cleared in time.
- The node now judges a ball by how its direction moves, so it reacts to a ball
  that will sail over the robot as well as one that will hit, and to nothing
  that is not climbing in the picture the way an incoming ball does.

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

# Reference

The detail behind steps A to G, in the order of a first bring-up on a new robot.
Stages 1 to 4 are done on this Pupper; step C above is stage 4c and stage 5 again.

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

What the calibration tools write on the Pupper (`config/calib/*.yaml`) stays as
it is through a pull, unless the same file was also changed in the repo; then
git refuses the pull and names the file. Those files are kept in the repo, so
after calibrating, copy them back and commit them, on the Mac:

```bash
scp 'pi@192.168.1.73:~/sar-pupper/projectile-dodging/config/calib/*.yaml' ~/Desktop/CS123/pi/sar-pupper/projectile-dodging/config/calib/
```
 Everything else the code needs (numpy, OpenCV, PyYAML,
SciPy, ROS 2 Jazzy) is already on the course image. It also expects
`~/gemini_pupper` and `~/pupperv3-monorepo` where the labs put them.

## 2. Check the code runs (no robot needed)

```bash
python3 -m unittest discover -s tests     # expect: Ran 100 tests ... OK (more once other tools' tests are added)
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
| `in use by the dodge node` / `The dodge node is running and has the rear cameras` | Nothing is wrong: only one program can read a camera at a time | `bash robot/stop.sh && bash robot/start.sh --no-dodge` |
| `NOT WORKING: ... in use by another program: pid ...` | Something else has the camera | Stop that program: a `view.py` or `record.py` left running in another terminal, or a browser |
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

Throw balls, with the robot standing (`bash robot/walk.sh`, below): lying
relaxed, the node reports `cannot_dodge` for a predicted hit instead of the
dodge it would make. `watch.sh` prints an `EVENT ... "dodge" ... "dry_run": true` line
for each one it would have dodged. Every such moment, and every throw it saw
but would not have dodged, is saved under `data/throws/` and shown in the
**Pupper Dodging View** (the section after stage 8): open it now, in a browser, at the
address `start.sh` printed. In the `watch.sh` line, "lag" is how old a frame is
when the node has acted on it and "cpu" is the work per frame, split into
decoding the camera's JPEG and searching it for the ball. Each camera does that
work in a process of its own (since 2026-11-26; `top` shows them as three extra
`python3` lines beside the dodge node). Measured on this Pupper on 2026-11-26
(evening light, nobody throwing): the fisheye at about 20 pictures a second with
3-6 ms lag, the ELPs at 20-26 a second with 9-28 ms lag, and 24-39 ms of work
per picture. Take that as healthy: about 20 a second or more, lag under 50 ms.
Frame rate is what decides a dodge: in simulation, throws from 2-3 m are
dodged 73-100 % of the time at 30 pictures a second, about half the time at
15, and never at 7, which is what the Pupper managed before the cameras had
their own processes. If the rates are far below 20, lag climbs, or `top` shows
the Pi near 100 % on all four cores, something else is using the processor:
`start.sh` names the `llm-agent` service if it is running; stop it.
`python3 tools/bench_detect.py data/throws/<a saved throw>` shows what each
camera's work costs on the Pi by itself. `COLOUR CAST` after a camera in the
`watch.sh` line means that camera is not showing the white balance it was
given: the node sends its settings again by itself; if the words stay, run
`tools/tune_exposure.py` and then the colour step for that camera.

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
| Command, to the mux passing it on | The velocity mux's delay. A few ms with `robot/fast_mux.py`, the default since 2026-11-26; 40-43 ms were measured with the monorepo's mux (`--slow-mux`) |
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
| Dodges (or does not dodge) differently from before 26 Nov night | The ball is now judged by how its direction moves, not by its size (`threat.use_bearing`) | To compare with the earlier way: put `threat: {use_bearing: false}` in a file such as `config/calib/method.yaml` and restart |
| Reacts too late | Open the throw in Pupper Dodging View: it says where the time went | Few pictures per second or old pictures (age over 100 ms) in "Cameras during this throw": the Pi is overloaded, see stage 7. Ball first seen close to the robot: lighting or colour calibration (stage 5). A long time from command to the mux passing it on: check `start.sh` was not given `--slow-mux`. Otherwise more time in the air: throw higher, or from further back |
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
  on the other; the detector now removes the colour cast itself. All of this was
  checked against the six pictures taken on the robot at that point.
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
- **Seen on the robot (2026-11-26):** both ELPs streaming together, the
  exposure tuner on the real cameras, and the dodge node in the real ROS stack.
  Armed throws that afternoon were dodged late or not at all; the causes were
  found by measurement on the Pi and fixed (see `plan.md`, Implementation
  status). Afterwards, in dry run with nobody throwing: 20-26 pictures a second
  and 3-35 ms lag (5-15 and 130-390 ms before), no false "throw seen", the
  dodge node at about a fifth of one core. All tests passed on the Pi (85 at the time).
- **Not verified - needs steps C to G at the top:** any throw since those
  fixes, armed or in dry run; the colour and size calibration, which predates
  them; the joystick's priority over the dodge command with `robot/fast_mux.py`
  on the robot; the white-balance wake running by itself after a power-up (the
  mechanism was shown by hand on the live cameras); the fixes for false dodges
  from handled and still balls (checked on five recordings and in simulation
  only); the colour step's new rules; whether separate USB
  controllers raise the rear cameras' rate; and how the real robot accelerates.
  Expect to adjust numbers in `config/` along the way. `/tmp/sar-dodge/robot.log`
  says what the node could not do.
