# Deploying survivor identification on the Pupper

Work through the stages in order. **None of this has run on the robot yet**, so each stage is
also the first real test of that part: expect to fix things, and tell me what you see.

Nothing moves the robot until stage 7. Until you turn motion on, the approach node is in
**dry run**: it goes through every state and reports the command it would send, and sends none.

All commands run **on the Pupper** (over SSH), from `~/sar-pupper/survivor-id`, unless a line
says "on the Mac". Projectile dodging's `DEPLOY.md` stages 1–6 (cameras found and calibrated,
mounting poses checked) come first: survivor ID uses those cameras and that calibration.

**Safety, throughout.** The joystick e-stop (right-stick press) always works and outranks
everything here. Whenever the robot can move (stages 7–9): someone at the joystick, open floor
with 1.5 m clear around the robot and around the person, nothing it could fall off, and the
person it walks to standing still with their feet together. It stops 0.7 m short by a range
estimate that is unproven: for the first runs, be ready to stop it yourself.

---

## 1. Get the code onto the Pupper

On the Mac. Three places changed: `sar-pupper/survivor-id` (new), two files in
`sar-pupper/projectile-dodging/robot` plus `config/dodge.yaml`, `sar-pupper/personality`
(`mission_feed.py`, `run.py`), and one file in `gemini_pupper/web`.

```bash
cd ~/Desktop/CS123/pi/sar-pupper
git status                       # look at what changed before adding it
git add survivor-id projectile-dodging personality
git commit -m "Survivor identification: perception, approach, arrival, mission events"
git push

cd ~/Desktop/CS123/pi/gemini_pupper
git diff web/services/geminiLive.ts      # the mission-event listener: about 60 lines
```

`gemini_pupper` is your lab repo with other uncommitted work in it, so get that one file to the
robot however you move lab work there (commit and pull, or `scp web/services/geminiLive.ts
pi@<pupper>:gemini_pupper/web/services/`).

On the Pupper:

```bash
cd ~/sar-pupper && git pull
cd ~/gemini_pupper && bash robot/setup.sh        # rebuilds web/dist with the listener (a few minutes)
cd ~/sar-pupper/survivor-id
bash tools/download_model.sh                     # yolov8s_pose.hef, about 25 MB, into models/
```

If the download says 404: the file name or release changed. Do not take a newer release
blindly – a model compiled for a newer HailoRT may not load. Tell me what
`hailortcli fw-control identify` prints and I will find the matching one.

## 2. Check the code runs (robot stack stopped)

```bash
bash ~/gemini_pupper/robot/stop.sh               # nothing else may hold the Hailo
python3 -m unittest discover -s tests            # expect: Ran 74 tests ... OK   (a few minutes on the Pi)
python3 tools/check_setup.py --no-ros
python3 tools/bench_hailo.py
```

`bench_hailo.py` is the first time this code meets the Hailo. Look for:

- **Nine output tensors**: three grids (80, 40, 20) × channels 64, 1 and 51. Anything else: stop and send me the printout.
- **Inferences per second.** The plan assumed 25–30. Put about 80 % of what you measure in
  `config/calib/measured.yaml` as `budget: {inferences_per_s: N}` (see `config/calib/README.md`).
  Under 15, tell me: the wide/zoom schedule needs rethinking.
- **CPU side**: JPEG decode + wide view + a zoom patch should total well under 100 ms.

Then a picture with a person in it (any phone photo copied over):

```bash
python3 tools/try_photos.py ~/photo.jpg --out /tmp/out      # prints who it found; look at /tmp/out/photo.jpg
```

## 3. First look through the robot's eyes (dry run, robot standing still)

```bash
bash robot/start.sh                  # stack + dodge (dry run) + survivor ID (dry run); about a minute
source robot/env.sh                  # in every terminal from here on
python3 tools/check_setup.py         # everything should be OK except the robot server lines
bash robot/watch.sh                  # one line, five times a second
python3 tools/view.py                # then on the Mac: ssh -N -L 8090:localhost:8090 pi@<pupper>, open http://localhost:8090
```

The robot does not need to stand for this; lying on the floor is fine, but the cameras are
then lower than when it stands, so do the measurements below with it standing (X on the gamepad).

Measure these and write them down (plan, Phase 0):

1. **Are people found at all?** Walk round the robot at 3 m. A skeleton should stay on you in
   the fisheye and in both rear cameras. **The rear cameras run at the 3 ms exposure dodging
   set for fast balls.** If people there are too dark to find, that is the exposure compromise
   the plan warned about (item 9): raise `exposure_100us` for both ELPs in
   `projectile-dodging/config/` a step at a time and note what dodging loses.
2. **Range.** Stand at a tape-measured 2, 3 and 4 m in front. `watch.sh people` prints
   `range_m`. If it reads consistently long or short, scale `person.torso_m` in
   `config/calib/measured.yaml` by true ÷ shown (0.48 is the default; the figure in the
   reference photo measures 0.53).
3. **The signal.** Arms up at 2.5, 3 and 4 m, in front, to each side, and behind; facing the
   robot, away, and side-on. You should turn green in the viewer, and `watch.sh` should print
   `EVENT ... locked` within about a second and a half. Cancel between tries: `bash robot/watch.sh cancel`.
4. **Where the rear cameras cut you off.** Arms up behind the robot, walking closer: note the
   distance at which the viewer says "cut off at the top of the frame". The tests assume about 2.8 m.
5. **Things that must not lock** (plan, success criteria): one arm up, waving, a throw, a
   stretch, hands on head, hands on hips, bending to pet the robot, pointing at it, clapping.
   Ten minutes of this with no `locked` is the bar. Note anything that does lock, and from where.

Record while you do 3 and 5, so thresholds can be tuned on the same footage afterwards:

```bash
bash robot/stop.sh && bash robot/start.sh --record data/recordings
# ... do the poses, saying aloud what each is ...
bash robot/stop.sh
python3 tools/replay.py data/recordings/<folder> --frames /tmp/replay     # with the stack stopped: replay needs the Hailo
```

Every threshold is in `config/survivor.yaml` under `gesture:`. Change one, replay the same
recordings, compare the events. False locks first, misses second.

## 4. Pin and hold (dry run, robot standing still)

With `robot/start.sh` running and the viewer open:

- Lock, lower your arms, walk about: the red box stays on you. Walk behind the robot: it
  follows you from camera to camera with the same number.
- A second person walks between you and the robot, then stands next to you: the red box stays on you.
- Step out of sight for five seconds and come back: `lost`, then `reacquired`.
- A second person signals while you are pinned: they appear under `queued`, the pin does not move.
- Turn the robot on the spot with the joystick while pinned: the bearing in `watch.sh` swings
  the other way and comes back; the pin does not jump to someone else.

## 5. Mission events reach Gemini (robot standing still)

```bash
bash robot/stop.sh && bash robot/start.sh --kiosk       # or --gemini and open the app from the Mac through the tunnel
python3 tools/check_setup.py                            # now the robot-server lines should be OK too
```

Connect in the app, then post an event by hand:

```bash
curl -s -X POST localhost:8000/mission/event -H 'Content-Type: application/json' -d '{"event": "locked", "details": {"range": 3}}'
```

It should answer with `"listeners": 1`, the app's log should show `MISSION EVENT: locked range=3`,
and Pupper should say its "I see you" line. `"listeners": 0` means the app is not connected or
`web/dist` was not rebuilt (stage 1). If the line shows in the log but Gemini says nothing, tell
me: that is the one piece I could not try (the Live session taking typed text mid-conversation).

In dry run the survivor nodes' own events are deliberately not passed on ("I'm coming to you"
would not be true). To hear them before stage 7, set `events: {announce_dry_run: true}` in
`config/calib/measured.yaml`.

## 6. Dry-run approach: read what it would do

Still motion off. Lock yourself at 3 m, to the side of the robot, and watch `watch.sh`:

- `LOCKED`, then `APPROACH`, with `cmd +0.00 +0.00 +1.50` (turning toward you) – the sign should
  be **positive when you are on the robot's left**.
- Walk round to the front: `cmd +0.60 ...`. Walk up to it: `+0.30` inside 2 m, then `ARRIVED`
  and `STAY` at 0.7 m, with a bark. (No bounce in dry run.)
- Step back past 1.2 m: `APPROACH` again.

If the turn sign is wrong or the stop distance is off by more than 20 cm against a tape
measure, fix that here (stage 3 item 2), not with the robot moving.

## 7. Motion on: open floor

**Someone at the joystick. Robot standing, in the walking controller. Open floor.**

```bash
bash robot/watch.sh motion on        # or start with: bash robot/start.sh --motion --gemini
```

First runs, in this order, stopping after each to look at what happened:

1. Person 2.5 m straight ahead. Arms up. Expect: bark, it walks, slows, stops about 0.7 m short,
   barks, and (with the robot server running) does **push-ups** – the stand-in for the bounce –
   then stands. Measure where it stopped.
2. The same from 4 m.
3. Person 3 m to the side: it should turn on the spot first, then walk.
4. Person 3 m behind and to one side.
5. Person 2 m directly behind: it should turn to look, lock, then come.
6. After it arrives, walk 2 m away: it should follow and stop again, without a second trick.
7. Press the joystick mid-approach: it must stop for as long as you hold the stick, then carry on.

`bash robot/watch.sh motion off` puts it back in dry run at any time; `watch.sh cancel` drops the pin.

Things I expect may need changing after these: `mission.stop_m`, `speed_far`, `turn_rate`,
`yaw_kp` (if it weaves), `tracker.cmd_tracking_factor` (how far it really goes per commanded metre).

## 8. Interruptions and obstacles

Only once stage 7 is boring.

- **Dodge during the approach**: `bash ../projectile-dodging/robot/watch.sh arm`, throw from the
  side while it walks. It should dodge, then carry on to the survivor.
- **No throws during the trick**: it cannot dodge then (`/survivor/state` says `can_dodge: false`).
  With a ball in the air at arrival it waits up to 3 s, then skips the trick.
- **Something in the way** (a box it cannot step over, terrain agility not running): it pushes
  for 1.5 s, says `blocked`, backs up 0.3 m, sidesteps 0.5 m, tries again, other side next time,
  then wider. This is the blind fallback and it is crude: a soft box, and be ready to stop it.
- **Survivor hides**: it walks to where they were, waits, turns once, tries four spots a metre
  around, and repeats until cancelled.

## 9. The real bounce

`bounce/README.md` has the steps: train the policy (needs an NVIDIA GPU), score it with
`check_feet.py`, **get a TA's OK before its first run on the robot**, fetch it with
`get_trick.py`, copy `bounce/trick.yaml` into `~/gemini_pupper/tricks/bounce/`. The approach
node plays `bounce` in place of `push_up` as soon as the robot server lists it; nothing here changes.

---

## When something is wrong

| What you see | Likely cause |
|---|---|
| `start.sh`: "A robot stack is already running" | `bash robot/stop.sh`, or `bash ~/gemini_pupper/robot/stop.sh` |
| `watch.sh`: `NO POSE MODEL`; log: "cannot start the pose model" | Something else holds the Hailo (`pgrep -af "follow_service\|hailo"`), or the `.hef` is missing or does not match this HailoRT |
| `elp_left:---` in `watch.sh` | The dodge node is not running or has no device for that camera (`../projectile-dodging/tools/find_cameras.py`); or you started with `--no-dodge` and the camera is busy |
| People found in front, never behind | Rear exposure too short (stage 3 item 1) |
| Locks on people who are not signalling | Record it, replay, raise `gesture.wrist_above_head` or `min_fraction`; send me the recording's events |
| Never locks on you | Look at the viewer's label: it says which test failed ("wrist not above the head", "range 1.8 m outside 2.0-5.0 m", "cut off at the top of the frame") |
| `LOCKED` forever with motion on | `watch.sh` shows `not-walking`: the walking controller is not active and the robot server (needed to stand up) is not reachable. Stand it up with the gamepad, or start with `--gemini` |
| It walks but Gemini says nothing | Stage 5 |
| It arrives but does not do the trick | `watch.sh` note says why ("no bounce: ..."): too close, tilted, balls flying, server unreachable, no trick installed |

Logs: `/tmp/sar-survivor/robot.log` (all nodes), `/tmp/gemini-pupper/server.log` (robot server).
