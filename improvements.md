# Improvements: what to do or redo on the Pupper

One list for everything still open. Each item says what to do, why, about how
long it takes, and where the full steps are. Items marked **(you)** need you
at the robot. Items marked **(code)** are changes still to be made in the
code; most need a test on the robot afterwards.

Two Claude sessions write this file, each in its own sections:

- **Projectile dodging: calibration, room, rolling balls, two balls**, from the
  "System limits" session.
- **Projectile dodging: speed, camera rate, robot response**, from the
  "Issues" session (added when its changes are pushed).

---

## Projectile dodging: calibration, room, rolling balls, two balls

### 1. Measure the two rear lenses, step C4 (you, about 10 min, once)

- **Why:** the rear lenses have never been measured.
  - Straight behind, the ball is at the outer edge of both rear pictures,
    which is where an unmeasured lens is furthest off.
  - In simulation, a lens 4° off at its edge cut straight-behind dodges from
    97 % to 60 %.
  - This calibration is done once, not again in each room.
- **How:**
  1. On the Mac, open `projectile-dodging/tools/checkerboard_9x6.png` full
     screen on a laptop or tablet.
  2. On the Pupper, run `python3 tools/calibrate_camera.py elp_left`, then
     `python3 tools/calibrate_camera.py elp_right`.
  3. Spend at least a third of the views at the picture's outer edge and its
     top corners.
- **Full steps:** `projectile-dodging/DEPLOY.md`, step C4.
- **Done when:** each run ends with a reprojection error under 1 px and
  writes `config/calib/elp_..._intrinsics.yaml`.
- **Then** tell Claude "calibrated", so the files are copied off the Pupper
  and committed.

### 2. Redo the colour and size calibration, steps C1–C3 (you, about 20 min; again in the demo room)

- **Why:** the current calibration is from 23 Nov and was made with a yellow
  cast in the picture.
  - `elp_left`'s colour range lets dark floor through.
  - `elp_right`'s size scale is 1.38 where it should be near 1. Its range
    reading is about a third off, and its range disagrees with `elp_left`'s by
    25 % straight behind.
  - The fisheye's colour range did not come out green at all. It is ignored,
    and that camera runs on generic settings.
- **How**, with the dodge node stopped (`bash robot/stop.sh`, then
  `bash robot/start.sh --no-dodge`):
  1. **C1:** run `python3 tools/tune_exposure.py all`.
  2. **C2:** for each camera, run `calibrate_color.py <camera>`, then
     `calibrate_color.py <camera> --distance 0.5 1 1.5 2`. Hold the ball by
     your fingertips from below, so the camera samples the ball and not your
     hand.
  3. **C3:** run `tools/view.py` for each camera to check the result.
- **Done when:**
  - each colour step finds the ball in 90–100 % of pictures, with a hue range
    inside about 28–75;
  - `elp_right`'s size scale is close to 1;
  - the ranges in `view.py` are within about 10 % of a tape measure.
- **Full steps:** `DEPLOY.md`, steps C1–C3.
- **Do it again in the demo room**, in the light you will throw in.

### 3. Put the rear cameras on different USB controllers, step B (you, 5 min)

- **Why:** both rear cameras share `xhci-hcd.0`. With the node running, each
  camera gets 15–21 pictures a second.
  - At those rates the camera speed limits dodging most.
  - It matters even more for two balls (item 7).
- **How:** move one rear camera's plug to another socket, then run
  `python3 tools/find_cameras.py` and `--assign`. Write down the two rates it
  prints.
- **Full steps:** `DEPLOY.md`, step B.
- The other camera-speed changes (half-size search, fisheye JPEG quality) are
  in the Issues session's section.

### 4. Check the leg screws before each session (you, 2 min)

- **Why:** on 27 Nov a screw had come out of the back-left hip
  (`leg_back_l_2`). That motor then failed under `start.sh` and `walk.sh`.
  Dodging is the hardest the legs work.
- **How:**
  - Before powering up, check that every leg screw is tight, the back-left
    hip especially.
  - If a screw keeps coming loose, a drop of removable thread-locker
    (blue, medium strength) holds it.

### 5. Set up the room and the throw (you, each session)

**Space and floor**

- Keep at least 1.5 m clear on every side.
  - The node has no map of the room. It picks a side from the ball alone.
- On hardwood, or if the feet slip in step F, put the robot on a rug or mat
  first. Change no number to make up for slipping.

**What a dodgeable throw looks like (in simulation)**

| Throw | Result |
|---|---|
| From 2–3 m, with a lob at least 0.5 m above your hand | 88–100 % dodged in every direction |
| From the front, beyond 3 m | 62 % at 3.5 m, 0 % at 5 m (the front camera sees far balls late) |
| From 1 m | Too close to dodge |
| Flat throw, no lob | Not dodged |

**Testing while walking**

- Dodging has not yet been tried while the robot walks: test standing still
  first.

### 6. Rolling balls along the floor (code, then you)

- **Today:** a ball rolled along the floor is never dodged.
  - The threat check treats anything at floor height as lying still ("on or
    below the floor"). That is what stops the node dodging balls on the floor
    and green cushions.
- **Proposed code:** a separate check for a ball on the floor, taking its
  range from how far below the horizon it is seen.
  - With the camera at a known height above the floor, a ball rolling 2 m
    away is seen about 4° below the horizon. That gives its range more
    steadily than its size in the picture does.
  - A ball counts only if it is seen closing in on the robot along the floor
    over several pictures.
  - A ball lying still, or one rolling past, does not count.
- **What you would do once it is written:**
  1. Roll a ball at the robot from 2–3 m in dry run, from the front, the
     sides and behind. Those throws are recorded.
  2. Check in Pupper Dodging View that a still ball and a ball rolling past
     are not dodged.
  3. Only then arm it.

### 7. Two balls at once (findings; test on the robot)

**What happens now (simulation, 27 Nov)**

- **Settings:** the Issues session's in-progress code: forward run 0.45 m/s,
  no diagonals, cameras at 17–19 pictures a second.
- **Two robot models:**
  - **Simple:** reacts in 0.06 s and gets up to speed with a 0.2 s lag.
  - **MuJoCo-fitted:** from `reports/falls-mujoco`. It is slower sideways and
    has about 0.1 s of delay.
- Each row is 64 tries. A difference under about 6 % is noise.

| Throw | Untouched, simple robot | Untouched, MuJoCo-fitted robot |
|---|---|---|
| One ball | 94 % | 75 % |
| Two together, same thrower | 83 % | 58 % |
| Two together, 20–60° apart | 70 % | 44 % |
| Two together, at right angles | 72 % | 52 % |
| Two together, from opposite sides | 75 % | 58 % |
| Second ball 0.2–0.4 s after the first | 89 % | 69 % |
| Second ball 0.4–0.8 s after the first | 83 % | 64 % |
| Second ball 0.8–1.6 s after the first | 88 % | 62 % |

**Why balls thrown together fail**

1. **The second ball is usually the one that hits.** The robot steps out of
   the first ball's way and stays in, or steps into, the second ball's path.
   That happened in 16 of 19 failures 20–60° apart, 12 of 18 at right angles
   and 11 of 16 from opposite sides.
2. **The second ball isn't known yet when the move is chosen.**
   - The dodge is decided when the first ball is confirmed, about 0.2 s after
     the throw.
   - At that moment the second ball has been seen only once or twice, too few
     looks to tell where it is going.
   - It is confirmed 50–90 ms later, once the robot is already moving.
3. **There is no time to wait for it.**
   - At the moment the first ball is confirmed, the dodge is usually already
     late: 0.6–0.7 s left, against 0.7–0.85 s needed.
4. **More time is what helps most.** In simulation, a move was picked with
   perfect knowledge of both balls (the shipped moves, the MuJoCo-fitted
   robot).

   | Throw | Both cleared, deciding when Pupper does now | Both cleared, deciding 0.1 s earlier |
   |---|---|---|
   | 20–60° apart | 62 % | 93 % |
   | At right angles | 62 % | 89 % |
   | Opposite sides | 83 % | not checked |
   | Same thrower | 74 % | not checked |

**What was tried, and did not help** (none was kept; each was within noise
in both robot models):

- choosing the move to clear every ball in the air, judged on each ball's
  predicted arc;
- a second look in the first 0.2 s of the dodge, changing direction once if
  a newly seen ball would hit;
- waiting one more look when another ball has just appeared;
- treating a ball seen only once or twice as aimed at the robot.

Changing direction freely mid-dodge made balls thrown together worse
(same thrower: 83 % fell to 50 %). The robot wastes the motion it has already made.

**What would help**

- **Deciding earlier:**
  - faster cameras (item 3 and the Issues section);
  - a lower threshold for a ball counting as confirmed (needs its own
    false-dodge check).
- **A faster first step:** the robot's measured response (step F; Issues
  section).

**What you can do now (you, 10 min, dry run, nothing moves)**

1. Have two people throw at the robot together from 2–3 m, about ten pairs:
   - from the same spot;
   - from 30–60° apart;
   - from opposite sides.
2. The recordings show when the second ball is first seen on the real
   cameras. That tells us whether the half-size search or step B gets the
   second ball known in time.
3. Only then arm it.
   - Leave 1.5 m clear on every side, and keep someone on the e-stop.

**Not yet run on the robot:** the second-ball change of 27 Nov (5109ca3: a
ball thrown 0.8–1.6 s after the first is dodged too).

### 8. Straight up and close overhead (decision, then code or hardware)

- **Why:** the rear cameras see only 30° above the horizontal, and they
  cannot be tilted. A high lob from behind is out of sight 0.1 s after
  release.
- **Options:**
  - **(code)** Take the ball's range from both rear cameras at once (stereo),
    not from its size. This needs C4 first.
  - **(hardware)** Add a camera looking up.
  - Either is a project of its own. Say if you want one.

### 9. Ball colour in changing light (you, then code)

- **You:**
  - More light in the room helps every camera.
  - Keep the blinds the same between calibration and throwing.
- **(code)**
  - Fix the fisheye's exposure as the rear cameras' already is.
  - Learn the ball's hue online from confirmed throws, so a slow change in
    light does not need a new C2.

### 10. Moves that need a TA's OK before their first run

None of these has been tried, and each needs a TA's OK first:

- crouching under a ball;
- a trained dodge policy;
- walking back to the starting spot after a dodge, so a long session doesn't
  drift towards a wall.

---

## Projectile dodging: speed, camera rate, robot response

*(The Issues session adds this section when its changes are pushed.)*
