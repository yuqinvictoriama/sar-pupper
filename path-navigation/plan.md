# Path navigation – the demo mission (plan v3)

Written 2026-11-26. This is the module that runs the demo from start to finish: hear a cry for help, turn to it, cross two obstacles, walk through thrown balls, reach the hidden survivor, stop on "You saved me, Pupper!", and comfort them. It sits beside `terrain-agility/plan.md`, `projectile-dodging/plan.md` and the mission personality, and it is the one thing that drives them in order.

**This file is the why. `README.md` is the how: it holds "Demo instructions" (materials, course, script, both calls), how to start a run, and what has been verified.**

**Status (v3, 2026-11-26, evening).** The module is built: the mission state machine (`nav/`), the robot node (`robot/nav_node.py`), the launch and start scripts, a simulator of the whole run (`sim/course_sim.py`) and Pupper Mission Control (`tools/mission_control.py`), a page that shows the run and starts it with one click. 46 tests pass off the robot, and the simulated run ends in front of the cover (figures in the README). **None of it has run on the robot**, and the sound module has still not been seen: its topic and sign convention are settings in `config/demo.yaml`.

**Changed in v3 – where the build departs from the text below:**

- **Demo instructions moved to `README.md`**, so there is one copy.
- **Operator commands are one topic, `/nav/command`** (`start`, `pause`, `resume`, `end`, `phrase`, `motion_on`, `goto:<STATE>`, `bearing:<deg>`), not four `Trigger` services and a `SetBool`. Mission Control's buttons and `tools/nav_cmd.sh` both write to it. `/nav/goal` was not built: nothing reads it.
- **The fix is the largest group of bearings that agree**, not a plain median; ties go to the group nearest where the voice is expected. Two bearings 40° apart never average to a direction nobody called from.
- **Confirm blends the two looks** (first fix and the nose-on one, half and half, when they agree within 10°). Two independent readings at ±5° give about ±3.5°: 0.37 m at the cover instead of 0.52 m.
- **The second listen waits for the walk-around to finish**, not only for 2.0 m, so a distance estimate that reads long cannot stop Pupper in front of the tall obstacle with terrain agility switched to report-only.
- **The low obstacle's landmark and the crossing's length come from the step-over node's reports** (`planning`, `started`, `progress`, `done`, with `advance`), not from terrain's state names, so they survive terrain agility growing new obstacle types.
- **Terrain agility has the three-way mode** (`/terrain/mode`: full, avoid, report) and publishes the box measurement and the crossing's advance. Built and committed in `terrain-agility/`.
- **A stuck walk-around is retried**: if terrain agility holds twice, navigation steps back 0.45 m and lets it look again (up to twice) before asking for the operator, and between the low obstacle and the second call it holds the line's direction without steering back onto the line itself (the line runs through the tall obstacle). The simulation found terrain agility stopping beside the tall obstacle in 2–10 % of walk-arounds; the terrain work then fixed its steering, and these stay as a safety net.
- **The course is a list of obstacles in order** (`course.obstacles`), and the mission handles several crossings in a row: each one gives a landmark and adds its own travel, terrain agility stays in `full` until the last listed obstacle is behind Pupper, and the second listen waits for all of them. As laid out on 2026-11-26: a flat 19 × 89 mm bar at 0.45 m (stepped over), then the 15 × 565 × 380 mm slab at 1.15 m (scaled across). A cylinder was considered for the first place and dropped by the owner the same day: terrain agility cannot step over a round bar and, in simulation, does not reliably walk around one from its measuring distance (README, "Why a flat bar and not a cylinder"). The 0.6 m / 1.7 m figures and the tall box in the text below are the first layout; the reasoning about spacing still holds.
- **Speeds follow the one measurement of the legs' real speed** (0.80 m in 5 s at 0.30 m/s commanded, about 0.53): `reckon.forward_factor` 0.53, and zone 1 and the gauntlet both commanded at 0.30 m/s (about 0.16 m/s on the floor), not the 0.25 and 0.20 m/s in the text below.
- **`dodge` is not sent on `/nav/event`**: the personality's bridge takes it from `/dodge/event` itself. Navigation only counts dodges.
- **After 30 s at the cover with no phrase, Pupper says the `hold` line once** ("I've stopped. I need my team. Still here.") and keeps waiting; there is no separate event for it.
- **Past the cover's expected distance with the camera pipeline silent, Pupper stops at 6.0 m** instead of walking on to the 7.2 m hard stop blind.
- **The event bridge and the call-out words live in `personality/`** (`event_bridge.py`, `callouts.py`); navigation reads the words from there.
- **Not built:** the free-space feed from terrain's scan to the dodge node (dodging decision D4), the recorded-clip fallback (the personality's own voice now covers it), `tools/calibrate_speed`, `yaw_drift`, `bearing_test`, `walk_line`, `show_run` (the run log in `data/runs/` is JSON lines; Mission Control shows the same numbers live).

**Changed in v2 (2026-11-26, earlier):**

- Hearing the phrase is our own job, not the sound module's. The detector is written and its tests pass; it has not been on the robot.
- The second call at 2 m is decided: the survivor will call again when Pupper asks.
- The sound module's accuracy is taken as about ±5°. The error budget, the cover width and the tall obstacle's position follow from that.
- Floor markers moved off the line: tape across Pupper's path would read as an obstacle.

**What this plan could not check.** Your partner's sound module was not readable when this was written. Every topic name for it is a proposal. Items marked **(confirm)** in the text below were written before the other components' code could be read; the build checked the terrain and dodging ones against the code.

## What changed, and what this replaces

- **Survivors are found by sound, not by sight.** The arms-up signal, the pose model on the Hailo, the people tracker, the visual lock and the approach node are retired. `survivor-id/` is archived (see "Archived" at the end).
- **Route mapping is this module.** The earlier plans left "route mapping" as a component to come. For the demo the route is one straight line of known length, so route mapping becomes: hold a line, know how far along it Pupper is, and switch the other components on and off by zone.
- **Terrain agility and dodging are unchanged in what they do.** They get a new driver (this module) and three small additions each (see "Changes needed in the other components").

## Goal

One uninterrupted run, in this order:

1. **Seek.** Pupper stands still and listens for a cry for help.
2. **Orient.** It marks the direction of the cry and turns to face it. That direction is the line it walks.
3. **Obstacles, 0–2 m.** It steps over a low box, then walks around a tall one.
4. **Gauntlet, 2–6 m.** It walks the line while dodging lobbed foam balls.
5. **Arrive, about 6 m.** The survivor is behind a board or box. The survivor says "You saved me, Pupper!" and the search ends.
6. **Rescue and recuperate.** Pupper stays and gives words of comfort and encouragement.

### How this plan reads the request

Each of these is a default that is easy to change; the ones still open are listed under "Decisions".

- **The 6 m and the 2 m are known in advance.** The sound module gives a direction, not a range, so the course length is a number in the config, not something Pupper measures from the cry.
- **Pupper stops in front of the cover.** It does not walk around the board to the survivor's side (decision N1).
- **The keyword ends the search wherever Pupper is**, as long as it is past 4 m. Distance alone never ends it.
- **Balls are thrown only in the gauntlet**, so dodging is armed only there (decision N4).
- **The sound module gives a direction and nothing else.** Hearing the phrase is our own detector's job.
- **The survivor calls twice**: at the start, and again when Pupper asks, about 2 m in.

## Demo instructions

Moved to `README.md`, section "Demo instructions": materials, course measurements, who does what, the run step by step, the two calls and the phrase, the throwing rules, the checklist before every run and what to do when something goes wrong. Mission Control shows the same cues live, one at a time.

## Fixed facts

### What the sound module gives (assumed)

- A bearing to the cry, in the robot's own frame, each time it hears one, good to about ±5° (your estimate; Phase 0 measures it).
- No range, and no phrase detection.

The four mics sit in one horizontal plane on a 132 × 221 mm rectangle, 78 mm above the hip axes, each facing 45° outward (design 2 notes). They are close to the leg motors.

### What Pupper does not have

- **No odometry.** The only motion sensing is the IMU and the velocity commands themselves. Distance along the line is an estimate; expect 10–20 % error until the speed scale is calibrated on the demo floor.
- **No compass.** Heading is the IMU's yaw relative to where it started. It drifts slowly; how slowly needs measuring (Phase 0).
- **No dodging during a step-over.** A crossing takes 45–62 s plus planning, and the legs are not in the walking controller for any of it.

### Numbers taken from the other plans

| Fact | Value | From |
|---|---|---|
| Step-over box | 19 × 89 mm, 500–600 mm wide, blue sides, white top | terrain |
| Box is seen from | 1 m; Pupper stops 36 cm short (body centre), squares up, measures | terrain |
| Crossing time | 45–62 s crawl + 2–20 s planning (about double on the Pi) + 4 s for the walking controller to restart | terrain |
| Clear floor needed beyond the box | 35 cm | terrain |
| Tall obstacle | at least 80 mm tall (250 mm is unmistakable), not floor-coloured | terrain |
| Walk-around trigger and clearance | obstacle within 1.2 m in a ±18 cm lane; passes with 22 cm between obstacle and body centre | terrain |
| After a walk-around | returns to the *heading* it had, not to the *line* it was on | terrain |
| Dodge envelope | underhand lobs from 3–4 m, one ball in the air, 3 s apart, Pupper walking at up to 0.3 m/s, 1 m clear around it | dodging |
| Dodge move | up to 0.4 m and 0.8 s, perpendicular to the ball's path; then about 1 s before it will dodge again | dodging |
| Simulated dodge rate | sides and rear 100 %, front diagonals about 93 %, straight ahead about 57 % | dodging |
| Mux rule | any message, even zeros, makes a source active; a source silent for 500 ms is dropped | both |

## Why the course is laid out this way

The measurements themselves are in "Demo instructions". Distance along the line from the start mark, to Pupper's body centre, is called **s** from here on. All of these are starting values to test.

- **Low box at 0.6 m, 600 mm wide.** Pupper sees it at once, walks 24 cm, stops 36 cm short and measures. A 5° aiming error is only 5 cm here, so both feet still land well inside the box's ends.
- **Tall obstacle at 1.7 m, centred.** A 5° error is 15 cm at this distance. The obstacle is 40 cm wide, so it is still in Pupper's path, but which side is shorter depends on where Pupper happens to be. Hence room on both sides, and no attempt to arrange the side in advance.
- **The tightest spot is between the two obstacles.** Pupper finishes the crossing roughly 0.7 m before the tall obstacle. The walk-around was simulated from 1.2 m. Run `sim/avoid_sim.py` from 0.7 m first; if it clips or stops, move the tall obstacle to 2.0 m and everything after it by the same 0.3 m rather than squeeze.
- **Throw window 3.0–5.2 m.** A backward dodge of 0.4 m from 3.0 m still ends about 0.5 m clear of the tall obstacle. A forward dodge of 0.4 m from 5.2 m still ends short of the stop point.
- **Throwers 3 m to the side, level with 4.0 m.** Every throw in the window then arrives 72°–112° off Pupper's nose from 3.0–3.2 m: the side sector, the easiest to dodge, and the escape is forward or backward along the line instead of off it.
- **Cover at 6.4 m, 1.2 m wide.** Pupper's centre stops near 6.0 m with its nose about 0.3 m from the cover. The width is what forgives the bearing error (see "Error budget").
- **Markers 1 m off the line.** Terrain agility cannot tell a flat mark from a low step unless it is the two-colour box, so a stripe across the lane would be walked around, or stopped for in the gauntlet.
- **Nothing else near the line.** People, bags and chair legs within about 0.5 m of it are obstacles too.

## Approach

- **A mission state machine on top, reflexes underneath.** The new node decides *which zone Pupper is in* and *where the line is*. It never decides a dodge or a footstep. Dodging and terrain agility keep their own mux inputs above it and interrupt it exactly as designed.
- **Walk a line, not a heading.** After a walk-around or a dodge Pupper is off to one side. Holding the heading alone would carry that offset all the way to the cover. The node tracks its sideways offset and steers back onto the line.
- **Listen only when standing still and silent.** Motor and footfall noise and Pupper's own voice all reach the mics. Robot-audition work treats this self-noise as the main obstacle to localising sound on a moving robot, so every bearing this plan uses is taken at a standstill, with Pupper not speaking.
- **Three independent ways to know it has arrived**: the keyword, the cover appearing ahead in the camera, and the distance estimate. The keyword is the truth; the other two stop Pupper from walking on forever if the keyword is missed.
- **Same shape as the other components**: logic in plain Python with tests that run on a laptop, ROS only at the edges, and a dry run by default (it reports what it would do and sends no velocity until `motion:=true`).

## The mission, state by state

```
IDLE → PREFLIGHT → SEEK → ORIENT → CONFIRM → ZONE1 → REFIX → ZONE2 → ARRIVE → RESCUE → DONE
                    ↑________|  (fix rejected)                          |
any moving state ⇄ PAUSED (joystick, fall, e-stop)      any state → HOLD (stand still, report)
```

| State | What Pupper does | Leaves when | Dodge | Terrain |
|---|---|---|---|---|
| **IDLE** | Nothing. Publishes nothing | Operator starts the run (`/nav/start`, or a joystick button) | off | off |
| **PREFLIGHT** | Checks everything is alive (list below). Sends `start`; Gemini says the opening line | All checks pass and 6 s have gone by for the line to finish | off | off |
| **SEEK** | Stands still, silent, listening | 3 bearings (or 1.5 s of them) that agree within 8° | off | off |
| **ORIENT** | Sends `heard`, barks. Turns in place to the bearing at 0.8 rad/s, shortest way, until within 5° | Facing it and still for 0.5 s | off | off |
| **CONFIRM** | Listens again, now facing the cry. Corrects the line by what it hears | Next cry within 10° of straight ahead → go. More than 25° off → back to SEEK. Nothing in 6 s → go with the first fix | off | off |
| **ZONE1** | Walks the line at 0.25 m/s. Terrain agility steps over the box, then walks around the tall obstacle | s ≥ 2.0 and terrain is back in CRUISE | off | full, then walk-around only once the box is crossed |
| **REFIX** | Stops. Sends `refix` ("Call out again"). Listens once more and redraws the line from where it now stands | A good bearing, or 6 s without one (keeps the old line) | off | report only |
| **ZONE2** | Walks the line at 0.20 m/s. Dodges. Steers back to the line after each dodge | Keyword (if s ≥ 4.0), or the cover ahead within stopping range (if s ≥ 4.8), or s ≥ 7.2 | **armed** | report only |
| **ARRIVE** | Stops, holds position. Sends `arrived` ("I'm here. Can you hear me?"), repeated every 8 s | Keyword. After 30 s without it: reports to the team and waits for the operator | armed until the keyword | report only |
| **RESCUE** | Sends `saved`. Stays beside the cover. Gemini gives the comfort and encouragement steps | Operator ends the run, or "stand down" | off | off |

**PAUSED** – the joystick, a fall or the e-stop takes over. The node keeps estimating where Pupper is from the commands actually sent, so the operator can nudge it and let go, and the run carries on from the right place.

**HOLD** – something it cannot solve alone: terrain reports HOLD, a crossing aborted, a turn that does not finish in 8 s, sensors gone quiet. Pupper stands still, says so, and waits for the operator. A crossing that aborts leaves the legs soft and the walking controller off on purpose (terrain plan); the run resumes only after a person stands Pupper up and presses resume.

### Preflight checks

Refuses to start, and says which check failed, unless:

- the walking controller is active and Pupper is standing level;
- IMU, fisheye frames, `/terrain/state`, `/dodge/state`, the sound module's heartbeat and the keyword node's (`/keyword/status`) have all been seen in the last second;
- the mux order is right (joystick, dodge, terrain, nav) and nobody else is driving;
- the mission feed has a listener (Gemini connected). Without one it warns and carries on: the bark and the recorded lines still work.

### Seek and orient, in detail

- **A fix is several bearings, not one.** Take every bearing the sound module reports while Pupper is still; use the circular median; require a spread under 8°. One bearing can be an echo off a wall.
- **The line is fixed in the IMU's frame at that moment**: line heading = yaw when heard + bearing. From here on the mics are not needed to walk it.
- **Confirm after turning.** A rectangular array is not equally accurate in every direction, and the first cry arrives from anywhere. A second listen with the survivor straight ahead is cheap and removes most of the first error.
- **If nothing is heard**: after 20 s Pupper says so (`no_sound`) and keeps listening. After 60 s it turns 90° and listens again, in case the cry was in a weak direction for the array.

### Walking the line

- **Where Pupper is.** Two numbers in a frame fixed at the start: **s** (along the line) and **d** (sideways off it, left positive). Updated 50 times a second from the velocity the mux actually sent to the legs (`/cmd_vel`, so dodges and walk-arounds are included), turned through the IMU's yaw, scaled by the measured speed factors, and smoothed with a 0.2 s lag because the legs do not follow a burst instantly.
- **During a step-over** there are no velocity commands. s advances by the distance the crossing plan covered, taken from the step-over node's report **(confirm it reports this; add it if not)**. Nothing is added during the 4 s the walking controller takes to restart.
- **The box is a landmark.** Its position on the course is known, and terrain agility measures the distance to it within about 1 cm. When it does, s is reset from it. Everything after the box is then reckoned from a known point, not from the start.
- **Steering.** Aim at a point 1 m ahead on the line:
  - wanted heading = line heading − atan(d ÷ 1 m), limited to 25° off the line;
  - turn rate = 1.5 × heading error, limited to 0.6 rad/s;
  - forward speed = cruise speed, reduced while the heading error is large; turn on the spot if it is over 35°;
  - no sideways command (sideways is Pupper's slow axis, and it keeps the nose, and so the fisheye, on the line).
- **Never silent while it wants to move.** Terrain agility acts only while a driver is asking for motion, and uses the driver's command as "where the robot was going". So the node keeps publishing its command while terrain or dodging has the mux, and publishes nothing at all in IDLE, PREFLIGHT, SEEK, CONFIRM and REFIX.

### The second listen (REFIX)

After the obstacles Pupper is typically 0.2–0.4 m off the line and has been through a minute of crawling on a drifting yaw. Before the gauntlet it stops once, asks the survivor to call again, and redraws the line from its current position straight to the voice. The distance still to go is kept (6.0 − s).

- A new bearing is accepted only if it is within 20° of where the old line says the survivor should be. Otherwise the old line stands.
- The survivor calls when asked. This is decided, and scripted in "Demo instructions".
- It is a 4–6 s pause, and it is the single cheapest way to arrive in front of the cover instead of beside it.

### The gauntlet

- **Speed 0.20 m/s.** Inside the dodging plan's envelope (up to 0.3 m/s), and slow enough for the throwers to get about four throws into the 2.2 m window at one every 3 s.
- **After each dodge** the node waits for the dodge node to let go of the mux, then carries on. Its position estimate already includes the dodge. It sends `dodge` so Gemini says the line.
- **Throw window**: between the two pairs of markers only. Before the first pair the tall obstacle is close behind; after the last pair the cover is close ahead. The dodge node should also be given terrain's free-space scan so it never picks an escape into either (dodging plan, decision D4).
- **Terrain agility reports but does not steer here.** There is nothing to step over or walk around between 2 m and the cover, and the one tall thing ahead is the destination. If something tall does appear in the lane before s = 4.8, Pupper stops for 3 s; if it is still there, terrain's walk-around is switched back on for that one obstacle.

### Arriving

- **The cover is the last tall thing ahead.** Past s = 4.8, a tall obstacle within 35° of the nose is taken to be the cover. If it is off to one side, Pupper steers at it. When it is within stopping range (nose about 0.3 m away), Pupper stops.
- **Pupper's "I'm here" is the survivor's cue.** They cannot see it arrive.
- **Keyword before 4 m is ignored and logged.** It guards against a false trigger ending the run early. The 4 m figure allows for the distance estimate reading short.
- **s = 7.2 m is a hard stop** whatever else is true.
- **A false stop can be undone.** If the phrase is detected by mistake, `/nav/resume` during the rescue state puts Pupper back into the state it left, at the same place on the line.

### Rescue and recuperate

The search is over: dodging is disarmed, terrain is off, Pupper holds position. Gemini is sent `saved` and does these in order, in Pupper's calm voice, one at a time:

1. **Presence.** "I've got you. You're not alone now. I'm staying right here."
2. **Credit.** "You did the hard part. You kept calling, and that's how I found you."
3. **Steadying.** "Breathe with me. In. And out. Good."
4. **The arrival questions already in the prompt**: their name, "Are you hurt?", "Can you walk?", "Is anyone else here who needs help?"
5. **Report.** "Team: survivor found, about six metres from the start, behind cover." Then what they said.
6. **Keep company.** Every 20 s or so while waiting: "Still here." "You're doing well."

The honesty rule from the personality stays: Pupper never says help is coming, or how long it will be, unless the team said so.

- **If Gemini is not connected**: bark, then a recorded clip of lines 1–3. Record it in advance; it is also the only way to get those words exactly.
- **Posture**: standing, by default. Lying down beside the survivor would read well and rests the motors, and with the throwing over there is no dodge to lose (decision N5).

## Error budget

How far to the side of the survivor Pupper ends up, from the bearing error alone:

| Bearing error | Miss at the cover, one fix at the start (6 m) | With the second listen at about 2.3 m (3.7 m to go) |
|---|---|---|
| 3° | 0.31 m | 0.19 m |
| 5° | 0.52 m | 0.32 m |
| 8° | 0.84 m | 0.52 m |
| 10° | 1.06 m | 0.65 m |

- **At your estimate of ±5°**: one fix alone could leave Pupper 0.52 m off centre. A 1.2 m cover reaches 0.6 m either side, so that is inside it with almost nothing to spare. With the second call the figure is 0.32 m, leaving about 0.3 m. That is why the second call is in the script and the cover is 1.2 m wide.
- **Still measure it (Phase 0).** ±5° on an open bench can be worse beside a board in an echoing room, and the first fix is taken with the survivor in a random direction.
- Homing on the cover over the last metre takes up some of what is left.
- Distance: after the box reset, the remaining 5 m at 10–20 % error is 0.5–1 m. That is why distance never ends the run on its own.

## Time budget for one run

| Stage | Seconds |
|---|---|
| Opening line, seek, orient, confirm | 20–35 |
| Walk to the box, square up, measure | about 15 |
| Plan the crossing (on the Pi) | 5–40; short if the same box was crossed in warm-up |
| Crossing, and the walking controller restarting | 50–66 |
| Walk around the tall obstacle | about 8 |
| Second listen | 4–6 |
| Gauntlet, 3.7 m at 0.20 m/s with 4 dodges | 25–35 |
| Arrival and keyword | 5–10 |
| Comfort and report | 45–60 |
| **Total** | **about 3 to 4½ minutes** |

The step-over is a third of the run and the quietest part. Gemini narrates it (`stepping_over`), and a warm-up crossing before the demo makes the planning step quick.

## Interfaces

### Sound module → navigation (proposed; agree these with your partner first)

| Topic | Content | Notes |
|---|---|---|
| `/sound/bearing` | time, bearing in degrees, confidence 0–1 | **Zero is the nose, positive is to Pupper's left** (the ROS convention). One message per cry or several per second while a cry lasts |
| `/sound/status` | alive, mics found | Heartbeat for preflight |

A small `sound_adapter.py` is the only file that knows the partner's real topic names and conventions, so nothing else changes if they differ.

Questions for your partner, because the plan leans on the answers:

1. Is the ±5° the same in every direction, and at 6 m as well as close up?
2. Does it tell a voice from other noise (a clap, a ball landing, the audience)?
3. Which process opens the mics, and how? Two more listeners need audio at the same time: our phrase detector and Gemini's microphone.
4. What does it report while Pupper itself is speaking?
5. How long after a cry does the bearing arrive?

### Phrase detector → navigation (built, not yet on the robot)

Our own node, `robot/keyword_node.py`, wrapping `keyphrase/`. It needs no network and nothing from the sound module except a microphone it is allowed to open.

| Topic | Content | Notes |
|---|---|---|
| `/keyword/detected` | JSON: phrase variant, microphone channel, time | Once each time "You saved me, Pupper!" is heard. Repeats inside 2 s count once |
| `/keyword/status` | JSON: alive, device, input level per mic | Once a second; preflight checks it |

- **How it works.** A small on-robot speech model (sherpa-onnx's keyword spotter) matches the phrase from plain text; nothing is trained. "Pupper" is not a dictionary word, so the list also accepts what the model tends to hear: Pepper, Puppy, Papa, always after "saved me". Quiet audio is levelled first. A hit on any microphone counts.
- **It only reports.** Navigation accepts a detection only in the gauntlet or at the cover, and only past s = 4.0 m.
- **Measured off the robot, synthetic voices only**: 40 of 40 clean; 36–38 of 40 with hiss 15 dB below the voice; 32–35 at 10 dB; 20 at 5 dB; 38–39 with room echo. No false detections on Pupper's own lines, the cry for help, demo chatter (468 clips) or 15 minutes of noise. Seven on 900 clips written to be confusable, all from two sentences ("Are you safe? Me and Pupper are fine." and "You saved my puppy from the rain.").
- **What that means for the demo.** Noise is the weak point, so the survivor speaks up, from within a metre, and repeats every 3 s: at 80 % per try, three tries reach 99 %. Pupper stops at the cover whether or not it hears the phrase, and the operator can send the detection by hand.
- **Not known until it runs on the robot**: real voices, the clip-on mics, motor noise while standing, the real cover and room, and whether the node can open the mics while the sound module has them. `keyphrase/README.md` has the bring-up steps.
- **Backup path.** Gemini also hears the phrase through its own microphone and can start the comfort lines, a second or two later. It cannot stop the legs; only the node above does that.

### Navigation's own topics

| Topic | Type | Direction | Purpose |
|---|---|---|---|
| `/nav_cmd_vel` | `Twist` | out | Into `cmd_vel_mux` |
| `/nav/state` | JSON string | out | State, s, d, line heading, zone, seconds in state |
| `/nav/event` | JSON string | out | `start`, `heard`, `no_sound`, `stepping_over`, `going_around`, `refix`, `arrived`, `saved`, `hold`, `cancelled` |
| `/nav/goal` | `PoseStamped` in `base_link` | out | Where the line's end is from here. Replaces `/survivor/goal` |
| `/nav/start`, `/nav/pause`, `/nav/resume`, `/nav/abort` | `Trigger` | in | Operator |
| `/nav/motion` | `SetBool` | in | Dry-run switch, as for the other components |

It reads: the sound topics, `/keyword/detected`, `/keyword/status`, the IMU, `/cmd_vel`, `/cmd_vel_mux/active_source`, `/terrain/state`, `/terrain/event`, `/terrain/obstacles`, `/terrain/scan`, `/terrain/crossing_status`, `/dodge/event`, `/dodge/state`, `/emergency_stop`.

### Mux order for the demo

joystick → `/dodge_cmd_vel` → `/terrain_cmd_vel` → **`/nav_cmd_vel`** → Gemini.

The earlier lists put navigation below Gemini. For the demo it goes above, so a stray Gemini move cannot take the legs mid-run. The survivor and person-follower inputs are not launched.

### Changes needed in the other components

All small; all **(confirm)** against the code.

| Component | Change |
|---|---|
| Terrain agility | Add `/nav_cmd_vel` to `fsm.intent_topics` and to the mux file |
| Terrain agility | A mode switch with three settings: **full** (step over and walk around), **walk-around only**, **report only** (publishes obstacles and scan, never steers). Today there is only on/off |
| Terrain agility | The crossing report carries how far the body advanced; the box measurement (distance, height) is published in a form a program can read |
| Dodging | Arm and disarm from a topic or service the nav node can call |
| Dodging | Take free space from `/terrain/scan` when choosing an escape (its decision D4) |
| Personality | New and changed call-outs (below); the event names added to `mission_feed.py` and `test_personality.py`; `event_bridge.py` moved out of `survivor-id/robot/` and pointed at `/nav/event` |
| Launch | A new `demo.launch.py`: the dodging launch (with `follower:=false`), the terrain launch, the personality server, the sound module, the keyword node, the nav node. It takes over from `sar_survivor.launch.py` |

Not launching the survivor nodes frees the Hailo and a good share of the Pi's CPU and heat, which the three camera pipelines and the sound module can use.

### What Pupper says

| Event | Line | Change |
|---|---|---|
| `start` | "Pupper, on duty. If you need help, call out. I will come to your voice." | Was "raise both arms high" |
| `heard` | "I hear you. Keep calling. I'm coming to you." | Replaces `locked` |
| `no_sound` | "I can't hear anyone yet. Call out if you need help. Still searching." | New |
| `stepping_over` | "Low obstacle. Stepping over. Still coming." | New |
| `going_around` | "Something is in my way. Going around. Still coming." | Was `blocked` |
| `refix` | "Call out again, so I can find you." | New |
| `dodge` | "Something flew past me. I'm okay. Still coming." | Unchanged |
| `arrived` | "I'm here. I can't see you. Can you hear me?" | Changed |
| `saved` | The comfort steps above | New |
| `hold` | "I've stopped. I need my team. Still here." | New |

Retired: `candidate`, `locked`, `lost`, `reacquired`.

Two rules for the prompt:

- **Pupper never says the phrase itself, or "saved me" in any line.** Its own voice reaches its own mics. The lines in the table above were tested against the detector and do not set it off.
- If Gemini hears "You saved me, Pupper" before the `saved` event arrives, it starts the comfort steps and does not start them again when the event comes.

After any line, the node waits for the line to finish (about 1 s per three words, at least 3 s) before it trusts a bearing or opens a listening state.

## How this fits the existing codebase

| What exists | How navigation uses it |
|---|---|
| `cmd_vel_mux` (monorepo) | One more input, added in our own mux file. No monorepo edit |
| Terrain agility's state machine | Does every obstacle. Navigation only sets its mode and reads its state |
| Dodge node | Does every dodge. Navigation only arms it and reads its events |
| `speed_scale` from terrain's calibration | Starting value for the forward speed factor |
| Mission feed and event bridge (personality) | Unchanged path to Gemini; new event names |
| `gemini_walk` walking controller | The controller navigation drives. Speed stays far below its 1.5 m/s cap |
| Robot server (`/system/mode`, `/stop/immediate`) | Preflight, and clearing any queued Gemini move at the start |
| The pattern of `follow_me_pupper/lab` | Pure logic, tests off the robot, ROS at the edges |

Course rules carried over: no edits to the monorepo in place, no keys in the repo, no raised speed or tilt limits, and **nobody runs this without a person at the e-stop.**

### Proposed layout (`sar-pupper/path-navigation/`)

```
path-navigation/
  plan.md, README.md
  keyphrase/  BUILT. The phrase detector, pure Python: spotter, audio (WAV and microphone), README
  nav/        pure Python, no ROS: course (layout and zones), heading (circular maths, fixes),
              reckon (where Pupper is), steer (line following), mission (the state machine), events
  robot/      keyword_node.py (BUILT), nav_node.py, sound_adapter.py, demo.launch.py, demo_mux.yaml, start_demo.sh
  tools/      keyword_test, make_keywords, get_keyword_model.sh (BUILT); check_demo (preflight by hand),
              calibrate_speed, yaw_drift, bearing_test, walk_line, fake_sound (send a bearing by hand),
              show_run (plot a logged run)
  sim/        course_sim: the whole run in 2D with noisy bearings, speed error, yaw drift and fake dodges
  config/     keyword.yaml and keywords.txt (BUILT); demo.yaml: every number in this plan
  models/     the keyword model, fetched by a script (git-ignored)
  tests/      test_keyphrase.py and its recordings (BUILT, 23 tests); the nav tests
  data/       run logs (git-ignored)
```

Every run logs state, s, d, yaw, every bearing and every event with timestamps. After a bad run, that log is the only way to tell a bad bearing from a bad distance estimate.

## Build order

**Phase 0 – agree and measure (before writing much code)**
1. Sit down with your partner: topic names, the sign of the bearing, and how three listeners (their module, our phrase detector, Gemini) share the mics. An hour.
2. Bearing accuracy: Pupper standing, a person calling from 6 m at 8 directions, 5 cries each, with the cover in place and the Gemini kiosk running. Then the same after turning to face them.
3. Yaw drift: standing for 3 minutes, and across one crossing.
4. Speed factor at 0.20 and 0.25 m/s on the demo floor: walk 4 m, measure with tape.
5. The phrase detector on the robot: bring-up steps 1–4 in `keyphrase/README.md`, survivor behind the real cover, motors holding position. Ten tries, then two minutes of everything else.
6. CPU with everything running that the demo runs.

**Phase 1 – logic off the robot.** `nav/` and `sim/course_sim.py`. Tests for every state transition, angle wrap-around, the line-following law, position through a dodge and a crossing. Tune steering in the simulator with the Phase 0 noise figures. Target: reaches the cover in 95 of 100 simulated runs.

**Phase 2 – walk a line.** Open floor, no obstacles, bearing sent by hand with `fake_sound`. Ten 6 m runs: where it ends against tape, and what it believed. Then the same with a hand-pushed sidestep in the middle.

**Phase 3 – real sound.** Seek, orient, confirm with a real person calling, Pupper not walking on afterwards. Then seek-to-walk on open floor.

**Phase 4 – zone 1.** Fix, box, tall obstacle, second listen. Terrain agility has to have reached rung 8 of its own bring-up ladder first.

**Phase 5 – zone 2.** Walking with the dodge node in dry run, then armed, then with throws. Cover stop. Keyword stop.

**Phase 6 – the whole run.** Without throws, then with. Then the demo script with everyone in their places.

### If time runs short

Each line is a working demo on its own; cut from the bottom.

| If this is not ready | Do this instead |
|---|---|
| Sound bearing is not accurate enough | Second listen stays on; widen the cover. Last resort: Pupper turns to the cry for show, and an operator-supplied bearing sets the line |
| Step-over is not reliable on the robot | Terrain in walk-around only: Pupper goes around both obstacles. Or start the crossing from a ruler-placed mark |
| Rear cameras are not both working | Move the throwers forward so balls arrive 30°–70° off the nose, where the fisheye alone sees them |
| The phrase detector is unreliable in the room | Pupper stops at the cover anyway; the operator presses "Survivor said the phrase" in Mission Control (or `bash tools/nav_cmd.sh phrase`) |
| Gemini is offline in the room | Bark and recorded clips for `heard`, `arrived` and the comfort lines |

## Success criteria

- **Bearing**: within 5° in 9 of 10 fixes at 6 m after the confirm step.
- **Line**: on open floor, ends within 0.4 m of the line after 6 m, and its own distance is within 10 % of the tape.
- **Zone 1**: both obstacles passed untouched in 8 of 10 runs (the terrain plan's own criteria apply to each).
- **Zone 2**: carries on after every dodge, 10 of 10; side throws avoided at the dodging plan's rate (80 % or better).
- **Arrival**: stops in front of the cover, not beside it, in 9 of 10 runs. Stops within 1 s of the end of the keyword in 9 of 10. No stop on anything but the keyword, the cover or the operator, in 10 runs.
- **Before demo day**: three complete runs in a row, with throws, in the demo room.

## Things that are easy to miss

1. **The cover is a tall obstacle.** Left alone, terrain agility will walk Pupper neatly around the survivor's hiding place.
2. **Pupper's own voice reaches its own mics.** No talking while listening, and a pause after every line.
3. **The sign of the bearing.** Left-positive or right-positive, and where zero is. If it is wrong Pupper turns confidently the wrong way. Check it with one clap before anything else.
4. **A walk-around returns to the heading, not the line.** So does a dodge. Without steering back, the offsets add up across 6 m.
5. **Six metres is a guess.** The keyword and the cover are what end the run.
6. **Zeros are a command to the mux.** A node that publishes zero velocity while listening blocks everything below it; a node that goes silent mid-walk makes terrain agility let go.
7. **Do not count distance while the legs are not listening.** During a crossing, and for 4 s after it, velocity commands move nothing.
8. **A false dodge in zone 1 throws away the box measurement** and restarts the approach. Hence dodging off until the gauntlet.
9. **No dodging for about a minute** during the step-over. Throwers must know.
10. **Echoes.** In a room, a cry from behind a board arrives partly off the walls and over the top. Test with the real cover in the real room.
11. **Angles wrap.** 179° and −179° are 2° apart. Every average and every difference of headings needs circular maths.
12. **Speed factor depends on speed.** A factor measured at 1 m/s is wrong at 0.2 m/s. Calibrate at the speeds the run uses.
13. **Who owns the mics.** The sound module, our phrase detector and Gemini's microphone may all want the same USB receivers.
14. **A foam ball under a foot.** Balls from earlier throws lie in the lane. Both detectors ignore them correctly; Pupper will walk into them. They are light enough to be kicked aside, but watch for it in testing.
15. **Applause is a sound source.** Bearings count only in the listening states.
16. **The survivor cannot see Pupper.** They need an audible cue to say the keyword.
17. **The first crossing of the day plans slowly.** Do one in warm-up.
18. **Mission events have an allow-list.** A new event name that is not added to `mission_feed.py` is refused and Pupper says nothing.
19. **Tape on the floor is an obstacle.** So is the tape measure. Markers go 1 m to the side; the tape measure comes up before the run.
20. **Sentences that sound like the phrase.** "Are you safe? Me and Pupper are fine" set the detector off in testing. Nobody near the robot says "saved me" during a run.
21. **The phrase must come after Pupper stops talking.** The survivor's cue is the end of "Can you hear me?", not the start.

## Decisions

### Made

- **N2 – second listen at 2 m: on.** The survivor calls again when Pupper asks (2026-11-26).
- **N3 – who hears the phrase: our own detector**, not the sound module (2026-11-26). Built in `keyphrase/`.
- **Sound module accuracy: about ±5°**, your estimate, used for the error budget until Phase 0 measures it.

### Open

- **N1 – at the cover: stop in front (planned), or walk around it to the survivor?** Going around is a second walk-around with terrain agility, about 10 s more, and ends with Pupper beside the person. It needs 1 m of clear floor beside the cover.
- **N4 – dodging armed only in the gauntlet (planned), or for the whole run?**
- **N5 – in the rescue state: stand (planned) or lie down?** And is the arrival bounce kept? It was designed for the visual arrival, is not trained, and is left out here.
- **N6 – gauntlet speed.** 0.20 m/s gives about four throws. Slower gives more throws and a longer demo.
- **N7 – how the run starts**: a joystick button (planned) or a spoken "start the search".
- **N8 – does the survivor keep calling all the way?** If so, a later version could re-aim at every pause instead of twice.
- **N9 – which microphone the phrase detector listens on**: the array's mics shared with the sound module (planned), or one of its own. Settle with your partner in Phase 0.

## Archived

Superseded by this plan on 2026-11-26. Kept for reference, not launched in the demo.

- **`survivor-id/`** in the repo, and its plan in the project docs: the arms-up signal, pose model, people tracker, visual lock, approach node, turn-to-look, the persistence ladder and the arrival bounce. In the repo: `git mv survivor-id archive/survivor-id`, after moving `event_bridge.py` out.
- **Emotion detection**: no plan or code for it was found in the project docs. If it lives somewhere else in the repo, it moves to `archive/` the same way.
- **In the personality**: the "raise both arms" opening line and the `candidate`, `locked`, `lost`, `reacquired` call-outs.
- **Still used from the archived work**: the mission-event path to Gemini (`mission_feed.py`, the web-app listener, `event_bridge.py`), which this plan depends on.

## Sources

- Ince et al., on a robot's own motion noise degrading sound localisation and speech recognition (Interspeech 2010): https://www.isca-archive.org/interspeech_2010/ince10_interspeech.pdf
- Ego-noise reduction of a mobile robot (2023), on why self-noise is hard to remove: https://arxiv.org/pdf/2303.00829
- sherpa-onnx, the keyword spotter and model the phrase detector uses: https://github.com/k2-fsa/sherpa-onnx
- Project docs read for this plan: `terrain-agility/plan.md` and `README.md`, `projectile-dodging-plan.md`, `claude/survivor-id-plan.md`, both mission-personality docs, and the design 2 notes.
