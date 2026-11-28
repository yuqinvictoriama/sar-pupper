# Path navigation: the demo, start to finish

This is the module that runs the demo. Pupper listens for a cry for help, turns to it, walks to it (around anything in its way), asks the survivor to call again, walks through thrown balls, stops in front of the survivor's cover, hears "You saved me, Pupper!" and comforts them. `plan.md` is the why. This file is how to run it, and **"Demo instructions" below is the one place the team's set-up and script live.**

**Status, 2026-11-27.** **The obstacle course is gone from the demo** (decided 2026-11-27): no step-over bar and no slab to scale across. The first 2 m are open floor, and terrain agility only walks around anything in the way and finds the cover. What Pupper could do at the cover (circle a cardboard sheet until it sees the survivor) is planned in [`../survivor-search/plan.md`](../survivor-search/plan.md), not built. Built and tested off the robot only: 77 tests (1 skips without the phrase model), and a simulation of the whole run (see "What is verified and what is not" for how often it ends in front of the cover). **None of it has run on the robot.** The sound-direction module is now your partner's code (`sound-localization/`, his commit ca06155), run unchanged by `robot/sound_node.py`; it has been tested on synthetic four-channel recordings only, never with the DJI receiver on the Pupper. [`SOUND_INTEGRATION.md`](SOUND_INTEGRATION.md) has how it was joined, what was measured and what to fix. "What is verified and what is not" near the end says exactly which claims rest on what.

## Start here

**On a laptop, no robot (the whole run in simulation):**

```bash
python3 path-navigation/tools/mission_control.py --sim --open
```

On a Mac you can instead double-click `path-navigation/mission-control.command`. It needs Python 3 with numpy and nothing else. Press **Start**, or type `start` in the console at the bottom of the page, or in a terminal:

```bash
path-navigation/tools/pupper watch
```

**On the Pupper (the real run):**

```bash
bash ~/sar-pupper/path-navigation/robot/start_demo.sh
```

It also starts the sound module (the DJI receiver must be plugged in, in Q mode: "Wiring in the sound module"). Then on the laptop open `http://<pupper>.local:8078`, and in Mission Control: **Stand up**, tick **Allow motion**, press **Start**. Stop everything with `robot/stop_demo.sh`. Read "First time on the robot" before doing this for the first time.

---

## Demo instructions

Everything the team needs to set up and run the demo. Mission Control shows the same cues live, one at a time, in large type: put the laptop where the survivor's helper and the throwers can see it.

### Materials

| Item | Specification | Qty |
|---|---|---|
| **Cover** (hides the survivor) | A sheet of cardboard or a board, **at least 1.2 m wide and 0.8 m tall**, matte, not floor-coloured, not green. It must stand on its own (a box behind it, or two folded flaps as feet) | 1 |
| **Balls** | The neon green foam pickleballs, 74 mm | 6–8 |
| **Markers** | Flat tape, 30 cm strips: the start mark, three pairs of side markers, two thrower spots. Not cones (a cone is an obstacle) | 1 roll |
| **Tape measure** | 8 m or longer | 1 |
| **Sound module** | DJI Mic 3 receiver in **Q (Quadraphonic)** mode on the Pupper's USB, and four transmitters fixed at the corners of a **120 mm × 234 mm** rectangle on Pupper's back: TX1 front-left, TX2 front-right, TX3 back-left, TX4 back-right, all at the same height. All charged | 1 + 4 |
| **Gamepad** | The e-stop (right-stick press). Charged | 1 |
| **Battery** | Charged, plus a spare. A run is 2 to 3 minutes, more with the comfort steps | 2 |
| **Laptop** | On the robot's network, showing Mission Control | 1 |
| **Floor** | About **8 m long and 7 m wide**, flat, matte, evenly lit | – |

### The course

Lay the tape measure from the start mark to where the cover will stand. That is **the line**. Every distance is read off that tape, from the start mark. **Take the tape measure away before the run**: it is an obstacle too.

```
 metres  0                                   2.0   3.0                 5.2      6.0   6.4
         |-----------------------------------|-----|===================|--------|-----|▓▓ survivor
       start                               second first-throw      last-throw  Pupper cover
        mark                                 call    markers          markers    stops
         |<------ first 2 m: open floor ----->|<------------ last 4 m: dodging ------------>|
                                              Thrower L: 3 m left of the line, level with 4.0 m
                                              Thrower R: 3 m right of the line, level with 4.0 m
```

| Place | At | How |
|---|---|---|
| **Start mark** | 0 m | A small cross Pupper stands over. It may face any way; facing away from the survivor shows off the turn |
| **Second-call markers** | 2.00 m | One each side, **1 m off the line**. Pupper stops for the second call between here and about 2.8 m |
| **First-throw markers** | 3.00 m | One each side, 1 m off the line |
| **Last-throw markers** | 5.20 m | One each side, 1 m off the line |
| **Cover**, near face | 6.40 m | Centred on the line, square to it. Pupper stops about 0.3 m short of it |
| **Survivor** | Behind the middle of the cover | Sitting or crouching, out of sight, head within half a metre of the cover |
| **Throwers** | 3 m either side of the line, level with 4.0 m | Mark the two spots. They do not move during the run |

These numbers are `course:` in `config/demo.yaml` (`course.obstacles` is empty: nothing to cross). If you move anything, change it there too and run `python3 sim/course_sim.py --runs 100` before trusting the new layout.

Rules for the floor:

- **Nothing across the line.** No tape, no cables, no marker in Pupper's path: a stripe on the floor looks like an obstacle to the camera, and Pupper would walk around it. All markers sit 1 m to the side.
- **1 m clear either side of the line** for things, **1.5 m** for people. Thrower spots are 3 m out.
- **No neon green or yellow-green** near the course, clothes included. The balls are the only green thing.
- **Same floor and lighting as the calibration.** If the room or the lights change, redo the floor and ball colours.

**Nothing on the first 2 m (since 2026-11-27).** The step-over bar and the slab are no longer part of the demo. If something is in the way anyway (a bag, a chair leg), terrain agility walks around it and Pupper steers back onto its line. Keep round things (bottles, tubes) out of the lane: in simulation Pupper sometimes stalls in front of one.

### Who does what

| Role | Job |
|---|---|
| **Operator** | Holds the gamepad the whole time (right-stick press is the e-stop). Starts the run in Mission Control. Stays quiet |
| **Survivor** | Hidden behind the cover. Makes the first call, the second call, and says the phrase |
| **Thrower L, Thrower R** | Lob balls from their marks, only inside the throw window |
| **Everyone else** | 1.5 m from the line and silent until Pupper says "I've got you" |

### The run, step by step

| # | Pupper does | Pupper says | The team does |
|---|---|---|---|
| 1 | Operator presses **Start**. Pupper stands still | "Pupper, on duty. If you need help, call out. I will come to your voice." | Silence. Survivor waits for the line to finish |
| 2 | Listens | – | **First call.** Survivor calls **"Help! Help me!"** loudly toward Pupper, once every 4 s |
| 3 | Turns to face the voice, listens once more | "I hear you. Keep calling. I'm coming to you." | Survivor **keeps calling every 4 s until Pupper starts walking**, then stops |
| 4 | Walks the open floor along the line to the voice, slowly (about 15 s), around anything in the way | – | Silence. **No throws.** Nobody near the line |
| 5 | Stops at the 2 m markers | **"Call out again, so I can find you."** | **Second call.** Survivor waits for the sentence to end, then calls **"Help! Help me!"** twice, 2 s apart, then stays quiet. Everyone else silent |
| 6 | Turns a little toward the voice and walks on, slowly | – | Throwers get ready |
| 7 | Walks from the 3.0 m markers to the 5.2 m markers, dodging | "Something flew past me. I'm okay. Still coming." after a dodge | **Throw window.** Throwers follow the throwing rules below |
| 8 | Stops in front of the cover | **"I'm here. I can't see you. Can you hear me?"** | Throwers stop. Silence |
| 9 | Holds still, listening | – | **The phrase.** Survivor says **"You saved me, Pupper!"** in one go, clearly, at speaking-up volume, toward Pupper. If Pupper does not answer, again every 3 s |
| 10 | Stays beside the cover: the rescue and recuperate state | "I've got you. You're not alone now. I'm staying right here." Then "You did the hard part…", "Breathe with me…", then its questions | Survivor answers: name, hurt or not, can walk or not, anyone else. The audience may react now |
| 11 | Reports to the team | "Team: survivor found, behind cover…" | Operator presses **End run** |

### The two calls and the phrase

| | When | Survivor says | Until |
|---|---|---|---|
| **First call** | After Pupper's opening line ends | "Help! Help me!" every 4 s | Pupper starts walking |
| **Second call**, about 2 m in | After Pupper says "Call out again, so I can find you." | "Help! Help me!" **twice, 2 s apart** | Done after two |
| **The phrase** | After Pupper says "I'm here. I can't see you. Can you hear me?" | "You saved me, Pupper!" every 3 s | Pupper says "I've got you" |

- **Wait for Pupper to stop talking before calling.** While it speaks it cannot listen; it waits about a second after its own line before it trusts what it hears. Mission Control's banner turns green when it is the survivor's turn.
- **Why the second call:** the sound module is taken to be good to about ±5°. One fix at the start would put Pupper up to 0.5 m to one side of the survivor after 6 m; a second fix with about 4 m to go brings that to about 0.3 m, inside a 1.2 m cover. If Pupper hears nothing within 6 s it carries on along its first line.
- **Nobody says "You saved me, Pupper" at any other time**, or anything close to it, rehearsal chatter included. Pupper ignores the phrase until it is 4 m along the line, but do not test that on demo day.
- **Between calls the survivor is silent.** So is everyone else: to the microphones, any voice is a direction.
- **One bearing per call, and a pause makes the next one.** The sound module locates a sound once, as it starts, and then waits for quiet before it will locate another. "Help! Help me!" is one call; keep at least a second of silence between calls. A shout that is too quiet gives nothing at all (it must reach the module's threshold, `dji.threshold`; tune it in the room, see `SOUND_INTEGRATION.md`).

### Throwing rules

- **Only while Pupper's body is between the 3.0 m and 5.2 m markers.** Mission Control's banner says "THROW WINDOW OPEN" and "STOP THROWING".
- **Underhand lobs only**, released at waist to chest height, peaking between the thrower's shoulder and about 30 cm above their head.
- **From the marked spots**, 3 m to the side. Balls then arrive from Pupper's side, where it dodges best, and its escape is forward or back along its own line.
- **One ball in the air at a time, at least 3 s apart.** Throwers alternate: left, right, left. Expect four to eight throws.
- **Never before the 3 m markers.** The dodge reflex is switched off until the second call is done.
- **Stop at "I'm here".**
- Balls that land stay where they are until the run ends.

### Before every run

- [ ] Battery charged; gamepad on; e-stop tested once today.
- [ ] Course measured from the start mark; tape measure removed; first 2 m clear; no balls left in the lane.
- [ ] Colours calibrated in this room and this light (floor, ball).
- [ ] DJI receiver plugged in and in Q mode; four transmitters on, at their corners (TX1 front-left … TX4 back-right). Sound panel's **Module** row names the receiver, not "not listening".
- [ ] Mission Control open; under **Systems** every name is green (IMU, walking controller, sound module, phrase detector, terrain agility, dodge node, voice).
- [ ] **Direction check:** one call from the cover with Pupper on the start mark. The ray on Mission Control's compass points at the cover. (Left and right swapped? See "Wiring in the sound module".)
- [ ] **Phrase check:** the survivor, behind the cover, says the phrase three times with Pupper at the stop point. "PHRASE HEARD" lights three times.
- [ ] Pupper on the start mark, **standing**; **Allow motion** ticked; everyone in place; quiet.

### If something goes wrong

| What happens | What to do |
|---|---|
| Pupper says "I can't hear anyone yet" | Survivor calls louder and keeps the 4 s rhythm. Check nobody else is talking |
| The sound module is not giving bearings | Operator clicks the direction of the cover on Mission Control's compass. That bearing is used as the fix. Afterwards: the Sound panel's **Module** row says why (receiver not found, not in Q mode, or calls flagged) |
| Pupper turns the wrong way | Operator presses **End run**, puts Pupper back, starts again |
| Pupper settles onto the floor | That is a safe abort. Mission Control says "Pupper has stopped". Stand it up (**Stand up**), then **Resume** |
| Pupper stops beside something in its way, steps back, tries again | That is its own retry. If it stops for good, nudge it clear with the joystick; it carries on when you let go |
| A ball knocks Pupper over | It pauses. Set it on its feet, **Resume** |
| Pupper stops for the second call and hears nothing | It carries on along its first line after 6 s. Nothing to do |
| No answer to the phrase after three tries | Operator presses **Survivor said the phrase**. Pupper is already stopped at the cover |
| Pupper stops early, as if it heard the phrase | Operator presses **Not the phrase: carry on** |
| Anything else needs a hand on the robot | Touch the joystick: the run pauses, and carries on about a second after you let go |
| Anything looks unsafe | **E-stop. Always** |

If a part is not ready on the day, each of these is still a working demo:

| Not ready | Do this instead |
|---|---|
| Sound bearing | Pupper turns to a bearing the operator clicks on the compass (first and second call) |
| Terrain agility | `robot/start_demo.sh --no-terrain` on open floor; Pupper then stops at the cover by distance alone (6.0 m), so measure the course carefully |
| Phrase detector | `--no-keyword`; the operator presses **Survivor said the phrase** |
| Dodging | `--no-dodge`; the throwers stand down |
| Gemini | `--no-voice`; Pupper's lines appear in Mission Control, and the personality's own voice says the call-outs if it has recordings |
| One stage only | Click that stage in Mission Control, then **Start from …**: place Pupper there first, facing along the line |

---

## Pupper Mission Control

One page, port 8078. It is the same page on the robot and in simulation.

| Part | What it shows |
|---|---|
| **Top bar** | The mission's state, the run time, distance along the line and off it, who has the legs (the mux), **Allow motion**, **Pause/Resume**, **End run**, **Start** |
| **Cue banner** | The one thing the people in the room should be doing now, and Pupper's last line. Green = your turn, red = stop or keep clear |
| **Stages** | The eight stages, with the current one marked. Click one while idle to start the run from there |
| **The course** | Top view: the open first 2 m, markers, throw window, the cover, survivor, throwers; Pupper where it believes it is; the line to the voice (violet); bearings as rays while it listens; balls in flight (simulation) |
| **Sound** | A compass with every bearing heard (solid = used, dashed = ignored because Pupper was talking or moving), the fix with the ±5° margin, and the phrase detector's microphone levels. **Click the dial to send a bearing by hand.** **Survivor said the phrase** stands in for the detector |
| **Terrain and cover** | What the front camera sees (obstacles by bearing and range, ±80°, to 2.5 m), terrain agility's state and mode, the nearest tall thing and its width, and the cover once it is seen (from 4.8 m on). Crossing rows appear only if `course.obstacles` lists something to cross |
| **Projectiles** | Balls being tracked around Pupper, the dodge node's state, armed or not, dodges so far, camera frame rates, and a link to Pupper Dodging View (port 8077) |
| **Systems** | What is alive. Robot: **Stand up**, **Relax**. Simulation: speed, a new room, the bearing error, and switches for the second call, the throws and the phrase |
| **Console** | One log, the same lines as `tools/pupper watch` in a terminal: every command in its terminal form (marked *page* or *terminal*, whichever sent it), every mission event and state change with the run time, each line Pupper said, and the output of the checks. Filters: everything, the run, commands, checks. **A prompt** takes the same words as `tools/pupper` (`start`, `pause`, `goto gauntlet`, `bearing 30`, `run log`, `help`), with suggestions as you type, Tab to complete and ↑/↓ for history. **Checks** run with one click (below) |

What the buttons can and cannot do:

- Mission Control only writes `/nav/command` and reads `/nav/telemetry`. It cannot arm the dodge node or move a leg itself; `robot/nav_node.py` does that, and it obeys the e-stop, the joystick and its own motion switch.
- **Allow motion is off when the stack starts.** Until it is on, navigation publishes no velocity, terrain agility only reports, and the dodge node stays unarmed: a full dry run with real sound.
- **Start anyway** (amber) appears when something under Systems is missing. It runs without it; the compass and the phrase button stand in for the sound module and the detector. The IMU is the one thing it will not start without.
- **Stand up** asks the lab's robot server (`/walking/activate`) to put `gemini_walk` in charge of the legs. Pupper gets to its feet: floor clear, operator at the e-stop.
- On the robot the page is open to anyone on the same network, and a click there can make Pupper walk once motion is allowed. Keep it to the team's laptop.

Keys: **Space** pauses or resumes; **/** or **Ctrl+K** goes to the console's prompt; **`** makes the console taller; **?** lists the keys and every command. There is deliberately no key for Start, Allow motion or Stand up. Every button's tooltip names its terminal command.

### From a terminal: `tools/pupper`

The same buttons and the same console, from any terminal on the Pupper or on the laptop (Python 3 only, no ROS). It talks to Mission Control's server, so whatever it sends appears in the page's console marked *terminal*, and every button pressed on the page appears in `pupper watch`.

| Command | Does |
|---|---|
| `tools/pupper` | One look: the state, the cue, Pupper's last line, sound, terrain, dodge, what is missing |
| `tools/pupper watch` | A live view: the same, redrawn twice a second, with the console scrolling under it. Ctrl+C leaves |
| `tools/pupper log` | Follow the console only |
| `tools/pupper start` · `start --anyway` · `pause` · `resume` · `end` | The run |
| `tools/pupper motion on` · `motion off` | The dry-run switch. On the robot `motion on` asks first (someone at the e-stop) |
| `tools/pupper goto gauntlet` | Start from a stage (`approach`, `second-call`, `gauntlet`, `cover`, or the state's name) |
| `tools/pupper bearing 30` · `phrase` | Stand in for the sound module (degrees, **left positive**) and the phrase detector |
| `tools/pupper stand` · `relax` | Robot only: the lab's robot server stands Pupper up or lets the legs go (`stand` asks first) |
| `tools/pupper actions` · `run log` · `run stop` | List the checks, run one (its output prints here and in the page), stop it |
| `tools/pupper sim speed 5` · `sim reset` · `sim moe 8` | Simulation only |

From the laptop: `tools/pupper --url http://<pupper>.local:8078 watch`, or `export PUPPER_URL=http://<pupper>.local:8078` once. `--yes` skips the questions (for scripts). `tools/nav_cmd.sh` still sends commands straight over ROS, for when Mission Control is not running; those do not appear in the console.

**Checks** (the console's buttons and `pupper run <name>`) are a fixed list of programs with fixed arguments, chosen by name: no shell and nothing typed reaches them. One runs at a time, with a time limit.

| Name | On | Runs |
|---|---|---|
| `log` | robot | `tail -n 60 /tmp/sar-demo/robot.log` |
| `sound_log` | robot | the partner's script's device lines and first bearings, from the robot log |
| `audio` | robot | `arecord -l`, `pactl list short sources`: is the DJI receiver there? |
| `nodes` | robot | `ros2 node list` |
| `top`, `temp`, `load` | robot (`load` both) | the busiest processes, `vcgencmd measure_temp`, `uptime` |
| `git` | both | the last five commits and what is changed in `path-navigation` |
| `update` | robot | `git pull --ff-only`, after a confirmation. Running nodes keep the old code until `robot/start_demo.sh` is run again |
| `tests`, `sim20` | laptop | this module's tests; 20 simulated runs |

## How it works

```
IDLE → PREFLIGHT → SEEK → ORIENT → CONFIRM → ZONE1 → REFIX → ZONE2 → ARRIVE → RESCUE → DONE
                    ↑________|  (fix rejected)
any active state ⇄ PAUSED (joystick, e-stop, fall, operator)        any active state → HOLD (needs a person)
```

| State | What Pupper does | Leaves when | Dodge | Terrain |
|---|---|---|---|---|
| PREFLIGHT | Says the opening line | The line has finished | off | off |
| SEEK | Stands still, publishes nothing, listens | 3 bearings agree within 8° | off | off |
| ORIENT | Turns on the spot to the fix, shortest way | Within 5° for 0.5 s | off | off |
| CONFIRM | Listens again, nose-on, and blends the two looks | A bearing within 10° (go), over 25° off (back to SEEK), or 6 s (go with the first) | off | off |
| ZONE1 | Walks the line, commanded 0.30 m/s (about 0.16 m/s on the floor); terrain agility walks it around anything in the way | Past 2.0 m | off | avoid (full only if `course.obstacles` lists something to cross) |
| REFIX | Stops, asks for the second call, redraws the line from where it stands | A bearing within 20° of where the survivor should be, or 6 s | off | report |
| ZONE2 | Walks the line, commanded 0.30 m/s (about 0.16 m/s on the floor), steering at the cover once it is seen | The phrase (past 4.0 m), the cover within 0.45 m (past 4.8 m), or 7.2 m | **armed** | report |
| ARRIVE | Stands in front of the cover; says "I'm here" every 10 s; after 30 s tells the team | The phrase, or the operator's button | armed | report |
| RESCUE | Stands. The voice does the comfort steps | Operator ends the run | off | off |

- **It decides zones and the line, never a dodge or a footstep.** Projectile dodging and terrain agility sit above it in the velocity mux (`robot/demo_mux.yaml`: joystick, dodge, terrain, navigation, Gemini) and interrupt it as designed.
- **Where Pupper is** comes from the commands the mux actually sent to the legs and the IMU's yaw (`nav/reckon.py`): there is no odometry. (With a course to cross, each obstacle would also be a landmark; the demo has none since 2026-11-27.)
- **It walks a line, not a heading** (`nav/steer.py`): after a walk-around or a dodge it steers back onto the line.
- **Bearings count only while Pupper stands still and is not talking**, and a fix is the largest group that agree, so one echo does not move it.
- **Every run is logged** to `data/runs/<date-time>.jsonl` (state, distance, offset, yaw, every bearing, every event). After a bad run that file tells a bad bearing from a bad distance estimate.

## Wiring in the sound module

The sound module is your partner's code in [`sound-localization/`](sound-localization/README.md) (his commit ca06155): a DJI Mic 3 receiver in Q mode with four transmitters at the corners of Pupper's back, and `localize_angle.py`, which prints a bearing each time it hears a sound. **His files are not edited**; a test checks they are byte for byte as he committed them. `robot/sound_node.py` starts his script as it is, reads what it prints (`robot/dji_bearing.py`), and publishes:

| Topic | What |
|---|---|
| `/sound/bearing` | `std_msgs/String`, JSON: `{"bearing": +11.2, "confidence": 1.0, "check": "ok", "fit": 0.98, "rms": 0.03, "delays": [...], "residual": 0.01}`. **`bearing` is his number unchanged: degrees, 0 = nose, positive to the RIGHT** |
| `/sound/status` | Once a second. `alive` is true only while his script runs with a four-channel device open; it also names the device and carries the last line his script printed (so an error reaches Mission Control) |

`demo.launch.py` starts it by default (`sound:=dji`; `sound:=none` for no sound module; `start_demo.sh --no-sound`). His script's settings (device, threshold, microphone spacing) are `config/demo.yaml`, section `dji`, passed to it on its command line. `topics.sound_positive: right` turns his convention into navigation's (left positive), in `robot/sound_adapter.py`, the one place it happens.

The bridge adds two checks to each bearing, made on his numbers. One microphone pair can lock onto a wrong correlation peak (it happens in quiet rooms; `SOUND_INTEGRATION.md` explains it and the fix for his script), and the four delays then do not add up: for a rectangle, CH4 = CH2 + CH3. A bearing that fails that, or whose fit magnitude is under 0.5, is published and logged but not used for the fix (`sound.min_confidence: 0.5`).

Check it, in this order, motion off:

1. **The receiver.** `arecord -l` lists it; `python3 sound-localization/test_channels.py --transmitters 4` peaks on four different channels for TX1 … TX4.
2. **The stack.** `robot/start_demo.sh`; Mission Control's Sound panel shows **Module**: the receiver's name. `/tmp/sar-demo/robot.log` has "with 4 input channel(s)".
3. `python3 tools/fake_sound.py 30` (stack running): the compass shows rays 30° to the left and Pupper reports it would turn left. (It publishes in the configured convention, so it checks navigation, not the module.)
4. **One clap** at 1 m straight ahead, then one from Pupper's left. In the log, straight ahead prints delays near `CH2 0, CH3 +32.7, CH4 +32.7` samples and a bearing near 0; left prints near `CH2 +16.8, CH3 0, CH4 +16.8` and a bearing near −90 (his convention), and the compass ray points **left**. A constant offset in one channel's delays at every angle is the radio link, not the room: see `SOUND_INTEGRATION.md`.
5. One call from the cover with Pupper on the start mark, facing away. The ray points at the cover.

Recordings can be replayed through his unchanged script on a laptop: `python3 tools/sound_replay.py calls.wav --truth 15` prints each bearing, its checks, and whether navigation would use it.

## First time on the robot

Nothing here has moved the robot. Do these in order, with the operator at the e-stop from rung 3 on. Stop at the first rung that fails.

| Rung | Do | Want |
|---|---|---|
| 1 | Terrain agility's own ladder (`terrain-agility/deploy.md`) and the dodge node's (`projectile-dodging/DEPLOY.md`) | Both done on their own first |
| 2 | `robot/start_demo.sh` (no `--motion`). Open Mission Control | Every node reports in `/tmp/sar-demo/robot.log`; Systems all green except, perhaps, the sound module |
| 3 | Sound checks 1–3 above, motion off. Then **Start** with motion off | Bearings on the compass; state goes SEEK → ORIENT and stays there (dry run); "would ask for" shows a turn the right way |
| 4 | Open floor, no obstacles, `--no-terrain --no-dodge`. Stand up, **Allow motion**, **Start**, call from 3 m | Turns to the caller, walks the line. Measure where it stops against the tape; set `reckon.forward_factor` = its present value × tape ÷ what it believed |
| 5 | The same over 6 m with a board as the cover (terrain on, in the gauntlet it only reports) | Stops about 0.3 m short of the cover; "to the cover" counts down |
| 6 | Phrase check with the real survivor behind the real cover (`keyphrase/README.md`, step 3) | 9 or 10 of 10 heard, nothing else sets it off |
| 7 | Zone 1 only, with a bag in the lane at 1 m: **End run** at the second call | Walks around the bag, back onto the line, stops for the call |
| 8 | Gauntlet only: click **Gauntlet**, **Start from Gauntlet**, dodge node unarmed first (`dodge.yaml`), then armed, then with throws | Carries on along the line after each dodge |
| 9 | The whole run without throws, then with. Then the script with everyone in place | Three complete runs in a row before demo day |

**The walking speed.** Pupper walks much slower than it is told to. Measured once on your floor on 2026-11-26 (terrain agility's `tools/walk_forward.py`): **0.80 m in 5 s from standing with 0.30 m/s commanded, about 0.53 of the commanded speed**. So `reckon.forward_factor` is 0.53, and both walking speeds (`walk.cruise`, `walk.gauntlet`) are 0.30 m/s commanded, the one speed that has a measurement; on the floor that is about 0.16 m/s. With the factor left at 1.0 Pupper would believe it had walked twice as far as it had and stop about half-way down the course. That one figure includes getting going from a standstill and is forwards only, so:

- re-measure it over 4 m at 0.30 m/s (rung 4) and put the result in `config/demo.yaml`;
- the sideways factor is taken to be the same, unmeasured;
- if you change `walk.cruise` or `walk.gauntlet`, measure the factor again at the new speed (it is not the same at every speed).

Also measure on the demo floor: the IMU's yaw drift over three minutes standing and over one run, and the bearing error at 6 m with the cover in place.

## Files

```
path-navigation/
  README.md, plan.md
  mission-control.command   double-click (Mac): Mission Control in simulation
  nav/         the mission, plain Python, no ROS
               config (every number), heading (angles, fixes), reckon (where Pupper is), steer (the line),
               events (the lines and the team's cues), mission (the state machine), board (what every other
               component last said, and the snapshot Mission Control draws)
  robot/       nav_node.py, sound_node.py + dji_bearing.py (the partner's script on ROS), sound_adapter.py,
               keyword_node.py, demo.launch.py, demo_mux.yaml, start_demo.sh, stop_demo.sh, env.sh
  sound-localization/   the partner's DJI four-microphone scripts, unchanged (his README)
  SOUND_INTEGRATION.md  how they were joined to navigation, what was measured, what to fix
  sim/         course_sim.py: the whole run in 2D
  tools/       mission_control.py + .html, pupper (the terminal client), fake_sound.py, sound_replay.py, nav_cmd.sh,
               soak_sim.py; keyword_test.py, make_keywords.py,
               get_keyword_model.sh (the phrase detector's)
  keyphrase/   the phrase detector (its own README)
  config/      demo.yaml (every number here), keyword.yaml, keywords.txt
  tests/       test_mission.py, test_node.py, test_sim.py, test_keyphrase.py, test_sound_integration.py,
               test_mission_control.py
               (+ dji_synth.py and fake_sounddevice/: synthetic calls through the partner's script)
  data/runs/   run logs (git-ignored)
```

Tests, on a laptop: `python3 -m pytest path-navigation/tests -q` (needs pytest, numpy and pyyaml; the phrase detector's tests also need `sherpa-onnx` and its model, and skip without them).

## What is verified and what is not

**Verified off the robot**

- 77 tests (1 skips without the phrase model): angles and fixes, the position estimate, line following, every state and transition of the mission with hand-made inputs, the node on a stand-in for ROS (dry run publishes no velocity and marks its events `dry_run`; the dodge node is armed only in the gauntlet), the sound adapter's conventions, the partner's script end to end on synthetic calls, Mission Control's server, console, checks and `tools/pupper`. The crossing tests keep the old bar-then-slab course explicitly.
- The whole run in simulation, 200 different rooms per row (random starting direction; the partner's sound module as it behaves on synthetic calls: one bearing per call, about 8 % echoes, and a share of calls giving nothing usable; the legs at 0.53 of commanded speed as measured and a further 8 % short that the mission does not know about; IMU drift 1.5° a minute; throws every 3 s). Seed 1, 2026-11-27:

| Sound module's margin of error | Calls with no usable bearing | Survivor calls again at 2 m | Ended in front of the cover | Sideways miss at the cover, median (worst) | First fix, median (slowest) |
|---|---|---|---|---|---|
| ±5° (as given) | 40 % (quiet-room figure) | yes | **200 of 200** | 0.11 m (0.39 m) | 22 s (147 s) |
| ±5° | 0 % | yes | 199 of 200 | 0.08 m (6.61 m, one run) | 14 s (22 s) |
| ±5° | 60 % | yes | 199 of 200 | 0.11 m (0.44 m) | 36 s (409 s) |
| ±10° | 40 % | yes | 191 of 200 | 0.17 m (4.56 m) | 26 s (158 s) |
| ±5° | 40 % | no | 200 of 200 | 0.11 m (0.48 m) | 22 s (147 s) |

  Re-run on 2026-11-27 after the obstacle course was removed (`sim/course_sim.py --runs 200 --seed 1`, with `--drop`, `--moe` and `--no-second-call` for the other rows). The cover reaches 0.6 m either side of the line, so a miss under about 0.55 m is "in front of it"; the few misses of several metres have not been examined run by run (likely a wrong fix). A simulated run now takes a median of about 2 minutes (112 s), against about 4 with the two crossings. **The second call now matters less**: with no crossings the walk to 2 m takes about 15 s instead of over two minutes, so the IMU's heading drifts far less before the gauntlet; without it the worst miss is 0.48 m instead of 0.39 m. Keep it in the script anyway: it is cheap, and the real drift is unmeasured. One more row, not in the table: if the survivor calls only every 6 s and 60 % of calls give nothing, only 178 of 200 runs end in front of the cover, and 16 never get a first fix within 7 minutes; call every 4 s.

  **One thing these runs do suggest about dodging, to be checked on the robot:** the simulated dodge is a stand-in, but it moves the same slow legs. With the legs at 0.53 of commanded speed its burst covers about half the planned distance, and the simulated hit rate rose from about 1 throw in 11 to about 4 in 10. If the real legs fall as far short in a burst, dodges will be late. The dodging work has the tool for it (`projectile-dodging/tools/step_response.py`) and has been told.

- In that simulation the real mission code drives terrain agility's real state machine and steering. That is how the stuck walk-around was found (see below).
- Mission Control was run in simulation in a browser (2026-11-27, after the redesign): `start` typed at the console's prompt goes through all eight stages to the rescue state, the console shows every command, state change and line, and the checks run from it. `tools/pupper` drove the same server from a terminal. Tests cover the console, the command echo, the fixed list of checks and the CLI (`tests/test_mission_control.py`). **None of it has been opened on the robot**, and the robot-only checks (`log`, `audio`, `nodes`, `top`, `temp`, `update`) have not been run there.

**Not verified: needs the robot**

- Everything in `robot/`: `nav_node.py` has only run against a stand-in for ROS; `demo.launch.py` and `start_demo.sh` have only been syntax-checked. In particular the launch file loads the dodging launch and swaps its mux file, which no test covers.
- Real sound: the DJI receiver on the Pupper (it was not plugged in on 2026-11-27: `arecord -l` listed no capture device), whether the four radio channels keep their timing, accuracy at 6 m beside a board in an echoing room, the threshold for a real shout, and which process gets the receiver when the phrase detector also needs it (`SOUND_INTEGRATION.md`, "Beware").
- The phrase detector on real microphones (it has only heard synthetic voices).
- Walking: the speed factor beyond its one measurement (0.80 m in 5 s), the sideways factor, whether the legs turn on the spot at the commanded rates, yaw drift over a run.
- The Pi's CPU with every node running at once. The dodging work measured the Pi at about 80 % busy with the dodge stack alone on 2026-11-26, and the voice's server at about a third of a core while idle; if the load goes higher, camera frame rates drop and dodging gets worse. Navigation is light by design (plain Python at 20 Hz, no images; telemetry is built 5 times a second and only while Mission Control is open), but what it adds on the Pi has not been measured. Watch the frame rates under Projectiles in Mission Control, and close the page if they sag.
- The simulation's stand-ins are simple. Its dodge rate and throw count say nothing about the real dodge node. Its sound is one bearing per call (as the partner's script gives), the true direction plus noise, with 40 % of calls giving nothing usable (as measured on synthetic quiet-room recordings), not a model of a room.

**Known limits**

- **The stuck walk-around is fixed at its source.** The simulation here found terrain agility stopping beside the tall obstacle ("blocked on both sides") in 2–10 % of walk-arounds; the terrain work fixed its steering the same day. Navigation keeps its own safety net: if terrain agility holds twice it steps back 0.45 m and lets it look again (up to twice) before asking for the operator, and between the low obstacle and the second call it holds the line's direction without steering back onto the line itself, which runs through the tall obstacle.
- **The crossing machinery is kept, unused.** The step-over and scale-across work stopped on 2026-11-27. Navigation still knows how to treat a crossing as a landmark, add its length from the step-over node's report and HOLD if its end is never reported (the overnight soak's fixes, `reports/overnight-2026-11-27.md`), and the tests keep it working on the old bar-then-slab course; with `course.obstacles` empty none of it runs. Stepping back never reverses into a tall obstacle just passed, and stage jumps (`goto:`) are taken only while idle: both still apply.
- **The camera on the robot is not yet calibrated** (terrain agility found it about 165 mm up and looking 11° up, not 250 mm and level). Until `terrain-agility/deploy.md` step 4.3 is done, no distance terrain agility measures on the robot is right, and where Pupper stops in front of the cover rests on that distance.
- **Distance is an estimate.** It rests on one 5-second measurement of the walking speed; expect 10–20 % error until that is re-measured over 4 m. That is why distance alone never ends the run: the phrase and the cover do.
- **A second "saved" in one mission is not spoken** by the voice (after a false stop and **Not the phrase: carry on**, the real phrase still stops Pupper, and Gemini hears the phrase itself).
- The free-space feed from terrain's scan to the dodge node (so a dodge never backs into something) is not built.

## What changed in the other components

| Component | Change | State |
|---|---|---|
| Terrain agility | `/terrain/mode` (full, avoid, report, off); the box measurement in `/terrain/state`; `advance` in `/terrain/crossing_status`; `tests/test_modes.py` | Committed (61fe385) |
| Terrain agility (its own work, the same day) | Obstacles told apart by shape, a slab that is scaled across, and the steering fix for the stuck walk-around that this module's simulation reported. Since 2026-11-27 the demo uses only the walk-around and the obstacle report (modes avoid and report) | See `terrain-agility/README.md` |
| Projectile dodging | None. Navigation calls its `/dodge/arm` service and reads `/dodge/state` and `/dodge/event` | – |
| Personality | None by this module. Its `callouts.py` is the source of Pupper's lines, and its `event_bridge.py` listens to `/nav/event` | Done in the personality work |
| Visual survivor ID, emotion detection | Archived (`archive/survivor-id`). No emotion-detection code was found anywhere in the repository | Committed (d0a574e) |
