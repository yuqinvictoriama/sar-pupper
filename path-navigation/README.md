# Path navigation: the demo, start to finish

This is the module that runs the demo. Pupper listens for a cry for help, turns to it, steps over a low bar, scales across a slab, asks the survivor to call again, walks through thrown balls, stops in front of the survivor's cover, hears "You saved me, Pupper!" and comforts them. `plan.md` is the why. This file is how to run it, and **"Demo instructions" below is the one place the team's set-up and script live.**

**Status, 2026-11-27.** Built and tested off the robot only: 72 tests (1 skips without the phrase model), and a simulation of the whole run (see "What is verified and what is not" for how often it ends in front of the cover). **None of it has run on the robot.** The sound-direction module is now your partner's code (`sound-localization/`, his commit ca06155), run unchanged by `robot/sound_node.py`; it has been tested on synthetic four-channel recordings only, never with the DJI receiver on the Pupper. [`SOUND_INTEGRATION.md`](SOUND_INTEGRATION.md) has how it was joined, what was measured and what to fix. "What is verified and what is not" near the end says exactly which claims rest on what.

## Start here

**On a laptop, no robot (the whole run in simulation):**

```bash
python3 path-navigation/tools/mission_control.py --sim --open
```

On a Mac you can instead double-click `path-navigation/mission-control.command`. It needs Python 3 with numpy and nothing else. Press **Start**.

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
| **Step-over obstacle** | **A flat bar**: a length of "1×4" timber, **19 mm tall × 89 mm deep, cut 600 mm long**, flat side down. Anything 15–25 mm tall, 76–102 mm deep and at least 400 mm long works. It must not slide or rock: tape it down. Not a cylinder or round bar (see the note under the course table). `terrain-agility/deploy.md` section 1 has the finish its camera needs | 1 |
| **Scale-across obstacle** | **The slab of wood: 15 mm tall, 565 mm wide, 380 mm deep**, white top, raw board edge. Laid with the **565 mm side across the line** and square to it; Pupper crosses it with its feet on top. It must not slide: tape it down or give it a non-slip underside | 1 |
| **Cover** (hides the survivor) | A board or large box, **at least 1.2 m wide and 0.8 m tall**, matte, not floor-coloured, not green. It must stand on its own | 1 |
| **Balls** | The neon green foam pickleballs, 74 mm | 6–8 |
| **Markers** | Flat tape, 30 cm strips: the start mark, three pairs of side markers, two thrower spots. Not cones (a cone is an obstacle) | 1 roll |
| **Tape measure** | 8 m or longer, and a ruler for the low obstacle | 1 |
| **Sound module** | DJI Mic 3 receiver in **Q (Quadraphonic)** mode on the Pupper's USB, and four transmitters fixed at the corners of a **120 mm × 234 mm** rectangle on Pupper's back: TX1 front-left, TX2 front-right, TX3 back-left, TX4 back-right, all at the same height. All charged | 1 + 4 |
| **Gamepad** | The e-stop (right-stick press). Charged | 1 |
| **Battery** | Charged, plus a spare. A run is 4 to 6 minutes, over two of them on the obstacles | 2 |
| **Laptop** | On the robot's network, showing Mission Control | 1 |
| **Floor** | About **8 m long and 7 m wide**, flat, matte, evenly lit | – |

### The course

Lay the tape measure from the start mark to where the cover will stand. That is **the line**. Every distance is read off that tape, from the start mark. **Take the tape measure away before the run**: it is an obstacle too.

```
 metres  0    0.45 0.54     1.15      1.53   2.0   3.0                 5.2      6.0   6.4
         |------|=|----------|==========|-----|-----|===================|--------|-----|▓▓ survivor
       start   low bar          slab        second first-throw      last-throw  Pupper cover
        mark (step over)   (scale across)    call    markers          markers    stops
         |<------- first 2 m: obstacles ------->|<------------ last 4 m: dodging ------------>|
                                              Thrower L: 3 m left of the line, level with 4.0 m
                                              Thrower R: 3 m right of the line, level with 4.0 m
```

| Place | At | How |
|---|---|---|
| **Start mark** | 0 m | A small cross Pupper stands over. It may face any way; facing away from the survivor shows off the turn |
| **Low bar**, near edge | 0.45 m (far edge at 0.54 m) | Across the line, square to it, centred on it. Pupper walks 9 cm, stops 36 cm short of it, measures, and steps over: it ends with its body centre about 0.67 m from the start mark |
| **Slab**, near edge | 1.15 m (far edge at 1.53 m) | 565 mm side across the line, square to it, centred on it. That leaves 0.61 m of bare floor after the bar (a crossing is refused with less than 0.35 m; keep 0.6 m: in the overnight soak, runs stalled between the two when the gap was under about 0.45 m, and 0.5 m or more was fine). Pupper stops 36 cm short, measures, and scales across: it ends with its body centre about 1.66 m from the start mark. Keep 0.35 m or more of bare floor beyond the slab |
| **Second-call markers** | 2.00 m | One each side, **1 m off the line**. Pupper stops for the second call between here and about 2.8 m |
| **First-throw markers** | 3.00 m | One each side, 1 m off the line |
| **Last-throw markers** | 5.20 m | One each side, 1 m off the line |
| **Cover**, near face | 6.40 m | Centred on the line, square to it. Pupper stops about 0.3 m short of it |
| **Survivor** | Behind the middle of the cover | Sitting or crouching, out of sight, head within half a metre of the cover |
| **Throwers** | 3 m either side of the line, level with 4.0 m | Mark the two spots. They do not move during the run |

These numbers are `course:` in `config/demo.yaml`. If you move anything, change it there too and run `python3 sim/course_sim.py --runs 100` before trusting the new layout.

Rules for the floor:

- **Nothing across the line.** No tape, no cables, no marker in Pupper's path: a stripe on the floor looks like a low obstacle to the camera. All markers sit 1 m to the side.
- **1 m clear either side of the line** for things, **1.5 m** for people. Thrower spots are 3 m out.
- **No neon green or yellow-green** near the course, clothes included. The balls are the only green thing.
- **Same floor and lighting as the calibration.** If the room or the lights change, redo the floor, obstacle and ball colours.

**Why a flat bar and not a cylinder (decided 2026-11-26).** The first obstacle is a flat bar, **stepped over**: each foot crosses it in one stride, touching the bar's top once on the way. A cylinder was considered and dropped:

- **A round bar cannot be stepped over.** Terrain agility plans every footfall, and for a 50 mm round bar there is no plan (the stride needed is about 160 mm against about 110 mm available, and a round top is no foothold). Its notes: `terrain-agility/deploy.md`, 1c.
- **It is not reliably walked around either.** In this module's simulation, with terrain agility's real decision and steering code, about half the runs stalled in front of a 700 mm cylinder, because Pupper first walks up to 36 cm from it to measure it.
- **Keep round things off the course.** If one ends up in the lane, Pupper will try to go around it and may stop there.

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
| 4 | Walks to the low bar, stops, squares up, measures it, and steps over it. A little under a minute, all of it very slow | "Low obstacle. Stepping over. Still coming." | Silence. **No throws.** Nobody near the line |
| 5 | Waits a few seconds for its walking legs to come back, walks to the slab, stops, measures it, and scales across it with its feet on top. About 1½ minutes | "Low obstacle. Stepping over. Still coming." (the same line again) | Silence. No throws |
| 6 | Walks on from the slab and stops at the 2 m markers | **"Call out again, so I can find you."** | **Second call.** Survivor waits for the sentence to end, then calls **"Help! Help me!"** twice, 2 s apart, then stays quiet. Everyone else silent |
| 7 | Turns a little toward the voice and walks on, slowly | – | Throwers get ready |
| 8 | Walks from the 3.0 m markers to the 5.2 m markers, dodging | "Something flew past me. I'm okay. Still coming." after a dodge | **Throw window.** Throwers follow the throwing rules below |
| 9 | Stops in front of the cover | **"I'm here. I can't see you. Can you hear me?"** | Throwers stop. Silence |
| 10 | Holds still, listening | – | **The phrase.** Survivor says **"You saved me, Pupper!"** in one go, clearly, at speaking-up volume, toward Pupper. If Pupper does not answer, again every 3 s |
| 11 | Stays beside the cover: the rescue and recuperate state | "I've got you. You're not alone now. I'm staying right here." Then "You did the hard part…", "Breathe with me…", then its questions | Survivor answers: name, hurt or not, can walk or not, anyone else. The audience may react now |
| 12 | Reports to the team | "Team: survivor found, behind cover…" | Operator presses **End run** |

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
- **Never during the two crossings.** Pupper cannot dodge while it is on an obstacle, and the dodge reflex is switched off until the second call is done.
- **Stop at "I'm here".**
- Balls that land stay where they are until the run ends.

### Before every run

- [ ] Battery charged; gamepad on; e-stop tested once today.
- [ ] Course measured from the start mark; low obstacle square; tape measure removed; no balls left in the lane.
- [ ] Colours calibrated in this room and this light (floor, obstacles, ball).
- [ ] One warm-up crossing of each obstacle done today, so the crossings plan quickly.
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
| Pupper walks around the bar or the slab instead of crossing it | Let it. The run carries on. (It does that if the slab is skewed more than about 20°, turned end-on, or has less than 35 cm of bare floor beyond it) |
| Pupper stops mid-crossing and settles onto the floor | That is a safe abort. Mission Control says "Pupper has stopped". Stand it up (**Stand up**), then **Resume** |
| Mission Control says "the crossing has not reported its end" | The step-over node went quiet mid-crossing (well past the crossing's planned time, with no progress for 30 s). Pupper waits for the operator. Check terrain agility in `/tmp/sar-demo/robot.log`; if Pupper is clear of the obstacle, **Resume** |
| Pupper stops beside an obstacle, steps back, tries again | That is its own retry. If it stops for good, nudge it clear with the joystick; it carries on when you let go |
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
| Step-over | `robot/start_demo.sh --no-terrain` on open floor, or set terrain agility to walk around everything (`fsm.mode: avoid` in `terrain.yaml`) |
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
| **The course** | Top view: obstacles, markers, throw window, cover, survivor, throwers; Pupper where it believes it is; the line to the voice (violet); bearings as rays while it listens; balls in flight (simulation) |
| **Sound** | A compass with every bearing heard (solid = used, dashed = ignored because Pupper was talking or moving), the fix with the ±5° margin, and the phrase detector's microphone levels. **Click the dial to send a bearing by hand.** **Survivor said the phrase** stands in for the detector |
| **Terrain** | What the front camera sees (obstacles by bearing and range), terrain agility's state and mode, the low obstacle as measured, the crossing's progress |
| **Projectiles** | Balls being tracked around Pupper, the dodge node's state, armed or not, dodges so far, camera frame rates, and a link to Pupper Dodging View (port 8077) |
| **Systems** | What is alive. Robot: **Stand up**, **Relax**. Simulation: speed, a new room, the bearing error, and switches for the second call, the throws and the phrase |
| **What happened** | Every mission event with its time, and each line Pupper said |

What the buttons can and cannot do:

- Mission Control only writes `/nav/command` and reads `/nav/telemetry`. It cannot arm the dodge node or move a leg itself; `robot/nav_node.py` does that, and it obeys the e-stop, the joystick and its own motion switch.
- **Allow motion is off when the stack starts.** Until it is on, navigation publishes no velocity, terrain agility only reports, and the dodge node stays unarmed: a full dry run with real sound.
- **Start anyway** (amber) appears when something under Systems is missing. It runs without it; the compass and the phrase button stand in for the sound module and the detector. The IMU is the one thing it will not start without.
- **Stand up** asks the lab's robot server (`/walking/activate`) to put `gemini_walk` in charge of the legs. Pupper gets to its feet: floor clear, operator at the e-stop.
- On the robot the page is open to anyone on the same network, and a click there can make Pupper walk once motion is allowed. Keep it to the team's laptop.

The same commands from a terminal on the Pupper: `bash tools/nav_cmd.sh start | pause | resume | end | phrase | motion_on | bearing:30 | goto:ZONE2 | watch`.

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
| ZONE1 | Walks the line, commanded 0.30 m/s (about 0.16 m/s on the floor); terrain agility steps over the low bar, then scales across the slab (or walks around either, if it refuses it) | Past 2.0 m with every obstacle on the course's list behind it | off | full, then avoid once the last obstacle is passed |
| REFIX | Stops, asks for the second call, redraws the line from where it stands | A bearing within 20° of where the survivor should be, or 6 s | off | report |
| ZONE2 | Walks the line, commanded 0.30 m/s (about 0.16 m/s on the floor), steering at the cover once it is seen | The phrase (past 4.0 m), the cover within 0.45 m (past 4.8 m), or 7.2 m | **armed** | report |
| ARRIVE | Stands in front of the cover; says "I'm here" every 10 s; after 30 s tells the team | The phrase, or the operator's button | armed | report |
| RESCUE | Stands. The voice does the comfort steps | Operator ends the run | off | off |

- **It decides zones and the line, never a dodge or a footstep.** Projectile dodging and terrain agility sit above it in the velocity mux (`robot/demo_mux.yaml`: joystick, dodge, terrain, navigation, Gemini) and interrupt it as designed.
- **Where Pupper is** comes from the commands the mux actually sent to the legs and the IMU's yaw (`nav/reckon.py`): there is no odometry. The low obstacle is a landmark (terrain agility measures the distance to it), and the crossing's length is added from the step-over node's own report.
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
| 7 | Zone 1 only: the low bar and the slab, **End run** at the second call. First each obstacle alone, then both | Steps over, scales across, stops for the call |
| 8 | Gauntlet only: click **Gauntlet**, **Start from Gauntlet**, dodge node unarmed first (`dodge.yaml`), then armed, then with throws | Carries on along the line after each dodge |
| 9 | The whole run without throws, then with. Then the script with everyone in place | Three complete runs in a row before demo day |

**The walking speed.** Pupper walks much slower than it is told to. Measured once on your floor on 2026-11-26 (terrain agility's `tools/walk_forward.py`): **0.80 m in 5 s from standing with 0.30 m/s commanded, about 0.53 of the commanded speed**. So `reckon.forward_factor` is 0.53, and both walking speeds (`walk.cruise`, `walk.gauntlet`) are 0.30 m/s commanded, the one speed that has a measurement; on the floor that is about 0.16 m/s. With the factor left at 1.0 Pupper would believe it had walked twice as far as it had and stop about half-way down the course. That one figure includes getting going from a standstill and is forwards only, so:

- re-measure it over 4 m at 0.30 m/s (rung 4) and put the result in `config/demo.yaml`;
- the sideways factor is taken to be the same, unmeasured;
- if you change `walk.cruise` or `walk.gauntlet`, measure the factor again at the new speed (it is not the same at every speed).

Also measure on the demo floor: the IMU's yaw drift over three minutes standing and across one crossing, and the bearing error at 6 m with the cover in place.

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
  tools/       mission_control.py + .html, fake_sound.py, sound_replay.py, nav_cmd.sh; keyword_test.py, make_keywords.py,
               get_keyword_model.sh (the phrase detector's)
  keyphrase/   the phrase detector (its own README)
  config/      demo.yaml (every number here), keyword.yaml, keywords.txt
  tests/       test_mission.py, test_node.py, test_sim.py, test_keyphrase.py, test_sound_integration.py
               (+ dji_synth.py and fake_sounddevice/: synthetic calls through the partner's script)
  data/runs/   run logs (git-ignored)
```

Tests, on a laptop: `python3 -m pytest path-navigation/tests -q` (needs pytest, numpy and pyyaml; the phrase detector's tests also need `sherpa-onnx` and its model, and skip without them).

## What is verified and what is not

**Verified off the robot**

- 49 tests: angles and fixes, the position estimate, line following, every state and transition of the mission with hand-made inputs, the node on a stand-in for ROS (dry run publishes no velocity and marks its events `dry_run`; the dodge node is armed only in the gauntlet), the sound adapter's conventions.
- The whole run in simulation, 200 different rooms per row (random starting direction; the partner's sound module as it behaves on synthetic calls: one bearing per call, about 8 % echoes, and a share of calls giving nothing usable; the legs at 0.53 of commanded speed as measured and a further 8 % short that the mission does not know about; IMU drift 1.5° a minute; throws every 3 s). Seed 1, 2026-11-27:

| Sound module's margin of error | Calls with no usable bearing | Survivor calls again at 2 m | Ended in front of the cover | Sideways miss at the cover, median (worst) | First fix, median (slowest) |
|---|---|---|---|---|---|
| ±5° (as given) | 40 % (quiet-room figure) | yes | **199 of 200** | 0.13 m (0.56 m) | 22 s (147 s) |
| ±5° | 0 % | yes | 199 of 200 | 0.08 m (6.58 m, one run) | 14 s (22 s) |
| ±5° | 60 % | yes | 181 of 200 | 0.17 m (0.66 m) | 36 s (409 s) |
| ±10° | 40 % | yes | 182 of 200 | 0.19 m (4.07 m) | 26 s (158 s) |
| ±5° | 40 % | no | 188 of 200 | 0.35 m (0.99 m) | 22 s (147 s) |

  The cover reaches 0.6 m either side of the line, so a miss under about 0.55 m is "in front of it"; most runs that failed stopped at the cover but past its edge, and the few misses of several metres have not been examined run by run (likely a wrong fix). The course is the one in "Demo instructions": every run stepped over the bar and scaled across the slab. **The second call matters more on this course than on a shorter one**: two crossings take over two minutes, the IMU's heading drifts all that time, and the second call is what corrects it. A simulated run takes about 4 minutes, 52 s of it on the bar and 81 s on the slab.

  **One thing these runs do suggest about dodging, to be checked on the robot:** the simulated dodge is a stand-in, but it moves the same slow legs. With the legs at 0.53 of commanded speed its burst covers about half the planned distance, and the simulated hit rate rose from about 1 throw in 11 to about 4 in 10. If the real legs fall as far short in a burst, dodges will be late. The dodging work has the tool for it (`projectile-dodging/tools/step_response.py`) and has been told.

- In that simulation the real mission code drives terrain agility's real state machine and steering. That is how the stuck walk-around was found (see below).
- Mission Control was run in simulation in a browser: one click on Start goes through all eight stages to the rescue state.

**Not verified: needs the robot**

- Everything in `robot/`: `nav_node.py` has only run against a stand-in for ROS; `demo.launch.py` and `start_demo.sh` have only been syntax-checked. In particular the launch file loads the dodging launch and swaps its mux file, which no test covers.
- Real sound: the DJI receiver on the Pupper (it was not plugged in on 2026-11-27: `arecord -l` listed no capture device), whether the four radio channels keep their timing, accuracy at 6 m beside a board in an echoing room, the threshold for a real shout, and which process gets the receiver when the phrase detector also needs it (`SOUND_INTEGRATION.md`, "Beware").
- The phrase detector on real microphones (it has only heard synthetic voices).
- Walking: the speed factor beyond its one measurement (0.80 m in 5 s), the sideways factor, whether the legs turn on the spot at the commanded rates, yaw drift across the two crossings.
- The Pi's CPU with every node running at once. The dodging work measured the Pi at about 80 % busy with the dodge stack alone on 2026-11-26, and the voice's server at about a third of a core while idle; if the load goes higher, camera frame rates drop and dodging gets worse. Navigation is light by design (plain Python at 20 Hz, no images; telemetry is built 5 times a second and only while Mission Control is open), but what it adds on the Pi has not been measured. Watch the frame rates under Projectiles in Mission Control, and close the page if they sag.
- The simulation's stand-ins are simple. Its dodge rate and throw count say nothing about the real dodge node. Its sound is one bearing per call (as the partner's script gives), the true direction plus noise, with 40 % of calls giving nothing usable (as measured on synthetic quiet-room recordings), not a model of a room.

**Known limits**

- **The stuck walk-around is fixed at its source.** The simulation here found terrain agility stopping beside the tall obstacle ("blocked on both sides") in 2–10 % of walk-arounds; the terrain work fixed its steering the same day. Navigation keeps its own safety net: if terrain agility holds twice it steps back 0.45 m and lets it look again (up to twice) before asking for the operator, and between the low obstacle and the second call it holds the line's direction without steering back onto the line itself, which runs through the tall obstacle.
- **The course numbers depend on the obstacles.** They are laid out for a 19 × 89 mm bar (stepped over: 0.58 m of travel, 52 s) and the slab (scaled across: 0.87 m, 81 s), as terrain agility measured them in its physics simulation. Navigation reads each crossing's length and end from the step-over node, so different obstacles need only `course.obstacles` changed, and a run of `sim/course_sim.py --runs 100` to check the spacing.
- **Lost terrain messages and blind reversing** (from the overnight soak, `reports/overnight-2026-11-27.md`; fixed 2026-11-27, simulation only). A crossing whose end is never reported now HOLDs, after max(1.5 × its planned time, planned time + 30 s), pushed out 30 s by each progress report, instead of waiting for ever. A crossing whose start was missed is still counted. Stepping back never reverses into an obstacle behind that is too tall to cross (0.08 m or more): with less than 0.10 m of room it does not reverse at all and only asks terrain agility to go around. Stage jumps (`goto:`) are taken only while idle. The cost, in a 400-course paired soak: one course (low bar, carton, slab), where the old blind reversal got away with it because Pupper was to the carton's side, now ends in HOLD. In 600 runs with faults injected, 17 runs that had reached the cover only because an operator jumped a stage from HOLD or PAUSED no longer do: from there, **End run** and start again from the stage.
- **Known issue: a walk-around just before a crossing.** After walking around a carton or round bar, Pupper keeps the line's direction (without steering back onto it) past every nearby obstacle, so if a low obstacle follows closely it can pass beside it without crossing, and terrain agility's alignment can then push it into the corner of the next one. Limiting this to the obstacle walked around made it worse overall in the soak (24 runs lost, 2 won of 1000), so it stays as it is. The demo's bar-then-slab course has no walk-around and is unaffected.
- **Two crossings in a row have never been run on the robot.** The terrain work ran bar-then-slab once in its physics simulation and once from rendered camera frames (2026-11-26); this module's simulation runs it every time, with the crossing itself as a stand-in.
- **The camera on the robot is not yet calibrated** (terrain agility found it about 165 mm up and looking 11° up, not 250 mm and level). Until `terrain-agility/deploy.md` step 4.3 is done, no distance terrain agility measures on the robot is right, and navigation's landmarks rest on those distances.
- **Terrain agility's slab crossing drifts** up to 45 mm sideways and 3.6° in yaw in its own simulation. The second call, straight after the slab, redraws the line and absorbs it.
- **Distance is an estimate.** It rests on one 5-second measurement of the walking speed; expect 10–20 % error until that is re-measured over 4 m. That is why distance alone never ends the run: the phrase and the cover do.
- **A second "saved" in one mission is not spoken** by the voice (after a false stop and **Not the phrase: carry on**, the real phrase still stops Pupper, and Gemini hears the phrase itself).
- The free-space feed from terrain's scan to the dodge node (so a dodge never backs onto the slab) is not built. The throw window starts well past the last obstacle for that reason.

## What changed in the other components

| Component | Change | State |
|---|---|---|
| Terrain agility | `/terrain/mode` (full, avoid, report, off); the box measurement in `/terrain/state`; `advance` in `/terrain/crossing_status`; `tests/test_modes.py` | Committed (61fe385) |
| Terrain agility (its own work, the same day) | Obstacles told apart by shape, a slab that is scaled across, and the steering fix for the stuck walk-around that this module's simulation reported | In its working tree on 2026-11-26; see `terrain-agility/README.md` |
| Projectile dodging | None. Navigation calls its `/dodge/arm` service and reads `/dodge/state` and `/dodge/event` | – |
| Personality | None by this module. Its `callouts.py` is the source of Pupper's lines, and its `event_bridge.py` listens to `/nav/event` | Done in the personality work |
| Visual survivor ID, emotion detection | Archived (`archive/survivor-id`). No emotion-detection code was found anywhere in the repository | Committed (d0a574e) |
