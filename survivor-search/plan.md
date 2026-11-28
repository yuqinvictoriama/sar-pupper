# Survivor search at the cover: circle the sheet until Pupper sees the survivor

Written 2026-11-27. A plan only: **nothing here is built, simulated or tried on the robot.** It sits beside `path-navigation/` (which brings Pupper to the cover) and uses what terrain agility and the sound module already publish. Figures marked *measured* come from the files cited; everything else is an estimate to be checked.

## 1. What we want

The survivor sits or crouches behind a **cardboard sheet**, at least 1.2 m wide and 0.8 m tall, standing upright across the line at 6.4 m. Pupper cannot see them. They keep calling for help.

Today Pupper walks the line, stops in front of the sheet (`ARRIVE`: the camera lens about 0.45 m from it), says "I'm here. I can't see you. Can you hear me?", and waits for "You saved me, Pupper!".

The new part: **from that stop, Pupper walks round the sheet towards the cries, edge first, until it can see the survivor past the edge; then it faces them and says so.** If one side is closed (a wall, a box, a person), it tries the other. If it gets all the way round without seeing anyone, it stops and asks the operator.

```
 top view, metres, the line runs up the page            survivor (S) 0.3 m behind the middle

                        .  .  .  S  .  .  .                 behind the sheet
            (3) look ->  o                 <- or the other side
                        /  ==============  \                the sheet, 1.2 m wide, thin
            (2) round  o   ^            ^   o
                        \  left edge  right edge
            (1) to edge  o
                          \__ P  <- (0) ARRIVE: stopped, lens 0.45 m from the sheet
                              |
                              |  the line, from the start mark
```

- (0) **Choose a side.** Stand still. Measure the sheet's two edges with the camera, and listen to one or two calls.
- (1) **To the edge.** Turn, then walk **forwards** to a point about 0.45 m out past the chosen edge, level with the front of the sheet.
- (2) **Round the edge.** Arc round the edge, keeping 0.45 m from it.
- (3) **Look.** Stop every 0.3–0.4 m of arc: look for the survivor, listen. **Seen:** face them, say the line, wait for the phrase. **Not seen** by the back of the sheet: carry on to the other edge.

## 2. What exists today (and what does not)

| Need | What there is | Where |
|---|---|---|
| See the sheet and its edges | Terrain agility's scan: front fisheye only, no depth sensor. It learns the floor's colour, and anything not floor is an obstacle. It uses ±80° of bearing and reaches 2.5 m. Each obstacle in `/terrain/obstacles` (about 8 Hz) carries `nearest`, `bearing_deg`, `left_deg`, `right_deg`, `left_xy`, `right_xy` (body frame, m: x forward, y left), `width`, `height`, `tall`, `is_box`. A 0.8 m sheet reads as `tall` | `terrain-agility/robot/terrain_node.py` 437–449, `terrain/scan.py` |
| Range to anything | Range = camera height ÷ tan(depression). **The camera is not calibrated**: the config says 250 mm high and level; one picture suggests about 165 mm high, looking about 11° up. Every range is wrong until `terrain-agility/deploy.md` step 4.3 is done | `config/terrain.yaml` 18–19 |
| Get round something | A TangentBug-style walk-around: 0.22 m clearance, it keeps the side it chose, it remembers obstacle points for 6 s, and it **leaves as soon as the original heading is clear**. It gets past things; **nothing follows an edge or orbits** | `terrain/steering.py`, `terrain/state_machine.py` 391 |
| Where the cries come from | The partner's DJI four-mic script: one bearing per call (degrees, positive right), with a fit magnitude, plus the bridge's checks. Tested on synthetic recordings only, never on the robot | `path-navigation/robot/dji_bearing.py`, `SOUND_INTEGRATION.md` |
| Where Pupper is | Commanded velocity × 0.53 (forwards; the sideways factor is unmeasured), plus IMU yaw. No odometry. Drift unmeasured (simulations assume 1.5–8° a minute) | `path-navigation/nav/reckon.py` |
| See a person | Archived: YOLOv8s-pose on the Hailo-8L, then a "both arms up" rule. **It never ran on the Pi, ROS or the Hailo.** Only one process may hold the Hailo. Its rear-camera inputs are no longer published | `archive/survivor-id/` |
| Rear cameras | Two ELP USB cameras, 120° wide, yawed ±122.5°, looking back over the body, partly filled by it. Opened directly by the dodge processes, not republished. **Only the front fisheye looks forwards** | `projectile-dodging/plan.md` 22–46 |
| Who drives | The mux: joystick > dodge > terrain > navigation > Gemini, 500 ms timeout | `path-navigation/robot/demo_mux.yaml` |

## 3. Geometry: where Pupper has to get to

Take the survivor's nearest part (a shoulder, a knee) to be 0.3 m behind the middle of a 1.2 m sheet. Pupper's lens sees past the edge only when the line from the lens to the survivor misses the sheet.

- From in front of the sheet, a lens at distance *a* in front and *y* out to the side crosses the sheet's plane at *y* × 0.3 ÷ (0.3 + *a*) from the middle. That has to be more than 0.6 m (the half-width).
  - From the ARRIVE stop (*a* = 0.45 m, *y* = 0) nothing is visible, as intended.
  - From 0.45 m in front and 1.0 m out, the line crosses at 0.4 m: still blocked.
- **So Pupper has to come about level with the sheet's plane** (or behind it), more than 0.6 m out from the middle, before any of the survivor shows. Level with the plane and 0.95 m out (0.35 m past the edge), the survivor is about 1.0 m away, about 72° to the side of the line's direction. After a quarter-turn round the edge, they are roughly 0.5–0.8 m ahead.
- **Distance walked**, one side (estimates):
  - stop point to edge-level: about 1.1 m;
  - round the edge to see behind: about 0.5 m.
  - At the floor speed actually measured (0.30 m/s commanded, about 0.16 m/s on the floor), that is about 10 s of walking plus the turns and the stops.
- **A whole loop** round a thin 1.2 m sheet at 0.45 m from it is 2 × 1.2 + 2π × 0.45 ≈ 5.2 m, about 33 s of walking. With the turns and the stops to look and listen, expect 60–80 s, all on dead reckoning.

**Three things the geometry warns about:**

1. **A thin sheet seen edge-on almost disappears.** Beside the sheet, looking along its plane, the camera sees a strip a few millimetres wide (cardboard) on a floor-coloured background. Terrain's scan works in 5° bins, and may not report the sheet at all from there. **Pupper must not rely on seeing the sheet while it is level with it.** It should take the edge's position while it can still see the face (from the front, at step 0 and on the way to the edge), and carry it by dead reckoning for the last half metre or so. Measuring this is the first job (section 7, P0).
2. **The camera is blind beyond ±80°.** Walking sideways, the direction of travel is at 90°: blind. **So every move is turn first, then walk forwards.** The edge being circled sits at 60–90° to the side during the arc, at the edge of the view or past it. Remembered points expire after 6 s in terrain's steering, so the arc needs its own memory of where the edge is.
3. **Cardboard is light.** A brush at 0.16 m/s can push it over onto the survivor. Keep at least 0.30 m between the body's edge and the sheet. That is the 0.45 m to the lens used above, a little more than terrain's 0.22 m from obstacle to body centre.

## 4. Sound at the cover: which side, and a warning

Choosing the side to go round first:

- **From the voice.** At the ARRIVE stop the sheet spans about ±53° (its edges at `left_deg` and `right_deg`). A bearing clearly to one side of the sheet's middle (more than about 10°) says the survivor is nearer that edge: go that way.
- **From what is in the way.** If something else tall continues past one edge (a wall, a box), that side is closed: go the other way.
- **Neither:** the configured default side (left, say). The other side comes next if the first fails.

The warning, which nothing has measured yet: **behind a 0.8 m sheet at 0.5–1 m, the direct path from the mouth to the microphones is blocked.** The sound reaches them over the top of the sheet and round its edges. Expect:

- **Bearings that point at the nearer edge, or straight at the middle**, rather than at the mouth.
- **Low fit magnitudes.** Sound coming over the top arrives from well above the microphones. His fit magnitude is about cos(elevation), so at 45° it is about 0.7, and at 60° it is 0.5, the bridge's `fit_low`. Steeper than that and the bridge flags the bearing `fit` (confidence 0.3), and navigation does not use it.
- **The near field.** His script assumes a plane wave. At under 1 m from a 120 × 234 mm array that is no longer true. The synthetic tests at 2 m were fine (`test_near_field_and_a_survivor_standing_at_two_metres`); nothing nearer has been tried.

So at the cover, use a bearing **only to choose a side**, and accept `fit`-flagged bearings for that (they still have the right left/right sign more often than not; to be checked). Never steer by sound at the cover. And, as everywhere else, **listen only while standing still and silent**: footfall and motor noise reach the microphones.

## 5. "Seeing the survivor": three ways, cheapest first

| | How | Cost | Risks |
|---|---|---|---|
| **A. Something tall behind the plane** | From terrain's obstacle list: a `tall` obstacle that is not the sheet and lies behind the sheet's plane, roughly towards the cries | No new code on the camera, no CPU | It cannot tell a person from a box or a wall. A survivor in floor-coloured clothes may not stand out. Depends on the camera being calibrated |
| **B. A colour marker** | The survivor wears something of one distinctive colour (a bright orange or magenta vest; **not neon green**, which is the balls, and not the floor's colour). A colour-blob check on the fisheye frame, only while searching | A small new detector, a few ms a frame at reduced size, only during the search | It is a demo aid, not person detection: say so in the presentation. Calibrate in the room's light, as the balls are |
| **C. Person detection** | Revive `archive/survivor-id/robot/hailo_pose.py`: the wide pass only, on the front fisheye only, only during the search | The Hailo (one process only: check nothing else holds it), and a ROS wrapper. **It has never run on the Pi** | At 0.5–1 m from a lens 165 mm off the floor, a person is a pair of legs or a torso, heavily foreshortened. The archive notes that nothing locked inside 2 m. A person half hidden by the sheet is a partial body. Untested |

**Recommendation: A to steer the search, B to decide "seen".** C comes after, if there is time, as the honest version of B. A alone would stop at the first box behind the sheet. B is the one that can be made reliable for a demo in the time left.

The CPU is already tight: four cores full with the full stack, and the dodge stack alone about 80 % busy (`path-navigation/README.md`, `projectile-dodging/plan.md`). The dodge is not armed during the search (the throwers stop at "I'm here"). B and C run only while searching.

## 6. The behaviour

### Where the code lives

A new folder, `survivor-search/`, holds the search as plain Python with no ROS, like `path-navigation/nav/`:

- `search/geometry.py`: the sheet's edges, waypoints, line of sight.
- `search/search.py`: the state machine below.
- `search/marker.py`: option B, the colour check.
- `tests/` and a simulator.

Navigation calls it from a new mission state, **SEARCH**, so one node still owns `/nav_cmd_vel`. The e-stop, the joystick pause, **Allow motion** and the HOLD rules then apply unchanged, and nothing new is added to the mux. Changes to `path-navigation` are kept to that hook: the new state, its cue, and a stage in Mission Control.

### Modes while searching

- **Terrain in `report`.** Terrain's own walk-around would try to take Pupper past the sheet and back to the old heading, which fights the search. In `report` it only publishes the obstacles and the scan.
- **Dodge disarmed.**
- **Speed:** 0.30 m/s commanded (the one speed with a measurement; the legs barely move below 0.20). Turning on the spot within navigation's existing limits. No raised speed caps or gains.

### States (inside SEARCH)

| State | What Pupper does | Leaves when |
|---|---|---|
| **MEASURE** | Stands still for 1.5 s. Averages the sheet's `left_xy` and `right_xy` from `/terrain/obstacles` and fixes the sheet's position in its own frame. That frame is anchored to the sheet, not the start mark. Listens for up to two calls. Chooses the side | Edges measured (width 0.8–2.0 m), or 5 s: then HOLD, "I can't make out the cover" |
| **TO_EDGE** | Turns towards the waypoint 0.45 m out past the chosen edge, level with the front of the sheet, then walks forwards to it. While the sheet's face is in view (within ±80°), it re-measures the edge each scan | At the waypoint (by dead reckoning, corrected by the last edge sighting) |
| **ROUND** | Arcs round the edge at 0.45 m: an edge-following controller that holds the remembered edge point at a fixed range, abeam. This is new; terrain agility has nothing like it. It stops every 0.3–0.4 m of arc | Each stop: to LOOK |
| **LOOK** | Stands still for 1.5 s. Runs the "seen" check (B), checks terrain for something tall behind the plane (A), and listens | Seen: FOUND. Not seen and the back middle is not yet reached: ROUND. Back middle reached: on to the other edge. Back where it started: GIVE_UP |
| **FOUND** | Turns to face the survivor and stops 0.6 m or more from them. Says "I can see you now. I'm right here." Then it waits for the phrase as ARRIVE does today | The phrase: RESCUE |
| **GIVE_UP** | Stops. Says "I can't find a way to you. Team, please help." | The operator |

**Safety, whatever the state:**

- **Stop** if any return in `/terrain/scan` within ±40° of the direction of travel is nearer than 0.25 m.
- **HOLD** if the search has taken over 90 s, or has used more than one loop.
- **HOLD** if the scan goes stale (terrain silent for over 1 s), the same rule as the gauntlet.

### Open decisions (the owner's)

1. **What counts as "seen":** option B (a marker the survivor wears), or wait for C.
2. **Is the phrase still the trigger for RESCUE?** Today, yes. Alternatively FOUND could lead straight to the comfort steps. Keeping the phrase keeps the current script and the keyphrase work.
3. **When the search starts:** straight after "I'm here", or only if no phrase comes within, say, 10 s. The first shows the search every time; the second keeps today's run as the fallback.
4. **How the sheet stands up:** feet or a box behind it add depth and change what terrain sees. A sheet with nothing behind it is the simplest for the camera, but is easier to knock over.

## 7. Phases

### P0: measure, motion off (on the robot, read-only, nobody near the legs)

1. **Camera calibration** (`terrain-agility/deploy.md` step 4.3). Every range in this plan rests on it.
2. **The sheet in the scan.** With Pupper standing still, place the sheet face-on at 0.45, 1.0 and 2.0 m, then at 45°, then edge-on beside the robot. Record `/terrain/obstacles`. Want:
   - face-on: both edges within 0.05 m;
   - edge-on: what is reported, if anything.
3. **Sound behind the sheet.** A person calls from behind the sheet, with the robot at 0.5, 1.0 and 2.0 m, straight on and offset 0.5 m. Record with `arecord` and replay through `tools/sound_replay.py`. Want: how often the left/right sign is right, and how many bearings the bridge flags `fit`.
4. **Turning on the spot:** the commanded rate against the IMU's, at the rates the search would use.
5. **Option B's colour**, in the demo room's light, from 0.5–1.5 m, partly hidden by the sheet.

### P1: simulation

Add to `path-navigation/sim/course_sim.py` (or a copy in `survivor-search/sim/`):

- a thin sheet;
- a survivor behind it;
- a line-of-sight test for "seen";
- the scan, cast as it already is, with the sheet dropping out of the scan when seen edge-on, as P0 measures;
- sound at the cover as P0 measures it, sign errors included.

Run the real search code in 200 rooms (sheet skewed ±15°, survivor 0.2–0.5 m behind and ±0.4 m off centre, one side sometimes closed). Want: found in 190 of 200, never closer than 0.30 m to the sheet.

### P2: on the robot, sheet only

Operator at the e-stop, **Allow motion** on, nobody behind the sheet. Start from the ARRIVE stop. Ten runs each side. Want: reaches the far side of the edge, and never touches the sheet. Measure where it ends against tape.

### P3: on the robot, with the survivor

Ten runs. Want:

- found in 8 of 10 within 60 s;
- the sheet never touched;
- the survivor never approached closer than 0.6 m.

Only then into the demo script.

## 8. Things to beware of

- **None of the inputs has been checked on the robot**: not the scan of a thin sheet, not the sound behind it, not the turn rates, not the camera height.
- **Dead reckoning drifts:** up to about 5 m of walking and several turns. Re-anchor on the sheet whenever its face is in view; never trust the start-mark frame at the cover.
- **The survivor's feet or a bag may show early**, before the edge. That is fine for "seen", and is not a reason to walk closer.
- **The survivor moving** while Pupper circles can make A and B flicker. Ask for two LOOKs in a row before FOUND.
- **People standing beside the sheet** (helpers, the audience) are tall obstacles and close a side. Keep 1.5 m clear around the cover, as the course already asks for the line.
- **The Hailo, if C is used:** one process only, and nothing on the Pi has ever opened it for this.
- **Pupper's own voice:** do not listen while it is speaking, the same rule as SEEK.
