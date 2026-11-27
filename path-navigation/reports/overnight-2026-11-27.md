# Overnight soak of the mission, 2026-11-26/27

**Everything here is simulation.** None of it ran on the robot, with the partner's sound module, or with terrain agility's real perception. The runs drive `nav/mission.py` (the real one) through `sim/course_sim.py` (the real terrain-agility state machine and steering, against simulated sound, walking, crossings and dodging). The findings are about the logic that strings the components together, nothing more.

## What was run

| | Runs | Seeds |
|---|---|---|
| `pytest tests` | 49 passed, 1 skipped at the start; 53 passed, 1 skipped with `tests/test_mission_control.py` | |
| Soak, no faults (randomised course only) | 1000 | 0–999 |
| Soak, with faults | 4000 | 10000–13999 |
| Paired soak, current mission vs proposed fixes 1–4 (same terrain code, same seeds) | 1000 clean + 1500 faulted, each | 0–999, 20000–21499 |
| Trial of a fifth fix (rejected, see below) | 1000 clean + 3500 faulted | |

Each failure was re-run with faults removed one at a time to find the smallest set that still shows it. That roughly doubled the number of runs above.

The harness is `tools/soak_sim.py`. Repro: `python3 tools/soak_sim.py --repro SEED [--clean] [--keep fault1,fault2]`. It prints every state change, the events and the outcome. For each seed it draws:

- **The course:** the bar and the slab, plus a carton and/or a round bar a third of the time each, in any order. The first is 0.40–0.60 m from the start, then gaps of 0.35–0.75 m. `refix_at` and `throw_from` move back if the obstacles run long.
- **The room:** margin of error 3, 5 or 8 degrees; second call 85 % of the time; phrase said 90 %; throws 80 %.
- **The faults**, in the faulted runs.

### The faults

- Sound: late, never, noisy, sign flipped, zero offset, bearing delivered late.
- `/terrain/state` silent (dropout) or frozen (stall); `/terrain/obstacles` silent.
- `crossing_status` "started", "progress" or "done" lost; "advance" missing.
- Operator: pause, resume, abort, end, start!, phrase, goto:<stage>, motion off/on, joystick nudge, at random moments.
- Odometry: speed factor 0.7–1.25, IMU drift up to 8 degrees a minute, walk lag.
- The IMU or the walking controller going quiet.

### What it flags

Exceptions, illegal state changes (every `_go` is recorded and checked against the mission's diagram), resumes to the wrong state, deadlocks (90 s of sim time in a moving state with no progress and no HOLD), false finishes, velocity wanted while stopped, reflex modes in the wrong state, the body passing through an obstacle, and snapshots that are not strict JSON.

## Mission Control `--sim`

Started headless, the page (42 kB) and `/api/state` were served, and a run went from Start to RESCUE at speed 20. Bad commands were refused (400) and a non-JSON body got 415.

Three faults, fixed in `tools/mission_control.py`:

- A non-number speed or option (`{"speed":"abc"}`) or a JSON body that is not an object raised inside the handler, and the connection was dropped with no reply. Now they get a 400.
- If `sim.step()` ever raised, the simulation thread died silently and the page froze. Now the error is printed, shown as `server.error` in the state, and cleared by a reset.
- `bool("false")` is True, so a switch sent as text was always on. The page sends real booleans, so this was latent.

Also, a taken port now gives one line instead of a traceback. That is how I found 8078 already in use by another Mission Control (`--sim`, started by someone else about 11 minutes before).

**Note:** before I saw that, my probe sent speed 20, motion_on and start! to that running instance. I set its speed back to 2. Its simulated run had gone on to RESCUE. All later checks used port 8079, and nothing of mine is left running.

## Failure classes

Counts below are from the 4000 faulted runs unless marked "clean".

### 1. Deadlock: a lost crossing "done" (121 runs, every one minimal to `crossing_drop_done`)

If the step-over node's `done` never arrives (it crashed, or the message was lost), `crossing_active` stays True for ever. Then:

- `_zone1` asks for nothing;
- `_stalled` is exempt;
- the tilt and walking checks are off.

Pupper stands in ZONE1 with no HOLD, and Mission Control keeps showing "NO THROWS, crossing".

- Repro: `--repro 10001 --keep crossing_drop_done`, also 10025, 10047, 10093, 10111.
- Fix A below: a deadline from `started` of max(1.5 × duration, duration + 30) s, pushed out by each `progress`, then HOLD "the crossing has not reported its end". Paired soak: deadlocks went from 50 to 0, and they became HOLDs instead.

### 2. A lost crossing "started": the plan flag sticks, and the crossing is not counted

Repro: `--repro 10002 --keep cmd1,crossing_drop_started`. For the full effect, use the default course (bar at 0.45, slab at 1.15) with `crossing_drop_started` and resumes at 90 s and 200 s.

- The mission first pauses ("the walking controller is not active"), which is a sound fail-safe.
- `crossing_planning` is never cleared, because the `done` branch does not reset it, so `_stalled` is off for the rest of ZONE1.
- The crossing itself is not counted: no advance and no `passed`. With both crossings missed, s ended 1.2 m short. Zone 2's "blocked" check then took the cover for an obstacle in the lane and walked round it before seeing it.
- Fix B: a `done` after `planning` with no `started` is counted as a crossing, and `crossing_planning` is cleared on every `done`. Tested: s 6.34 at the end, both obstacles passed.

### 3. The back-off reverses blind into an obstacle it has just walked around (clean seed 841)

Course: round bar 0.528, carton 1.22, slab 1.934, bar 2.689.

- Terrain HOLDs twice beside the carton at x 1.73. `_terrain_watch` reverses 0.45 m, and the body centre ends inside the carton (x 1.55; the carton runs 1.22–1.568).
- Repro: `--repro 841 --clean`.
- Fix C: the reverse is limited to the room behind before a tall obstacle (height of 0.08 m or more) or one it walked around, with 0.35 m for the tail and margin. Under 0.10 m it only stands, then goes around from there.
- A first version that also counted the low obstacles it had crossed cost 20-odd clean runs, and was revised.
- Paired soak: the collision is gone; no clean run changed outcome.

### 4. goto:<stage> accepted mid-run (249 runs)

`_jump` refuses only `ACTIVE` states, so a goto from PAUSED, HOLD or RESCUE resets the line and the distance and starts walking. The page disables the stage buttons unless the state is IDLE or DONE, and the README says "while idle", but `tools/nav_cmd.sh goto:ZONE1` in RESCUE would set Pupper walking from in front of the survivor.

- Repro: `--repro 10007 --keep cmd0` (goto:ZONE1 in RESCUE), or `--repro 10132 --keep cmd0` (in HOLD).
- Fix D: only from IDLE or DONE.

### 5. After a walk-around it never steers back to the line (clean seed 10946): NOT fixed

In `_zone1`, the "hold the line's direction beside an obstacle" window (near − 0.6 to far + 0.3) applies to every course obstacle. Several in a row therefore keep d at 0 all the way.

- Repro: `--repro 10946 --clean`. Course: carton, bar, slab, round bar. It goes around the carton, holds y = +0.54 past the bar (never crossed), and terrain's ALIGN pushes the body into the slab's corner.
- Limiting the window to walked-around obstacles (plus the one about to be walked around) removed this collision but lost 24 and won 2 of 1000 clean runs. So it is not proposed; it needs a different fix, such as steering back gradually.
- The configured demo course (bar then slab) is not affected.

### 6. The bar-to-slab gap: course layout, not a nav bug (clean)

In the bar-then-slab order the run stalls when the gap between the bar's far edge and the slab is under about 0.45 m. After the bar crossing the nose is too close to the slab to measure it, so the outcome is HOLD (no progress, or terrain blocked).

| Gap (m) | 0.35 | 0.40 | 0.45 | 0.50 | 0.55 | 0.60 | 0.65 | 0.70 | 0.75 |
|---|---|---|---|---|---|---|---|---|---|
| Reached | 0/25 | 2/53 | 46/57 | 44/49 | 50/57 | 38/43 | 48/55 | 45/49 | 31/36 |

The misses at 0.50 and above are runs where the survivor never says the phrase. The demo's 0.61 m is fine. Suggest a line in Demo instructions: at least 0.5 m clear between bar and slab.

### 7. The sound module's sign and zero: the run cannot recover

Runs with only that one fault:

| Fault | Reached |
|---|---|
| Sign flipped | 2 / 51 |
| Fixed zero offset of 10–40 degrees | 2 / 41 (2/13 at 10–20 degrees, 0/28 above) |
| Noisy (margin of error 10–30 degrees, 15–40 % echoes) | 24 / 62 |
| Bearings delivered 1–4 s late | 38 / 58 |
| First cry 20–120 s late | 44 / 73 |

With the sign flipped, CONFIRM rejects the fix twice, and on the third try (`max_reorients`) it walks the wrong way anyway. The refix is then rejected as an echo, and Pupper says "I'm here" up to 4 m to the side (`--repro 10029`). Nothing tells the team.

The README's clap test (step 2) checks the sign. Nothing checks the zero, yet a 10-degree zero error already loses most runs. Suggest also a clap dead ahead, which must read within ±5 degrees. Mission Control could also warn after two `fix_rejected` in a row whose offsets are about twice the turn just made, which is what a flipped sign gives.

### 8. Finishing by distance away from the cover (148 runs; 1 clean, seed 23)

When the line is off (noisy, offset or flipped sound, or a speed error), Pupper passes beside the cover and stops at `hard_stop` (7.2 m believed). It says "I'm here" there, the survivor's phrase ends the run, and the result is RESCUE 1–2 m beside the cover.

- Minimal faults: odometry 50, sound_offset 34, sound_noisy 15, sound_flipped 11.
- This is the designed fallback. Note that `hard_stop` is 0.8 m past the cover's face, so a Pupper that misses sideways walks on past the cover's plane before stopping.

### 9. Stopping at the cover's end (about 1 in 200 runs, clean included)

"The cover is in front of me" with the body centre within 10 cm of the cover's end (|y| 0.55–0.70), for example clean seeds 490, 526 and 728.

Worse, with only the cover's corner in the 35-degree cone, it once stopped 16 cm beyond the end (`--repro 10300 --keep sound_noisy`, y −0.76).

### 10. Collisions that are terrain agility's

Clean seed 222: terrain's ALIGN sidesteps into a carton just walked around (carton 1.87–2.218, then the bar at 2.603, a 0.385 m gap). The round bar was walked through in a few runs with heavy odometry or sound error (12972, 10744). The round-bar stall itself is known and not re-reported.

### 11. Exceptions: 4 runs, not reproducible

`JSONDecodeError` at t = 0 in seeds 13206, 13211, 13215 and 13224, close together in one batch. Re-running them gives no exception.

The likely cause is terrain agility's `decision.load_envelope` reading its envelope JSON while another session was rewriting it. That is terrain's code; it would crash a node started at that moment too.

No other exceptions in 15,500 runs (all batches together). There were no illegal transitions, no wrong resumes, no velocity while stopped, no reflex modes in the wrong state, and no non-strict JSON snapshots.

### Smaller things

- `CourseSim.run()` collects events with `log[seen:]`, but `Board.record` trims the log to 200 entries. After 200 events the run's event list (and its crossing and walk-around counts) stops growing. Normal runs have about 40 events, so this is latent.
- In the sim, the survivor's phrase is only heard within 2.5 m. That is a stand-in, and it decides several of the classes above.
- The partner's DJI sound experiments landed in b006305 (`path-navigation/sound-localization/`). Running `pytest` from `path-navigation/` now also collects `sound-localization/test_channels.py`, which needs `sounddevice`. Run `pytest tests`.

## Proposed fixes to nav/mission.py (not applied: another session owns the file)

These are fixes A–D above, in `reports/overnight-2026-11-27-mission.diff`, now rebased onto edad014 (see "Re-check on the one-bearing simulator" below). The diff also adds 5 tests to `tests/test_mission.py`. Apply it with `git apply path-navigation/reports/overnight-2026-11-27-mission.diff`.

Paired soak, current mission vs fixes A–D:

| | Current | A–D |
|---|---|---|
| Clean, reached | 655 / 1000 | 655 / 1000 |
| Clean, walked through an obstacle | 2 | 1 (seed 222, terrain's) |
| Faulted, deadlocks | 50 | 0 |
| Faulted, goto mid-run | 84 | 0 |
| Faulted, reached | 465 / 1500 | 421 / 1500 |

All 45 runs lost under faults were ones the current mission "reached" only through an operator goto from HOLD or PAUSED. With D, that goto is refused, so these are not real losses. One run was won.

The fixes were sent to the "Demo path navigation" session with these repro seeds, followed by the retraction of fix 5.

## Re-check on the one-bearing simulator (edad014, the partner's sound integration)

The diff was rebased onto edad014. `nav/mission.py` and `tests/test_mission.py` were not changed by that commit, so only the context moved. It passes `git apply --check` at edad014 (and at a6ffc73, which does not touch path-navigation).

It now also adds 5 tests to `tests/test_mission.py`:

- a crossing with no `done` HOLDs;
- a slow crossing that keeps reporting is not given up;
- a crossing whose `started` was missed is still counted;
- the back-off never reverses into a tall obstacle just passed;
- no goto from PAUSED, HOLD or RESCUE.

Four of these fail without the fixes; the slow-crossing one passes on both, as a guard.

**Tests**, `pytest tests` on a copy of edad014 (not the shared tree):

| | Result |
|---|---|
| Without the fixes | 65 passed, 2 skipped |
| With the diff applied | 70 passed, 2 skipped |

**Paired soak on the new simulator**: one bearing per cry, `drop_p` 0.4, `window_s` 20, `min_confidence` 0.5. Same seeds and the same code for both columns.

| | Current mission | Fixes A–D |
|---|---|---|
| Clean, seeds 0–399, reached | 265 | 264 |
| Faulted, seeds 30000–30599, reached | 194 | 177 |
| Faulted, deadlocks (all `crossing_drop_done`) | 15 | 0 (16 HOLD "the crossing has not reported its end" instead) |
| Faulted, goto accepted mid-run | 27 | 0 |
| Exceptions, illegal transitions, wrong resumes, velocity while stopped, reflexes in the wrong state | 0 | 0 |

- **The 17 faulted runs lost** are all runs the current mission "reached" only through an operator goto from HOLD or PAUSED, which fix D refuses.
- **The one clean run lost (seed 12)** is a trade-off in fix C: low bar 0.459, carton 1.245, slab 2.201. Stuck at the slab 0.39 m past the carton's far edge, the current mission reversed 0.45 m blind. It got away with it because it was off to the carton's side. Fix C does not reverse there, and ends in HOLD. Fix C does not know the lateral offset, so it errs towards not reversing.
- **The sim change, apart from the fixes:** with no faults, the current mission reached 265 of seeds 0–399 on the new simulator and 268 of the same seeds on the old two-bearing one.

## What remains unverified

- All of it on the robot: real crossings, real terrain messages and their timing, real dropouts.
- The partner's real bearings: their noise, delay, sign and zero.
- Whether the step-over node really sends `duration` in `started`. Fix A falls back to 90 s if not.
- Whether the real step-over node's "progress" rate keeps fix A's deadline alive on a slow crossing. It is pushed 30 s past each progress message.
- Reversing over a low bar just crossed (fix C still allows it) has no physics in the sim.
- How the tall-obstacle threshold in fix C (0.08 m, the sim's own) matches terrain agility's idea of "tall".
