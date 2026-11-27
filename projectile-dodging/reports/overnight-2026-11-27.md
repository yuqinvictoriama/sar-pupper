# Overnight check of the false-dodge fixes, 2026-11-26/27

Everything here was done **off the robot**, on the Mac, by replaying recordings the Pupper had saved and in the simulator. Nothing was run on the Pupper. Its files were only read (copied with `rsync`/`scp`); no node, camera or motor was started there.

## What was replayed

| Source | Count | What it is |
|---|---|---|
| `data/throws/` on the Pupper | 330 | every throw the dodge node saved on 2026-11-26 (21 armed dodges, 16 dry-run dodges, 12 saved by hand, 281 "seen, no dodge") |
| `data/events/` on the Pupper | 9 | the older dodge dumps of 2026-11-26 midday |
| **Real recordings, total** | **339** | |
| Simulated cases | 195 | `dodge/sim.py` cameras at the rates measured on the Pupper (rear 21 pictures a second, 40 ms; fisheye 20, 70 ms; lag 0.25 s, delay 0.07 s): 60 lobs aimed at the robot from every side (2–3 m, apex 0.4–0.9 m), 30 aimed 0.7–1.5 m wide, and 15 each of: a ball on the floor, a still ball the cameras read at different ranges (fisheye ×1.2–1.8, rear ×0.4–0.8 in size), a still ball with a flickering blob, a ball lifted out of a basket, a ball tossed up and caught, a ball carried past, a ball dropped |

Not replayed: the six long recordings in `data/recordings/` (3.6 GB, 2026-11-23, before the colour-cast fix), left on the robot because the Mac's disk was 97 % full. The repo has no local `data/` (it is git-ignored). The 213 recordings from 16:xx were copied in batches, searched once into the cache, and their pictures deleted again for the same reason.

The pictures were searched with the calibration files that were on the Pupper (`config/calib/`, copied from it). Its fisheye colour calibration is for hue 8–27 (the hand) and is ignored by `load_config`, so the fisheye used the starting colour limits. **Redoing the fisheye colour calibration with the ball is still needed.**

### Ground truth

Only what someone said is used; everything else is counted as "unmarked".

- `label.json` (Pupper Dodging View): 25 recordings; "dodged" and "hit" are taken as throws that should be dodged (8), "off target" as misses (4), "not a throw" as none (13).
- From the Projectile dodging session: the live dodges `213905`, `213910`, `214011` were false (still patches; the last ended in a fall), and the `205xxx-dry` dry-run dodges were handled balls. I took **all ten** `20261126-205xxx-dry` as "not a throw" (the plan says ten in four minutes with nobody throwing; the message said five; two of them also carry a human label "not a throw").
- That gives 8 throws to dodge and 27 recordings that must not dodge. Three of the 8 throws (`155451`, `155727`, `160106`) are from 15:5x, when the cameras ran at 3–8 pictures a second; no setting dodges them.
- The 15 live dodges of the armed lobs at 21:38–21:40 are reported separately: 3 are known false, the other 12 are unmarked (probably real throws, but nobody said so).

## The tool

`tools/sweep_thresholds.py` (new). Each real recording is searched **once**, exactly as the camera processes on the robot do it (`FramePipeline.process_jpeg`, still filter, search near the tracked ball), and the blobs are cached; each setting is then tried by running the dodge core on those blobs (about 15 s for all 339 plus the simulated cases on eight cores). It refuses `detector.*` and `cameras.*` keys, which change what the cameras find. One approximation: the search near a tracked ball follows the tracks of the configuration the cache was made with. The recommended setting was checked afterwards with `tools/replay.py`, which searches the pictures again (below): it found one false dodge more than the cache under the current config (`205423-dry`) and none more under the recommended one.

Scoring: a false dodge is any dodge in a "miss" or "none" case; a missed throw is a "hit" recording with no dodge, or a simulated lob the robot did not get clear of. The tool recommends the setting with the lowest 3 × false dodges + missed throws (`--false-weight`), one key at a time.

```
python3 tools/sweep_thresholds.py data/throws --cache /tmp/sweep-cache                  # sweep
python3 tools/sweep_thresholds.py data/throws --cache /tmp/sweep-cache --only \
    --set threat.loom_min=0.15 --set threat.loom_support=0.45 --set threat.bearing_max_closing_speed=6.0
```

## Results

### On fa4fd42 (the code the coordinator asked for, and what was on the Pupper)

| Setting | Real false dodges | Real throws missed | Sim false dodges | Sim lobs not cleared | Unmarked recordings dodged |
|---|---|---|---|---|---|
| current config | 3 / 27 | 4 / 8 | 19 / 135 | 2 / 60 | 11 / 304 |
| **recommended** (below) | **2 / 27** | **4 / 8** | **3 / 135** | **2 / 60** | 10 / 304 |

Checked again by searching the pictures (`tools/replay.py --labels`, the 38 labelled or armed recordings whose pictures were still on the Mac): false dodges with no throw **4 of 20 → 2 of 20**; labelled and armed hits dodged 10 of 16 in both (so no throw was lost). The armed session: 8 of the 12 unmarked live dodges fire in replay with either setting; the 3 known false ones fire with neither.

The same with another set of simulated cases (`--seed 23`): sim false dodges 16 → 5 of 135, sim lobs not cleared 5 → 5 of 60.

Which simulated cases are behind the change: **a ball lifted out of a basket** fired 12 of 15 times and **a ball tossed up and caught** 7 of 15 with the current config; 3 and 0 with the recommended one. Floor balls, the cameras' range disagreement (the 4.5 m / 1.5 m case), flickering still balls, carried and dropped balls fired 0 times in either. So the fixes of 2026-11-26 night hold for still balls in simulation; what they left open is a ball moving **up** near the robot.

Why: a ball rising beside the robot climbs in elevation just like an incoming lob, so the bearing test takes it for one coming at 3–4 m/s. The size test that should catch it ("is it getting bigger?") is skipped whenever the expected approach over the looks is below `loom_min` (0.3) of the range. With three looks over 0.1 s at 3 m/s from 1.2 m, that is 0.31 m against 0.35 m: skipped, and the dodge fires. Lowering `loom_min` to 0.15 lets the size test speak in those cases; `loom_support` 0.45 asks for a little more growth.

### Recommended setting and its margin

| Key | Now | Recommended | Same score from … to | Notes |
|---|---|---|---|---|
| `threat.loom_min` | 0.3 | **0.15** | 0.1 to 0.2 | 0.3 and above: 18–24 sim false dodges. No real recording changed between 0.1 and 0.6 |
| `threat.loom_support` | 0.3 | **0.45** | 0.45 only | 0.3: 5 sim false; 0.6: one more sim lob not cleared |
| `threat.bearing_max_closing_speed` | 7.0 | **6.0** | 6.0 to 6.5 | **Thin.** 7.0 lets `204859-dry` (labelled not a throw, read at 6.4 m/s) fire; 5.0 loses the labelled throw `204819-dry` and 4 unmarked armed ones. The fastest real throw that fired was read at 5.2 m/s (`213835`) |

Keys that changed nothing in the sweep: `min_climb` (0–0.06), `trigger_margin_s` (0.3–0.75), `confirm_cycles` (1–4), `sigma_lat_max_m` (0.1–0.3), `k_sigma` (0.5–2.0).

Not recommended although the sweep liked them, because each rests on one recording or costs real throws:
- `threat.min_closing_speed` 2.0 removes 2 simulated false dodges for 1 simulated lob, but two real throws were read at 2.2 and 2.4 m/s (`213955`, `214007`); 2.5 loses 24 of 60 simulated lobs.
- `threat.margin_m` 0 removes one real false dodge (`205525`) but also shrinks the escape's clearance (the planner uses the same margin).
- `tracker.gate_angle_deg` 60, `min_hits` 4, `confirm_hits` 4: each removes the last simulated false dodge and loses real or simulated throws.

The full first-round table (one key at a time, each round building on the previous picks, so rows after `bearing_max_closing_speed` already include 5.0) is in the scratchpad log and can be remade with the command above.

### On the newer code (c3329ad, 3842b7a)

While this ran, the Projectile dodging session committed c3329ad ("straight behind, make the most of the tenth of a second the rear cameras get") and then 3842b7a.

**A crash in c3329ad.** `Tracker._climbs` read `tr.history[-2]` when a track's second sighting came more than 0.35 s after its first (the history keeps 0.35 s). Replayed through c3329ad's core, **21 of the 339 recordings raised an IndexError** (for example `20261126-123829-dry` and many `16xxxx-seen`). Reported to that session; they confirmed it and fixed it in 3842b7a with a test. On the robot `robot/camera_worker.py` catches the exception, so it would have cost one picture, not the node. **On 3842b7a: 0 of 339 raise.**

Results on 3842b7a (cached blobs as above):

| Setting | Real false dodges | Real throws missed | Sim false dodges | Sim lobs not cleared | Unmarked recordings dodged |
|---|---|---|---|---|---|
| fa4fd42, current config | 3 / 27 | 4 / 8 | 19 / 135 | 2 / 60 | 11 / 304 |
| 3842b7a, current config | **7 / 27** | 4 / 8 | 20 / 135 | 1 / 60 | **23 / 304** |
| 3842b7a, recommended | 6 / 27 | 4 / 8 | 12 / 135 | 1 / 60 | 17 / 304 |

The armed session is the same on both: 8 of the 12 unmarked live dodges fire, the 3 known false ones do not. So the straight-behind changes **caught no extra real throw in these recordings** (one more simulated lob is cleared) and **added four labelled false dodges**, all in the handled-ball dry-run session:

| Recording | What fired | Which new rule let it through |
|---|---|---|
| `204813-dry` | 2 looks, `elp_left` only, from +87° (the side, not behind), 3.8 m, tau 0.69 s | two-look judgement (`tracker.two_look_cameras`) |
| `205203-seen` | 3 looks from both rear cameras, from −147°, 1.0 m | pooled rear cameras (`threat.bearing_pool`) |
| `205322-dry` | 3 looks from both rear cameras, from −158°, 1.4 m, not confident | pooled rear cameras, then fired unconfident (`threat.unseen_fire_s`) |
| `205455-dry` | 3 looks, `elp_left`, from +99°, not confident | fired unconfident while the ball was out of sight (`threat.unseen_fire_s`) |

Turning any one of those options off brings real false dodges from 7 down to 5 or 6, not to 3; together they make the difference. Suggestions for that session (not made here): use the two-look rule only within a cone straight behind (|bearing| above about 150°, where it is meant to help), since `204813` is a side ball; and require `confident` for an unseen fire when the range is under about 1.5 m (handled balls are close).

The recordings are a dry-run evening with someone handling balls next to the robot and 12 unmarked armed throws; the straight-behind case the new rules are for is barely in them (`213916`, from −162°, fires with or without them). Judge them on throws from straight behind.

## Still dodged falsely with the recommended setting

- `205520-dry` and `205525-dry` (handled balls, fisheye, 1.2–1.4 m). `205525`: the blob's range read 1.37 → 0.88 → 0.88 m over 0.15 s, which looks like a ball coming in at 3.2 m/s and agrees with the bearing's 3.9 m/s; probably a hand uncovering the ball. Nothing in the threshold set tells it apart. The bearing test seems to have reached its limit here; a ball handled 1–1.5 m from the robot will sometimes pass for a throw.
- On 3842b7a also the four in the table above.

## Real throws still missed

- `155451-mark`, `155727-mark`, `160106-dodge`: 15:5x, 3–8 pictures a second. Not a threshold problem.
- `213938-dodge` (labelled "dodged" live): in replay the ball is judged "will miss" 35 times. Either the live dodge was for a ball that would have missed, or the replayed track differs. Unresolved; worth a slow-motion video next session.
- Of the unmarked armed live dodges, `213814`, `213838` and `214033` do not fire in replay (no flying track, or "not approaching").
- Every real throw that did fire was decided with 0.25–0.33 s left (median 0.29 s); the robot needs about 0.45–0.6 s. Thrown from 1–1.5 m, as these were, a lob cannot be dodged whatever the thresholds.

## bench_detect.py on the recordings

On the Mac, with the sweep running alongside (so the times say little about the Pi). Blobs per picture are the useful part:

| Recordings | Camera | Pictures | Pictures with a blob passed on | Blobs per picture |
|---|---|---|---|---|
| armed lobs, 21:38–21:40 | elp_left | 1121 | 286 | 0.3 |
| | elp_right | 1297 | 476 | 0.5 |
| | fisheye | 1227 | 706 | 1.1 |
| handled balls, 20:5x | elp_left | 819 | 578 | 2.9 |
| | elp_right | 1008 | 457 | 1.0 |
| | fisheye | 935 | 237 | 0.3 |
| 15:5x | elp_left | 405 | 401 | 7.0 |

The rear-left camera passes on far more blobs than the others, as the plan says (its colour calibration accepts noise on a dark floor). Redoing its calibration is still needed.

## Changes committed

- `tools/sweep_thresholds.py` (new): the sweep described above.
- `tests/test_sweep.py` (new, 4 tests): cached blobs give the same dodge as searching the pictures; labels from `label.json` and a CSV; repeatable simulated cases; detector keys refused.
- `tools/bench_detect.py`: a recording copied without its pictures is skipped with a note instead of stopping the tool.

Test suite on the working tree (3842b7a plus these files), Python 3.11, numpy 1.26.4, OpenCV 4.11: **110 passed**.

## Proposed change to `config/dodge.yaml` (not made: that file belongs to the Projectile dodging session)

```diff
--- a/projectile-dodging/config/dodge.yaml
+++ b/projectile-dodging/config/dodge.yaml
@@ threat:
-  bearing_max_closing_speed: 7.0   # m/s. A ball coming faster than this from 2.5 m arrives in under 0.36 s, less than
+  bearing_max_closing_speed: 6.0   # m/s. A ball coming faster than this from 2.5 m arrives in under 0.36 s, less than
                            # the robot needs to get clear; and the slower the limit, the less a still thing's
-                           # flicker can pass for a climb
+                           # flicker can pass for a climb. 7.0 let a handled ball read at 6.4 m/s fire (2026-11-26
+                           # 20:48); 5.0 lost real throws. The fastest real throw fired on was read at 5.2 m/s
@@
-  loom_min: 0.3            # where the ball should have come nearer by this share of its range between the looks ...
-  loom_support: 0.3        # ... its size must show at least this share of that approach (a ball lifted by hand does not)
+  loom_min: 0.15           # where the ball should have come nearer by this share of its range between the looks ...
+  loom_support: 0.45       # ... its size must show at least this share of that approach (a ball lifted by hand does not).
+                           # At 0.3 the size test was skipped for a ball lifted or tossed up 1-1.5 m away (three looks,
+                           # 0.1 s, 3 m/s): simulated, it fired 19 times in 30; with 0.15 / 0.45, 3 (tools/sweep_thresholds.py)
```

The crash fix that was proposed for `dodge/track.py` is already in 3842b7a.

## Not verified

- Nothing here has run on the Pupper. The recommended setting is checked on 339 replayed recordings and in simulation only.
- Ground truth is thin: 8 labelled throws (3 from the slow 15:5x cameras) and 27 labelled non-throws, mostly from one handled-ball session. The 12 unmarked armed dodges are probably throws but nobody said so.
- Replay feeds pictures at the rate they were recorded and puts a fixed 30 ms between picture and decision; the robot's real timing differs from throw to throw.
- The cache's search near the tracked ball followed the tracks of the code it was made with (fa4fd42; the 16:xx block while the other session was editing). A full `replay.py` search of the labelled recordings gave the same answers except one more false dodge (`205423-dry`) under the current config.
- The simulated handled balls (lift, toss) are my guesses at how a hand moves a ball; the real `205xxx` ones are the evidence that the case exists.
- The fisheye was searched with the starting colour limits (its calibration is for the hand); the rear-left camera's calibration is noisy. Both need redoing with the ball, and the numbers above may move once they are.
