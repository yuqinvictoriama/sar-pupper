# Terrain agility: overnight sweep and stress report (27 Nov 2026)

**Nothing in this report was run on the robot.** Everything below is MuJoCo simulation, or six
real camera frames from the Pupper (26 Nov 2026, glossy wooden floor) replayed on the laptop.
Where a number is simulation-only it says so.

New tools (both laptop-only, nothing moves):

- `tools/sweep_sim.py`: randomised MuJoCo runs of the three behaviours through the real
  gauge, `decide`, `TerrainFsm._after_gauge` ALIGN pulses, `plan_crossing`/`plan_scale_across`
  and the handover/player loop, as `stepover_node` plans them.
- `tools/stress_gauge.py`: degrades the real frames (glare, blur, exposure, colour balance,
  noise, JPEG) and finds the level at which the reading changes or a phantom appears.
- `tests/real_wood_floor/`: the six slab frames, six clear-floor frames and the floor model taught
  there (a copy of `config/floor_model.json`), so `stress_gauge.py` can run from the repo.

How to run (see also the venv and `PUPPER_XML` notes for the physics tests):

```
PUPPER_XML=.../pupper_v3_complete.position.xml python tools/sweep_sim.py --cases bar,slab,sequence,carton,roundbar --runs 60 --workers 4 --out sweep.jsonl
python tools/sweep_sim.py --summary sweep.jsonl
python tools/stress_gauge.py            # --relearn, --pose yaml, --only "glare", --json out.json
```

## 1. Tests

| Run | Code | Result |
|---|---|---|
| 1 | before 4473f60's new tests, with that session's then-uncommitted FSM changes | 178 passed, 0 skipped, 0 failed (26 min 49 s); the physics tests ran |
| 2 | HEAD 6ab9ac2 plus another session's edits in progress (`handover.py`, `player.py`, `stepover_node.py`, `crossing_sim.py`, 23:41) | 183 passed, 9 failed (19 min 51 s) |

All 9 failures in run 2 are in `test_robot_nodes`, and every one of them is
`Handover.update() got an unexpected keyword argument 'up'`. That is the other session's edit,
which was half-done at the time. None of my files is involved. I have not re-run the tests after
that session finished.

## 2. Simulation sweep (`sweep_sim.py`), simulation only

What is randomised in each run:

- **Camera.** Height 165 ± 10 mm, pitch −11 ± 2°, forward offset 150 ± 10 mm, roll σ 0.5°. The
  gauge assumes the nominal pose, so these spreads stand for calibration error.
- **Start.** Distance at the near stop 0.33–0.41 m, lateral offset ±0.10 m, yaw ±8°.
- **Walking.** Factors for how far the robot really moves for each commanded pulse (forward 0.45–0.65,
  turn 0.4–1.0, sideways 0.3–0.9).
- **Floor and load.** Floor friction 0.35–1.0, payload 0.7–1.3×.
- **Rendering.** Ten frames, each with fresh noise and JPEG q90.
- **Sequence case.** The slab's offset and yaw (±4°), plus where the robot stops before it.
- **Carton and round bar.** Lateral offset ±0.15 m and yaw ±20°.

A run counts as **ok** in these cases:

- Bar, slab and sequence: the robot crossed with the tilt abort not triggered, nothing pushed more
  than 10 mm, and no unplanned contact over 10 N.
- Carton: it walked round with at least 0.15 m clearance.
- Round bar (a 20 mm diameter cylinder, which must not be stepped on): the robot refused, or chose
  to walk round.

Walking between gauge looks is a teleport scaled by the walking factors, not the real walking
policy (see section 8).

### 2.1 Current code (camera ±10 mm / ±2°), 300 runs

| Case | ok | 95 % CI | Main failure modes |
|---|---|---|---|
| bar (20 mm × 30 mm) | 21/60 = 35 % | 24–48 % | 33 refused (17 "off square", 15 "foot not in clear view", 15 "outside the start range"), 4 hard touches, 2 tilt aborts |
| slab (15 mm × 380 mm) | 17/60 = 28 % | 19–41 % | 31 refused (same three reasons), 7 short or fell, 4 hard touches, 1 tilt abort |
| bar then slab | 8/60 = 13 % | 7–24 % | 37 bar refused, 6 slab refused, 4 slab hard touches, 4 tilt aborts |
| carton (walk round) | 60/60 = 100 % | 94–100 % | none |
| round bar | 48/60 = 80 % | 68–88 % | **12 would have stepped over it** |

**ALIGN turns the wrong way.** Of the runs that received a turn pulse, every one failed: 0/32
for the bar, 0/31 for the slab and 0/37 for the sequence. In `terrain/state_machine.py`:

- The code says skew > 0 means "the left end is farther away", which is right (a Cuboid turned
  +6° measures as skew −6°).
- But the robot then turns *towards* the farther end. That increases the skew, until the pulse
  budget runs out and the robot refuses for being off square or out of range.

Patch 1 in section 7 fixes this. I could not apply it because `state_machine.py` belonged to
another session.

### 2.2 With the ALIGN sign patched (applied in the sweep only, `--patch-align`), 240 runs

| Case | ok | 95 % CI | Main failure modes |
|---|---|---|---|
| bar | 46/60 = 77 % | 65–86 % | 7 tilt aborts, 5 hard touches, 2 refused |
| slab | 38/60 = 63 % | 51–74 % | 14 short or fell, 5 hard touches, 3 tilt aborts |
| bar then slab | 34/60 = 57 % | 44–68 % | 9 bar tilt, 7 slab hard touches, 4 slab short, 3 slab tilt |
| round bar | 7/60 = 12 % | 6–22 % | **53 would have stepped over it** (squaring up brings it into range) |

### 2.3 ALIGN patched, good calibration (`--cam-spread 0.25`: ±2.5 mm, ±0.5°), 239 runs

| Case | ok | 95 % CI | Failures |
|---|---|---|---|
| bar | 59/60 = 98 % | 91–100 % | 1 tilt abort (near-distance error −5.8 mm, a front foot touched down early) |
| slab | 58/60 = 97 % | 89–99 % | 2 hard touches |
| bar then slab | 58/60 = 97 % | 89–99 % | 2 slab hard touches |
| round bar | 8/59 = 14 % | 7–25 % | **51 would have stepped over it** |

Measurement errors in this sweep:

- Near distance: 5th–95th percentile within ±5 mm.
- Slab depth: −13 to +19 mm.

One round-bar run hung inside a worker and never finished. I killed it after 25 minutes, so that
row has 59 runs.

### 2.4 How accurate the calibration has to be (from 2.1 and 2.2)

**Distance to the bar.**

- The range error is about (height error) / (camera height).
- Near-distance errors within ±8 mm: every run was fine.
- Errors more negative than −15 mm (the bar is nearer than measured): 0/7 ok, including tilt
  aborts at 45–50° with a front foot landing on the bar.
- This needs camera height to about **±4 mm**.

**Slab depth.** Depth changes by about 33–52 mm for every degree of pitch error.

- Depth under-read by more than 30 mm: 0/11 ok. The rear feet end 40–60 mm short and drop off
  the far edge.
- Depth over-read by more than 30 mm: some hard touches (a rear shin on the far edge) and tilt
  aborts.
- This needs pitch to about **±0.5°**.

**The pose in `config/terrain.yaml`.** I ran a gauge-only sweep assuming that pose (250 mm,
level) while the simulated camera sits at 165 mm / −11°, so the uncalibrated case. The results are in
section 2.5. On the real frames the same yaml pose reads the slab as 188 mm deep, which falls in the
gap between the two crossings, so there the robot refuses. In simulation it often does not.
**Calibration must be done before any crossing.**

### 2.5 Gauge only, assuming the yaml pose (`--assumed yaml --no-physics`)

30 runs per case, measurement and decision only (no physics). The camera is simulated at the nominal
pose (with the usual ±10 mm / ±2° spread); the gauge assumes the yaml pose. Current ALIGN code.

| Case | Decided to cross | Refused or went round | Note |
|---|---|---|---|
| bar | 3/30 | 27/30 (18 "over the height limit for this depth": height read about 7 mm high) | the 3 crossings had depth −11 to −17 mm, near −11 to +4 mm |
| slab | **16/30** | 14/30 | **every one of the 16 had depth under-read by 84–148 mm** |
| carton | walked round 30/30 | | |
| round bar | 1/30 would step over | 29/30 refused | |

The slab result is the dangerous one. In 2.1, an under-read of more than 30 mm never ended well
(rear feet short). So **with the yaml pose the robot would try to scale a slab it has badly
mis-measured more than half the time**. It must not be run uncalibrated.

## 3. Round bar: a dangerous step-over (simulation only)

The camera is 165 mm high. From there, a 20 mm round bar 0.36 m away reads as about 20 mm high
by 78 mm deep. That is a plausible flat bar, so the robot plans a step-over and would put a foot
on a rolling cylinder.

- In 2.1, 12/60 runs would have crossed it.
- Once ALIGN is fixed, the robot squares up properly, and 51–53 of 60 would.
- I did **not** run the round bar in physics, so I do not know whether a foot would roll.

There is a cue that tells them apart. The apparent depth changes with distance. Between looks at
0.46 m and 0.36 m, over 16 random camera poses:

| Object | Change in apparent depth |
|---|---|
| Round bar | 19–38 mm |
| Flat bar | at most 5 mm |

The proposal (sketch in section 7, patch 3) is:

1. Gauge once at the far stop and again at the near stop.
2. Refuse a step-over when the apparent depth changes by more than 10 mm.

This needs one extra gauge pause during the approach, but no new interface.

## 4. Real frames: lens rim and floor model

Six slab frames and six clear-floor frames were replayed through the real `terrain_node`, using
the nominal pose (165 mm, −11°, 150 mm forward).

**The slab gauge works.**

- It reads h 14.9 mm and d 354 mm (true 380 mm), 0.555 m away.
- The side face is seen on 0.98 of the bearings.
- `decide` says scale_across.

**The lens rim stops the walking scan.**

- Beyond about ±85° bearing, the fisheye's own rim (and the legs) read as TALL at 0.10 m, inside
  the lane.
- With the slab frames, `terrain_node` therefore goes to HOLD "no way round (blocked on both
  sides)" and never approaches.
- With patch 2 (ignore bearings beyond 80°) and patch 1 both applied, it APPROACHes the slab.
- On the clear-floor frames it then approaches a tape measure lying 0.18 m away. The gauge
  reading for it is nonsense: skew −81°, depth −940 mm. That is a separate thing to look at: the
  gauge should reject a reading like that outright.

**The floor model only works where it was taught.**

- A model taught from the clear-floor frames, which were taken at another spot in the same room,
  fails at the slab spot.
- `config/floor_model.json` was taught at the slab spot. That is why the real-frame results here
  pass with it.
- On the robot, teach the floor where the crossing will happen.

**None of the perception results depend on `floor_model.json`.** I re-checked this as the
coordinator asked:

- No sweep run failed in perception. The simulated floor is taught per run from a clear render.
- On the real frames, the only perception failures are the rim HOLD and the stress levels below.
  The rim HOLD happens with the saved model as well.

## 5. Stress test on the real frames (`stress_gauge.py`)

Each entry is the mildest level, on each side of the undisturbed level, at which the reading
changed. Glare is a warm highlight blob of the given strength (0–1). A "phantom" means the
walking scan sees an obstacle in the lane on the clear-floor frames.

The first column uses the saved floor model and the nominal pose.

| Degradation | Breaks at (saved floor model) | What happens | With `--relearn` |
|---|---|---|---|
| Glare at the slab's foot | 0.35 | height not read | 0.3 |
| Glare on the floor in front | 0.4 | height not read | 0.3 |
| Glare on the slab's top | 0.5 | reads only 315 mm wide | 0.6 |
| Glare on the floor beyond | 0.6 | phantom 6/6 | 0.6 |
| Gaussian blur | 2 px | height −3 mm (still crossed) | same |
| Vertical motion blur | 9 px | not centred, so go round | same |
| Exposure | below 0.8 / above 1.25 | phantom 1/6 / height lost | 0.55 / 1.1 |
| Brightness offset | −15 / +40 | phantom 6/6 / height lost | held / +25 |
| Contrast | 0.6 / 1.1 | height lost / phantom 1/6 | 0.6 / 1.1 |
| Gamma | 0.75 / 1.1 | height lost / phantom 6/6 | 0.75 / held |
| Red gain | 0.95 / 1.05 | height lost / phantom 6/6 | 0.95 / held |
| Blue gain | 0.95 / 1.05 | phantom 6/6 / height lost | 0.85 / 1.05 |
| Sensor noise σ | 3 | phantom 1/6 | 8 |
| JPEG quality | 70 | phantom 1/6 | 15 |

**Reading the table.**

- Almost every failure is safe. The robot either loses the height and walks round, or stops for a
  phantom.
- None of them gives a wrong "cross".
- With the yaml pose instead, brightness +35, exposure 1.2 or a ±5 % colour shift moves the
  distance by 85–175 mm. That would be dangerous if it ever led to a crossing.

**It is fragile.**

- A darkening of only 15 grey levels, or a 5 % white-balance shift, makes phantoms on the clear
  floor with the saved model.
- Most of that goes away when the floor is taught again from the same, degraded, frames
  (`--relearn`).
- Auto-exposure or white-balance drift between teaching the floor and walking is therefore the
  main risk.
- Teach the floor just before each run, and lock exposure and white balance. Both are
  laptop-side suggestions only and have not been tried on the Pupper.

**Over-fitting caveat.** The gauge thresholds were tuned on these same six frames (e625cc7). The
undisturbed margins are therefore probably better than on new frames.

## 6. What I changed and committed

New files only:

- `tools/sweep_sim.py`
- `tools/stress_gauge.py`
- `tests/real_wood_floor/` (12 frames with their pose files, plus `floor_model.json`, about 2.5 MB)
- this report

No existing file was edited. `config/terrain.yaml` and `config/floor_model.json` were not
touched. The tools take their poses and overrides on the command line.

## 7. Proposed patches (not applied)

**Patch 1: ALIGN turn sign** (`terrain/state_machine.py`, about line 419). This is the biggest
single gain in simulation: bar 35 → 77 %, slab 28 → 63 %, sequence 13 → 57 %.

**Apply it only together with patch 3.** Fixing ALIGN alone makes the round-bar step-over much more
likely.

```diff
@@ -414,9 +414,10 @@
         if self.align_tries < cfg.align_max_tries:
             cmd = None
             if abs(skew_deg) > cfg.align_skew_deg:
-                # skew > 0: the left end is farther away -> turn left (positive yaw) to face it
+                # skew > 0: the left end is farther away, so the robot is turned left of square
+                # -> turn right (negative yaw) to face it
                 dur = max(cfg.align_min_pulse, 0.8 * abs(est.skew) / cfg.align_yaw_rate)
-                cmd = (0.0, 0.0, math.copysign(cfg.align_yaw_rate, est.skew))
+                cmd = (0.0, 0.0, -math.copysign(cfg.align_yaw_rate, est.skew))
             elif est.ends_seen and abs(est.y_centre) > cfg.align_centre:
```

Before applying it, check the sign on the robot once, at rest, with motion enabled only for that
pulse. The sign convention in this report comes from the simulation and from the gauge's own
tests.

**Patch 2: ignore the lens rim in the walking scan** (`terrain/scan.py`). `scan.py` is clean in
git, so I could have applied this one. I did not, because I could only check it on six frames, and
the session that owns the component could not be reached.

```diff
@@ ScanConfig
     min_columns: int = 3              # bearings an obstacle must span
+    max_bearing_deg: float = 80.0     # wider than this the lens's own rim (and the robot's legs) fill the picture
@@ build_scan
     for j in range(n_az):
+        if abs(view.bearings[j]) > cfg.max_bearing_deg:
+            continue
         col_valid = labels[r_lo:r_hi + 1, j] != INVALID
```

**Patch 3 (sketch): two-look check against round bars** (`terrain/state_machine.py`, `_after_gauge`).

```python
# keep the first good estimate taken at the far stop (about 0.46 m)
if self.first_look is None and est.near_x > 0.42:
    self.first_look = est
# at the near stop, before accepting a step-over:
if decision.action == STEP_OVER and self.first_look is not None:
    if abs(est.depth - self.first_look.depth) > 0.010:   # m; flat bars change by at most 5 mm
        decision = Decision(GO_AROUND, ["apparent depth changed with distance: round, not flat"])
```
```

The `/terrain/state` fields and `/terrain/crossing_status` are unchanged. A refusal reuses the
existing go-round path.

## 8. Not verified

- **Nothing was run on the Pupper.** All success rates are simulation only.
- **Walking.** Between gauge looks the robot is teleported by randomised walking factors. The real
  walking policy is not in the loop, so approach errors on the robot may be larger.
- **Lighting.** The simulated renders have noise and JPEG artefacts, but no glare or reflections from
  a glossy floor. Real-frame stress covers only one slab, one spot and six frames.
- **Round bar.** Not run in physics: whether a foot rolls on it is not known. Patch 3 was checked
  only as a measurement (depth change between two looks), not inside the state machine.
- **Patches.** Patches 1 and 2 were checked in the sweep and with `terrain_node` on the real
  frames. The full test suite has not been run with them applied.
- **Tests.** The test run after the other session finished its `handover`/`player` edits has not
  been done (see section 1).
