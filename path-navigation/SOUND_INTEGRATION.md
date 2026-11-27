# Sound localisation: joining the partner's module to path navigation

**Date:** 2026-11-27. **Partner's code:** `path-navigation/sound-localization/`, merged in commit `ca06155` (Max's PR #1, "Add DJI sound localization", commit `b006305`). **This work:** the bridge that runs his code inside the demo, the checks made on its output, the settings navigation changed because of what it does, and tests.

**In one paragraph.** His four-microphone script now gives the demo its bearings. It runs **unchanged**, as a child process of a new ROS node (`robot/sound_node.py`), and every bearing it prints goes to `/sound/bearing`. A test checks that his five files are byte for byte what he committed. His conventions were checked end to end on synthetic four-channel calls, and they are right: 0° is the nose, positive is to the right, and navigation converts once. On those synthetic calls, bearings within ±40° of the nose had a median error of 1–2°. In quiet recordings a fault in the correlation step gave roughly one bearing in four an error of 20–90°. The bridge now flags those, and a two-line change in his `audio_utils.py` removes them (measured: 23 of 60 frames affected, then 0 of 60). **Nothing here has run with the DJI receiver or on the Pupper.** The receiver was not plugged in when the Pupper was checked (2026-11-27). The biggest unknowns are whether the four radio channels keep sample-accurate timing, and which process gets the receiver when the phrase detector also needs a microphone.

---

## 1. What the partner's code is

| File | What it does |
|---|---|
| `localize_angle.py` | The one used. Opens a 4-channel input (DJI Mic 3 receiver in Q mode). For each sound louder than a threshold, it estimates the time differences between channels (GCC-PHAT, `audio_utils.gcc_phat_tdoa`), fits a plane-wave direction by least squares, and **prints** a bearing |
| `audio_utils.py` | Device selection (auto-picks a device named "DJI" or "Wireless Mic Rx"), GCC-PHAT with parabolic sub-sample peak, WAV writer |
| `test_channels.py` | Prompts for each transmitter in turn and checks that each one peaks on its own channel; saves a 4-channel WAV |
| `localize_stereo.py` | The earlier two-microphone version (left/right only, cannot tell front from back). Not used |
| `README.md` | His instructions, the layout, and the conversion to ROS |

What `localize_angle.py` assumes, all of which the integration relies on:

```text
CH1 ───────── CH2     FRONT        transmitters TX1..TX4 on input channels CH1..CH4
 │             │                   120 mm left-right (CH1-CH2, CH3-CH4)
 │             │                   234 mm front-back (CH1-CH3, CH2-CH4)
CH3 ───────── CH4     BACK         all four at the same height, rectangle square to Pupper's body
```

- **Bearing:** degrees, 0 = Pupper's nose, **positive to the right**, ±180 behind, −90 left.
- **One estimate per sound event.** It estimates once, on the first 0.35 s window (taken every 0.1 s) whose RMS over all four channels reaches `--threshold` (0.01, i.e. −40 dBFS). It then does nothing until a window falls below 0.65 × threshold (−44 dBFS).
- **"uncertain"** when the fitted delays are nearly zero: fit magnitude below 0.089 (`--deadband-samples` 1.5 across the 120 mm side), printed as "uncertain near 0° (angle would be …)", or exactly zero, "uncertain (near-zero delays)".
- **Fit magnitude:** the length of the fitted direction vector. It is 1 for an ideal plane wave in the microphones' plane and cos(elevation) for a source above them. He prints a caution above 1.25 and below 0.15.
- It prints only. It publishes nothing and never moves the robot.

## 2. How it is joined

```text
DJI receiver (USB, Q mode)
   │ 4 channels, 48 kHz
   ▼
sound-localization/localize_angle.py      his script, unchanged, a child process
   │ stdout: "RMS=… | delays vs CH1: CH2=…, CH3=…, CH4=… samples | bearing=+11.2° | fit magnitude=0.98"
   ▼
robot/dji_bearing.py                       parses each line, adds two checks, restarts the script if it exits
   ▼
robot/sound_node.py                        ROS: /sound/bearing (JSON, his number unchanged), /sound/status (1 Hz)
   ▼
robot/nav_node.py → robot/sound_adapter.py topics.sound_positive = right: his degrees → radians, left positive
   ▼
nav/mission.py                             uses a bearing only if Pupper is still, not talking, confidence ≥ 0.5
```

**Why a child process and not an import.** His estimator lives inside `main()`, in the loop that reads the stream, and it prints rather than returns. Importing it would mean restructuring his code. Running it as a child process means the code on the robot is exactly the code he wrote and tested, and any change he makes reaches the robot with a `git pull`. The cost: the bridge depends on his output format. `tests/test_sound_integration.py` runs his real script and parses what it really prints, so a format change fails a test rather than silently giving no bearings.

**What `/sound/bearing` carries.** One JSON message per located sound: `{"bearing": +11.2, "confidence": 1.0, "check": "ok", "fit": 0.98, "rms": 0.0312, "delays": [-3.21, 31.90, 28.70], "residual": 0.01, "stamp": …}`. `bearing` is his number, positive right. "uncertain" lines are not published: they are counted in the status.

**What `/sound/status` carries.** Once a second: `alive` is true only while his script runs **and** has printed that it opened a device with at least four channels. It also carries the device name, the counts (bearings, uncertain, flagged), the last reading, and the last line his script printed. If he exits with "Four-channel localization needs four USB capture channels", that line reaches Mission Control's Sound panel. Navigation now trusts this heartbeat over the old fallback ("a publisher exists on the bearing topic"), because the bridge is a publisher even when his script is dead.

**The two checks** (in `dji_bearing.parse_line`, made on his printed numbers):

| check | confidence | when | why |
|---|---|---|---|
| `ok` | 1.0 | both checks pass | |
| `inconsistent` | 0.0 | \|CH4 − (CH2 + CH3)\| > 3 samples | For a rectangle, p4 − p1 = (p2 − p1) + (p3 − p1), so for any direction the delays add up: CH4 = CH2 + CH3. One pair locked onto a wrong correlation peak breaks this (section 4.3) |
| `fit` | 0.3 | fit outside 0.5 … 1.25 | Fit is cos(elevation). On this course the survivor is at most about 35° above the microphones, so a real call fits above 0.8. His own warning starts at 0.15 |
| (no bearing) | – | his script said "uncertain" | Section 5.7 |

`sound.min_confidence` is 0.5, so only `ok` bearings are used. All of them are logged in `data/runs/*.jsonl` and drawn on Mission Control's compass (dashed if not used).

## 3. Files

**Not changed:** everything in `sound-localization/`. The test `test_partner_files_are_exactly_as_committed` compares each file with `git show ca06155:<file>`.

**New**

| File | |
|---|---|
| `robot/dji_bearing.py` | Command line from `config/demo.yaml` section `dji`; the parser; the checks; `Localizer` (start, read, restart after 3 s, stop with Ctrl+C as a person would, status). No ROS |
| `robot/sound_node.py` | The ROS node around it |
| `tools/sound_replay.py` | Plays a 4-channel WAV through his unchanged script on a laptop and prints each bearing, its checks, and whether navigation would use it |
| `tests/test_sound_integration.py` | 13 tests (section 4.1) |
| `tests/dji_synth.py` | Synthetic 4-channel calls: a band-limited burst from a point source at a given bearing, range and elevation; exact path delays per microphone; optional per-channel timing offsets |
| `tests/fake_sounddevice/sounddevice.py` | Stands in for the `sounddevice` package so his script runs on a recording; at the end it sends Ctrl+C, which his script catches |

**Changed**

| File | Change |
|---|---|
| `nav/config.py`, `config/demo.yaml` | New section `dji` (his command-line settings, at his defaults, plus the checks). `topics.sound_positive` left → **right**. `sound.min_confidence` 0.0 → **0.5**. `sound.window_s` 14 → **20 s** (section 4.5) |
| `nav/board.py` | A heartbeat with `"alive": false` is kept (and shown) but no longer counts as alive |
| `robot/nav_node.py` | The "publisher exists" fallback for sound liveness applies only until the first heartbeat |
| `robot/demo.launch.py` | `sound:=dji` (default) starts `sound_node.py`; `sound:=none` starts nothing; any other value is still run as a shell command |
| `robot/start_demo.sh` | Starts the sound module by default; `--no-sound`; reports his script's start-up errors from the log |
| `tools/mission_control.html` | Sound panel: a **Module** row (device and last check, or "not listening" and his last message) |
| `sim/course_sim.py` | Models his script: one bearing per call (was two), 40 % of calls giving nothing usable; `--drop`, `--call-every`; prints time to first fix |
| `robot/sound_adapter.py`, `tests/test_node.py`, `tests/fake_ros.py`, `README.md`, `plan.md` | Convention and wording; the node test now sends −90 (left, in his convention) |

## 4. What was verified, and how

Everything here is **off the robot**. His real script ran on synthetic recordings through the fake `sounddevice`. The synthetic calls model the geometry exactly (point source, near field, elevation). They do **not** model reflections, the radio link, unequal channel gains, or real background noise; white noise at a chosen level stands in for that.

### 4.1 Tests

All of `path-navigation/tests` passes on a laptop: 66 passed, 1 skipped (`python3 -m pytest path-navigation/tests -q`; the phrase detector's test skips without its model). The 13 new ones:

- his files unchanged since `ca06155`;
- parsing his real line format (padding, "uncertain …", caution and status lines ignored, "Selected [n] … with 4 input channel(s)");
- the checks; `expected_delays` matches his geometry (straight ahead CH2 0, CH3 +32.75, CH4 +32.75 samples; right −16.79, 0, −16.79; CH4 = CH2 + CH3 at every angle);
- **end to end, his script → bridge → adapter**, at −90, −30, −8, 0, 12, 40, 135 and 180°: every bearing within 6° and of the right sign in navigation's convention;
- near field and elevation: 2 m away, 34° up (a standing survivor at the second call), fit 0.75–0.9, within 6°;
- one bearing per call (three calls with 1.2 s gaps give three);
- the quiet-room fault is flagged (section 4.3);
- a 2-channel device: his script exits with code 1, status says not alive with his message, and it is restarted after `restart_s`;
- the ROS node on a stand-in for ROS: publishes `ok` bearings, not "uncertain" ones, and a status with counts.

### 4.2 Sign convention and accuracy on clean calls

Eleven directions around the circle, at 50 m (plane wave), 6 m, 2 m, and 2 m with the source 34° up, one call each: the sign was right every time, and the worst error was 3.8° (at 135°). At 34° elevation the fit magnitude was 0.78–0.85, as cos 34° = 0.83 predicts, and the azimuth was unaffected (worst 2.6°). The planar-wave assumption costs nothing measurable at 2 m.

### 4.3 A fault found in quiet recordings, and its cause

In 20 rooms per condition, each with 3 calls from a random bearing within ±40° (240 calls in all):

| Condition | Bearings | All bearings: median / 90 % / worst error | Off by > 10° | Pass the checks | Passing: median / 90 % / worst | Passing, off by > 10° |
|---|---|---|---|---|---|---|
| 6 m, quiet (noise −66 dBFS) | 54 (6 "uncertain") | 2.7° / 57° / 94° | 15 | 35 | 1.4° / 5.0° / 8.5° | 0 |
| 6 m, quiet, quieter call | 60 | 1.6° / 7.0° / 76° | 5 | 53 | 1.5° / 3.6° / 6.8° | 0 |
| 6 m, noisier (−54 dBFS) | 60 | 0.8° / 2.4° / 5.7° | 0 | 60 | 0.8° / 2.4° / 5.7° | 0 |
| 2 m, 34° up, quiet | 59 (1 "uncertain") | 4.5° / 58° / 78° | 18 | 42 | 1.9° / 7.8° / 11.1° | 1 |

**The paradox (more noise, fewer errors) is the clue.** In each bad case, one microphone pair's delay came out near 0 samples (for example CH4: 0.19 where 25.0 was expected), and the fit magnitude still looked normal (0.98), so his warnings did not fire. The cause is in the correlation step, and it would happen with real audio:

1. The analysis window is cut with hard edges, and at the moment of the estimate the call is still going on, so the cut lands mid-sound **at the same instant on all four channels**. A hard edge puts energy at every frequency, and that energy is aligned across channels, which is to say at zero lag.
2. GCC-PHAT gives every frequency bin the same weight, including the bins where the voice has no energy. In a quiet room those bins hold almost nothing but that aligned edge energy, and it can outvote the voice band. With more background noise, those bins hold independent noise instead, and the true peak wins.

To confirm it, the same 60 quiet-room frames were run through modified copies of his `gcc_phat_tdoa`, kept in a scratch file, not in the repo. Frames with a pair more than 3 samples off: **his: 23 of 60; with a Hann window on the frame: 0; with PHAT limited to 200–4000 Hz: 0; both: 0.** In the noisier condition all four gave 0 of 60.

A real room is never as quiet as −66 dBFS in every band. But voices have little energy above 4 kHz, and the DJI transmitters are low-noise, so the right conditions for this fault are plausible. Until his code changes, the bridge's checks catch it. In the table, no bearing that passed the checks was more than 10° off at 6 m. The one exception was 11.1° in the 2 m elevated case. The cost is that 25 of 60 quiet-room calls at 6 m gave no usable bearing.

### 4.4 Timing offset between radio channels

TDOA needs the four channels sample-aligned: 1 sample is 0.021 ms, or 7.1 mm of path. Five calls at −30 … +30°, from 6 m, with one channel delayed by a fixed amount:

| Offset | CH2 (front right) late: error | flagged | CH4 (back right) late: error | flagged |
|---|---|---|---|---|
| 0 | −0.6 … +3.0° | 0/5 | same | 0/5 |
| 0.5 sample (0.01 ms) | −2.1 … +1.1° | 0/5 | −1.0 … +1.2° | 0/5 |
| 1 sample | −3.8 … +0.4° | 0/5 | −1.2 … +0.9° | 0/5 |
| 2 samples (0.04 ms) | **−5.3 … −3.2°** | 1/5 | −1.8 … +0.3° | 0/5 |
| 4 samples | −9.7 … −7.7° | 4/5 | −4.2 … 0.0° | 3/5 |
| 8 samples (0.17 ms) | −19 … −15° | 5/5 | up to +50° | 5/5 |

A front channel matters most for bearings near the nose: left-right is measured across only 120 mm. **An offset of 2 samples, 0.04 ms, between the front transmitters biases every bearing by about 4–5°, and the checks do not catch it.** Whether the DJI receiver keeps four independent radio links aligned to that precision is not known. It is the first thing to measure (section 6, step 4).

A bias like this is a zero error: every bearing is off the same way. The overnight soak (`reports/overnight-2026-11-27.md`, section 7) found that the run does not recover from one. With a fixed zero offset of 10–20°, 2 of 13 simulated runs reached the cover, and above 20°, 0 of 28. So the straight-ahead clap in section 6 must read within a few degrees of 0.

### 4.5 What the demo's whole run does with it

`sim/course_sim.py` now gives one bearing per call, the true direction plus Gaussian noise (±5° read as 2 standard deviations), 8 % echoes 30–90° off, and 40 % of calls giving nothing usable. The 40 % is the quiet-room figure from 4.3; in the noisier condition it was 0 %. 200 simulated rooms per row, seed 1:

| Sound | Calls with no usable bearing | Second call | Ended in front of the cover | Sideways miss, median (worst) | First fix, median (slowest) | Runs that said "I can't hear anyone yet" |
|---|---|---|---|---|---|---|
| ±5° | 40 % (default) | yes | **199 of 200** | 0.13 m (0.56 m) | 22 s (147 s) | 58 |
| ±5° | 0 % | yes | 199 of 200 | 0.08 m (6.58 m, one run, not examined) | 14 s (22 s) | 0 |
| ±5° | 60 % | yes | 181 of 200 | 0.17 m (0.66 m) | 36 s (409 s) | 119 |
| ±10° | 40 % | yes | 182 of 200 | 0.19 m (4.07 m) | 26 s (158 s) | 75 |
| ±5° | 40 % | no | 188 of 200 | 0.35 m (0.99 m) | 22 s (147 s) | 58 |

"Ended in front of the cover" means it stopped less than about 0.55 m to the side of the line (the cover reaches 0.6 m either side). With one bearing per call, a ±10° module loses 18 runs (182 of 200); under the old model, with two bearings per call, it lost 6 (194 of 200). The ±5° margin matters more now.

Before this change the simulator gave **two** bearings per call and never dropped one. With one per call, the first fix comes much later and Pupper more often says "I can't hear anyone yet". That is why `sound.window_s` went from 14 to 20 s: Pupper stands still while it listens, so a 20 s old bearing is still good. In 100-run comparisons at a 40 % drop rate, 20 s cut the runs that said "I can't hear anyone yet" from 42 to 27, with no run lost. At 60 %, runs reaching the cover rose from 81 to 87. Calling every 3 s instead of 4 s gave mixed results within run-to-run noise, so the Demo instructions keep 4 s.

## 5. Beware

1. **Never run with the hardware.** On 2026-11-27 the Pupper listed no capture device at all (`arecord -l`: only the HiFiBerry DAC and HDMI, which are outputs). Its system `python3` has `sounddevice` 0.5.2. A second read-only check later the same day could not reach the Pupper (SSH timed out), so it is not known whether `numpy` imports in that interpreter, and his script has not been started there.
2. **Who gets the microphones.** Once plugged in, the DJI receiver is the Pupper's only capture device. Three things want audio:
   - his script (four channels; it auto-selects the device by name, which is the raw ALSA device `hw:N,0`);
   - the phrase detector (`robot/keyword_node.py`: `config/keyword.yaml`, `device: null` = the system default, 1 channel);
   - possibly Gemini's voice, if its browser runs on the Pupper.

   PipeWire runs on the Pupper (its `pulse` and `default` devices list 32 inputs). A raw ALSA device is normally exclusive: if his script opens `hw:N,0`, PipeWire, and so the phrase detector, cannot open it, and the reverse. **This must be tried with the receiver plugged in.** The likely working arrangement is for both to go through PipeWire: `dji.device: pulse` and `keyword.yaml` `device: pulse`, with the receiver's PipeWire profile set to four channels (`pactl list cards`). In that case check with `test_channels.py --device pulse` that the four channels are still four separate channels and are not mixed down.
3. **Radio timing between transmitters** (4.4). Each transmitter has its own radio link. If the receiver delivers them with a fixed offset, it shows as the same delay error at every angle, and it can be measured with claps from known directions. A per-channel correction would then be a small addition: to his script, or to the bridge as a correction to the delays before the fit. If the offset changes from one connection to the next, or drifts, TDOA from these transmitters will not reach ±5°. Ask Max what his live tests showed.
4. **The threshold is absolute.** `dji.threshold` 0.01 is an RMS over all four channels and 0.35 s. A shout from 6 m behind a board may not reach it (in the synthetic runs a call peaking at 0.03 never did). And if the room's background is above 0.65 × threshold (about −44 dBFS), his script never re-arms: it locates one sound and then nothing more until the room goes quiet. Tune it in the room: record a few calls (`arecord -D hw:<card> -c 4 -r 48000 -f S16_LE calls.wav`, or `test_channels.py`), then run `tools/sound_replay.py calls.wav --threshold …`. The status message carries `last_rms`.
5. **One bearing per call, from the call's onset.** "Help! Help me!" gives one bearing, perhaps two if the gap between the words is long enough. Navigation needs 3 agreeing bearings within 20 s, and in a quiet room about 40 % of calls fail the checks. The survivor's 4 s rhythm (Demo instructions) is therefore needed, and a pause of a second between calls is what lets the next one count.
6. **Pupper's own voice can use up a call.** His script triggers on any loud sound, Pupper's speaker included. If the survivor starts calling before Pupper's line has ended, the script has already triggered on Pupper's voice. That bearing is ignored, because navigation does not listen while Pupper talks. But the script does not re-arm until the room has been quiet for a whole 0.35 s window, so the survivor's first call goes unlocated. The Demo instructions already say to wait for Pupper to finish; Mission Control's banner turns green when it is the survivor's turn.
7. **"uncertain near 0°" does not mean "straight ahead".** That wording comes from the two-microphone script, where near-zero delay does mean broadside. With four microphones in a rectangle, all four delays near zero means no horizontal direction: sound from overhead, one signal copied to every channel, or electrical noise. The bridge publishes no bearing for it.
8. **The rectangle must be measured and square.** If the transmitters are not 120 × 234 mm apart, bearings are distorted, and differently at different angles; set `dji.width_mm` and `dji.height_mm` to what was measured. If the rectangle is turned relative to Pupper's body, every bearing is off by that angle, and from 10° on that loses most runs (4.4); set `topics.sound_zero_deg` (left positive) to correct it. If the channel mapping is wrong (TX2 on CH3, say), the bearings are badly wrong; `test_channels.py` checks this.
9. **Output-format coupling.** The bridge reads his printed line. If he changes it, `test_his_script_end_to_end…` fails on a laptop. Run the tests after pulling his changes.
10. **The interpreter.** `dji.python: python3` is the Pupper's system Python, which is where `sounddevice` is installed. A virtual environment would need numpy and sounddevice too.

## 6. To fix, or to do next

**In his code (recommendations for Max; nothing in his folder was edited):**

1. **Window the frame, or band-limit PHAT, in `gcc_phat_tdoa`.** Either of these took the quiet-room fault from 23 of 60 frames to 0 (4.3):
   ```python
   first = first * np.hanning(len(first)); second = second * np.hanning(len(second))     # after removing the mean
   # and/or, after normalising cross_power:
   freqs = np.fft.rfftfreq(fft_size, 1.0 / sample_rate); cross_power[(freqs < 200) | (freqs > 4000)] = 0
   ```
   `localize_stereo.py` uses the same function and would benefit too.
2. **Use the fourth delay as a check**, not only in the fit: with four microphones in a rectangle, CH4 = CH2 + CH3. Today the fit magnitude can look normal (0.98) when one pair is wrong.
3. **"uncertain near 0°"** in `localize_angle.py` should say "no direction" (5.7).
4. **A machine-readable output** (for example a `--json` flag printing one JSON object per estimate), or the estimate as a function that returns rather than prints. Either would let the bridge stop parsing text. If either is added, `robot/dji_bearing.py` is the one file to change.

**On the robot (in this order, motion off):**

1. Plug in the receiver (Q mode). `arecord -l`; `pactl list cards short`; `pactl list sources short`.
2. `python3 sound-localization/test_channels.py --transmitters 4`: four distinct channels, TX1 → CH1 … TX4 → CH4.
3. `robot/start_demo.sh`. Mission Control's Sound panel shows **Module**: the receiver. Check that the phrase detector still hears (its levels move). If either cannot open the device, see 5.2.
4. Claps at 1 m from straight ahead, right, behind and left, three each. Compare the printed delays with `robot/dji_bearing.expected_delays`: ahead (0, +32.7, +32.7), right (−16.8, 0, −16.8), behind (0, −32.7, −32.7), left (+16.8, 0, +16.8) samples. A constant difference at every angle is a radio offset (5.3). Repeat after switching the transmitters off and on, to see whether it stays the same. **Straight ahead must read within about ±3°**; a constant error here costs runs (4.4).
5. Record calls from the cover at 6 m, and from 2 m, in the demo room. Replay them with `tools/sound_replay.py --truth …` to set `dji.threshold`, and to see how many calls pass the checks. If very few pass and the errors are small, lower `sound.min_confidence` (0.3 lets `fit`-flagged bearings through; 0 lets everything through).
6. The README's "First time on the robot" ladder, rung 3 on.

## 7. How to run what is here

```bash
python3 -m pytest path-navigation/tests -q                                  # needs numpy, pyyaml, pytest
python3 path-navigation/tools/sound_replay.py calls.wav --truth 15          # a recording through his script
python3 path-navigation/sim/course_sim.py --runs 200 --drop 0.4             # the whole run, his module's behaviour
```

On the Pupper, `robot/start_demo.sh` starts it with the rest. On its own:

```bash
source path-navigation/robot/env.sh && python3 path-navigation/robot/sound_node.py
```
