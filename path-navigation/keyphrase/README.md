# Keyphrase: hearing "You saved me, Pupper!"

The survivor ends the search by saying **"You saved me, Pupper!"**. This is the part that hears it. It runs on the robot, needs no network, and does not depend on the sound-direction module. `plan.md` says how navigation uses it; this file is how to run it.

**Status.** Built and tested off the robot only (23 tests, all on synthetic voices). It has never heard a real microphone, the real room, or a real person. The first job on the robot is step 3 below.

## What is here

| File | What it is |
|---|---|
| `keyphrase/spotter.py` | The detector. Audio in as numpy arrays, detections out. No ROS, no audio device |
| `keyphrase/audio.py` | Reads WAV files and live microphones as the same kind of audio blocks |
| `robot/keyword_node.py` | ROS 2 node. Publishes `/keyword/detected` and a once-a-second `/keyword/status`. **Not run yet**: there is no ROS where this was written |
| `tools/keyword_test.py` | Try it without ROS: on recordings, or live with a level meter. **The live mode is not run yet** for the same reason (no microphone) |
| `tools/get_keyword_model.sh` | Fetches the model (18 MB) into `models/` |
| `tools/make_keywords.py` | Rebuilds `config/keywords.txt` after editing the phrases in `config/keyword.yaml` |
| `config/keyword.yaml` | The phrases, how eagerly to accept them, and which microphone |
| `tests/test_keyphrase.py`, `tests/audio/` | 23 tests and 17 short synthetic recordings |

## How it works

- A small English speech model (sherpa-onnx's keyword spotter, 3.3 million parameters) listens continuously and fires when the sounds match one of the phrases in `config/keyword.yaml`. The phrases are plain text; nothing is trained.
- "Pupper" is not a dictionary word, so the list holds what the model tends to hear instead: *Pupper, Pepper, Puppy, Papa*. All of them need "saved me" in front.
- Quiet audio is brought up to a steady level first. Without that, a voice turned down by 30–40 dB was mostly missed in testing.
- One detector per microphone channel; a hit on any counts. Hits within 2 s of each other are reported once.
- It only reports. Navigation decides whether a report ends the search (it must be past 4 m along the line).

## Set up

```bash
pip install sherpa-onnx sounddevice numpy pyyaml      # on Ubuntu 24.04 add --break-system-packages, or use the venv the other nodes use
bash tools/get_keyword_model.sh
python3 -m pytest tests/test_keyphrase.py -q          # 23 passed (laptop or robot; pytest needed)
```

Check that `pip` finds a ready-built `sherpa-onnx` for the Pi (64-bit ARM) before demo week; this was only installed on a desktop-class machine.

## Bring-up on the robot

1. `python3 tools/keyword_test.py --list` – find the microphone. Put part of its name in `config/keyword.yaml` (`audio.device`), and its channel count and sample rate.
2. `python3 tools/keyword_test.py --wav tests/audio/yes_clean_1.wav` – should print one hit. Proves the model runs on the Pi.
3. **The real test.** Robot standing under its walking controller, survivor behind the real cover, in the demo room:
   `python3 tools/keyword_test.py --live --record heard.wav`
   The survivor says the phrase ten times, 5 s apart. Want: 9 or 10 heard. Then two minutes of everything else (the cry for help, Pupper's own lines through its speaker, people talking, balls landing). Want: none heard. Keep `heard.wav`.
4. Watch the level meter while the survivor speaks. Peaks between about −30 and −10 dB are healthy. Pinned at the top means the mic gain is too high; below −50 means move the mic or raise its gain.
5. With the ROS stack up: `python3 robot/keyword_node.py`, then `ros2 topic echo /keyword/detected` and say the phrase.

**Who owns the microphones** is the open question (see the plan). If the sound-direction node already holds the device and this node cannot open it too, the choices are: share the device through the system's sound server, give this node its own microphone, or have the direction node pass its audio to `PhraseSpotter.feed()`, which takes arrays from anywhere.

## Tuning

Everything is in `config/keyword.yaml`; run `python3 tools/make_keywords.py` after a change (needs `pip install sentencepiece`, laptop is fine) and copy `config/keywords.txt` to the robot.

| Symptom | Change |
|---|---|
| Survivor has to repeat it | `boost` up toward 4.0, `threshold` down toward 0.04. Check the level meter first |
| It fires on something else | `threshold` up toward 0.25, `boost` down toward 1.5; or remove the phrase variant named in the detection |
| Never fires on the robot, fires on the test files | Wrong device or channel, or the level is far too low |

Do not add the bare phrase "YOU SAVED ME". It also fired on "he saved me a slice of pizza" and "you shaved me".

## What was measured

Synthetic voices only (40 different speakers), with simulated noise and echo. Shipped settings.

| Condition | Phrase heard |
|---|---|
| Clean | 40 of 40 |
| Hiss 15 dB below the voice | 36–38 of 40 |
| Hiss 10 dB below the voice | 32–35 of 40 |
| Hiss 5 dB below the voice | 20 of 40 |
| Room echo, short (0.3 s) and long (0.6 s) | 38–39 of 40 |
| Said slowly, quickly, or with a pause after "me" (clean, noisy, echo) | 175 of 180 |

| Other sound | False detections |
|---|---|
| 26 sentences × 6 voices × 3 conditions: Pupper's own lines, "Help! Help me!", demo chatter | 0 of 468 |
| 60 more sentences × 5 voices × 3 conditions, written to be confusable | 7 of 900, all on two sentences: "Are you safe? Me and Pupper are fine." and "You saved my puppy from the rain." |
| 15 minutes of noise (hiss, rumble, thumps) | 0 |

- **Noise is the weak point, not echo.** The demo script has the survivor repeat the phrase every 3 s; at 80 % per try, three tries reach 99 %.
- **Speed**: 20 s of audio takes 0.45 s for one microphone and 1.6 s for four, on a desktop core. The Pi is slower but has plenty of room.
- **Not known**: real voices, shouting, accents, the clip-on mics, motor noise while standing, the real cover and room.

## Limits

- English, and one phrase. A different phrase means editing the YAML and re-testing, not retraining.
- Sentences that sound like the phrase can trigger it. Nobody near the robot should say "saved me" during a run, and Pupper's own lines must not contain it.
- The test voices are text-to-speech. Real results will differ; step 3 above is what counts.
