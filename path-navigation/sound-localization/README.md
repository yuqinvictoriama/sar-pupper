# DJI sound localization experiment

This folder holds the standalone Mac/Pi experiments for the DJI Mic 3 four-transmitter receiver. It does not start or move Pupper and does not estimate source distance.

## Files

| File | Purpose |
|---|---|
| `test_channels.py` | Prompt near each transmitter, record separate channels, compare channel RMS, and save a multichannel WAV |
| `audio_utils.py` | Shared input-device selection, GCC-PHAT delay estimation, and WAV writing helpers |
| `localize_angle.py` | Live four-channel TDOA direction estimate for the current rectangle |
| `localize_stereo.py` | Preserved live two-channel TDOA experiment |

## Install

Use the Python environment you will run the scripts with:

```bash
python3 -m pip install numpy sounddevice
```

On the Mac, allow Microphone access for Terminal or VS Code. On the Pi, use the project's Python environment and ensure ALSA exposes the receiver as a four-channel input.

## First: verify four independent channels

Set the receiver to **Q (Quadraphonic)** mode, connect it over USB, then run:

```bash
python3 path-navigation/sound-localization/test_channels.py --transmitters 4
```

Speak near TX1, TX2, TX3, and TX4 separately when prompted. The four prompts should peak on four distinct input channels. The multichannel recording is saved under `recordings/` by default. If the receiver is not auto-selected, add `--device "Wireless Mic Rx"` or use its listed device index.

## Then: live angle estimate

The current assumed physical and channel layout is:

```text
CH1 ───────── CH2     FRONT
 │             │      120 mm across
 │             │      234 mm front-to-back
CH3 ───────── CH4     BACK
```

Confirm TX-to-channel mapping with the channel test and mount each transmitter at the matching corner. Run:

```bash
python3 path-navigation/sound-localization/localize_angle.py --threshold 0.01
```

The printed robot bearing uses **0° forward, positive to the right, ±180° backward, and −90° to the left**. This is a direction-only experiment: it assumes a direct path and a source roughly in the microphone plane. Reflections, source height, channel mapping, and wireless timing can affect the result.

For the original two-microphone live test, run `python3 path-navigation/sound-localization/localize_stereo.py --spacing-mm 120`. It reports the first-heard channel, path-length difference, and broadside angle.

## Later Pupper ROS integration

`path-navigation/robot/sound_adapter.py` expects radians from Pupper's nose with positive angles to the **left**. For integration, convert this script's bearing with `ros_bearing_rad = -math.radians(robot_bearing_deg)`, then publish using the configured sound message type. The scripts here only print bearings; they do not publish ROS messages or command motion.
