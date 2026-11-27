#!/usr/bin/env python3
"""Experimental four-microphone direction estimate from a DJI Mic 3 receiver."""
from __future__ import annotations

import argparse
import queue
import sys

import numpy as np
import sounddevice as sd

from audio_utils import choose_input_device, gcc_phat_tdoa, list_input_devices


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", help="input device index or unique name fragment")
    parser.add_argument("--rate", type=int, default=48000, help="capture sample rate")
    parser.add_argument("--width-mm", type=float, default=120.0, help="left/right mic spacing")
    parser.add_argument("--height-mm", type=float, default=234.0, help="front/back mic spacing")
    parser.add_argument("--threshold", type=float, default=0.01, help="event RMS threshold, linear 0–1")
    parser.add_argument("--frame-seconds", type=float, default=0.35, help="analysis window duration")
    parser.add_argument("--deadband-samples", type=float, default=1.5,
                        help="report direction as uncertain below this fitted delay magnitude")
    args = parser.parse_args()
    if min(args.rate, args.width_mm, args.height_mm) <= 0 or args.frame_seconds <= 0 or args.threshold < 0:
        parser.error("rate, dimensions, and frame duration must be positive; threshold cannot be negative")

    devices = list_input_devices()
    if not devices:
        return 1
    try:
        device_index, device = choose_input_device(devices, args.device)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2
    channels = int(device["max_input_channels"])
    print(f"\nSelected [{device_index}] {device['name']} with {channels} input channel(s).")
    if channels < 4:
        print("Four-channel localization needs four USB capture channels. Set the receiver to Q mode.", file=sys.stderr)
        return 1

    width, height = args.width_mm / 1000.0, args.height_mm / 1000.0
    # Mic 1/2 are at the front; Mic 3/4 are at the back.
    # Channel mapping: CH1 front-left, CH2 front-right, CH3 back-left, CH4 back-right.
    positions = np.array([[-width / 2, -height / 2], [width / 2, -height / 2],
                          [-width / 2, height / 2], [width / 2, height / 2]], dtype=float)
    diagonal = float(np.linalg.norm([width, height]))
    max_delay = diagonal / 343.0
    frame_size = max(256, int(round(args.frame_seconds * args.rate)))
    hop_size = max(1, int(round(0.10 * args.rate)))
    blocks: queue.Queue[np.ndarray] = queue.Queue()
    statuses: list[str] = []

    def callback(indata, frames, timing, status):
        if status:
            statuses.append(str(status))
        blocks.put(indata.copy())

    print("\nChannel layout (confirm with test_channels.py before using bearings):")
    print("  CH1 ───────── CH2    FRONT")
    print("   │             │")
    print("  CH3 ───────── CH4    BACK")
    print(f"  Width {width * 1000:.0f} mm × front/back {height * 1000:.0f} mm")
    print("Bearing: 0° front; positive turns right; +90° right; ±180° back; -90° left.")
    print("TDOA sign: each printed delay is t_channel - t_CH1; positive means that channel heard later.")
    print("Model assumes a direct sound path and a source roughly in the mic plane. Press Ctrl+C to stop.\n")

    rolling = np.empty((0, 4), dtype=np.float32)
    samples_since_estimate = 0
    armed = True
    try:
        with sd.InputStream(device=device_index, channels=4, samplerate=args.rate,
                            blocksize=1024, dtype="float32", callback=callback):
            while True:
                try:
                    block = blocks.get(timeout=1.0)
                except queue.Empty:
                    continue
                rolling = np.concatenate((rolling, block), axis=0)
                if len(rolling) > frame_size:
                    rolling = rolling[-frame_size:]
                samples_since_estimate += len(block)
                if len(rolling) < frame_size or samples_since_estimate < hop_size:
                    continue
                samples_since_estimate %= hop_size
                frame = rolling.astype(np.float64, copy=False)
                rms = float(np.sqrt(np.mean(frame * frame)))
                if rms < args.threshold * 0.65:
                    armed = True
                    continue
                if rms < args.threshold or not armed:
                    continue

                delays = np.array([
                    gcc_phat_tdoa(frame[:, 0], frame[:, channel], args.rate, max_delay)
                    for channel in range(1, 4)
                ])
                # t_i - t_1 = -(p_i - p_1) dot u / c; u points toward the source.
                geometry = -(positions[1:] - positions[0])
                rhs = 343.0 * delays / args.rate
                direction, *_ = np.linalg.lstsq(geometry, rhs, rcond=None)
                fit_magnitude = float(np.linalg.norm(direction))
                if fit_magnitude < 1e-9:
                    angle_text = "uncertain (near-zero delays)"
                else:
                    # Coordinates: x right, y back. Convert to robot bearing: 0 front, + clockwise/right.
                    angle = float(np.degrees(np.arctan2(direction[0], -direction[1])))
                    if fit_magnitude < args.deadband_samples * 343.0 / (args.rate * min(width, height)):
                        angle_text = f"uncertain near 0° (angle would be {angle:+.1f}°)"
                    else:
                        angle_text = f"{angle:+.1f}°"
                print(f"RMS={rms:.4f} | delays vs CH1: CH2={delays[0]:+7.2f}, "
                      f"CH3={delays[1]:+7.2f}, CH4={delays[2]:+7.2f} samples | "
                      f"bearing={angle_text} | fit magnitude={fit_magnitude:.2f}", flush=True)
                if fit_magnitude > 1.25:
                    print("  Delay pattern exceeds the ideal planar-wave geometry; treat bearing cautiously.")
                elif fit_magnitude < 0.15:
                    print("  Very small delay pattern; bearing is sensitive to timing noise.")
                armed = False
    except KeyboardInterrupt:
        print("\nStopped angle localization.")
    except Exception as exc:
        print(f"Could not start four-channel input: {exc}", file=sys.stderr)
        print("Check macOS/Linux microphone permissions, USB data cable, and receiver Q mode.")
        return 1
    if statuses:
        print(f"Audio stream status: {', '.join(statuses[-3:])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
