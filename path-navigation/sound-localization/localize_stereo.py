#!/usr/bin/env python3
"""Preserved live TDOA angle test for a two-channel DJI receiver."""
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
    parser.add_argument("--rate", type=int, default=48000)
    parser.add_argument("--spacing-mm", type=float, default=120.0)
    parser.add_argument("--threshold", type=float, default=0.01)
    parser.add_argument("--frame-seconds", type=float, default=0.35)
    parser.add_argument("--deadband-samples", type=float, default=1.5)
    parser.add_argument("--swap-channels", action="store_true",
                        help="analyze input CH2 as Mic 1 and input CH1 as Mic 2")
    args = parser.parse_args()
    if min(args.rate, args.spacing_mm) <= 0 or args.frame_seconds <= 0 or args.threshold < 0:
        parser.error("rate, spacing, and frame duration must be positive; threshold cannot be negative")

    devices = list_input_devices()
    if not devices:
        return 1
    try:
        device_index, device = choose_input_device(devices, args.device)
    except ValueError as exc:
        print(f"Error: {exc}", file=sys.stderr)
        return 2
    if int(device["max_input_channels"]) < 2:
        print("Stereo localization requires two input channels.", file=sys.stderr)
        return 1

    spacing = args.spacing_mm / 1000.0
    max_delay = spacing / 343.0
    frame_size = max(256, int(round(args.frame_seconds * args.rate)))
    hop_size = max(1, int(round(0.10 * args.rate)))
    blocks: queue.Queue[np.ndarray] = queue.Queue()
    statuses: list[str] = []

    def callback(indata, frames, timing, status):
        if status:
            statuses.append(str(status))
        blocks.put(indata.copy())

    mic1_ch, mic2_ch = (1, 0) if args.swap_channels else (0, 1)
    print(f"\nSelected [{device_index}] {device['name']}; spacing {spacing * 1000:.0f} mm.")
    print(f"Analysis mapping: Mic 1=CH{mic1_ch + 1}, Mic 2=CH{mic2_ch + 1}.")
    print("tau = t_Mic2 - t_Mic1; positive means Mic 1 heard first.")
    print("Angle: 0° broadside; negative toward Mic 1, positive toward Mic 2.")
    print("Press Ctrl+C to stop.\n")

    rolling = np.empty((0, 2), dtype=np.float32)
    samples_since_estimate = 0
    armed = True
    try:
        with sd.InputStream(device=device_index, channels=2, samplerate=args.rate,
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
                delay = gcc_phat_tdoa(frame[:, mic1_ch], frame[:, mic2_ch], args.rate, max_delay)
                path_diff = 343.0 * delay / args.rate
                angle = float(np.degrees(np.arcsin(np.clip(-path_diff / spacing, -1.0, 1.0))))
                if abs(delay) <= args.deadband_samples:
                    first, angle_label = "near-simultaneous", "uncertain near 0°"
                else:
                    first, angle_label = ("Mic 1" if delay > 0 else "Mic 2"), f"{angle:+.1f}°"
                print(f"RMS={rms:.4f} | delay={delay:+.2f} samples ({delay / args.rate * 1000:+.3f} ms) | "
                      f"path diff r2-r1={path_diff * 100:+.2f} cm | first={first} | angle={angle_label}",
                      flush=True)
                armed = False
    except KeyboardInterrupt:
        print("\nStopped stereo localization.")
    except Exception as exc:
        print(f"Could not start two-channel input: {exc}", file=sys.stderr)
        return 1
    if statuses:
        print(f"Audio stream status: {', '.join(statuses[-3:])}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
