#!/usr/bin/env python3
"""Check whether two or four DJI transmitters arrive on separate USB channels."""
from __future__ import annotations

import argparse
from datetime import datetime
from pathlib import Path
import sys
import time

import numpy as np
import sounddevice as sd

from audio_utils import choose_input_device, list_input_devices, write_pcm16_wav


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--device", help="input device index or unique name fragment")
    parser.add_argument("--transmitters", type=int, choices=(2, 4), default=4,
                        help="number of transmitters to check (default: 4)")
    parser.add_argument("--seconds", type=float, default=4.0, help="record seconds for each transmitter")
    parser.add_argument("--rate", type=int, default=48000, help="capture sample rate")
    parser.add_argument("--output", type=Path, help="WAV path; defaults to recordings/dji_channels_<time>.wav")
    args = parser.parse_args()
    if args.seconds <= 0 or args.rate <= 0:
        parser.error("--seconds and --rate must be positive")

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
    if channels < args.transmitters:
        print(f"Receiver exposes {channels} channels; {args.transmitters} are required for this check.", file=sys.stderr)
        print("Set the Mic 3 receiver to Q (Quadraphonic) mode and check DJI's computer compatibility list.")
        return 1

    recordings = []
    for tx in range(1, args.transmitters + 1):
        input(f"\nSpeak near TX{tx} only. Press Return to record {args.seconds:g} seconds... ")
        print("Recording...", flush=True)
        try:
            block = sd.rec(int(round(args.seconds * args.rate)), samplerate=args.rate,
                           channels=args.transmitters, dtype="float32", device=device_index,
                           blocking=True)
        except Exception as exc:
            print(f"Could not record: {exc}", file=sys.stderr)
            return 1
        recordings.append(block)
        time.sleep(0.15)

    print("\nRMS level by transmitter prompt and recorded channel (dBFS):")
    strongest_channels = []
    for tx, block in enumerate(recordings, start=1):
        levels = 20.0 * np.log10(np.maximum(np.sqrt(np.mean(block.astype(np.float64) ** 2, axis=0)), 1e-12))
        strongest = int(np.argmax(levels))
        strongest_channels.append(strongest)
        columns = "  ".join(f"CH{i + 1}: {level:6.1f}" for i, level in enumerate(levels))
        print(f"  TX{tx} prompt: {columns}  | strongest: CH{strongest + 1}")

    audio = np.concatenate(recordings, axis=0)
    output = args.output or Path("recordings") / f"dji_channels_{datetime.now().strftime('%Y%m%d_%H%M%S')}.wav"
    write_pcm16_wav(output, audio, args.rate)
    print(f"\nSaved {audio.shape[0] / args.rate:.1f} seconds, {args.transmitters} channels to {output.resolve()}")
    if len(set(strongest_channels)) == args.transmitters:
        print("PASS: each transmitter prompt peaked on a different channel.")
    else:
        print("CHECK: prompts did not peak on four unique channels; inspect the WAV and receiver Q-mode setup.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
