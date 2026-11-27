"""Small shared helpers for the DJI receiver experiments."""
from __future__ import annotations

import wave
from pathlib import Path

import numpy as np
import sounddevice as sd


def list_input_devices() -> list[tuple[int, dict]]:
    devices = [(i, d) for i, d in enumerate(sd.query_devices())
               if int(d["max_input_channels"]) > 0]
    print("Audio input devices:")
    for index, device in devices:
        print(f"  [{index}] {device['name']} — {device['max_input_channels']} input channel(s), "
              f"default rate {device['default_samplerate']:.0f} Hz")
    if not devices:
        print("  No input devices found.")
    return devices


def choose_input_device(devices: list[tuple[int, dict]], selector: str | None = None) -> tuple[int, dict]:
    if selector is not None:
        if selector.isdigit():
            matches = [(i, d) for i, d in devices if i == int(selector)]
        else:
            matches = [(i, d) for i, d in devices if selector.lower() in d["name"].lower()]
    else:
        matches = [(i, d) for i, d in devices
                   if "dji" in d["name"].lower() or "wireless mic rx" in d["name"].lower()]
        if len(matches) != 1 and len(devices) == 1:
            matches = devices
    if len(matches) == 1:
        return matches[0]
    if not matches:
        raise ValueError("Could not uniquely select an input device; pass --device with its index or name.")
    raise ValueError("More than one input matches; pass --device with its index or a unique name fragment.")


def gcc_phat_tdoa(mic1: np.ndarray, mic2: np.ndarray, sample_rate: int,
                  max_delay_seconds: float) -> float:
    """Return tau=t_mic2-t_mic1 in fractional samples, bounded by geometry."""
    first = np.asarray(mic1, dtype=np.float64)
    second = np.asarray(mic2, dtype=np.float64)
    first = first - np.mean(first)
    second = second - np.mean(second)
    fft_size = 1 << (2 * len(first) - 1).bit_length()
    cross_power = np.fft.rfft(second, n=fft_size) * np.conj(np.fft.rfft(first, n=fft_size))
    cross_power /= np.maximum(np.abs(cross_power), 1e-12)
    correlation = np.fft.irfft(cross_power, n=fft_size)
    max_lag = min(int(np.ceil(max_delay_seconds * sample_rate)), len(first) - 1)
    lags = np.arange(-max_lag, max_lag + 1, dtype=int)
    bounded = correlation[np.mod(lags, fft_size)]
    peak = int(np.argmax(bounded))
    fraction = 0.0
    if 0 < peak < len(bounded) - 1:
        left, center, right = bounded[peak - 1:peak + 2]
        denominator = left - 2.0 * center + right
        if abs(denominator) > 1e-12:
            fraction = float(np.clip(0.5 * (left - right) / denominator, -1.0, 1.0))
    limit = max_delay_seconds * sample_rate
    return float(np.clip(float(lags[peak]) + fraction, -limit, limit))


def write_pcm16_wav(path: Path, audio: np.ndarray, sample_rate: int) -> None:
    """Write float audio shaped (samples, channels) as a standard PCM16 WAV."""
    path.parent.mkdir(parents=True, exist_ok=True)
    pcm = (np.clip(np.asarray(audio, dtype=np.float32), -1.0, 1.0) * 32767).astype("<i2")
    with wave.open(str(path), "wb") as wav:
        wav.setnchannels(pcm.shape[1])
        wav.setsampwidth(2)
        wav.setframerate(sample_rate)
        wav.writeframes(pcm.tobytes())
