"""Where audio comes from: a WAV file (tests, tuning) or a live microphone (robot).

Both give the same thing: blocks of float32 audio shaped (n, channels).
"""
from __future__ import annotations

import queue
import wave
from typing import Iterator

import numpy as np


def read_wav(path) -> tuple[np.ndarray, int]:
    """16-bit PCM WAV -> (float32 array shaped (n, channels), sample rate)."""
    with wave.open(str(path), "rb") as w:
        if w.getsampwidth() != 2:
            raise ValueError(f"{path}: need 16-bit PCM, got {8 * w.getsampwidth()}-bit")
        data = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2")
        return (data.reshape(-1, w.getnchannels()) / 32768.0).astype(np.float32), w.getframerate()


def write_wav(path, pcm: np.ndarray, sample_rate: int) -> None:
    pcm = np.asarray(pcm, dtype=np.float32)
    if pcm.ndim == 1:
        pcm = pcm[:, None]
    with wave.open(str(path), "wb") as w:
        w.setnchannels(pcm.shape[1])
        w.setsampwidth(2)
        w.setframerate(sample_rate)
        w.writeframes((np.clip(pcm, -1, 1) * 32767).astype("<i2").tobytes())


def wav_blocks(path, block_s: float = 0.1) -> Iterator[tuple[np.ndarray, int]]:
    pcm, rate = read_wav(path)
    step = int(block_s * rate)
    for i in range(0, len(pcm), step):
        yield pcm[i:i + step], rate


def list_input_devices() -> list[tuple[int, str, int, float]]:
    """(index, name, input channels, default sample rate) for every capture device."""
    import sounddevice as sd

    return [(i, d["name"], d["max_input_channels"], d["default_samplerate"])
            for i, d in enumerate(sd.query_devices()) if d["max_input_channels"] > 0]


def resolve_device(device):
    """None, an index, or part of a device name -> something sounddevice accepts."""
    if device is None or isinstance(device, int):
        return device
    if str(device).isdigit():
        return int(device)
    matches = [i for i, name, *_ in list_input_devices() if str(device).lower() in name.lower()]
    if not matches:
        raise LookupError(f"no input device with '{device}' in its name; try tools/keyword_test.py --list")
    return matches[0]


def mic_blocks(device=None, channels: int = 1, sample_rate: int = 48000,
               block_s: float = 0.1, stop=None) -> Iterator[tuple[np.ndarray, int]]:
    """Blocks from a live microphone until `stop` (a threading.Event) is set."""
    import sounddevice as sd

    q: queue.Queue = queue.Queue(maxsize=50)

    def on_audio(indata, frames, time_info, status):  # runs on the audio thread
        try:
            q.put_nowait(indata.copy())
        except queue.Full:
            pass  # the detector fell behind: drop audio, never block capture

    with sd.InputStream(device=resolve_device(device), channels=channels, samplerate=sample_rate,
                        blocksize=int(block_s * sample_rate), dtype="float32", callback=on_audio):
        while stop is None or not stop.is_set():
            try:
                yield q.get(timeout=0.5), sample_rate
            except queue.Empty:
                continue
