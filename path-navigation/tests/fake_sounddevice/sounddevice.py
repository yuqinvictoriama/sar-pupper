"""A stand-in for the `sounddevice` package, so the partner's scripts run unchanged on recorded audio.

Put this folder first on PYTHONPATH and his script imports this instead of the real package:

    FAKE_SD_AUDIO=clip.npy PYTHONPATH=tests/fake_sounddevice python3 sound-localization/localize_angle.py

FAKE_SD_AUDIO     a .npy (float32, shaped (samples, channels)) or a 16-bit PCM .wav
FAKE_SD_NAME      the device name it reports (default "Wireless Mic Rx (replay)", which his auto-select picks)
FAKE_SD_RATE      sample rate it reports (default: the WAV's, or 48000)
FAKE_SD_SPEED     how many times faster than real time to play (default 20)
FAKE_SD_TAIL_S    seconds to wait after the last block before Ctrl+C (default 0.6)

When the audio has been played it interrupts the process as Ctrl+C would, which his scripts catch.
Only what localize_angle.py, localize_stereo.py and test_channels.py use is here.
"""
from __future__ import annotations

import os
import signal
import threading
import time
import wave

import numpy as np

__version__ = "fake"


def _audio():
    path = os.environ.get("FAKE_SD_AUDIO", "")
    if not path:
        return np.zeros((48000, 4), dtype=np.float32), 48000
    if path.endswith(".wav"):
        with wave.open(path, "rb") as w:
            data = np.frombuffer(w.readframes(w.getnframes()), dtype="<i2")
            return (data.reshape(-1, w.getnchannels()) / 32768.0).astype(np.float32), w.getframerate()
    return np.load(path).astype(np.float32), int(os.environ.get("FAKE_SD_RATE", "48000"))


def query_devices():
    audio, rate = _audio()
    return [{"name": "Fake speaker", "max_input_channels": 0, "max_output_channels": 2, "default_samplerate": rate},
            {"name": os.environ.get("FAKE_SD_NAME", "Wireless Mic Rx (replay)"),
             "max_input_channels": int(audio.shape[1]), "max_output_channels": 0,
             "default_samplerate": float(os.environ.get("FAKE_SD_RATE", rate))}]


class InputStream:
    def __init__(self, device=None, channels=1, samplerate=48000, blocksize=1024, dtype="float32",
                 callback=None, **_):
        self.channels, self.blocksize, self.callback = int(channels), int(blocksize), callback
        self.audio, _ = _audio()
        self.thread = None

    def _play(self):
        speed = float(os.environ.get("FAKE_SD_SPEED", "20"))
        rate = float(os.environ.get("FAKE_SD_RATE", "48000"))
        for i in range(0, len(self.audio) - self.blocksize + 1, self.blocksize):
            block = np.ascontiguousarray(self.audio[i:i + self.blocksize, :self.channels])
            self.callback(block, len(block), None, None)
            time.sleep(self.blocksize / rate / speed)
        time.sleep(float(os.environ.get("FAKE_SD_TAIL_S", "0.6")))
        os.kill(os.getpid(), signal.SIGINT)

    def __enter__(self):
        self.thread = threading.Thread(target=self._play, daemon=True)
        self.thread.start()
        return self

    def __exit__(self, *exc):
        return False


def rec(frames, samplerate=48000, channels=1, dtype="float32", device=None, blocking=True, **_):
    audio, _ = _audio()
    out = np.zeros((int(frames), int(channels)), dtype=np.float32)
    n = min(len(audio), int(frames))
    out[:n] = audio[:n, :channels]
    return out
