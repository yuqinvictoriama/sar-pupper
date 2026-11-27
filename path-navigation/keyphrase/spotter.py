"""Phrase detector for "You saved me, Pupper!".

Pure Python: no ROS and no audio device. Audio goes in as numpy arrays,
detections come out. The ROS node and the test tool both wrap this.

It uses sherpa-onnx's open-vocabulary keyword spotter (a 3.3 M-parameter
streaming model trained on English), so the phrase is plain text in
config/keyword.yaml and nothing has to be trained.
"""
from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent.parent
MODEL_NAME = "sherpa-onnx-kws-zipformer-gigaspeech-3.3M-2024-01-01"
_STEM = "epoch-12-avg-2-chunk-16-left-64"


def find_model(model_dir: str | os.PathLike | None = None) -> Path:
    """Where the model files are. Raises FileNotFoundError with the fix."""
    candidates = [model_dir, os.environ.get("PUPPER_KWS_MODEL"), HERE / "models" / MODEL_NAME]
    for c in candidates:
        if c and (Path(c) / "tokens.txt").is_file():
            return Path(c)
    raise FileNotFoundError(
        "keyword model not found. Run tools/get_keyword_model.sh "
        f"(expected {HERE / 'models' / MODEL_NAME})"
    )


@dataclass
class Detection:
    phrase: str      # which line of keywords.txt matched, e.g. "YOU_SAVED_ME_PUPPER"
    channel: int     # which microphone heard it
    time_s: float    # seconds of audio fed so far when it fired


class LevelControl:
    """Brings quiet audio up to a steady level before the detector sees it.

    The model was trained on normally recorded speech. A voice from behind
    a board, through a clip-on mic set for a loud room, can be 30 dB below
    that. This follows the loudest recent 100 ms and scales to a target.
    """

    def __init__(self, target_rms: float = 0.08, max_gain: float = 60.0,
                 decay: float = 0.97, floor: float = 1e-4):
        self.target, self.max_gain, self.decay, self.floor = target_rms, max_gain, decay, floor
        self.envelope = floor

    def __call__(self, chunk: np.ndarray) -> np.ndarray:
        rms = float(np.sqrt(np.mean(np.square(chunk, dtype=np.float64)))) if len(chunk) else 0.0
        self.envelope = max(rms, self.envelope * self.decay, self.floor)
        gain = min(self.max_gain, self.target / self.envelope)
        return np.clip(chunk * gain, -1.0, 1.0).astype(np.float32)


class PhraseSpotter:
    """Feed it audio; it returns a Detection each time the phrase is heard.

    One detector stream per microphone. A hit on any microphone counts, and
    hits within `refractory_s` of each other are reported once.
    """

    BLOCK_S = 0.1  # audio is levelled and decoded in pieces this long

    def __init__(self, model_dir=None, keywords_file=None, channels: int = 1,
                 refractory_s: float = 2.0, level_control: bool = True, num_threads: int = 1):
        import sherpa_onnx  # imported here so the rest of the package loads without it

        model = find_model(model_dir)
        keywords_file = Path(keywords_file or HERE / "config" / "keywords.txt")
        if not keywords_file.is_file():
            raise FileNotFoundError(f"{keywords_file} missing: run tools/make_keywords.py")
        self._spotter = sherpa_onnx.KeywordSpotter(
            tokens=str(model / "tokens.txt"),
            encoder=str(model / f"encoder-{_STEM}.onnx"),
            decoder=str(model / f"decoder-{_STEM}.onnx"),
            joiner=str(model / f"joiner-{_STEM}.onnx"),
            keywords_file=str(keywords_file),
            num_threads=num_threads,
        )
        self.channels = channels
        self.refractory_s = refractory_s
        self._streams = [self._spotter.create_stream() for _ in range(channels)]
        self._levels = [LevelControl() if level_control else None for _ in range(channels)]
        self._pending = [np.zeros(0, np.float32) for _ in range(channels)]
        self._fed_s = 0.0
        self._last_hit_s = -1e9
        self.level_db = [-120.0] * channels  # input level before levelling, for a meter

    def feed(self, pcm: np.ndarray, sample_rate: int = 16000) -> list[Detection]:
        """pcm: float32 in [-1, 1], shape (n,) or (n, channels). Any sample rate."""
        pcm = np.asarray(pcm, dtype=np.float32)
        if pcm.ndim == 1:
            pcm = pcm[:, None]
        if pcm.shape[1] < self.channels:
            raise ValueError(f"audio has {pcm.shape[1]} channel(s), detector was built for {self.channels}")
        block = max(1, int(self.BLOCK_S * sample_rate))
        hits: list[Detection] = []
        for ch in range(self.channels):
            self._pending[ch] = np.concatenate([self._pending[ch], pcm[:, ch]])
        while len(self._pending[0]) >= block:
            self._fed_s += block / sample_rate
            for ch in range(self.channels):
                chunk, self._pending[ch] = self._pending[ch][:block], self._pending[ch][block:]
                rms = float(np.sqrt(np.mean(np.square(chunk, dtype=np.float64))))
                self.level_db[ch] = 20 * np.log10(max(rms, 1e-6))
                if self._levels[ch] is not None:
                    chunk = self._levels[ch](chunk)
                stream = self._streams[ch]
                stream.accept_waveform(sample_rate, chunk)
                while self._spotter.is_ready(stream):
                    self._spotter.decode_stream(stream)
                    phrase = self._spotter.get_result(stream)
                    if phrase:
                        self._spotter.reset_stream(stream)
                        if self._fed_s - self._last_hit_s >= self.refractory_s:
                            self._last_hit_s = self._fed_s
                            hits.append(Detection(phrase, ch, round(self._fed_s, 2)))
        return hits
