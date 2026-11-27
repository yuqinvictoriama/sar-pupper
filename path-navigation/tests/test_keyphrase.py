"""Off-robot tests for the "You saved me, Pupper!" detector.

Run:  python3 -m pytest tests/test_keyphrase.py -q
Needs sherpa-onnx, numpy, and the model (tools/get_keyword_model.sh); skipped without them.
The recordings in tests/audio are synthetic voices (see tests/audio/README.md).
"""
from pathlib import Path

import numpy as np
import pytest

from keyphrase import LevelControl, PhraseSpotter, find_model
from keyphrase.audio import read_wav, wav_blocks, write_wav

AUDIO = Path(__file__).parent / "audio"
pytest.importorskip("sherpa_onnx")
try:
    find_model()
except FileNotFoundError as e:  # pragma: no cover
    pytest.skip(str(e), allow_module_level=True)


def hits_in(name, **kw):
    pcm, _ = read_wav(AUDIO / name)
    spotter = PhraseSpotter(channels=pcm.shape[1], **kw)
    return [h for block, rate in wav_blocks(AUDIO / name) for h in spotter.feed(block, rate)]


@pytest.mark.parametrize("name", ["yes_clean_1.wav", "yes_clean_2.wav", "yes_clean_3.wav",
                                  "yes_noisy.wav", "yes_echo.wav", "yes_quiet.wav", "yes_pause.wav"])
def test_hears_the_phrase(name):
    assert len(hits_in(name)) == 1


@pytest.mark.parametrize("name", sorted(p.name for p in AUDIO.glob("no_*.wav")))
def test_ignores_everything_else(name):
    """Pupper's own lines, the cry for help, chatter, and noise."""
    assert hits_in(name) == []


def test_quiet_voice_needs_the_level_control():
    assert hits_in("yes_quiet.wav", level_control=False) == []
    assert len(hits_in("yes_quiet.wav")) == 1


def test_any_microphone_counts_and_48k_is_fine():
    hits = hits_in("yes_second_mic_48k.wav")
    assert len(hits) == 1 and hits[0].channel == 1


def test_repeats_close_together_count_once():
    """Said at about 0.5 s, 2.1 s and 6 s: the second is inside the 2 s hold-off."""
    hits = hits_in("yes_three_times.wav")
    assert len(hits) == 2
    assert hits[1].time_s - hits[0].time_s > 2.0
    assert len(hits_in("yes_three_times.wav", refractory_s=0.5)) == 3


def test_block_size_does_not_matter():
    pcm, rate = read_wav(AUDIO / "yes_clean_1.wav")
    spotter = PhraseSpotter()
    rng = np.random.default_rng(0)
    hits, i = [], 0
    while i < len(pcm):
        n = int(rng.integers(1, 4000))
        hits += spotter.feed(pcm[i:i + n], rate)
        i += n
    assert len(hits) == 1


def test_detector_survives_a_long_quiet_stretch_then_hears():
    pcm, rate = read_wav(AUDIO / "yes_clean_2.wav")
    spotter = PhraseSpotter()
    silence = np.random.default_rng(1).normal(0, 0.002, (rate * 30, 1)).astype(np.float32)
    assert spotter.feed(silence, rate) == []
    assert len(spotter.feed(pcm, rate)) == 1


def test_too_few_channels_is_an_error():
    with pytest.raises(ValueError):
        PhraseSpotter(channels=2).feed(np.zeros(1600, np.float32))


def test_level_control_raises_quiet_and_never_clips():
    lc = LevelControl()
    quiet = np.full(1600, 0.001, np.float32)
    assert abs(float(np.sqrt(np.mean(lc(quiet) ** 2))) - 0.06) < 0.03  # 60x gain cap
    loud = lc(np.full(1600, 0.9, np.float32))
    assert float(np.abs(loud).max()) <= 1.0


def test_wav_round_trip(tmp_path):
    x = np.random.default_rng(2).uniform(-0.5, 0.5, (800, 2)).astype(np.float32)
    write_wav(tmp_path / "a.wav", x, 48000)
    y, rate = read_wav(tmp_path / "a.wav")
    assert rate == 48000 and y.shape == x.shape and np.abs(x - y).max() < 1e-3
