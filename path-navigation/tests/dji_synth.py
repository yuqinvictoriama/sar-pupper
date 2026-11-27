"""Synthetic four-channel recordings of a call from a known direction, for testing localize_angle.py.

The microphone layout is the one his script assumes (CH1 front-left, CH2 front-right,
CH3 back-left, CH4 back-right; 120 mm across, 234 mm front to back). Each channel gets
the same burst, delayed by the exact path length from a point source, so the near field
and a source above the microphones are modelled, not only the plane wave his script fits.
Not modelled: reflections, wireless timing between transmitters, unequal channel gains.
"""
from __future__ import annotations

import math

import numpy as np

C = 343.0


def mic_positions(width_mm=120.0, height_mm=234.0):
    """(4, 3) in metres: x right, y back, z up; centre of the rectangle at the origin."""
    w, h = width_mm / 1000.0, height_mm / 1000.0
    return np.array([[-w / 2, -h / 2, 0], [w / 2, -h / 2, 0], [-w / 2, h / 2, 0], [w / 2, h / 2, 0]])


def source_position(bearing_right_deg, range_m=50.0, elevation_deg=0.0):
    """A point at this bearing (0 = nose, positive right), range from the centre, elevation above the mic plane."""
    a, e = math.radians(bearing_right_deg), math.radians(elevation_deg)
    return np.array([range_m * math.cos(e) * math.sin(a), -range_m * math.cos(e) * math.cos(a),
                     range_m * math.sin(e)])


def burst(seconds=0.6, rate=48000, seed=0, low=300.0, high=3500.0):
    """Band-limited noise with a voice-like rise and fall: a stand-in for a shout of "Help!"."""
    rng = np.random.default_rng(seed)
    n = int(seconds * rate)
    spec = np.fft.rfft(rng.standard_normal(n))
    f = np.fft.rfftfreq(n, 1.0 / rate)
    spec[(f < low) | (f > high)] = 0
    x = np.fft.irfft(spec, n)
    env = np.sin(np.linspace(0, math.pi, n)) ** 2       # smooth onset: a sharp one rings in every channel alike
    x = x * env
    return x / np.max(np.abs(x))


def delayed(signal, delay_samples, total):
    """`signal` placed `delay_samples` (fractional) into a buffer of `total` samples."""
    n = 1 << int(math.ceil(math.log2(total + len(signal))))
    padded = np.zeros(n)
    padded[:len(signal)] = signal
    f = np.fft.rfftfreq(n)
    shifted = np.fft.irfft(np.fft.rfft(padded) * np.exp(-2j * math.pi * f * delay_samples), n)
    return shifted[:total]


def recording(bearing_right_deg, range_m=50.0, elevation_deg=0.0, calls=1, rate=48000, lead_s=0.8, gap_s=1.2,
              level=0.2, noise=0.0005, seed=0, width_mm=120.0, height_mm=234.0, channel_offsets=(0, 0, 0, 0)):
    """(samples, 4) float32: silence, then `calls` bursts from the source, each followed by `gap_s` of quiet.

    channel_offsets: extra delay per channel in samples, to model wireless timing between transmitters.
    """
    mics = mic_positions(width_mm, height_mm)
    src = source_position(bearing_right_deg, range_m, elevation_deg)
    dist = np.linalg.norm(mics - src, axis=1)
    rel = (dist - dist.min()) / C * rate                      # samples after the first microphone
    rng = np.random.default_rng(seed + 1000)
    one = burst(rate=rate, seed=seed)
    per_call = int((len(one) / rate + gap_s) * rate)
    total = int(lead_s * rate) + calls * per_call
    out = noise * rng.standard_normal((total, 4))
    for k in range(calls):
        start = int(lead_s * rate) + k * per_call
        for ch in range(4):
            out[:, ch] += level * delayed(one, start + rel[ch] + channel_offsets[ch], total)
    return out.astype(np.float32)
