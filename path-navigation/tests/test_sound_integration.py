"""The partner's sound localisation (sound-localization/, commit ca06155) as navigation uses it.

His script runs unchanged, as a child process, on synthetic four-channel calls from known
directions (tests/dji_synth.py), through a stand-in for the sounddevice package
(tests/fake_sounddevice/). These tests check the joins: his sign convention, the bridge's
parsing and checks, the node, and that his files are still exactly as he committed them.
They say nothing about real microphones, a real room or the DJI radio link.
"""
import importlib
import json
import math
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "robot"))
sys.path.insert(0, str(ROOT / "tests"))

import dji_bearing  # noqa: E402
import fake_ros  # noqa: E402
from dji_bearing import Localizer, expected_delays, parse_line  # noqa: E402
from dji_synth import recording  # noqa: E402
from nav.board import Board  # noqa: E402
from nav.config import DemoConfig, Dji, Topics  # noqa: E402
from sound_adapter import SoundAdapter  # noqa: E402

PARTNER = ROOT / "sound-localization"
PARTNER_COMMIT = "ca06155"
FAKE_SD = ROOT / "tests" / "fake_sounddevice"


def err(a, b):
    return (a - b + 180.0) % 360.0 - 180.0


def run_script(tmp_path, audio, dji=None, timeout=60):
    """His localize_angle.py on `audio` via the bridge; the readings it produced, and the Localizer."""
    path = tmp_path / "calls.npy"
    np.save(path, audio)
    env = dict(os.environ, PYTHONPATH=str(FAKE_SD), FAKE_SD_AUDIO=str(path), FAKE_SD_SPEED="60")
    got = []
    loc = Localizer(dji or Dji(), on_reading=lambda r, t: got.append(r), python=sys.executable, env=env)
    loc.start()
    loc.proc.wait(timeout)
    loc.reader.join(5)
    return got, loc


# ------------------------------------------------------------------ his code is untouched
def test_partner_files_are_exactly_as_committed():
    try:
        listed = subprocess.run(["git", "ls-tree", "--name-only", PARTNER_COMMIT, "path-navigation/sound-localization/"],
                                cwd=ROOT.parent, capture_output=True, text=True, check=True).stdout.split()
    except (OSError, subprocess.CalledProcessError):
        pytest.skip("git or commit %s not available" % PARTNER_COMMIT)
    assert {Path(n).name for n in listed} == {"README.md", "audio_utils.py", "localize_angle.py",
                                              "localize_stereo.py", "test_channels.py"}
    for name in listed:
        committed = subprocess.run(["git", "show", "%s:%s" % (PARTNER_COMMIT, name)], cwd=ROOT.parent,
                                   capture_output=True, check=True).stdout
        assert (ROOT.parent / name).read_bytes() == committed, "%s differs from the partner's commit" % name


# ------------------------------------------------------------------ parsing his output
LINE = ("RMS=0.0312 | delays vs CH1: CH2=  -3.21, CH3= +31.90, CH4= +28.70 samples | bearing=+11.2° | "
        "fit magnitude=0.98")


def test_parse_his_estimate_line():
    r = parse_line(LINE)
    assert r["bearing"] == pytest.approx(11.2) and r["rms"] == pytest.approx(0.0312)
    assert r["delays"] == pytest.approx([-3.21, 31.90, 28.70]) and r["fit"] == pytest.approx(0.98)
    assert r["residual"] == pytest.approx(0.01, abs=1e-6) and r["check"] == "ok" and r["confidence"] == 1.0


def test_uncertain_lines_give_no_bearing_and_other_lines_are_not_estimates():
    near = LINE.replace("bearing=+11.2°", "bearing=uncertain near 0° (angle would be +3.4°)")
    zero = LINE.replace("bearing=+11.2°", "bearing=uncertain (near-zero delays)")
    for line in (near, zero):
        r = parse_line(line)
        assert r["bearing"] is None and r["uncertain"] and r["confidence"] == 0.0
    for line in ("  Delay pattern exceeds the ideal planar-wave geometry; treat bearing cautiously.",
                 "Selected [3] Wireless Mic Rx with 4 input channel(s).", "", "Stopped angle localization."):
        assert parse_line(line) is None
    assert dji_bearing.parse_selected("Selected [3] Wireless Mic Rx: USB Audio (hw:3,0) with 4 input channel(s).") \
        == (3, "Wireless Mic Rx: USB Audio (hw:3,0)", 4)


def test_checks_on_his_numbers():
    inconsistent = LINE.replace("CH4= +28.70", "CH4=  +0.19")                # one pair locked onto a wrong peak
    assert parse_line(inconsistent)["check"] == "inconsistent" and parse_line(inconsistent)["confidence"] == 0.0
    low_fit = LINE.replace("fit magnitude=0.98", "fit magnitude=0.22")
    high_fit = LINE.replace("fit magnitude=0.98", "fit magnitude=1.40")
    for line in (low_fit, high_fit):
        assert parse_line(line)["check"] == "fit" and parse_line(line)["confidence"] == pytest.approx(0.3)
    assert parse_line(LINE, Dji(max_residual_samples=0.001))["check"] == "inconsistent"


def test_expected_delays_are_his_geometry():
    assert expected_delays(0) == pytest.approx([0.0, 32.75, 32.75], abs=0.01)       # back mics hear later
    assert expected_delays(90) == pytest.approx([-16.79, 0.0, -16.79], abs=0.01)    # right mics hear first
    for b in (-150, -40, 0, 25, 90):                                                # a rectangle: CH4 = CH2 + CH3
        d2, d3, d4 = expected_delays(b)
        assert d4 == pytest.approx(d2 + d3, abs=1e-9)


def test_command_line_passes_every_setting():
    cmd = dji_bearing.command(Dji(device="Wireless Mic Rx", threshold=0.02), python="python3")
    assert cmd[:3] == ["python3", "-u", str(PARTNER / "localize_angle.py")]
    assert cmd[cmd.index("--threshold") + 1] == "0.02" and cmd[cmd.index("--device") + 1] == "Wireless Mic Rx"
    assert "--device" not in dji_bearing.command(Dji())                             # "" leaves his auto-select


# ------------------------------------------------------------------ his script, end to end
def test_his_script_end_to_end_sign_convention_and_accuracy(tmp_path):
    """Calls from known directions through his unchanged script, the bridge and the adapter."""
    truth = [-90.0, -30.0, -8.0, 0.0, 12.0, 40.0, 135.0, 180.0]
    audio = np.concatenate([recording(b, range_m=6.0, calls=1, seed=i, noise=0.002) for i, b in enumerate(truth)])
    got, loc = run_script(tmp_path, audio)
    assert len(got) == len(truth), [r["text"] for r in got]
    assert loc.device is not None and loc.device[2] == 4
    adapter = SoundAdapter(Topics())                                   # demo.yaml: deg, positive right
    for b, r in zip(truth, got):
        assert r["check"] == "ok", r
        assert abs(err(r["bearing"], b)) < 6.0, (b, r["text"])
        nav_left_rad = adapter.from_text(json.dumps({"bearing": r["bearing"]}))[0]
        assert abs(err(math.degrees(nav_left_rad), -b)) < 6.0         # navigation: positive LEFT


def test_near_field_and_a_survivor_standing_at_two_metres(tmp_path):
    """The second call comes from about 2 m, a mouth about 1.35 m above the microphones (elevation 34 deg)."""
    truth = [-20.0, 0.0, 15.0]
    audio = np.concatenate([recording(b, range_m=2.0, elevation_deg=34.0, calls=1, seed=10 + i, noise=0.002)
                            for i, b in enumerate(truth)])
    got, _ = run_script(tmp_path, audio)
    assert len(got) == len(truth)
    for b, r in zip(truth, got):
        assert abs(err(r["bearing"], b)) < 6.0 and 0.75 < r["fit"] < 0.9 and r["check"] == "ok", (b, r)


def test_one_bearing_per_call(tmp_path):
    """His script locates a sound once, then waits for quiet: the survivor must pause between calls."""
    got, _ = run_script(tmp_path, recording(10.0, range_m=6.0, calls=3, gap_s=1.2, seed=3, noise=0.002))
    assert len(got) == 3


def test_quiet_room_wrong_pairs_are_caught(tmp_path):
    """Low background noise: his GCC-PHAT sometimes locks one pair onto a zero-lag peak (SOUND_INTEGRATION.md).

    The bridge's checks must flag those; what passes them is within 12 deg in these recordings.
    """
    audio = np.concatenate([recording(25.0, range_m=6.0, calls=3, seed=s, noise=0.0005) for s in (100, 101, 102)])
    got, _ = run_script(tmp_path, audio)
    assert len(got) == 9
    flagged = [r for r in got if r["check"] != "ok"]
    assert flagged, "expected the known failure in these recordings; has his gcc_phat_tdoa changed?"
    for r in got:
        if r["check"] == "ok":
            assert abs(err(r["bearing"], 25.0)) < 12.0, r


def test_script_that_cannot_open_four_channels_is_not_alive_and_is_restarted(tmp_path):
    stereo = recording(0.0, calls=1)[:, :2]
    clock = [0.0]
    path = tmp_path / "stereo.npy"
    np.save(path, stereo)
    env = dict(os.environ, PYTHONPATH=str(FAKE_SD), FAKE_SD_AUDIO=str(path))
    loc = Localizer(Dji(restart_s=3.0), python=sys.executable, env=env, clock=lambda: clock[0])
    loc.start()
    loc.proc.wait(30)
    loc.reader.join(5)
    status = loc.status()
    assert status["alive"] is False and status["exit_code"] == 1
    assert "Four-channel localization needs four USB capture channels" in status["last_message"] or \
        any("Four-channel" in line for line in loc.log)
    assert loc.poll() is False                     # not before restart_s
    clock[0] = 3.5
    assert loc.poll() is True and loc.restarts == 1
    loc.proc.wait(30)
    loc.stop()


# ------------------------------------------------------------------ liveness and the node
def test_a_heartbeat_saying_not_alive_does_not_count():
    board = Board(DemoConfig())
    board.sound_alive(10.0, {"alive": False, "last_message": "Four-channel localization needs ..."})
    assert board.alive(10.5)["sound"] is False and board.sound_status["last_message"].startswith("Four")
    board.sound_alive(11.0, {"alive": True})
    assert board.alive(11.5)["sound"] is True


def test_sound_node_publishes_bearings_and_status(monkeypatch):
    bus = fake_ros.install()
    monkeypatch.setattr(dji_bearing.Localizer, "start", lambda self: None)
    sys.modules.pop("sound_node", None)
    sound_node = importlib.import_module("sound_node")
    node = sound_node.SoundNode()
    node.localizer.feed("Selected [3] Wireless Mic Rx with 4 input channel(s).")
    node.localizer.feed(LINE)
    node.localizer.feed(LINE.replace("bearing=+11.2°", "bearing=uncertain (near-zero delays)"))
    sent = [json.loads(m.data) for m in bus.sent.get("/sound/bearing", [])]
    assert len(sent) == 1 and sent[0]["bearing"] == pytest.approx(11.2) and sent[0]["check"] == "ok"
    node.timers[0][1]()
    status = json.loads(bus.sent["/sound/status"][-1].data)
    assert status["bearings"] == 1 and status["uncertain"] == 1 and status["device"] == "Wireless Mic Rx"
    assert status["alive"] is False                 # no process is running in this test
    assert SoundAdapter(Topics()).from_text(bus.sent["/sound/bearing"][0].data)[0] == pytest.approx(math.radians(-11.2))
