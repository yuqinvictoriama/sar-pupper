"""Off-robot tests for tools/sweep_thresholds.py.

    python3 -m unittest tests.test_sweep -v
"""
from __future__ import annotations

import json
import os
import sys
import tempfile
import unittest

import cv2

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
sys.path.insert(0, os.path.join(HERE, "..", "tools"))
sys.path.insert(0, HERE)

import sweep_thresholds as st  # noqa: E402
from dodge.camera_models import build_cameras  # noqa: E402
from dodge.config import load_config  # noqa: E402
from dodge.recording import RecordingWriter  # noqa: E402
from dodge.sim import aimed_throw  # noqa: E402
from test_dodge import render_ball  # noqa: E402

CFG = load_config(use_calib=False)


def rendered_throw(folder: str, bearing: float = 135.0) -> None:
    cam = build_cameras(CFG)["elp_left"]
    throw = aimed_throw(CFG, bearing, 3.5, rise=0.8)
    offset = float(CFG["cameras"]["elp_left"]["time_offset_s"])
    w = RecordingWriter(folder, {"kind": "synthetic"})
    t = 0.0
    while t < throw.t_land:
        img, _ = render_ball(cam, throw.position(t))
        ok, enc = cv2.imencode(".jpg", img, [cv2.IMWRITE_JPEG_QUALITY, 90])
        w.add_frame("elp_left", 1000.0 + t - offset, enc.tobytes())
        t += 1.0 / 60.0
    w.close()


class SweepTests(unittest.TestCase):
    def test_cached_blobs_give_the_same_dodge_as_searching_the_pictures(self):
        with tempfile.TemporaryDirectory() as root:
            rec = os.path.join(root, "throw")
            rendered_throw(rec)
            cache = os.path.join(root, "cache")
            os.makedirs(cache)
            det = st.cached_detection((rec, CFG, cache))
            live = [e for e in det["events"] if e["event"] == "dodge"]
            self.assertEqual(len(live), 1)
            again = st.cached_detection((rec, CFG, cache))           # second time from the cache
            self.assertEqual(len(again["frames"]), len(det["frames"]))
            dodges = st.run_core_on_blobs(again, CFG)
            self.assertEqual(len(dodges), 1)
            self.assertAlmostEqual(dodges[0]["threat"]["tau"], live[0]["threat"]["tau"], places=3)
            # a setting that can never be met: no dodge
            never = st.with_settings(CFG, {"threat.min_climb": 10.0})
            self.assertEqual(st.run_core_on_blobs(again, never), [])
            self.assertEqual(CFG["threat"]["min_climb"], load_config(use_calib=False)["threat"]["min_climb"])

    def test_labels_come_from_label_json_and_the_csv_overrides_them(self):
        with tempfile.TemporaryDirectory() as root:
            for name, word in (("a-dodge", "dodged"), ("b-seen", "not a throw"), ("c-mark", "off target"), ("d-seen", None)):
                os.makedirs(os.path.join(root, name))
                open(os.path.join(root, name, "elp_left.idx"), "w").close()
                if word:
                    with open(os.path.join(root, name, "label.json"), "w") as f:
                        json.dump({"result": word, "note": ""}, f)
            labels_csv = os.path.join(root, "labels.csv")
            with open(labels_csv, "w") as f:
                f.write("# comment\nb-seen,hit\n")
            recs = st.find_recordings([root])
            self.assertEqual(len(recs), 4)
            labels = st.read_labels(recs, labels_csv)
            self.assertEqual(labels, {"a-dodge": "hit", "b-seen": "hit", "c-mark": "miss"})
            # no pictures and nothing cached: left out rather than failing
            self.assertIsNone(st.cached_detection((recs[0], CFG, None)))

    def test_simulated_cases_are_repeatable_and_labelled(self):
        a = st.synthetic_cases(CFG, 2, 3)
        b = st.synthetic_cases(CFG, 2, 3)
        self.assertEqual([c[2] for c in a], [c[2] for c in b])
        self.assertEqual({c[1] for c in a}, {"hit", "miss", "none"})
        floor = next(c for c in a if c[0] == "ball on the floor")
        self.assertFalse(st.run_synthetic((CFG, floor))["triggered"])

    def test_detector_settings_cannot_be_swept_on_cached_blobs(self):
        sys_argv = sys.argv
        try:
            sys.argv = ["sweep_thresholds.py", "--no-real", "--sweep", "detector.min_area_px=8,10"]
            with self.assertRaises(SystemExit):
                st.main()
        finally:
            sys.argv = sys_argv


if __name__ == "__main__":
    unittest.main()
