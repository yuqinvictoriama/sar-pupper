"""The whole run in simulation (sim/course_sim.py): the real mission and the real terrain-agility
state machine against a simulated room. Skipped where numpy or terrain-agility is missing."""
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
pytest.importorskip("numpy")
try:
    from sim.course_sim import CourseSim, SimOptions
except ImportError as exc:                              # terrain-agility not beside this folder
    pytest.skip(f"course simulator not importable: {exc}", allow_module_level=True)

from nav.config import DemoConfig, apply  # noqa: E402

ORDER = ["start", "heard", "line_set", "box_landmark", "stepping_over", "crossed", "refix", "arrived", "saved"]
SLAB = {"name": "slab", "shape": "flat", "depth": 0.38, "width": 0.565, "height": 0.015}
CARTON = {"name": "carton", "shape": "flat", "depth": 0.348, "width": 0.455, "height": 0.13}
CYLINDER = {"name": "cylinder", "shape": "round", "depth": 0.05, "width": 0.70, "height": 0.05}


def test_one_run_goes_through_every_stage_in_order():
    res = CourseSim(DemoConfig(), SimOptions(seed=1)).run()
    names = [n for _, n in res["events"]]
    at = [names.index(n) for n in ORDER]
    assert at == sorted(at), names
    assert res["reached"] and res["throws"] >= 3
    # the course as laid out: the low bar is stepped over, then the slab is scaled across. Two crossings.
    assert res["crossings"] == 2 and res["walk_arounds"] == 0 and names.count("stepping_over") == 2


def test_a_crossing_then_a_walk_around_still_works():
    """The earlier course: the slab, then a carton too tall to cross."""
    cfg = apply(DemoConfig(), {"course": {"obstacles": [dict(SLAB, near=0.50), dict(CARTON, near=1.60)]}})
    results = [CourseSim(cfg, SimOptions(seed=k)).run() for k in range(8)]
    assert all(r["reached"] and r["crossings"] == 1 and r["walk_arounds"] == 1 for r in results), results
    assert min(r["clear_tall"] for r in results) > 0.12                 # never touched the carton


def test_a_round_bar_is_never_crossed():
    """A cylinder is measured and refused by terrain agility (no flat top to stand on). This pins down
    that the mission does not count on a crossing there; it does NOT show the cylinder is passed
    reliably: in simulation about half the runs stall in front of it."""
    cfg = apply(DemoConfig(), {"course": {"obstacles": [dict(CYLINDER, near=0.45), dict(SLAB, near=2.30)],
                                          "refix_at": 3.3, "throw_from": 4.2}})
    for seed in range(4):
        sim = CourseSim(cfg, SimOptions(seed=seed))
        res = sim.run(max_time=120.0)
        assert 0 not in sim.crossed, res            # obstacle 0, the cylinder, was not crossed


def test_it_ends_in_front_of_the_cover_in_nearly_every_room():
    results = [CourseSim(DemoConfig(), SimOptions(seed=k)).run() for k in range(30)]
    reached = sum(r["reached"] for r in results)
    assert reached >= 29, [(k, r["state"], r["reason"], r["x"], r["y"]) for k, r in enumerate(results) if not r["reached"]]
    assert max(abs(r["y"]) for r in results if r["reached"]) < 0.6       # inside a 1.2 m cover
    assert all(r["crossings"] == 2 for r in results)                    # the bar and the slab, every time


def test_without_the_phrase_it_still_stops_at_the_cover():
    sim = CourseSim(DemoConfig(), SimOptions(seed=3, says_phrase=False))
    res = sim.run(max_time=420.0)
    assert res["state"] == "ARRIVE" and 5.3 < res["x"] < 6.3 and res["clear_cover"] > 0.1
    names = [n for _, n in res["events"]]
    assert names.count("arrived") >= 2 and "hold" in names and "saved" not in names
    sim.command("phrase")                               # the operator's button
    for _ in range(40):
        sim.step()
    assert sim.mission.state == "RESCUE"


def test_without_the_second_call_it_keeps_its_first_line():
    res = CourseSim(DemoConfig(), SimOptions(seed=4, second_call=False)).run()
    names = [n for _, n in res["events"]]
    assert "refix_timeout" in names and res["state"] == "RESCUE"


def test_dodging_is_armed_only_in_the_gauntlet_and_terrain_never_steers_there():
    sim = CourseSim(DemoConfig(), SimOptions(seed=5))
    sim.step(); sim.command("motion_on"); sim.command("start!")
    seen = set()
    while sim.t < 400 and sim.mission.state != "RESCUE":
        sim.step()
        seen.add((sim.mission.state, sim.board.wanted["arm_dodge"], sim.board.wanted["terrain_mode"]))
        if sim.mission.state == "ZONE2" and not sim.mission.blocked_avoiding:
            assert sim.board.mux_source != "/terrain_cmd_vel"
    armed_in = {s for s, armed, _ in seen if armed}
    assert armed_in <= {"ZONE2", "ARRIVE"} and "ZONE2" in armed_in
    assert {m for s, _, m in seen if s == "ZONE1"} == {"full", "avoid"}
    assert {m for s, _, m in seen if s == "SEEK"} == {"off"}


def test_a_tap_on_the_joystick_pauses_and_the_run_carries_on():
    sim = CourseSim(DemoConfig(), SimOptions(seed=6, throws=False))
    sim.step(); sim.command("motion_on"); sim.command("start!")
    while sim.mission.state != "ZONE2" and sim.t < 300:
        sim.step()
    sim.command("nudge:0,0.3")
    states = set()
    for _ in range(80):
        sim.step(); states.add(sim.mission.state)
    assert "PAUSED" in states and sim.mission.state == "ZONE2"
    res = sim.run(max_time=400.0, start="resume")
    assert res["state"] == "RESCUE"


def test_snapshot_is_what_mission_control_draws():
    import json
    sim = CourseSim(DemoConfig(), SimOptions(seed=7))
    sim.step(); sim.command("motion_on"); sim.command("start!")
    for _ in range(600):
        sim.step()
    snap = json.loads(json.dumps(sim.snapshot()))
    for key in ("mode", "nav", "course", "sound", "keyword", "terrain", "dodge", "mux", "events", "truth", "limits", "alive", "said"):
        assert key in snap, key
    assert snap["nav"]["cue"]["text"] and snap["limits"]["require"]
