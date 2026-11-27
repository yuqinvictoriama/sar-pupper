"""The mission logic, off the robot: angles, the position estimate, line following, and every
state of nav/mission.py driven with hand-made inputs. No ROS, no numpy."""
import math
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from nav import events as EV  # noqa: E402
from nav import mission as M  # noqa: E402
from nav.config import DemoConfig, apply, load_config  # noqa: E402
from nav.heading import FixCollector, circular_median, diff, wrap  # noqa: E402
from nav.reckon import Reckoner  # noqa: E402
from nav.steer import line_command  # noqa: E402

D = math.radians
ALIVE = {"imu": True, "walking": True, "sound": True, "keyword": True, "terrain": True, "dodge": True, "voice": True}


# ------------------------------------------------------------------ angles
def test_angles_wrap():
    assert abs(diff(D(179), D(-179)) - D(-2)) < 1e-9
    assert abs(wrap(D(540)) - D(180)) < 1e-9


def test_median_ignores_an_echo():
    assert abs(circular_median([D(10), D(12), D(95)]) - D(12)) < 1e-9
    assert abs(circular_median([D(178), D(-179), D(-177)]) - D(-179)) < 1e-9


def test_a_fix_needs_bearings_that_agree():
    c = FixCollector(3, D(8), 14.0)
    c.add(0.0, D(20)); c.add(1.0, D(80))
    assert c.fix(1.0) is None
    c.add(2.0, D(23)); c.add(3.0, D(18))
    f = c.fix(3.0)
    assert f.count == 3 and f.rejected == 1 and abs(f.heading - D(20.33)) < D(0.1)
    assert c.fix(20.0) is None                      # old bearings are forgotten


def test_two_bearings_that_disagree_never_average():
    c = FixCollector(1, D(8), 14.0)
    c.add(0.0, D(0)); c.add(0.5, D(40))
    assert abs(c.best(1.0, prior=D(3)).heading - D(0)) < 1e-9       # the one nearer where the voice should be
    assert abs(c.best(1.0, prior=D(35)).heading - D(40)) < 1e-9


# ------------------------------------------------------------------ where Pupper is
def walk(r, seconds, cmd, yaw=0.0, t0=0.0, legs=True):
    n = int(round(seconds * 20))
    for k in range(n + 1):
        r.update(t0 + 0.05 * k, cmd, yaw, legs)
    return t0 + seconds


def test_reckoning_a_straight_walk():
    r = Reckoner(lag_s=0.0)
    r.update(0.0, None, 0.0)
    r.set_line(0.0, 0.0)
    walk(r, 4.0, (0.25, 0.0, 0.0))
    assert abs(r.s - 1.0) < 0.02 and abs(r.d) < 1e-6


def test_reckoning_uses_the_speed_factor_and_the_yaw():
    r = Reckoner(forward_factor=0.8, lag_s=0.0)
    r.update(0.0, None, D(90))
    r.set_line(D(90), 0.0)
    walk(r, 4.0, (0.25, 0.0, 0.0), yaw=D(90))
    assert abs(r.s - 0.8) < 0.02


def test_a_sideways_dodge_shows_as_offset_and_a_backward_one_as_lost_distance():
    r = Reckoner(lag_s=0.0)
    r.update(0.0, None, 0.0)
    r.set_line(0.0, 2.0)
    t = walk(r, 0.8, (0.0, 0.5, 0.0))
    assert abs(r.d - 0.4) < 0.03 and abs(r.s - 2.0) < 1e-6
    walk(r, 0.5, (-0.8, 0.0, 0.0), t0=t)
    assert abs(r.s - 1.6) < 0.05


def test_nothing_is_counted_while_the_legs_are_not_listening():
    r = Reckoner(lag_s=0.0)
    r.update(0.0, None, 0.0)
    r.set_line(0.0, 0.0)
    walk(r, 3.0, (0.25, 0.0, 0.0), legs=False)
    assert abs(r.s) < 1e-9
    r.advance(0.58)                                 # the crossing itself
    assert abs(r.s - 0.58) < 1e-9


def test_the_box_is_a_landmark_and_a_redrawn_line_keeps_the_distance():
    r = Reckoner(lag_s=0.0)
    r.update(0.0, None, 0.0)
    r.set_line(0.0, 0.0)
    walk(r, 1.0, (0.25, 0.0, 0.0))
    r.set_s(0.24)
    assert abs(r.s - 0.24) < 1e-9
    r.y += 0.3                                      # pushed off the line
    assert abs(r.d - 0.3) < 1e-9
    r.set_line(D(-5))
    assert abs(r.s - 0.24) < 1e-9 and abs(r.d) < 1e-9
    x, y, yaw = r.course_pose()
    assert abs(y - 0.3) < 1e-9 and abs(r.course_line()["heading"] - D(-5)) < 1e-9


# ------------------------------------------------------------------ line following
def test_steering_goes_back_to_the_line():
    w = DemoConfig().walk
    assert line_command(0.0, 0.0, 0.25, w) == (0.25, 0.0, 0.0)
    vx, vy, wz = line_command(0.3, 0.0, 0.25, w)    # left of the line: turn right
    assert wz < 0 and vy == 0.0 and 0.1 <= vx <= 0.25
    vx, _, wz = line_command(0.0, D(60), 0.25, w)   # facing well left: turn on the spot, to the right
    assert vx == 0.0 and wz == -w.max_yaw_rate
    _, _, wz = line_command(5.0, 0.0, 0.25, w)      # far off: never more than 25 degrees off the line
    assert abs(wz) <= w.max_yaw_rate


# ------------------------------------------------------------------ config and words
def test_config_refuses_names_it_does_not_know():
    with pytest.raises(KeyError):
        apply(DemoConfig(), {"course": {"lenght": 6.0}})
    with pytest.raises(KeyError):
        apply(DemoConfig(), {"corse": {}})
    assert apply(DemoConfig(), {"walk": {"cruise": "0.3"}}).walk.cruise == 0.3


def test_demo_yaml_matches_the_built_in_defaults():
    pytest.importorskip("yaml")
    cfg, base = load_config().to_dict(), DemoConfig().to_dict()
    cfg.pop("source"); base.pop("source")
    assert cfg == base, "config/demo.yaml and nav/config.py differ: change both, or say why in the README"


def test_the_lines_are_the_personalitys_and_never_say_the_phrase():
    assert set(EV.LINES) == set(EV.FALLBACK_LINES)
    if "callouts.py" in EV.LINES_SOURCE:
        assert EV.LINES == EV.FALLBACK_LINES, "personality/callouts.py changed a line: copy it into nav/events.py"
    for name, line in list(EV.LINES.items()) + [("dodge", EV.DODGE_LINE)]:
        assert "saved me" not in line.lower(), name
    assert EV.speak_time("refix") == 4.0 and EV.speak_time("fix") == 0.0


# ------------------------------------------------------------------ the mission, state by state
class Rig:
    """Feeds the mission by hand. The 'robot' stands wherever the test says it stands."""

    def __init__(self, motion=True, cfg=None, alive=None):
        self.m = M.Mission(cfg or DemoConfig(), motion=motion)
        self.t = 100.0
        self.yaw = 0.0
        self.alive = dict(alive or ALIVE)
        self.terrain = {"alive": True, "state": "CRUISE", "mode": "off", "scan_fresh": True}
        self.dodge = {"alive": True, "state": "SEARCH", "armed": False}
        self.obstacles = []
        self.mux = "none"
        self.sent = None                            # the last command the mux passed to the legs
        self.events = []
        self.out = None

    def tick(self, seconds=0.05, **kw):
        n = max(1, int(round(seconds * 20)))
        for _ in range(n):
            self.t += 0.05
            inp = M.Inputs(t=self.t, yaw=self.yaw, cmd_vel=self.sent, mux_source=self.mux, terrain=dict(self.terrain),
                           obstacles=list(self.obstacles), dodge=dict(self.dodge), alive=dict(self.alive),
                           bearings=kw.pop("bearings", []), keyword=kw.pop("keyword", ""),
                           commands=kw.pop("commands", []), crossing=kw.pop("crossing", []),
                           dodge_events=kw.pop("dodge_events", []), estop=kw.pop("estop", False),
                           imu_ok=kw.pop("imu_ok", True), tilt_deg=kw.pop("tilt_deg", 0.0))
            self.out = self.m.tick(inp)
            self.events += [name for name, _ in self.out.events]
            # the legs do what navigation asks unless something above it has the mux
            self.sent = self.out.cmd if self.mux in ("none", "/nav_cmd_vel") else self.sent
            if self.out.cmd is not None and self.mux == "none":
                self.yaw = wrap(self.yaw + self.out.cmd[2] * 0.05)
        return self.out

    def cry(self, bearing_deg, n=3):
        for _ in range(n):
            self.tick(bearings=[(D(bearing_deg), 1.0)])
            self.tick(0.5)

    def to_seek(self):
        self.tick(commands=["start"])
        assert self.m.state == M.PREFLIGHT
        self.tick(8.0)
        assert self.m.state == M.SEEK
        self.tick(1.0)

    def to_zone1(self, bearing=0.0):
        self.to_seek()
        self.cry(bearing)
        for _ in range(200):                        # turn, then the line "I hear you" finishes
            self.tick()
            if self.m.state == M.CONFIRM and self.m.status()["listening"]:
                break
        assert self.m.state == M.CONFIRM, self.m.state
        self.cry(0.0, n=1)
        assert self.m.state == M.ZONE1, self.m.state

    def to_zone2(self):
        self.tick(commands=["goto:ZONE2"])
        assert self.m.state == M.ZONE2
        self.tick(0.2)


def test_start_is_refused_until_everything_is_alive_and_can_be_forced():
    r = Rig(alive=dict(ALIVE, keyword=False))
    r.tick(commands=["start"])
    assert r.m.state == M.IDLE and r.events == ["preflight_failed"] and r.m.missing == ["keyword"]
    r.tick(commands=["start!"])
    assert r.m.state == M.PREFLIGHT and "start" in r.events
    no_imu = Rig(alive=dict(ALIVE, imu=False))
    no_imu.tick(commands=["start!"])
    assert no_imu.m.state == M.IDLE


def test_listening_states_publish_nothing_at_all():
    r = Rig()
    r.to_seek()
    assert r.out.cmd is None and r.out.terrain_mode == "off" and r.out.arm_dodge is False


def test_bearings_do_not_count_while_pupper_is_talking_or_moving():
    r = Rig()
    r.tick(commands=["start"])
    for _ in range(4):                              # the opening line is still being said
        r.tick(bearings=[(D(30), 1.0)])
    r.tick(8.0)
    assert r.m.state == M.SEEK and r.m.collector.count(r.t) == 0
    r.sent = (0.3, 0.0, 0.0); r.mux = "/teleop_cmd_vel"     # being driven: not still
    r.tick(0.2)
    assert r.m.state == M.PAUSED


def test_seek_orient_confirm_then_walk():
    r = Rig()
    r.to_seek()
    r.cry(120.0)
    assert r.m.state == M.ORIENT and "heard" in r.events and "fix" in r.events
    r.tick(0.5)
    assert r.out.cmd[2] > 0                         # the voice is to the left: turn left
    r.tick(9.0)
    assert r.m.state == M.CONFIRM and abs(diff(r.yaw, D(120))) < D(6)
    assert r.out.cmd is None
    second = r.yaw + D(4.0)
    r.cry(4.0, n=1)                                 # nose-on it reads 4 degrees left of where Pupper stopped
    assert r.m.state == M.ZONE1 and "line_set" in r.events
    # two looks at the same voice, blended half and half
    assert abs(diff(r.m.reck.line.heading, (D(120) + second) / 2)) < D(0.2)
    assert r.out.terrain_mode == "full" and r.out.arm_dodge is False


def test_orient_takes_the_short_way_round():
    r = Rig()
    r.yaw = D(170)
    r.to_seek()
    r.cry(30.0)                                     # the voice is at 200 = -160 degrees
    r.tick(0.3)
    assert r.out.cmd[2] > 0


def test_confirm_goes_with_the_first_fix_if_nothing_more_is_heard():
    r = Rig()
    r.to_seek()
    r.cry(0.0)
    r.tick(20.0)
    assert r.m.state == M.ZONE1


def test_confirm_sends_it_back_to_seek_if_the_first_fix_was_wrong():
    r = Rig()
    r.to_seek()
    r.cry(0.0)
    r.tick(6.0)
    r.cry(70.0, n=1)
    assert r.m.state == M.SEEK and "fix_rejected" in r.events


def test_dry_run_reports_but_never_moves_or_switches_a_reflex_on():
    r = Rig(motion=False)
    r.to_seek()
    r.cry(90.0)
    r.tick(2.0)
    assert r.m.state == M.ORIENT and r.out.cmd is None and r.m.status()["would_cmd"][2] > 0
    assert r.out.arm_dodge is False and r.out.terrain_mode == "report"


def test_zone1_keeps_asking_while_terrain_has_the_legs_and_is_silent_during_a_crossing():
    r = Rig()
    r.to_zone1()
    r.tick(0.5)
    assert r.out.cmd[0] > 0.2
    r.terrain.update(state="APPROACH"); r.mux = "/terrain_cmd_vel"; r.sent = (0.22, 0.0, 0.0)
    r.tick(0.5)
    assert r.out.cmd == (r.m.cfg.walk.cruise, 0.0, 0.0)
    r.terrain.update(state="PLAN", box={"near_x": 0.35, "height": 0.019, "depth": 0.089}); r.sent = (0.0, 0.0, 0.0)
    r.tick(0.1, crossing=[{"state": "planning"}])
    first, second = r.m.cfg.course.obstacles
    stop = first["near"] - 0.35                     # the obstacle's place on the course, less the measured gap
    assert "box_landmark" in r.events and abs(r.m.reck.s - stop) < 0.02
    r.terrain.update(state="STEP_OVER"); r.mux = "none"; r.sent = None
    r.tick(0.1, crossing=[{"state": "started", "advance": 0.58}])
    assert "stepping_over" in r.events
    r.tick(1.0, crossing=[{"state": "progress", "progress": 0.5}])
    assert r.out.cmd is None and abs(r.m.reck.s - (stop + 0.29)) < 0.02      # counted as it happens
    r.terrain.update(state="CRUISE")
    r.tick(0.1, crossing=[{"state": "done", "advance": 0.58}])
    assert abs(r.m.reck.s - (stop + 0.58)) < 0.02
    assert r.out.terrain_mode == "full" and r.m.passed == 1     # the slab is still ahead: it may cross again
    r.tick(2.0)
    assert r.out.cmd is None                        # the walking controller is still coming back
    r.tick(3.0)
    assert r.out.cmd is not None and r.out.cmd[0] > 0
    # the second obstacle: its own landmark, announced again, and only then is everything else walked around
    r.terrain.update(state="PLAN", box={"near_x": 0.37, "height": 0.015, "depth": 0.38}); r.mux = "/terrain_cmd_vel"
    r.tick(0.1, crossing=[{"state": "planning"}])
    assert abs(r.m.reck.s - (second["near"] - 0.37)) < 0.02 and r.events.count("box_landmark") == 2
    r.terrain.update(state="SCALE_ACROSS"); r.mux = "none"; r.sent = None      # a state name the mission was never told about
    r.tick(0.1, crossing=[{"state": "started", "advance": 0.87}])
    r.terrain.update(state="CRUISE")
    r.tick(0.1, crossing=[{"state": "done", "advance": 0.87}])
    assert r.events.count("stepping_over") == 2 and r.m.passed == 2 and r.out.terrain_mode == "avoid"
    assert abs(r.m.reck.s - (second["near"] - 0.37 + 0.87)) < 0.02


def test_a_landmark_far_from_where_the_next_obstacle_should_be_is_not_believed():
    r = Rig()
    r.to_zone1()
    r.m.reck.set_s(3.0)                             # nowhere near the first obstacle
    r.tick(0.1, crossing=[{"state": "planning"}])
    assert abs(r.m.reck.s - 3.0) < 0.05


def test_a_failed_crossing_holds_for_the_operator():
    r = Rig()
    r.to_zone1()
    r.tick(0.1, crossing=[{"state": "started"}])
    r.tick(0.1, crossing=[{"state": "failed", "why": "tilt"}])
    assert r.m.state == M.HOLD and "hold" in r.events and r.out.cmd in (None, M.ZERO)
    r.tick(1.0, commands=["resume"])
    assert r.m.state == M.ZONE1


def test_second_listen_waits_for_the_walk_around_then_redraws_the_line():
    r = Rig()
    r.to_zone1()
    r.m.reck.set_s(2.1)
    r.tick(1.0)
    assert r.m.state == M.ZONE1                     # far enough, but the course's obstacles are not behind it yet
    for _ in r.m.cfg.course.obstacles:              # it walks around each of them (terrain agility's choice)
        r.terrain.update(state="AVOID"); r.tick(0.2)
        assert "going_around" in r.events
        r.terrain.update(state="CRUISE"); r.tick(1.0)
    assert r.m.state == M.REFIX and "refix" in r.events and r.out.terrain_mode == "report"
    r.sent = None
    r.tick(5.0)
    assert r.out.cmd is None
    stood, s_there = r.yaw, r.m.reck.s
    r.cry(-6.0, n=2)
    assert r.m.state == M.ZONE2 and "line_redrawn" in r.events
    line = r.m.reck.line                            # drawn from where it stood, toward the voice, distance kept
    assert abs(diff(line.heading, stood + D(-6))) < D(1) and abs(line.s0 - s_there) < 0.02
    assert r.out.arm_dodge is True


def test_second_listen_rejects_an_echo_and_times_out():
    r = Rig()
    r.tick(commands=["goto:REFIX"]); r.tick(5.0)
    r.cry(60.0, n=1); r.tick(3.0)
    assert r.m.state == M.ZONE2 and "refix_rejected" in r.events
    r2 = Rig()
    r2.tick(commands=["goto:REFIX"]); r2.tick(12.0)
    assert r2.m.state == M.ZONE2 and "refix_timeout" in r2.events


def cover(rng, bearing=0.0, width=1.2):
    half = math.degrees(math.atan2(width / 2, max(rng, 0.2)))
    return {"nearest": rng, "bearing_deg": bearing, "left_deg": bearing + half, "right_deg": bearing - half,
            "width": width, "tall": True, "is_box": False}


def test_the_phrase_is_ignored_early_and_ends_the_search_late():
    r = Rig()
    r.to_zone2()
    r.tick(keyword="YOU_SAVED_ME_PUPPER")           # at 2 m: too early
    assert r.m.state == M.ZONE2 and "keyword_ignored" in r.events
    r.m.reck.set_s(4.5)
    r.tick(keyword="YOU_SAVED_ME_PUPPER")
    assert r.m.state == M.RESCUE and "saved" in r.events
    assert r.out.arm_dodge is False and r.out.terrain_mode == "off"
    r.tick(1.0, commands=["resume"])                # it was not the phrase: carry on
    assert r.m.state == M.ZONE2


def test_it_stops_at_the_cover_and_waits_for_the_phrase():
    r = Rig()
    r.to_zone2()
    r.obstacles = [cover(1.5)]
    r.tick(0.5)
    assert r.m.state == M.ZONE2                     # a tall thing ahead, but the cover is not expected yet
    r.obstacles = []
    r.tick(3.5)                                     # (it stood for the obstacle; clear it and walk on)
    r.m.reck.set_s(5.0)
    r.obstacles = [cover(1.5)]
    r.tick(0.3)
    assert "cover_seen" in r.events and r.m.state == M.ZONE2
    r.obstacles = [cover(0.4)]
    r.tick(0.2)
    assert r.m.state == M.ARRIVE and "arrived" in r.events and r.out.arm_dodge is True
    r.tick(1.0)
    assert r.out.cmd is None                        # braked, then silent
    r.tick(11.0)
    assert r.events.count("arrived") == 2           # says it again while waiting
    r.tick(25.0)
    assert "hold" in r.events and r.m.state == M.ARRIVE     # tells the team, keeps waiting
    r.tick(commands=["phrase"])                     # the operator stands in for the detector
    assert r.m.state == M.RESCUE


def test_hard_stop_and_blind_stop():
    r = Rig()
    r.to_zone2()
    r.m.reck.set_s(7.25); r.tick(0.1)
    assert r.m.state == M.ARRIVE
    r2 = Rig()
    r2.to_zone2()
    r2.terrain["scan_fresh"] = False
    r2.m.reck.set_s(6.05); r2.tick(0.1)
    assert r2.m.state == M.ARRIVE and "camera" in r2.m.arrive_why


def test_dodges_are_counted_but_not_announced_twice():
    r = Rig()
    r.to_zone2()
    r.tick(dodge_events=[{"event": "dodge", "dry_run": False}])
    r.tick(dodge_events=[{"event": "dodge", "dry_run": True}, {"event": "throw_seen"}])
    assert r.m.dodges == 1 and "dodged" in r.events and "dodge" not in r.events


def test_joystick_pauses_and_lets_go_estop_needs_resume():
    r = Rig()
    r.to_zone2()
    r.mux = "/teleop_cmd_vel"; r.sent = (0.0, 0.3, 0.0)
    r.tick(1.0)
    assert r.m.state == M.PAUSED and r.out.arm_dodge is False
    moved = 0.3 * r.m.cfg.reckon.sideways_factor    # a second at 0.3 m/s commanded, at the legs' measured speed
    assert abs(r.m.reck.d - moved) < 0.05           # the nudge is counted
    r.mux = "none"; r.sent = None
    r.tick(1.5)
    assert r.m.state == M.ZONE2
    r.tick(estop=True)
    assert r.m.state == M.PAUSED
    r.tick(3.0)
    assert r.m.state == M.PAUSED
    r.tick(commands=["resume"])
    assert r.m.state == M.ZONE2


def test_terrain_stuck_twice_backs_off_then_holds():
    r = Rig()
    r.to_zone1()
    for _ in range(2):
        r.terrain.update(state="HOLD", reason="blocked on both sides"); r.tick(0.2)
        r.terrain.update(state="CRUISE"); r.tick(0.2)
    assert "backing_off" in r.events
    r.tick(2.5)
    assert r.out.cmd[0] < 0
    r.tick(3.0)
    for _ in range(6):
        r.terrain.update(state="HOLD", reason="blocked on both sides"); r.tick(0.2)
        r.terrain.update(state="CRUISE"); r.tick(4.0)
    assert r.m.state == M.HOLD


def test_end_run_and_starting_from_a_stage():
    r = Rig()
    r.to_zone2()
    r.tick(commands=["end"])
    assert r.m.state == M.DONE and "cancelled" in r.events
    r.tick(3.0)
    assert r.m.state == M.IDLE
    r.tick(commands=["goto:ARRIVE"])
    assert r.m.state == M.ARRIVE and abs(r.m.reck.s - 6.0) < 1e-6 and "arrived" in r.events
    r.tick(commands=["goto:ZONE1"])                 # not while a run is going
    assert r.m.state == M.ARRIVE


def test_status_is_json_and_carries_a_cue():
    import json
    r = Rig()
    r.to_seek()
    st = json.loads(json.dumps(r.m.status()))
    assert st["state"] == "SEEK" and st["cue"]["who"] == "Survivor" and "Help" in st["cue"]["text"]
