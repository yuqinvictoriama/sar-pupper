"""Off-robot tests for reaching the survivor: the state machine alone, and the whole loop
(perception + mission + a robot that moves as commanded) in the stand-in world.

    python3 -m unittest discover -s tests -v
"""
from __future__ import annotations

import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, ".."))

from world import CFG, Sim, at  # noqa: E402

from survivor.mission import Inputs, Mission  # noqa: E402

TOPIC = CFG["mission"]["cmd_topic"]


def status(target=None, **kw) -> dict:
    st = {"t": 0.0, "enabled": True, "phase": "LOCKED" if target else "SCAN", "target": target, "look": None,
          "queued": [], "people": 1, "tracks": [], "pose": {"x": 0.0, "y": 0.0, "yaw_deg": 0.0}}
    st.update(kw)
    return st


def target(bearing=0.0, rng=3.0, visible=True, **kw) -> dict:
    t = {"id": 1, "bearing_deg": bearing, "range_m": rng, "visible": visible, "since_seen_s": 0.0 if visible else 5.0,
         "sigma_m": 0.4, "box_frac": 0.2, "cam": "fisheye", "x": rng, "y": 0.0}
    t.update(kw)
    return t


def signalling(bearing, dist, **kw):
    """A person who stands for a second, then throws both arms up."""
    who = at(bearing, dist, pose="down")
    sim = Sim([who], **kw)
    sim.run(1.0)
    who.pose = "up"
    return who, sim


def stay(sim) -> bool:
    return sim.out.state == "STAY"


class StateMachineTests(unittest.TestCase):
    def drive(self, mission, seconds, t0=0.0, **fields):
        """Tick for a while with fixed inputs; returns (last output, time)."""
        t, out = t0, None
        for _ in range(int(round(seconds * 20))):
            t += 0.05
            out = mission.tick(Inputs(now=t, **fields))
        return out, t

    def test_nothing_pinned_means_no_commands(self):
        m = Mission(CFG, motion=True)
        out, _ = self.drive(m, 1.0, status=status())
        self.assertEqual((out.state, out.cmd, out.actions), ("SCAN", None, []))
        out, _ = self.drive(m, 0.1, status=None)                       # the survivor node is not running
        self.assertEqual((out.state, out.cmd), ("SCAN", None))
        out, _ = self.drive(m, 0.1, status=status(target()), status_age=3.0)   # ... or has gone quiet
        self.assertEqual((out.state, out.cmd), ("SCAN", None))

    def test_face_then_go_and_slow_down_near(self):
        m = Mission(CFG, motion=True)
        out, t = self.drive(m, 1.0, status=status(target(bearing=80.0)), mux_active=TOPIC)
        self.assertEqual(out.state, "APPROACH")
        self.assertEqual(out.cmd[0], 0.0)                              # well off the nose: turn on the spot
        self.assertGreater(out.cmd[2], 0.4)
        out, t = self.drive(m, 0.1, t, status=status(target(bearing=-80.0)), mux_active=TOPIC)
        self.assertLess(out.cmd[2], -0.4)
        out, t = self.drive(m, 0.1, t, status=status(target(bearing=5.0, rng=3.0)), mux_active=TOPIC)
        self.assertEqual(out.cmd[0], CFG["mission"]["speed_far"])
        out, t = self.drive(m, 0.1, t, status=status(target(bearing=5.0, rng=1.5)), mux_active=TOPIC)
        self.assertEqual(out.cmd[0], CFG["mission"]["speed_near"])
        for cmd in (out.cmd,):
            self.assertLessEqual(abs(cmd[0]), 0.75)                    # inside what the walking policy was trained on
            self.assertLessEqual(abs(cmd[2]), 2.0)

    def test_stands_up_first_if_the_legs_are_not_walking(self):
        m = Mission(CFG, motion=True)
        out = m.tick(Inputs(now=0.05, status=status(target()), walking=False))
        self.assertEqual(out.state, "LOCKED")
        self.assertEqual(out.actions, ["bark", "clear_queue", "stand"])
        self.assertIsNone(out.cmd)
        out, t = self.drive(m, 2.0, 0.05, status=status(target()), walking=False)
        self.assertEqual((out.state, out.actions, out.cmd), ("LOCKED", [], None))    # asked once, now waiting
        out, t = self.drive(m, 3.0, t, status=status(target()), walking=True)
        self.assertEqual(out.state, "LOCKED")                          # walking controller is up but not ready yet
        self.assertEqual(out.cmd, (0.0, 0.0, 0.0))                     # holding the mux so nothing else drives meanwhile
        out, t = self.drive(m, 2.0, t, status=status(target()), walking=True)
        self.assertEqual(out.state, "APPROACH")

    def test_fallen_over_waits_for_a_person(self):
        m = Mission(CFG, motion=True)
        out, t = self.drive(m, 1.0, status=status(target()), mux_active=TOPIC)
        self.assertEqual(out.state, "APPROACH")
        out, t = self.drive(m, 5.0, t, status=status(target()), fallen=True, walking=False)
        self.assertEqual(out.state, "PAUSED")
        self.assertIsNone(out.cmd)
        self.assertNotIn("stand", out.actions)                         # it never tries to get up by itself

    def test_joystick_and_terrain_crossing_pause_it(self):
        m = Mission(CFG, motion=True)
        self.drive(m, 1.0, status=status(target()), mux_active=TOPIC)
        out, t = self.drive(m, 0.5, 1.0, status=status(target()), mux_active="/teleop_cmd_vel")
        self.assertEqual((out.state, out.cmd), ("PAUSED", None))
        out, t = self.drive(m, 0.5, t, status=status(target()), mux_active=TOPIC, terrain_state="STEP_OVER")
        self.assertEqual((out.state, out.cmd), ("PAUSED", None))
        out, t = self.drive(m, 0.5, t, status=status(target()), mux_active=TOPIC, terrain_state="IDLE")
        self.assertEqual(out.state, "APPROACH")
        self.assertIsNotNone(out.cmd)

    def test_blocked_by_terrain_or_by_no_progress(self):
        m = Mission(CFG, motion=True)
        self.drive(m, 1.0, status=status(target()), mux_active=TOPIC, terrain_state="IDLE")
        out = m.tick(Inputs(now=1.1, status=status(target()), mux_active=TOPIC, terrain_state="IDLE", terrain_blocked=True))
        self.assertEqual(out.state, "BLOCKED")
        self.assertEqual([e["event"] for e in out.events], ["blocked"])
        self.assertLess(out.cmd[0], 0.0)                               # backs off first

        m = Mission(CFG, motion=True)                                  # terrain agility not running: pushing, no closer
        events, t = [], 0.0
        for _ in range(60):
            t += 0.05
            out = m.tick(Inputs(now=t, status=status(target()), mux_active=TOPIC))
            events += out.events
        self.assertEqual([e["event"] for e in events], ["blocked"])

        m = Mission(CFG, motion=True)                                  # terrain agility running and happy: its call, not ours
        events, t = [], 0.0
        for _ in range(100):
            t += 0.05
            events += m.tick(Inputs(now=t, status=status(target()), mux_active=TOPIC, terrain_state="IDLE")).events
        self.assertEqual(events, [])

        m = Mission(CFG, motion=True)                                  # something above us in the mux has the legs
        events, t = [], 0.0
        for _ in range(100):
            t += 0.05
            events += m.tick(Inputs(now=t, status=status(target()), mux_active="/dodge_cmd_vel")).events
        self.assertEqual(events, [])

    def test_detours_alternate_sides_and_widen(self):
        m = Mission(CFG, motion=True)
        t, sides, strafe = 0.0, [], {}
        for _ in range(int(80 / 0.05)):
            t += 0.05
            out = m.tick(Inputs(now=t, status=status(target(bearing=10.0)), mux_active=TOPIC))
            sides += [e["side"] for e in out.events if e["event"] == "blocked"]
            if out.cmd and out.cmd[1]:
                strafe[len(sides)] = strafe.get(len(sides), 0.0) + abs(out.cmd[1]) * 0.05
        self.assertEqual(sides[:4], ["left", "right", "left", "right"])    # toward the survivor's side first
        self.assertAlmostEqual(strafe[1], 0.5, delta=0.06)
        self.assertAlmostEqual(strafe[3], 1.0, delta=0.06)
        self.assertGreater(len(sides), 5)                              # and it keeps trying

    def test_route_mode_leaves_the_driving_to_route_mapping(self):
        m = Mission(CFG, motion=True, drive="route")
        out, t = self.drive(m, 1.0, status=status(target(rng=3.0)))
        self.assertEqual((out.state, out.cmd), ("APPROACH", None))     # publishes nothing: /nav_cmd_vel gets through
        out, t = self.drive(m, 0.2, t, status=status(target(rng=3.0, visible=False)))
        self.assertEqual((out.state, out.cmd), ("REACQUIRE", None))
        out, t = self.drive(m, 0.2, t, status=status(target(rng=1.4)))
        self.assertEqual(out.state, "APPROACH")
        self.assertGreater(out.cmd[0], 0.0)                            # the last metre and a half is ours

    def test_dry_run_asks_for_nothing_that_moves(self):
        m = Mission(CFG)                                               # motion defaults to off
        self.assertFalse(m.motion)
        actions, t = [], 0.0
        for rng in (3.0, 3.0, 0.6, 0.6, 0.6, 0.6):
            for _ in range(20):
                t += 0.05
                actions += m.tick(Inputs(now=t, status=status(target(rng=rng)), walking=False)).actions
        self.assertEqual(actions, ["bark", "bark"])                    # on lock and on arrival; no stand-up, trick or queue clearing
        self.assertEqual(m.state, "STAY")

    def test_bounce_waits_for_quiet_air_then_goes_without(self):
        m = Mission(CFG, motion=True)
        self.drive(m, 1.0, status=status(target()), mux_active=TOPIC)
        actions, notes, t = [], [], 1.0
        for _ in range(100):
            t += 0.05
            out = m.tick(Inputs(now=t, status=status(target(rng=0.65)), mux_active=TOPIC, dodge_quiet_s=0.5))
            actions += out.actions
            notes.append(out.note)
        self.assertEqual(actions, ["bark"])                            # the arrival bark, and no trick
        self.assertIn("waiting for the air to clear before the bounce", notes)
        self.assertIn("no bounce: balls still flying", notes)
        self.assertEqual(m.state, "STAY")

    def test_no_bounce_when_tilted_or_too_close(self):
        for extra, tgt in (({"level": False}, target(rng=0.65)), ({}, target(rng=0.4)), ({"server": False}, target(rng=0.65))):
            m = Mission(CFG, motion=True)
            self.drive(m, 1.0, status=status(target()), mux_active=TOPIC)
            actions, t = [], 1.0
            for _ in range(40):
                t += 0.05
                actions += m.tick(Inputs(now=t, status=status(tgt), mux_active=TOPIC, **extra)).actions
            self.assertNotIn("celebrate", actions, extra)
            self.assertEqual(m.state, "STAY")

    def test_cancel_lets_go_at_once(self):
        m = Mission(CFG, motion=True)
        self.drive(m, 1.0, status=status(target()), mux_active=TOPIC)
        out = m.tick(Inputs(now=1.1, status=status()))
        self.assertEqual((out.state, out.cmd), ("SCAN", None))


class WholeLoopTests(unittest.TestCase):
    def test_walks_up_stops_barks_and_bounces_once(self):
        who, sim = signalling(20, 3.5)
        self.assertTrue(sim.run(40.0, until=stay))
        self.assertEqual(sim.states, ["SCAN", "CANDIDATE", "LOCKED", "APPROACH", "ARRIVED", "CELEBRATE", "STAY"])
        self.assertEqual(sim.names(), ["candidate", "locked", "arrived"])
        self.assertEqual([a for _, a in sim.actions], ["bark", "clear_queue", "bark", "celebrate", "walk"])
        self.assertAlmostEqual(sim.range_to(who), CFG["mission"]["stop_m"], delta=0.12)
        locked = next(e for e in sim.events if e["event"] == "locked")
        self.assertLess(locked["t"] - 1.0, 2.0)                        # success criterion: locked within 2 s
        who.pose = "down"
        sim.run(5.0)
        self.assertEqual(sim.out.state, "STAY")
        self.assertEqual(sim.names().count("arrived"), 1)
        self.assertLess(max(abs(c[0]) for c in sim.cmds if c), 0.75)

    def test_from_every_side(self):
        for bearing, dist in ((90, 3.0), (-100, 3.8), (140, 3.2), (-150, 4.0)):
            who, sim = signalling(bearing, dist)
            self.assertTrue(sim.run(40.0, until=stay), (bearing, dist, sim.states))
            self.assertLess(abs(sim.range_to(who) - CFG["mission"]["stop_m"]), 0.15, (bearing, dist))

    def test_close_behind_or_straight_behind_it_turns_to_look_first(self):
        for bearing, dist in ((150, 2.2), (-150, 2.2), (180, 2.6), (180, 3.8)):
            who, sim = signalling(bearing, dist)
            self.assertTrue(sim.run(40.0, until=stay), (bearing, dist, sim.states))
            self.assertIn("LOOK", sim.states, (bearing, dist))
            self.assertLess(sim.states.index("LOOK"), sim.states.index("LOCKED"))
            self.assertLess(abs(sim.range_to(who) - CFG["mission"]["stop_m"]), 0.15)

    def test_arms_down_behind_is_not_worth_a_turn(self):
        sim = Sim([at(150, 2.2, pose="down"), at(180, 3.0, pose="t")])
        sim.run(6.0)
        self.assertEqual(sim.states, ["SCAN"])
        self.assertTrue(all(c is None for c in sim.cmds))

    def test_something_in_the_way_blocked_then_round_it(self):
        who, sim = signalling(0, 4.0, obstacles=[(2.0, 0.0, 0.45)])
        self.assertTrue(sim.run(90.0, until=stay), sim.states)
        self.assertIn("BLOCKED", sim.states)
        self.assertIn("blocked", sim.names())
        self.assertLess(sim.names().index("blocked"), sim.names().index("arrived"))
        self.assertAlmostEqual(sim.range_to(who), CFG["mission"]["stop_m"], delta=0.15)

    def test_walled_in_it_keeps_trying_and_never_gives_up(self):
        ring = [(2.0 * math.cos(a), 2.0 * math.sin(a), 0.75) for a in [i * math.pi / 8 for i in range(16)]]
        who, sim = signalling(0, 4.0, obstacles=ring)
        sim.run(120.0)
        self.assertGreater(sim.names().count("blocked"), 5)
        self.assertIn(sim.out.state, ("APPROACH", "BLOCKED"))
        self.assertNotIn("arrived", sim.names())

    def test_dry_run_watches_and_locks_but_does_not_move(self):
        who, sim = signalling(30, 3.0, motion=False)
        sim.run(8.0)
        self.assertEqual(sim.states, ["SCAN", "CANDIDATE", "LOCKED", "APPROACH"])
        self.assertEqual(sim.names(), ["candidate", "locked"])
        self.assertEqual([a for _, a in sim.actions], ["bark"])
        self.assertEqual((sim.scene.x, sim.scene.y, sim.scene.yaw), (0.0, 0.0, 0.0))

    def test_survivor_drops_out_of_sight_then_comes_back(self):
        who, sim = signalling(0, 4.5)
        self.assertTrue(sim.run(10.0, until=lambda s: s.out.state == "APPROACH" and s.range_to(who) < 3.2))
        sim.scene.people = []                                          # hidden (behind a wall, say)
        sim.run(3.0)
        self.assertEqual(sim.out.state, "REACQUIRE")
        self.assertLess(sim.range_to(who), 2.5)                        # it kept walking to where they were
        sim.scene.people = [who]
        who.pose = "down"                                              # arms down now: still them
        self.assertTrue(sim.run(40.0, until=stay), sim.states)
        self.assertEqual(sim.names(), ["candidate", "locked", "lost", "reacquired", "arrived"])

    def test_survivor_gone_for_good_it_searches_round_the_spot_for_ever(self):
        who, sim = signalling(0, 3.5)
        self.assertTrue(sim.run(10.0, until=lambda s: s.out.state == "APPROACH" and s.range_to(who) < 3.0))
        sim.scene.people = []
        sim.run(90.0)
        self.assertEqual(sim.out.state, "REACQUIRE")
        self.assertLess(sim.range_to(who), 2.0)                        # still hunting near the last place it saw them
        self.assertNotIn("arrived", sim.names())
        self.assertGreater(abs(sim.scene.yaw), 2 * math.pi)            # it looked all the way round

    def test_survivor_walks_off_it_follows_but_bounces_only_once(self):
        who, sim = signalling(0, 3.0)
        self.assertTrue(sim.run(40.0, until=stay))
        who.pose = "down"
        sim.walk(who, 2.5, 0.5)
        self.assertEqual(sim.out.state, "APPROACH")
        self.assertTrue(sim.run(40.0, until=stay))
        self.assertEqual(sim.names().count("arrived"), 2)
        self.assertEqual([a for _, a in sim.actions].count("celebrate"), 1)

    def test_survivor_steps_right_up_it_backs_off(self):
        who, sim = signalling(0, 3.0)
        self.assertTrue(sim.run(40.0, until=stay))
        who.pose = "down"
        sim.walk(who, -0.4, 0.0, speed=0.5)
        self.assertLess(sim.range_to(who), 0.45)
        sim.run(4.0)
        self.assertGreaterEqual(sim.range_to(who), 0.45)
        self.assertEqual(sim.out.state, "STAY")

    def test_second_signaller_is_ignored_while_the_first_is_pinned(self):
        first, second = at(0, 3.5, pose="down"), at(100, 3.0, pose="down")
        sim = Sim([first, second])
        sim.run(1.0)
        first.pose = "up"
        sim.run(2.5)
        second.pose = "up"
        self.assertTrue(sim.run(40.0, until=stay))
        self.assertLess(sim.range_to(first), 0.9)
        self.assertGreater(sim.range_to(second), 2.0)

    def test_joystick_takes_over_then_hands_back(self):
        who, sim = signalling(0, 4.0)
        self.assertTrue(sim.run(10.0, until=lambda s: s.out.state == "APPROACH"))
        sim.mux = "/teleop_cmd_vel"
        x = sim.scene.x
        sim.run(2.0)
        self.assertEqual(sim.out.state, "PAUSED")
        self.assertAlmostEqual(sim.scene.x, x, delta=0.05)
        sim.mux = "none"
        self.assertTrue(sim.run(40.0, until=stay))

    def test_robot_that_walks_slower_than_told_still_gets_there(self):
        """Dead reckoning is wrong by 30 %: bearing and range come from the cameras, so it does not matter."""
        who, sim = signalling(-60, 4.0, slip=0.7)
        self.assertTrue(sim.run(60.0, until=stay), sim.states)
        self.assertAlmostEqual(sim.range_to(who), CFG["mission"]["stop_m"], delta=0.15)

    def test_no_trick_installed_still_arrives_and_stays(self):
        who, sim = signalling(0, 3.0, tricks=())
        self.assertTrue(sim.run(40.0, until=stay), sim.states)
        self.assertEqual([a for _, a in sim.actions], ["bark", "clear_queue", "bark", "celebrate"])

    def test_server_hands_the_legs_back_after_the_trick_so_nothing_more_is_asked(self):
        who, sim = signalling(0, 3.0, hands_back=True)
        self.assertTrue(sim.run(40.0, until=stay), sim.states)
        self.assertEqual([a for _, a in sim.actions], ["bark", "clear_queue", "bark", "celebrate"])


if __name__ == "__main__":
    unittest.main()
