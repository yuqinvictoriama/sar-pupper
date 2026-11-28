"""What the mission reports, the line Pupper says for each, and the team's cue for each stage.

The words themselves belong to personality/callouts.py: that list is what Gemini is given
and what the robot's own voice says. This file reads it, so a line is reworded in one place.
If that file cannot be read (path-navigation copied somewhere on its own), the copy below is
used; tests/test_mission.py fails if the two ever differ.

The list is used three ways here:
  - robot/nav_node.py publishes every event on /nav/event; personality/event_bridge.py passes
    the spoken ones to Pupper's voice and drops the rest;
  - the mission waits for a line to finish before it listens (Pupper's voice reaches its own mics);
  - Mission Control shows the line and the cue, so the team can follow a run without the script.
"""
from __future__ import annotations

import importlib.util
from pathlib import Path

# The mission's spoken events and their lines (a copy of personality/callouts.py, see above).
FALLBACK_LINES = {
    "start": "Pupper, on duty. If you need help, call out. I will come to your voice.",
    "heard": "I hear you. Keep calling. I'm coming to you.",
    "no_sound": "I can't hear anyone yet. Call out if you need help. Still searching.",
    "stepping_over": "Low obstacle. Stepping over. Still coming.",
    "going_around": "Something is in my way. Going around. Still coming.",
    "refix": "Call out again, so I can find you.",
    "arrived": "I'm here. I can't see you. Can you hear me?",
    "saved": "I've got you. You're not alone now. I'm staying right here. "
             "You did the hard part. You kept calling, and that's how I found you. "
             "Breathe with me. In. And out. Good.",
    "hold": "I've stopped. I need my team. Still here.",
    "cancelled": "Team: rescue cancelled. Standing by.",
}
# Said by the dodge node's own event (the bridge listens to /dodge/event), shown in Mission Control.
DODGE_LINE = "Something flew past me. I'm okay. Still coming."
CALLOUTS_FILE = Path(__file__).resolve().parent.parent.parent / "personality" / "callouts.py"


def load_lines(path=CALLOUTS_FILE) -> tuple[dict, str]:
    """(event -> words, where they came from)."""
    try:
        spec = importlib.util.spec_from_file_location("pupper_callouts", str(path))
        module = importlib.util.module_from_spec(spec)
        import sys
        sys.modules[spec.name] = module             # dataclasses look their module up by name
        spec.loader.exec_module(module)
        lines = {name: c.words for name, c in module.CALLOUTS.items() if name in FALLBACK_LINES and c.words}
        if set(lines) == set(FALLBACK_LINES):
            return lines, str(path)
    except Exception:
        pass
    return dict(FALLBACK_LINES), "path-navigation/nav/events.py (personality/callouts.py not readable)"


LINES, LINES_SOURCE = load_lines()

# Reported and logged, never spoken.
SILENT = {
    "preflight_failed", "fix", "fix_rejected", "line_set", "line_redrawn", "refix_rejected", "refix_timeout",
    "box_landmark", "crossed", "walked_around", "cover_seen", "keyword_ignored", "paused", "resumed",
    "blocked", "jumped", "motion", "stand_down", "turning_to_listen", "dodged", "no_answer", "backing_off",
}

# The phrase the survivor says. Pupper must never say it: no line above may contain these words.
PHRASE = "You saved me, Pupper!"
FORBIDDEN_IN_LINES = ("saved me",)

MIN_SPEAK_S = 3.0
WORDS_PER_S = 3.0
SETTLE_AFTER_SPEECH_S = 1.0


def speak_time(event: str) -> float:
    """Seconds to wait after sending a spoken event before trusting the microphones."""
    line = LINES.get(event)
    if not line:
        return 0.0
    return max(MIN_SPEAK_S, len(line.split()) / WORDS_PER_S) + SETTLE_AFTER_SPEECH_S


def is_spoken(event: str) -> bool:
    return event in LINES


def cue(state: str, *, quiet: bool = False, s: float = 0.0, course=None, crossing: bool = False,
        reason: str = "", motion: bool = True) -> dict:
    """The one thing the people in the room should be doing now: {"who", "text", "tone"}.

    tone: go (do this now), wait (be silent, keep clear), stop, done.
    """
    def c(who, text, tone="wait"):
        return {"who": who, "text": text, "tone": tone}

    if state == "IDLE":
        return c("Everyone", "Places. Operator holds the gamepad. Press Start when the room is quiet.")
    if state == "PREFLIGHT":
        return c("Everyone", "Silence. Survivor: wait for Pupper to finish its opening line.")
    if state == "SEEK":
        if quiet:
            return c("Survivor", "Wait for Pupper to stop talking.")
        return c("Survivor", "FIRST CALL: \"Help! Help me!\" loudly toward Pupper, once every 4 seconds.", "go")
    if state in ("ORIENT", "CONFIRM"):
        return c("Survivor", "Keep calling \"Help! Help me!\" every 4 seconds until Pupper starts walking.", "go")
    if state == "ZONE1":
        if crossing:
            return c("Throwers", "NO THROWS. Pupper is crossing the low obstacle (a minute or more). Keep clear of the line.", "stop")
        return c("Everyone", "Silence. Pupper is walking to the voice. No throws until it passes the 3 m markers.", "stop")
    if state == "REFIX":
        if quiet:
            return c("Survivor", "Get ready: wait for \"Call out again, so I can find you.\" to finish.")
        return c("Survivor", "SECOND CALL: \"Help! Help me!\" twice, 2 seconds apart. Then stay quiet.", "go")
    if state == "ZONE2":
        lo, hi = (course.throw_from, course.throw_to) if course is not None else (3.0, 5.2)
        if s < lo:
            return c("Throwers", "Get ready. Throw only after Pupper passes the first pair of markers.")
        if s <= hi:
            return c("Throwers", "THROW WINDOW OPEN. Underhand lobs from your marks, one at a time, 3 seconds apart.", "go")
        return c("Throwers", "STOP THROWING. Pupper is past the last markers.", "stop")
    if state == "ARRIVE":
        if quiet:
            return c("Survivor", "Wait for \"Can you hear me?\" to finish.")
        return c("Survivor", "THE PHRASE: \"You saved me, Pupper!\" clearly, toward Pupper. Repeat every 3 seconds.", "go")
    if state == "RESCUE":
        return c("Survivor", "Answer Pupper: your name, are you hurt, can you walk, is anyone else here. "
                             "Operator: press End run when it has reported.", "done")
    if state == "PAUSED":
        return c("Operator", f"Paused: {reason or 'operator'}. Press Resume when Pupper is standing and clear.", "stop")
    if state == "HOLD":
        return c("Operator", f"Pupper has stopped: {reason or 'needs its team'}. Sort it out, then Resume or End run.", "stop")
    if state == "DONE":
        return c("Everyone", "Run over. Collect the balls. Reset the course.", "done")
    return c("Everyone", "")
