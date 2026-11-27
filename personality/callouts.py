"""Pupper's call-outs: the one list of mission events and the words for each.

Everything else reads this list, so an event is added or reworded in one place:

    prompt.md         its "# Call-outs" section is filled in from here (run.mission_prompt)
    mission_feed.py   refuses any event that is not here; knows how long a line takes
    voice.py          says the same words itself when no Gemini is connected
    event_bridge.py   passes on only the ROS events that are here
    test_personality.py, README.md

The events are the ones in path-navigation/plan.md ("What Pupper says"), plus the dodge
node's own `dodge`, and two the feed needs: `reconnected` and `test`.

No imports from the rest of the project: this file is read on the robot and on a desk.
"""
from __future__ import annotations

from dataclasses import dataclass

# Where things stand, and the refrain that says so. The feed keeps the status; Gemini is told
# it in the events that need it, so the refrain does not depend on what Gemini remembers.
REFRAINS = {"searching": "Still searching.", "coming": "Still coming.", "here": "Still here."}
OFF = "off"  # no mission running


@dataclass(frozen=True)
class Callout:
    name: str
    words: str                  # the exact words. The robot's own voice says these; "" = nothing
    prompt: str = ""            # what the prompt tells Gemini; empty = the words, in quotes
    status: str | None = None   # where things stand after this event; None = no change
    refrain: bool = False       # the words end with the refrain for the current status
    interrupts: bool = True     # False: dropped if Pupper is in the middle of another line
    min_gap_s: float = 3.0      # the same event again sooner than this is not said twice
    internal: bool = False      # sent by the feed itself; cannot be posted

    def prompt_line(self) -> str:
        return "- %s: %s" % (self.name, self.prompt or '"%s"' % self.words)

    def aloud(self, status: str | None = None) -> str:
        """The words the robot's own voice says, given where things stand."""
        if self.refrain and status in REFRAINS:
            return ("%s %s" % (self.words, REFRAINS[status])).strip()
        return self.words

    def clip(self, status: str | None = None) -> str:
        """The recording's file name without .wav: 'heard', or 'dodge_coming' for a refrain."""
        return "%s_%s" % (self.name, status) if self.refrain and status in REFRAINS else self.name


_LIST = [
    Callout("start", "Pupper, on duty. If you need help, call out. I will come to your voice.",
            status="searching"),
    Callout("heard", "I hear you. Keep calling. I'm coming to you.", status="coming"),
    Callout("no_sound", "I can't hear anyone yet. Call out if you need help. Still searching.",
            status="searching", interrupts=False, min_gap_s=15.0),
    Callout("stepping_over", "Low obstacle. Stepping over. Still coming.", min_gap_s=20.0),
    Callout("going_around", "Something is in my way. Going around. Still coming.", min_gap_s=20.0),
    Callout("refix", "Call out again, so I can find you.", min_gap_s=5.0),
    # A dodge is the one event that can come many times and at any moment. It never cuts
    # across another line, and its refrain follows the mission, not Gemini's memory.
    Callout("dodge", "Something flew past me. I'm okay.",
            prompt='"Something flew past me. I\'m okay." Then where things stand.',
            refrain=True, interrupts=False, min_gap_s=10.0),
    # Navigation repeats this every 8 s until the survivor answers, so the gap is shorter.
    Callout("arrived", "I'm here. I can't see you. Can you hear me?", status="here", min_gap_s=6.0),
    # The comfort steps run once. The robot's own voice says steps 1 to 3 (plan: "a recorded
    # clip of lines 1-3"); the questions need Gemini.
    Callout("saved", "I've got you. You're not alone now. I'm staying right here. "
                     "You did the hard part. You kept calling, and that's how I found you. "
                     "Breathe with me. In. And out. Good.",
            prompt='do the comfort steps under "When you reach someone".',
            status="here", min_gap_s=120.0),
    Callout("hold", "I've stopped. I need my team. Still here.", min_gap_s=10.0),
    Callout("cancelled", "Team: rescue cancelled. Standing by.", status=OFF),
    Callout("reconnected", "",
            prompt="your connection dropped and came back. Say only where things stand. "
                   "With saved=yes, do not start the comfort steps again.",
            refrain=True, internal=True, min_gap_s=0.0),
    Callout("test", "Voice check. Pupper here.", min_gap_s=0.0),
]

CALLOUTS: dict[str, Callout] = {c.name: c for c in _LIST}

# Retired with the arms-up mission (2026-11-26). Named so a sender still using one is told
# what replaced it, not just "unknown".
RETIRED = {"candidate": "", "locked": "heard", "blocked": "going_around", "lost": "no_sound",
           "reacquired": "heard"}


def postable() -> list[str]:
    """The names a program may post, in mission order."""
    return [c.name for c in _LIST if not c.internal]


def prompt_lines() -> str:
    """The call-outs as the prompt lists them."""
    return "\n".join(c.prompt_line() for c in _LIST)


def unknown(name: str) -> str:
    """Why a name is refused, with the real choices."""
    if RETIRED.get(name):
        return "event '%s' was retired with the arms-up mission; send '%s'" % (name, RETIRED[name])
    return "unknown event '%s'; known: %s" % (name, ", ".join(postable()))


def speak_seconds(words: str) -> float:
    """How long a line takes to say: about three words a second, at least 3 s (navigation plan)."""
    n = len(words.split())
    return round(max(3.0, n / 3.0), 1) if n else 0.0


def clips() -> dict[str, str]:
    """Every recording the robot's own voice can use: file name (no .wav) -> words."""
    out = {}
    for c in _LIST:
        if not c.words:
            continue
        out[c.name] = c.words
        if c.refrain:
            for status in REFRAINS:
                out[c.clip(status)] = c.aloud(status)
    return out
