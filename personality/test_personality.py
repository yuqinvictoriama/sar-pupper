#!/usr/bin/env python3
"""Desk checks for the mission personality: no robot, no ROS, no Gemini.

    python3 personality/test_personality.py

They check what other code and the mission rely on: the prompt's headings, a
call-out for every mission event, the tools the prompt names, and that run.py
really hands the mission prompt and voice to the lab's server, kiosk and chat.
They cannot check how Pupper sounds. For that: python3 personality/run.py chat.
"""

from __future__ import annotations

import asyncio
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import run  # noqa: E402

FAILED = []


def check(name: str, ok: bool, detail: str = "") -> None:
    print(("  ok    " if ok else "  FAIL  ") + name + (f"  ({detail})" if detail and not ok else ""))
    if not ok:
        FAILED.append(name)


def skip(name: str, why: str) -> None:
    print(f"  skip  {name}  ({why})")


def section(prompt: str, heading: str) -> str:
    """The text under one '# ' heading."""
    lines, keep = [], False
    for line in prompt.splitlines():
        if line.startswith("# "):
            keep = line[2:].strip().lower() == heading.lower()
        elif keep:
            lines.append(line)
    return "\n".join(lines).strip()


# The event names the mission nodes publish: /survivor/event in survivor-id/plan.md,
# /dodge/event in projectile-dodging/plan.md, and "start" for the start of a search.
EVENTS = ["start", "candidate", "locked", "blocked", "dodge", "lost", "reacquired", "arrived", "cancelled"]
HEADINGS = ["Who you are", "How you talk", "On duty and off duty", "Your reflexes", "Call-outs",
            "When you reach someone", "What you never do", "How you move", "When a tool says no",
            "Follow mode", "Tricks", "Safety"]
MAX_WORDS = 1500  # Google's Live API guidance warns against long, multi-page prompts


def test_prompt() -> None:
    print("The prompt (personality/prompt.md)")
    p = run.mission_prompt()
    check("comments are stripped before Gemini sees it", "<!--" not in p and "-->" not in p)
    missing = [h for h in HEADINGS if not section(p, h)]
    check("every section is there and has text", not missing, f"missing: {missing}")
    words = len(p.split())
    check(f"it is short enough to follow ({words} words, limit {MAX_WORDS})", words <= MAX_WORDS)
    callouts = section(p, "Call-outs")
    missing = [e for e in EVENTS if not re.search(rf"^- {e}:", callouts, flags=re.M)]
    check("every mission event has a call-out", not missing, f"missing: {missing}")
    check("it says how events arrive (MISSION EVENT:)", "MISSION EVENT:" in section(p, "Your reflexes"))
    for refrain in ("Still searching", "Still coming", "Still here"):
        check(f'the refrain "{refrain}" is in the character and the call-outs',
              refrain in section(p, "Who you are") and refrain in callouts)
    # The same word checks the lab's tests run on lab/prompt.md (TODO 1, 6 and 11).
    first = " ".join(section(p, h) for h in ("Who you are", "How you move", "When a tool says no"))
    check('lab check, TODO 1: 60+ words and says "tool"', len(first.split()) >= 60 and "tool" in first.lower())
    follow = section(p, "Follow mode").lower()
    check('lab check, TODO 6: Follow mode says "end" or "stop"', "end" in follow or "stop" in follow)
    last = (section(p, "Tricks") + section(p, "Safety")).lower()
    check('lab check, TODO 11: says "stop", "fall" and "order"', all(w in last for w in ("stop", "fall", "order")))


def test_lab_tools() -> None:
    print("The tools the prompt names exist in the lab")
    run.use_lab_code()
    try:
        import test_tools  # noqa: F401  the lab's fake ROS; must come before robot_api
        import tool_registry
        tool_registry.load_lab()
    except Exception as e:
        skip("tools exist", f"could not load the lab: {type(e).__name__}: {e}")
        return
    p = run.mission_prompt()
    for name in ("stop", "stand_up", "robot_status"):
        check(f"{name}: in the prompt and a real tool", name in p and name in tool_registry.TOOLS)
    for name in ("lie_down", "follow", "play_trick"):
        check(f"{name}: a real tool (the prompt talks about it in words)", name in tool_registry.TOOLS)


def test_run() -> None:
    print("run.py hands the personality to the lab's code")
    run.use_lab_code()
    mission = run.mission_prompt()

    try:
        import test_tools  # noqa: F401
        import server as lab_server
    except Exception as e:
        skip("server serves the mission prompt", f"{type(e).__name__}: {e}")
    else:
        lab_prompt = lab_server.system_prompt()
        lab_server.PROMPT_FILE = run.PROMPT_FILE  # what run.server() does before uvicorn starts
        check("server: GET /tools would send the mission prompt", lab_server.system_prompt() == mission)
        check("server: and that is not the lab's prompt", lab_prompt != mission)

    try:
        import kiosk as lab_kiosk
    except Exception as e:
        skip("kiosk sets the mission voice", f"{type(e).__name__}: {e}")
    else:
        before, started = dict(lab_kiosk.SETTINGS), []

        async def fake_main(key_file):
            started.append(key_file)

        lab_kiosk.main = fake_main
        run.kiosk("key-file", run.VOICE)
        check("kiosk: the app's settings get the mission voice", lab_kiosk.SETTINGS.get("voiceName") == run.VOICE)
        check("kiosk: the lab's own settings are kept",
              all(lab_kiosk.SETTINGS[k] == v for k, v in before.items()) and started == ["key-file"])

    try:
        import gemini_chat
    except Exception as e:
        skip("chat uses the mission prompt", f"{type(e).__name__}: {e}")
    else:
        gemini_chat.main = lambda: None
        run.chat([])

        async def setup():  # the lab's chat builds its fake robot inside the event loop
            return gemini_chat.DeskBackend().setup()

        prompt, tools = asyncio.run(setup())
        check("chat: Gemini gets the mission prompt", prompt == mission)
        check("chat: and the lab's tools, on the fake robot", {"stop", "stand_up", "move"} <= {t["name"] for t in tools})


if __name__ == "__main__":
    test_prompt()
    test_lab_tools()
    test_run()
    print()
    if FAILED:
        sys.exit(f"{len(FAILED)} failed: " + "; ".join(FAILED))
    print("all checks passed")
