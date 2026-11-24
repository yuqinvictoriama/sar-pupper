#!/usr/bin/env python3
"""Run Gemini Pupper with the mission personality (personality/prompt.md).

    python3 personality/run.py chat              talk to it in text, on a fake robot: nothing moves
    python3 personality/run.py server            the robot server, serving the mission prompt
    python3 personality/run.py kiosk <key-file>  the kiosk on the robot's screen, in the mission voice

start.sh runs the last two on the robot. Nothing in gemini_pupper is edited:
each command imports the lab's own file (robot/gemini_chat.py, server.py,
kiosk.py) and changes one value in it before it runs.

  server   server.PROMPT_FILE      lab/prompt.md  ->  personality/prompt.md
           + mission_feed.install  adds /mission/event and /ws/mission (mission events for Gemini)
  kiosk    kiosk.SETTINGS          adds voiceName (the lab leaves the app's default, Puck)
  chat     gemini_chat.prompt_text reads personality/prompt.md

gemini_pupper is looked for next to sar-pupper (~/gemini_pupper on the robot,
pi/gemini_pupper in the laptop clone). Set GEMINI_PUPPER_DIR if it is elsewhere.
"""

from __future__ import annotations

import argparse
import asyncio
import os
import re
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
PROMPT_FILE = HERE / "prompt.md"
sys.path.insert(0, str(HERE))
import mission_feed  # noqa: E402

# The Gemini voice Pupper speaks with on a mission. The web app offers Puck,
# Charon, Kore, Fenrir and Zephyr. Google describes Charon as "Informative" and
# Puck, the app's default, as "Upbeat". README.md says how to audition others.
VOICE = "Charon"


def mission_prompt() -> str:
    """prompt.md without its <!-- comments -->, exactly as the robot server sends it."""
    return re.sub(r"<!--.*?-->", "", PROMPT_FILE.read_text(), flags=re.S).strip()


def lab_robot_dir() -> Path:
    """gemini_pupper/robot, where the lab's server, kiosk and chat live."""
    repo = Path(os.environ.get("GEMINI_PUPPER_DIR") or HERE.parent.parent / "gemini_pupper")
    robot = repo.expanduser() / "robot"
    if not (robot / "server.py").exists():
        sys.exit(f"gemini_pupper not found at {repo}. Set GEMINI_PUPPER_DIR to its folder.")
    return robot


def use_lab_code() -> Path:
    """Make the lab's robot/ files importable, the way they are when run in place."""
    robot = lab_robot_dir()
    if str(robot) not in sys.path:
        sys.path.insert(0, str(robot))
    return robot


def server() -> None:
    """The lab's robot server (port 8000), with the mission prompt in GET /tools."""
    os.chdir(use_lab_code())  # robot/start.sh starts server.py from robot/
    import uvicorn

    import server as lab_server  # needs ROS, like the lab's server

    lab_server.PROMPT_FILE = PROMPT_FILE  # system_prompt() reads this each time the app connects
    mission_feed.install(lab_server.app)  # POST /mission/event -> WS /ws/mission -> the app -> Gemini
    uvicorn.run(lab_server.app, host=os.environ.get("HOST", "127.0.0.1"), port=int(os.environ.get("PORT", "8000")))


def kiosk(key_file: str, voice: str) -> None:
    """The lab's kiosk, which also writes the mission voice into the app's settings."""
    use_lab_code()
    import kiosk as lab_kiosk

    lab_kiosk.SETTINGS["voiceName"] = voice
    asyncio.run(lab_kiosk.main(key_file))


def chat(args: list[str]) -> None:
    """The lab's text chat with the mission prompt (other arguments go to it unchanged)."""
    use_lab_code()
    import gemini_chat

    gemini_chat.prompt_text = lambda *_: mission_prompt()
    sys.argv = ["personality/run.py chat", *args]
    print("Mission personality. Try: start the search / MISSION EVENT: locked range=3 / MISSION EVENT: dodge\n")
    gemini_chat.main()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="command", required=True)
    sub.add_parser("server", help="the robot server with the mission prompt")
    k = sub.add_parser("kiosk", help="the kiosk with the mission voice")
    k.add_argument("key_file", help="file holding the Gemini API key (~/.config/gemini-pupper/api_key)")
    k.add_argument("--voice", default=VOICE, help=f"Gemini voice (default {VOICE})")
    sub.add_parser("chat", help="talk to it in text; extra arguments go to the lab's chat (--robot, --show, --model)")
    args, rest = ap.parse_known_args()
    if args.command == "server":
        server()
    elif args.command == "kiosk":
        kiosk(args.key_file, args.voice)
    else:
        chat(rest)


if __name__ == "__main__":
    main()
