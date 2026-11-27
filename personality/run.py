#!/usr/bin/env python3
"""Run Gemini Pupper with the mission personality (personality/prompt.md).

    python3 personality/run.py chat              talk to it in text, on a fake robot: nothing moves
    python3 personality/run.py prompt            print the prompt exactly as Gemini gets it
    python3 personality/run.py say heard         send one mission event to the running server, and
                                                 see who said it (Gemini, the robot's own voice, nobody)
    python3 personality/run.py doctor            why is Pupper not talking? Checks every link
    python3 personality/run.py clips             record the robot's own voice (laptop, needs the Gemini key)
    python3 personality/run.py server            the robot server, serving the mission prompt
    python3 personality/run.py kiosk <key-file>  the kiosk on the robot's screen, in the mission voice

start.sh runs the last two on the robot. Nothing in gemini_pupper is edited:
each command imports the lab's own file (robot/gemini_chat.py, server.py,
kiosk.py) and changes one value in it before it runs.

  server   server.system_prompt    lab/prompt.md  ->  personality/prompt.md + the call-outs
           + mission_feed.install  adds /mission/event and /ws/mission (mission events for Gemini)
  kiosk    kiosk.SETTINGS          adds voiceName (the lab leaves the app's default, Puck)
  chat     gemini_chat.prompt_text the same mission prompt

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
import callouts  # noqa: E402
import mission_feed  # noqa: E402

# The Gemini voice Pupper speaks with on a mission. The web app offers Puck,
# Charon, Kore, Fenrir and Zephyr. Google describes Charon as "Informative" and
# Puck, the app's default, as "Upbeat". README.md says how to audition others.
VOICE = "Charon"
PLACEHOLDER = "{{CALLOUTS}}"   # the line in prompt.md that callouts.py fills in
KEY_FILE = Path.home() / ".config" / "gemini-pupper" / "api_key"
SERVER = "http://127.0.0.1:8000"


def mission_prompt() -> str:
    """The prompt as Gemini gets it: prompt.md without its <!-- comments -->, with the call-outs in."""
    text = re.sub(r"<!--.*?-->", "", PROMPT_FILE.read_text(), flags=re.S).strip()
    lines = callouts.prompt_lines()
    if PLACEHOLDER in text:
        return text.replace(PLACEHOLDER, lines)
    # Someone deleted the placeholder. A prompt without call-outs is a Pupper that says
    # nothing on a mission, so they go at the end. test_personality.py reports it.
    return text + "\n\n# Call-outs\nWhen a mission event arrives, say its line.\n" + lines


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


def give_prompt(lab_server) -> None:
    """Make the lab's server send the mission prompt in GET /tools."""
    if not callable(getattr(lab_server, "system_prompt", None)):
        sys.exit("the lab's server.py no longer has system_prompt(); personality/run.py needs updating to match it")
    lab_server.PROMPT_FILE = PROMPT_FILE          # what the lab's own function would read
    lab_server.system_prompt = mission_prompt     # called each time the app connects


def server() -> None:
    """The lab's robot server (port 8000), with the mission prompt in GET /tools."""
    os.chdir(use_lab_code())  # robot/start.sh starts server.py from robot/
    import uvicorn

    import server as lab_server  # needs ROS, like the lab's server

    give_prompt(lab_server)
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
    print("Mission personality. Try: start the search / MISSION EVENT: heard / MISSION EVENT: dodge status=coming\n")
    gemini_chat.main()


def details_from(pairs: list[str]) -> dict:
    """['distance=3', 'saved=yes'] -> {'distance': 3, 'saved': 'yes'}."""
    out: dict = {}
    for pair in pairs:
        key, eq, value = pair.partition("=")
        if not eq or not key or not value:
            sys.exit(f"'{pair}' is not key=value")
        try:
            out[key] = int(value)
        except ValueError:
            try:
                out[key] = float(value)
            except ValueError:
                out[key] = value
    return out


def say(event: str, pairs: list[str], url: str) -> None:
    """Send one mission event to the running server, as the event bridge would."""
    import event_bridge  # its post() and nothing else: no ROS needed

    ok, text = event_bridge.post(url, event, details_from(pairs), timeout=5.0)
    print(text)
    if not ok:
        sys.exit(1)


def api_key(key_file: str | None) -> str:
    key = os.environ.get("GEMINI_API_KEY") or os.environ.get("GOOGLE_API_KEY")
    path = Path(key_file).expanduser() if key_file else KEY_FILE
    if not key and path.exists():
        key = path.read_text().strip()
    if not key:
        sys.exit(f"No Gemini key: set GEMINI_API_KEY, or put it in {path}.")
    return key


def clips(voice_name: str, model: str, key_file: str | None, only: list[str]) -> None:
    """Record every call-out in the mission voice, into personality/clips/."""
    import voice

    texts = callouts.clips()
    unknown = [n for n in only if n not in texts]
    if unknown:
        sys.exit(f"no such recording: {', '.join(unknown)}; known: {', '.join(texts)}")
    print(f"Recording {len(only) or len(texts)} lines as {voice_name} with {model} into {voice.CLIPS}")
    made = voice.make_clips(api_key(key_file), voice_name, texts, model=model, only=only or None)
    print(f"{len(made)} recorded. Listen to them before the robot relies on them, then commit the folder.")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="command", required=True)
    sub.add_parser("server", help="the robot server with the mission prompt")
    k = sub.add_parser("kiosk", help="the kiosk with the mission voice")
    k.add_argument("key_file", help="file holding the Gemini API key (~/.config/gemini-pupper/api_key)")
    k.add_argument("--voice", default=VOICE, help=f"Gemini voice (default {VOICE})")
    sub.add_parser("chat", help="talk to it in text; extra arguments go to the lab's chat (--robot, --show, --model)")
    sub.add_parser("prompt", help="print the prompt exactly as Gemini gets it")
    s = sub.add_parser("say", help="send one mission event to the running server")
    s.add_argument("event", help="one of: " + ", ".join(callouts.postable()))
    s.add_argument("details", nargs="*", help="key=value, e.g. distance=3")
    s.add_argument("--url", default=SERVER, help=f"the robot server (default {SERVER})")
    sub.add_parser("doctor", help="check every link between a mission event and Pupper's voice")
    c = sub.add_parser("clips", help="record the robot's own voice for when Gemini is not connected")
    c.add_argument("only", nargs="*", help="just these recordings (default: all)")
    c.add_argument("--voice", default=VOICE, help=f"Gemini voice (default {VOICE})")
    c.add_argument("--model", default=None, help="Gemini speech model (default: voice.TTS_MODEL)")
    c.add_argument("--key-file", default=None, help=f"file holding the Gemini key (default {KEY_FILE})")
    args, rest = ap.parse_known_args()
    if args.command == "chat":
        chat(rest)
        return
    if args.command == "doctor":
        import doctor
        sys.exit(doctor.main(rest))
    if rest:
        ap.error("unrecognised arguments: " + " ".join(rest))
    if args.command == "server":
        server()
    elif args.command == "kiosk":
        kiosk(args.key_file, args.voice)
    elif args.command == "prompt":
        print(mission_prompt())
    elif args.command == "say":
        say(args.event, args.details, args.url)
    elif args.command == "clips":
        import voice
        clips(args.voice, args.model or voice.TTS_MODEL, args.key_file, args.only)


if __name__ == "__main__":
    main()
