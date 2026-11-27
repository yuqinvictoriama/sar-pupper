#!/usr/bin/env python3
"""Why is Pupper not talking? Follow a mission event from the robot to the speaker, link by link.

    python3 personality/doctor.py           check everything, print what is wrong and the fix
    python3 personality/doctor.py --say     then send the `test` event and report who said it

Run it on the robot after personality/start.sh (start.sh runs it itself at the end). It only
reads: nothing is changed and nothing moves. `--say` makes Pupper say "Voice check. Pupper here."

A line is spoken only if every one of these holds, and each used to fail without a sound:

    the built web app listens for mission events     (web/dist rebuilt after geminiLive.ts changed)
    the robot server is the mission one              (personality/start.sh, not the lab's start.sh)
    a Gemini app is connected to the feed            (key, network, live session)
    the sound goes to the robot's speaker            (not to a USB microphone receiver; not muted)
    Gemini's microphone is one Chromium may open     (not held open directly by another program)
    something sends the events                       (event_bridge.py, or the navigation node)

When Gemini is the missing link, the robot's own voice (voice.py) says the line instead.
"""
from __future__ import annotations

import json
import os
import re
import shutil
import socket
import subprocess
import sys
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import callouts  # noqa: E402
from voice import LocalVoice  # noqa: E402

SERVER = "http://127.0.0.1:8000"
KEY_FILE = Path(os.environ.get("GEMINI_KEY_FILE") or Path.home() / ".config" / "gemini-pupper" / "api_key")
GEMINI_HOST = "generativelanguage.googleapis.com"
SOUND_SERVERS = ("pipewire", "pipewire-pulse", "wireplumber", "pulseaudio", "pipewire-media-", "jackd")
LOOKS_LIKE_A_MIC = re.compile(r"\bmic\b|microphone|dji|wireless|receiver|webcam|camera", re.I)

results: list[str] = []


def report(level: str, text: str, fix: str = "") -> None:
    """level: ok, PROBLEM or note."""
    results.append(level)
    print("  %-8s%s" % (level, text))
    if fix and level != "ok":
        print("          -> " + fix)


# ------------------------------------------------------------------ small helpers (desk-tested)
def run(cmd: list[str], timeout: float = 3.0) -> str | None:
    """A command's output, or None if it is not installed or failed."""
    if not shutil.which(cmd[0]):
        return None
    try:
        out = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.TimeoutExpired):
        return None
    return out.stdout if out.returncode == 0 else None


def get(url: str, timeout: float = 2.0):
    """(reached, JSON or None)."""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as r:
            text = r.read().decode("utf-8", "replace")
    except Exception:
        return False, None
    try:
        return True, json.loads(text)
    except ValueError:
        return True, None


def parse_volume(text: str | None) -> tuple[float | None, bool]:
    """wpctl get-volume: 'Volume: 0.40 [MUTED]' -> (0.4, True)."""
    m = re.search(r"Volume:\s*([0-9.]+)", text or "")
    return (float(m.group(1)) if m else None), "MUTED" in (text or "")


def parse_description(text: str | None) -> str:
    """wpctl inspect: the device's name as a person would know it."""
    for key in ("node.description", "node.nick", "node.name"):
        m = re.search(r"%s\s*=\s*\"([^\"]*)\"" % re.escape(key), text or "")
        if m:
            return m.group(1)
    return ""


def device_holders(asound: str = "/proc/asound", proc: str = "/proc") -> list[dict]:
    """Programs holding a sound device open directly: card, direction, pid, command.

    A device opened straight through ALSA belongs to that one program. The sound server
    (PipeWire) shares a device between programs, so it is the one holder that is fine.
    """
    out = []
    for status in sorted(Path(asound).glob("card*/pcm*/sub*/status")):
        try:
            m = re.search(r"owner_pid\s*:\s*(\d+)", status.read_text())
        except OSError:
            continue
        if not m:
            continue        # "closed"
        pid = int(m.group(1))
        card, pcm = status.parents[2], status.parents[1].name
        try:
            name = (card / "id").read_text().strip()
        except OSError:
            name = card.name
        try:
            command = (Path(proc) / str(pid) / "comm").read_text().strip()
        except OSError:
            command = "?"
        out.append({"card": name, "direction": "microphone" if pcm.endswith("c") else "speaker",
                    "pid": pid, "command": command})
    return out


def is_sound_server(command: str) -> bool:
    return any(command.startswith(s) for s in SOUND_SERVERS)


def built_app(gp: Path) -> tuple[bool, bool, bool]:
    """(the web app is built, the built app listens for mission events, the source does)."""
    assets, source = gp / "web" / "dist" / "assets", gp / "web" / "services" / "geminiLive.ts"

    def has(path: Path) -> bool:
        try:
            return "ws/mission" in path.read_text(errors="ignore")
        except OSError:
            return False

    scripts = list(assets.glob("*.js")) if assets.is_dir() else []
    return bool(scripts), any(has(f) for f in scripts), has(source)


# ------------------------------------------------------------------ the checks
def check_web_app(gp: Path) -> None:
    print("The web app (%s)" % gp)
    if not (gp / "robot" / "server.py").exists():
        report("PROBLEM", "gemini_pupper is not there", "set GEMINI_PUPPER_DIR to its folder")
        return
    built, listens, source = built_app(gp)
    setup = "bash %s/robot/setup.sh   (rebuilds web/dist; a few minutes), then stop.sh and start.sh" % gp
    if not built:
        report("PROBLEM", "the web app is not built", setup)
    elif listens:
        report("ok", "the built app listens for mission events")
    elif source:
        report("PROBLEM", "the built app does not listen for mission events, though its source does: "
               "web/dist is older than the change. Gemini never hears an event", setup)
    else:
        report("PROBLEM", "the app has no mission-event listener at all: web/services/geminiLive.ts here is the "
               "lab's original. Gemini never hears an event",
               "copy web/services/geminiLive.ts from the laptop clone (the change is not committed there), then " + setup)


def check_server(url: str) -> dict:
    """Returns the feed's summary ({} if there is none)."""
    print("The robot server (%s)" % url)
    up, _ = get(url + "/health")
    if not up:
        report("PROBLEM", "not running", "bash personality/start.sh --kiosk")
        return {}
    report("ok", "running")
    _, spec = get(url + "/tools", timeout=5.0)
    prompt = (spec or {}).get("prompt", "") if isinstance(spec, dict) else ""
    if "search and rescue robot dog" not in prompt:
        report("PROBLEM", "it is the lab's server, not the mission one: Gemini has the lab's prompt and no mission events",
               "bash personality/stop.sh, then bash personality/start.sh --kiosk")
    elif "{{" in prompt or "- heard:" not in prompt:
        report("PROBLEM", "the mission prompt it serves has no call-outs", "python3 personality/test_personality.py")
    else:
        report("ok", "serves the mission prompt, call-outs included (%d words)" % len(prompt.split()))
    reached, feed = get(url + "/mission/recent")
    if not reached or not isinstance(feed, dict):
        report("PROBLEM", "no mission-event feed on it", "start it with personality/start.sh, not the lab's start.sh")
        return {}
    if feed.get("events") != callouts.postable():
        report("PROBLEM", "it was started before callouts.py last changed: it knows other events than this folder does",
               "bash personality/stop.sh, then bash personality/start.sh --kiosk")
    else:
        report("ok", "has the mission-event feed, with this folder's %d events" % len(callouts.postable()))
    return feed


def check_gemini(feed: dict) -> None:
    print("Gemini")
    try:
        have_key = KEY_FILE.stat().st_size > 0
    except OSError:
        have_key = False
    report("ok" if have_key else "PROBLEM", "the Gemini key file is there" if have_key else "no Gemini key at %s" % KEY_FILE,
           "bash ~/gemini_pupper/robot/setup.sh asks for it")
    try:
        socket.create_connection((GEMINI_HOST, 443), timeout=3).close()
        report("ok", "the network reaches Gemini")
    except OSError:
        report("PROBLEM", "the network does not reach Gemini (%s): no Gemini voice" % GEMINI_HOST,
               "join a network; until then the robot's own voice says the call-outs")
    if not feed:
        return
    n = int(feed.get("listeners", 0) or 0)
    if n == 1:
        report("ok", "one Gemini app is connected to the feed; the mission stands at: %s" % feed.get("status"))
    elif n > 1:
        report("note", "%d apps are connected to the feed (the kiosk and a laptop tab?): each will say every line" % n,
               "close the extra tab")
    else:
        report("PROBLEM", "no Gemini app is connected to the feed: no live session, or the built app does not listen",
               "on the robot's screen the kiosk presses WAKE PUPPER by itself (tail /tmp/gemini-pupper/kiosk.log); "
               "from a laptop, press it in the app")
    last = (feed.get("recent") or [])[-3:]
    for item in last:
        print("          last: %s  (said by %s)" % (item.get("line"), item.get("by", "?")))


def check_sound() -> None:
    print("Sound")
    if not shutil.which("wpctl"):
        report("note", "no wpctl here (not the robot?): speaker and microphone not checked")
    else:
        for what, target, word in (("speaker", "@DEFAULT_AUDIO_SINK@", "comes out of"),
                                   ("microphone", "@DEFAULT_AUDIO_SOURCE@", "listens on")):
            name = parse_description(run(["wpctl", "inspect", target]))
            volume, muted = parse_volume(run(["wpctl", "get-volume", target]))
            if not name and volume is None:
                report("PROBLEM", "no default %s: is PipeWire running for this user?" % what, "wpctl status")
                continue
            text = "Gemini %s: %s (volume %s%s)" % (word, name or "?", "?" if volume is None else "%.0f%%" % (volume * 100),
                                                   ", MUTED" if muted else "")
            pick = "wpctl status lists the devices; wpctl set-default <id> picks one"
            if muted or (volume is not None and volume < 0.2):
                report("PROBLEM", text, "wpctl set-mute %s 0; wpctl set-volume %s 1.0" % (target, target))
            elif what == "speaker" and LOOKS_LIKE_A_MIC.search(name):
                report("PROBLEM", text + ". That looks like a microphone receiver, not the robot's speaker: "
                       "Pupper is talking into it", pick)
            else:
                report("ok" if what == "speaker" else "note", text, "if that is not the one you mean: " + pick)
    for h in device_holders():
        if is_sound_server(h["command"]):
            continue
        consequence = ("Chromium cannot open it, so Gemini cannot hear through it" if h["direction"] == "microphone"
                       else "Gemini's voice cannot come out of it")
        report("PROBLEM", "the %s '%s' is held open directly by %s (pid %d): while it is, %s"
               % (h["direction"], h["card"], h["command"], h["pid"], consequence),
               "have that program open the shared device (name 'pipewire' or 'default'), not the card itself")


def check_local_voice() -> None:
    print("The robot's own voice (used when no Gemini app is connected)")
    voice = LocalVoice()
    text = voice.describe()
    if text.startswith("none"):
        report("PROBLEM", text + ": with Gemini away, Pupper is silent",
               "sudo apt install espeak-ng, or on a laptop: python3 personality/run.py clips, then copy personality/clips")
        return
    report("ok", text)
    missing = [n for n in callouts.clips() if not (voice.clips / (n + ".wav")).is_file()]
    if missing and voice.clips.is_dir() and any(voice.clips.glob("*.wav")):
        report("note", "no recording for: " + ", ".join(missing), "python3 personality/run.py clips " + " ".join(missing))


def check_senders() -> None:
    print("Who sends the events")
    if not shutil.which("pgrep"):
        report("note", "no pgrep here: not checked")
        return
    bridge = subprocess.run(["pgrep", "-f", "[p]ersonality/event_bridge.py"], capture_output=True).returncode == 0
    report("ok" if bridge else "PROBLEM",
           "the event bridge is running (/nav/event, /dodge/event -> the feed)" if bridge else
           "the event bridge is not running: nothing passes ROS events to the feed",
           "bash personality/start.sh starts it; its log is /tmp/gemini-pupper/event_bridge.log")


def main(argv: list[str] | None = None) -> int:
    argv = sys.argv[1:] if argv is None else argv
    if any(a in ("-h", "--help") for a in argv):
        print(__doc__)
        return 0
    del results[:]
    gp = Path(os.environ.get("GEMINI_PUPPER_DIR") or HERE.parent.parent / "gemini_pupper").expanduser()
    check_web_app(gp)
    feed = check_server(SERVER)
    check_gemini(feed)
    check_sound()
    check_local_voice()
    check_senders()
    if "--say" in argv and feed:
        import event_bridge
        ok, text = event_bridge.post(SERVER, "test", {}, timeout=5.0)
        print("Voice check\n  %-8s%s" % ("ok" if ok else "PROBLEM", text))
        results.append("ok" if ok else "PROBLEM")
    bad = results.count("PROBLEM")
    print("\n%d checks, %s" % (len(results), "all OK" if not bad else "%d PROBLEM%s" % (bad, "" if bad == 1 else "S")))
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
