#!/usr/bin/env python3
"""Pupper Mission Control: one page that shows the whole demo and starts it with one click.

    python3 tools/mission_control.py --sim      on a laptop: the full run in simulation (no robot, no ROS)
    python3 tools/mission_control.py --robot    on the Pupper: the real run (robot/start_demo.sh starts this)
    python3 tools/mission_control.py            --robot if ROS is here, otherwise --sim

Then open the address it prints (port 8078). The page shows the course with Pupper on it,
the sound bearings and the fix, terrain agility's view and state, the dodge reflex and the
balls, what Pupper is saying, and one cue for the people in the room. Its buttons:

    Start            run the mission from the start mark (one click)
    Allow motion     the dry-run switch: until it is on, navigation publishes no velocity
    Pause / Resume, End run
    a stage          start the run from that stage instead (Pupper placed there, facing along the line)
    Survivor said it stand in for the phrase detector
    the compass      click a direction to stand in for the sound module
    Stand up / Relax (robot only) ask the lab's robot server to stand Pupper up or let the legs go

In --robot mode it is a small ROS node: it reads /nav/telemetry and writes /nav/command, and
nothing else. It cannot arm the dodge node or move a leg by itself; everything goes through
robot/nav_node.py, which obeys the e-stop, the joystick and its own motion switch.

In --sim mode it runs sim/course_sim.py in this process: the real mission code and the real
terrain-agility state machine against a simulated room. Needs numpy.

The terminal is part of it. Every command, from the page or from `tools/pupper` in a terminal,
goes to the same /api/command and is written to the console (/api/console) in its terminal form,
with the mission's events and the output of the actions below. The page's console and
`tools/pupper watch` show the same lines. Actions are a fixed list of read-only checks (and, on
the robot, `git pull --ff-only` after a confirmation): named programs with fixed arguments, never
a shell and never text from the request.
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import socket
import subprocess
import sys
import threading
import time
import traceback
import urllib.request
import webbrowser
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))
PAGE = HERE / "mission_control.html"
COMMANDS = ("start", "start!", "pause", "resume", "end", "abort", "phrase", "motion_on", "motion_off")
STAGES = ("SEEK", "ORIENT", "CONFIRM", "ZONE1", "REFIX", "ZONE2", "ARRIVE", "RESCUE")


def cli_form(text: str) -> str:
    """A /nav/command string as it is typed in a terminal: `goto:ZONE2` -> `pupper goto ZONE2`."""
    named = {"start!": "start --anyway", "motion_on": "motion on", "motion_off": "motion off",
             "robot:stand": "stand", "robot:relax": "relax"}
    if text in named:
        return "pupper " + named[text]
    if text.startswith(("goto:", "bearing:", "nudge:")):
        verb, _, arg = text.partition(":")
        return "pupper %s %s" % (verb, arg)
    return "pupper " + text


def valid_command(text: str) -> bool:
    """Only the commands the page has buttons for reach the robot."""
    if text in COMMANDS:
        return True
    if text.startswith("goto:"):
        return text[5:] in STAGES
    if text.startswith("bearing:"):
        try:
            return -180.0 <= float(text[8:]) <= 180.0
        except ValueError:
            return False
    return False


# ------------------------------------------------------------------ the console: one log for the page and the terminal
class Console:
    """The last few hundred lines: commands (in their terminal form), mission events, Pupper's lines,
    state changes and the output of actions. Numbered, so a reader asks only for what is new."""

    def __init__(self, size: int = 800):
        self.lines: deque = deque(maxlen=size)
        self.n = 0
        self.lock = threading.Lock()

    def add(self, kind: str, text: str, src: str = "", **extra) -> None:
        with self.lock:
            self.n += 1
            self.lines.append(dict(extra, n=self.n, t=round(time.time(), 2), kind=kind, src=src, text=str(text)))

    def since(self, n: int, limit: int = 400) -> dict:
        with self.lock:
            new = [line for line in self.lines if line["n"] > n]
            return {"lines": new[-limit:], "last": self.n}


def watch_events(backend, console: Console, period: float = 0.25) -> None:
    """Copy the mission's new events and state changes into the console (a thread)."""
    seen, state = None, None
    while True:
        try:
            snap = backend.state()
        except Exception:                                   # noqa: BLE001  the console must not stop the page
            snap = {}
        nav = snap.get("nav") or {}
        events = snap.get("events") or []
        keys = [(e.get("t"), e.get("event"), e.get("run_s")) for e in events]
        if seen is None:
            new = []                                        # what was there before this server started is not news
        elif seen in keys:
            new = events[keys.index(seen) + 1:]
        else:
            new = events                                    # a new run or a new room: all of it is new
        if keys:
            seen = keys[-1]
        for e in new:
            details = ", ".join("%s %s" % kv for kv in (e.get("details") or {}).items())
            console.add("event", e.get("event", "").replace("_", " ") + (": " + details if details else ""),
                        src="mission", run_s=e.get("run_s"), state=e.get("state"))
            if e.get("line"):
                console.add("say", e["line"], src="pupper", run_s=e.get("run_s"))
        st = nav.get("state")
        if st and st != state:
            if state is not None:
                why = nav.get("reason") or ""
                console.add("state", "%s -> %s%s" % (state, st, (": " + why) if why else ""), src="mission",
                            run_s=nav.get("run_s"))
            state = st
        time.sleep(period)


# ------------------------------------------------------------------ actions: a fixed list, run by name
REPO = ROOT.parent                                           # sar-pupper/
ROBOT_LOG = Path("/tmp/sar-demo/robot.log")


def action_table(mode: str) -> list[dict]:
    """What the console's quick actions may run here. Fixed programs and arguments: nothing from a request."""
    py = sys.executable
    git = ["git", "-C", str(REPO)]
    both = [
        {"id": "git", "label": "Code version", "group": "code", "steps": [git + ["log", "--oneline", "-5"],
                                                                         git + ["status", "--short", "--", "path-navigation"]],
         "about": "the last five commits and what is changed in path-navigation"},
        {"id": "load", "label": "CPU load", "group": "system", "steps": [["uptime"]],
         "about": "load averages (the Pi has four cores)"},
    ]
    if mode == "robot":
        return both + [
            {"id": "log", "label": "Robot log", "group": "logs", "steps": [["tail", "-n", "60", str(ROBOT_LOG)]],
             "about": "the last 60 lines of %s" % ROBOT_LOG},
            {"id": "sound_log", "label": "Sound module log", "group": "logs",
             "steps": [["grep", "-a", "-m", "40", "-e", "Selected", "-e", "Four-channel", "-e", "Could not",
                        "-e", "bearing=", str(ROBOT_LOG)]],
             "about": "the partner's script's device lines and first bearings, from the robot log"},
            {"id": "audio", "label": "Audio devices", "group": "system",
             "steps": [["arecord", "-l"], ["pactl", "list", "short", "sources"]],
             "about": "capture devices: the DJI receiver should be listed"},
            {"id": "nodes", "label": "ROS nodes", "group": "system", "steps": [["ros2", "node", "list"]],
             "about": "every node running on the Pupper"},
            {"id": "top", "label": "Busiest processes", "group": "system",
             "steps": [["ps", "-eo", "pcpu,pmem,etime,comm", "--sort=-pcpu"]], "head": 14,
             "about": "what is using the CPU"},
            {"id": "temp", "label": "Pi temperature", "group": "system", "steps": [["vcgencmd", "measure_temp"]],
             "about": "throttling starts at 80 C"},
            {"id": "update", "label": "Update code (git pull)", "group": "code",
             "steps": [git + ["pull", "--ff-only"]], "timeout": 60,
             "confirm": "Pull the latest code from GitHub onto the Pupper (fast-forward only)?\n\n"
                        "Running nodes keep the old code until robot/start_demo.sh is run again.",
             "about": "git pull --ff-only: the Pupper's code as pushed to main"},
        ]
    return both + [
        {"id": "tests", "label": "Run the tests", "group": "checks", "timeout": 600,
         "steps": [[py, "-m", "pytest", str(ROOT / "tests"), "-q"]],
         "about": "path-navigation's tests (about 30 s)"},
        {"id": "sim20", "label": "20 simulated runs", "group": "checks", "timeout": 600,
         "steps": [[py, str(ROOT / "sim" / "course_sim.py"), "--runs", "20"]],
         "about": "how often the run ends in front of the cover, in 20 rooms"},
    ]


def shown(argv: list) -> str:
    """argv as a person would type it, with this repository's paths shortened."""
    out = []
    for a in argv:
        a = str(a)
        if a == sys.executable:
            a = "python3"
        elif a.startswith(str(REPO) + os.sep):
            a = os.path.relpath(a, str(ROOT))
        elif a == str(REPO):
            a = ".."
        out.append(a if a and not any(c in a for c in " '\"") else "'%s'" % a)
    return " ".join(out)


class Actions:
    """Runs one action at a time in a thread, its output line by line into the console."""

    def __init__(self, mode: str, console: Console):
        self.table = {a["id"]: a for a in action_table(mode)}
        self.console = console
        self.lock = threading.Lock()
        self.running = None                     # (id, Popen or None)
        self.stopped = False

    def listing(self) -> dict:
        with self.lock:
            running = self.running[0] if self.running else None
        return {"actions": [{"id": a["id"], "label": a["label"], "group": a["group"], "about": a.get("about", ""),
                             "confirm": a.get("confirm"), "shell": " ; ".join(shown(st) for st in a["steps"]),
                             "terminal": "pupper run " + a["id"]} for a in self.table.values()],
                "running": running}

    def start(self, ident: str, src: str = "page") -> tuple[bool, str]:
        act = self.table.get(ident)
        if act is None:
            return False, "no such action here (pupper actions lists them)"
        with self.lock:
            if self.running:
                return False, "%s is still running: wait, or stop it" % self.running[0]
            self.running, self.stopped = (ident, None), False
        self.console.add("cmd", "pupper run " + ident, src=src)
        threading.Thread(target=self._run, args=(act,), daemon=True, name="action-" + ident).start()
        return True, "started"

    def stop(self) -> tuple[bool, str]:
        with self.lock:
            if not self.running:
                return False, "nothing is running"
            self.stopped = True
            proc = self.running[1]
        if proc is not None and proc.poll() is None:
            proc.terminate()
        return True, "stopping"

    def _run(self, act: dict) -> None:
        ident, t0 = act["id"], time.monotonic()
        try:
            for argv in act["steps"]:
                if self.stopped:
                    break
                self.console.add("shell", "$ " + shown(argv), src=ident)
                if shutil.which(argv[0]) is None and not Path(argv[0]).exists():
                    self.console.add("err", "%s is not installed here" % argv[0], src=ident)
                    continue
                proc = subprocess.Popen(argv, cwd=str(ROOT), stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                        stdin=subprocess.DEVNULL, text=True, errors="replace", bufsize=1)
                with self.lock:
                    self.running = (ident, proc)
                timer = threading.Timer(float(act.get("timeout", 20)), proc.kill)
                timer.start()
                n, head = 0, act.get("head")
                for line in proc.stdout:
                    n += 1
                    if head is None or n <= head:
                        self.console.add("out", line.rstrip("\n"), src=ident)
                code = proc.wait()
                timer.cancel()
                if code:
                    self.console.add("err", "exit %s" % code, src=ident)
            self.console.add("info", "%s %s in %.1f s" % (act["label"], "stopped" if self.stopped else "done",
                                                          time.monotonic() - t0), src=ident)
        except OSError as exc:
            self.console.add("err", "%s: %s" % (act["label"], exc), src=ident)
        finally:
            with self.lock:
                self.running = None


# ------------------------------------------------------------------ simulation backend
class SimBackend:
    mode = "sim"

    def __init__(self, config=None):
        from nav.config import load_config
        from sim.course_sim import CourseSim, SimOptions
        self.CourseSim, self.SimOptions = CourseSim, SimOptions
        self.cfg = load_config(config)
        self.lock = threading.Lock()
        self.speed = 2.0
        self.options = {"seed": 1, "moe_deg": self.cfg.sound.moe_deg, "second_call": True, "throws": True,
                        "says_phrase": True, "survivor_calls": True}
        self.sim = None
        self.error = None                       # why the simulation stopped, if it did
        self.reset()
        threading.Thread(target=self._loop, daemon=True, name="simulation").start()

    def _option(self, key: str, value):
        """A page value in the option's own type. bool("false") is True, so booleans are read by hand."""
        current = self.options[key]
        if isinstance(current, bool):
            if isinstance(value, str):
                return value.strip().lower() in ("1", "true", "yes", "on")
            return bool(value)
        return type(current)(value)

    def reset(self, options: dict | None = None) -> None:
        with self.lock:
            for key, value in (options or {}).items():
                if key in self.options:
                    self.options[key] = self._option(key, value)
            self.options["seed"] += 1                   # a different room each time
            self.sim = self.CourseSim(self.cfg, self.SimOptions(**self.options), motion=True)
            self.error = None
            self.sim.step()

    def _loop(self) -> None:
        from sim.course_sim import DT
        owed, last = 0.0, time.monotonic()
        while True:
            now = time.monotonic()
            owed = min(owed + (now - last) * self.speed, 2.0)
            last = now
            with self.lock:
                while owed >= DT and self.error is None:
                    try:
                        self.sim.step()
                    except Exception as exc:        # noqa: BLE001  keep serving the page, say what stopped it
                        self.error = "%s: %s" % (type(exc).__name__, exc)
                        traceback.print_exc()
                        print("The simulation has stopped (%s). Reset it from the page, or restart." % self.error,
                              file=sys.stderr)
                    owed -= DT
            time.sleep(0.02)

    def command(self, text: str) -> tuple[bool, str]:
        with self.lock:
            if text.startswith("nudge:") or valid_command(text):
                self.sim.command(text)
                return True, "sent"
        return False, "not a command"

    def configure(self, body: dict) -> tuple[bool, str]:
        if "speed" in body:
            self.speed = max(0.0, min(20.0, float(body["speed"])))
        if body.get("action") == "reset":
            self.reset(body.get("options"))
        elif isinstance(body.get("options"), dict):
            with self.lock:
                for key, value in body["options"].items():
                    if key in self.options:
                        self.options[key] = self._option(key, value)
                        setattr(self.sim.opt, key, self.options[key])
        return True, "ok"

    def state(self) -> dict:
        with self.lock:
            snap = self.sim.snapshot()
        snap["server"] = {"mode": "sim", "speed": self.speed, "options": dict(self.options), "age": 0.0,
                          "connected": True, "error": self.error}
        return snap


# ------------------------------------------------------------------ robot backend
class RobotBackend:
    mode = "robot"

    def __init__(self, robot_server: str = "http://127.0.0.1:8000"):
        import rclpy
        from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
        from rclpy.node import Node
        from std_msgs.msg import String

        self.String = String
        self.robot_server = robot_server.rstrip("/")
        self.lock = threading.Lock()
        self.snap, self.t_snap = None, 0.0
        rclpy.init()
        self.node = Node("mission_control")
        self.pub = self.node.create_publisher(String, "/nav/command", 10)
        self.node.create_subscription(String, "/nav/telemetry", self._on_telemetry, 5)

        def spin():
            ex = SingleThreadedExecutor()
            ex.add_node(self.node)
            try:
                ex.spin()
            except (ExternalShutdownException, KeyboardInterrupt):
                pass

        threading.Thread(target=spin, daemon=True, name="ros").start()

    def _on_telemetry(self, msg) -> None:
        try:
            snap = json.loads(msg.data)
        except ValueError:
            return
        with self.lock:
            self.snap, self.t_snap = snap, time.monotonic()

    def _server(self, path: str) -> tuple[bool, str]:
        req = urllib.request.Request(self.robot_server + path, data=b"{}", method="POST",
                                     headers={"Content-Type": "application/json"})
        try:
            with urllib.request.urlopen(req, timeout=10.0) as r:
                out = json.loads(r.read().decode("utf-8", "replace"))
            return bool(out.get("success", True)), str(out.get("message", "done"))
        except Exception as exc:
            return False, "the robot server did not answer (%r). Is Pupper's voice started?" % (exc,)

    def command(self, text: str) -> tuple[bool, str]:
        if text == "robot:stand":
            return self._server("/walking/activate")
        if text == "robot:relax":
            return self._server("/walking/deactivate")
        if not valid_command(text):
            return False, "not a command"
        self.pub.publish(self.String(data=text))
        return True, "sent"

    def configure(self, body: dict) -> tuple[bool, str]:
        return False, "simulation settings do nothing on the robot"

    def state(self) -> dict:
        with self.lock:
            snap, age = self.snap, time.monotonic() - self.t_snap
        if snap is None:
            return {"mode": "robot", "server": {"mode": "robot", "connected": False, "age": None},
                    "waiting": "No word from the navigation node yet (/nav/telemetry). Is robot/start_demo.sh running?"}
        snap = dict(snap)
        snap["server"] = {"mode": "robot", "connected": age < 1.5, "age": round(age, 1)}
        return snap


# ------------------------------------------------------------------ the page
def make_handler(backend, console: Console | None = None, actions: Actions | None = None):
    console = console or Console()
    actions = actions or Actions(backend.mode, console)

    class Handler(BaseHTTPRequestHandler):
        server_version = "PupperMissionControl/1"

        def log_message(self, *args):               # quiet: the page polls ten times a second
            pass

        def _send(self, code: int, body: bytes, kind: str = "application/json") -> None:
            self.send_response(code)
            self.send_header("Content-Type", kind)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def _json(self, code: int, data) -> None:
            self._send(code, json.dumps(data, default=str).encode())

        def do_GET(self):
            path = self.path.split("?", 1)[0]
            if path in ("/", "/index.html"):
                self._send(200, PAGE.read_bytes(), "text/html; charset=utf-8")
            elif path == "/api/state":
                self._json(200, backend.state())
            elif path == "/api/console":
                query = self.path.partition("?")[2]
                params = dict(p.partition("=")[::2] for p in query.split("&") if p)
                try:
                    since = int(params.get("since", "0"))
                except ValueError:
                    since = 0
                self._json(200, console.since(since))
            elif path == "/api/actions":
                self._json(200, actions.listing())
            else:
                self._json(404, {"error": "not found"})

        def do_POST(self):
            path = self.path.split("?", 1)[0]
            # only a page served from here may press the buttons (a web page elsewhere cannot POST JSON to us)
            if "application/json" not in (self.headers.get("Content-Type") or ""):
                self._json(415, {"ok": False, "message": "JSON only"})
                return
            try:
                length = min(int(self.headers.get("Content-Length") or 0), 4096)
                body = json.loads(self.rfile.read(length) or b"{}")
            except ValueError:
                self._json(400, {"ok": False, "message": "not JSON"})
                return
            if not isinstance(body, dict):
                self._json(400, {"ok": False, "message": "a JSON object, please"})
                return
            src = "terminal" if (self.headers.get("X-Pupper-Client") or "").lower() == "cli" else "page"
            try:
                if path == "/api/command":
                    text = str(body.get("command", "")).strip()
                    ok, message = backend.command(text)
                    console.add("cmd" if ok else "err", cli_form(text) + ("" if ok else "   (refused: %s)" % message),
                                src=src)
                elif path == "/api/sim":
                    ok, message = backend.configure(body)
                    console.add("cmd" if ok else "err", "pupper sim " + " ".join(
                        "%s %s" % (k, json.dumps(v) if isinstance(v, (dict, list)) else v) for k, v in body.items()),
                        src=src)
                elif path == "/api/action":
                    if body.get("stop"):
                        ok, message = actions.stop()
                    else:
                        ok, message = actions.start(str(body.get("id", "")), src=src)
                else:
                    self._json(404, {"ok": False, "message": "not found"})
                    return
            except (TypeError, ValueError) as exc:      # a bad value from the page: say so, do not drop the line
                self._json(400, {"ok": False, "message": "bad value: %s" % exc})
                return
            self._json(200 if ok else 400, {"ok": ok, "message": message})

    return Handler


def addresses(port: int) -> list:
    out = ["http://localhost:%d" % port]
    host = socket.gethostname()
    if host:
        out.append("http://%s.local:%d" % (host.split(".")[0], port))
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("10.255.255.255", 1))
        out.append("http://%s:%d" % (s.getsockname()[0], port))
        s.close()
    except OSError:
        pass
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sim", action="store_true", help="simulate the run in this process (laptop)")
    ap.add_argument("--robot", action="store_true", help="talk to robot/nav_node.py over ROS (on the Pupper)")
    ap.add_argument("--port", type=int, default=8078)
    ap.add_argument("--config", default=None, help="another demo.yaml (simulation)")
    ap.add_argument("--open", action="store_true", help="open the page in the browser")
    ap.add_argument("--local", action="store_true", help="listen on this computer only (127.0.0.1)")
    args = ap.parse_args()

    backend = None
    if not args.sim:
        try:
            backend = RobotBackend()
        except ImportError as exc:
            if args.robot:
                sys.exit("--robot needs ROS (source robot/env.sh first): %s" % exc)
            print("ROS is not here (%s): running the simulation." % exc)
    if backend is None:
        try:
            backend = SimBackend(args.config)
        except ImportError as exc:
            sys.exit("the simulation needs numpy and the terrain-agility folder beside this one: %s" % exc)

    console = Console()
    console.add("info", "Mission Control started (%s). Terminal: tools/pupper watch" % backend.mode, src="server")
    threading.Thread(target=watch_events, args=(backend, console), daemon=True, name="console").start()
    actions = Actions(backend.mode, console)

    # On the robot the page has to be reachable from the laptop; a simulation on a laptop need not be.
    host = "127.0.0.1" if (args.local or (backend.mode == "sim" and not args.robot)) else "0.0.0.0"
    try:
        server = ThreadingHTTPServer((host, args.port), make_handler(backend, console, actions))
    except OSError as exc:
        sys.exit("Port %d is taken (%s): Mission Control may already be running. Open http://localhost:%d, "
                 "or start this one with --port <another>." % (args.port, exc.strerror or exc, args.port))
    print("Pupper Mission Control (%s). Open:" % ("SIMULATION" if backend.mode == "sim" else "ROBOT"))
    for a in (addresses(args.port) if host == "0.0.0.0" else ["http://localhost:%d" % args.port]):
        print("   ", a)
    print("From a terminal:  python3 %s watch      (help: tools/pupper --help)" % os.path.relpath(HERE / "pupper"))
    if args.open:
        threading.Timer(0.5, webbrowser.open, args=("http://localhost:%d" % args.port,)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
