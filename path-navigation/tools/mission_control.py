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
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
import threading
import time
import urllib.request
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(ROOT))
PAGE = HERE / "mission_control.html"
COMMANDS = ("start", "start!", "pause", "resume", "end", "abort", "phrase", "motion_on", "motion_off")
STAGES = ("SEEK", "ORIENT", "CONFIRM", "ZONE1", "REFIX", "ZONE2", "ARRIVE", "RESCUE")


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
        self.reset()
        threading.Thread(target=self._loop, daemon=True, name="simulation").start()

    def reset(self, options: dict | None = None) -> None:
        with self.lock:
            for key, value in (options or {}).items():
                if key in self.options:
                    self.options[key] = type(self.options[key])(value)
            self.options["seed"] += 1                   # a different room each time
            self.sim = self.CourseSim(self.cfg, self.SimOptions(**self.options), motion=True)
            self.sim.step()

    def _loop(self) -> None:
        from sim.course_sim import DT
        owed, last = 0.0, time.monotonic()
        while True:
            now = time.monotonic()
            owed = min(owed + (now - last) * self.speed, 2.0)
            last = now
            with self.lock:
                while owed >= DT:
                    self.sim.step()
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
                        self.options[key] = type(self.options[key])(value)
                        setattr(self.sim.opt, key, self.options[key])
        return True, "ok"

    def state(self) -> dict:
        with self.lock:
            snap = self.sim.snapshot()
        snap["server"] = {"mode": "sim", "speed": self.speed, "options": dict(self.options), "age": 0.0,
                          "connected": True}
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
def make_handler(backend):
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
            if path == "/api/command":
                ok, message = backend.command(str(body.get("command", "")).strip())
            elif path == "/api/sim":
                ok, message = backend.configure(body)
            else:
                self._json(404, {"ok": False, "message": "not found"})
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

    # On the robot the page has to be reachable from the laptop; a simulation on a laptop need not be.
    host = "127.0.0.1" if (args.local or (backend.mode == "sim" and not args.robot)) else "0.0.0.0"
    server = ThreadingHTTPServer((host, args.port), make_handler(backend))
    print("Pupper Mission Control (%s). Open:" % ("SIMULATION" if backend.mode == "sim" else "ROBOT"))
    for a in (addresses(args.port) if host == "0.0.0.0" else ["http://localhost:%d" % args.port]):
        print("   ", a)
    if args.open:
        threading.Timer(0.5, webbrowser.open, args=("http://localhost:%d" % args.port,)).start()
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
