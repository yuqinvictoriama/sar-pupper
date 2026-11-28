"""Mission Control's web server in simulation mode, in this process on a free port: it serves the
page and the state, and a bad request gets an answer instead of a dropped connection."""
import json
import sys
import threading
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / "tools"))
pytest.importorskip("numpy")
try:
    import mission_control as MC
    MC.SimBackend                                       # noqa: B018
    from sim.course_sim import CourseSim  # noqa: F401
except ImportError as exc:
    pytest.skip(f"simulation not importable: {exc}", allow_module_level=True)


@pytest.fixture(scope="module")
def server():
    backend = MC.SimBackend()
    backend.speed = 0.0                                 # hold the clock still: the test drives nothing
    srv = ThreadingHTTPServer(("127.0.0.1", 0), MC.make_handler(backend))
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    yield "http://127.0.0.1:%d" % srv.server_address[1], backend
    srv.shutdown()
    srv.server_close()


def post(base, path, body, kind="application/json"):
    data = body if isinstance(body, bytes) else json.dumps(body).encode()
    req = urllib.request.Request(base + path, data=data, headers={"Content-Type": kind}, method="POST")
    try:
        with urllib.request.urlopen(req, timeout=5) as r:
            return r.status, json.loads(r.read())
    except urllib.error.HTTPError as e:
        return e.code, json.loads(e.read())


def test_it_serves_the_page_and_the_state(server):
    base, _ = server
    with urllib.request.urlopen(base + "/", timeout=5) as r:
        assert r.status == 200 and b"<html" in r.read().lower()
    with urllib.request.urlopen(base + "/api/state", timeout=5) as r:
        snap = json.loads(r.read())
    assert snap["nav"]["state"] == "IDLE" and snap["server"]["mode"] == "sim" and snap["server"]["error"] is None


def test_commands_are_checked(server):
    base, _ = server
    assert post(base, "/api/command", {"command": "motion_off"}) == (200, {"ok": True, "message": "sent"})
    assert post(base, "/api/command", {"command": "rm -rf /"})[0] == 400
    assert post(base, "/api/command", {"command": "goto:NOWHERE"})[0] == 400
    assert post(base, "/api/command", {"command": "start"}, kind="text/plain")[0] == 415


def test_a_bad_value_gets_an_answer_not_a_dropped_connection(server):
    base, backend = server
    assert post(base, "/api/command", [1, 2])[0] == 400
    assert post(base, "/api/sim", {"speed": "fast"})[0] == 400
    assert post(base, "/api/sim", {"options": {"moe_deg": "wide"}})[0] == 400
    assert post(base, "/api/sim", {"speed": 0})[0] == 200


def test_switches_given_as_text_are_read_as_switches(server):
    base, backend = server
    assert post(base, "/api/sim", {"options": {"throws": "false"}})[0] == 200
    assert backend.options["throws"] is False and backend.sim.opt.throws is False
    assert post(base, "/api/sim", {"options": {"throws": True}})[0] == 200
    assert backend.options["throws"] is True


# ------------------------------------------------------------------ the terminal: console, actions, tools/pupper
def get(base, path):
    with urllib.request.urlopen(base + path, timeout=5) as r:
        return json.loads(r.read())


def post_cli(base, path, body):
    req = urllib.request.Request(base + path, data=json.dumps(body).encode(), method="POST",
                                 headers={"Content-Type": "application/json", "X-Pupper-Client": "cli"})
    with urllib.request.urlopen(req, timeout=5) as r:
        return json.loads(r.read())


def test_every_command_is_echoed_to_the_console_in_its_terminal_form(server):
    base, _ = server
    last = get(base, "/api/console?since=0")["last"]
    post(base, "/api/command", {"command": "goto:ZONE2"})
    post_cli(base, "/api/command", {"command": "motion_off"})
    post(base, "/api/command", {"command": "rm -rf /"})
    got = get(base, "/api/console?since=%d" % last)
    lines = [(x["src"], x["kind"], x["text"]) for x in got["lines"]]
    assert ("page", "cmd", "pupper goto ZONE2") in lines and ("terminal", "cmd", "pupper motion off") in lines
    assert any(k == "err" and t.startswith("pupper rm -rf /") and "refused" in t for _, k, t in lines)
    assert got["last"] == last + 3 and get(base, "/api/console?since=%d" % got["last"])["lines"] == []
    post(base, "/api/command", {"command": "end"})


def test_cli_form_matches_what_tools_pupper_accepts():
    pupper = load_pupper()
    for cmd in ("start", "start!", "pause", "resume", "end", "phrase", "motion_on", "motion_off", "goto:ZONE2",
                "goto:REFIX", "bearing:30", "bearing:-12.5", "robot:stand", "robot:relax"):
        typed = MC.cli_form(cmd).split()
        assert typed[0] == "pupper" and pupper.to_command(typed[1:]) == cmd, cmd
    assert pupper.to_command(["goto", "gauntlet"]) == "goto:ZONE2" and pupper.to_command(["goto", "nowhere"]) is None
    assert pupper.to_command(["bearing", "left"]) is None and pupper.to_command(["rm", "-rf"]) is None


def load_pupper():
    import importlib.machinery
    import importlib.util
    loader = importlib.machinery.SourceFileLoader("pupper_cli", str(ROOT / "tools" / "pupper"))
    spec = importlib.util.spec_from_loader("pupper_cli", loader)
    mod = importlib.util.module_from_spec(spec)
    loader.exec_module(mod)
    return mod


def test_actions_are_a_fixed_list_run_by_name_only(server):
    base, _ = server
    listed = get(base, "/api/actions")
    ids = {a["id"] for a in listed["actions"]}
    assert {"git", "load", "tests", "sim20"} <= ids and "update" not in ids        # git pull is robot only
    assert all(a["terminal"] == "pupper run " + a["id"] for a in listed["actions"])
    assert post(base, "/api/action", {"id": "uptime; rm -rf /"})[0] == 400
    assert post(base, "/api/action", {"id": "load", "steps": [["rm", "-rf", "/"]]})[0] == 200   # steps ignored
    for _ in range(100):
        if get(base, "/api/actions")["running"] is None:
            break
        import time
        time.sleep(0.05)
    lines = get(base, "/api/console?since=0")["lines"]
    shells = [x["text"] for x in lines if x["kind"] == "shell"]
    assert "$ uptime" in shells and not any("rm" in t for t in shells)


def test_robot_actions_never_include_a_shell_or_a_request_value():
    for mode in ("robot", "sim"):
        for act in MC.action_table(mode):
            for argv in act["steps"]:
                assert isinstance(argv, list) and argv[0] not in ("sh", "bash", "zsh", "sudo", "rm")
                assert not any(";" in str(a) or "|" in str(a) for a in argv)
    robot = {a["id"]: a for a in MC.action_table("robot")}
    assert robot["update"]["confirm"] and robot["update"]["steps"][0][-2:] == ["pull", "--ff-only"]


def test_tools_pupper_drives_the_server(server, capsys):
    base, _ = server
    pupper = load_pupper()
    assert pupper.main(["--url", base, "status"]) == 0 and "SIMULATION" in capsys.readouterr().out
    assert pupper.main(["--url", base, "goto", "nowhere"]) == 1
    assert pupper.main(["--url", base, "motion", "off"]) == 0
    assert get(base, "/api/console?since=0")["lines"][-1]["text"] == "pupper motion off"
    assert pupper.main(["--url", "http://127.0.0.1:9", "status"]) == 2             # nobody there: says so
