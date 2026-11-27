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
