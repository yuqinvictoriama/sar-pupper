"""The few things survivor identification asks of Gemini Pupper's robot server (FastAPI, port 8000).

Plain urllib, short timeouts, and every call returns (ok, message) instead of raising: the
mission carries on without the server (it then cannot clear Gemini's queue or play a trick).
The server answers {"success": bool, "message": str} with HTTP 200 even when it says no, so
ok here means "reached it AND it said yes".

    stop_now()       POST /stop/immediate    drop Gemini's queued moves, leave follow mode
    mode()           GET  /system/mode       "mode: locomotion", "mode: fallen over ...", "mode: motors idle ...", "mode: trick <name> ..."
    stand_up()       POST /walking/activate  queue "gemini_walk takes the legs" (stands up if lying down)
    tricks()         GET  /animations        {name: seconds}
    play(name)       POST /animation         QUEUES the trick and returns at once
    busy()           GET  /queue/status      is a job running or waiting?
    mission_event()  POST /mission/event     a line for Gemini to say (personality/mission_feed.py)
"""
from __future__ import annotations

import json
import urllib.error
import urllib.request


class RobotClient:
    def __init__(self, url: str = "http://127.0.0.1:8000", timeout_s: float = 4.0):
        self.url, self.timeout = url.rstrip("/"), float(timeout_s)

    def _call(self, method: str, path: str, body: dict | None = None, timeout: float | None = None):
        """(reached, parsed JSON or text)"""
        data = None if body is None else json.dumps(body).encode()
        req = urllib.request.Request(self.url + path, data=data, method=method,
                                     headers={"Content-Type": "application/json"} if data is not None else {})
        try:
            with urllib.request.urlopen(req, timeout=timeout or self.timeout) as r:
                text = r.read().decode("utf-8", "replace")
        except urllib.error.HTTPError as exc:
            return False, "HTTP %d %s" % (exc.code, exc.read().decode("utf-8", "replace")[:200])
        except Exception as exc:                               # refused, timed out...
            return False, repr(exc)
        try:
            return True, json.loads(text)
        except ValueError:
            return True, text

    def _result(self, method: str, path: str, body: dict | None = None, timeout: float | None = None):
        """(ok, message) for the endpoints that answer {"success", "message"}."""
        reached, out = self._call(method, path, body, timeout)
        if not reached:
            return False, str(out)
        if isinstance(out, dict):
            return bool(out.get("success", True)), str(out.get("message", ""))
        return True, str(out)

    def reachable(self) -> bool:
        return self._call("GET", "/health", timeout=1.0)[0]

    def stop_now(self):
        return self._result("POST", "/stop/immediate", {})

    def mode(self):
        return self._result("GET", "/system/mode", timeout=1.5)

    def stand_up(self):
        return self._result("POST", "/walking/activate", {}, timeout=10.0)

    def tricks(self):
        """(reached, {name: seconds})"""
        reached, out = self._call("GET", "/animations", timeout=2.0)
        if not reached or not isinstance(out, dict):
            return False, {}
        return True, {str(name): float((info or {}).get("duration_s", 0.0) or 0.0) if isinstance(info, dict) else 0.0
                      for name, info in out.items()}

    def play(self, name: str):
        return self._result("POST", "/animation", {"animation_name": name}, timeout=10.0)

    def busy(self):
        """(reached, a job is running or queued)"""
        reached, out = self._call("GET", "/queue/status", timeout=1.5)
        if not reached or not isinstance(out, dict):
            return False, False
        return True, bool(out.get("running")) or int(out.get("queued", 0) or 0) > 0

    def mission_event(self, name: str, **details):
        return self._result("POST", "/mission/event", {"event": name, "details": details}, timeout=1.5)
