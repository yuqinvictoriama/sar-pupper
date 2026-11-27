"""Mission events for Gemini: the robot's reflexes report, the app tells Gemini, Gemini speaks.

The navigation, dodge and terrain nodes cannot talk to Gemini; only the web app holds the
Live session. This adds three routes to the lab's robot server (run.py installs them; nothing
in gemini_pupper/robot changes):

    POST /mission/event   {"event": "heard", "details": {"distance": 3}}
                          from personality/event_bridge.py (or run.py say), on the robot itself
    WS   /ws/mission      each event as one text line, "MISSION EVENT: heard distance=3",
                          to every open copy of the app, which passes it to the Live session
    GET  /mission/recent  who is listening, where the mission stands, the last few lines
                          and whether a line is still being spoken (speaking, quiet_in_s)

What the feed decides, so that no sender has to (MissionFeed.post):

  - Only events in callouts.py get through, and only with plain names and numbers: a mission
    event is an instruction to Gemini's voice, so nothing free-form reaches it. The server
    listens on 127.0.0.1 only, so the sender is a program on the robot.
  - The same event twice in a few seconds is said once. A low-priority line (a dodge) never
    cuts across a line that is still being spoken.
  - If no app is listening (no network, session dropped, web app not rebuilt), the robot's own
    voice says the line (voice.py). The answer says who spoke, so a silent mission is visible.
  - It keeps where the mission stands (searching, coming, here). A dodge is sent with it, and
    an app that connects in mid-mission is told it, because a new Gemini session has forgotten
    everything said before.
"""
import re
import time
from collections import deque

from callouts import CALLOUTS, OFF, REFRAINS, postable, speak_seconds, unknown
from voice import LocalVoice

NAME = re.compile(r"^[a-z][a-z_]{0,31}$")
KEY = re.compile(r"^[a-z][a-z_]{0,23}$")
VALUE = re.compile(r"^[A-Za-z0-9_.+-]{1,24}$")
MAX_DETAILS = 6
RESUME_WINDOW_S = 600.0   # a mission that went quiet longer ago than this is not picked up again


def line(event: str, details: dict | None = None) -> str:
    """'MISSION EVENT: <name> [key=value ...]', or ValueError if anything is not plain."""
    event = str(event).strip()
    if not NAME.match(event):
        raise ValueError("event name must be lower-case letters and underscores")
    if not isinstance(details or {}, dict):
        raise ValueError("details must be a set of key=value pairs")
    parts = ["MISSION EVENT:", event]
    for key, value in list((details or {}).items())[:MAX_DETAILS]:
        if isinstance(value, bool):
            value = "yes" if value else "no"
        elif isinstance(value, float):
            value = ("%.1f" % value).rstrip("0").rstrip(".")
        value = str(value)
        if not KEY.match(str(key)) or not VALUE.match(value):
            raise ValueError("detail %r=%r is not a plain word or number" % (key, value))
        parts.append("%s=%s" % (key, value))
    return " ".join(parts)


class MissionFeed:
    def __init__(self, keep: int = 20, voice: LocalVoice | None = None, clock=time.monotonic):
        self.clients: set = set()
        self.recent: deque = deque(maxlen=keep)
        self.voice = voice or LocalVoice()
        self.clock = clock
        self.status = OFF          # where the mission stands: off, searching, coming, here
        self.saved = False         # Gemini has been told to start the comfort steps
        self.last: dict = {}       # event name -> when it was last said
        self.last_event = None     # when anything was last said
        self.busy_until = 0.0      # when the line being spoken should be over

    # -------------------------------------------------------------- sending
    async def send(self, text: str) -> int:
        """To every connected app. Returns how many got it."""
        sent = 0
        for ws in list(self.clients):
            try:
                await ws.send_text(text)
                sent += 1
            except Exception:
                self.clients.discard(ws)
        return sent

    def note(self, text: str, by: str) -> None:
        self.recent.append({"t": round(time.time(), 2), "line": text, "by": by})

    async def post(self, event, details: dict | None = None) -> dict:
        """One mission event in; who said it out. ValueError if it may not be said at all."""
        name = str(event).strip()
        callout = CALLOUTS.get(name)
        if callout is None or callout.internal:
            if not NAME.match(name):
                raise ValueError("event name must be lower-case letters and underscores; known: " + ", ".join(postable()))
            raise ValueError(unknown(name))
        details = dict(details or {}) if isinstance(details or {}, dict) else details
        line(name, details)   # refuse anything not plain before it changes any state
        now = self.clock()

        if name == "start":   # a new search: nothing from the last one carries over
            self.saved = False
            self.last = {k: v for k, v in self.last.items() if k == "start"}
        if callout.status is not None:
            self.status = callout.status
        if name == "saved" and self.saved:
            return self.skipped(name, "Gemini has already started the comfort steps")

        since = now - self.last.get(name, -1e9)
        if since < callout.min_gap_s:
            return self.skipped(name, "said %.0f s ago" % since)
        if not callout.interrupts and now < self.busy_until:
            return self.skipped(name, "Pupper is in the middle of another line")

        if callout.refrain and self.status in REFRAINS:
            details["status"] = self.status
        text = line(name, details)
        listeners = await self.send(text)
        if listeners:
            by, message = "gemini", text
        else:
            words = callout.aloud(self.status)
            how = self.voice.say(callout.clip(self.status), words)
            by = "local" if how else "nobody"
            message = ("no Gemini app is connected; the robot's own voice (%s) said: %s" % (how, words) if how else
                       "NOT SAID: no Gemini app is connected, and the robot has no voice of its own "
                       "(no recording in personality/clips, no espeak-ng)")
        self.note(text, by)
        self.last_event = now
        seconds = speak_seconds(callout.aloud(self.status)) if by != "nobody" else 0.0
        if by != "nobody":
            self.last[name] = now
            self.busy_until = now + seconds
        if name == "saved" and by == "gemini":
            # Only Gemini can ask the questions. If the robot's own voice said steps 1 to 3,
            # a Gemini that connects later is not told the steps are done.
            self.saved = True
        return {"success": by != "nobody", "message": message, "line": text, "listeners": listeners,
                "spoken_by": by, "speak_s": seconds, "status": self.status}

    def skipped(self, name: str, why: str) -> dict:
        return {"success": True, "message": "%s not said: %s" % (name, why), "line": "",
                "listeners": len(self.clients), "spoken_by": "skipped", "speak_s": 0.0, "status": self.status}

    # -------------------------------------------------------------- an app connects
    def resume_line(self) -> str:
        """What a newly connected app is told, or '' if no mission is under way."""
        if self.status not in REFRAINS or self.last_event is None:
            return ""
        if self.clock() - self.last_event > RESUME_WINDOW_S:
            return ""
        details = {"status": self.status}
        if self.saved:
            details["saved"] = True
        return line("reconnected", details)

    async def joined(self, ws) -> None:
        self.clients.add(ws)
        text = self.resume_line()
        if text:
            try:
                await ws.send_text(text)
            except Exception:
                self.clients.discard(ws)
                return
            self.note(text, "gemini")
            self.busy_until = self.clock() + speak_seconds(REFRAINS[self.status])

    def summary(self) -> dict:
        # quiet_in_s: how long until the line being spoken should be over (0 = nothing is being
        # said). An estimate from the line's length, not a measurement: Gemini's audio plays in
        # the browser, and the server cannot hear it.
        quiet = max(0.0, self.busy_until - self.clock())
        return {"listeners": len(self.clients), "status": self.status, "saved": self.saved,
                "speaking": quiet > 0.0, "quiet_in_s": round(quiet, 1),
                "local_voice": self.voice.describe(), "events": postable(), "recent": list(self.recent)}


def install(app, feed: MissionFeed | None = None) -> MissionFeed:
    """Add the routes to a FastAPI app, ahead of its catch-all static mount."""
    # Imported here so line() and MissionFeed work on a desk without FastAPI. (No "from
    # __future__ import annotations" in this file: FastAPI must see the real WebSocket class
    # on ws_mission.)
    from fastapi import Body, HTTPException, WebSocket, WebSocketDisconnect
    from starlette.routing import Mount

    feed = feed or MissionFeed()

    async def post_event(body: dict = Body(...)):
        try:
            return await feed.post(body.get("event", ""), body.get("details") or {})
        except (ValueError, AttributeError) as exc:
            raise HTTPException(422, str(exc))

    async def recent():
        return feed.summary()

    async def ws_mission(ws: WebSocket):
        await ws.accept()
        await feed.joined(ws)
        try:
            while True:
                await ws.receive_text()
        except WebSocketDisconnect:
            pass
        finally:
            feed.clients.discard(ws)

    routes = app.router.routes
    before = len(routes)
    app.add_api_route("/mission/event", post_event, methods=["POST"])
    app.add_api_route("/mission/recent", recent, methods=["GET"])
    app.add_api_websocket_route("/ws/mission", ws_mission)
    # The server mounts the web app at "/", which swallows every path that comes after it in
    # the list. Move the new routes in front of it.
    new = routes[before:]
    del routes[before:]
    at = next((i for i, r in enumerate(routes) if isinstance(r, Mount) and r.path in ("", "/")), len(routes))
    routes[at:at] = new
    return feed
