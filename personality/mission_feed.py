"""Mission events for Gemini: the robot's reflexes report, the app tells Gemini, Gemini speaks.

The dodge and survivor nodes cannot talk to Gemini; only the web app holds the Live session.
This adds two routes to the lab's robot server (run.py installs them; nothing in
gemini_pupper/robot changes):

    POST /mission/event   {"event": "arrived", "details": {"range": 0.7}}
                          from survivor-id/robot/event_bridge.py, on the robot itself
    WS   /ws/mission      each event as one text line, "MISSION EVENT: arrived range=0.7",
                          to every open copy of the app, which passes it to the Live session
    GET  /mission/recent  the last few lines, to see what was sent

Only lines built here reach Gemini, from names and numbers that fit a strict pattern: a
mission event is an instruction to Gemini's voice, so nothing free-form goes through. The
server listens on 127.0.0.1 only, so the sender is a program on the robot.
"""
import re
import time
from collections import deque

NAME = re.compile(r"^[a-z][a-z_]{0,31}$")
KEY = re.compile(r"^[a-z][a-z_]{0,23}$")
VALUE = re.compile(r"^[A-Za-z0-9_.+-]{1,24}$")
MAX_DETAILS = 6


def line(event: str, details: dict | None = None) -> str:
    """'MISSION EVENT: <name> [key=value ...]', or ValueError if anything is not plain."""
    event = str(event).strip()
    if not NAME.match(event):
        raise ValueError("event name must be lower-case letters and underscores")
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
    def __init__(self, keep: int = 20):
        self.clients: set = set()
        self.recent: deque = deque(maxlen=keep)

    async def send(self, text: str) -> int:
        """To every connected app. Returns how many got it."""
        self.recent.append({"t": round(time.time(), 2), "line": text})
        sent = 0
        for ws in list(self.clients):
            try:
                await ws.send_text(text)
                sent += 1
            except Exception:
                self.clients.discard(ws)
        return sent


def install(app, feed: MissionFeed | None = None) -> MissionFeed:
    """Add the routes to a FastAPI app, ahead of its catch-all static mount."""
    # Imported here so line() works on a desk without FastAPI. (No "from __future__ import
    # annotations" in this file: FastAPI must see the real WebSocket class on ws_mission.)
    from fastapi import Body, HTTPException, WebSocket, WebSocketDisconnect
    from starlette.routing import Mount

    feed = feed or MissionFeed()

    async def post_event(body: dict = Body(...)):
        try:
            text = line(body.get("event", ""), body.get("details") or {})
        except (ValueError, AttributeError) as exc:
            raise HTTPException(422, str(exc))
        listeners = await feed.send(text)
        return {"success": True, "message": text, "listeners": listeners}

    async def recent():
        return {"listeners": len(feed.clients), "recent": list(feed.recent)}

    async def ws_mission(ws: WebSocket):
        await ws.accept()
        feed.clients.add(ws)
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
