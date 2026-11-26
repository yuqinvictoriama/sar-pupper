"""Turn one saved throw into numbers and videos.

A throw is a recording folder the dodge node wrote (see dodge/recording.py):
the camera frames, the IMU and a live trace of what the node detected and did.

    analyse(path)   -> the timeline of the throw: when the ball was first seen, when the node
                       decided, when the command went out, when the body moved, how late each
                       camera's pictures were, what move was planned. Saved as throw.json.
    render_videos() -> per camera, the plain video and one with the detections drawn on.
    write_summary() -> throws.csv, one line per throw.

All times in throw.json are seconds relative to `t_ref`: the moment of the
decision for a dodge, the moment the ball was last tracked for a throw that was
seen but not dodged, the moment of the button press for a manual mark.
"""
from __future__ import annotations

import csv
import json
import math
import os
import shutil
import subprocess

import numpy as np

from .config import load_config
from .recording import RecordingReader

ANALYSIS_VERSION = 3
MIN_TURN_RATE = 0.25       # rad/s: the body must turn at least this fast to count as reacting
GAP_S = 0.08               # the ball unseen for longer than this is a gap worth reporting
KINDS = {"dodge": "dodged (armed)", "dry": "would dodge (dry run)", "seen": "seen, no dodge", "mark": "marked by hand"}


def reaction_delay(t, gyro, t_cmd):
    """Seconds from the command to the body starting to turn, or nan if that cannot be told apart
    from how much it was already moving. Also returns the turn rate that counted as reacting."""
    t = np.asarray(t, dtype=float)
    if len(t) == 0:
        return float("nan"), MIN_TURN_RATE
    gyro = np.asarray(gyro, dtype=float).reshape(len(t), 3)
    before = (t > t_cmd - 1.0) & (t < t_cmd - 0.05)
    if before.sum() < 5:
        return float("nan"), MIN_TURN_RATE
    rate = np.linalg.norm(gyro - gyro[before].mean(axis=0), axis=1)
    thr = max(MIN_TURN_RATE, float(rate[before].mean() + 6.0 * rate[before].std()))
    for k in np.nonzero(t >= t_cmd)[0]:
        nxt = rate[k:k + 3]
        if rate[k] > thr and (nxt > thr).sum() >= min(2, len(nxt)):      # not a single stray sample
            return float(t[k] - t_cmd), thr
    return float("nan"), thr


def quat_to_rot(w, x, y, z):
    n = math.sqrt(w * w + x * x + y * y + z * z) or 1.0
    w, x, y, z = w / n, x / n, y / n, z / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class _Orientation:
    """Body-to-world rotation at any time, from the recorded IMU."""

    def __init__(self, imu):
        self.t = np.array([s[0] for s in imu]) if imu else np.zeros(0)
        self.q = [s[1] for s in imu]

    def at(self, t: float) -> np.ndarray:
        if not len(self.t):
            return np.eye(3)
        i = int(np.clip(np.searchsorted(self.t, t), 0, len(self.t) - 1))
        if i > 0 and abs(self.t[i - 1] - t) < abs(self.t[i] - t):
            i -= 1
        return quat_to_rot(*self.q[i])


def rebuild_trace(rec: RecordingReader, cfg: dict, latency: float = 0.03):
    """For a recording without a live trace (made by tools/record.py or an older node): run the
    detector and the dodge core over the frames now and return what the trace would have held.
    Processing delay is not known for such a recording, so `latency` is assumed."""
    import cv2

    from .core import DodgeCore
    from .detect import BallDetector
    core = DodgeCore(cfg, armed=False)
    dets = {name: BallDetector.for_camera(cam, cfg["detector"]) for name, cam in cfg["cameras"].items()}
    imu, i = rec.imu(), 0
    trace, events = [], []
    for t_raw, cam, jpeg in rec.frames():
        if cam not in dets:
            continue
        t = t_raw + float(cfg["cameras"][cam].get("time_offset_s", 0.0))
        now = t + latency
        while i < len(imu) and imu[i][0] <= now:
            core.set_imu(imu[i][0], imu[i][1], imu[i][2])
            i += 1
        bgr = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            continue
        blobs = dets[cam].detect(bgr)
        core.process_blobs(cam, t, blobs, now, (bgr.shape[1], bgr.shape[0]))
        core.command(now)
        r = {"k": "f", "cam": cam, "t": round(t, 4), "raw": round(t_raw, 4), "done": round(now, 4), "ms": 0.0,
             "size": [bgr.shape[1], bgr.shape[0]]}
        if blobs:
            r["blobs"] = [[round(b.u, 1), round(b.v, 1), b.w, b.h] for b in blobs]
        state = core.trace_state()
        if state is not None:
            r["core"] = state
        trace.append(r)
        for e in core.pop_events():
            events.append(e)
            trace.append(dict(e, k="e"))
    return trace, events


def _kind_of(path: str, meta: dict, event: dict | None) -> str:
    kind = meta.get("kind")
    if kind in KINDS:
        return kind
    if event is not None and event.get("event") == "dodge":
        return "dry" if _truthy(event.get("dry_run")) else "dodge"
    if event is not None and event.get("event") == "throw_seen":
        return "seen"
    return "mark"


def _pick_track(frames, event):
    """The track the throw is about: the one that caused the dodge, else the ball in flight seen most."""
    if event is not None and event.get("event") == "dodge" and (event.get("threat") or {}).get("track") is not None:
        return event["threat"]["track"]
    best = {}
    for r in frames:
        for tr in (r.get("core") or {}).get("tracks", []):
            if tr.get("confirmed"):
                score = (1 if tr.get("flying") else 0, tr["hits"])
                best[tr["id"]] = max(best.get(tr["id"], (0, 0)), score)
    return max(best, key=lambda k: best[k]) if best else None


def analyse(path: str, cfg: dict | None = None, save: bool = True) -> dict:
    """Work out the timeline of the throw in folder `path`. Returns the dictionary saved as throw.json."""
    rec = RecordingReader(path)
    meta = rec.meta or {}
    cfg = cfg or meta.get("config") or load_config()
    events = rec.events()
    trace = rec.trace()
    imu = rec.imu()
    rebuilt = False
    if not any(r.get("k") == "f" for r in trace):
        trace, replayed = rebuild_trace(rec, cfg)
        events = events or replayed
        rebuilt = True
    frames = [r for r in trace if r.get("k") == "f"]
    cmds = [r for r in trace if r.get("k") == "c"]
    mux = [r for r in trace if r.get("k") == "m"]
    event = meta.get("event") or next((e for e in events if e.get("event") == "dodge"), None) or (events[0] if events else None)
    kind = _kind_of(path, meta, event)
    tid = _pick_track(frames, event)
    orient = _Orientation(imu)

    # ---- the ball's track through the recording
    series = []                      # (done, frame record, track dict)
    for r in frames:
        for tr in (r.get("core") or {}).get("tracks", []):
            if tr["id"] == tid:
                series.append((r["done"], r, tr))
    sightings = [(r["t"], r["cam"], r) for _d, r, tr in series if abs(tr.get("seen", -1) - r["t"]) < 2e-3 and r.get("blobs")]
    t_abs = {}                       # absolute times of each milestone
    if series:
        t_abs["first_seen"] = series[0][2]["t0"]
        t_abs["first_known"] = series[0][0]
        conf = next((d for d, _r, tr in series if tr.get("confirmed")), None)
        if conf is not None:
            t_abs["tracked"] = conf
    for r in frames:
        if any(th.get("track") == tid and th.get("threat") for th in (r.get("core") or {}).get("threats", [])):
            t_abs["threat"] = r["done"]
            break
    threat = (event or {}).get("threat") or {}
    plan = (event or {}).get("plan") or {}
    # What the node made of the ball, picture by picture, from when it was tracked until it decided (or lost it).
    waited = {}
    until = float(event["t"]) if event is not None and event.get("event") == "dodge" else float("inf")
    for d, r, tr in series:
        if not tr.get("confirmed") or d >= until:
            continue
        core = r.get("core") or {}
        mine = next((th for th in core.get("threats", []) if th.get("track") == tid), None)
        why = core.get("hold") or (mine or {}).get("reason") or ""
        if why:
            waited[why] = waited.get(why, 0) + 1
    dodged = kind in ("dodge", "dry")
    if dodged:
        t_ref = float(event["t"])
        t_abs["decision"] = t_ref
        if threat.get("tau") is not None:
            t_abs["contact"] = t_ref + float(threat["tau"])
    elif kind == "seen" and series:
        t_ref = max(tr.get("seen", d) for d, _r, tr in series)
    else:
        t_ref = float(event["t"]) if event is not None else (frames[-1]["done"] if frames else 0.0)

    armed = kind == "dodge"
    if dodged and armed:
        first_cmd = next((c for c in cmds if c["t"] >= t_ref - 0.005 and (abs(c["vx"]) + abs(c["vy"])) > 1e-3), None)
        if first_cmd is not None:
            t_abs["command"] = first_cmd["t"]
            through = next((m for m in mux if m["t"] >= first_cmd["t"] - 0.002 and
                            abs(m["vx"] - first_cmd["vx"]) < 0.05 and abs(m["vy"] - first_cmd["vy"]) < 0.05), None)
            if through is not None:
                t_abs["mux"] = through["t"]
            if imu:
                delay, _thr = reaction_delay([s[0] for s in imu], [s[2] for s in imu], first_cmd["t"])
                if not math.isnan(delay) and delay < 1.0:
                    t_abs["body"] = first_cmd["t"] + delay

    # ---- where the ball was, in the robot's own frame (x forward, y left, z up)
    def in_body(tr):
        p = orient.at(tr["t"]).T @ np.array(tr["p"], dtype=float)
        return [round(float(v), 3) for v in p]

    path_pts = []
    step = max(1, len(series) // 90)
    for d, _r, tr in series[::step]:
        path_pts.append([round(tr["t"] - t_ref, 3)] + in_body(tr))

    def where(tr):
        p = in_body(tr)
        rng = math.sqrt(sum(v * v for v in p))
        return {"range_m": round(rng, 2), "bearing_deg": round(math.degrees(math.atan2(p[1], p[0])), 0),
                "elevation_deg": round(math.degrees(math.atan2(p[2], math.hypot(p[0], p[1]))), 0)}

    ball = {}
    if series:
        ball["first"] = where(series[0][2])
        if dodged:
            at = min(series, key=lambda s: abs(s[0] - t_ref))[2]
            ball["at_decision"] = where(at)
        ball["speed_m_s"] = round(max(math.sqrt(sum(c * c for c in tr["v"])) for _d, _r, tr in series), 1)
        ball["flying"] = any(tr.get("flying") for _d, _r, tr in series)

    # ---- times the ball was out of sight while it was being tracked
    gaps = []
    if sightings:
        end = min(t_abs.get("contact", float("inf")), max(tr.get("t", 0.0) for _d, _r, tr in series))
        in_flight = [s for s in sightings if s[0] <= end]
        times = [s[0] for s in in_flight] + [end]
        for i in range(len(in_flight)):
            dt = times[i + 1] - times[i]
            if dt > GAP_S:
                _t, cam, r = in_flight[i]
                w, h = r.get("size", [0, 0])
                b = _ball_blob(r, None)
                edge = "the picture"
                if b is not None and h:
                    u, vv = b[0], b[1]
                    edge = ("the top of the picture" if vv < 0.12 * h else "the bottom of the picture" if vv > 0.88 * h
                            else "the side of the picture" if (u < 0.08 * w or u > 0.92 * w) else "the middle of the picture")
                gaps.append({"start": round(times[i] - t_ref, 3), "end": round(times[i + 1] - t_ref, 3),
                             "duration": round(dt, 3), "camera": cam, "where": edge,
                             "to_the_end": i == len(in_flight) - 1})

    # ---- per camera: how fast and how fresh its pictures were
    cams = {}
    for name in sorted({r["cam"] for r in frames}):
        fr = [r for r in frames if r["cam"] == name]
        span = fr[-1]["t"] - fr[0]["t"] if len(fr) > 1 else 0.0
        cams[name] = {
            "frames": len(fr),
            "fps": round((len(fr) - 1) / span, 1) if span > 0 else 0.0,
            "age_ms": round(1000.0 * float(np.mean([r["done"] - r["t"] for r in fr])), 0),
            "process_ms": round(float(np.mean([r.get("ms", 0.0) for r in fr])), 1),
            "frames_with_ball": sum(1 for s in sightings if s[1] == name),
            "frames_with_anything": sum(1 for r in fr if r.get("blobs")),
            "first_ball": round(min((s[0] for s in sightings if s[1] == name), default=float("nan")) - t_ref, 3),
        }
        if math.isnan(cams[name]["first_ball"]):
            cams[name]["first_ball"] = None

    times = {k: round(v - t_ref, 3) for k, v in t_abs.items()}

    def span_ms(a, b):
        return round(1000.0 * (t_abs[b] - t_abs[a])) if a in t_abs and b in t_abs else None

    numbers = {
        "seen_to_decision_ms": span_ms("first_seen", "decision"),      # ball first in a picture -> decided to dodge
        "first_picture_delay_ms": span_ms("first_seen", "first_known"),  # that picture reaching the tracker
        "tracking_ms": span_ms("first_known", "tracked"),               # until enough pictures agreed it is a ball
        "decision_to_command_ms": span_ms("decision", "command"),
        "command_to_mux_ms": span_ms("command", "mux"),
        "command_to_body_ms": span_ms("command", "body"),              # the robot's own latency
        "decision_to_body_ms": span_ms("decision", "body"),            # recognised -> starts moving
        "seen_to_body_ms": span_ms("first_seen", "body"),              # the whole reaction
        "time_left_at_decision_ms": span_ms("decision", "contact"),
        "time_left_when_moving_ms": span_ms("body", "contact"),
        "needed_ms": round(1000.0 * float(plan["t_need"])) if plan.get("t_need") is not None else None,
    }

    out = {
        "version": ANALYSIS_VERSION, "id": os.path.basename(os.path.normpath(path)), "kind": kind,
        "kind_text": KINDS[kind], "t_ref": round(t_ref, 4), "rebuilt": rebuilt,
        "note": (event or {}).get("note", ""),
        "late": _truthy((event or {}).get("late", False)), "times": times, "numbers": numbers,
        "plan": plan, "threat": threat, "ball": ball, "path": path_pts, "gaps": gaps, "cameras": cams,
        "verdicts": (event or {}).get("verdicts", {}), "waited": waited, "track": tid,
        "window": [round(frames[0]["t"] - t_ref, 3), round(frames[-1]["t"] - t_ref, 3)] if frames else [0.0, 0.0],
        "box": (cfg.get("robot") or {}).get("box"),
    }
    left, needed = numbers["time_left_at_decision_ms"], numbers["needed_ms"]
    # "late" from the node means it fired because time was up. Whether that mattered depends on how much was missing.
    out["timing"] = ("ok" if not out["late"] or left is None or needed is None else
                     "tight" if left >= 0.8 * needed else "late")
    out["story"] = story(out)
    rec.close()
    if save:
        with open(os.path.join(path, "throw.json"), "w") as f:
            json.dump(out, f, indent=1)
    return out


def _truthy(v) -> bool:
    """Older recordings hold the node's true/false values as the words "True"/"False"."""
    return v is True or (isinstance(v, str) and v.strip().lower() == "true") or (isinstance(v, (int, float)) and v == 1)


def _ball_blob(r: dict, uv):
    """The detection in frame record r that is the ball: the one nearest pixel uv, or the biggest."""
    blobs = r.get("blobs") or []
    if not blobs:
        return None
    if uv is None:
        return max(blobs, key=lambda b: b[2] * b[3])
    return min(blobs, key=lambda b: (b[0] - uv[0]) ** 2 + (b[1] - uv[1]) ** 2)


def describe_move(vx: float, vy: float) -> str:
    fore = "forward" if vx > 0 else "backward" if vx < 0 else ""
    side = "left" if vy > 0 else "right" if vy < 0 else ""
    if fore and side:
        return "run %s-%s" % (fore, side)
    return ("run %s" % fore) if fore else ("sidestep %s" % side if side else "stay")


def story(t: dict) -> list:
    """The throw in a few plain sentences, most important first."""
    n, out = t["numbers"], []
    first = (t.get("ball") or {}).get("first")
    if t["track"] is None:
        out.append("No ball was tracked in this recording: the cameras did not pick one out. Play the video with "
                   "detections on to see whether the ball was in the picture at all.")
        return out
    if t["kind"] in ("dodge", "dry"):
        plan = t["plan"]
        move = describe_move(plan.get("vx", 0.0), plan.get("vy", 0.0))
        s = "Decided to %s (vx %+.1f, vy %+.1f m/s for %.2f s)" % (move, plan.get("vx", 0.0), plan.get("vy", 0.0), plan.get("duration", 0.0))
        if n["seen_to_decision_ms"] is not None:
            s += " %d ms after the ball first appeared in a picture" % n["seen_to_decision_ms"]
        if n["time_left_at_decision_ms"] is not None:
            s += ", with %d ms left before contact" % n["time_left_at_decision_ms"]
        out.append(s + ".")
        if t.get("timing") == "late":
            out.append("Too late: this move needs about %d ms and only %d ms were left." % (n["needed_ms"], n["time_left_at_decision_ms"]))
        elif t.get("timing") == "tight":
            out.append("Tight: it waited until the last moment it could (the move needs about %d ms, %d ms were left)." % (
                n["needed_ms"], n["time_left_at_decision_ms"]))
        waited = sorted((t.get("waited") or {}).items(), key=lambda kv: -kv[1])
        if waited and (n["seen_to_decision_ms"] or 0) >= 150:
            out.append("While it was tracking and had not decided, its verdict was: %s." % ", ".join(
                "%s (%d pictures)" % kv for kv in waited[:3]))
        if t["kind"] == "dry":
            out.append("Dry run: no command was sent, so there is no robot latency to measure.")
        elif n["decision_to_body_ms"] is not None:
            s = "The body started to move %d ms after the decision" % n["decision_to_body_ms"]
            if n["command_to_mux_ms"] is not None:
                s += " (command sent after %d ms, through the mux %d ms later)" % (n["decision_to_command_ms"] or 0, n["command_to_mux_ms"])
            out.append(s + ".")
            if n["time_left_when_moving_ms"] is not None:
                out.append("That left %d ms of movement before the ball was due." % n["time_left_when_moving_ms"])
        elif n["decision_to_command_ms"] is not None:
            out.append("The command went out %d ms after the decision; the IMU does not show clearly when the body moved." % n["decision_to_command_ms"])
    elif t["kind"] == "seen":
        v = t.get("verdicts") or {}
        why = ", ".join("%s (%d pictures)" % (k, c) for k, c in sorted(v.items(), key=lambda kv: -kv[1])) or "no verdict recorded"
        out.append("A ball in flight was tracked but no dodge was made. What the node made of it: %s." % why)
    else:
        out.append("Saved by hand. %s" % ("A ball was tracked." if t["track"] is not None else ""))
    if first:
        out.append("First seen %.1f m away at %+.0f deg (0 = ahead, +90 = left), %+.0f deg above level." % (
            first["range_m"], first["bearing_deg"], first["elevation_deg"]))
    for g in t["gaps"]:
        if g["duration"] >= 0.12:
            out.append("Out of sight for %d ms: last seen by %s, in %s%s." % (
                round(g["duration"] * 1000), g["camera"], g["where"],
                " and not seen again before it arrived" if g["to_the_end"] else ""))
    slow = [c for c, s in t["cameras"].items() if s["age_ms"] > 80 and s["frames_with_ball"]]
    if slow:
        out.append("Pictures from %s were on average more than 80 ms old when processed." % ", ".join(slow))
    return out


# ------------------------------------------------------------------ videos
class _Writer:
    """An .mp4 file: H.264 through ffmpeg where it is installed (plays anywhere), else OpenCV's own."""

    _h264 = None        # does this machine's ffmpeg have an H.264 encoder? (found out once)

    @classmethod
    def has_h264(cls) -> bool:
        if cls._h264 is None:
            cls._h264 = False
            if shutil.which("ffmpeg"):
                try:
                    listing = subprocess.run(["ffmpeg", "-hide_banner", "-encoders"], capture_output=True, text=True, timeout=10)
                    cls._h264 = "libx264" in listing.stdout
                except (OSError, subprocess.SubprocessError):
                    pass
        return cls._h264

    def __init__(self, path: str, size, fps: float):
        self.path, self.size, self.proc, self.cv = path, size, None, None
        fps = self.fps = max(1.0, min(240.0, float(fps)))
        if self.has_h264():
            try:
                self.proc = subprocess.Popen(
                    ["ffmpeg", "-loglevel", "error", "-y", "-f", "rawvideo", "-pix_fmt", "bgr24", "-s", "%dx%d" % size,
                     "-r", "%.3f" % fps, "-i", "-", "-an", "-c:v", "libx264", "-preset", "veryfast", "-crf", "24",
                     "-pix_fmt", "yuv420p", "-movflags", "+faststart", path],
                    stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            except OSError:
                self.proc = None
        if self.proc is None:
            self._open_cv(fps)

    def _open_cv(self, fps: float) -> None:
        import cv2
        self.cv = cv2.VideoWriter(self.path, cv2.VideoWriter_fourcc(*"mp4v"), fps, self.size)

    def write(self, bgr) -> None:
        if self.proc is not None:
            try:
                self.proc.stdin.write(bgr.tobytes())
                return
            except (BrokenPipeError, OSError):          # ffmpeg without H.264: start again with OpenCV
                self.proc = None
                self._open_cv(self.fps)
        if self.cv is not None:
            self.cv.write(bgr)

    def close(self) -> bool:
        if self.proc is not None:
            try:
                self.proc.stdin.close()
            except OSError:
                pass
            ok = self.proc.wait() == 0
        else:
            ok = self.cv is not None and self.cv.isOpened()
            if self.cv is not None:
                self.cv.release()
        return ok and os.path.exists(self.path) and os.path.getsize(self.path) > 1000


def draw_overlay(bgr, r: dict | None, t_rel: float, cam: str, throw: dict):
    """What the node saw in this frame, drawn on it. r is the frame's trace record (None if it was not processed)."""
    import cv2
    out = bgr
    scale = max(1.0, bgr.shape[1] / 960.0)
    thick = max(1, int(round(scale)))
    state = ((r or {}).get("core") or {}).get("state", "SEARCH") if r is not None else "not processed"
    ball = None
    if r is not None:
        tr = next((k for k in (r.get("core") or {}).get("tracks", []) if k["id"] == throw.get("track")), None)
        if tr is not None and abs(tr.get("seen", -1) - r["t"]) < 2e-3:
            ball = _ball_blob(r, None)
        for b in r.get("blobs") or []:
            x0, y0 = int(b[0] - b[2] / 2) - 3, int(b[1] - b[3] / 2) - 3
            x1, y1 = int(b[0] + b[2] / 2) + 3, int(b[1] + b[3] / 2) + 3
            if b is ball:
                cv2.circle(out, (int(b[0]), int(b[1])), int(max(b[2], b[3]) / 2 + 6 * scale), (0, 255, 0), 2 * thick, cv2.LINE_AA)
            else:
                cv2.rectangle(out, (x0, y0), (x1, y1), (0, 220, 255), thick)
    colour = {"DODGE": (0, 0, 255), "THREAT": (0, 140, 255), "TRACK": (0, 255, 0), "RECOVER": (255, 160, 0)}.get(state, (255, 255, 255))
    text = "%s  %+.3f s  %s" % (cam, t_rel, state)
    cv2.putText(out, text, (int(12 * scale), int(30 * scale)), cv2.FONT_HERSHEY_SIMPLEX, 0.8 * scale, (0, 0, 0), 4 * thick, cv2.LINE_AA)
    cv2.putText(out, text, (int(12 * scale), int(30 * scale)), cv2.FONT_HERSHEY_SIMPLEX, 0.8 * scale, colour, 2 * thick, cv2.LINE_AA)
    if ball is not None:
        cv2.putText(out, "ball", (int(ball[0] + ball[2]), int(ball[1])), cv2.FONT_HERSHEY_SIMPLEX, 0.6 * scale, (0, 255, 0), thick, cv2.LINE_AA)
    return out


def render_videos(path: str, throw: dict | None = None, max_width: int = 1280) -> dict:
    """Write videos/<camera>.mp4 (plain) and videos/<camera>_detections.mp4 for the throw in `path`.
    Returns {file name: True/False}."""
    import cv2
    throw = throw or analyse(path)
    rec = RecordingReader(path)
    by_raw = {}
    for r in rec.trace():
        if r.get("k") == "f":
            by_raw[(r["cam"], round(r["raw"], 3))] = r
    out_dir = os.path.join(path, "videos")
    os.makedirs(out_dir, exist_ok=True)
    result = {}
    offsets = {}
    for cam, rows in rec.index.items():
        if len(rows) < 2:
            continue
        fps = (len(rows) - 1) / max(rows[-1][0] - rows[0][0], 1e-3)
        plain = seen = None
        for i, (t_raw, _off, _ln) in enumerate(rows):
            bgr = cv2.imdecode(np.frombuffer(rec.jpeg(cam, i), dtype=np.uint8), cv2.IMREAD_COLOR)
            if bgr is None:
                continue
            if bgr.shape[1] > max_width:
                bgr = cv2.resize(bgr, (max_width, int(round(bgr.shape[0] * max_width / bgr.shape[1]))), interpolation=cv2.INTER_AREA)
            if bgr.shape[1] % 2 or bgr.shape[0] % 2:                       # H.264 needs even sizes
                bgr = bgr[:bgr.shape[0] // 2 * 2, :bgr.shape[1] // 2 * 2]
            if plain is None:
                size = (bgr.shape[1], bgr.shape[0])
                plain = _Writer(os.path.join(out_dir, cam + ".mp4"), size, fps)
                seen = _Writer(os.path.join(out_dir, cam + "_detections.mp4"), size, fps)
            plain.write(bgr)
            r = by_raw.get((cam, round(t_raw, 3)))
            if r is not None:
                offsets[cam] = r["t"] - r["raw"]
            t_rel = t_raw + offsets.get(cam, 0.0) - throw["t_ref"]
            seen.write(draw_overlay(bgr.copy(), _scaled(r, bgr.shape[1]), t_rel, cam, throw))
        if plain is not None:
            result[cam + ".mp4"] = plain.close()
            result[cam + "_detections.mp4"] = seen.close()
    rec.close()
    return result


def _scaled(r: dict | None, width: int):
    """The frame record with its detections scaled to a video that is `width` pixels wide."""
    if r is None or not r.get("blobs") or not r.get("size") or r["size"][0] == width:
        return r
    k = width / float(r["size"][0])
    return dict(r, blobs=[[b[0] * k, b[1] * k, b[2] * k, b[3] * k] for b in r["blobs"]])


# ------------------------------------------------------------------ the folder of throws
def is_throw(path: str) -> bool:
    return os.path.isdir(path) and os.path.exists(os.path.join(path, "meta.json")) and any(
        n.endswith(".idx") for n in os.listdir(path))


def list_throws(dirs) -> list:
    """Folders of saved throws under the given directories, newest first."""
    out = []
    for d in dirs:
        if os.path.isdir(d):
            out += [os.path.join(d, n) for n in os.listdir(d) if is_throw(os.path.join(d, n))]
    return sorted(out, key=lambda p: os.path.basename(p), reverse=True)


def load_throw(path: str, refresh: bool = False) -> dict:
    """throw.json for a folder, analysing it first if that has not been done (or was done by an older version)."""
    p = os.path.join(path, "throw.json")
    if not refresh and os.path.exists(p):
        try:
            with open(p) as f:
                t = json.load(f)
            if t.get("version") == ANALYSIS_VERSION:
                return t
        except (OSError, ValueError):
            pass
    return analyse(path)


def load_label(path: str) -> dict:
    try:
        with open(os.path.join(path, "label.json")) as f:
            return json.load(f)
    except (OSError, ValueError):
        return {"result": "", "note": ""}


def save_label(path: str, result: str, note: str) -> dict:
    label = {"result": str(result)[:20], "note": str(note)[:300]}
    with open(os.path.join(path, "label.json"), "w") as f:
        json.dump(label, f)
    return label


CSV_COLUMNS = ["id", "kind", "result", "note", "timing", "from_deg", "first_seen_range_m", "seen_to_decision_ms",
               "decision_to_command_ms", "command_to_mux_ms", "command_to_body_ms", "decision_to_body_ms",
               "seen_to_body_ms", "time_left_at_decision_ms", "time_left_when_moving_ms", "needed_ms",
               "vx", "vy", "duration_s", "move", "longest_gap_ms", "cameras"]


def summary_row(t: dict, label: dict) -> dict:
    n, plan, first = t["numbers"], t.get("plan") or {}, (t.get("ball") or {}).get("first") or {}
    return {
        "id": t["id"], "kind": t["kind_text"], "result": label.get("result", ""), "note": label.get("note", ""),
        "timing": t.get("timing", ""),
        "from_deg": (t.get("threat") or {}).get("bearing_deg", "") if t["kind"] in ("dodge", "dry") else first.get("bearing_deg", ""),
        "first_seen_range_m": first.get("range_m", ""),
        **{k: ("" if n.get(k) is None else n[k]) for k in CSV_COLUMNS if k.endswith("_ms") and k in n},
        "vx": plan.get("vx", ""), "vy": plan.get("vy", ""), "duration_s": plan.get("duration", ""),
        "move": describe_move(plan.get("vx", 0.0), plan.get("vy", 0.0)) if plan else "",
        "longest_gap_ms": round(1000 * max((g["duration"] for g in t.get("gaps", [])), default=0.0)),
        "cameras": " ".join("%s:%.0ffps/%.0fms" % (c, s["fps"], s["age_ms"]) for c, s in t.get("cameras", {}).items()),
    }


def write_summary(folder: str, dirs=None) -> str:
    """throws.csv in `folder`: one line per saved throw under dirs (default: folder itself)."""
    rows = [summary_row(load_throw(p), load_label(p)) for p in list_throws(dirs or [folder])]
    os.makedirs(folder, exist_ok=True)
    out = os.path.join(folder, "throws.csv")
    with open(out, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=CSV_COLUMNS, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)
    return out
