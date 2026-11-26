#!/usr/bin/env python3
"""Pupper Dodging View: a page in your browser for every throw the dodge node saved.

    bash robot/dodging_view.sh                 # on the Pupper; then open the address it prints
    python3 tools/dodging_view.py --port 8077  # the same, by hand
    python3 tools/dodging_view.py --no-ros     # only look at saved throws (works on a laptop copy too)

The dodge node saves every dodge, every throw it saw but did not dodge, and
every manual mark to data/throws/. This tool turns each one into numbers
(throw.json), keeps data/throws/throws.csv up to date, makes videos
(videos/*.mp4) when asked, and serves a page that lists the throws, plays each camera with what
the node detected drawn on top, and shows the timeline: first sight of the
ball, decision, command, the body starting to move, expected contact.

It never moves the robot and cannot arm it. Its one action on the robot is
the pair of save buttons, which ask the node to keep what its cameras saw in
the last few seconds or will see in the next few (for throws the robot does
not react to at all, which it would otherwise not save).
"""
from __future__ import annotations

import argparse
import io
import json
import os
import re
import shutil
import socket
import sys
import threading
import time
import zipfile
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import unquote, urlparse

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

from dodge import throwlog  # noqa: E402
from dodge.config import load_config  # noqa: E402
from dodge.recording import RecordingReader  # noqa: E402

PAGE = os.path.join(HERE, "dodging_view.html")
ID_RE = re.compile(r"^[A-Za-z0-9_.-]+$")
RESULTS = ("", "dodged", "hit", "off target", "not a throw")


class Lab:
    """The saved throws on disk, and the background work of analysing them."""

    def __init__(self, dirs, make_videos: bool = False, window_s: float = 3.2):
        self.window_s = window_s         # how many seconds the node keeps, and so saves, per throw
        self.dirs = [d for d in dirs if d]
        self.home = self.dirs[0]
        os.makedirs(self.home, exist_ok=True)
        self.make_videos = make_videos
        self.lock = threading.Lock()
        self.cache: dict = {}            # id -> (folder mtime, throw dict)
        self.frames: dict = {}           # id -> frame index for the player
        self.busy = ""
        self.errors: dict = {}
        self.stop = False
        self.wanted: set = set()         # throws whose videos were asked for from the page
        self._disk = (0.0, None)

    # ---- finding throws
    def paths(self) -> dict:
        return {os.path.basename(p): p for p in throwlog.list_throws(self.dirs)}

    def path(self, tid: str):
        if not ID_RE.match(tid or ""):
            return None
        return self.paths().get(tid)

    def throw(self, tid: str, path: str) -> dict | None:
        """The analysed throw, or None while its analysis is still pending."""
        p = os.path.join(path, "throw.json")
        if not os.path.exists(p):
            return None
        mt = os.path.getmtime(p)
        with self.lock:
            hit = self.cache.get(tid)
            if hit and hit[0] == mt:
                return hit[1]
        try:
            with open(p) as f:
                t = json.load(f)
        except (OSError, ValueError):
            return None
        if t.get("version") != throwlog.ANALYSIS_VERSION:
            return None
        with self.lock:
            self.cache[tid] = (mt, t)
        return t

    def videos(self, path: str) -> list:
        d = os.path.join(path, "videos")
        return sorted(n for n in os.listdir(d) if n.endswith(".mp4")) if os.path.isdir(d) else []

    def listing(self) -> list:
        out = []
        for tid, path in self.paths().items():
            t = self.throw(tid, path)
            row = {"id": tid, "ready": t is not None, "label": throwlog.load_label(path),
                   "videos": len(self.videos(path)), "making": tid in self.wanted, "error": self.errors.get(tid, "")}
            if t is not None:
                row.update({k: t[k] for k in ("kind", "kind_text", "late", "timing", "numbers", "plan", "rebuilt", "note")})
                row["first"] = (t.get("ball") or {}).get("first")
                dodged = t["kind"] in ("dodge", "dry")
                row["from_deg"] = (t.get("threat") or {}).get("bearing_deg") if dodged else (row["first"] or {}).get("bearing_deg")
                row["tracked"] = t.get("track") is not None
                row["move"] = throwlog.describe_move(t["plan"].get("vx", 0.0), t["plan"].get("vy", 0.0)) if t.get("plan") else ""
                row["gap_ms"] = round(1000 * max((g["duration"] for g in t.get("gaps", [])), default=0.0))
            out.append(row)
        return out

    def disk(self) -> dict:
        if self._disk[1] is not None and time.time() - self._disk[0] < 15.0:
            return self._disk[1]
        used = 0
        for path in self.paths().values():
            for base, _dirs, names in os.walk(path):
                used += sum(os.path.getsize(os.path.join(base, n)) for n in names)
        self._disk = (time.time(), {"free_gb": round(shutil.disk_usage(self.home).free / 1e9, 1),
                                    "throws_gb": round(used / 1e9, 2)})
        return self._disk[1]

    # ---- one throw, for the player
    def frame_index(self, tid: str, path: str, t: dict) -> dict:
        with self.lock:
            if tid in self.frames:
                return self.frames[tid]
        rec = RecordingReader(path)
        by_raw = {(r["cam"], round(r["raw"], 3)): r for r in rec.trace() if r.get("k") == "f"}
        if not by_raw:                                   # no live trace: what a replay would have detected
            trace, _events = throwlog.rebuild_trace(rec, rec.meta.get("config") or load_config())
            by_raw = {(r["cam"], round(r["raw"], 3)): r for r in trace if r.get("k") == "f"}
        out = {}
        for cam, rows in rec.index.items():
            ts, blobs, ball, state, size, offset = [], [], [], [], None, 0.0
            for t_raw, _off, _ln in rows:
                r = by_raw.get((cam, round(t_raw, 3)))
                if r is not None:
                    offset = r["t"] - r["raw"]
                    size = size or r.get("size")
                ts.append(round(t_raw + offset - t["t_ref"], 4))
                bl = (r or {}).get("blobs") or []
                blobs.append(bl)
                core = (r or {}).get("core") or {}
                state.append(core.get("state", "SEARCH") if r is not None else "")
                tr = next((k for k in core.get("tracks", []) if k["id"] == t.get("track")), None)
                seen_now = tr is not None and r is not None and abs(tr.get("seen", -1) - r["t"]) < 2e-3 and bl
                ball.append(bl.index(throwlog._ball_blob(r, None)) if seen_now else -1)
            out[cam] = {"t": ts, "blobs": blobs, "ball": ball, "state": state, "size": size}
        rec.close()
        with self.lock:
            self.frames[tid] = out
            while len(self.frames) > 6:
                self.frames.pop(next(iter(self.frames)))
        return out

    def jpeg(self, path: str, cam: str, i: int) -> bytes | None:
        rec = RecordingReader(path)
        try:
            if cam not in rec.index or not (0 <= i < len(rec.index[cam])):
                return None
            return rec.jpeg(cam, i)
        finally:
            rec.close()

    # ---- background: analyse new throws, make their videos, keep throws.csv current
    def work(self) -> None:
        try:
            os.nice(10)                  # the dodge node comes first
        except OSError:
            pass
        try:
            import cv2
            cv2.setNumThreads(1)
        except Exception:
            pass
        while not self.stop:
            changed = False
            for tid, path in list(self.paths().items()):
                if self.stop:
                    return
                if tid in self.errors:
                    continue
                try:
                    if time.time() - os.path.getmtime(os.path.join(path, "meta.json")) < 1.5:
                        continue         # the node may still be writing it
                    t = self.throw(tid, path)
                    if t is None:
                        self.busy = "working out the timings of " + tid
                        t = throwlog.analyse(path)
                        changed = True
                    if (self.make_videos or tid in self.wanted) and not self.videos(path):
                        self.busy = "making the videos of " + tid
                        throwlog.render_videos(path, t)
                        changed = True
                    self.wanted.discard(tid)
                except Exception as exc:                 # one bad recording must not stop the rest
                    self.errors[tid] = "%s: %s" % (type(exc).__name__, exc)
            self.busy = ""
            if changed or not os.path.exists(os.path.join(self.home, "throws.csv")):
                self.write_csv()
            time.sleep(1.0)

    def write_csv(self) -> None:
        try:
            throwlog.write_summary(self.home, self.dirs)
        except Exception as exc:
            self.errors["throws.csv"] = str(exc)


class Robot:
    """The live link to the dodge node: its state, and the "save the last few seconds" request. Optional."""

    def __init__(self, use_ros: bool):
        self.state, self.state_time, self.node, self.pub, self.problem = None, 0.0, None, None, ""
        if not use_ros:
            self.problem = "started with --no-ros"
            return
        try:
            import rclpy
            from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
            from rclpy.node import Node
            from std_msgs.msg import String
        except Exception as exc:
            self.problem = "ROS is not available here (%s): showing saved throws only" % type(exc).__name__
            return
        self.String = String
        rclpy.init()
        self.node = Node("dodging_view")
        self.node.create_subscription(String, "/dodge/state", self.on_state, 5)
        self.pub = self.node.create_publisher(String, "/dodge/mark", 5)
        ex = SingleThreadedExecutor()
        ex.add_node(self.node)

        def spin():
            try:
                ex.spin()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass
            except Exception:
                pass
        threading.Thread(target=spin, daemon=True, name="ros").start()

    def on_state(self, msg) -> None:
        try:
            self.state, self.state_time = json.loads(msg.data), time.time()
        except ValueError:
            pass

    def status(self) -> dict:
        live = self.state is not None and time.time() - self.state_time < 2.0
        return {"ros": self.node is not None, "live": live, "problem": self.problem,
                "state": self.state if live else None}

    def mark(self, note: str, wait: float = 0.0) -> bool:
        """Ask the node to save the last few seconds, or (wait > 0) what happens in the next `wait` seconds."""
        if self.pub is None:
            return False
        self.pub.publish(self.String(data=("wait=%.1f;" % wait if wait > 0 else "") + note[:150]))
        return True


def make_handler(lab: Lab, robot: Robot):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def log_message(self, *args):          # quiet: this runs next to the robot's own logs
            pass

        def send(self, code: int, body: bytes, ctype: str, extra: dict | None = None) -> None:
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            for k, v in (extra or {}).items():
                self.send_header(k, v)
            self.end_headers()
            if self.command != "HEAD":
                self.wfile.write(body)

        def json(self, obj, code: int = 200) -> None:
            self.send(code, json.dumps(obj, default=str).encode(), "application/json", {"Cache-Control": "no-store"})

        def missing(self) -> None:
            self.json({"error": "not found"}, 404)

        def do_GET(self) -> None:
            try:
                self.route()
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as exc:
                try:
                    self.json({"error": "%s: %s" % (type(exc).__name__, exc)}, 500)
                except Exception:
                    pass

        do_HEAD = do_GET

        def route(self) -> None:
            parts = [unquote(p) for p in urlparse(self.path).path.split("/") if p]
            if not parts:
                with open(PAGE, "rb") as f:
                    return self.send(200, f.read(), "text/html; charset=utf-8", {"Cache-Control": "no-store"})
            if parts == ["api", "throws"]:
                return self.json({"throws": lab.listing(), "status": robot.status(), "busy": lab.busy,
                                  "folder": lab.home, "results": RESULTS, "disk": lab.disk(), "window_s": lab.window_s})
            if parts == ["api", "status"]:
                return self.json({"status": robot.status(), "busy": lab.busy})
            if parts == ["csv"]:
                lab.write_csv()
                with open(os.path.join(lab.home, "throws.csv"), "rb") as f:
                    return self.send(200, f.read(), "text/csv", {"Content-Disposition": 'attachment; filename="throws.csv"'})
            if len(parts) >= 3 and parts[0] in ("api", "video", "zip") or (len(parts) == 2 and parts[0] == "zip"):
                tid = parts[2] if parts[0] == "api" else parts[1]
                path = lab.path(tid)
                if path is None:
                    return self.missing()
                if parts[:2] == ["api", "throw"] and len(parts) == 3:
                    t = lab.throw(tid, path)
                    if t is None:
                        return self.json({"error": "this throw is still being worked out; try again in a moment"}, 503)
                    if parts[-1] == tid and self.path.endswith("?files"):
                        return self.json({"videos": lab.videos(path), "making": tid in lab.wanted, "folder": path})
                    return self.json({"throw": t, "label": throwlog.load_label(path), "frames": lab.frame_index(tid, path, t),
                                      "videos": lab.videos(path), "making": tid in lab.wanted, "folder": path})
                if parts[:2] == ["api", "frame"] and len(parts) == 5:
                    m = re.match(r"^(\d+)\.jpg$", parts[4])
                    data = lab.jpeg(path, parts[3], int(m.group(1))) if m else None
                    if data is None:
                        return self.missing()
                    return self.send(200, data, "image/jpeg", {"Cache-Control": "max-age=86400"})
                if parts[0] == "video" and len(parts) == 3 and parts[2] in lab.videos(path):
                    return self.file(os.path.join(path, "videos", parts[2]), "video/mp4")
                if parts[0] == "zip":
                    return self.zip(tid, path)
            return self.missing()

        def file(self, path: str, ctype: str) -> None:
            """A file, with byte ranges so a browser can play and seek a video."""
            size = os.path.getsize(path)
            start, end = 0, size - 1
            m = re.match(r"bytes=(\d*)-(\d*)", self.headers.get("Range", ""))
            if m and (m.group(1) or m.group(2)):
                if m.group(1):
                    start = int(m.group(1))
                    end = min(int(m.group(2)), size - 1) if m.group(2) else size - 1
                else:
                    start = max(0, size - int(m.group(2)))
            if start > end or start >= size:
                return self.send(416, b"", ctype, {"Content-Range": "bytes */%d" % size})
            with open(path, "rb") as f:
                f.seek(start)
                body = f.read(end - start + 1)
            extra = {"Accept-Ranges": "bytes"}
            if m:
                extra["Content-Range"] = "bytes %d-%d/%d" % (start, end, size)
            self.send(206 if m else 200, body, ctype, extra)

        def zip(self, tid: str, path: str) -> None:
            """Everything about one throw except the raw frame files: numbers, trace, IMU and the videos."""
            buf = io.BytesIO()
            with zipfile.ZipFile(buf, "w", zipfile.ZIP_STORED) as z:
                for name in ("throw.json", "label.json", "events.jsonl", "trace.jsonl", "imu.csv", "meta.json"):
                    if os.path.exists(os.path.join(path, name)):
                        z.write(os.path.join(path, name), os.path.join(tid, name))
                for name in lab.videos(path):
                    z.write(os.path.join(path, "videos", name), os.path.join(tid, "videos", name))
            self.send(200, buf.getvalue(), "application/zip", {"Content-Disposition": 'attachment; filename="%s.zip"' % tid})

        def do_POST(self) -> None:
            try:
                n = min(int(self.headers.get("Content-Length", "0") or 0), 10000)
                body = json.loads(self.rfile.read(n) or b"{}")
                parts = [unquote(p) for p in urlparse(self.path).path.split("/") if p]
                if parts == ["api", "mark"]:
                    ok = robot.mark(str(body.get("note", "")), float(body.get("wait", 0) or 0))
                    return self.json({"ok": ok, "error": "" if ok else "not connected to the dodge node: " + (robot.problem or "is the stack running?")})
                if len(parts) == 3 and parts[:2] == ["api", "videos"]:
                    if lab.path(parts[2]) is None:
                        return self.missing()
                    lab.wanted.add(parts[2])
                    return self.json({"ok": True})
                if len(parts) == 3 and parts[:2] == ["api", "label"]:
                    path = lab.path(parts[2])
                    if path is None:
                        return self.missing()
                    result = str(body.get("result", ""))
                    label = throwlog.save_label(path, result if result in RESULTS else "", str(body.get("note", "")))
                    lab.write_csv()
                    return self.json({"ok": True, "label": label})
                self.missing()
            except (BrokenPipeError, ConnectionResetError):
                pass
            except Exception as exc:
                self.json({"error": "%s: %s" % (type(exc).__name__, exc)}, 400)

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
    ap.add_argument("--port", type=int, default=8077)
    ap.add_argument("--dir", default=None, help="folder of saved throws (default: node.event_dir from the config)")
    ap.add_argument("--videos", action="store_true",
                    help="make the .mp4 files for every throw as it arrives. Off by default: encoding video takes processor "
                         "time the dodge node needs. The page plays the pictures either way and has a button per throw.")
    ap.add_argument("--no-videos", action="store_true", help="with --once: numbers and throws.csv only, no .mp4 files")
    ap.add_argument("--no-ros", action="store_true", help="saved throws only: no live state, no save button")
    ap.add_argument("--once", action="store_true", help="analyse everything, write the videos and throws.csv, then exit")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()
    cfg = load_config(args.config)
    home = args.dir or cfg["node"].get("event_dir", "data/throws")
    if not os.path.isabs(home):
        home = os.path.join(ROOT, home)
    window = float(cfg["node"].get("event_buffer_s", 2.0)) + float(cfg["node"].get("event_post_s", 1.2))
    lab = Lab([home, os.path.join(ROOT, "data", "events")] if not args.dir else [home],
              make_videos=(not args.no_videos) if args.once else args.videos, window_s=window)
    if args.once:
        for tid, path in lab.paths().items():
            print(tid)
            t = throwlog.load_throw(path)
            for line in t["story"]:
                print("   " + line)
            if lab.make_videos and not lab.videos(path):
                throwlog.render_videos(path, t)
        lab.write_csv()
        print("summary: " + os.path.join(home, "throws.csv"))
        return
    robot = Robot(not args.no_ros)
    threading.Thread(target=lab.work, daemon=True, name="develop").start()
    server = ThreadingHTTPServer(("0.0.0.0", args.port), make_handler(lab, robot))
    server.daemon_threads = True
    print("Pupper Dodging View is running. Open one of these in a browser:")
    for a in addresses(args.port):
        print("    " + a)
    print("Saved throws: %s   (Ctrl+C stops this page; the robot is not affected)" % home)
    if robot.problem:
        print("Note: " + robot.problem)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        lab.stop = True
        server.server_close()
        if robot.node is not None:
            import rclpy
            rclpy.try_shutdown()
        os._exit(0)


if __name__ == "__main__":
    main()
