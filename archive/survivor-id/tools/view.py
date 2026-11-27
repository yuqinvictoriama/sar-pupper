#!/usr/bin/env python3
"""See what survivor identification sees, live, in a browser.

    on the Pupper:   source robot/env.sh && python3 tools/view.py
    on the laptop:   ssh -N -L 8090:localhost:8090 pi@<pupper>     then open http://localhost:8090

It shows /survivor/debug/compressed: the three camera views with a skeleton on everyone, green
when both arms read as up, red for the pinned survivor, and the state along the top. The
survivor node only draws that picture while someone is watching, so this costs nothing when closed.
"""
from __future__ import annotations

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage

latest = {"jpeg": None, "n": 0}
cond = threading.Condition()

PAGE = b"""<!doctype html><title>Survivor ID</title>
<body style="margin:0;background:#111;color:#ccc;font-family:sans-serif">
<img src="/stream" style="width:100%"><p style="margin:8px">Green: both arms up. Red: the pinned survivor.
Amber: an arm is raised. Grey: nothing of note.</p></body>"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass

    def do_GET(self):
        if self.path != "/stream":
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(PAGE)
            return
        self.send_response(200)
        self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
        self.end_headers()
        seen = 0
        try:
            while True:
                with cond:
                    cond.wait_for(lambda: latest["n"] != seen, timeout=5.0)
                    seen, jpeg = latest["n"], latest["jpeg"]
                if jpeg is None:
                    continue
                self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(jpeg))
                self.wfile.write(jpeg)
                self.wfile.write(b"\r\n")
        except (BrokenPipeError, ConnectionResetError):
            pass


def on_image(msg: CompressedImage) -> None:
    with cond:
        latest["jpeg"], latest["n"] = bytes(msg.data), latest["n"] + 1
        cond.notify_all()


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", type=int, default=8090)
    args = ap.parse_args()
    rclpy.init()
    node = Node("survivor_view")
    node.create_subscription(CompressedImage, "/survivor/debug/compressed", on_image, 1)
    server = ThreadingHTTPServer(("127.0.0.1", args.port), Handler)
    threading.Thread(target=server.serve_forever, daemon=True).start()
    print("serving on http://localhost:%d (this machine only; tunnel it with ssh -L)" % args.port)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        time.sleep(0.1)


if __name__ == "__main__":
    main()
