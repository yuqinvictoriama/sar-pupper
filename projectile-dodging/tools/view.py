#!/usr/bin/env python3
"""See what a camera and the ball detector see, in a browser.

    python3 tools/view.py elp_left            # then open http://<pupper-ip>:8090
    python3 tools/view.py fisheye --mask      # show the colour mask instead of the picture

Over SSH from a laptop:  ssh -N -L 8090:localhost:8090 pi@<pupper-ip>  and open http://localhost:8090

Each detected ball gets a red box with its range and bearing. On the fisheye a
thin red circle marks where the detector stops looking (the rim of the lens is
ignored). The fisheye needs the robot stack running (its frames come from the
course camera node); the ELP cameras are opened directly, so the dodge node
must NOT be running at the same time (a USB camera serves one program at a time).
"""
from __future__ import annotations

import argparse
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from _common import cv2, load_config, open_feed, run_tool, wait_first

from dodge.camera_models import Camera
from dodge.detect import BallDetector, circle_params, draw_blobs
from dodge.measure import measure_blob

latest = {"jpeg": None}


class Handler(BaseHTTPRequestHandler):
    def log_message(self, *_a):
        pass

    def do_GET(self):
        if self.path.startswith("/stream"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    data = latest["jpeg"]
                    if data is not None:
                        self.wfile.write(b"--frame\r\nContent-Type: image/jpeg\r\nContent-Length: %d\r\n\r\n" % len(data))
                        self.wfile.write(data + b"\r\n")
                    time.sleep(0.1)
            except (BrokenPipeError, ConnectionResetError):
                return
        else:
            self.send_response(200)
            self.send_header("Content-Type", "text/html")
            self.end_headers()
            self.wfile.write(b"<html><body style='margin:0;background:#111'><img src='/stream' style='width:100%'></body></html>")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("camera")
    ap.add_argument("--port", type=int, default=8090)
    ap.add_argument("--mask", action="store_true", help="show the colour mask")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()
    cfg = load_config(args.config)
    cam_cfg = cfg["cameras"][args.camera]
    camera = Camera.from_config(args.camera, cam_cfg)
    det = BallDetector.for_camera(cam_cfg, cfg["detector"])
    feed = open_feed(cfg, args.camera, max_fps=15)
    wait_first(feed)
    threading.Thread(target=ThreadingHTTPServer(("0.0.0.0", args.port), Handler).serve_forever, daemon=True).start()
    print("open http://<pupper-ip>:%d   (Ctrl+C to stop)" % args.port)
    last_print = 0.0
    try:
        while True:
            item = feed.get(5.0)
            if item is None:
                print("no frames...")
                continue
            _t, bgr, _ = item
            cam = camera.with_resolution(bgr.shape[1], bgr.shape[0])
            blobs = det.detect(bgr)
            labels, lines = [], []
            for b in blobs:
                m = measure_blob(cam, b, cfg["ball"]["diameter_m"])
                if m is None:
                    labels.append("%.0fpx (ignored)" % b.diameter_px)
                    continue
                labels.append("%.2fm %+.0f/%+.0fdeg" % (m.range_m, m.azimuth_deg, m.elevation_deg))
                lines.append("%.1f px wide, %.2f m, bearing %+.0f deg, elevation %+.0f deg" % (
                    b.diameter_px, m.range_m, m.azimuth_deg, m.elevation_deg))
            show = cv2.cvtColor(det.mask(bgr), cv2.COLOR_GRAY2BGR) if args.mask else bgr
            show = draw_blobs(show, blobs, labels)
            circle = circle_params(det.image_circle, det.ref_size, bgr.shape[1], bgr.shape[0])
            if circle is not None:
                cv2.circle(show, (int(circle[0]), int(circle[1])), int(circle[2]), (0, 0, 255), 1)
            if show.shape[1] > 1280:
                show = cv2.resize(show, (1280, int(show.shape[0] * 1280 / show.shape[1])))
            ok, enc = cv2.imencode(".jpg", show, [cv2.IMWRITE_JPEG_QUALITY, 70])
            if ok:
                latest["jpeg"] = enc.tobytes()
            if time.time() - last_print > 1.0:
                last_print = time.time()
                level = float(bgr[::16, ::16].mean())
                dark = ""
                if level < 25.0 and cam_cfg.get("source") == "v4l2":
                    dark = "   [picture very dark, average %.0f of 255: run  python3 tools/tune_exposure.py %s]" % (level, args.camera)
                print("%d blob(s)%s%s" % (len(blobs), ": " + "; ".join(lines) if lines else "", dark))
    finally:
        feed.close()


if __name__ == "__main__":
    run_tool(main)
