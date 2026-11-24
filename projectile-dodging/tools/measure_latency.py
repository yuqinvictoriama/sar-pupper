#!/usr/bin/env python3
"""Measure how late a camera's timestamps are, by knocking on the robot.

    python3 tools/measure_latency.py elp_left          # then tap the robot firmly 5 times, 2-3 s apart
    python3 tools/measure_latency.py fisheye --write   # also save the result to config/calib/

A sharp tap shows up in the IMU at once and in the picture as a sudden jolt.
Comparing the two gives the error of the camera's timestamps relative to the
IMU, which is what `time_offset_s` in the config corrects. Rap with a knuckle
on the head, beside the cameras, so the cameras themselves shake. The robot
should be standing (or on its stand) looking at a lit scene with some detail,
the robot stack running. The dodge node must not be running if you test an ELP.

This only refines timing by a few hundredths of a second. If it will not
work, skip it: the cameras' own timestamps are used as they are.

The other two parts of the reaction time are reported elsewhere: processing
time by the dodge node itself (robot/watch.sh shows "lag"), and the time from
command to motion by tools/step_response.py.
"""
from __future__ import annotations

import argparse
import math
import time

from _common import cv2, load_config, np, open_feed, ros_node, run_tool, wait_first

from dodge.config import save_calib

SMALL_W = 320       # pictures are shrunk to this width before they are compared


class JoltMeter:
    """How far the whole picture moved between one frame and the next, in pixels of the full frame.

    A knock moves the picture by a pixel or less, and at high camera gain the
    picture's own noise changes every pixel by more than that. So the shift of
    the picture as a whole is solved for from all its pixels at once (the
    brightness change between two frames, set against the picture's slopes),
    on a shrunken, slightly blurred grey copy. That averages the noise away:
    a still camera reads a few hundredths of a pixel.
    """

    def __init__(self):
        self.prev = None
        self.scale = 1.0

    def add(self, bgr) -> float | None:
        h, w = bgr.shape[:2]
        size = (SMALL_W, max(16, int(round(SMALL_W * h / float(w)))))
        small = cv2.cvtColor(cv2.resize(bgr, size, interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2GRAY).astype(np.float32)
        small = cv2.GaussianBlur(small, (0, 0), 1.2)
        if self.prev is None or self.prev.shape != small.shape:
            self.prev, self.scale = small, w / float(SMALL_W)
            return None
        mean = 0.5 * (small + self.prev)
        gx = cv2.Sobel(mean, cv2.CV_32F, 1, 0, ksize=3, scale=0.125)
        gy = cv2.Sobel(mean, cv2.CV_32F, 0, 1, ksize=3, scale=0.125)
        gt = small - self.prev
        self.prev = small
        a = np.array([[float((gx * gx).sum()), float((gx * gy).sum())], [float((gx * gy).sum()), float((gy * gy).sum())]])
        rhs = -np.array([float((gx * gt).sum()), float((gy * gt).sum())])
        if np.linalg.cond(a) > 1e6:
            return 0.0                                     # a featureless picture: no shift can be measured
        dx, dy = np.linalg.solve(a, rhs)
        return math.hypot(dx, dy) * self.scale


def match_knocks(imu_t, imu_rate, frame_t, frame_moved):
    """Pair each knock in the IMU with the first frame in which the picture jolted.

    Returns (knock times, timestamp errors of the matched ones, jolt threshold,
    biggest jolt near each knock). A positive error means the camera's
    timestamps are late.
    """
    it, ig = np.asarray(imu_t, float), np.asarray(imu_rate, float)
    ft, fd = np.asarray(frame_t, float), np.asarray(frame_moved, float)
    period = float(np.median(np.diff(ft)))
    g_thr = max(0.4, float(np.median(ig)) + 6.0 * float(np.median(np.abs(ig - np.median(ig)))) + 0.2)
    d_thr = float(np.median(fd)) + 6.0 * float(np.median(np.abs(fd - np.median(fd)))) + 0.05
    knocks, last = [], -10.0
    for t, g in zip(it, ig):
        if g > g_thr and t - last > 1.0:
            knocks.append(float(t))
        if g > g_thr:
            last = t
    errors, biggest = [], []
    for tk in knocks:
        near = (ft > tk - 0.15) & (ft < tk + 0.5)
        biggest.append(float(fd[near].max()) if near.any() else 0.0)
        sel = np.nonzero(near & (fd > d_thr))[0]
        if len(sel):
            # The first jolted frame was exposed up to one frame period after the knock: on average half a period.
            errors.append(float(ft[sel[0]] - period / 2.0 - tk))
    return knocks, errors, d_thr, biggest, period


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("camera")
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--write", action="store_true")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()
    cfg = load_config(args.config)
    from sensor_msgs.msg import Imu
    node = ros_node("dodge_latency")
    imu = []

    def on_imu(msg):
        now = time.time()
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        w = msg.angular_velocity
        imu.append((t if abs(now - t) < 2.0 else now, (w.x * w.x + w.y * w.y + w.z * w.z) ** 0.5))
    node.create_subscription(Imu, cfg["node"].get("imu_topic", "/imu_sensor_broadcaster/imu"), on_imu, 200)
    feed = open_feed(cfg, args.camera)
    wait_first(feed)
    print("Recording %.0f s. Tap the robot's body firmly about five times, 2-3 seconds apart..." % args.seconds)
    frames = []
    meter = JoltMeter()
    end = time.time() + args.seconds
    while time.time() < end:
        item = feed.get(2.0)
        if item is None:
            continue
        t, bgr, _ = item
        moved = meter.add(bgr)
        if moved is not None:
            frames.append((t, moved))
    feed.close()
    if len(imu) < 100:
        raise SystemExit("No IMU data: is the robot stack running, and did you  source robot/env.sh  in this terminal?")
    if len(frames) < 30:
        raise SystemExit("Too few camera frames.")
    knocks, errors, d_thr, biggest, period = match_knocks([s[0] for s in imu], [s[1] for s in imu],
                                                          [f[0] for f in frames], [f[1] for f in frames])
    print("camera %.0f fps; %d knocks in the IMU, %d also seen in the picture" % (1.0 / period, len(knocks), len(errors)))
    if len(knocks) < 3:
        raise SystemExit("The IMU felt fewer than 3 knocks. Tap more sharply (a knuckle, not a push), 2-3 s apart.")
    if len(errors) < 3:
        raise SystemExit(
            "The IMU felt the knocks but the picture barely moved: at the knocks it shifted by %s px, and it takes more\n"
            "than %.2f px to stand out from this picture's own jitter. The camera has to actually shake:\n"
            "  - rap with a knuckle on the head, right beside the cameras, not on the body or the legs;\n"
            "  - have the robot standing or on its stand (lying on the floor it cannot move);\n"
            "  - point the camera at something with detail and light on it (shelves, not a blank wall or a dark corner).\n"
            "This measurement only refines timing. If it will not work now, skip it and carry on: the cameras' own\n"
            "timestamps are used as they are." % (", ".join("%.2f" % b for b in biggest), d_thr))
    err = float(np.median(errors))
    spread = float(np.max(errors) - np.min(errors))
    print("timestamps are %.0f ms %s (spread %.0f ms, about one frame of that is unavoidable)" % (
        abs(err) * 1000, "late" if err > 0 else "early", spread * 1000))
    print("-> time_offset_s for %s should be %.3f (config has %.3f)" % (
        args.camera, -err, float(cfg["cameras"][args.camera].get("time_offset_s", 0.0))))
    if args.write:
        path = save_calib(args.camera + "_timing", {"cameras": {args.camera: {"time_offset_s": round(-err, 4)}}})
        print("written to %s" % path)


if __name__ == "__main__":
    run_tool(main)
