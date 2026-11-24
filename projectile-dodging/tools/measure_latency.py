#!/usr/bin/env python3
"""Measure how late a camera's timestamps are, by knocking on the robot.

    python3 tools/measure_latency.py elp_left          # then tap the robot firmly 5 times, 2-3 s apart
    python3 tools/measure_latency.py fisheye --write   # also save the result to config/calib/

A sharp tap shows up in the IMU at once and in the picture as a sudden jolt.
Comparing the two gives the error of the camera's timestamps relative to the
IMU, which is what `time_offset_s` in the config corrects. The robot should be
standing (or sitting on its stand) looking at a scene with some detail, the
robot stack running. The dodge node must not be running if you test an ELP.

The other two parts of the reaction time are reported elsewhere: processing
time by the dodge node itself (robot/watch.sh shows "lag"), and the time from
command to motion by tools/step_response.py.
"""
from __future__ import annotations

import argparse
import time

from _common import cv2, load_config, np, open_feed, ros_node, run_tool, wait_first

from dodge.config import save_calib


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
    frames, prev = [], None
    end = time.time() + args.seconds
    while time.time() < end:
        item = feed.get(2.0)
        if item is None:
            continue
        t, bgr, _ = item
        small = cv2.cvtColor(cv2.resize(bgr, (160, 120), interpolation=cv2.INTER_AREA), cv2.COLOR_BGR2GRAY).astype(np.float32)
        if prev is not None:
            frames.append((t, float(np.abs(small - prev).mean())))
        prev = small
    feed.close()
    if len(imu) < 100:
        raise SystemExit("No IMU data: is the robot stack running and ROS_LOCALHOST_ONLY=1 exported?")
    if len(frames) < 30:
        raise SystemExit("Too few camera frames.")
    it = np.array([s[0] for s in imu])
    ig = np.array([s[1] for s in imu])
    ft = np.array([f[0] for f in frames])
    fd = np.array([f[1] for f in frames])
    period = float(np.median(np.diff(ft)))
    g_thr = max(0.4, float(np.median(ig)) + 6.0 * float(np.median(np.abs(ig - np.median(ig)))) + 0.2)
    d_thr = float(np.median(fd)) + 6.0 * float(np.median(np.abs(fd - np.median(fd)))) + 0.3
    knocks, last = [], -10.0
    for t, g in zip(it, ig):
        if g > g_thr and t - last > 1.0:
            knocks.append(t)
        if g > g_thr:
            last = t
    errors = []
    for tk in knocks:
        sel = np.nonzero((ft > tk - 0.15) & (ft < tk + 0.5) & (fd > d_thr))[0]
        if len(sel) == 0:
            continue
        # The first jolted frame was exposed up to one frame period after the knock: on average half a period.
        errors.append(float(ft[sel[0]] - period / 2.0 - tk))
    print("camera %.0f fps; %d knocks in the IMU, %d also seen in the picture" % (1.0 / period, len(knocks), len(errors)))
    if len(errors) < 3:
        raise SystemExit("Need at least 3 clean knocks. Tap harder, leave 2-3 s between taps, and make sure the camera "
                         "sees something with detail (not a blank wall).")
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
