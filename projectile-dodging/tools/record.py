#!/usr/bin/env python3
"""Record all cameras and the IMU, with timestamps, for offline tuning.

    python3 tools/record.py --seconds 30 --name rear-lobs-01

Writes data/recordings/<name>/ (see dodge/recording.py for the format). The
dodge node must NOT be running (it holds the USB cameras); the robot stack
should be, so the fisheye and the IMU are recorded too. Frames are stored as
the JPEGs the cameras produce, so recording costs little CPU.

Suggested sets (plan, Phase 1): 20+ lobs per sector, some throws that miss,
a minute of the empty room, a minute of the robot walking with no ball.
"""
from __future__ import annotations

import argparse
import os
import time

from _common import ROOT, cv2, load_config, np

from dodge.config import enabled_cameras
from dodge.recording import RecordingWriter


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--name", default=time.strftime("%Y%m%d-%H%M%S"))
    ap.add_argument("--cameras", default="", help="comma-separated subset")
    ap.add_argument("--no-ros", action="store_true", help="USB cameras only (no fisheye, no IMU)")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()
    cfg = load_config(args.config)
    cams = enabled_cameras(cfg)
    only = [c.strip() for c in args.cameras.split(",") if c.strip()]
    if only:
        cams = {k: v for k, v in cams.items() if k in only}
    path = os.path.join(ROOT, "data", "recordings", args.name)
    writer = RecordingWriter(path, {"kind": "recording", "started": time.time(), "config": cfg})
    counts = {name: 0 for name in cams}
    threads = []

    def on_usb(name, t, jpeg, bgr):
        if jpeg is None:                      # camera not in pass-through mode: re-encode (slower)
            ok, enc = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 92])
            if not ok:
                return
            jpeg = enc
        writer.add_frame(name, t, bytes(jpeg))
        counts[name] += 1

    from usb_camera import UsbCamera
    for name, cam in cams.items():
        if cam.get("source") == "v4l2":
            if not cam.get("device"):
                print("%s: no device set, skipped (tools/find_cameras.py --assign)" % name)
                continue
            c = dict(cam)
            c["max_process_fps"] = 0          # every frame
            th = UsbCamera(name, c, on_usb, log=lambda m: print("  " + m))
            th.start()
            threads.append(th)

    imu_count = [0]
    if not args.no_ros:
        from sensor_msgs.msg import CompressedImage, Imu

        from _common import ros_node
        node = ros_node("dodge_record")
        for name, cam in cams.items():
            if cam.get("source") == "ros_compressed":
                def on_img(msg, name=name):
                    now = time.time()
                    t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                    writer.add_frame(name, t if abs(now - t) < 2.0 else now, bytes(np.frombuffer(msg.data, dtype=np.uint8)))
                    counts[name] += 1
                node.create_subscription(CompressedImage, cam.get("topic", "/camera/image_raw/compressed"), on_img, 5)

        def on_imu(msg):
            now = time.time()
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            q, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
            writer.add_imu(t if abs(now - t) < 2.0 else now, (q.w, q.x, q.y, q.z), (w.x, w.y, w.z), (a.x, a.y, a.z))
            imu_count[0] += 1
        node.create_subscription(Imu, cfg["node"].get("imu_topic", "/imu_sensor_broadcaster/imu"), on_imu, 100)

    print("recording %.0f s to %s  (Ctrl+C to stop early)" % (args.seconds, path))
    t0 = time.time()
    try:
        while time.time() - t0 < args.seconds:
            time.sleep(1.0)
            print("  %4.0f s  " % (time.time() - t0) + "  ".join("%s %d" % kv for kv in counts.items())
                  + "  imu %d" % imu_count[0])
    except KeyboardInterrupt:
        pass
    for th in threads:
        th.stop()
    time.sleep(0.3)
    writer.close()
    dur = max(time.time() - t0, 1e-3)
    print("done: " + ", ".join("%s %.0f fps" % (k, v / dur) for k, v in counts.items()) + ", imu %.0f Hz" % (imu_count[0] / dur))
    missing = [k for k, v in counts.items() if v == 0]
    if missing:
        print("WARNING: no frames from %s" % ", ".join(missing))
    if not args.no_ros and imu_count[0] == 0:
        print("WARNING: no IMU data (is the robot stack running, and ROS_LOCALHOST_ONLY=1 exported?)")


if __name__ == "__main__":
    main()
