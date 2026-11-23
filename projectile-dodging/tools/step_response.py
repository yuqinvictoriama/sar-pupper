#!/usr/bin/env python3
"""Measure how fast Pupper actually gets out of the way. THIS MOVES THE ROBOT.

    python3 tools/step_response.py --vx 1.0                 # forward burst
    python3 tools/step_response.py --vy 0.5                 # sidestep to the left
    python3 tools/step_response.py --vx 1.0 --vy -0.5       # the diagonal the dodge uses
    python3 tools/step_response.py --vx 1.5 --write         # save the fitted lag to config/calib/

It sends the same kind of velocity burst the dodge does (on /dodge_cmd_vel,
through the mux), records the IMU, and works out from the acceleration how far
the robot moved and when. It prints the delay before the robot starts moving,
the time to cover 0.15 / 0.20 / 0.25 m, the peak body tilt (the tilt e-stop
trips at about 57 degrees for the default controller) and the lag of a
first-order model, which is what `escape.accel_lag_s` in the config means.

Needs: the robot stack running (robot/start.sh), the robot standing in a
walking controller on open floor with at least 1.5 m clear in the direction of
travel, and a person at the e-stop. Distances come from integrating the IMU,
so treat them as +-5 cm; a phone slow-motion video over floor tape is the
check.
"""
from __future__ import annotations

import argparse
import math
import os
import time

from _common import ROOT, load_config, np, ros_node

from dodge.config import save_calib
from dodge.core import quat_to_rot
from dodge.escape import lag_distance


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vx", type=float, default=0.0)
    ap.add_argument("--vy", type=float, default=0.0)
    ap.add_argument("--duration", type=float, default=0.5, help="length of the burst (s)")
    ap.add_argument("--topic", default=None, help="velocity topic (default: the dodge node's)")
    ap.add_argument("--write", action="store_true", help="save the fitted lag to config/calib/step_response.yaml")
    ap.add_argument("--yes", action="store_true", help="skip the confirmation question")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()
    if abs(args.vx) > 1.5 or abs(args.vy) > 0.5 or args.duration > 1.0:
        raise SystemExit("Limits: |vx| <= 1.5, |vy| <= 0.5 m/s, duration <= 1 s.")
    speed = math.hypot(args.vx, args.vy)
    if speed < 0.05:
        raise SystemExit("Give --vx and/or --vy.")
    cfg = load_config(args.config)
    topic = args.topic or cfg["node"].get("cmd_topic", "/dodge_cmd_vel")
    if not args.yes:
        print("The robot will move at vx %+.1f, vy %+.1f m/s for %.2f s (about %.2f m)." % (
            args.vx, args.vy, args.duration, speed * args.duration))
        if input("Floor clear, robot standing, someone at the e-stop? Type 'go': ").strip().lower() != "go":
            raise SystemExit("cancelled")
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Imu
    node = ros_node("dodge_step_response")
    samples = []

    def on_imu(msg):
        now = time.time()
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q, w, a = msg.orientation, msg.angular_velocity, msg.linear_acceleration
        samples.append((t if abs(now - t) < 2.0 else now, (q.w, q.x, q.y, q.z), (w.x, w.y, w.z), (a.x, a.y, a.z)))
    node.create_subscription(Imu, cfg["node"].get("imu_topic", "/imu_sensor_broadcaster/imu"), on_imu, 500)
    pub = node.create_publisher(Twist, topic, 10)
    time.sleep(1.2)                                   # standing still: baseline for the accelerometer
    if len(samples) < 50:
        raise SystemExit("No IMU data: is the robot stack running and ROS_LOCALHOST_ONLY=1 exported?")
    cmd = Twist()
    cmd.linear.x, cmd.linear.y = float(args.vx), float(args.vy)
    t_cmd = time.time()
    while time.time() - t_cmd < args.duration:
        pub.publish(cmd)
        time.sleep(0.01)
    t_stop = time.time()
    while time.time() - t_stop < 0.8:                 # hold zero so the mux keeps the robot still
        pub.publish(Twist())
        time.sleep(0.01)
    data = list(samples)
    t = np.array([s[0] for s in data])
    R0 = quat_to_rot(*data[max(0, int(np.searchsorted(t, t_cmd)) - 1)][1])
    acc_w = np.array([quat_to_rot(*s[1]) @ np.array(s[3]) for s in data])
    tilt = np.array([math.degrees(math.acos(max(-1.0, min(1.0, quat_to_rot(*s[1])[2, 2])))) for s in data])
    before = (t > t_cmd - 1.0) & (t < t_cmd - 0.05)
    acc_w = acc_w - acc_w[before].mean(axis=0)        # removes gravity and bias
    acc_b = acc_w @ R0                                # in the robot's starting frame: x forward, y left
    d_hat = np.array([args.vx, args.vy]) / speed
    along = acc_b[:, :2] @ d_hat
    sel = t >= t_cmd
    ts = t[sel] - t_cmd
    dt = np.diff(ts, prepend=0.0)
    vel = np.cumsum(along[sel] * dt)
    pos = np.cumsum(vel * dt)
    noise = float(np.std(along[before])) if before.any() else 0.2
    moving = np.nonzero(np.abs(along[sel]) > max(1.0, 6.0 * noise))[0]
    onset = float(ts[moving[0]]) if len(moving) else float("nan")

    def time_to(d):
        hit = np.nonzero(pos >= d)[0]
        return float(ts[hit[0]]) if len(hit) else float("nan")

    in_burst = ts <= args.duration
    best = (float("inf"), 0.2)
    delay = 0.0 if math.isnan(onset) else onset
    for lag in np.arange(0.02, 0.8, 0.01):
        model = np.array([lag_distance(max(0.0, x - delay), speed, lag) for x in ts[in_burst]])
        err = float(np.mean((model - pos[in_burst]) ** 2))
        if err < best[0]:
            best = (err, float(lag))
    lag = best[1]
    print("\ncommand %+.1f, %+.1f m/s for %.2f s" % (args.vx, args.vy, args.duration))
    print("  delay before the robot starts to move: %.0f ms" % (onset * 1000))
    for d in (0.15, 0.20, 0.25, 0.30):
        print("  time to move %.2f m: %s" % (d, "%.2f s" % time_to(d) if not math.isnan(time_to(d)) else "not reached"))
    print("  moved %.2f m by the end of the burst, %.2f m in total; top speed %.2f m/s" % (
        float(pos[in_burst][-1]) if in_burst.any() else 0.0, float(pos[-1]), float(vel.max())))
    print("  peak body tilt: %.0f degrees" % float(tilt[sel].max()))
    print("  first-order lag that fits: %.2f s   (config escape.accel_lag_s is %.2f, react_s is %.2f)" % (
        lag, float(cfg["escape"]["accel_lag_s"]), float(cfg["escape"]["react_s"])))
    out_dir = os.path.join(ROOT, "data", "step_response")
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, time.strftime("%Y%m%d-%H%M%S") + "_vx%+.1f_vy%+.1f.csv" % (args.vx, args.vy))
    np.savetxt(out, np.column_stack([ts, along[sel], vel, pos, tilt[sel]]), delimiter=",",
               header="t,accel_along,vel,pos,tilt_deg", comments="")
    print("  curve saved to %s" % out)
    if args.write:
        path = save_calib("step_response", {"escape": {"accel_lag_s": round(lag, 3)}})
        print("  accel_lag_s written to %s" % path)
        print("  (also raise escape.react_s if the delay above is much more than 60 ms)")


if __name__ == "__main__":
    main()
