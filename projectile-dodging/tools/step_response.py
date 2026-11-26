#!/usr/bin/env python3
"""Measure how Pupper responds to the kind of velocity burst a dodge sends. THIS MOVES THE ROBOT.

    python3 tools/step_response.py --vx 1.0                 # forward burst
    python3 tools/step_response.py --vy 0.5                 # sidestep to the left
    python3 tools/step_response.py --vx 1.0 --vy 0.5        # the diagonal the dodge uses

It sends the burst on /dodge_cmd_vel (through the mux) and records the IMU. From
the turn-rate sensor it reads how long the body takes to react, and from the
orientation the peak body tilt (the tilt e-stop trips at about 57 degrees for
the default controller).

Pupper's IMU sends orientation and turn rate but NO acceleration (its driver
leaves the accelerometer off), so how far the robot got cannot be read from the
IMU. To measure that, film a burst from the side in slow motion over two tape
marks, 0.15 m and 0.30 m from the foot nearest the direction of travel, and
count the time from the first movement to each mark. Then, with no robot needed:

    python3 tools/step_response.py --vx 1.0 --vy 0.5 --timed 0.15:0.31 0.30:0.47 --write

fits the lag (`escape.accel_lag_s` in the config) to those timings and saves it.
Without that the config's own lag stays in use.

Needs, for a burst: the robot stack running (robot/start.sh), the robot standing
in a walking controller on open floor with at least 1 m clear in the direction
of travel, and a person at the e-stop.
"""
from __future__ import annotations

import argparse
import math
import os
import time

from _common import load_config, np, ROOT, ros_node, run_tool

from dodge.config import MAX_SAVED_LAG_S, save_calib
from dodge.core import quat_to_rot
from dodge.escape import lag_distance
from dodge.throwlog import reaction_delay  # noqa: F401  (the same test the throw recorder uses)


LAGS = [round(0.01 * k, 2) for k in range(0, 81)]      # lags tried when fitting, 0 to 0.80 s


def has_acceleration(acc) -> bool:
    """False when the IMU's acceleration is all zeros, which is what Pupper's driver sends."""
    return bool(np.abs(np.asarray(acc, dtype=float)).max() > 1e-6) if len(acc) else False


def parse_timed(items):
    """['0.15:0.31', '0.30:0.47'] -> [(0.15, 0.31), (0.30, 0.47)]: metres covered, seconds after first movement."""
    out = []
    for item in items:
        try:
            d, sec = (float(x) for x in item.split(":"))
        except ValueError:
            raise SystemExit("--timed takes DISTANCE:SECONDS pairs, for example  --timed 0.15:0.31 0.30:0.47")
        if not (0.02 <= d <= 1.5 and 0.02 <= sec <= 3.0):
            raise SystemExit("--timed %s: distances are in metres (0.02 to 1.5), times in seconds (0.02 to 3)." % item)
        out.append((d, sec))
    return out


def fit_lag(points, speed):
    """The first-order lag that best explains (distance, seconds since first movement) pairs at a
    commanded speed. Returns (lag, rms error in metres, problem); problem is None or says why the
    timings cannot be right."""
    for d, sec in points:
        if d > speed * sec * 1.1:
            return float("nan"), float("nan"), (
                "%.2f m in %.2f s is %.2f m/s, faster than the %.2f m/s that was commanded. Check the timing, "
                "the distance and the --vx/--vy you gave." % (d, sec, d / sec, speed))
    best = (float("inf"), 0.0)
    for lag in LAGS:
        err = sum((lag_distance(sec, speed, lag) - d) ** 2 for d, sec in points) / len(points)
        if err < best[0]:
            best = (err, lag)
    err, lag = math.sqrt(best[0]), best[1]
    if lag >= LAGS[-1] - 1e-9:
        return lag, err, ("the robot covered the distance more slowly than any lag up to %.1f s explains. Either it "
                          "walks slower than the commanded %.2f m/s, or the times were counted from the command "
                          "instead of from the first movement." % (LAGS[-1], speed))
    return lag, err, None


def report_timed(args, cfg, speed) -> None:
    """Fit the lag to timings taken from a video. Does not touch the robot."""
    points = parse_timed(args.timed)
    lag, err, problem = fit_lag(points, speed)
    print("command %+.1f, %+.1f m/s; timings from the video:" % (args.vx, args.vy))
    for d, sec in points:
        print("  %.2f m after %.2f s" % (d, sec))
    if problem:
        raise SystemExit("These timings do not fit: " + problem + "\nNothing was written.")
    print("  first-order lag that fits: %.2f s, to within %.0f cm   (config escape.accel_lag_s is %.2f)" % (
        lag, err * 100, float(cfg["escape"]["accel_lag_s"])))
    if len(points) < 2:
        print("  (one timing only: a second mark, e.g. 0.30 m, makes this much more trustworthy)")
    if err > 0.04:
        print("  The two timings disagree by more than 4 cm with any single lag; time the video again before trusting this.")
    if args.write:
        if lag >= MAX_SAVED_LAG_S:
            raise SystemExit("A lag of %.2f s is too long to be a walking response, so it was not saved. "
                             "Check the timings." % lag)
        path = save_calib("step_response", {"escape": {"accel_lag_s": round(lag, 3)}})
        print("  accel_lag_s written to %s" % path)
    else:
        print("  (add --write to save it)")
    print("  Try it in the simulation:  python3 tools/sim_report.py --dist 1.8 2.0 --rise 0.4 0.7 --lag %.2f" % lag)



def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--vx", type=float, default=0.0)
    ap.add_argument("--vy", type=float, default=0.0)
    ap.add_argument("--duration", type=float, default=0.5, help="length of the burst (s)")
    ap.add_argument("--topic", default=None, help="velocity topic (default: the dodge node's)")
    ap.add_argument("--timed", nargs="+", metavar="M:S", default=None,
                    help="fit the lag to timings from a slow-motion video instead of moving the robot: "
                         "metres covered and seconds since the first movement, e.g. 0.15:0.31 0.30:0.47")
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
    if args.timed:
        report_timed(args, cfg, speed)
        return
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
    # A new ROS node needs a moment to find the IMU's publisher, longer on a busy Pi. Wait for the
    # first message rather than assuming it has arrived; nothing is commanded until the IMU is in.
    waited = time.time()
    while not samples and time.time() - waited < 8.0:
        time.sleep(0.05)
    if not samples:
        raise SystemExit(
            "No IMU data after 8 s, so nothing was commanded. The robot stack is not running, or is still starting.\n"
            "  is it running?   pgrep -af ros2_control_node      (prints nothing if not)\n"
            "  start it:        bash robot/start.sh               (wait for it to finish, about a minute)\n"
            "  stand it up:     bash robot/walk.sh\n"
            "then run this again. python3 tools/check_setup.py lists anything else that is missing.")
    first = time.time()
    time.sleep(1.5)                                   # standing still: baseline for the accelerometer
    rate = len(samples) / max(time.time() - first, 1e-3)
    if rate < 20.0:
        raise SystemExit("The IMU is only sending about %.0f messages a second, too few to measure a half-second move. "
                         "Nothing was commanded. See /tmp/sar-dodge/robot.log." % rate)
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
    gyro = np.array([s[2] for s in data])
    acc = np.array([s[3] for s in data])
    tilt = np.array([math.degrees(math.acos(max(-1.0, min(1.0, quat_to_rot(*s[1])[2, 2])))) for s in data])
    sel = t >= t_cmd
    ts = t[sel] - t_cmd
    delay, thr = reaction_delay(t, gyro, t_cmd)
    react = float(cfg["escape"]["react_s"])
    print("\ncommand %+.1f, %+.1f m/s for %.2f s" % (args.vx, args.vy, args.duration))
    if math.isnan(delay):
        print("  delay before the body reacts: could not tell (it never turned faster than %.2f rad/s; if the robot "
              "did not visibly move, the burst did not reach it: see below)" % thr)
    else:
        print("  delay before the body reacts: %.0f ms   (config escape.react_s is %.2f)" % (delay * 1000, react))
    print("  peak body tilt: %.0f degrees" % float(tilt[sel].max()))
    cols, header = [ts, np.linalg.norm(gyro[sel], axis=1), tilt[sel]], "t,turn_rate,tilt_deg"
    lag = None
    if has_acceleration(acc):
        lag, extra, extra_header = distance_from_acceleration(data, t, acc, t_cmd, speed, args)
        cols, header = cols + extra, header + extra_header
        if lag is not None:
            print("  first-order lag that fits: %.2f s   (config escape.accel_lag_s is %.2f)" % (
                lag, float(cfg["escape"]["accel_lag_s"])))
    else:
        print("  distance covered and lag: not measured. This robot's IMU sends orientation and turn rate but no")
        print("  acceleration, so they cannot be read from it. The config's lag of %.2f s stays in use." % float(
            cfg["escape"]["accel_lag_s"]))
        print("  To measure the lag, film a burst from the side in slow motion over tape marks 0.15 m and 0.30 m ahead")
        print("  of the leading foot, count the time from the first movement to each mark, then:")
        print("      python3 tools/step_response.py --vx %g --vy %g --timed 0.15:<seconds> 0.30:<seconds> --write" % (
            args.vx, args.vy))
    if math.isnan(delay):
        print("  If the robot stood still: is it standing in a walking controller (bash robot/walk.sh), and does")
        print("  python3 tools/check_setup.py show every line OK?")
    elif delay > react + 0.06:
        print("  The body reacts later than the config assumes: set escape.react_s to about %.2f." % (round(delay + 0.02, 2)))
    out_dir = os.path.join(ROOT, "data", "step_response")
    os.makedirs(out_dir, exist_ok=True)
    out = os.path.join(out_dir, time.strftime("%Y%m%d-%H%M%S") + "_vx%+.1f_vy%+.1f.csv" % (args.vx, args.vy))
    np.savetxt(out, np.column_stack(cols), delimiter=",", header=header, comments="")
    print("  curve saved to %s" % out)
    if args.write:
        if lag is None:
            print("  Nothing was written: there is no measured lag to save (see --timed above).")
        elif lag >= MAX_SAVED_LAG_S:
            print("  Nothing was written: a lag of %.2f s is too long to be a walking response." % lag)
        else:
            path = save_calib("step_response", {"escape": {"accel_lag_s": round(lag, 3)}})
            print("  accel_lag_s written to %s" % path)


def distance_from_acceleration(data, t, acc, t_cmd, speed, args):
    """For an IMU that does send acceleration: integrate it along the commanded direction.
    Returns (lag or None, extra CSV columns, their header)."""
    R0 = quat_to_rot(*data[max(0, int(np.searchsorted(t, t_cmd)) - 1)][1])
    acc_w = np.array([quat_to_rot(*s[1]) @ a for s, a in zip(data, acc)])
    before = (t > t_cmd - 1.0) & (t < t_cmd - 0.05)
    acc_w = acc_w - acc_w[before].mean(axis=0)        # removes gravity and bias
    acc_b = acc_w @ R0                                # in the robot's starting frame: x forward, y left
    along = acc_b[:, :2] @ (np.array([args.vx, args.vy]) / speed)
    sel = t >= t_cmd
    ts = t[sel] - t_cmd
    dt = np.diff(ts, prepend=0.0)
    vel = np.cumsum(along[sel] * dt)
    pos = np.cumsum(vel * dt)
    noise = float(np.std(along[before])) if before.any() else 0.2
    moving = np.nonzero(np.abs(along[sel]) > max(1.0, 6.0 * noise))[0]
    extra, header = [along[sel], vel, pos], ",accel_along,vel,pos"
    if not len(moving) or float(vel.max()) < 0.05:
        print("  distance covered and lag: not measured (the acceleration shows no movement along the commanded direction)")
        return None, extra, header
    onset = float(ts[moving[0]])
    for d in (0.15, 0.20, 0.25, 0.30):
        hit = np.nonzero(pos >= d)[0]
        print("  time to move %.2f m: %s" % (d, "%.2f s" % float(ts[hit[0]]) if len(hit) else "not reached"))
    in_burst = ts <= args.duration
    print("  moved %.2f m by the end of the burst, %.2f m in total; top speed %.2f m/s" % (
        float(pos[in_burst][-1]) if in_burst.any() else 0.0, float(pos[-1]), float(vel.max())))
    best = (float("inf"), None)
    for lag in LAGS[2:]:
        model = np.array([lag_distance(max(0.0, x - onset), speed, lag) for x in ts[in_burst]])
        err = float(np.mean((model - pos[in_burst]) ** 2))
        if err < best[0]:
            best = (err, lag)
    return best[1], extra, header


if __name__ == "__main__":
    run_tool(main)
