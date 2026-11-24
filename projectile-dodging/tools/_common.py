"""Shared helpers for the tools: imports from the package, and live camera feeds."""
from __future__ import annotations

import os
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "robot"))

import cv2  # noqa: E402
import numpy as np  # noqa: E402

from dodge.config import load_config  # noqa: E402,F401


class Feed:
    """Newest frame from one camera: (t, bgr, jpeg_bytes_or_None)."""

    def __init__(self, name: str):
        self.name = name
        self.cond = threading.Condition()
        self.item = None
        self.count = 0
        self.closers = []

    def put(self, t, bgr, jpeg=None) -> None:
        with self.cond:
            self.item = (t, bgr, jpeg)
            self.count += 1
            self.cond.notify_all()

    def get(self, timeout: float = 3.0):
        """Wait for a frame newer than the last one returned. None on timeout."""
        with self.cond:
            seen = getattr(self, "_seen", 0)
            end = time.time() + timeout
            while self.count == seen:
                left = end - time.time()
                if left <= 0:
                    return None
                self.cond.wait(left)
            self._seen = self.count
            return self.item

    def close(self) -> None:
        """Release the camera. Call before the program exits (see finish())."""
        ok = True
        for fn in self.closers:
            ok = (fn() is not False) and ok
        self.closed_cleanly = ok


_ros = {}


def ros_node(name: str = "dodge_tool"):
    """A background-spun rclpy node (created once)."""
    if "node" not in _ros:
        import rclpy
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.executors import ExternalShutdownException
        rclpy.init()
        node = Node(name)
        ex = SingleThreadedExecutor()
        ex.add_node(node)

        def spin():
            try:
                ex.spin()
            except (KeyboardInterrupt, ExternalShutdownException):
                pass                              # Ctrl+C: nothing to report
            except Exception:                     # rclpy raises assorted errors while shutting down
                if rclpy.ok():
                    raise
        threading.Thread(target=spin, daemon=True).start()
        _ros["node"] = node
    return _ros["node"]


_feeds = []


def finish(code: int = 0) -> None:
    """End a tool cleanly: release the cameras, stop ROS, then leave.

    A camera thread still inside OpenCV, or a ROS thread still spinning, when
    Python exits makes the C++ underneath abort ("terminate called without an
    active exception"). So wait for them, and if one will not stop, leave
    without running the exit handlers that trip over it.
    """
    clean = True
    for feed in _feeds:
        feed.close()
        clean = clean and getattr(feed, "closed_cleanly", True)
    if "node" in _ros:
        try:
            import rclpy
            rclpy.try_shutdown()
        except Exception:
            pass
    sys.stdout.flush()
    sys.stderr.flush()
    if not clean or "node" in _ros:
        os._exit(code)
    if code:
        sys.exit(code)


def run_tool(main) -> None:
    """Run a tool's main() and end cleanly whatever happens (Ctrl+C included)."""
    code = 0
    try:
        main()
    except KeyboardInterrupt:
        print()
    except SystemExit as exc:
        if isinstance(exc.code, str):
            print(exc.code, file=sys.stderr)
            code = 1
        else:
            code = int(exc.code or 0)
    except Exception:
        import traceback
        traceback.print_exc()
        code = 1
    finish(code)


def open_feed(cfg: dict, name: str, max_fps: float | None = None) -> Feed:
    """Start delivering frames from the named camera of the config."""
    cam = cfg["cameras"][name]
    feed = Feed(name)
    _feeds.append(feed)
    if cam.get("source") == "v4l2":
        from usb_camera import UsbCamera
        if not cam.get("device"):
            raise SystemExit("%s has no device set. Run tools/find_cameras.py --assign first." % name)
        c = dict(cam)
        if max_fps is not None:
            c["max_process_fps"] = max_fps
        th = UsbCamera(name, c, lambda _n, t, jpeg, bgr: feed.put(t, bgr, None if jpeg is None else bytes(jpeg)),
                       log=lambda m: print("  " + m))
        th.start()
        feed.closers.append(th.close)
        feed.thread = th
    else:
        from sensor_msgs.msg import CompressedImage
        node = ros_node()
        period = 0.0 if not max_fps else 1.0 / max_fps
        state = {"last": 0.0}

        def on_msg(msg):
            now = time.time()
            if now - state["last"] < period * 0.9:
                return
            state["last"] = now
            data = np.frombuffer(msg.data, dtype=np.uint8)
            bgr = cv2.imdecode(data, cv2.IMREAD_COLOR)
            if bgr is None:
                return
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            feed.put(t if abs(now - t) < 2.0 else now, bgr, data.tobytes())

        node.create_subscription(CompressedImage, cam.get("topic", "/camera/image_raw/compressed"), on_msg, 1)
    return feed


def wait_first(feed: Feed, timeout: float = 10.0):
    item = feed.get(timeout)
    if item is None:
        if hasattr(feed, "thread"):
            hint = feed.thread.problem or "It opened but sent no pictures: run  python3 tools/find_cameras.py"
        else:
            hint = ("Is the robot stack running (the fisheye comes from the course camera node)? "
                    "Did you  source robot/env.sh  in this terminal?")
        raise SystemExit("\nNo frames from %s after %.0f s.\n  %s" % (feed.name, timeout, hint))
    return item
