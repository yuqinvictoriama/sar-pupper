"""Each camera's capture and ball search, in a process of its own.

Why processes: capture, JPEG decoding and the ball search for three cameras used
to run as three threads of the dodge node. Python lets only one thread of a
process run Python code at a time, so the cameras, the IMU callback and the
command timer queued behind each other; on the Pupper (2026-11-26) that came to
3-8 pictures a second per camera and pictures 80 ms and more old when looked at.
Here every camera gets a process (and so a processor core) to itself and sends
the node only what it found: a few numbers per picture, plus the camera's own
JPEG for the throw recorder.

    farm = CameraFarm(cfg, ["fisheye", "elp_left", "elp_right"], on_result, log)
    farm.start()
    ...  on_result(result, jpeg) is called from a thread per camera
    farm.set_hint(core.track_hint())     # where the tracked ball is, so cameras look there first
    farm.stop()

CameraRunner is one camera's capture + search; it is what runs inside each
process, and with processes=False it runs as threads of the caller instead
(node.camera_processes: false in the config; the same code, for comparison and
for machines where starting processes is a problem).

Camera sources (cameras.<name>.source in the config):
    v4l2             a USB camera, opened directly (robot/usb_camera.py)
    ros_compressed   a sensor_msgs/CompressedImage topic (the fisheye, from the course camera node)
    recording        frames of a saved recording, played back in real time (tests and benchmarks)
"""
from __future__ import annotations

import multiprocessing
import os
import signal
import sys
import threading
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for _p in (ROOT, HERE):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from dodge.pipeline import FramePipeline, load_mask  # noqa: E402
from dodge.recording import RecordingReader  # noqa: E402


class CameraRunner:
    """Capture and ball search for one camera. Calls emit(result, jpeg) for every picture it looks at."""

    def __init__(self, name: str, cfg: dict, emit, log=print, keep_jpeg: bool = True, ros_node=None):
        self.name = name
        self.cfg = cfg
        self.cam = cfg["cameras"][name]
        self.emit = emit
        self.log = log
        self.keep_jpeg = keep_jpeg
        self.pipeline = FramePipeline(name, cfg, load_mask(self.cam, lambda m: log("%s: %s" % (name, m))))
        self.max_fps = float(self.cam.get("max_process_fps", 0) or 0)
        self.running = False
        self.usb = None
        self.ros_node = ros_node
        self._own_ros = False
        self._msg = None
        self._msg_event = threading.Event()
        self._threads: list = []
        self.problem = ""
        self.fps_in = 0.0

    # ------------------------------------------------------------------ control
    def start(self) -> None:
        self.running = True
        src = self.cam.get("source")
        if src == "v4l2":
            if not self.cam.get("device"):
                self.problem = "no device set"
                self.log("%s has no device set: run tools/find_cameras.py and put the path in config/calib. "
                         "Skipping it." % self.name)
                return
            from usb_camera import UsbCamera
            self.usb = UsbCamera(self.name, self.cam, self._on_usb, log=self.log)
            self.usb.start()
        elif src == "ros_compressed":
            self._start_ros()
        elif src == "recording":
            self._thread(self._play_recording, "play")
        else:
            self.problem = "unknown source %r" % (src,)
            self.log("%s: %s" % (self.name, self.problem))

    def stop(self) -> None:
        self.running = False
        self._msg_event.set()
        if self.usb is not None:
            self.usb.stop()
            self.usb.close(2.0)       # released before Python exits; exiting mid-capture aborts the process
        for th in self._threads:
            th.join(1.0)
        if self._own_ros:
            try:
                import rclpy
                self.ros_node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()
            except Exception:
                pass

    def set_hint(self, hint) -> None:
        self.pipeline.set_hint(hint)

    def status(self) -> dict:
        if self.usb is not None:
            return {"brightness": round(self.usb.brightness, 0), "problem": self.usb.problem,
                    "fps_in": round(self.usb.fps_in, 1)}
        return {"problem": self.problem, "fps_in": round(self.fps_in, 1)}

    def _thread(self, target, label: str) -> None:
        th = threading.Thread(target=target, daemon=True, name="%s-%s" % (self.name, label))
        th.start()
        self._threads.append(th)

    # ------------------------------------------------------------------ USB camera
    def _on_usb(self, _name: str, t_capture: float, jpeg, bgr) -> None:
        result = self.pipeline.process(t_capture, bgr, self.usb.decode_ms)
        self.emit(result, jpeg if self.keep_jpeg else None)

    # ------------------------------------------------------------------ ROS compressed-image topic
    def _start_ros(self) -> None:
        import rclpy
        from sensor_msgs.msg import CompressedImage
        if self.ros_node is None:                 # a process of our own: it needs its own small ROS node
            from rclpy.signals import SignalHandlerOptions
            rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
            self.ros_node = rclpy.create_node("dodge_camera_" + self.name)
            self._own_ros = True
            self._thread(self._spin, "ros")
        self.ros_node.create_subscription(CompressedImage, self.cam.get("topic", "/camera/image_raw/compressed"),
                                          self._on_msg, 1)
        self._thread(self._ros_work, "work")

    def _spin(self) -> None:
        import rclpy
        try:
            rclpy.spin(self.ros_node)
        except Exception as exc:
            if self.running:
                self.problem = "ROS stopped: %r" % (exc,)
                self.log("%s: %s" % (self.name, self.problem))

    def _on_msg(self, msg) -> None:
        self._msg = (msg, time.time())            # keep only the newest; decoding happens in the work thread
        self._msg_event.set()

    def _ros_work(self) -> None:
        last_t, n_in, window = 0.0, 0, time.monotonic()
        while self.running:
            if not self._msg_event.wait(timeout=0.5):
                continue
            self._msg_event.clear()
            item, self._msg = self._msg, None
            if item is None:
                continue
            msg, t_rx = item
            n_in += 1
            now_mono = time.monotonic()
            if now_mono - window >= 1.0:
                self.fps_in, n_in, window = n_in / (now_mono - window), 0, now_mono
            try:
                stamp = msg.header.stamp
                t = float(stamp.sec) + float(stamp.nanosec) * 1e-9
                if abs(t_rx - t) > 2.0:           # stamp missing or on another clock
                    t = t_rx
                if self.max_fps > 0 and (t - last_t) < 0.9 / self.max_fps:
                    continue
                last_t = t
                t0 = time.monotonic()
                data = np.frombuffer(msg.data, dtype=np.uint8)
                bgr = cv2.imdecode(data, cv2.IMREAD_COLOR)
                if bgr is None:
                    continue
                result = self.pipeline.process(t, bgr, (time.monotonic() - t0) * 1000.0)
                self.emit(result, data if self.keep_jpeg else None)
            except Exception as exc:
                self.problem = "frame processing failed: %r" % (exc,)
                self.log("%s: %s" % (self.name, self.problem))
                time.sleep(0.5)

    # ------------------------------------------------------------------ a saved recording, in real time
    def _play_recording(self) -> None:
        """Plays one camera of a recording at its own pace, stamped as if captured now.

        cameras.<name>: {source: recording, path: <folder>, camera: <name in the recording>, loop: false,
        speed: 1.0 (0 = as fast as it can be processed)}
        """
        try:
            reader = RecordingReader(self.cam["path"])
            rows = reader.index[self.cam.get("camera", self.name)]
        except Exception as exc:
            self.problem = "cannot read the recording: %r" % (exc,)
            self.log("%s: %s" % (self.name, self.problem))
            return
        src_cam = self.cam.get("camera", self.name)
        speed = float(self.cam.get("speed", 1.0))
        while self.running and rows:
            t_first, wall_first = rows[0][0], time.time()
            for t_rec, off, length in rows:
                if not self.running:
                    break
                due = wall_first + (t_rec - t_first) / speed if speed > 0 else 0.0
                wait = due - time.time()
                if wait > 0:
                    time.sleep(wait)
                elif speed > 0 and wait < -0.5 / max(self.max_fps, 30.0):
                    continue                      # behind: let this one go, as a live camera would
                t_capture = time.time()
                data = np.frombuffer(reader.read(src_cam, off, length), dtype=np.uint8)
                t0 = time.monotonic()
                bgr = cv2.imdecode(data, cv2.IMREAD_COLOR)
                if bgr is None:
                    continue
                result = self.pipeline.process(t_capture, bgr, (time.monotonic() - t0) * 1000.0)
                self.emit(result, data if self.keep_jpeg else None)
            if not self.cam.get("loop", False):
                break
        reader.close()
        self.problem = "recording finished"


# ---------------------------------------------------------------------- the process

def worker_main(name: str, cfg: dict, conn, keep_jpeg: bool, opencv_threads: int) -> None:
    """Entry point of one camera's process. Talks to the node over conn:

    to the node:   ("f", FrameResult, jpeg bytes or None)   one per picture looked at
                   ("s", status dict)                        twice a second
                   ("log", text)
    from the node: ("hint", track hint or None), ("stop",)
    """
    signal.signal(signal.SIGINT, signal.SIG_IGN)          # Ctrl+C reaches the node; the node stops us
    stop = threading.Event()
    signal.signal(signal.SIGTERM, lambda *_a: stop.set())
    cv2.setNumThreads(int(opencv_threads))
    send_lock = threading.Lock()

    def send(item) -> None:
        try:
            with send_lock:
                conn.send(item)
        except (OSError, ValueError):                      # the node is gone
            stop.set()

    def emit(result, jpeg) -> None:
        send(("f", result, None if jpeg is None else bytes(jpeg)))

    try:
        runner = CameraRunner(name, cfg, emit, log=lambda m: send(("log", str(m))), keep_jpeg=keep_jpeg)
        runner.start()
    except Exception as exc:
        send(("log", "%s: could not start: %r" % (name, exc)))
        time.sleep(1.0)
        return
    last_status = 0.0
    while not stop.is_set():
        try:
            if conn.poll(0.1):
                msg = conn.recv()
                if msg[0] == "hint":
                    runner.set_hint(msg[1])
                elif msg[0] == "stop":
                    break
        except (EOFError, OSError):
            break
        now = time.time()
        if now - last_status >= 0.5:
            last_status = now
            send(("s", runner.status()))
    runner.stop()


class CameraFarm:
    """All the cameras, each in its own process (or, with processes=False, as threads here)."""

    def __init__(self, cfg: dict, names, on_result, log=print, keep_jpeg: bool = True, processes: bool = True,
                 ros_node=None, opencv_threads: int = 1):
        self.cfg = cfg
        self.names = list(names)
        self.on_result = on_result
        self.log = log
        self.keep_jpeg = keep_jpeg
        self.processes = processes
        self.ros_node = ros_node
        self.opencv_threads = opencv_threads
        self.running = False
        self._status = {name: {} for name in self.names}
        self._runners: dict = {}
        self._procs: dict = {}
        self._conns: dict = {}
        self._send_lock = threading.Lock()
        self._hinted = False

    def start(self) -> None:
        self.running = True
        for name in self.names:
            if self.processes:
                self._spawn(name)
            else:
                runner = CameraRunner(name, self.cfg, self.on_result, log=self.log, keep_jpeg=self.keep_jpeg,
                                      ros_node=self.ros_node)
                self._runners[name] = runner
                runner.start()

    def _spawn(self, name: str) -> None:
        ctx = multiprocessing.get_context("spawn")        # a clean process: nothing of ROS or OpenCV is shared
        parent, child = ctx.Pipe()
        cfg = dict(self.cfg, cameras={name: self.cfg["cameras"][name]})
        proc = ctx.Process(target=worker_main, args=(name, cfg, child, self.keep_jpeg, self.opencv_threads),
                           daemon=True, name="dodge-cam-" + name)
        proc.start()
        child.close()
        self._procs[name], self._conns[name] = proc, parent
        threading.Thread(target=self._read, args=(name, parent, proc), daemon=True, name="cam-" + name).start()

    def _read(self, name: str, conn, proc) -> None:
        while self.running:
            try:
                msg = conn.recv()
            except (EOFError, OSError):
                break
            try:
                if msg[0] == "f":
                    self.on_result(msg[1], msg[2])
                elif msg[0] == "s":
                    self._status[name] = msg[1]
                elif msg[0] == "log":
                    self.log(msg[1])
            except Exception as exc:
                self.log("%s: handling a camera result failed: %r" % (name, exc))
        if not self.running:
            return
        self._status[name] = {"problem": "its process stopped (exit code %s); restarting it" % proc.exitcode}
        self.log("%s: the camera process stopped (exit code %s); restarting it in 2 s" % (name, proc.exitcode))
        time.sleep(2.0)
        if self.running:
            self._spawn(name)

    def set_hint(self, hint) -> None:
        """Tell every camera where the tracked ball is (None: nothing tracked)."""
        if hint is None and not self._hinted:
            return                                        # they already know
        self._hinted = hint is not None
        if not self.processes:
            for runner in self._runners.values():
                runner.set_hint(hint)
            return
        with self._send_lock:
            for conn in list(self._conns.values()):
                try:
                    conn.send(("hint", hint))
                except (OSError, ValueError):
                    pass

    def status(self, name: str) -> dict:
        runner = self._runners.get(name)
        return runner.status() if runner is not None else dict(self._status.get(name) or {})

    def stop(self) -> None:
        self.running = False
        for runner in self._runners.values():
            runner.stop()
        with self._send_lock:
            for conn in self._conns.values():
                try:
                    conn.send(("stop",))
                except (OSError, ValueError):
                    pass
        for proc in self._procs.values():
            proc.join(3.0)
            if proc.is_alive():
                proc.terminate()
        for conn in self._conns.values():
            try:
                conn.close()
            except OSError:
                pass
