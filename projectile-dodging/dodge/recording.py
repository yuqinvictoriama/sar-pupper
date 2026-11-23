"""Recordings: camera frames (as the JPEGs the cameras delivered) plus IMU, with timestamps.

A recording is a folder:

    meta.json          what was recorded, and the configuration at the time
    <camera>.bin       the frames, one JPEG after another
    <camera>.idx       one line per frame: time, byte offset, length
    imu.csv            time, quaternion (w x y z), gyro (x y z), accel (x y z)
    events.jsonl       dodge events, one JSON object per line

tools/record.py writes them, the dodge node dumps one around every dodge, and
tools/replay.py runs the whole dodge core on them offline.
"""
from __future__ import annotations

import json
import os
import threading
from collections import deque


class RecordingWriter:
    def __init__(self, path: str, meta: dict | None = None):
        self.path = path
        os.makedirs(path, exist_ok=True)
        self.lock = threading.Lock()
        self.bins: dict = {}
        self.idxs: dict = {}
        self.offsets: dict = {}
        self.counts: dict = {}
        self.imu = open(os.path.join(path, "imu.csv"), "w")
        self.imu.write("t,qw,qx,qy,qz,gx,gy,gz,ax,ay,az\n")
        self.events = open(os.path.join(path, "events.jsonl"), "w")
        with open(os.path.join(path, "meta.json"), "w") as f:
            json.dump(meta or {}, f, indent=1, default=str)

    def add_frame(self, cam: str, t: float, jpeg: bytes) -> None:
        with self.lock:
            if cam not in self.bins:
                self.bins[cam] = open(os.path.join(self.path, cam + ".bin"), "wb")
                self.idxs[cam] = open(os.path.join(self.path, cam + ".idx"), "w")
                self.offsets[cam] = 0
                self.counts[cam] = 0
            self.bins[cam].write(jpeg)
            self.idxs[cam].write("%.6f,%d,%d\n" % (t, self.offsets[cam], len(jpeg)))
            self.offsets[cam] += len(jpeg)
            self.counts[cam] += 1

    def add_imu(self, t: float, quat_wxyz, gyro, accel=(0.0, 0.0, 0.0)) -> None:
        with self.lock:
            self.imu.write("%.6f,%s\n" % (t, ",".join("%.6f" % float(v) for v in (*quat_wxyz, *gyro, *accel))))

    def add_event(self, event: dict) -> None:
        with self.lock:
            self.events.write(json.dumps(event, default=str) + "\n")

    def close(self) -> None:
        with self.lock:
            for f in list(self.bins.values()) + list(self.idxs.values()) + [self.imu, self.events]:
                f.close()


class RecordingReader:
    def __init__(self, path: str):
        self.path = path
        meta_path = os.path.join(path, "meta.json")
        self.meta = {}
        if os.path.exists(meta_path):
            with open(meta_path) as f:
                self.meta = json.load(f)
        self.index: dict = {}
        for name in sorted(os.listdir(path)):
            if name.endswith(".idx"):
                rows = []
                with open(os.path.join(path, name)) as f:
                    for line in f:
                        parts = line.strip().split(",")
                        if len(parts) == 3:
                            rows.append((float(parts[0]), int(parts[1]), int(parts[2])))
                self.index[name[:-4]] = rows
        self._files: dict = {}

    @property
    def cameras(self) -> list:
        return list(self.index)

    def close(self) -> None:
        for f in self._files.values():
            f.close()
        self._files = {}

    def read(self, cam: str, offset: int, length: int) -> bytes:
        if cam not in self._files:
            self._files[cam] = open(os.path.join(self.path, cam + ".bin"), "rb")
        f = self._files[cam]
        f.seek(offset)
        return f.read(length)

    def frames(self, cameras=None):
        """All frames in time order: (t, camera, jpeg_bytes)."""
        items = []
        for cam, rows in self.index.items():
            if cameras and cam not in cameras:
                continue
            items += [(t, cam, off, ln) for t, off, ln in rows]
        items.sort()
        for t, cam, off, ln in items:
            yield t, cam, self.read(cam, off, ln)

    def imu(self) -> list:
        """[(t, (qw,qx,qy,qz), (gx,gy,gz), (ax,ay,az)), ...]"""
        out = []
        p = os.path.join(self.path, "imu.csv")
        if not os.path.exists(p):
            return out
        with open(p) as f:
            next(f, None)
            for line in f:
                v = [float(x) for x in line.strip().split(",") if x != ""]
                if len(v) >= 8:
                    acc = tuple(v[8:11]) if len(v) >= 11 else (0.0, 0.0, 0.0)
                    out.append((v[0], tuple(v[1:5]), tuple(v[5:8]), acc))
        return out

    def events(self) -> list:
        p = os.path.join(self.path, "events.jsonl")
        if not os.path.exists(p):
            return []
        with open(p) as f:
            return [json.loads(line) for line in f if line.strip()]


class RingRecorder:
    """Keeps the last few seconds of frames and IMU in memory; dump() writes a slice as a recording."""

    def __init__(self, seconds: float):
        self.seconds = float(seconds)
        self.lock = threading.Lock()
        self.frames: dict = {}
        self.imu: deque = deque()

    def add_frame(self, cam: str, t: float, jpeg) -> None:
        if jpeg is None:
            return
        with self.lock:
            q = self.frames.setdefault(cam, deque())
            q.append((t, bytes(jpeg)))
            while q and q[0][0] < t - self.seconds:
                q.popleft()

    def add_imu(self, t: float, quat_wxyz, gyro, accel=(0.0, 0.0, 0.0)) -> None:
        with self.lock:
            self.imu.append((t, tuple(quat_wxyz), tuple(gyro), tuple(accel)))
            while self.imu and self.imu[0][0] < t - self.seconds:
                self.imu.popleft()

    def dump(self, path: str, meta: dict | None = None, events=()) -> str:
        with self.lock:
            frames = {cam: list(q) for cam, q in self.frames.items()}
            imu = list(self.imu)
        w = RecordingWriter(path, meta)
        for cam, items in frames.items():
            for t, jpeg in items:
                w.add_frame(cam, t, jpeg)
        for t, q, g, a in imu:
            w.add_imu(t, q, g, a)
        for e in events:
            w.add_event(e)
        w.close()
        return path
