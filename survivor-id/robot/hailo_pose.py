#!/usr/bin/env python3
"""YOLOv8-pose on the Hailo-8L: people with 17 body joints each.

    pose = HailoYoloPose("models/yolov8s_pose.hef")
    people = pose(image_bgr)          # [PoseDet(box, score, kpts)] in the image's own pixels
    pose.close()

The .hef comes from the Hailo model zoo (tools/download_model.sh), so the chip returns the
nine raw head tensors and survivor/pose_decode.py turns them into people.

The accelerator serves ONE program at a time. Stop the person follower, hailo_detection and
anything else using it first, or creating this object fails (start.sh passes follower:=false).

Quick check on a still picture:   python3 tools/try_photos.py some_photo.jpg
How fast it runs:                 python3 tools/bench_hailo.py
"""
from __future__ import annotations

import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from survivor import pose_decode  # noqa: E402


class HailoYoloPose:
    """Not thread-safe: call it from one thread."""

    def __init__(self, model_path: str, score_threshold: float = 0.35, iou_threshold: float = 0.6, max_people: int = 8):
        from hailo_platform import FormatType, HailoSchedulingAlgorithm, VDevice
        if not os.path.exists(model_path):
            raise FileNotFoundError("%s not found. Run tools/download_model.sh to fetch it." % model_path)
        self.kw = dict(score_threshold=score_threshold, iou_threshold=iou_threshold, max_people=max_people)
        params = VDevice.create_params()
        params.scheduling_algorithm = HailoSchedulingAlgorithm.ROUND_ROBIN
        self.target = VDevice(params)
        self.infer_model = self.target.create_infer_model(model_path)
        self.infer_model.set_batch_size(1)
        self.output_shapes = {}
        for name in self.infer_model.output_names:
            self.infer_model.output(name).set_format_type(FormatType.FLOAT32)      # let HailoRT undo the 8-bit scaling
            self.output_shapes[name] = tuple(self.infer_model.output(name).shape)
        in_shape = tuple(self.infer_model.input().shape)
        self.input_size = int(in_shape[0])
        if in_shape[0] != in_shape[1]:
            raise RuntimeError("%s: expected a square input, got %r" % (model_path, in_shape))
        # Fails here, with a clear message, if this is not a YOLOv8-pose model.
        pose_decode.group_heads([np.empty(s, np.float32) for s in self.output_shapes.values()])
        self._configured = self.infer_model.configure()
        self._bindings = self._configured.create_bindings(
            output_buffers={name: np.empty(shape, dtype=np.float32) for name, shape in self.output_shapes.items()})
        self.last_inference_ms = 0.0

    def close(self) -> None:
        """Release the accelerator so another program (or a restart) can have it."""
        configured, self._configured = self._configured, None
        if configured is not None:
            configured.__exit__(None, None, None)
            self.target.release()

    def __call__(self, image_bgr: np.ndarray) -> list:
        canvas, scale, pad = pose_decode.letterbox(image_bgr, self.input_size)
        rgb = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)                 # the model was trained on RGB
        t0 = time.time()
        self._bindings.input().set_buffer(np.ascontiguousarray(rgb))
        self._configured.run([self._bindings], 10000)
        self.last_inference_ms = (time.time() - t0) * 1000.0
        outputs = [self._bindings.output(name).get_buffer() for name in self.output_shapes]
        return pose_decode.decode(outputs, scale, pad, image_bgr.shape, input_size=self.input_size, **self.kw)
