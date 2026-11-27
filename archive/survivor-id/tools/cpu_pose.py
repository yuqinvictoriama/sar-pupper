"""The same pose model on a laptop CPU, for trying things without the robot.

    pip install ultralytics          (on the laptop; NOT on the robot)

It runs the original PyTorch YOLOv8-pose and takes the nine raw head tensors, the same ones
the Hailo-compiled model returns, then decodes them with survivor/pose_decode.py. So what is
tested here is the decoder the robot uses, not Ultralytics' own.

What it cannot tell you: the compiled model is quantised to 8 bits and is a little less sure
of itself. Thresholds chosen here are a starting point; check them on the robot (DEPLOY.md, stage 1).
"""
from __future__ import annotations

import numpy as np

import _common  # noqa: F401
from survivor import pose_decode


class TorchYoloPose:
    def __init__(self, weights: str = "yolov8s-pose.pt", score_threshold: float = 0.35, iou_threshold: float = 0.6,
                 max_people: int = 8):
        import torch
        from ultralytics import YOLO
        self.torch = torch
        self.model = YOLO(weights).model.eval()
        self.kw = dict(score_threshold=score_threshold, iou_threshold=iou_threshold, max_people=max_people)
        head = self.model.model[-1]
        self._raw: dict = {}
        for role, branches in (("box", head.cv2), ("score", head.cv3), ("kpt", head.cv4)):
            for i, branch in enumerate(branches):
                branch.register_forward_hook(lambda _m, _in, out, key=(role, i): self._raw.__setitem__(key, out))

    def raw(self, image_bgr: np.ndarray):
        """(nine head tensors as grid x grid x channels, scale, pad)"""
        canvas, scale, pad = pose_decode.letterbox(image_bgr)
        x = self.torch.from_numpy(np.ascontiguousarray(canvas[:, :, ::-1])).permute(2, 0, 1).float()[None] / 255.0
        self._raw.clear()
        with self.torch.no_grad():
            self.model(x)
        return [t[0].permute(1, 2, 0).cpu().numpy() for t in self._raw.values()], scale, pad

    def __call__(self, image_bgr: np.ndarray) -> list:
        outputs, scale, pad = self.raw(image_bgr)
        return pose_decode.decode(outputs, scale, pad, image_bgr.shape, **self.kw)

    def close(self) -> None:
        pass
