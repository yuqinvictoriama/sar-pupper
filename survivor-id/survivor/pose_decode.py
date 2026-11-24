"""YOLOv8-pose head tensors -> people with 17 keypoints.

The compiled model (Hailo model zoo yolov8s_pose / yolov8m_pose, 640 x 640) hands back nine
tensors, three per grid (80, 40 and 20 cells across):

    64 channels   box: four sides x 16-bin distance distributions (DFL)
     1 channel    person score
    51 channels   17 keypoints x (x, y, confidence)

The same layout comes out of the original PyTorch model's head, which is how this decoder
is tested off the robot (tools/cpu_pose.py). Scores and keypoint confidences are accepted
either as probabilities or as raw logits: values outside 0..1 mean logits.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

INPUT_SIZE = 640
DFL_BINS = 16
NUM_KPTS = 17
KPT_NAMES = ("nose", "l_eye", "r_eye", "l_ear", "r_ear", "l_shoulder", "r_shoulder", "l_elbow", "r_elbow",
             "l_wrist", "r_wrist", "l_hip", "r_hip", "l_knee", "r_knee", "l_ankle", "r_ankle")
K = {name: i for i, name in enumerate(KPT_NAMES)}
SKELETON = ((5, 7), (7, 9), (6, 8), (8, 10), (5, 6), (5, 11), (6, 12), (11, 12), (11, 13), (13, 15), (12, 14),
            (14, 16), (0, 1), (0, 2), (1, 3), (2, 4))


@dataclass
class PoseDet:
    box: np.ndarray        # x1, y1, x2, y2 in the pixels of the image given to the detector
    score: float
    kpts: np.ndarray       # (17, 3): x, y, confidence


def letterbox(img: np.ndarray, size: int = INPUT_SIZE):
    """Resize keeping the aspect ratio and pad to a square. Returns (canvas, scale, (pad_x, pad_y))."""
    import cv2
    h, w = img.shape[:2]
    scale = min(size / w, size / h)
    nw, nh = int(round(w * scale)), int(round(h * scale))
    canvas = np.full((size, size, 3), 114, np.uint8)
    px, py = (size - nw) // 2, (size - nh) // 2
    canvas[py:py + nh, px:px + nw] = cv2.resize(img, (nw, nh), interpolation=cv2.INTER_LINEAR)
    return canvas, scale, (px, py)


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-np.clip(x, -50.0, 50.0)))


def _as_prob(x: np.ndarray) -> np.ndarray:
    return x if (x.size and x.min() >= 0.0 and x.max() <= 1.0) else _sigmoid(x)


def _softmax(x, axis=-1):
    x = x - x.max(axis=axis, keepdims=True)
    e = np.exp(x)
    return e / e.sum(axis=axis, keepdims=True)


def nms(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> list:
    if len(boxes) == 0:
        return []
    x1, y1, x2, y2 = boxes.T
    areas = np.maximum(0.0, x2 - x1) * np.maximum(0.0, y2 - y1)
    order = scores.argsort()[::-1]
    keep = []
    while order.size:
        i = order[0]
        keep.append(int(i))
        rest = order[1:]
        if not rest.size:
            break
        w = np.maximum(0.0, np.minimum(x2[i], x2[rest]) - np.maximum(x1[i], x1[rest]))
        h = np.maximum(0.0, np.minimum(y2[i], y2[rest]) - np.maximum(y1[i], y1[rest]))
        iou = w * h / np.maximum(areas[i] + areas[rest] - w * h, 1e-9)
        order = rest[iou <= iou_threshold]
    return keep


def group_heads(outputs) -> dict:
    """Sort the nine tensors by grid size and role, from their shapes alone: {grid: {"box", "score", "kpt"}}."""
    arrays = list(outputs.values()) if isinstance(outputs, dict) else list(outputs)
    heads: dict = {}
    for a in arrays:
        a = np.asarray(a)
        if a.ndim == 4 and a.shape[0] == 1:
            a = a[0]
        if a.ndim != 3:
            raise ValueError("unexpected pose output shape %r" % (a.shape,))
        role = {4 * DFL_BINS: "box", 1: "score", 3 * NUM_KPTS: "kpt"}.get(a.shape[2])
        if role is None or a.shape[0] != a.shape[1]:
            raise ValueError("unexpected pose output shape %r (want grid x grid x 64, 1 or 51)" % (a.shape,))
        heads.setdefault(a.shape[0], {})[role] = a
    for grid, h in heads.items():
        if set(h) != {"box", "score", "kpt"}:
            raise ValueError("grid %d is missing %s" % (grid, {"box", "score", "kpt"} - set(h)))
    return heads


def decode(outputs, scale: float = 1.0, pad=(0, 0), frame_shape=None, score_threshold: float = 0.35,
           iou_threshold: float = 0.6, max_people: int = 8, input_size: int = INPUT_SIZE) -> list:
    """Head tensors -> [PoseDet], in the pixels of the frame that was letterboxed."""
    boxes, scores, kpts = [], [], []
    bins = np.arange(DFL_BINS, dtype=np.float32)
    for grid, h in group_heads(outputs).items():
        stride = input_size / grid
        score = _as_prob(h["score"].reshape(-1).astype(np.float32))
        hits = np.nonzero(score >= score_threshold)[0]
        if not hits.size:
            continue
        gy, gx = np.divmod(hits, grid)
        cx, cy = (gx + 0.5) * stride, (gy + 0.5) * stride
        raw = h["box"].reshape(-1, 4, DFL_BINS)[hits].astype(np.float32)
        left, top, right, bottom = ((_softmax(raw, axis=2) * bins).sum(axis=2) * stride).T
        boxes.append(np.stack([cx - left, cy - top, cx + right, cy + bottom], axis=1))
        scores.append(score[hits])
        k = h["kpt"].reshape(-1, NUM_KPTS, 3)[hits].astype(np.float32)
        kx = (k[:, :, 0] * 2.0 + gx[:, None]) * stride
        ky = (k[:, :, 1] * 2.0 + gy[:, None]) * stride
        kpts.append(np.stack([kx, ky, _as_prob(h["kpt"].reshape(-1, NUM_KPTS, 3)[:, :, 2].astype(np.float32))[hits]], axis=2))
    if not boxes:
        return []
    boxes, scores, kpts = np.concatenate(boxes), np.concatenate(scores), np.concatenate(kpts)
    keep = nms(boxes, scores, iou_threshold)[:max_people]
    boxes, scores, kpts = boxes[keep], scores[keep], kpts[keep]
    px, py = pad
    boxes[:, [0, 2]] = (boxes[:, [0, 2]] - px) / scale
    boxes[:, [1, 3]] = (boxes[:, [1, 3]] - py) / scale
    kpts[:, :, 0] = (kpts[:, :, 0] - px) / scale
    kpts[:, :, 1] = (kpts[:, :, 1] - py) / scale
    if frame_shape is not None:
        h_, w_ = frame_shape[:2]
        boxes[:, [0, 2]] = np.clip(boxes[:, [0, 2]], 0, w_ - 1)
        boxes[:, [1, 3]] = np.clip(boxes[:, [1, 3]], 0, h_ - 1)
    return [PoseDet(boxes[i].astype(np.float64), float(scores[i]), kpts[i].astype(np.float64)) for i in range(len(keep))]
