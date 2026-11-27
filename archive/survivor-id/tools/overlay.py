"""One picture of what survivor identification sees: the three camera views side by side with
skeletons, who is pinned, and the state line. Used by the node's debug image and by replay.py."""
from __future__ import annotations

import cv2
import numpy as np

import _common  # noqa: F401
import draw


def mosaic(core, now: float, height: int = 360, note: str = ""):
    """core: SurvivorPerception. Returns a BGR picture, or None if no view has been looked at yet."""
    tiles = []
    with core.lock:
        views = dict(core.last_views)
        survivor = core.tracker.survivor
        sid = None if survivor is None else survivor.id
        phase = core.phase(now)
        target = core.target(now)
        look = core.look
    thr = float(core.cfg["gesture"]["kpt_conf"])
    for cam in core.cfg["cameras"]:
        item = views.get(cam)
        if item is None:
            continue
        image, people = item
        canvas = image.copy()
        for track_id, obs in people:
            draw.person(canvas, obs, obs.gesture, label="#%d" % track_id, survivor=(track_id == sid), conf_thr=thr)
        scale = height / canvas.shape[0]
        canvas = cv2.resize(canvas, (int(round(canvas.shape[1] * scale)), height))
        cv2.putText(canvas, cam, (6, height - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.5, draw.WHITE, 1, cv2.LINE_AA)
        tiles.append(canvas)
    if not tiles:
        return None
    out = np.hstack(tiles)
    text = phase
    if target is not None:
        text += "  survivor #%d at %+.0f deg, %.1f m%s" % (
            target["id"], target["bearing_deg"], target["range_m"],
            "" if target["visible"] else " (not seen for %.1f s)" % target["since_seen_s"])
    if look is not None:
        text += "  wants to turn and look at #%d" % look["id"]
    draw.banner(out, (text + "  " + note).strip())
    return out
