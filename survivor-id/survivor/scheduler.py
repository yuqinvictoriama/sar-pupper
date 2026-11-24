"""Spending the pose model's time (plan, Pipeline steps 2 and 5).

Every cycle one camera gets a "wide" pass (cameras in rotation), and whatever budget is left
goes to "zoom" passes on individual people, most important first:

    the locked survivor -> people whose arms may be up -> new people -> everyone else
"""
from __future__ import annotations

from dataclasses import dataclass, field


@dataclass
class Plan:
    wide: str | None = None
    zoom: list = field(default_factory=list)       # tracks


class Scheduler:
    def __init__(self, cfg: dict, cameras: list):
        b = cfg["budget"]
        self.cameras = list(cameras)
        self.per_cycle = float(b["inferences_per_s"]) / float(b["cycle_hz"])
        self.zoom_skip = float(b["zoom_skip_px"])
        self.credit = 0.0
        self.turn = 0
        self.wide_done: dict = {}          # camera -> stamp of the last frame given a wide pass

    def plan(self, now: float, tracks: list, survivor, frame_times: dict) -> Plan:
        """frame_times: camera -> stamp of its newest frame (missing = no frame yet)."""
        plan = Plan()
        self.credit = min(self.credit + self.per_cycle, 2.0 * self.per_cycle + 1.0)
        for i in range(len(self.cameras)):
            cam = self.cameras[(self.turn + i) % len(self.cameras)]
            stamp = frame_times.get(cam)
            if stamp is not None and stamp != self.wide_done.get(cam) and self.credit >= 1.0:
                plan.wide = cam
                self.wide_done[cam] = stamp
                self.turn = (self.turn + i + 1) % len(self.cameras)
                self.credit -= 1.0
                break

        def rank(tr):
            if tr is survivor:
                return 0
            if now - tr.t_maybe <= 1.0 or now - tr.t_up <= 1.0:
                return 1
            return 2 if tr.hits < 5 else 3

        cands = []
        for tr in tracks:
            cam = tr.best_cam(now)
            obs = tr.obs_by_cam.get(cam)
            stamp = frame_times.get(cam)
            if obs is None or stamp is None or now - tr.t_last > 0.8:
                continue                                   # not in view right now
            if tr.wide_height.get(cam, 0.0) >= self.zoom_skip:
                continue                                   # already big enough in the wide pass
            if stamp <= tr.t_zoom:
                continue                                   # this frame has been zoomed on already
            cands.append((rank(tr), tr.t_zoom, tr.id, tr))
        for _rank, _tz, _id, tr in sorted(cands, key=lambda c: c[:3]):
            if self.credit < 1.0:
                break
            plan.zoom.append(tr)
            self.credit -= 1.0
        return plan
