"""Survivor identification, tied together: frames in, "who is the survivor and where" out.

    p = SurvivorPerception(cfg)
    p.set_imu(t, quat_wxyz)               # as IMU samples arrive
    p.set_cmd_vel(t, vx, vy, wz)          # the velocity actually sent to the legs (/cmd_vel)
    p.set_frame("fisheye", t, bgr)        # newest frame per camera
    p.step(now, detector)                 # one cycle: a wide pass and some zoom passes
    p.status(now), p.people(now), p.pop_events()

`detector(image_bgr, view)` returns [PoseDet] in the image's pixels. On the robot it is the
Hailo model (robot/hailo_pose.py); in the tests it is a stand-in that draws skeletons.
"""
from __future__ import annotations

import math
import threading

import numpy as np

from . import appearance
from .cameras import Rig, az_el, level_from_base, quat_to_rot, wrap, yaw_of
from .gesture import evaluate
from .person import angle, observe
from .scheduler import Scheduler
from .tracker import DeadReckoner, PeopleTracker, Track

SCAN, CANDIDATE, LOCKED, LOST = "SCAN", "CANDIDATE", "LOCKED", "LOST"


class SurvivorPerception:
    def __init__(self, cfg: dict, enabled: bool = True):
        self.cfg = cfg
        self.rig = Rig(cfg)
        self.dr = DeadReckoner(float(cfg["tracker"]["cmd_tracking_factor"]))
        self.tracker = PeopleTracker(cfg, self.dr)
        self.scheduler = Scheduler(cfg, list(cfg["cameras"]))
        self.enabled = bool(enabled)
        self.lock = threading.RLock()
        self.frames: dict = {}                # camera -> (stamp, bgr)
        self.R_wb = np.eye(3)
        self.events: list = []
        self.queue: list = []                 # ids of later signallers, in order (plan decision S1)
        self.look: dict | None = None
        self.was_visible = False
        self.stats = {"wide": 0, "zoom": 0, "t0": None, "infer_hz": 0.0}
        self.last_views: dict = {}            # camera -> (view image, [(track id, obs)]) for the debug picture

    # ------------------------------------------------------------------ inputs
    def set_imu(self, t: float, quat_wxyz) -> None:
        with self.lock:
            self.R_wb = quat_to_rot(*quat_wxyz)
            self.dr.set_imu_yaw(t, yaw_of(self.R_wb))

    def set_cmd_vel(self, t: float, vx: float, vy: float, wz: float) -> None:
        with self.lock:
            self.dr.set_cmd(t, vx, vy, wz)

    def set_frame(self, cam: str, t: float, bgr) -> None:
        """The newest frame from a camera. `bgr` may be a function returning the picture (or None),
        so a JPEG is only decoded if this cycle looks at that camera."""
        if cam in self.cfg["cameras"]:
            self.frames[cam] = (float(t), bgr)

    def _frame(self, cam: str):
        """(stamp, picture) or None."""
        item = self.frames.get(cam)
        if item is None:
            return None
        stamp, bgr = item
        if callable(bgr):
            bgr = bgr()
            if self.frames.get(cam) is item:
                if bgr is None:
                    self.frames.pop(cam, None)
                else:
                    self.frames[cam] = (stamp, bgr)
        return None if bgr is None else (stamp, bgr)

    def enable(self, on: bool) -> None:
        with self.lock:
            self.enabled = bool(on)

    def cancel(self, now: float) -> None:
        """Drop the pin (and the queue). Watching carries on."""
        with self.lock:
            if self.tracker.survivor is not None:
                self._event(now, "cancelled", id=self.tracker.survivor.id)
            self.tracker.survivor = None
            self.queue = []
            self.was_visible = False
            for tr in self.tracker.tracks:
                tr.filter.reset()

    # ------------------------------------------------------------------ one cycle
    def step(self, now: float, detector) -> None:
        with self.lock:
            if not self.enabled:
                return
            self.dr.advance(now)
            stamps = {cam: item[0] for cam, item in self.frames.items()}
            plan = self.scheduler.plan(now, self.tracker.tracks, self.tracker.survivor, stamps)
            R_lb = level_from_base(self.R_wb)
        if plan.wide is not None:
            self._wide(plan.wide, now, detector, R_lb)
        for track in plan.zoom:
            self._zoom(track, now, detector, R_lb)
        with self.lock:
            self.tracker.prune(now)
            self._decide(now)
            n = self.stats["wide"] + self.stats["zoom"]
            if self.stats["t0"] is None:
                self.stats["t0"], self.stats["n0"] = now, n
            elif now - self.stats["t0"] >= 2.0:
                self.stats["infer_hz"] = (n - self.stats["n0"]) / (now - self.stats["t0"])
                self.stats["t0"], self.stats["n0"] = now, n

    def _describe(self, image, det):
        t = self.cfg["tracker"]
        return appearance.describe(image, det.kpts, float(self.cfg["gesture"]["kpt_conf"]), tuple(t["appearance_bins"]))

    def _wide(self, cam: str, now: float, detector, R_lb) -> None:
        item = self._frame(cam)
        if item is None:
            return
        stamp, bgr = item
        view = self.rig.wide_view(cam, (bgr.shape[1], bgr.shape[0]))
        image = view.render(bgr)
        dets = detector(image, view)
        self.stats["wide"] += 1
        obs = [observe(d, view, R_lb, stamp, self.cfg) for d in dets]
        descs = [self._describe(image, d) for d in dets]
        skip = float(self.cfg["budget"]["zoom_skip_px"])
        with self.lock:
            pairs = self.tracker.update(obs, descs, now)
            for track, o in pairs:
                self._judge(track, o, stamp, now, counts=o.height_px >= skip)
            self.last_views[cam] = (image, [(tr.id, o) for tr, o in pairs])

    def _zoom(self, track: Track, now: float, detector, R_lb) -> None:
        cam = track.best_cam(now)
        item, last = self._frame(cam), track.obs_by_cam.get(cam)
        if item is None or last is None:
            return
        stamp, bgr = item
        b = self.cfg["budget"]
        top, bottom = last.box_rays[0], last.box_rays[1]
        height = angle(top, bottom)
        mid = top + bottom
        az, el = az_el(mid / np.linalg.norm(mid))
        el = float(el[0]) + 0.5 * (float(b["patch_margin_top"]) - float(b["patch_margin"])) * height
        axis_level = np.array([math.cos(el) * math.cos(az[0]), math.cos(el) * math.sin(az[0]), math.sin(el)])
        fov = max(math.degrees(height) * (1.0 + float(b["patch_margin"]) + float(b["patch_margin_top"])),
                  float(b["patch_px"]) / float(b["patch_max_px_per_deg"]))
        view = self.rig.patch_view(cam, (bgr.shape[1], bgr.shape[0]), R_lb.T @ axis_level, fov, int(b["patch_px"]), R_lb)
        image = view.render(bgr)
        dets = detector(image, view)
        self.stats["zoom"] += 1
        centre = np.array([view.c, view.c])
        seen = []
        for d in dets:
            if float(np.linalg.norm(0.5 * (d.box[:2] + d.box[2:]) - centre)) <= 0.35 * view.width:
                seen.append((observe(d, view, R_lb, stamp, self.cfg, zoom=True), self._describe(image, d)))
        with self.lock:
            track.t_zoom = stamp
            if track not in self.tracker.tracks:
                return
            # Two people can be in one patch (one walking past the other): take the one that is
            # this track's, not just whoever is nearest the middle.
            best = self.tracker.pick(track, seen, now)
            if best is None:
                track.filter.add(stamp, False)          # looked straight at them and found nobody: not the signal
                return
            o, desc = best
            self.tracker.update_track(track, o, desc, now)
            self._judge(track, o, stamp, now, counts=True)

    def _judge(self, track: Track, obs, stamp: float, now: float, counts: bool) -> None:
        res = evaluate(obs, self.cfg)
        obs.gesture = res
        if res.maybe:
            track.t_maybe = now
        # A camera that sees only part of someone has no say while another camera sees all of them.
        if obs.cam != track.best_cam(now):
            return
        track.gesture = res
        if counts:
            track.filter.add(stamp, res.up)
            if res.up:
                track.t_up = now

    # ------------------------------------------------------------------ decisions
    def _event(self, t: float, name: str, **data) -> None:
        self.events.append({"event": name, "t": round(t, 3), **data})

    def _where(self, track: Track) -> dict:
        b, r = self.tracker.polar(track)
        return {"id": track.id, "bearing_deg": round(math.degrees(b), 1), "range_m": round(r, 2)}

    def _decide(self, now: float) -> None:
        tk = self.tracker
        surv = tk.survivor
        lost_after = float(self.cfg["lock"]["lost_after_s"])
        surv_visible = surv is not None and tk.visible(surv, now, lost_after)
        for tr in tk.tracks:
            stamp = self.frames.get(tr.cam, (now,))[0]
            if tr is surv or not tk.confirmed(tr):
                continue
            if now - tr.t_up <= 0.5 and not tr.announced:
                tr.announced = True
                self._event(now, "candidate", **self._where(tr))
            if tr.filter.confirmed(stamp) and now - tr.t_up <= 0.5:
                if surv is None or not surv_visible:
                    tk.survivor = surv = tr               # first signal wins; signalling again re-locks a lost pin
                    surv_visible = True
                    self.was_visible = True
                    if tr.id in self.queue:
                        self.queue.remove(tr.id)
                    self._event(now, "locked", **self._where(tr))
                elif tr.id not in self.queue:
                    self.queue.append(tr.id)              # recorded, not chased (plan decision S1)
        if surv is not None:
            if self.was_visible and not surv_visible:
                self._event(now, "lost", **self._where(surv))
            elif not self.was_visible and surv_visible:
                self._event(now, "reacquired", **self._where(surv))
            self.was_visible = surv_visible
        # Turn-to-look: someone in a rear camera is cut off at the top (too close for their raised
        # hands to fit), or split by the seam between the two rear cameras straight behind, with an
        # elbow up or their shoulders out of the picture too. Only the front camera can see all of
        # them and say whether it is the signal.
        self.look = None
        if surv is None:
            for tr in tk.tracks:
                o = tr.obs_by_cam.get(tr.best_cam(now))
                if o is None or o.gesture is None or not (tk.confirmed(tr) and tk.visible(tr, now, 0.6) and self.rig.is_rear(o.cam)):
                    continue
                b, r = tk.polar(tr)
                at_seam = o.clipped_side and abs(wrap(b - math.pi)) <= math.radians(float(self.cfg["lock"]["seam_deg"]))
                if not (o.clipped_top or at_seam) or not o.gesture.undecided:
                    continue
                if r <= float(self.cfg["lock"]["look_max_range_m"]) and (self.look is None or r < self.look["range_m"]):
                    self.look = {"id": tr.id, "bearing_deg": round(math.degrees(b), 1), "range_m": round(r, 2)}

    # ------------------------------------------------------------------ outputs
    def pop_events(self) -> list:
        with self.lock:
            out, self.events = self.events, []
            return out

    def phase(self, now: float) -> str:
        tk = self.tracker
        if tk.survivor is not None:
            return LOCKED if tk.visible(tk.survivor, now, float(self.cfg["lock"]["lost_after_s"])) else LOST
        if any(now - tr.t_up <= 1.0 and tk.confirmed(tr) for tr in tk.tracks):
            return CANDIDATE
        return SCAN

    def target(self, now: float) -> dict | None:
        """The survivor, relative to the robot now (dead-reckoned while out of view)."""
        tk = self.tracker
        tr = tk.survivor
        if tr is None:
            return None
        b, r = tk.polar(tr)
        fisheye_frac = 0.0
        for cam, o in tr.obs_by_cam.items():
            if not self.rig.is_rear(cam) and not o.zoom and now - tr.t_last <= 0.6 and tr.cam == cam:
                fisheye_frac = o.box_frac
        return {"id": tr.id, "bearing_deg": round(math.degrees(b), 1), "range_m": round(r, 2),
                "visible": tk.visible(tr, now, float(self.cfg["lock"]["lost_after_s"])),
                "since_seen_s": round(now - tr.t_last, 2), "sigma_m": round(tk.sigma(tr, now), 2),
                "box_frac": round(fisheye_frac, 2), "cam": tr.cam,
                "x": round(float(tr.xy[0]), 3), "y": round(float(tr.xy[1]), 3)}

    def status(self, now: float) -> dict:
        with self.lock:
            self.dr.advance(now)
            return {"t": round(now, 3), "enabled": self.enabled, "phase": self.phase(now), "target": self.target(now),
                    "look": self.look, "queued": list(self.queue), "people": len(self.tracker.tracks),
                    "tracks": [dict(self._where(tr), visible=self.tracker.visible(tr, now))
                               for tr in self.tracker.tracks if self.tracker.confirmed(tr) or tr is self.tracker.survivor],
                    "pose": {"x": round(self.dr.x, 3), "y": round(self.dr.y, 3), "yaw_deg": round(math.degrees(wrap(self.dr.yaw)), 1)},
                    "infer_hz": round(self.stats["infer_hz"], 1),
                    "cameras": {cam: round(now - item[0], 2) for cam, item in self.frames.items()}}

    def people(self, now: float) -> list:
        with self.lock:
            self.dr.advance(now)
            out = []
            for tr in self.tracker.tracks:
                if not self.tracker.confirmed(tr) and tr is not self.tracker.survivor:
                    continue
                b, r = self.tracker.polar(tr)
                stamp = self.frames.get(tr.cam, (now,))[0]
                out.append({"id": tr.id, "bearing_deg": round(math.degrees(b), 1), "range_m": round(r, 2),
                            "arms_up": round(tr.filter.fraction(stamp), 2), "survivor": tr is self.tracker.survivor,
                            "visible": self.tracker.visible(tr, now), "cam": tr.cam,
                            "height_frac": round(tr.obs.box_frac, 3)})
            return out
