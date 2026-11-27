"""Track the ball in 3D with a ballistic Kalman filter.

State: position and velocity of the ball relative to the robot, in a
gravity-aligned frame ("world": z up, origin at base_link, heading from the
IMU). Working in this frame means body pitch and roll while walking do not
look like ball motion, and gravity always points along -z.

Each measurement is a 3D point (lens position + range * ray) whose error is
small across the ray (bearing is accurate) and large along it (range comes from
the ball's apparent size). Measurements from different cameras therefore
combine correctly even though the lenses are 19 cm apart.

The quantities the plan talks about come straight out of the state:
azimuth, elevation, angular size, and their rates.
"""
from __future__ import annotations

import math

import numpy as np

G = 9.81


class Track:
    _next_id = 1

    def __init__(self, t: float, p: np.ndarray, P_pos: np.ndarray, speed_sigma: float, cam: str, good: bool = True,
                 ray: np.ndarray | None = None, origin: np.ndarray | None = None, range_m: float | None = None):
        self.id = Track._next_id
        Track._next_id += 1
        self.x = np.zeros(6)
        self.x[:3] = p
        self.P = np.zeros((6, 6))
        self.P[:3, :3] = P_pos
        self.P[3:, 3:] = np.eye(3) * speed_sigma ** 2
        self.t = t              # time the state refers to
        self.t0 = t
        self.t_last = t         # last time a measurement was accepted
        self.hits = 1
        self.confirmed = False
        self.flying = False     # seen moving like a thrown ball (set by the tracker, and it stays set)
        self.cams = {cam}
        self.last_cam = cam
        # recent raw measurements: (t, point in the world frame, camera, range trustworthy, ray from the lens, range, lens position)
        self.history = [self._look(t, p, cam, good, ray, origin, range_m)]
        self.p_first = np.array(p, dtype=float)          # where it was first seen

    # ---- convenience views of the state
    @property
    def p(self) -> np.ndarray:
        return self.x[:3]

    @property
    def v(self) -> np.ndarray:
        return self.x[3:]

    @property
    def age(self) -> float:
        return self.t_last - self.t0

    @property
    def range_m(self) -> float:
        return float(np.linalg.norm(self.x[:3]))

    @property
    def travel_m(self) -> float:
        """How far it has moved ACROSS the line of sight since it was first seen. Range is read from the
        blob's size and jitters by 10 cm or more from picture to picture, so movement along the line of
        sight proves nothing; bearing is good to a centimetre. A lob always moves across: it climbs."""
        d = self.x[:3] - self.p_first
        n = float(np.linalg.norm(self.p_first))
        if n > 1e-6:
            u = self.p_first / n
            d = d - float(d @ u) * u
        return float(np.linalg.norm(d))

    @property
    def closing_speed(self) -> float:
        """Speed toward the robot (m/s); positive = approaching."""
        r = self.range_m
        return float(-np.dot(self.x[:3], self.x[3:]) / r) if r > 1e-6 else 0.0

    @staticmethod
    def _look(t, z, cam, good, ray=None, origin=None, range_m=None):
        z = np.array(z, dtype=float)
        origin = np.zeros(3) if origin is None else np.array(origin, dtype=float)
        if ray is None:
            d = z - origin
            n = float(np.linalg.norm(d))
            ray = d / n if n > 1e-9 else np.array([1.0, 0.0, 0.0])
        if range_m is None:
            range_m = float(np.linalg.norm(z - origin))
        return (t, z, cam, bool(good), np.array(ray, dtype=float), float(range_m), origin)

    def remember(self, t: float, z: np.ndarray, cam: str = "", good: bool = True, window: float = 0.35,
                 ray: np.ndarray | None = None, origin: np.ndarray | None = None, range_m: float | None = None) -> None:
        """good: the range of this measurement can be trusted (the blob was not cut off by the picture's edge)."""
        self.history.append(self._look(t, z, cam, good, ray, origin, range_m))
        self.history = [h for h in self.history if h[0] >= t - window][-24:]

    def hit_course(self, min_looks: int = 3, min_span: float = 0.06, drop: float = 0.0, arrive=None):
        """How the ball's BEARING has moved in the one camera that has seen it most, or None if no camera
        has seen it min_looks times yet.

        A camera reads a ball's direction to a quarter of a degree, and its range, from how big it looks,
        to 10-15 % at best: at 2.5 m the ball is 6-8 pixels across in the fisheye, and one pixel more is
        "0.4 m nearer", 8 m/s over one picture. So how fast the ball is coming cannot be read from its
        size. It can be read from its direction. For a ball in free flight that will arrive at the
        robot, the tangent of its elevation rises at a steady g / (2 x its speed toward the robot), and
        its azimuth stays put. A still thing does not climb at all; a ball that will fall short sinks.

        drop, arrive: the ball is aimed at a point drop metres below the lens (the body, not the camera on
        top of it), which bends the climb in the last quarter second. Given (speed toward the robot,
        time of arrival) from a first pass, the elevations are taken as seen from that point.

        Returns a dict: cam, n, span, t (time of the last look), az and q = tan(elevation) at t with
        their rates az_dot and q_dot, the scatter of each fit (sig_az_dot, sig_q_dot, rms_az, rms_q),
        the looks' times and horizontal ranges (ts, r_h; from looks whose range can be trusted when
        there are two or more), and the lens position.
        """
        by_cam = {}
        for h in self.history:
            by_cam.setdefault(h[2], []).append(h)
        best = None
        for looks in by_cam.values():
            span = looks[-1][0] - looks[0][0]
            if len(looks) >= min_looks and span >= min_span and (best is None or (len(looks), span) > best[0]):
                best = ((len(looks), span), looks)
        if best is None:
            return None
        looks = best[1][-8:]
        ts = np.array([h[0] for h in looks])
        rays = np.stack([h[4] for h in looks])
        flat = np.hypot(rays[:, 0], rays[:, 1])
        if float(flat.min()) < 0.1:
            return None                                # almost straight overhead: azimuth means nothing there
        az = np.unwrap(np.arctan2(rays[:, 1], rays[:, 0]))
        q = rays[:, 2] / flat
        if drop > 0.0 and arrive is not None:
            v_c, T = arrive
            q = q + drop / np.maximum(v_c * (T - ts), 0.3)
        tc = ts - ts.mean()
        S = float((tc * tc).sum())
        az_dot = float((tc * (az - az.mean())).sum() / S)
        q_dot = float((tc * (q - q.mean())).sum() / S)
        rms_az = float(np.sqrt(np.mean((az - az.mean() - az_dot * tc) ** 2)))
        rms_q = float(np.sqrt(np.mean((q - q.mean() - q_dot * tc) ** 2)))
        good = [i for i, h in enumerate(looks) if h[3]]
        use = good if len(good) >= 2 else list(range(len(looks)))
        return {"cam": looks[0][2], "n": len(looks), "span": float(ts[-1] - ts[0]), "t": float(ts[-1]),
                "az": float(az.mean() + az_dot * tc[-1]), "q": float(q.mean() + q_dot * tc[-1]),
                "t_mid": float(ts.mean()), "az_mid": float(az.mean()), "q_mid": float(q.mean()),
                "az_dot": az_dot, "q_dot": q_dot, "rms_az": rms_az, "rms_q": rms_q,
                "sig_az_dot": max(rms_az, 0.004) / math.sqrt(S), "sig_q_dot": max(rms_q, 0.004) / math.sqrt(S),
                "ts": ts[use], "r_h": np.array([looks[i][5] * flat[i] for i in use]), "ranges_good": len(good) >= 2,
                "origin": looks[-1][6]}

    def one_camera(self, need: int, min_span: float = 0.03):
        """The recent measurements of the single camera that has seen the ball most, or None.

        Each camera reads the ball's range from how big it looks, with an error of its own (its size
        calibration, a hand over part of the ball, the light). The same still ball can read 4.5 m in one
        camera and 1.5 m in the next: put together, that is a ball rushing in at 10 m/s. On the Pupper
        (2026-11-26) a ball being taken out of a basket beside the robot set off a "dodge" this way ten
        times in four minutes. Within one camera the error is the same from picture to picture, so how
        the ball moves is judged from one camera's pictures at a time.
        """
        by_cam = {}
        for h in self.history:
            if h[3]:
                by_cam.setdefault(h[2], []).append(h)
        best = None
        for pts in by_cam.values():
            span = pts[-1][0] - pts[0][0]
            if len(pts) >= need and span >= min_span and (best is None or (len(pts), span) > best[0]):
                best = ((len(pts), span), pts)
        return None if best is None else best[1]

    def fitted_velocity(self):
        """Straight-line velocity through one camera's recent measurement points (no gravity assumed), or None.

        Used to tell a thrown ball from a still object: the filter itself always
        assumes free fall, so a ball-coloured thing hanging above the robot looks
        to it as if it were dropping.
        """
        pts_h = self.one_camera(3)
        if pts_h is None:
            return None
        ts = np.array([h[0] for h in pts_h])
        pts = np.stack([h[1] for h in pts_h])
        tc = ts - ts.mean()
        return (tc[:, None] * (pts - pts.mean(axis=0))).sum(axis=0) / float((tc * tc).sum())

    def camera_closing(self):
        """Speed toward the robot as one camera alone has seen it (m/s; positive = approaching), or None
        if no camera has had three good looks at it yet. See one_camera. (Two are not enough: a still
        ball's blob flickers by 15 % in area from one picture to the next, which is 6 m/s over 40 ms.)"""
        pts_h = self.one_camera(3, 0.05)
        if pts_h is None:
            return None
        ts = np.array([h[0] for h in pts_h])
        rs = np.array([float(np.linalg.norm(h[1])) for h in pts_h])
        tc = ts - ts.mean()
        return float(-(tc * (rs - rs.mean())).sum() / (tc * tc).sum())

    def azimuth_elevation_deg(self):
        p = self.x[:3]
        return (math.degrees(math.atan2(p[1], p[0])),
                math.degrees(math.atan2(p[2], math.hypot(p[0], p[1]))))

    def angular_size(self, ball_diameter: float):
        """(theta, theta_rate): angular diameter and its growth rate, rad and rad/s."""
        r = max(self.range_m, ball_diameter / 2.0 + 1e-6)
        s = min(1.0, (ball_diameter / 2.0) / r)
        theta = 2.0 * math.asin(s)
        r_dot = -self.closing_speed
        theta_dot = -2.0 * s / (r * math.sqrt(max(1e-9, 1.0 - s * s))) * r_dot
        return theta, theta_dot


def accel(v: np.ndarray, drag_k: float) -> np.ndarray:
    a = np.array([0.0, 0.0, -G])
    if drag_k > 0.0:
        a = a - drag_k * np.linalg.norm(v) * v
    return a


def drag_coefficient(ball: dict) -> float:
    """k in a_drag = -k |v| v  (1/m)."""
    area = math.pi * (ball["diameter_m"] / 2.0) ** 2
    return 1.2 * float(ball.get("drag_cd", 0.47)) * area / (2.0 * ball["mass_kg"])


class Tracker:
    def __init__(self, cfg: dict, ball: dict):
        self.cfg = cfg
        self.q_a = float(cfg.get("accel_sigma", 4.0))
        self.q_p = float(cfg.get("pos_walk_sigma", 0.3))
        self.speed_sigma = float(cfg.get("init_speed_sigma", 7.0))
        self.gate = float(cfg.get("gate_chi2", 20.0))
        self.gate_cos = math.cos(math.radians(float(cfg.get("gate_angle_deg", 45.0))))
        self.confirm_hits = int(cfg.get("confirm_hits", 3))
        self.max_coast = float(cfg.get("max_coast_s", 0.25))
        self.flight_coast = float(cfg.get("flight_coast_s", 0.6))
        self.flight_speed = float(cfg.get("flight_speed", 1.5))
        self.flight_travel = float(cfg.get("flight_travel_m", 0.15))
        self.tent_coast = float(cfg.get("tentative_coast_s", 0.12))
        self.max_tracks = int(cfg.get("max_tracks", 24))
        self.max_range = float(cfg.get("max_range_m", 8.0))
        self.drag_k = drag_coefficient(ball) if cfg.get("use_drag", True) else 0.0
        self.tracks: list = []

    # ---- model
    def predict_state(self, x: np.ndarray, P: np.ndarray, dt: float):
        """State and covariance dt seconds later (dt may be slightly negative)."""
        if dt == 0.0:
            return x.copy(), P.copy()
        a = accel(x[3:], self.drag_k)
        xn = x.copy()
        xn[:3] = x[:3] + x[3:] * dt + 0.5 * a * dt * dt
        xn[3:] = x[3:] + a * dt
        F = np.eye(6)
        F[:3, 3:] = np.eye(3) * dt
        h = abs(dt)
        qa2 = self.q_a ** 2
        Q = np.zeros((6, 6))
        Q[:3, :3] = np.eye(3) * (qa2 * h ** 4 / 4.0 + self.q_p ** 2 * h)
        Q[:3, 3:] = Q[3:, :3] = np.eye(3) * (qa2 * h ** 3 / 2.0)
        Q[3:, 3:] = np.eye(3) * (qa2 * h * h)
        return xn, F @ P @ F.T + Q

    @staticmethod
    def meas_cov(ray: np.ndarray, sigma_t: float, sigma_r: float) -> np.ndarray:
        rr = np.outer(ray, ray)
        return sigma_r ** 2 * rr + sigma_t ** 2 * (np.eye(3) - rr)

    # ---- update with one camera frame
    def update(self, t: float, meas: list, R_wb: np.ndarray | None = None) -> None:
        """meas: Measurement objects in base_link; R_wb rotates base_link into the world frame."""
        if R_wb is None:
            R_wb = np.eye(3)
        items = []
        for m in meas:
            ray = R_wb @ m.ray
            z = R_wb @ m.origin + m.range_m * ray
            if m.range_m > self.max_range:
                continue
            items.append((m, z, self.meas_cov(ray, m.sigma_t, m.sigma_r), ray, R_wb @ m.origin))

        # Score every (track, measurement) pair, then assign best-first. All pairs are scored in one
        # go: a cluttered room gives tens of tracks and blobs, and one small solve per pair in Python
        # was a large share of the node's time on the Pupper.
        pred = [self.predict_state(tr.x, tr.P, t - tr.t) for tr in self.tracks]
        pairs = []
        if pred and items:
            xs = np.stack([xp[:3] for xp, _ in pred])                      # (T, 3)
            Ps = np.stack([Pp[:3, :3] for _, Pp in pred])                  # (T, 3, 3)
            zs = np.stack([it[1] for it in items])                         # (M, 3)
            Rs = np.stack([it[2] for it in items])                         # (M, 3, 3)
            y = zs[None, :, :] - xs[:, None, :]                            # (T, M, 3)
            S = Ps[:, None, :, :] + Rs[None, :, :, :]                      # (T, M, 3, 3)
            try:
                d2 = np.einsum("tmi,tmi->tm", y, np.linalg.solve(S, y[..., None])[..., 0])
            except np.linalg.LinAlgError:
                d2 = np.full(y.shape[:2], np.inf)
                for ti in range(y.shape[0]):
                    for mi in range(y.shape[1]):
                        try:
                            d2[ti, mi] = float(y[ti, mi] @ np.linalg.solve(S[ti, mi], y[ti, mi]))
                        except np.linalg.LinAlgError:
                            pass
            # ... and the blob must lie in the direction the track is expected in, as that camera sees it.
            # Every camera's rays start at the robot, so a track whose range is vague (a ball cut off by
            # the picture's edge) otherwise matches a blob on the opposite side of the robot, and the two
            # together put the "ball" at the robot itself (seen on the Pupper, 2026-11-26).
            rays = np.stack([it[3] for it in items])                       # (M, 3)
            los = xs[:, None, :] - np.stack([it[4] for it in items])[None, :, :]
            dist = np.linalg.norm(los, axis=2)
            cos = np.einsum("tmi,mi->tm", los, rays) / np.maximum(dist, 1e-6)
            d2 = np.where((cos >= self.gate_cos) | (dist < 0.3), d2, np.inf)
            for ti, mi in zip(*np.nonzero(d2 < self.gate)):
                pairs.append((float(d2[ti, mi]), int(ti), int(mi)))
        pairs.sort()
        used_t, used_m = set(), set()
        for d2, ti, mi in pairs:
            if ti in used_t or mi in used_m:
                continue
            used_t.add(ti)
            used_m.add(mi)
            m, z, Rm = items[mi][:3]
            xp, Pp = pred[ti]
            S = Pp[:3, :3] + Rm
            K = Pp[:, :3] @ np.linalg.inv(S)
            tr = self.tracks[ti]
            tr.x = xp + K @ (z - xp[:3])
            IKH = np.eye(6)
            IKH[:, :3] -= K
            tr.P = IKH @ Pp @ IKH.T + K @ Rm @ K.T    # Joseph form: stays symmetric positive
            tr.t = t
            tr.t_last = max(tr.t_last, t)
            tr.hits += 1
            tr.cams.add(m.cam)
            tr.last_cam = m.cam
            tr.remember(t, z, m.cam, not (getattr(m, "truncated", False) or getattr(m, "vague", False)),
                        ray=items[mi][3], origin=items[mi][4], range_m=m.range_m)
            if tr.hits >= self.confirm_hits:
                tr.confirmed = True
                if not tr.flying and tr.travel_m >= self.flight_travel:
                    # Fast AND gone somewhere across the picture: a still object's jittering range can
                    # look like speed, but it does not move across the line of sight.
                    v_fit = tr.fitted_velocity()
                    tr.flying = v_fit is not None and float(np.linalg.norm(v_fit)) >= self.flight_speed

        self.prune(t)       # first: tracks that have gone unseen too long make room
        for mi, (m, z, Rm, _ray, _origin) in enumerate(items):
            if mi in used_m:
                continue
            if len(self.tracks) >= self.max_tracks and not self._make_room(t):
                break
            self.tracks.append(Track(t, z, Rm, self.speed_sigma, m.cam,
                                     not (getattr(m, "truncated", False) or getattr(m, "vague", False)),
                                     ray=_ray, origin=_origin, range_m=m.range_m))

    def _make_room(self, t: float) -> bool:
        """With every slot taken, drop the track least likely to be a ball in flight. False if none may go.

        Before 2026-11-26 a new blob was simply ignored while all slots were full. In a room with
        balls lying about and patches of colour noise the slots were always full, so a thrown ball
        often got no track at all and the robot never reacted to it.
        """
        victim, worst = None, None
        for i, tr in enumerate(self.tracks):
            if tr.flying or tr.t0 >= t:
                continue                              # never a ball seen in flight, nor a track begun in this very picture
            if not tr.confirmed:
                if tr.t_last >= t:
                    continue                          # seen again just now: it may be a ball on its way to being confirmed
                key = (0, tr.hits, tr.t_last)         # a tentative track that was not seen this time goes first
            elif tr.closing_speed > 1.0:
                continue                              # coming this way: keep
            else:
                key = (1, -tr.range_m, 0.0)           # then a still thing, the farthest first: it matters least
            if worst is None or key < worst:
                victim, worst = i, key
        if victim is None:
            return False
        del self.tracks[victim]
        return True

    def prune(self, t_now: float) -> None:
        keep = []
        for tr in self.tracks:
            # A ball in flight is followed on its arc for longer while unseen: a lob leaves the top of a
            # rear camera's picture in mid-flight and comes down on the robot from above.
            limit = (self.flight_coast if tr.flying else self.max_coast) if tr.confirmed else self.tent_coast
            if t_now - tr.t_last <= limit and np.isfinite(tr.x).all():
                keep.append(tr)
        self.tracks = keep

    def confirmed(self) -> list:
        return [tr for tr in self.tracks if tr.confirmed]
