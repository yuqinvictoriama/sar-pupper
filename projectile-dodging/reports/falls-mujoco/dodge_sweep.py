"""Dodge-profile sweep: each real dodge command and each candidate profile change, ~20
randomised seeds per case (floor friction 0.4-0.8, 20-40 ms actuation latency, per-motor
strength, mass/COM), on the default walking policy.

    venv/bin/python dodge_sweep.py                 # everything -> results.json + tables
    venv/bin/python dodge_sweep.py --seeds 5 --only shipped
"""
from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import dataclass, replace, asdict
from multiprocessing import Pool

import numpy as np

from pupper_policy_sim import Policy, PupperSim, SimCfg, Trace, randomized

ARRIVE = 0.06       # s from the decision to the command reaching the policy

# the real commands (vx, vy, burst s), mirrored diagonals included
REAL = {
    "diag F-R (0.8,-0.4) 0.34": (0.8, -0.4, 0.34),
    "diag F-L (0.8,+0.4) 0.34": (0.8, 0.4, 0.34),
    "diag B-R (-0.64,-0.4) 0.47": (-0.64, -0.4, 0.47),
    "diag B-L (-0.64,+0.4) 0.47": (-0.64, 0.4, 0.47),
    "side R (0,-0.5) 0.36": (0.0, -0.5, 0.36),
    "side L (0,+0.5) 0.40": (0.0, 0.5, 0.40),
    "fwd (0.8,0) 0.40": (0.8, 0.0, 0.40),
    "fwd (1.0,0) 0.37": (1.0, 0.0, 0.37),
    "back (-0.8,0) 0.31": (-0.8, 0.0, 0.31),
}


def kind(vx, vy):
    if vx and vy:
        return "diag"
    return "side" if vy else "fore-aft"


@dataclass
class Profile:
    vx: float
    vy: float
    burst: float
    ramp_in: float = 0.0
    ramp_out: float = 0.2
    second_gap: float | None = None    # s between decisions; second dodge is the opposite one

    def one(self, t, vx, vy):
        """Command at t s after a decision."""
        t = t - ARRIVE
        if t < 0:
            return 0.0, 0.0
        if self.ramp_in > 0 and t < self.ramp_in:
            f = t / self.ramp_in
        elif t < self.burst:
            f = 1.0
        elif t < self.burst + self.ramp_out:
            f = 1.0 - (t - self.burst) / self.ramp_out
        else:
            f = 0.0
        return f * vx, f * vy

    def __call__(self, t):
        if self.second_gap is not None and t >= self.second_gap:
            return self.one(t - self.second_gap, -self.vx, -self.vy)
        return self.one(t, self.vx, self.vy)

    @property
    def end(self):
        return ARRIVE + self.burst + self.ramp_out


def variant_profiles(name, vx, vy, burst, rng):
    """Map a real command to the profile each variant would send (None = not applicable)."""
    diag = bool(vx and vy)
    if name == "shipped":
        return Profile(vx, vy, burst)
    if name == "ramp_out 0.4":
        return Profile(vx, vy, burst, ramp_out=0.4)
    if name == "ramp_out 0.6":
        return Profile(vx, vy, burst, ramp_out=0.6)
    if name == "ramp_in 0.08":
        return Profile(vx, vy, burst, ramp_in=0.08)
    if name == "ramp_in 0.10":
        return Profile(vx, vy, burst, ramp_in=0.10)
    if name == "ramp_in 0.10 + out 0.4":
        return Profile(vx, vy, burst, ramp_in=0.10, ramp_out=0.4)
    if name == "diag -> y only":     # the lateral axis, undiminished (0.5)
        return Profile(0.0, math.copysign(0.5, vy), burst) if diag else None
    if name == "diag -> x only":     # the fore-aft axis, undiminished (1.0 / -0.8)
        return Profile(vx / 0.8, 0.0, burst) if diag else None
    if name == "diag scale 0.6":
        return Profile(vx / 0.8 * 0.6, vy / 0.8 * 0.6, burst) if diag else None
    if name == "caps vy0.4 vx0.8":
        return Profile(max(-0.8, min(0.8, vx)), max(-0.4, min(0.4, vy)), burst)
    if name == "vx cap 0.45 (trot only)":   # below gallop_speed 0.5: the policy never enters the fast gait
        return Profile(max(-0.45, min(0.45, vx)), vy, burst) if vx else None
    if name == "second dodge (opposite)":
        return Profile(vx, vy, burst, second_gap=float(rng.uniform(1.5, 2.5)))
    if name == "second dodge gap 0.6-1.2":
        return Profile(vx, vy, burst, second_gap=float(rng.uniform(0.6, 1.2)))
    if name == "second dodge, ramp_out 0.4":
        return Profile(vx, vy, burst, ramp_out=0.4, second_gap=float(rng.uniform(1.5, 2.5)))
    raise KeyError(name)


VARIANTS = ["shipped", "ramp_out 0.4", "ramp_out 0.6", "ramp_in 0.08", "ramp_in 0.10",
            "ramp_in 0.10 + out 0.4", "diag -> y only", "diag -> x only", "diag scale 0.6",
            "caps vy0.4 vx0.8", "vx cap 0.45 (trot only)", "second dodge (opposite)",
            "second dodge, ramp_out 0.4", "second dodge gap 0.6-1.2"]

_POL = None


def _policy():
    global _POL
    if _POL is None:
        _POL = Policy()
    return _POL


def run_case(job):
    """One run: stand (random 1.5-3 s, so the gait phase differs), decide, dodge, watch."""
    cmd_name, variant, seed, cond = job
    vx, vy, burst = REAL[cmd_name]
    rng = np.random.default_rng(seed * 7919 + 17)
    prof = variant_profiles(variant, vx, vy, burst, rng)
    if prof is None:
        return None
    base = SimCfg(**cond.get("base", {}))
    cfg = randomized(seed, base, **cond.get("dr", {})) if cond.get("randomise", True) else replace(base, seed=seed)
    sim = PupperSim(cfg, _policy())
    settle = float(rng.uniform(1.5, 3.0))
    tr = sim.run(lambda t: (0.0, 0.0), settle, trace=Trace(), sub=5)
    td = tr.t[-1]
    i_dec = len(tr.t)
    pos0 = np.array(tr.pos[-1]); yaw0 = tr.yaw[-1]
    tail = 2.5
    T = (prof.second_gap + prof.end + tail) if prof.second_gap else (prof.end + tail)
    sim.run(lambda t: prof(t - td), T, t0=td, trace=tr, sub=5)

    t = np.array(tr.t[i_dec:]) - td
    tilt = np.degrees(np.array(tr.tilt[i_dec:]))
    gyro = np.degrees(np.array(tr.gyro[i_dec:]))
    P = np.array(tr.pos[i_dec:]) - pos0
    c, s = math.cos(yaw0), math.sin(yaw0)
    fwd = P[:, 0] * c + P[:, 1] * s
    lat = -P[:, 0] * s + P[:, 1] * c
    h = np.array(tr.height[i_dec:])
    burst_end = ARRIVE + prof.burst

    def window(lo, hi):
        w = (t >= lo) & (t < hi)
        k = np.argmax(tilt[w])
        return dict(peak_tilt=float(tilt[w][k]), t_peak_after_burst=float(t[w][k] - (lo + burst_end)),
                    roll_rate=float(np.abs(gyro[w, 0]).max()), pitch_rate=float(np.abs(gyro[w, 1]).max()),
                    yaw_rate=float(np.abs(gyro[w, 2]).max()))

    first_hi = prof.second_gap if prof.second_gap else t[-1] + 1
    out = dict(cmd=cmd_name, variant=variant, seed=seed, kind=kind(vx, vy), vx=prof.vx, vy=prof.vy,
               burst=prof.burst, settle=settle, friction=cfg.friction, latency=cfg.act_latency,
               mass_scale=cfg.mass_scale, **window(0.0, first_hi))
    fell = tr.estop_t is not None or bool((tilt > 60).any()) or bool((h < 0.07).any())
    out["fallen"] = fell
    out["fall_t"] = (tr.estop_t - td) if tr.estop_t is not None else None
    out["over30"] = bool(tilt.max() > 30.0) or fell
    out["peak_tilt_all"] = float(tilt.max())
    # displacement along the commanded direction and sideways, at 0.35 / 0.5 s after the decision
    u = np.array([prof.vx, prof.vy]); u = u / max(np.linalg.norm(u), 1e-9)
    for tt in (0.35, 0.5):
        k = int(np.searchsorted(t, tt))
        out[f"along_{tt}"] = float(fwd[k] * u[0] + lat[k] * u[1])
        out[f"lat_{tt}"] = float(lat[k] * (math.copysign(1, prof.vy) if prof.vy else 1.0))
        out[f"fwd_{tt}"] = float(fwd[k])
    if prof.second_gap:
        out["second_gap"] = prof.second_gap
        w2 = window(prof.second_gap, t[-1] + 1)
        out.update({f"second_{k}": v for k, v in w2.items()})
    return out


def summarise(rows, by=("variant", "cmd")):
    groups = {}
    for r in rows:
        groups.setdefault(tuple(r[k] for k in by), []).append(r)
    out = []
    for key, rs in groups.items():
        a = lambda k: np.array([r[k] for r in rs if r.get(k) is not None], float)
        d = dict(zip(by, key))
        d.update(n=len(rs), tilt_med=float(np.median(a("peak_tilt"))), tilt_p90=float(np.percentile(a("peak_tilt"), 90)),
                 ge12=float(np.mean(a("peak_tilt") >= 12)), over30=float(np.mean([r["over30"] for r in rs])),
                 fallen=float(np.mean([r["fallen"] for r in rs])),
                 roll=float(np.median(a("roll_rate"))), pitch=float(np.median(a("pitch_rate"))),
                 yaw=float(np.median(a("yaw_rate"))), yaw_p90=float(np.percentile(a("yaw_rate"), 90)),
                 tpk=float(np.median(a("t_peak_after_burst"))),
                 along35=float(np.median(a("along_0.35"))), along50=float(np.median(a("along_0.5"))),
                 lat35=float(np.median(a("lat_0.35"))), lat50=float(np.median(a("lat_0.5"))))
        if "second_peak_tilt" in rs[0]:
            d["second_tilt_med"] = float(np.median(a("second_peak_tilt")))
            d["second_ge12"] = float(np.mean(a("second_peak_tilt") >= 12))
        out.append(d)
    return out


def table(summ, keys=("variant", "cmd")):
    hdr = (f"{'case':58s} {'n':>3s} {'tilt med':>8s} {'p90':>5s} {'>=12':>5s} {'>30/fall':>8s} {'fell':>5s} "
           f"{'roll':>5s} {'pitch':>5s} {'yaw':>5s} {'tpk':>5s} {'al.35':>6s} {'al.50':>6s} {'lat.35':>6s} {'lat.50':>6s}")
    lines = [hdr]
    for d in summ:
        name = " | ".join(str(d[k]) for k in keys)
        lines.append(f"{name:58s} {d['n']:3d} {d['tilt_med']:8.1f} {d['tilt_p90']:5.1f} {d['ge12']*100:4.0f}% "
                     f"{d['over30']*100:7.0f}% {d['fallen']*100:4.0f}% {d['roll']:5.0f} {d['pitch']:5.0f} {d['yaw']:5.0f} "
                     f"{d['tpk']:+5.2f} {d['along35']:6.2f} {d['along50']:6.2f} {d['lat35']:6.2f} {d['lat50']:6.2f}")
    return "\n".join(lines)


CONDITIONS = {
    "dr": dict(randomise=True),                                        # the main condition
    "nominal": dict(randomise=False, base=dict(friction=0.8, act_latency=0.016)),   # as trained
    "stiff": dict(randomise=True, base=dict(physics="stiff")),         # firm contacts, converged solver
    "no_heading_hold": dict(randomise=True, base=dict(heading_hold=False)),
    "52Hz": dict(randomise=True, base=dict(control_hz=52.0)),
    "low_fric": dict(randomise=True, dr=dict(fric=(0.3, 0.45))),
    "weak_heavy": dict(randomise=True, dr=dict(kp=(0.7, 0.9), tau=(2.2, 2.7), mass=(1.1, 1.25), lat=(0.03, 0.05))),
    # stress conditions, beyond the plausible band, to find where falls start
    "head_mass_0.35": dict(randomise=True, base=dict(extra_mass=0.35)),
    "latency_50_70": dict(randomise=True, dr=dict(lat=(0.05, 0.07))),
    "very_weak": dict(randomise=True, dr=dict(kp=(0.6, 0.8), tau=(1.8, 2.4))),
    "slippery_0.2": dict(randomise=True, dr=dict(fric=(0.2, 0.3))),
    "stiff_worst": dict(randomise=True, base=dict(physics="stiff", extra_mass=0.35),
                        dr=dict(kp=(0.7, 0.9), tau=(2.2, 2.7), mass=(1.1, 1.25), lat=(0.03, 0.05))),
}
CONDITIONS["stiff_lat45"] = dict(randomise=True, base=dict(physics="stiff"), dr=dict(lat=(0.04, 0.05)))
for _ms in (16, 30, 40, 50, 60, 70):   # latency ladder: fixed latency +-5 ms, everything else randomised
    CONDITIONS[f"lat{_ms}"] = dict(randomise=True, dr=dict(lat=((_ms - 5) / 1000, (_ms + 5) / 1000)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--only", nargs="*", default=None, help="variants")
    ap.add_argument("--cond", nargs="*", default=["dr"])
    ap.add_argument("--cmds", nargs="*", default=None)
    ap.add_argument("--out", default="results.json")
    ap.add_argument("--procs", type=int, default=8)
    args = ap.parse_args()
    variants = args.only or VARIANTS
    cmds = args.cmds or list(REAL)
    jobs = []
    for cname in args.cond:
        for v in variants:
            for c in cmds:
                for s in range(args.seeds):
                    jobs.append((c, v, s, {**CONDITIONS[cname], "name": cname}))
    t0 = time.time()
    with Pool(args.procs) as p:
        rows = p.map(run_case, jobs, chunksize=4)
    for j, r in zip(jobs, rows):
        if r is not None:
            r["cond"] = j[3]["name"]
    rows = [r for r in rows if r is not None]
    json.dump(rows, open(args.out, "w"), indent=0)
    print(f"{len(rows)} runs in {time.time() - t0:.0f} s -> {args.out}")
    for cname in args.cond:
        rs = [r for r in rows if r["cond"] == cname]
        print(f"\n=== condition {cname}: per variant x command")
        print(table(summarise(rs)))
        print(f"\n=== condition {cname}: per variant x kind")
        print(table(summarise(rs, by=("variant", "kind")), keys=("variant", "kind")))


if __name__ == "__main__":
    main()
