"""Fore-aft speed scan across the policy's fast-gait switch (gallop_speed 0.5): burst 0.37 s,
shipped ramp_out 0.2, 20 seeds, two calibrated conditions. Also a pure-sideways speed scan."""
import multiprocessing as mp
import numpy as np
import dodge_sweep as D
cmds = {}
for vx in (0.3, 0.45, 0.49, 0.51, 0.6, 0.8, 1.0, -0.3, -0.45, -0.49, -0.51, -0.8):
    cmds[f"vx {vx:+.2f}"] = (vx, 0.0, 0.37)
for vy in (0.3, 0.4, 0.5):
    cmds[f"vy {vy:+.2f}"] = (0.0, vy, 0.37)
for vx in (0.45, -0.45):
    cmds[f"diag ({vx:+.2f},0.4)"] = (vx, 0.4, 0.37)
D.REAL.update(cmds)
if __name__ == "__main__":
    jobs = [(c, "shipped", s, {**D.CONDITIONS[k], "name": k}) for k in ("lat40", "lat50", "stiff_lat45") for c in cmds for s in range(20)]
    with mp.get_context("fork").Pool(8) as p:
        rows = p.map(D.run_case, jobs, chunksize=4)
    print(f"{'cond':12s} {'cmd':18s} tilt_med  p90 >=12 >30/fall pitch  al.50 lat.50")
    for k in ("lat40", "lat50", "stiff_lat45"):
        for c in cmds:
            rs = [r for r, j in zip(rows, jobs) if j[0] == c and j[3]["name"] == k]
            pt = np.array([r["peak_tilt"] for r in rs])
            print(f"{k:12s} {c:18s} {np.median(pt):7.1f} {np.percentile(pt,90):5.1f} {np.mean(pt>=12)*100:3.0f}% {np.mean([r['over30'] for r in rs])*100:5.0f}% "
                  f"{np.median([r['pitch_rate'] for r in rs]):5.0f} {np.median([r['along_0.5'] for r in rs]):6.2f} {np.median([r['lat_0.5'] for r in rs]):6.2f}")
