"""Compact tables from results*.json: condition x variant x kind."""
import json, sys
import numpy as np
rows = []
for f in sys.argv[1:]:
    rows += json.load(open(f))
def agg(rs):
    pt = np.array([r["peak_tilt"] for r in rs]); yw = np.array([r["yaw_rate"] for r in rs])
    return (f"{len(rs):3d} {np.median(pt):5.1f} {np.percentile(pt,90):5.1f} {pt.max():5.1f} {np.mean(pt>=12)*100:4.0f}% "
            f"{np.mean([r['over30'] for r in rs])*100:4.0f}% {np.median([r['roll_rate'] for r in rs]):5.0f} "
            f"{np.median([r['pitch_rate'] for r in rs]):5.0f} {np.median(yw):5.0f} {yw.max():5.0f} "
            f"{np.median([r['t_peak_after_burst'] for r in rs]):+5.2f} "
            f"{np.median([r['lat_0.35'] for r in rs]):5.2f} {np.median([r['lat_0.5'] for r in rs]):5.2f} "
            f"{np.median([r['along_0.5'] for r in rs]):5.2f}")
print(f"{'cond':16s} {'variant':28s} {'kind':9s}   n   med   p90   max  >=12  >30  roll pitch   yaw ymax   tpk lat35 lat50 al50")
keys = sorted({(r["cond"], r["variant"], r["kind"]) for r in rows}, key=lambda k: (k[0] != "dr", k[0], k[1], k[2]))
for k in keys:
    rs = [r for r in rows if (r["cond"], r["variant"], r["kind"]) == k]
    print(f"{k[0]:16s} {k[1]:28s} {k[2]:9s} {agg(rs)}")
