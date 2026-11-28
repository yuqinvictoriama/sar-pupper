"""Markdown tables for REPORT.md from the result files."""
import json
import numpy as np
L = lambda f: json.load(open(f))
main = L("results_main.json") + [r for r in L("results_stress.json") if r["cond"] == "dr"]
cal = L("results_cal.json")
groups = {
    "A: DR 20-40 ms": [r for r in main],
    "B: 40-50 ms": [r for r in cal if r["cond"] in ("lat40", "lat50")],
    "C: firm contacts 20-40 ms": [r for r in cal if r["cond"] == "stiff"],
}
def stats(rs):
    pt = np.array([r["peak_tilt"] for r in rs])
    m = lambda k: np.median([r[k] for r in rs])
    return dict(n=len(rs), med=np.median(pt), ge12=np.mean(pt >= 12) * 100, o30=np.mean([r["over30"] for r in rs]) * 100,
                fell=np.mean([r["fallen"] for r in rs]) * 100, roll=m("roll_rate"), pitch=m("pitch_rate"), yaw=m("yaw_rate"),
                tpk=m("t_peak_after_burst"), lat35=m("lat_0.35"), lat50=m("lat_0.5"), al35=m("along_0.35"), al50=m("along_0.5"))
variants = ["shipped", "ramp_out 0.4", "ramp_out 0.6", "ramp_in 0.08", "ramp_in 0.10", "ramp_in 0.10 + out 0.4",
            "diag -> y only", "diag -> x only", "diag scale 0.6", "caps vy0.4 vx0.8", "vx cap 0.45 (trot only)",
            "second dodge (opposite)", "second dodge gap 0.6-1.2", "second dodge, ramp_out 0.4"]
print("### Per profile and dodge kind\n")
print("Peak tilt median / share of runs with peak >= 12 deg / share over 30 deg or fallen, for each condition; "
      "body rates (median of each run's peak, deg/s) and displacement (median, m) are from condition B.\n")
print("| profile | kind | A tilt | A >=12 | A >30/fell | B tilt | B >=12 | B >30/fell | C tilt | C >=12 | C >30/fell | B roll/pitch/yaw | B peak after burst end (s) | B lateral @0.35/0.5 | B along cmd @0.35/0.5 |")
print("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
for v in variants:
    for k in ("diag", "side", "fore-aft"):
        S = {}
        for g, rs in groups.items():
            sel = [r for r in rs if r["variant"] == v and r["kind"] == k]
            S[g] = stats(sel) if sel else None
        if not any(S.values()):
            continue
        cells = []
        for g in groups:
            s = S[g]
            cells += ([f"{s['med']:.1f}", f"{s['ge12']:.0f}%", f"{s['o30']:.0f}%"] if s else ["-", "-", "-"])
        b = S["B: 40-50 ms"]
        cells += [f"{b['roll']:.0f}/{b['pitch']:.0f}/{b['yaw']:.0f}", f"{b['tpk']:+.2f}",
                  f"{b['lat35']:.2f}/{b['lat50']:.2f}", f"{b['al35']:.2f}/{b['al50']:.2f}"]
        print(f"| {v} | {k} | " + " | ".join(cells) + " |")
print("\n### Shipped profile per real command\n")
print("| command (vx, vy, burst) | A tilt | A >=12 | B tilt | B >=12 | B >30/fell | B roll/pitch/yaw | B lateral @0.5 | B along @0.5 |")
print("|---|---|---|---|---|---|---|---|---|")
cmds = []
for r in main:
    if r["cmd"] not in cmds: cmds.append(r["cmd"])
for c in cmds:
    a = stats([r for r in groups["A: DR 20-40 ms"] if r["variant"] == "shipped" and r["cmd"] == c])
    b = stats([r for r in groups["B: 40-50 ms"] if r["variant"] == "shipped" and r["cmd"] == c])
    print(f"| {c} | {a['med']:.1f} | {a['ge12']:.0f}% | {b['med']:.1f} | {b['ge12']:.0f}% | {b['o30']:.0f}% | "
          f"{b['roll']:.0f}/{b['pitch']:.0f}/{b['yaw']:.0f} | {b['lat50']:.2f} | {b['al50']:.2f} |")
