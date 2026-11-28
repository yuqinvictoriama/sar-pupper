"""Print one dodge's timeline: command the policy saw (after clip + heading hold), reference
mode, tilt and body rates. python trace_one.py vx vy burst [ramp_out] [seed] [heading_hold 0/1]"""
import sys, math
import numpy as np
from pupper_policy_sim import PupperSim, SimCfg, Trace, randomized
from dodge_sweep import Profile
vx, vy, burst = map(float, sys.argv[1:4])
ramp = float(sys.argv[4]) if len(sys.argv) > 4 else 0.2
seed = int(sys.argv[5]) if len(sys.argv) > 5 else 0
hh = bool(int(sys.argv[6])) if len(sys.argv) > 6 else True
cfg = randomized(seed, SimCfg(heading_hold=hh))
s = PupperSim(cfg)
tr = s.run(lambda t: (0, 0), 2.0, trace=Trace())
td = tr.t[-1]; n0 = len(tr.t); c0 = len(tr.cmd)
p = Profile(vx, vy, burst, ramp_out=ramp)
s.run(lambda t: p(t - td), 2.0, t0=td, trace=tr)
print(f"friction {cfg.friction:.2f} latency {cfg.act_latency*1000:.0f} ms; burst ends at +{0.06+burst:.2f}, zero at +{p.end:.2f}")
print("  t    raw(vx,vy)     policy cmd (vx,vy,wz)   mode   tilt  roll/pitch/yaw rate deg/s")
for i in range(c0, len(tr.cmd)):
    t = tr.t[i] - td
    if t > 1.6: break
    g = np.degrees(tr.gyro[i])
    print(f"{t:5.2f} ({tr.raw[i][0]:+.2f},{tr.raw[i][1]:+.2f})  ({tr.cmd[i][0]:+.2f},{tr.cmd[i][1]:+.2f},{tr.cmd[i][2]:+.2f})  {tr.mode[i]:5s} {math.degrees(tr.tilt[i]):5.1f}  {g[0]:+5.0f} {g[1]:+5.0f} {g[2]:+5.0f}")
