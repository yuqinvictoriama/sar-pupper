"""Sanity checks: standing stays level, steady walks move at about the commanded speed."""
import math, sys, time
import numpy as np
from pupper_policy_sim import PupperSim, SimCfg, Policy, Trace

pol = Policy()
physics = sys.argv[1] if len(sys.argv) > 1 else "train"
print(f"policy control rate: 50 Hz (trained; robot runs 520 Hz / repeat_action 10 = 52 Hz); physics={physics}")

def steady(vx, vy, T=6.0, cfg=None, hz=50.0):
    s = PupperSim(cfg or SimCfg(physics=physics, control_hz=hz), pol)
    tr = s.run(lambda t: (0.0, 0.0), 2.0, trace=Trace())   # stand first
    tr = s.run(lambda t: (vx, vy), T, t0=2.0, trace=tr)
    t = np.array(tr.t); P = np.array(tr.pos); tilt = np.degrees(tr.tilt); yaw = np.degrees(tr.yaw)
    w = t > 2.0 + T - 3.0
    v = (P[w][-1] - P[w][0]) / (t[w][-1] - t[w][0])
    # body-frame speed using mean heading in the window
    h = math.radians(np.mean(yaw[w]))
    vb = (v[0]*math.cos(h) + v[1]*math.sin(h), -v[0]*math.sin(h) + v[1]*math.cos(h))
    return vb, tilt[t > 2.0].max(), np.mean(tr.height[-50:]), yaw[-1], tr.estop_t, tr.mode[-1]

t0 = time.time()
s = PupperSim(SimCfg(physics=physics), pol)
tr = s.run(lambda t: (0.0, 0.0), 8.0, trace=Trace())
tilt = np.degrees(tr.tilt); P = np.array(tr.pos)
print(f"stand 8 s: max tilt after 1 s {tilt[np.array(tr.t)>1].max():.2f} deg, final tilt {tilt[-1]:.2f}, "
      f"height {tr.height[-1]*1000:.0f} mm, drift {np.linalg.norm(P[-1,:2]-P[50,:2])*1000:.0f} mm, yaw {tr.yaw[-1]*57.3:+.1f} deg")
for vx, vy in [(0.3, 0), (0.5, 0), (0.75, 0), (1.0, 0), (1.5, 0), (-0.5, 0), (-0.8, 0), (0, 0.3), (0, 0.5), (0, -0.5), (0.8, -0.4)]:
    vb, mt, h, yw, es, mode = steady(vx, vy)
    print(f"cmd ({vx:+.2f},{vy:+.2f}) [{mode}]: body speed ({vb[0]:+.2f},{vb[1]:+.2f}) m/s, max tilt {mt:5.1f} deg, "
          f"height {h*1000:.0f} mm, yaw {yw:+6.1f} deg, estop {es}")
print(f"({time.time()-t0:.1f} s wall)")
