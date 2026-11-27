#!/usr/bin/env python3
"""Find where the front camera really is: its height, how much it looks down, and how far
ahead of the body centre it sits. Everything terrain agility measures depends on these.

Do it after `tools/learn_colors.py floor`, with the robot STANDING under its walking
controller on level floor. Use any obstacle of known height whose side face shows against
both the floor and its own top (the scale-across slab, with its raw edge, is fine);
`tools/learn_colors.py check` must print a height for it, not "not read".

  1. Put the box square across the robot's path. Measure, with a ruler on the floor, from
     the FRONT OF THE FRONT FEET to the box's near face, and record it:
         python3 tools/calibrate_camera.py add 0.25
     Each `add` prints the three angles it read. If one says "not seen", fix that first.
  2. Move the box (not the robot) and repeat, three or four distances between 0.20 and 0.55 m:
         python3 tools/calibrate_camera.py add 0.35
         python3 tools/calibrate_camera.py add 0.50
     Adding a distance again replaces what was recorded for it.
  3. Solve and write the result into config/terrain.yaml:
         python3 tools/calibrate_camera.py solve --box-height 0.015 --box-depth 0.38 --write
     (--box-height: the obstacle's real height in metres, measured with calipers or a ruler.
      --box-depth: its real size fore-aft, near face to far edge. With it the far edge of the
      top is used too, which makes the fit much firmer. Leave it out for an obstacle whose far
      edge the camera cannot see.)
     `solve` lists, for each distance, how far each angle is from the fit: a distance that
     was measured wrongly stands out there. `list` shows what is recorded.
  4. Check: put the box at a new distance and run  python3 tools/learn_colors.py check
     The gauge's height should be within about 3 mm and its distance within about 1 cm.

`clear` forgets the recorded observations. They are kept in config/camera_calibration.json,
and the frames each `add` used in data/calibration/ (for looking at afterwards).
"""
import argparse
import json
import math

import cv2
import numpy as np

import _frames
from terrain import kinematics as K
from terrain.config import TerrainConfig
from terrain.gauge import fine_view, measure_obstacle, solve_camera_mount


def toe_offset() -> float:
    """Body centre to the front of the front feet in the standing pose (m)."""
    _, feet = K.default_stance()
    return float(feet[0, 0]) + K.FOOT_ENVELOPE_RADIUS


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    a = sub.add_parser("add")
    a.add_argument("distance", type=float, help="metres from the front of the front feet to the box's near face")
    _frames.add_frame_args(a)
    s = sub.add_parser("solve")
    s.add_argument("--box-height", type=float, required=True, help="the box's real height (m)")
    s.add_argument("--box-depth", type=float, default=None, help="the box's real fore-aft depth (m): uses its far edge too")
    s.add_argument("--keep-x-forward", action="store_true", help="do not fit the lens's fore-aft position")
    s.add_argument("--write", action="store_true", help="write the result into terrain.yaml")
    s.add_argument("--config", default=None)
    for name in ("clear", "list"):
        sub.add_parser(name).add_argument("--config", default=None)
    args = ap.parse_args()
    cfg = TerrainConfig(args.config)
    store = cfg.path.parent / "camera_calibration.json"
    data = json.loads(store.read_text()) if store.exists() else {"observations": []}
    pose_now = {"height": cfg.pose.height, "pitch_deg": cfg.pose.pitch_deg, "x_forward": cfg.pose.x_forward}

    def deg(v):
        return "not seen" if v is None or not math.isfinite(v) else f"{v:.2f} deg"

    if args.cmd == "clear":
        store.unlink(missing_ok=True)
        print("observations cleared")
        return

    if args.cmd == "list":
        for o in sorted(data["observations"], key=lambda o: o["toe_distance"]):
            print(f"{o['toe_distance'] * 100:5.1f} cm from the toes: foot {deg(o['foot_dep'])}, top edge {deg(o['top_dep'])}, "
                  f"far edge {deg(o.get('far_dep'))}")
        print(f"{len(data['observations'])} observation(s)")
        return

    if args.cmd == "add":
        if not 0.10 <= args.distance <= 0.70:
            raise SystemExit("use distances between 0.10 and 0.70 m (from the front of the front feet)")
        if data["observations"] and data.get("pose") != pose_now:
            raise SystemExit("terrain.yaml's camera pose changed since the earlier observations: run `clear` first")
        frames = _frames.frames_from_args(args, 10)
        h, w = frames[0][0].shape[:2]
        view = fine_view(cfg.model.scaled(w, h), cfg.pose)
        floor = cfg.load_floor()
        if not floor.trained:
            raise SystemExit("learn the floor first:  python3 tools/learn_colors.py floor")
        ms = [measure_obstacle(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB), view, floor, R_level_body=R, cfg=cfg.gauge,
                               ball=cfg.ball, near_depression_deg=cfg.near_depression_deg)
              for bgr, R, _ in frames]
        ms = [m for m in ms if m is not None and math.isfinite(m.foot_dep)]
        if len(ms) < max(1, len(frames) // 2):
            raise SystemExit("the box was not seen clearly straight ahead; run `python3 tools/learn_colors.py check`")

        def med(name):
            v = [getattr(m, name) for m in ms if math.isfinite(getattr(m, name))]
            return float(np.median(v)) if len(v) >= max(1, len(ms) // 2) else float("nan")

        foot, top, far = med("foot_dep"), med("top_dep"), med("far_dep")
        if foot > cfg.near_depression_deg - 3.0:
            raise SystemExit(f"too close: the foot of the box is {foot:.0f} deg below the horizon, at the bottom edge of what "
                             f"the camera uses ({cfg.near_depression_deg:.0f} deg). Nothing recorded; move it 5 cm farther away.")
        skew = math.degrees(float(np.median([m.skew for m in ms])))
        D = args.distance + toe_offset()
        if not getattr(args, "frames", None):       # keep what was looked at, for checking a poor fit afterwards
            folder = cfg.path.parent.parent / "data" / "calibration" / f"{round(args.distance * 1000):04d}mm"
            for old in folder.glob("frame_*"):
                old.unlink()
            _frames.save(frames, folder)
        again = [o for o in data["observations"] if abs(o["toe_distance"] - args.distance) < 0.005]
        data["observations"] = [o for o in data["observations"] if o not in again]
        data["observations"].append({"toe_distance": args.distance, "body_distance": D, "foot_dep": foot, "top_dep": top,
                                     "far_dep": far})
        data["pose"] = pose_now
        store.write_text(json.dumps(data, indent=1))
        print(f"recorded: obstacle {args.distance * 100:.1f} cm from the toes = {D * 100:.1f} cm from the body centre "
              f"({len(ms)} of {len(frames)} frames)" + ("; replaces the earlier reading at this distance" if again else ""))
        print(f"  below the horizon: foot of its near face {deg(foot)}, its top edge {deg(top)}, far edge of its top {deg(far)}")
        if not math.isfinite(top):
            print("  NOTE: no side face was seen at its foot, so its top edge is not used. If `learn_colors.py check` also says "
                  "\"height not read\", put tape in another colour along the edge and add this distance again.")
        if not math.isfinite(far):
            print("  NOTE: its far edge was not seen (something stands right behind it, or it is too deep to see over)")
        if abs(skew) > 4.0:
            print(f"  NOTE: the box looks {skew:+.0f} deg off square; square it up and add this distance again")
        print(f"  {len(data['observations'])} distance(s) recorded")
        return

    recorded = sorted(data["observations"], key=lambda o: o["toe_distance"])
    obs = [(o["body_distance"], o["foot_dep"], o["top_dep"], o.get("far_dep", float("nan"))) for o in recorded]
    if len(obs) < 2:
        raise SystemExit("record the box at two or more distances first (`add`)")
    if data.get("pose") != pose_now:
        raise SystemExit("terrain.yaml's camera pose changed since the observations were taken: `clear` and redo")
    spread = max(o[0] for o in obs) - min(o[0] for o in obs)
    fit_x = (not args.keep_x_forward) and len(obs) >= 3 and spread >= 0.20
    details = {}
    try:
        hc, corr, xf, rms = solve_camera_mount(obs, args.box_height, cfg.pose.x_forward, solve_x_forward=fit_x,
                                               height0=cfg.pose.height, box_depth=args.box_depth, details=details)
    except ValueError as e:
        raise SystemExit(f"no fit: {e}")
    pitch = cfg.pose.pitch_deg + corr
    print(f"camera height  {hc * 1000:.0f} mm   (was {cfg.pose.height * 1000:.0f})")
    print(f"pitch          {pitch:+.2f} deg, + = looking down   (was {cfg.pose.pitch_deg:+.2f})")
    print(f"x_forward      {xf * 1000:.0f} mm ahead of the body centre   (was {cfg.pose.x_forward * 1000:.0f}; "
          + ("fitted)" if fit_x else "kept: fitting it needs 3+ distances spread over 20 cm)"))
    print(f"fit residual   {rms:.2f} deg rms over {len(obs)} distances "
          + ("(good)" if rms < 0.35 else "(HIGH)"))
    print("how far each angle is from the fit (deg):")
    worst = (0.0, None)
    for o, res in zip(recorded, details["residuals"]):
        cells = ["   --  " if not math.isfinite(v) else f"{v:+6.2f} " for v in res]
        print(f"  {o['toe_distance'] * 100:5.1f} cm   foot {cells[0]}  top edge {cells[1]}  far edge {cells[2]}")
        big = float(np.nanmax(np.abs(res))) if np.isfinite(res).any() else 0.0
        if big > worst[0]:
            worst = (big, o["toe_distance"])
    if args.box_depth is None:
        print("  (far edge not used: give --box-depth, the box's fore-aft size, to use it)")
    if rms >= 0.35 and worst[1] is not None:
        print(f"the reading at {worst[1] * 100:.1f} cm fits worst: check that distance with the ruler (from the FRONT of the "
              "front feet to the near face, box square), add it again, and solve again. If every row is off, check "
              "--box-height and --box-depth.")
    if not args.write:
        print("add --write to put these into terrain.yaml")
        return
    if rms > 0.8:
        raise SystemExit("not written: the fit is too poor to trust")
    ok = all([
        _frames.set_yaml_value(cfg.path, "camera", "height", f"{hc:.4f}"),
        _frames.set_yaml_value(cfg.path, "camera", "pitch_deg", f"{pitch:.2f}"),
        _frames.set_yaml_value(cfg.path, "camera", "x_forward", f"{xf:.4f}"),
    ])
    store.unlink(missing_ok=True)
    print(f"written to {cfg.path}; observations cleared" if ok else f"could not update {cfg.path}: edit camera.height, "
          "pitch_deg and x_forward by hand")


if __name__ == "__main__":
    main()
