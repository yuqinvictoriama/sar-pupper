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
  2. Move the box (not the robot) and repeat, three or four distances between 0.20 and 0.55 m:
         python3 tools/calibrate_camera.py add 0.35
         python3 tools/calibrate_camera.py add 0.50
  3. Solve and write the result into config/terrain.yaml:
         python3 tools/calibrate_camera.py solve --box-height 0.015 --write
     (--box-height: the obstacle's real height in metres, measured with calipers or a ruler.)
  4. Check: put the box at a new distance and run  python3 tools/learn_colors.py check
     The gauge's height should be within about 3 mm and its distance within about 1 cm.

`clear` forgets the recorded observations. Observations are kept in config/camera_calibration.json.
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
    s.add_argument("--keep-x-forward", action="store_true", help="do not fit the lens's fore-aft position")
    s.add_argument("--write", action="store_true", help="write the result into terrain.yaml")
    s.add_argument("--config", default=None)
    c = sub.add_parser("clear")
    c.add_argument("--config", default=None)
    args = ap.parse_args()
    cfg = TerrainConfig(args.config)
    store = cfg.path.parent / "camera_calibration.json"
    data = json.loads(store.read_text()) if store.exists() else {"observations": []}
    pose_now = {"height": cfg.pose.height, "pitch_deg": cfg.pose.pitch_deg, "x_forward": cfg.pose.x_forward}

    if args.cmd == "clear":
        store.unlink(missing_ok=True)
        print("observations cleared")
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
        foot = float(np.median([m.foot_dep for m in ms]))
        tops = [m.top_dep for m in ms if math.isfinite(m.top_dep)]
        top = float(np.median(tops)) if len(tops) >= len(ms) // 2 else float("nan")
        skew = math.degrees(float(np.median([m.skew for m in ms])))
        D = args.distance + toe_offset()
        data["observations"].append({"toe_distance": args.distance, "body_distance": D, "foot_dep": foot, "top_dep": top})
        data["pose"] = pose_now
        store.write_text(json.dumps(data, indent=1))
        print(f"recorded: obstacle {args.distance * 100:.1f} cm from the toes = {D * 100:.1f} cm from the body centre; "
              f"foot of the face {foot:.2f} deg below the horizon, top edge {top:.2f} deg ({len(ms)} frames)")
        if not math.isfinite(top):
            print("  NOTE: no side face was seen, so only the foot of the obstacle is used (height from its top edge is not)")
        if abs(skew) > 4.0:
            print(f"  NOTE: the box looks {skew:+.0f} deg off square; square it up and add this distance again "
                  "(then `clear` and redo if you want a clean set)")
        print(f"  {len(data['observations'])} observation(s) so far")
        return

    obs = [(o["body_distance"], o["foot_dep"], o["top_dep"]) for o in data["observations"]]
    if len(obs) < 2:
        raise SystemExit("record the box at two or more distances first (`add`)")
    if data.get("pose") != pose_now:
        raise SystemExit("terrain.yaml's camera pose changed since the observations were taken: `clear` and redo")
    spread = max(o[0] for o in obs) - min(o[0] for o in obs)
    fit_x = (not args.keep_x_forward) and len(obs) >= 3 and spread >= 0.20
    hc, corr, xf, rms = solve_camera_mount(obs, args.box_height, cfg.pose.x_forward, solve_x_forward=fit_x,
                                           height0=cfg.pose.height)
    pitch = cfg.pose.pitch_deg + corr
    print(f"camera height  {hc * 1000:.0f} mm   (was {cfg.pose.height * 1000:.0f})")
    print(f"pitch          {pitch:+.2f} deg, + = looking down   (was {cfg.pose.pitch_deg:+.2f})")
    print(f"x_forward      {xf * 1000:.0f} mm ahead of the body centre   (was {cfg.pose.x_forward * 1000:.0f}; "
          + ("fitted)" if fit_x else "kept: fitting it needs 3+ distances spread over 20 cm)"))
    print(f"fit residual   {rms:.2f} deg rms over {len(obs)} distances "
          + ("(good)" if rms < 0.35 else "(HIGH: re-measure the distances and the box height)"))
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
