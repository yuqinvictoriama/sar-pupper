#!/usr/bin/env python3
"""Refine where the ELP cameras point, using the fisheye as the reference.

    python3 tools/record.py --seconds 40 --name overlap-left
        (during it: carry the ball slowly through the zone both the fisheye and
         the LEFT rear camera can see - beside the robot's left shoulder, 1-3 m
         out, high and low)
    python3 tools/calibrate_extrinsics.py data/recordings/overlap-left elp_left

Whenever both cameras see the ball at the same moment, their two lines of sight
must meet at the ball. The tool finds the small yaw / pitch / roll correction
to the ELP's mounting angles that makes them meet best, and writes it to
config/calib/<camera>_extrinsics.yaml. Positions are kept from the CAD.
Do the colour calibration of both cameras first.
"""
from __future__ import annotations

import argparse

from _common import cv2, load_config, np, run_tool

from dodge.camera_models import Camera, rotation_base_from_camera
from dodge.config import save_calib
from dodge.detect import BallDetector
from dodge.measure import measure_blob
from dodge.recording import RecordingReader


def ray_gap(o1, d1, o2, d2):
    """Shortest distance between two lines of sight, and the distance along each to that point."""
    w = o1 - o2
    b = float(d1 @ d2)
    d, e = float(d1 @ w), float(d2 @ w)
    den = 1.0 - b * b
    if den < 1e-9:
        return float(np.linalg.norm(np.cross(w, d1))), 0.0, 0.0
    s = (b * e - d) / den
    t = (e - b * d) / den
    return float(np.linalg.norm(w + s * d1 - t * d2)), s, t


def solve(pairs, extr: dict, t_ref: np.ndarray):
    """pairs: [(ref_ray_base, ref_range, elp_ray_cam, elp_range)]. Returns corrected (yaw, pitch, roll) and residuals."""
    from scipy.optimize import least_squares
    t_elp = np.array([extr["x"], extr["y"], extr["z"]])
    base = np.array([extr.get("yaw_deg", 0.0), extr.get("pitch_deg", 0.0), extr.get("roll_deg", 0.0)])

    def residuals(delta):
        R = rotation_base_from_camera(*(base + delta))
        out = []
        for ray_f, rng_f, ray_c, rng_e in pairs:
            ray_e = R @ ray_c
            gap, s, _t = ray_gap(t_ref, ray_f, t_elp, ray_e)
            dist = max(0.3, 0.5 * (rng_f + rng_e))
            out.append(gap / dist)                                   # how far the sight lines miss, as an angle
            p_f = t_ref + rng_f * ray_f                              # weak help from the size-based ranges
            p_e = t_elp + rng_e * ray_e
            out.extend(0.15 * (p_f - p_e) / dist)
        return np.array(out)

    fit = least_squares(residuals, np.zeros(3), loss="soft_l1", f_scale=0.02)
    before = residuals(np.zeros(3))[::4]
    after = residuals(fit.x)[::4]
    return base + fit.x, np.degrees(np.median(before)), np.degrees(np.median(after))


def collect(path: str, cfg: dict, elp: str, ref: str = "fisheye", max_dt: float = 0.02):
    rec = RecordingReader(path)
    cams = {n: Camera.from_config(n, cfg["cameras"][n]) for n in (ref, elp)}
    dets = {n: BallDetector.for_camera(cfg["cameras"][n], cfg["detector"]) for n in (ref, elp)}
    seen = {ref: [], elp: []}
    for t_raw, cam, jpeg in rec.frames([ref, elp]):
        bgr = cv2.imdecode(np.frombuffer(jpeg, dtype=np.uint8), cv2.IMREAD_COLOR)
        if bgr is None:
            continue
        blobs = dets[cam].detect(bgr)
        if len(blobs) != 1 or blobs[0].touches_edge:
            continue                                                  # exactly one clean ball, or skip the frame
        camera = cams[cam].with_resolution(bgr.shape[1], bgr.shape[0])
        m = measure_blob(camera, blobs[0], cfg["ball"]["diameter_m"])
        if m is None:
            continue
        ray_c, _ = camera.model.unproject(np.array([[blobs[0].u, blobs[0].v]]))
        seen[cam].append((t_raw + float(cfg["cameras"][cam].get("time_offset_s", 0.0)), m.ray, m.range_m, ray_c[0]))
    rec.close()
    pairs = []
    ref_t = np.array([s[0] for s in seen[ref]])
    for t, _ray_b, rng_e, ray_c in seen[elp]:
        if len(ref_t) == 0:
            break
        i = int(np.argmin(np.abs(ref_t - t)))
        if abs(ref_t[i] - t) <= max_dt:
            pairs.append((seen[ref][i][1], seen[ref][i][2], ray_c, rng_e))
    return pairs, cams[ref].t, len(seen[ref]), len(seen[elp])


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("recording")
    ap.add_argument("camera", help="elp_left or elp_right")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()
    cfg = load_config(args.config)
    pairs, t_ref, n_ref, n_elp = collect(args.recording, cfg, args.camera)
    print("ball seen in %d fisheye frames and %d %s frames; %d moments in both" % (n_ref, n_elp, args.camera, len(pairs)))
    if len(pairs) < 40:
        raise SystemExit("Need at least 40 moments where both cameras see the ball. Move the ball slowly through the "
                         "zone both can see (beside the robot, on that camera's side). Nothing was written.")
    extr = cfg["cameras"][args.camera]["extrinsics"]
    angles, before, after = solve(pairs, extr, t_ref)
    print("sight lines missed each other by %.2f deg (median) before, %.2f deg after" % (before, after))
    print("yaw %.2f -> %.2f, pitch %.2f -> %.2f, roll %.2f -> %.2f degrees" % (
        extr.get("yaw_deg", 0.0), angles[0], extr.get("pitch_deg", 0.0), angles[1], extr.get("roll_deg", 0.0), angles[2]))
    change = np.abs(angles - np.array([extr.get("yaw_deg", 0.0), extr.get("pitch_deg", 0.0), extr.get("roll_deg", 0.0)]))
    if change.max() > 15.0 or after > before:
        raise SystemExit("That correction is not believable (over 15 degrees, or no improvement). Check left/right "
                         "assignment, colour calibration and timing first. Nothing was written.")
    path = save_calib(args.camera + "_extrinsics", {"cameras": {args.camera: {"extrinsics": {
        "yaw_deg": round(float(angles[0]), 3), "pitch_deg": round(float(angles[1]), 3), "roll_deg": round(float(angles[2]), 3)}}}})
    print("written to %s" % path)


if __name__ == "__main__":
    run_tool(main)
