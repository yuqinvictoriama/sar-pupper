#!/usr/bin/env python3
"""Plan a crossing off the robot (no ROS) and print it: which footsteps, how long, and how
long the planning took on this computer.

    python3 tools/plan_box.py --height 0.015 --depth 0.38                 # a slab: scaled across
    python3 tools/plan_box.py --height 0.019 --depth 0.089 --near 0.36    # a low bar: stepped over
"""
import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from terrain.config import TerrainConfig  # noqa: E402
from terrain.crawl_planner import BoxSpec, load_templates, plan_crossing, plan_scale_across  # noqa: E402


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--height", type=float, required=True)
    ap.add_argument("--depth", type=float, required=True)
    ap.add_argument("--near", type=float, default=0.36, help="body centre to the near face (m)")
    ap.add_argument("--skew", type=float, default=0.0, help="degrees")
    ap.add_argument("--search", action="store_true", help="search from scratch if no stored pattern fits (slow)")
    ap.add_argument("--config", default=None)
    args = ap.parse_args()
    cfg = TerrainConfig(args.config)
    box = BoxSpec(args.height, args.depth, args.near, math.radians(args.skew))
    t0 = time.time()
    if args.depth >= cfg.decision.slab_min_depth - 0.003:
        print("scale across (a slab deep enough to walk on)")
        plan = plan_scale_across(box, cfg.crawl)
    else:
        print(f"step over (a low bar), from {len(load_templates())} stored footstep patterns")
        plan = plan_crossing(box, cfg.crawl, search=args.search or cfg.allow_search)
    print(plan.summary())
    print(f"planning took {time.time() - t0:.1f} s")
    raise SystemExit(0 if plan.ok else 1)


if __name__ == "__main__":
    main()
