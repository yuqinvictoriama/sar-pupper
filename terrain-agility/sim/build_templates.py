"""Offline: find footstep patterns for a grid of boxes, keep the ones the robot will use, and
work out which boxes can be crossed (the planning envelope).

    python3 sim/build_templates.py            # writes config/crawl_templates.json (about half an hour on 2 cores)

Searching for a footstep sequence from scratch takes many seconds, too long on the robot.
This runs that search once, here, for a grid of box sizes and writes each solution relative
to the box (a "template"). It then plans every box of a finer grid, from every distance the
robot may stop at to measure, exactly the way the robot will (crawl_planner.plan_crossing
with the stored patterns, nearest box first), and keeps the patterns that were used.

The envelope it writes - for each box depth, the tallest box that could be planned from
every starting distance, and every lower one too - is what terrain/decision.py goes by.

The run can be interrupted and restarted: finished searches and plans are kept in --cache.
"""

from __future__ import annotations

import argparse
import itertools
import json
import sys
import time
from multiprocessing import Pool
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from terrain import kinematics as K  # noqa: E402
from terrain.crawl_planner import HIP_HEIGHTS, BoxSpec, CrawlConfig, _Planner, plan_crossing  # noqa: E402

SEARCH_HEIGHTS = (0.019, 0.025, 0.030, 0.034)
DEPTHS = (0.060, 0.064, 0.070, 0.076, 0.082, 0.089, 0.095, 0.102)
# The grid the envelope is established on.
HEIGHTS = (0.019, 0.022, 0.025, 0.028, 0.031)
# Where the robot may be standing when it has measured the box: the gauge distance
# (terrain.yaml fsm.gauge_distance, 0.36 m from the body centre) give or take its tolerance.
NEARS = (0.30, 0.33, 0.36, 0.39, 0.42)
SEARCH_STAND_OFF = 0.020
SEARCH_EXTRA_MARGIN = 0.004            # patterns are searched for with this much balance margin to spare


def near_x(stand_off, cfg):
    _, feet = K.default_stance()
    return float(feet[0, 0]) + K.FOOT_ENVELOPE_RADIUS + cfg.foot_gap + stand_off


def discover(args):
    h, d, hh, budget = args
    base = CrawlConfig()
    cfg = CrawlConfig(hip_height=hh, max_evaluations=budget, min_margin=base.min_margin + SEARCH_EXTRA_MARGIN)
    P = _Planner(BoxSpec(h, d, near_x(SEARCH_STAND_OFF, cfg)), cfg)
    plan = P.plan()
    return (h, d, hh, P.encode(plan.steps) if plan.ok else None, P.evals)


def cover(args):
    h, d, near, templates = args
    t0 = time.time()
    plan = plan_crossing(BoxSpec(h, d, near), CrawlConfig(), templates=templates, search=False)
    if not plan.ok:
        return (h, d, near, None, time.time() - t0)
    source = plan.template.get("mirror_of", plan.template)
    # (the pattern itself is recorded, not its position in the list: the list changes between runs)
    return (h, d, near, {"pattern": {k: source[k] for k in ("steps", "hip_height", "height", "depth")},
                         "hip_height": plan.config.hip_height,
                         "swings": len(plan.steps), "seconds": round(plan.duration, 1),
                         "margin_mm": round(min(s.margin for s in plan.steps) * 1000, 1)}, time.time() - t0)


def key(*parts):
    return "/".join(f"{p * 1000:.0f}" for p in parts)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--budget", type=int, default=2500, help="search evaluations per box and hip height")
    ap.add_argument("--jobs", type=int, default=2)
    ap.add_argument("--cache", default="/tmp/crawl_templates_cache.json")
    ap.add_argument("--out", default=str(Path(__file__).resolve().parent.parent / "config" / "crawl_templates.json"))
    args = ap.parse_args()
    cache_path = Path(args.cache)
    cache = json.loads(cache_path.read_text()) if cache_path.exists() else {}
    cache.setdefault("search", {})
    cache.setdefault("cover", {})

    def save():
        cache_path.write_text(json.dumps(cache))

    t0 = time.time()
    with Pool(args.jobs) as pool:
        todo = [(h, d, hh, args.budget) for h in SEARCH_HEIGHTS for d in DEPTHS for hh in HIP_HEIGHTS
                if key(h, d, hh) not in cache["search"]]
        for h, d, hh, tpl, evals in pool.imap_unordered(discover, todo, chunksize=1):
            cache["search"][key(h, d, hh)] = tpl
            save()
            print(f"search h{h * 1000:.0f} d{d * 1000:.0f} H{hh * 1000:.0f}: {'found' if tpl else 'none'} "
                  f"({evals} evaluations, {time.time() - t0:.0f} s)", flush=True)
        templates = []
        for h in SEARCH_HEIGHTS:
            for d in DEPTHS:
                for hh in HIP_HEIGHTS:
                    tpl = cache["search"].get(key(h, d, hh))
                    if tpl and not any(t["steps"] == tpl and t["hip_height"] == hh for t in templates):
                        templates.append({"steps": tpl, "hip_height": hh, "height": h, "depth": d})
        print(f"{len(templates)} distinct patterns", flush=True)

        cases = list(itertools.product(HEIGHTS, DEPTHS, NEARS))
        todo = [(h, d, n, templates) for h, d, n in cases if key(h, d, n) not in cache["cover"]]
        for h, d, n, hit, dt in pool.imap_unordered(cover, todo, chunksize=1):
            cache["cover"][key(h, d, n)] = hit if hit is not None else False
            save()
            print(f"plan h{h * 1000:.0f} d{d * 1000:.0f} from {n * 1000:.0f}: "
                  + (f"pattern for {hit['pattern']['height'] * 1000:.0f}x{hit['pattern']['depth'] * 1000:.0f}, "
                     f"{hit['swings']} swings, {hit['seconds']} s, margin {hit['margin_mm']} mm"
                     if hit else "NO PLAN") + f"  ({dt:.0f} s; {time.time() - t0:.0f} s)", flush=True)

    kept = []
    for h, d, n in cases:
        c = cache["cover"][key(h, d, n)]
        if c and c["pattern"] not in kept:
            kept.append(c["pattern"])
    kept.sort(key=lambda t: (t["height"], t["depth"], t["hip_height"]))
    table = {}
    for h, d, n in cases:
        c = cache["cover"][key(h, d, n)]
        table.setdefault(f"{h * 1000:.0f}x{d * 1000:.0f}", {})[f"{n * 1000:.0f}"] = (
            {**{k: v for k, v in c.items() if k != "pattern"}, "template": kept.index(c["pattern"])} if c else None)
    envelope = {}
    for d in DEPTHS:
        best = 0.0
        for h in HEIGHTS:                                  # every lower box must be plannable too
            if all(table[f"{h * 1000:.0f}x{d * 1000:.0f}"][f"{n * 1000:.0f}"] for n in NEARS):
                best = h
            else:
                break
        envelope[f"{d * 1000:.0f}"] = best
    Path(args.out).parent.mkdir(parents=True, exist_ok=True)
    Path(args.out).write_text(json.dumps({
        "about": "Footstep patterns for the step-over crawl, found by sim/build_templates.py. Each has the crossing "
                 "height and the box it was found for, and its steps; a step is [leg, kind, offset_mm]: 'lo' = before "
                 "the box (offset from the take-off line), 'top' = middle of the box top, 'hi' = beyond the box "
                 "(offset from the landing line), 'final'. planning_envelope_max_height_by_depth_mm: for each box "
                 "depth (mm), the tallest box (m) that plans from every starting distance in 'coverage'.",
        "templates": kept,
        "planning_envelope_max_height_by_depth_mm": envelope,
        "coverage": table,
    }, indent=1))
    ok = sum(1 for c in cache["cover"].values() if c)
    print(f"kept {len(kept)} of {len(templates)} patterns; {ok}/{len(cases)} cases plan; wrote {args.out} ({time.time() - t0:.0f} s)")
    print("        " + "  ".join(f"d{d * 1000:.0f}" for d in DEPTHS) + "    (one mark per starting distance " +
          ", ".join(f"{n * 100:.0f}" for n in NEARS) + " cm)")
    for h in HEIGHTS:
        print(f"  h{h * 1000:.0f}:  " + "  ".join(
            "".join("Y" if table[f"{h * 1000:.0f}x{d * 1000:.0f}"][f"{n * 1000:.0f}"] else "-" for n in NEARS) for d in DEPTHS))
    print("envelope (depth mm -> tallest box mm): " + ", ".join(f"{d}: {v * 1000:.0f}" for d, v in envelope.items()))


if __name__ == "__main__":
    main()
