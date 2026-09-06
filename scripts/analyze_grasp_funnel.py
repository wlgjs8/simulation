"""Grasp funnel, read offline from diag/poses.jsonl -- no Isaac, no new rollouts.

The six-model std40 board reports one number per model (bolts resting in the right box at
t=30 s). That number cannot say WHERE a model loses: a policy that never grasps and a policy
that grasps twice as often and drops every one score the same. griponly is exactly that case
-- most lifts of any model (64) and fewest placements (13) -- so the loss has to be located
before any fidelity lever is chosen.

Every diag/poses.jsonl already carries what is needed: per tick, both TCP poses, both finger
link poses, the finger joint positions (= jaw gap) and all 20 bolt poses. This walks that
stream and emits, per carry episode: where the jaw was on the bolt when it closed, how high
it lifted, how far it carried, and how it ended -- released open, or slipped out of a closed
jaw. The final-position count is recomputed with the rig's own criterion so the funnel can be
checked against the published board before any of its other numbers are believed.
"""
import argparse
import json
import math
import multiprocessing as mp
import pathlib
import sys

import numpy as np

# --- geometry, copied from eval_closed_loop.py (single source: that file) -------------
TABLE_Z = -0.295
SURF_Z = TABLE_Z + 0.020            # 20 mm foam work surface
REST_Z = SURF_Z + 0.0092            # head radius: a bolt lying on its side on the foam
BOX_X, BOX_HW, BOX_HD = 0.765, 0.120, 0.190
BOX_CY = {"gray": +0.260, "black": -0.260}
ARM_OF_COLOR = {"gray": "left", "black": "right"}
COLOR_OF_ARM = {v: k for k, v in ARM_OF_COLOR.items()}
FULL_OPEN_M = 0.098                 # 98 mm total opening at 100%
SHAFT_D, HEAD_D = 0.012, 0.0184     # what a jaw gap means: shaft grip vs head grip

# --- event thresholds (all reported in the output so a re-read can move them) ---------
HELD_R = 0.050      # bolt centre within this of the TCP = plausibly between the jaws.
                    # The reference is the TCP, NOT the finger link origins the log also
                    # carries: those sit 248 mm up inside the housing (measured on
                    # griponly_s100), while a carried bolt tracks the TCP to 10-17 mm.
CLOSED_MM = 30.0    # a jaw wider than this is not holding a 12/18.4 mm bolt
OFF_TABLE = 0.008   # bolt centre this far above rest = off the foam
LIFT_M = 0.030      # the board's "lifted" threshold
BRIDGE = 5          # ticks of dropout to bridge inside one carry episode
MIN_TICKS = 5       # shorter than this is contact noise, not a carry
OPEN_MM = 4.0       # jaw widening within RELEASE_S that counts as a commanded release
RELEASE_S = 0.30


def _load(path):
    """poses.jsonl -> (t, jaw mid/gap per side, bolt xyz). One pass, no per-tick dicts kept."""
    t, arms, bolts, names = [], {}, [], None
    for line in open(path):
        line = line.strip()
        if not line:
            continue
        d = json.loads(line)
        t.append(d["time_ns"] * 1e-9)
        if names is None:
            names = sorted(d["bolts"])
        bolts.append([d["bolts"][n][:3] for n in names])
        for side, a in d["arms"].items():
            fl = np.asarray(a["fingers"]["finger_left"][:3])
            fr = np.asarray(a["fingers"]["finger_right"][:3])
            q = a["finger_q_m"]
            rec = arms.setdefault(side, {"mid": [], "gap": [], "tcp": []})
            rec["mid"].append((fl + fr) / 2.0)   # kept for reference; not the grasp point
            # the probe's mapping: gap = FULL_OPEN - (q_left - q_right); 1% ~= 0.98 mm
            rec["gap"].append((FULL_OPEN_M - (q[0] - q[1])) * 1000.0)
            rec["tcp"].append(a["tcp"][:3])
    out = {"t": np.asarray(t), "bolts": np.asarray(bolts), "names": names, "arms": {}}
    for side, rec in arms.items():
        out["arms"][side] = {k: np.asarray(v) for k, v in rec.items()}
    return out


def _intervals(mask):
    """Contiguous True runs, bridging short dropouts, as [start, end) index pairs."""
    idx = np.flatnonzero(mask)
    if idx.size == 0:
        return []
    runs, s, p = [], idx[0], idx[0]
    for i in idx[1:]:
        if i - p > BRIDGE:
            runs.append((s, p + 1))
            s = i
        p = i
    runs.append((s, p + 1))
    return [r for r in runs if r[1] - r[0] >= MIN_TICKS]


def analyze(run_dir, colors):
    run_dir = pathlib.Path(run_dir)
    d = _load(run_dir / "diag" / "poses.jsonl")
    t, B = d["t"], d["bolts"]                      # B: (n, 20, 3)
    dt = float(np.median(np.diff(t))) if t.size > 1 else 0.01
    rel_k = max(1, int(round(RELEASE_S / dt)))
    n = t.size

    # the rig's criterion, recomputed: a bolt resting inside a box footprint at the end,
    # excluding any that started there (frozen scenes put none there, but the rig checks).
    def in_box(p, col):
        return abs(p[0] - BOX_X) <= BOX_HW and abs(p[1] - BOX_CY[col]) <= BOX_HD

    pre = {i for i in range(B.shape[1])
           if any(in_box(B[0, i], c) for c in BOX_CY)}
    placed = misplaced = 0
    for i in range(B.shape[1]):
        if i in pre:
            continue
        for col in BOX_CY:
            if in_box(B[-1, i], col):
                if col == colors[i]:
                    placed += 1
                else:
                    misplaced += 1

    episodes = []
    for side, a in d["arms"].items():
        tcp, gap = a["tcp"], a["gap"]
        own = COLOR_OF_ARM[side]
        dist = np.linalg.norm(B - tcp[:, None, :], axis=2)          # (n, 20)
        off = B[:, :, 2] > REST_Z + OFF_TABLE
        closed = gap < CLOSED_MM
        for bi in range(B.shape[1]):
            for i0, i1 in _intervals((dist[:, bi] < HELD_R) & off[:, bi] & closed):
                z = B[i0:i1, bi, 2]
                seg_gap = gap[i0:i1]
                end = i1 - 1
                after = gap[end:min(n, end + rel_k + 1)]
                opened = float(after.max() - gap[end]) if after.size else 0.0
                p_end = B[end, bi]
                # where the bolt comes to rest, which is what the board actually counts
                final = B[-1, bi]
                rest_col = next((c for c in BOX_CY if in_box(final, c)), None)
                carry = float(np.linalg.norm(B[end, bi, :2] - B[i0, bi, :2]))
                # what happens to the bolt in the half second AFTER the carry ends:
                # a genuine drop keeps falling, a set-down does not.
                fall_k = min(n - 1, end + int(round(0.5 / dt)))
                fall = float(B[end, bi, 2] - B[fall_k, bi, 2])
                # how far the release point is from the arm's own box footprint (0 = inside)
                own_cy = BOX_CY[own]
                dbx = max(0.0, abs(p_end[0] - BOX_X) - BOX_HW)
                dby = max(0.0, abs(p_end[1] - own_cy) - BOX_HD)
                to_box = float(np.hypot(dbx, dby))
                speed = (float(np.linalg.norm(tcp[end] - tcp[max(0, end - rel_k)]) /
                               max(dt * rel_k, 1e-6)))
                episodes.append(dict(
                    side=side, bolt=int(bi), bolt_color=colors[bi],
                    own_color_bolt=bool(colors[bi] == own),
                    t0=round(float(t[i0]), 2), t1=round(float(t[end]), 2),
                    dur_s=round(float(t[end] - t[i0]), 2),
                    lift_mm=round(float(z.max() - REST_Z) * 1000.0, 1),
                    lifted=bool(z.max() - REST_Z > LIFT_M),
                    gap_at_close_mm=round(float(seg_gap[0]), 1),
                    gap_min_mm=round(float(seg_gap.min()), 1),
                    # a jaw narrower than the shaft can only be gripping through TPU
                    # deformation; wider than the head is not gripping the bolt at all
                    grip_site=("shaft" if seg_gap.min() < HEAD_D * 1000.0 else "head_or_wider"),
                    end_z_mm=round(float((p_end[2] - REST_Z)) * 1000.0, 1),
                    end_xy=[round(float(p_end[0]), 3), round(float(p_end[1]), 3)],
                    to_own_box_m=round(to_box, 3),
                    fall_after_mm=round(fall * 1000.0, 1),
                    gap_at_end_mm=round(float(gap[end]), 1),
                    end_over_box=bool(any(in_box(p_end, c) for c in BOX_CY)),
                    carry_m=round(carry, 3),
                    jaw_opened_mm=round(opened, 1),
                    end_kind=("released" if opened > OPEN_MM else "slipped"),
                    tcp_speed_end=round(speed, 3),
                    rests_in=rest_col or "table",
                    rest_correct=bool(rest_col == colors[bi]),
                ))
    return dict(run=run_dir.name, ticks=n, dt=round(dt, 4),
                placed=placed, misplaced=misplaced, episodes=episodes)


def _job(args):
    run_dir, colors = args
    try:
        return analyze(run_dir, colors)
    except Exception as e:                       # a bad run must not sink the batch
        return dict(run=pathlib.Path(run_dir).name, error=f"{type(e).__name__}: {e}",
                    placed=0, misplaced=0, episodes=[])


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("root", help="directory of <model>_s<seed> run dirs")
    ap.add_argument("--scene-states", required=True)
    ap.add_argument("--out", default="/tmp/grasp_funnel.json")
    ap.add_argument("--jobs", type=int, default=8)
    ap.add_argument("--models", default="", help="comma-separated filter")
    ap.add_argument("--pair", default="",
                    help="A,B: seed-matched comparison of two conditions with a sign test")
    ap.add_argument("--detail", action="store_true",
                    help="per-arm release-site table -- where the jaw opens relative to the box")
    args = ap.parse_args()

    states = json.load(open(args.scene_states))
    root = pathlib.Path(args.root)
    runs = sorted(p for p in root.iterdir() if (p / "diag" / "poses.jsonl").exists())
    keep = [m for m in args.models.split(",") if m]
    jobs = []
    for p in runs:
        model, _, seed = p.name.rpartition("_s")
        if keep and model not in keep:
            continue
        if seed not in states:
            print(f"  [skip] {p.name}: seed {seed} not in scene states", file=sys.stderr)
            continue
        jobs.append((str(p), states[seed]["colors"]))
    print(f"  {len(jobs)} runs, {args.jobs} workers", flush=True)

    with mp.Pool(args.jobs) as pool:
        res = pool.map(_job, jobs)

    per_model = {}
    for r, (rd, _) in zip(res, jobs):
        model = pathlib.Path(rd).name.rpartition("_s")[0]
        m = per_model.setdefault(model, dict(
            runs=0, placed=0, misplaced=0, episodes=0, own_color=0, lifted=0,
            released=0, slipped=0, released_over_box=0, released_off_box=0,
            slipped_after_lift=0, rest_correct=0, errors=0,
            lift_mm=[], gap_min_mm=[], carry_m=[], end_z_mm=[], dur_s=[]))
        m["runs"] += 1
        if r.get("error"):
            m["errors"] += 1
            print(f"  [error] {r['run']}: {r['error']}", file=sys.stderr)
        m["placed"] += r["placed"]
        m["misplaced"] += r["misplaced"]
        for e in r["episodes"]:
            m["episodes"] += 1
            m["own_color"] += e["own_color_bolt"]
            m["lift_mm"].append(e["lift_mm"])
            m["gap_min_mm"].append(e["gap_min_mm"])
            m["carry_m"].append(e["carry_m"])
            m["end_z_mm"].append(e["end_z_mm"])
            m["dur_s"].append(e["dur_s"])
            if e["lifted"]:
                m["lifted"] += 1
            if e["end_kind"] == "released":
                m["released"] += 1
                m["released_over_box" if e["end_over_box"] else "released_off_box"] += 1
            else:
                m["slipped"] += 1
                if e["lifted"]:
                    m["slipped_after_lift"] += 1
            m["rest_correct"] += e["rest_correct"]

    hdr = (f"{'model':<12}{'runs':>5}{'placed':>7}{'misp':>5}{'carries':>8}{'lifted':>7}"
           f"{'released':>9}{'  over box':>10}{'slipped':>8}{'slip/lift':>10}"
           f"{'lift p50':>9}{'gap p50':>8}{'carry p50':>10}")
    print("\n" + hdr)
    print("-" * len(hdr))
    for model in sorted(per_model, key=lambda k: -per_model[k]["placed"]):
        m = per_model[model]
        p50 = lambda k: float(np.median(m[k])) if m[k] else float("nan")
        print(f"{model:<12}{m['runs']:>5}{m['placed']:>7}{m['misplaced']:>5}"
              f"{m['episodes']:>8}{m['lifted']:>7}{m['released']:>9}"
              f"{m['released_over_box']:>10}{m['slipped']:>8}{m['slipped_after_lift']:>10}"
              f"{p50('lift_mm'):>9.1f}{p50('gap_min_mm'):>8.1f}{p50('carry_m'):>10.3f}")

    if args.detail:
        # The headline table counts releases; this one says WHERE they happen, which is the
        # number the hardware log (real_release_sites.py) reports in the same units.
        hdr2 = (f"{'model':<12}{'arm':<7}{'carries':>8}{'released':>9}{'over box':>10}"
                f"{'to-box p50/p90 (mm)':>22}{'lift p50':>10}{'carry p50':>11}{'hold p50':>10}")
        print("\n" + hdr2)
        print("-" * len(hdr2))
        for model in sorted(per_model, key=lambda k: -per_model[k]["placed"]):
            for side in ("left", "right"):
                E = [e for r, (rd, _) in zip(res, jobs)
                     if pathlib.Path(rd).name.rpartition("_s")[0] == model
                     for e in r["episodes"] if e["side"] == side]
                if not E:
                    continue
                rel = [e for e in E if e["end_kind"] == "released"]
                tb = [e["to_own_box_m"] * 1000.0 for e in rel]
                print(f"{model:<12}{side:<7}{len(E):>8}{len(rel):>9}"
                      f"{sum(e['end_over_box'] for e in rel):>10}"
                      f"{np.median(tb) if tb else float('nan'):>11.0f}/"
                      f"{np.percentile(tb, 90) if tb else float('nan'):<10.0f}"
                      f"{np.median([e['lift_mm'] for e in E]):>10.0f}"
                      f"{np.median([e['carry_m'] for e in E]) * 1000:>11.0f}"
                      f"{np.median([e['dur_s'] for e in E]):>10.1f}")

    if args.pair:
        # Run-to-run spread on this rig is large (a same-optics repeat of the 2026-09-06 board
        # moved 5 -> 9 placements on the same 8 seeds), so conditions are compared WITHIN a
        # seed and the verdict is how many seeds moved, not the totals.
        A, B = args.pair.split(",")
        per_seed = {}
        for r, (rd, _) in zip(res, jobs):
            name = pathlib.Path(rd).name
            model, _, seed = name.rpartition("_s")
            if model not in (A, B):
                continue
            rel = [e for e in r["episodes"] if e["end_kind"] == "released"]
            per_seed.setdefault(seed, {})[model] = dict(
                placed=r["placed"], carries=len(r["episodes"]),
                released=len(rel), over=sum(e["end_over_box"] for e in rel),
                to_box=[e["to_own_box_m"] for e in rel])
        seeds = sorted(s_ for s_, v in per_seed.items() if A in v and B in v)
        print(f"\n  seed-matched {A} vs {B}: {len(seeds)} seeds")
        print(f"  {'metric':<22}{A:>12}{B:>12}{'B-A':>8}   "
              f"{'seeds B>A / B<A':>16}   {'sign test p':>12}")
        print("  " + "-" * 86)
        for label, key in (("placed", "placed"), ("carries", "carries"),
                           ("releases", "released"), ("releases over box", "over")):
            a = [per_seed[s_][A][key] for s_ in seeds]
            b = [per_seed[s_][B][key] for s_ in seeds]
            up = sum(1 for x, y in zip(a, b) if y > x)
            dn = sum(1 for x, y in zip(a, b) if y < x)
            n = up + dn
            # two-sided exact sign test on the seeds that moved
            pv = (min(1.0, 2 * sum(math.comb(n, k) for k in range(min(up, dn) + 1)) / 2 ** n)
                  if n else 1.0)
            print(f"  {label:<22}{sum(a):>12}{sum(b):>12}{sum(b) - sum(a):>8}   "
                  f"{up:>7} / {dn:<6}   {pv:>12.3f}")
        for label, model in ((A, A), (B, B)):
            d = [x * 1000 for s_ in seeds for x in per_seed[s_][model]["to_box"]]
            if d:
                print(f"  {label} release->box: p50 {np.median(d):.0f} mm, "
                      f"p90 {np.percentile(d, 90):.0f} mm, over box "
                      f"{sum(1 for x in d if x <= 0)}/{len(d)}")

    for m in per_model.values():
        for k in ("lift_mm", "gap_min_mm", "carry_m", "end_z_mm", "dur_s"):
            v = m.pop(k)
            m[k + "_p50"] = round(float(np.median(v)), 3) if v else None
            m[k + "_p90"] = round(float(np.percentile(v, 90)), 3) if v else None
    json.dump(dict(thresholds=dict(held_r=HELD_R, closed_mm=CLOSED_MM,
                                   off_table=OFF_TABLE, lift_m=LIFT_M,
                                   open_mm=OPEN_MM, release_s=RELEASE_S, bridge=BRIDGE,
                                   min_ticks=MIN_TICKS),
                   per_model=per_model, runs=res), open(args.out, "w"), indent=1)
    print(f"\n  -> {args.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
