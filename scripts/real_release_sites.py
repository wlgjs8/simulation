"""Where does the real robot open the jaw? The hardware side of the sim carry funnel.

analyze_grasp_funnel.py found that in sim, griponly opens its jaw a median 140 mm short of
the box while every other model releases directly over it. That is only a sim-vs-real gap if
the real robot does something different, and the deployed runner already logs enough to check:
policy_steps.jsonl carries, per policy step and per arm, the commanded and measured jaw
percent and the measured TCP pose.

So: find every closed segment, take the TCP where it re-opens, and measure the distance to
that arm's box footprint. Same number, same units as the sim funnel's `to_own_box_m`.

Box coordinates come from the SIM cell model (eval_closed_loop.py BOX_X/BOX_CY), which was
built to match the real cell but was never surveyed against it -- so read the two arms and the
two eras against each other, not as absolute placement accuracy.
"""
import argparse
import glob
import json

import numpy as np

BOX_X, BOX_HW, BOX_HD = 0.765, 0.120, 0.190
BOX_CY = {"left": +0.260, "right": -0.260}      # left = gray box, right = black box
CLOSED_PCT = 15.0     # jaw command below this = closed on something
MIN_HOLD_S = 0.4      # shorter is a twitch, not a carry


def segments(closed, t):
    out, i = [], 0
    while i < len(closed):
        if closed[i]:
            j = i
            while j + 1 < len(closed) and closed[j + 1]:
                j += 1
            if t[j] - t[i] > MIN_HOLD_S:
                out.append((i, j))
            i = j + 1
        else:
            i += 1
    return out


def read(path):
    rows = []
    for line in open(path):
        line = line.strip()
        if not line:
            continue
        try:
            d = json.loads(line)
        except json.JSONDecodeError:
            continue                       # a rollout killed mid-write ends in a partial line
        a = d.get("arms")
        if not a:
            continue                       # chunks.jsonl rows have no per-arm state
        if all(a.get(s, {}).get("gripper_cmd_pct") is not None and a[s].get("meas_pose")
               for s in ("left", "right")):
            rows.append(d)
    return rows


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("logs", nargs="+", help="policy_steps .jsonl files (globs ok)")
    ap.add_argument("--min-steps", type=int, default=200)
    # An empty full close reads about -2% on the real jaw while a held bolt stalls at 6-9%
    # (bolt_v2 campaign, measured). Without this filter a real "close" is not comparable to a
    # sim CARRY, which requires a bolt actually between the jaws.
    ap.add_argument("--held-min-meas", type=float, default=None,
                    help="keep only closes whose measured minimum is at least this percent")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    paths = sorted({p for g in args.logs for p in glob.glob(g)})
    per_arm = {"left": [], "right": []}
    sessions = []
    for path in paths:
        rows = read(path)
        if len(rows) < args.min_steps:
            continue
        t = np.array([r["t_mono"] for r in rows], dtype=float)
        t = (t - t[0]) * (1e-9 if (t[-1] - t[0]) > 1e6 else 1.0)
        rec = {"log": path.split("/")[-1], "steps": len(rows), "dur_s": round(float(t[-1]), 1)}
        for side in ("left", "right"):
            g = np.array([float(r["arms"][side]["gripper_cmd_pct"]) for r in rows])
            gm = np.array([float(r["arms"][side].get("gripper_meas_pct") or np.nan)
                           for r in rows])
            P = np.array([r["arms"][side]["meas_pose"][:3] for r in rows], dtype=float)
            events = []
            for i, j in segments(g < CLOSED_PCT, t):
                p = P[j]
                d = float(np.hypot(max(0.0, abs(p[0] - BOX_X) - BOX_HW),
                                   max(0.0, abs(p[1] - BOX_CY[side]) - BOX_HD)))
                events.append(dict(t_open=round(float(t[j]), 1),
                                   hold_s=round(float(t[j] - t[i]), 2),
                                   to_box_m=round(d, 3), over_box=bool(d <= 0.0),
                                   open_z_mm=round(float(p[2]) * 1000.0, 1),
                                   min_meas_pct=round(float(np.nanmin(gm[i:j + 1])), 1)))
            if args.held_min_meas is not None:
                events = [e for e in events if e["min_meas_pct"] >= args.held_min_meas]
            per_arm[side] += events
            rec[side] = dict(closes=len(events),
                             over_box=sum(e["over_box"] for e in events),
                             to_box_p50_mm=round(float(np.median([e["to_box_m"] for e in events]))
                                                 * 1000.0, 0) if events else None,
                             hold_p50_s=round(float(np.median([e["hold_s"] for e in events])), 1)
                             if events else None)
        sessions.append(rec)
        print(f"{rec['log']:<45}{rec['dur_s']:>7.1f}s  " +
              "  ".join(f"{s[0].upper()} {rec[s]['closes']:2d} closes, "
                        f"{rec[s]['over_box']:2d} over box, "
                        f"{rec[s]['to_box_p50_mm'] if rec[s]['to_box_p50_mm'] is not None else -1:4.0f} mm"
                        for s in ("left", "right")))

    print(f"\n{'arm':<7}{'closes':>7}{'over box':>10}{'to-box p50/p90 (mm)':>22}{'hold p50 (s)':>14}"
          f"{'min meas p50 (%)':>18}")
    print("-" * 78)
    for side in ("left", "right"):
        E = per_arm[side]
        if not E:
            continue
        d = [e["to_box_m"] * 1000 for e in E]
        print(f"{side:<7}{len(E):>7}{sum(e['over_box'] for e in E):>10}"
              f"{np.median(d):>11.0f}/{np.percentile(d, 90):<10.0f}"
              f"{np.median([e['hold_s'] for e in E]):>14.1f}"
              f"{np.median([e['min_meas_pct'] for e in E]):>18.1f}")
    if args.out:
        json.dump(dict(sessions=sessions, events=per_arm), open(args.out, "w"), indent=1)
        print(f"\n  -> {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
