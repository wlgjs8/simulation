import json, random, collections
runs = {}
for f in ("A,B", "A,C", "A,D"):
    for r in json.load(open(f"/tmp/pb_full_{f}.json"))["runs"]:
        name = r["run"].rsplit("/", 1)[-1]
        c, seed = name.split("_s")
        runs[(c, int(seed))] = r
seeds = sorted({s for _, s in runs})
def arm_counts(r, arm):
    E = [e for e in r["episodes"] if e["side"] == arm and e["lifted"]]
    return dict(lifted=len(E),
                over=sum(e["end_kind"] == "released" and e["end_over_box"] for e in E),
                off=sum(e["end_kind"] == "released" and not e["end_over_box"] for e in E),
                slip=sum(e["end_kind"] == "slipped" and e["t1"] <= 30.0 for e in E),
                trunc=sum(e["t1"] > 30.0 for e in E),
                trunc_over=sum(e["t1"] > 30.0 and e["end_over_box"] for e in E),
                correct=sum(e["rest_correct"] for e in E))
print("cond arm   lifted  rel_over rel_off slip  held@30s(over box)  rest_correct  off/lifted [95% CI seeds-bootstrap]")
rng = random.Random(0)
tab = {}
for c in "ABCD":
    for arm in ("left", "right"):
        per = {s: arm_counts(runs[(c, s)], arm) for s in seeds}
        tot = collections.Counter()
        for v in per.values(): tot.update(v)
        tab[(c, arm)] = per
        boots = []
        for _ in range(4000):
            bs = [rng.choice(seeds) for _ in seeds]
            L = sum(per[s]["lifted"] for s in bs); O = sum(per[s]["off"] for s in bs)
            if L: boots.append(O / L)
        boots.sort()
        ratio = tot["off"] / max(tot["lifted"], 1)
        print(f"{c:4} {arm:5} {tot['lifted']:>6} {tot['over']:>9} {tot['off']:>7} {tot['slip']:>4} {tot['trunc']:>8}({tot['trunc_over']})"
              f"{tot['correct']:>14}   {ratio:.2f} [{boots[100]:.2f}, {boots[3900]:.2f}]")
# paired sign test on per-seed off-box releases after lift, left arm, A vs X
from math import comb
def sign_p(a, b):
    n = a + b
    if n == 0: return 1.0
    k = min(a, b)
    return min(1.0, 2 * sum(comb(n, i) for i in range(k + 1)) / 2 ** n)
for arm in ("left", "right"):
    for x in "BCD":
        for key in ("off", "lifted", "correct"):
            up = sum(tab[(x, arm)][s][key] > tab[("A", arm)][s][key] for s in seeds)
            dn = sum(tab[(x, arm)][s][key] < tab[("A", arm)][s][key] for s in seeds)
            print(f"{arm:5} A vs {x} {key:8} seeds {x}>A {up:>2} / {x}<A {dn:>2}  p={sign_p(up, dn):.3f}")
