import json, random, collections
from math import comb
runs = {}
for f in ("A,D", "A,E", "A,G"):
    for r in json.load(open(f"/tmp/rb3_{f}.json"))["runs"]:
        c, s = r["run"].rsplit("/", 1)[-1].split("_s"); runs[(c, int(s))] = r
C = "ADEG"
seeds = sorted({s for _, s in runs})
def counts(r, arm):
    E = [e for e in r["episodes"] if e["side"] == arm and e["lifted"]]
    return dict(closes=r["attempts"].get(arm, 0),
                carries=sum(e["side"] == arm for e in r["episodes"]),
                lifted=len(E),
                over=sum(e["end_kind"] == "released" and e["end_over_box"] for e in E),
                off=sum(e["end_kind"] == "released" and not e["end_over_box"] for e in E),
                slip=sum(e["end_kind"] == "slipped" and e["t1"] <= 30.0 for e in E),
                held30=sum(e["t1"] > 30.0 for e in E),
                correct=sum(e["rest_correct"] for e in E))
tab = {(c, arm): {s: counts(runs[(c, s)], arm) for s in seeds} for c in C for arm in ("left", "right")}
rng = random.Random(0)
print("cond arm   closes carries lifted  over  off slip held30 correct  off/lifted [95% CI]  closes->carry")
for c in C:
    for arm in ("left", "right"):
        per = tab[(c, arm)]; tot = collections.Counter()
        for v in per.values(): tot.update(v)
        boots = []
        for _ in range(4000):
            bs = [rng.choice(seeds) for _ in seeds]
            L = sum(per[s]["lifted"] for s in bs); O = sum(per[s]["off"] for s in bs)
            if L: boots.append(O / L)
        boots.sort()
        print(f"{c:4} {arm:5} {tot['closes']:>6} {tot['carries']:>7} {tot['lifted']:>6} {tot['over']:>5} {tot['off']:>4} {tot['slip']:>4} {tot['held30']:>6} {tot['correct']:>7}"
              f"   {tot['off']/max(tot['lifted'],1):.2f} [{boots[100]:.2f},{boots[3900]:.2f}]   {tot['carries']/max(tot['closes'],1):.2f}")
def sign_p(a, b):
    n = a + b
    return 1.0 if n == 0 else min(1.0, 2 * sum(comb(n, i) for i in range(min(a, b) + 1)) / 2 ** n)
print()
for base, x in (("A","D"),("A","E"),("A","G"),("D","E"),("D","G")):
    for arm in ("left", "right"):
        cells = []
        for key in ("correct", "off", "lifted"):
            up = sum(tab[(x, arm)][s][key] > tab[(base, arm)][s][key] for s in seeds)
            dn = sum(tab[(x, arm)][s][key] < tab[(base, arm)][s][key] for s in seeds)
            cells.append(f"{key} {up:>2}/{dn:<2} p={sign_p(up, dn):.3f}")
        print(f"{base} vs {x} {arm:5}  " + "   ".join(cells))
