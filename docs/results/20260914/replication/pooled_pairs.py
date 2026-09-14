"""Seed-matched sign tests pooled over the separation board (gpu12) and this replication (gpu13)."""
import json
from math import comb
def load(p):
    out = {}
    for r in json.load(open(p))["runs"]:
        c, s = r["run"].rsplit("/", 1)[-1].split("_s"); out[(c, int(s))] = r
    return out
boards = {"sep": load("../separation/funnel_runs.json"), "rep": load("funnel_runs.json")}
def sign_p(a, b):
    n = a + b
    return 1.0 if n == 0 else min(1.0, 2 * sum(comb(n, i) for i in range(min(a, b) + 1)) / 2 ** n)
def metric(r, key, arm=None):
    if key == "placed": return r["placed"]
    E = [e for e in r["episodes"] if e["side"] == arm and e["lifted"]]
    if key == "correct": return sum(e["rest_correct"] for e in E)
    if key == "off": return sum(e["end_kind"] == "released" and not e["end_over_box"] for e in E)
    if key == "lifted": return len(E)
print("pooled over both batches (80 seed pairs):")
for base, x, key, arm in (("A","G","placed",None), ("A","E","placed",None), ("A","D","placed",None),
                          ("D","E","placed",None), ("D","G","placed",None),
                          ("D","G","correct","right"), ("D","E","correct","left"), ("A","G","correct","left"),
                          ("A","G","off","left"), ("A","D","off","left"), ("A","E","off","left")):
    up = dn = tb = tx = 0; per = []
    for name, b in boards.items():
        seeds = sorted({s for c, s in b if c == base})
        u = sum(metric(b[(x, s)], key, arm) > metric(b[(base, s)], key, arm) for s in seeds)
        d = sum(metric(b[(x, s)], key, arm) < metric(b[(base, s)], key, arm) for s in seeds)
        up += u; dn += d
        tb += sum(metric(b[(base, s)], key, arm) for s in seeds); tx += sum(metric(b[(x, s)], key, arm) for s in seeds)
        per.append(f"{name} {u}/{d}")
    print(f"  {base}->{x} {key:7} {arm or 'both':5}  total {tb:>3} -> {tx:>3}   seeds up/down {up}/{dn}  p={sign_p(up, dn):.4f}   ({', '.join(per)})")
