"""All-11 G-photometry board: per-model funnel, per-arm outcomes, and seed-matched sign tests."""
import json, random, collections
from math import comb
HOST = {}
runs = {}
for h in ("gpu13", "gpu12"):
    for r in json.load(open(f"funnel_runs_{h}.json"))["runs"]:
        model, _, seed = r["run"].rsplit("/", 1)[-1].rpartition("_s")
        runs[(model, int(seed))] = r; HOST[model] = h
models = sorted(HOST)
seeds = sorted({s for _, s in runs})
SHORT = {"boltv2_40k": "plain_first", "boltv2_plain_40k": "plain_r4", "boltv2_r5_40k": "plain_r5", "boltv2ph3_40k": "ph3",
         "boltv2_griponly_40k": "griponly", "boltv2_veldrop50_40k": "veldrop50", "boltv2_griponly_r6_40k": "griponly_r6",
         "boltv2_griponly_r6knorm_40k": "r6knorm", "boltv2_griponly_r6knormcrop_40k": "r6knormcrop",
         "boltv2_griponly_devjit_40k": "devjit_r6", "boltv2_griponly_devjit_r7_40k": "devjit_r7"}
def per(r, key, arm=None):
    if key == "placed": return r["placed"]
    if key == "closes": return sum(r["attempts"].values()) if arm is None else r["attempts"].get(arm, 0)
    E = [e for e in r["episodes"] if (arm is None or e["side"] == arm)]
    L = [e for e in E if e["lifted"]]
    return {"carries": len(E), "lifted": len(L),
            "over": sum(e["end_kind"] == "released" and e["end_over_box"] for e in L),
            "off": sum(e["end_kind"] == "released" and not e["end_over_box"] for e in L),
            "correct": sum(e["rest_correct"] for e in L)}[key]
def tot(m, key, arm=None): return sum(per(runs[(m, s)], key, arm) for s in seeds)
def sign_p(a, b):
    n = a + b
    return 1.0 if n == 0 else min(1.0, 2 * sum(comb(n, i) for i in range(min(a, b) + 1)) / 2 ** n)
rng = random.Random(0)
def boot_ratio(m, arm):
    vals = []
    for _ in range(4000):
        bs = [rng.choice(seeds) for _ in seeds]
        L = sum(per(runs[(m, s)], "lifted", arm) for s in bs); O = sum(per(runs[(m, s)], "off", arm) for s in bs)
        if L: vals.append(O / L)
    vals.sort(); return vals[100], vals[3900]
order = sorted(models, key=lambda m: -tot(m, "placed"))
print(f"{'model':12s} {'host':5s} {'placed':>6} {'misp':>4} {'closes':>6} {'carries':>7} {'lifted':>6} | {'L corr':>6} {'L off/lift [95% CI]':>22} | {'R corr':>6} {'R off/lift':>10}")
for m in order:
    misp = sum(runs[(m, s)]["misplaced"] for s in seeds)
    lo, hi = boot_ratio(m, "left")
    lo_l = tot(m, "lifted", "left"); lo_r = tot(m, "lifted", "right")
    print(f"{SHORT[m]:12s} {HOST[m]:5s} {tot(m,'placed'):>6} {misp:>4} {tot(m,'closes'):>6} {tot(m,'carries'):>7} {tot(m,'lifted'):>6} | "
          f"{tot(m,'correct','left'):>6} {tot(m,'off','left')/max(lo_l,1):>8.2f} [{lo:.2f},{hi:.2f}] | "
          f"{tot(m,'correct','right'):>6} {tot(m,'off','right')/max(lo_r,1):>10.2f}")
print()
ref = "boltv2_griponly_devjit_40k"
print(f"seed-matched sign tests on placements vs {SHORT[ref]} (seeds other>ref / other<ref):")
for m in order:
    if m == ref: continue
    up = sum(per(runs[(m, s)], "placed") > per(runs[(ref, s)], "placed") for s in seeds)
    dn = sum(per(runs[(m, s)], "placed") < per(runs[(ref, s)], "placed") for s in seeds)
    print(f"  {SHORT[m]:12s} {tot(m,'placed'):>3} vs {tot(ref,'placed'):>3}   {up:>2}/{dn:<2}  p={sign_p(up, dn):.3f}")
print()
a, b = "boltv2_griponly_devjit_r7_40k", "boltv2_griponly_devjit_40k"
print("devjit_r7 vs devjit_r6, seed-matched (r7>r6 / r7<r6):")
for key, arm in (("placed", None), ("closes", None), ("lifted", None), ("correct", "left"), ("off", "left"), ("lifted", "left"), ("correct", "right"), ("lifted", "right")):
    up = sum(per(runs[(a, s)], key, arm) > per(runs[(b, s)], key, arm) for s in seeds)
    dn = sum(per(runs[(a, s)], key, arm) < per(runs[(b, s)], key, arm) for s in seeds)
    print(f"  {key:8s} {arm or 'both':5s} r7 {tot(a,key,arm):>4}  r6 {tot(b,key,arm):>4}   {up:>2}/{dn:<2} p={sign_p(up, dn):.3f}")
