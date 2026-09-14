import json, math, sys
old = json.load(open(sys.argv[1])); new = json.load(open(sys.argv[2]))
worst = {"gray": 0.0, "black": 0.0}; tilt = {"gray": [], "black": []}
for seed, rec in new.items():
    o = old[seed]
    assert rec["colors"] == o["colors"]
    for c, a, b in zip(rec["colors"], rec["bolts"], o["bolts"]):
        worst[c] = max(worst[c], math.dist(a["p"], b["p"]))
        w, x, y, z = a["q"]
        axis_z = 2 * (x * z - y * w)                 # bolt local X in world z
        tilt[c].append(math.degrees(math.asin(max(-1, min(1, axis_z)))))
print("max position change vs historical frozen scenes (mm):", {k: round(v * 1000, 2) for k, v in worst.items()})
for c, t in tilt.items():
    t = sorted(t); print(f"{c}: axis tilt from horizontal (deg) p10 {t[len(t)//10]:.1f} p50 {t[len(t)//2]:.1f} p90 {t[9*len(t)//10]:.1f}")
print("seeds needing extended settle:", {s: r["settle_steps"] for s, r in new.items() if "settle_steps" in r})
print("all tagged with geometry:", all(r.get("bolt_geometry", {}).get("name") == "iso_heads_20260914" for r in new.values()))
