import json, math, sys
old = json.load(open(sys.argv[1])); new = json.load(open(sys.argv[2]))
gray, black_ext, black_noext = [], [], []
for seed, rec in new.items():
    for c, a, b in zip(rec["colors"], rec["bolts"], old[seed]["bolts"]):
        d = math.dist(a["p"][:2], b["p"][:2]) * 1000
        if c == "gray": gray.append((d, seed))
        elif "settle_steps" in rec: black_ext.append(d)
        else: black_noext.append(d)
gray.sort()
q = lambda v, f: v[int(f * (len(v) - 1))]
print("gray xy displacement mm: p50 %.1f p90 %.1f p99 %.1f max %.1f (seed %s); >20 mm: %d of %d" % (
    q([g[0] for g in gray], .5), q([g[0] for g in gray], .9), q([g[0] for g in gray], .99), gray[-1][0], gray[-1][1],
    sum(g[0] > 20 for g in gray), len(gray)))
print("black xy displacement mm, seeds with normal 1000-step settle: max %.4f (n=%d)" % (max(black_noext), len(black_noext)))
print("black xy displacement mm, seeds with extended settle: p50 %.2f max %.2f (n=%d)" % (
    sorted(black_ext)[len(black_ext)//2], max(black_ext), len(black_ext)))
