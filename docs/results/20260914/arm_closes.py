import json, collections
runs = {}
for f in ("A,B", "A,C", "A,D"):
    for r in json.load(open(f"/tmp/pb_full_{f}.json"))["runs"]:
        c, s = r["run"].rsplit("/", 1)[-1].split("_s"); runs[(c, int(s))] = r
print("attempts field example:", next(iter(runs.values()))["attempts"])
for c in "ABCD":
    tot = collections.Counter(); carries = collections.Counter(); first = collections.defaultdict(list)
    for (cc, s), r in runs.items():
        if cc != c: continue
        tot.update(r["attempts"])
        for e in r["episodes"]:
            carries[e["side"]] += 1
            first[e["side"]].append(e["t0"])
    fl = {a: (sorted(v)[len(v)//2] if v else None) for a, v in first.items()}
    print(c, "closes", dict(tot), "carries", dict(carries), "carry t0 p50", {a: round(x, 1) for a, x in fl.items() if x})
