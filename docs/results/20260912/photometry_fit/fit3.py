#!/usr/bin/env python3
"""Round 3 of the cell-photometry fit: whole-frame CDF matching with the material ranges the
diagnostic actually implicated.

Rounds 1-2 scored six region statistics and both plateaued with the work surface railed
(tab p95 = 255) at every setting. Painting the clipped pixels showed why: the table top blows out
as one solid block and the gray bolts render as polished chrome (metallic 1.0). The table's
diffuseColor is a linear albedo (0.42) while the foam pad next to it comes from an sRGB texture
that lands near 0.10 linear, so at any exposure that puts the pad at 110 the table is at ~300.
Both fixes are outside round 1-2's sampling ranges, and a percentile objective could not see a
large saturated block anyway -- the CDF can, as mass in the top bin.
"""
import json, os, random, subprocess, sys, time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path("/home/plaif/workspace/simulation_rb5")
OUT = Path(os.environ.get("FIT_OUT", "/home/plaif/fit3"))
GPUS = ["0", "1", "4"]
N_TRIALS = int(sys.argv[1]) if len(sys.argv) > 1 else 90
SEED = int(sys.argv[2]) if len(sys.argv) > 2 else 11

REF = json.load(open("/home/plaif/real_hist.json"))
BINS = np.asarray(REF["bins"], dtype=float)
REF_H = {k: np.asarray(v) for k, v in REF["hist"].items()}
REF_C = {k: np.cumsum(v) for k, v in REF_H.items()}


def hist(path):
    cap = cv2.VideoCapture(str(path))
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if n < 5:
        cap.release()
        return None
    H = np.zeros(len(BINS) - 1)
    for i in np.linspace(5, n - 5, 10).astype(int):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(i))
        ok, fr = cap.read()
        if not ok:
            continue
        y = 0.2126 * fr[..., 2] + 0.7152 * fr[..., 1] + 0.0722 * fr[..., 0]
        H += np.histogram(y.astype(np.float32), bins=BINS)[0]
    cap.release()
    return H / H.sum() if H.sum() else None


def score(h, arm):
    # 1-Wasserstein between CDFs, in luminance units, plus a log penalty on the saturated mass
    # because matching *how much* rails is the whole point and it is only 2-3% of the frame.
    c = np.cumsum(h)
    w1 = float(np.abs(c - REF_C[arm]).sum() * (BINS[1] - BINS[0]))
    sat = np.log10(max(h[-1], 1e-4) + 1e-3) - np.log10(REF_H[arm][-1] + 1e-3)
    return w1 + 30.0 * sat * sat


def sample(rng):
    S = 10 ** rng.uniform(-0.6, 0.6)
    return {
        "key": S * 10 ** rng.uniform(-0.7, 0.7),
        "key_angle": rng.uniform(8, 45),
        "dome": S * 10 ** rng.uniform(-1.4, 0.2),
        "sun": S * 10 ** rng.uniform(-0.5, 1.2),
        "sun_angle": rng.uniform(0.5, 6),
        "sun_az": rng.uniform(0, 360),
        "sun_el": rng.uniform(15, 65),
        "table_alb": rng.uniform(0.04, 0.20),
        "box_alb": rng.uniform(0.06, 0.30),
        "insert_alb": rng.uniform(0.04, 0.20),
        "green_scale": rng.uniform(0.25, 0.90),
        "bg_alb": rng.uniform(0.22, 0.55),
        "bg_metal": rng.uniform(0.0, 0.35),
        "bg_rough": rng.uniform(0.35, 0.85),
        "bb_metal": rng.uniform(0.0, 0.35),
    }


def env_for(p):
    t, b, i, g = p["table_alb"], p["box_alb"], p["insert_alb"], p["green_scale"]
    v = p["bg_alb"]
    mat = (f"table.metallic=0;riser.metallic=0;boxgray.metallic=0;"
           f"table.rgb={t:.3f},{t + .01:.3f},{t + .015:.3f};"
           f"boxgray.rgb={b:.3f},{b + .01:.3f},{b + .03:.3f};"
           f"insert.rgb={i:.3f},{i:.3f},{i + .02:.3f};"
           f"green.rgb={0.10 * g:.3f},{0.38 * g:.3f},{0.27 * g:.3f};"
           f"bolt_gray.rgb={v:.3f},{v + .01:.3f},{v + .03:.3f};"
           f"bolt_gray.metallic={p['bg_metal']:.3f};bolt_gray.rough={p['bg_rough']:.3f};"
           f"bolt_black.metallic={p['bb_metal']:.3f}")
    return {"EVAL_RTX": "/rtx/post/tonemap/op=0", "EVAL_MAT": mat,
            "EVAL_KEY_INTENSITY": f"{p['key']:.4f}", "EVAL_KEY_ANGLE": f"{p['key_angle']:.2f}",
            "EVAL_DOME_INTENSITY": f"{p['dome']:.4f}", "EVAL_SUN_INTENSITY": f"{p['sun']:.4f}",
            "EVAL_SUN_ANGLE": f"{p['sun_angle']:.2f}", "EVAL_SUN_AZIMUTH": f"{p['sun_az']:.1f}",
            "EVAL_SUN_ELEVATION": f"{p['sun_el']:.1f}"}


def launch(tag, p, gpu):
    d = OUT / tag
    subprocess.run(["rm", "-rf", str(d), str(ROOT / "outputs/shared_stack" / f"fit_{tag}")])
    d.mkdir(parents=True, exist_ok=True)
    env = dict(os.environ, OMNI_KIT_ACCEPT_EULA="YES", CUDA_VISIBLE_DEVICES=gpu,
               PYTHONPATH="/home/plaif/workspace/openpi/packages/openpi-client/src",
               LD_LIBRARY_PATH="/home/plaif/simrig/lib", **env_for(p))
    log = open(d / "run.log", "w")
    return subprocess.Popen(
        ["/home/plaif/isaac_rig/.venv/bin/python", "-u", "scripts/record_shared_stack.py",
         "--diagnostics-dir", str(d / "diag"), "--shared-stack", "--episodes", "1",
         "--episode-sec", "4", "--port", "8044", "--scene-states",
         str(ROOT / "assets/scene_states40_aligned_rb5_foam.json"), "--seed", "100",
         "--n-per-color", "10", "--layout", "aligned", "--tag", f"fit_{tag}"],
        cwd=ROOT, env=env, stdout=log, stderr=subprocess.STDOUT)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    rng = random.Random(SEED)
    trials = [(f"t{i:03d}", sample(rng)) for i in range(N_TRIALS)]
    res_path = OUT / "results.jsonl"
    done, best = set(), None
    if res_path.exists():
        for line in res_path.read_text().splitlines():
            r = json.loads(line)
            done.add(r["tag"])
            if r["score"] != float("inf") and (best is None or r["score"] < best["score"]):
                best = r
        trials = [(t, p) for t, p in trials if t not in done]
        print(f"resuming: {len(done)} done, {len(trials)} left", flush=True)
    for batch in range(0, len(trials), len(GPUS)):
        chunk = trials[batch:batch + len(GPUS)]
        procs = [(tag, p, launch(tag, p, g)) for (tag, p), g in zip(chunk, GPUS)]
        for tag, p, pr in procs:
            try:
                pr.wait(timeout=300)
            except subprocess.TimeoutExpired:
                pr.kill()
                pr.wait(timeout=60)
                print(f"  [warn] {tag} killed after 300 s (shutdown hang)", flush=True)
        for tag, p, _ in procs:
            rec = {"tag": tag, "params": p}
            tot = 0.0
            for arm in ("left", "right"):
                h = hist(OUT / tag / "diag" / f"{arm}_wrist.mp4")
                if h is None:
                    tot = float("inf")
                    break
                c = np.cumsum(h)
                rec[arm] = {"p10": float(BINS[np.searchsorted(c, .10)]),
                            "p50": float(BINS[np.searchsorted(c, .50)]),
                            "p90": float(BINS[np.searchsorted(c, .90)]),
                            "sat": round(float(h[-1] * 100), 2)}
                tot += score(h, arm)
            rec["score"] = tot
            with res_path.open("a") as f:
                f.write(json.dumps(rec) + "\n")
            if best is None or tot < best["score"]:
                best = rec
        print(f"[{time.strftime('%H:%M:%S')}] {batch + len(chunk)}/{len(trials)} "
              f"best={best['score']:.2f} ({best['tag']})", flush=True)
    print("BEST", json.dumps(best, indent=1))
    print("DONE", flush=True)


main()
