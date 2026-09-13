#!/usr/bin/env python3
"""Fit the rig's cell lighting to the deployment photometry by random search.

Each trial renders 4 s of the shared stack and is scored on eight per-arm region statistics
measured from 36 deployment runs (18k wrist frames, 2026-09-10/11). The knobs are the ones the
sweeps showed actually move those statistics: the tonemapper is pinned to the hard clamp (op 0),
because Kit's default Iray-Reinhard rolls highlights off and no light level reproduces a clipped
fingertip through it.
"""
import itertools, json, os, random, subprocess, sys, time
from pathlib import Path

import cv2
import numpy as np

ROOT = Path("/home/plaif/workspace/simulation_rb5")
OUT = Path(os.environ.get("FIT_OUT", "/home/plaif/fit_light"))
GPUS = ["0", "1", "4"]
N_TRIALS = int(sys.argv[1]) if len(sys.argv) > 1 else 60
SEED = int(sys.argv[2]) if len(sys.argv) > 2 else 0

# (tab.m, tab95, tip.p50, tipclip, top.m, topclip) per arm, from the deployment logs.
TARGET = {"left":  (110.1, 176.4, 96.8, 10.75, 116.4, 3.49),
          "right": (106.6, 167.2, 87.0,  8.60, 116.0, 2.03)}
# clip fractions are the whole point of the fit, so they carry the most weight; they are compared
# on a log scale because 0.1% vs 10% is the failure and 10% vs 12% is not.
W = (1.0, 1.0, 1.0, 3.0, 1.0, 2.0)
LOGSCALE = (False, False, False, True, False, True)


def stats(path):
    cap = cv2.VideoCapture(str(path))
    n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if n < 5:
        cap.release()
        return None
    S = []
    for i in np.linspace(5, n - 5, 8).astype(int):
        cap.set(cv2.CAP_PROP_POS_FRAMES, int(i))
        ok, fr = cap.read()
        if not ok:
            continue
        y = 0.2126 * fr[..., 2] + 0.7152 * fr[..., 1] + 0.0722 * fr[..., 0]
        y = y.astype(np.float32)
        h = y.shape[0]
        tab, tip, top = y[int(h * .35):int(h * .75)], y[int(h * .84):], y[:int(h * .20)]
        S.append([tab.mean(), np.percentile(tab, 95), np.percentile(tip, 50),
                  (tip > 250).mean() * 100, top.mean(), (top > 250).mean() * 100])
    cap.release()
    return np.asarray(S).mean(0) if S else None


def score(m, arm):
    t = TARGET[arm]
    s = 0.0
    for v, tv, w, lg in zip(m, t, W, LOGSCALE):
        if lg:
            e = (np.log10(max(v, 0.01) + 0.05) - np.log10(tv + 0.05)) / 0.5
        else:
            e = (v - tv) / max(tv, 1.0)
        s += w * e * e
    return s


def sample(rng):
    # Round 1 bottomed out against its own box/table albedo floor and kept a large dome, which is
    # what lights the far field: every good trial had top.m ~200 against the deployment's 116.
    # Round 2 therefore drops the albedo floors and lets the dome go an order of magnitude lower,
    # and parameterises an overall scale separately from the three lights' ratios, because with a
    # hard clamp the absolute level is what sets every clip fraction.
    S = 10 ** rng.uniform(-0.8, 0.8)
    return {
        "key": S * 10 ** rng.uniform(-0.7, 0.7),
        "key_angle": rng.uniform(5, 45),
        "dome": S * 10 ** rng.uniform(-1.6, 0.3),
        "sun": S * 10 ** rng.uniform(-0.7, 1.2),
        "sun_angle": rng.uniform(0.5, 6),
        "sun_az": rng.uniform(0, 360),
        "sun_el": rng.uniform(12, 65),
        "box_alb": rng.uniform(0.10, 0.40),
        "table_alb": rng.uniform(0.14, 0.40),
        "insert_alb": rng.uniform(0.08, 0.28),
        "green_scale": rng.uniform(0.35, 1.0),
    }


def env_for(p):
    b, i, g = p["box_alb"], p["insert_alb"], p["green_scale"]
    mat = (f"table.metallic=0;riser.metallic=0;boxgray.metallic=0;"
           f"table.rgb={p['table_alb']:.3f},{p['table_alb'] + .02:.3f},{p['table_alb'] + .03:.3f};"
           f"boxgray.rgb={b:.3f},{b + .01:.3f},{b + .03:.3f};"
           f"insert.rgb={i:.3f},{i:.3f},{i + .02:.3f};"
           f"green.rgb={0.10 * g:.3f},{0.38 * g:.3f},{0.27 * g:.3f}")
    return {"EVAL_RTX": "/rtx/post/tonemap/op=0", "EVAL_MAT": mat,
            "EVAL_KEY_INTENSITY": f"{p['key']:.4f}", "EVAL_KEY_ANGLE": f"{p['key_angle']:.2f}",
            "EVAL_DOME_INTENSITY": f"{p['dome']:.4f}", "EVAL_SUN_INTENSITY": f"{p['sun']:.4f}",
            "EVAL_SUN_ANGLE": f"{p['sun_angle']:.2f}", "EVAL_SUN_AZIMUTH": f"{p['sun_az']:.1f}",
            "EVAL_SUN_ELEVATION": f"{p['sun_el']:.1f}"}


def launch(tag, p, gpu):
    d = OUT / tag
    subprocess.run(["rm", "-rf", str(d), str(ROOT / "outputs/shared_stack" / f"fit_{tag}")])
    d.mkdir(parents=True, exist_ok=True)  # record_shared_stack insists on creating diag/ itself
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
    done, best = {}, None
    if res_path.exists():          # resume: a killed driver must not throw away paid-for renders
        for line in res_path.read_text().splitlines():
            r = json.loads(line)
            done[r["tag"]] = r
            if r["score"] != float("inf") and (best is None or r["score"] < best["score"]):
                best = r
        trials = [(t, p) for t, p in trials if t not in done]
        print(f"resuming: {len(done)} done, {len(trials)} left, best={best and best['score']:.2f}",
              flush=True)
    for batch in range(0, len(trials), len(GPUS)):
        chunk = trials[batch:batch + len(GPUS)]
        procs = [(tag, p, launch(tag, p, g)) for (tag, p), g in zip(chunk, GPUS)]
        for tag, p, pr in procs:
            # Isaac sometimes hangs on shutdown with non-daemon threads still alive, well after
            # the episode and the video are written. Killing it costs nothing: the measurement
            # reads the mp4 from disk, not the process.
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
                m = stats(OUT / tag / "diag" / f"{arm}_wrist.mp4")
                if m is None:
                    tot = float("inf")
                    break
                rec[arm] = [round(float(v), 2) for v in m]
                tot += score(m, arm)
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
