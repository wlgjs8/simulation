"""Render the SAME wrist-camera moment under several photometry variants, for eye comparison.

A closed-loop episode runs under the stock rig, exactly as a board episode does. Every
--ladder-every-sec of simulated time the physics is held (renders use delta_time=0 and the step
index is checked), and each variant in the spec is applied in turn -- tonemapper, lights,
materials -- rendered to lossless PNG for both wrists, and then the stock state is restored and
re-rendered before the policy's own frame is returned. The policy therefore only ever sees stock
frames, and every variant of one snapshot shares the geometry pixel for pixel: the only thing
that differs between the columns of a montage is the photometry.

Spec (JSON):
  {"every_sec": 1.0,
   "variants": [
     {"label": "A_stock"},
     {"label": "D_fit", "preset": "config/photometry/cell_fit_20260912_t013.json"},
     {"label": "B_tonemap", "env": {"EVAL_RTX": "/rtx/post/tonemap/op=0"}, "exposure_match": "D_fit"},
     ...]}
A variant's env uses the eval_closed_loop knob names (EVAL_RTX, EVAL_MAT, EVAL_KEY_INTENSITY,
...); a preset's env is loaded first and the variant's own env overrides it; anything unset is the
stock value captured from the running stage. "exposure_match": L scales the key/dome/sun
intensities once, at the first snapshot, until the pooled median luminance of both wrists equals
variant L's at that snapshot, and then holds that scale -- a fixed exposure, like the railed D405.
"bolt_visual": "plain" | "threaded" switches the bolts between the cylinder look and the render-only
threaded shaft; it needs the episode launched with EVAL_BOLT_VISUAL=threaded so the thread prims
exist (the collider is the same cylinder either way).

The episode itself must be launched with NO photometry knobs, so that "stock" is what the stage
holds when the hook is installed; this is checked.

Output: <ladder-dir>/ladder.jsonl (one record per snapshot: time, per-arm TCP, gripper %, nearest
bolt and its colour/height, per-variant settle residual), <ladder-dir>/s###/<label>_<side>.png,
<ladder-dir>/ladder_meta.json.

Run (arguments after --ladder-* are passed to eval_closed_loop.py):
  OMNI_KIT_ACCEPT_EULA=YES python scripts/photometry_ladder.py --ladder-dir /tmp/ladder \\
      --ladder-spec config/photometry/ladder_20260913.json --shared-stack --episodes 1 \\
      --episode-sec 60 --port 8048 --scene-states assets/scene_states40_aligned_rb5_foam.json \\
      --seed 100 --n-per-color 10 --layout aligned --tag ladder_s100
"""
import argparse
import json
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
_PHOTOMETRY_ENV = ("EVAL_PHOTOMETRY", "EVAL_RTX", "EVAL_MAT", "EVAL_KEY_INTENSITY", "EVAL_KEY_ANGLE",
                   "EVAL_DOME_INTENSITY", "EVAL_SUN_INTENSITY", "EVAL_SUN_ANGLE",
                   "EVAL_SUN_AZIMUTH", "EVAL_SUN_ELEVATION")

parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
parser.add_argument("--ladder-dir", required=True, type=Path)
parser.add_argument("--ladder-spec", required=True, type=Path)
parser.add_argument("--ladder-settle", type=int, default=8,
                    help="orchestrator steps per variant before the frame is kept")
parser.add_argument("--ladder-max", type=int, default=200, help="stop taking snapshots after this many")
options, forwarded = parser.parse_known_args()
if "--shared-stack" not in forwarded:
    parser.error("the ladder hooks the shared stack; pass --shared-stack")
_set = [k for k in _PHOTOMETRY_ENV if os.environ.get(k, "").strip()]
if _set:
    parser.error(f"launch the episode under the stock rig; these photometry knobs are set: {_set}")
OUT = options.ladder_dir.resolve()
OUT.mkdir(parents=True, exist_ok=False)
SPEC = json.loads(options.ladder_spec.read_text())
sys.argv = [sys.argv[0], *forwarded]
sys.path.insert(0, str(ROOT / "scripts"))
import shared_stack  # noqa: E402

LIGHT_KEYS = {"EVAL_KEY_INTENSITY": "key", "EVAL_KEY_ANGLE": "key_angle", "EVAL_DOME_INTENSITY": "dome",
              "EVAL_SUN_INTENSITY": "sun", "EVAL_SUN_ANGLE": "sun_angle",
              "EVAL_SUN_AZIMUTH": "sun_az", "EVAL_SUN_ELEVATION": "sun_el"}


def _parse_mat(text):
    out = {}
    for item in (text or "").split(";"):
        item = item.strip()
        if not item:
            continue
        lhs, _, rhs = item.partition("=")
        name, _, field = lhs.strip().partition(".")
        out.setdefault(name, {})[field] = (tuple(float(v) for v in rhs.split(","))
                                           if field == "rgb" else float(rhs))
    return out


def _variant_env(v):
    env = {}
    if v.get("preset"):
        path = Path(v["preset"])
        path = path if path.is_absolute() else ROOT / path
        env.update({k: str(x) for k, x in json.loads(path.read_text())["env"].items()})
    env.update({k: str(x) for k, x in v.get("env", {}).items()})
    unknown = set(env) - set(_PHOTOMETRY_ENV)
    if unknown:
        raise SystemExit(f"ladder variant {v['label']}: unknown knobs {sorted(unknown)}")
    return env


original_run = shared_stack.run_episode


def instrumented(args, settings, scene, world, rep, arts, cameras, tcp_views,
                 bolt_views, bolt_prims, overview, materials):
    import carb
    import numpy as np
    from PIL import Image
    from pxr import Gf, UsdGeom, UsdLux, UsdShade

    stage = world.stage
    st = carb.settings.get_settings()
    dt = world.get_physics_dt()

    # ---- the stock state, captured from the running stage -------------------------------------
    if stage.GetPrimAtPath("/World/sun").IsValid():
        raise SystemExit("ladder: a sun prim exists, so the episode was not launched stock")
    key = UsdLux.DistantLight(stage.GetPrimAtPath("/World/key"))
    dome = UsdLux.DomeLight(stage.GetPrimAtPath("/World/dome"))
    shaders = {name: UsdShade.Shader(stage.GetPrimAtPath(str(m.GetPath()) + "/S"))
               for name, m in materials.items()}
    stock = {
        "rtx": {},
        "light": {"key": float(key.GetIntensityAttr().Get()), "key_angle": float(key.GetAngleAttr().Get()),
                  "dome": float(dome.GetIntensityAttr().Get()), "sun": 0.0, "sun_angle": 1.5,
                  "sun_az": 0.0, "sun_el": 35.0},
        "mat": {name: {"rgb": tuple(float(c) for c in sh.GetInput("diffuseColor").Get()),
                       "rough": float(sh.GetInput("roughness").Get()),
                       "metallic": float(sh.GetInput("metallic").Get())}
                for name, sh in shaders.items()},
    }
    variants = []
    for v in SPEC["variants"]:
        env = _variant_env(v)
        rtx = {}
        for item in env.get("EVAL_RTX", "").split(";"):
            if item.strip():
                k, _, raw = item.partition("=")
                rtx[k.strip()] = raw.strip()
        light = dict(stock["light"])
        light.update({LIGHT_KEYS[k]: float(x) for k, x in env.items() if k in LIGHT_KEYS})
        mat = {n: dict(f) for n, f in stock["mat"].items()}
        for n, fields in _parse_mat(env.get("EVAL_MAT")).items():
            if n not in mat:
                raise SystemExit(f"ladder variant {v['label']}: unknown material {n}")
            mat[n].update(fields)
        variants.append({"label": v["label"], "rtx": rtx, "light": light, "mat": mat,
                         "exposure_match": v.get("exposure_match"), "scale": 1.0,
                         "bolt_visual": v.get("bolt_visual")})
    for k in {k for v in variants for k in v["rtx"]}:
        stock["rtx"][k] = st.get(k)
    threads = {p: stage.GetPrimAtPath(f"{p}/thread") for p in bolt_prims}
    has_threads = all(t.IsValid() for t in threads.values())
    if any(v["bolt_visual"] for v in variants) and not has_threads:
        raise SystemExit("ladder: a variant sets bolt_visual but the episode has no thread prims; "
                         "launch it with EVAL_BOLT_VISUAL=threaded")
    stock["bolt_visual"] = "threaded" if has_threads else "plain"
    stock_variant = {"label": "_stock", "rtx": {k: str(x) for k, x in stock["rtx"].items()},
                     "light": stock["light"], "mat": stock["mat"], "scale": 1.0,
                     "bolt_visual": stock["bolt_visual"]}

    # ---- applying a state -----------------------------------------------------------------------
    def set_carb(k, raw):
        old = st.get(k)
        if isinstance(old, bool):
            val = str(raw).lower() == "true"
        elif isinstance(old, int):
            val = int(float(raw))
        else:
            val = float(raw)
        st.set(k, val)
        if st.get(k) != val:
            raise RuntimeError(f"carb {k} did not take {val!r} (reads {st.get(k)!r})")

    def apply(v):
        if has_threads:
            threaded = (v.get("bolt_visual") or stock["bolt_visual"]) == "threaded"
            for p, t in threads.items():
                UsdGeom.Imageable(t).CreatePurposeAttr().Set(
                    UsdGeom.Tokens.default_ if threaded else UsdGeom.Tokens.guide)
                UsdGeom.Imageable(stage.GetPrimAtPath(f"{p}/shaft")).CreatePurposeAttr().Set(
                    UsdGeom.Tokens.guide if threaded else UsdGeom.Tokens.default_)
        for k in stock["rtx"]:
            set_carb(k, v["rtx"].get(k, stock["rtx"][k]))
        L, s = v["light"], v["scale"]
        key.GetIntensityAttr().Set(L["key"] * s)
        key.GetAngleAttr().Set(L["key_angle"])
        dome.GetIntensityAttr().Set(L["dome"] * s)
        sun_prim = stage.GetPrimAtPath("/World/sun")
        if L["sun"] > 0.0:
            if not sun_prim.IsValid():
                sun = UsdLux.DistantLight.Define(stage, "/World/sun")
                UsdGeom.Xformable(sun.GetPrim()).AddRotateXYZOp()
                sun_prim = sun.GetPrim()
            sun = UsdLux.DistantLight(sun_prim)
            sun.CreateIntensityAttr().Set(L["sun"] * s)
            sun.CreateAngleAttr().Set(L["sun_angle"])
            UsdGeom.Xformable(sun_prim).GetOrderedXformOps()[0].Set(
                Gf.Vec3f(float(L["sun_el"] - 90.0), 0.0, float(L["sun_az"])))
        elif sun_prim.IsValid():
            stage.RemovePrim("/World/sun")
        for n, f in v["mat"].items():
            shaders[n].GetInput("diffuseColor").Set(Gf.Vec3f(*f["rgb"]))
            shaders[n].GetInput("roughness").Set(float(f["rough"]))
            shaders[n].GetInput("metallic").Set(float(f["metallic"]))

    def render(settle):
        before = world.current_time_step_index
        frames, prev, resid = {}, None, None
        for i in range(settle):
            rep.orchestrator.step(rt_subframes=4, delta_time=0.0, pause_timeline=False)
            cur = {s: np.asarray(get_rgba[s]())[..., :3].astype(np.uint8) for s in cameras}
            if prev is not None:
                resid = max(float(np.abs(cur[s].astype(np.int16) - prev[s]).mean()) for s in cur)
            prev, frames = cur, cur
        if world.current_time_step_index != before:
            raise RuntimeError("ladder render advanced physics")
        return frames, resid

    def lum(frames):
        y = np.concatenate([(0.2126 * f[..., 0] + 0.7152 * f[..., 1] + 0.0722 * f[..., 2]).ravel()
                            for f in frames.values()])
        return float(np.median(y))

    # ---- phase bookkeeping, so the montage can pick moments by what the arms are doing -----------
    bolt_color = {}   # filled at the first snapshot from the bindings shared_stack verified and wrote

    def load_colors():
        meta = json.loads((scene.ROOT / "outputs/shared_stack" / args.tag / "scene_materials.json").read_text())
        for b in meta["visual_bindings"]:
            bolt_color[b["prim"].rsplit("/", 1)[0]] = b["color"]
        missing = [p for p in bolt_prims if p not in bolt_color]
        if missing:
            raise RuntimeError(f"ladder: no colour binding recorded for {missing[:3]}")

    def phase():
        if not bolt_color:
            load_colors()
        xyz = {p: np.asarray(bolt_views[p].get_world_poses()[0])[0] for p in bolt_prims}
        arms = {}
        for side, art in arts.items():
            names = list(art.dof_names)
            q = np.asarray(art.get_joint_positions())
            f = q[[names.index("finger_left_joint"), names.index("finger_right_joint")]]
            grip = float(np.clip(100 * (1 - (f[0] - f[1]) / (2 * scene.FINGER_TRAVEL_M)), 0, 100))
            tcp = np.asarray(tcp_views[side].get_world_poses()[0])[0]
            near = min(bolt_prims, key=lambda p: float(np.linalg.norm(xyz[p] - tcp)))
            arms[side] = {"tcp": tcp.round(4).tolist(), "gripper_pct": round(grip, 1),
                          "nearest_bolt": near.rsplit("/", 1)[-1], "nearest_color": bolt_color[near],
                          "nearest_dist_m": round(float(np.linalg.norm(xyz[near] - tcp)), 4),
                          "nearest_z_m": round(float(xyz[near][2]), 4)}
        return arms

    # ---- the hook ---------------------------------------------------------------------------------
    get_rgba = {s: cam.get_rgba for s, cam in cameras.items()}
    first = next(iter(cameras))
    state = {"next_t": 0.0, "n": 0}
    stream = (OUT / "ladder.jsonl").open("w")

    def snapshot():
        t = world.current_time_step_index * dt
        d = OUT / f"s{state['n']:03d}"
        d.mkdir()
        rec = {"snapshot": state["n"], "sim_step": int(world.current_time_step_index),
               "sim_time_s": round(t, 3), "arms": phase(), "variants": {}}
        rendered = {}
        order = sorted(variants, key=lambda v: v["exposure_match"] is not None)
        for v in order:
            if v["exposure_match"] and state["n"] == 0:
                target = rendered[v["exposure_match"]]["median"]
                for _ in range(5):
                    apply(v)
                    frames, _ = render(options.ladder_settle)
                    med = lum(frames)
                    if abs(med - target) < 1.5:
                        break
                    # op 0 writes display-encoded values, so light scales roughly as value^2.2
                    v["scale"] *= float(np.clip((max(target, 1.0) / max(med, 1.0)) ** 2.2, 0.05, 20.0))
            apply(v)
            frames, resid = render(options.ladder_settle)
            rendered[v["label"]] = {"median": lum(frames)}
            for s, f in frames.items():
                Image.fromarray(f).save(d / f"{v['label']}_{s}.png")
            rec["variants"][v["label"]] = {"median_lum": round(rendered[v["label"]]["median"], 1),
                                           "settle_residual": resid, "scale": v["scale"]}
        apply(stock_variant)
        render(options.ladder_settle)
        stream.write(json.dumps(rec) + "\n")
        stream.flush()
        print(f"[ladder] s{state['n']:03d} t={t:6.2f}s " + " ".join(
            f"{k}:{x['median_lum']:.0f}" for k, x in rec["variants"].items()) + " | " + " ".join(
            f"{s[0]} grip {a['gripper_pct']:.0f} {a['nearest_color']} {a['nearest_dist_m'] * 1000:.0f}mm"
            for s, a in rec["arms"].items()), flush=True)
        state["n"] += 1
        state["next_t"] = t + float(SPEC.get("every_sec", 1.0))

    def hooked():
        if state["n"] < options.ladder_max and world.current_time_step_index * dt >= state["next_t"]:
            snapshot()
        return get_rgba[first]()

    cameras[first].get_rgba = hooked
    try:
        return original_run(args, settings, scene, world, rep, arts, cameras, tcp_views,
                            bolt_views, bolt_prims, overview, materials)
    finally:
        stream.close()
        cameras[first].get_rgba = get_rgba[first]
        (OUT / "ladder_meta.json").write_text(json.dumps({
            "spec": SPEC, "spec_path": str(options.ladder_spec), "settle": options.ladder_settle,
            "stock": stock, "variants": variants, "episode_argv": forwarded,
            "episode_photometry": scene.photometry_metadata()}, indent=2, default=str))


shared_stack.run_episode = instrumented
import eval_closed_loop  # noqa: E402

raise SystemExit(eval_closed_loop.main())
