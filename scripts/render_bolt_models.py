"""Close-up renders of the bolt models in a bolt geometry spec, for comparison with photos.

Builds each colour's threaded shaft and head from scripts/bolt_visual.py exactly as the rig does
(same meshes, the rig's stock bolt materials), lays one bolt on its side and stands one on its head
tip-up next to it, and renders them from a few views. No robot, no physics.

Run: OMNI_KIT_ACCEPT_EULA=YES python scripts/render_bolt_models.py \
        --geometry config/bolts/iso_heads_20260914.json --out outputs/bolt_models.png
"""
import argparse
import math
import pathlib

ROOT = pathlib.Path(__file__).resolve().parents[1]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--geometry", required=True)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True})
    import numpy as np
    import omni.replicator.core as rep
    from isaacsim.core.api import World
    from isaacsim.core.utils.stage import get_current_stage
    from PIL import Image, ImageDraw
    from pxr import Gf, Sdf, UsdGeom, UsdLux, UsdShade, Vt

    import sys
    sys.path.insert(0, str(ROOT / "scripts"))
    import bolt_visual

    spec = bolt_visual.load_geometry(ROOT / args.geometry if not pathlib.Path(args.geometry).is_absolute()
                                     else args.geometry)
    world = World(stage_units_in_meters=1.0)
    stage = get_current_stage()

    def mat(path, rgb, rough, metallic):
        m = UsdShade.Material.Define(stage, path)
        sh = UsdShade.Shader.Define(stage, path + "/S")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        m.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        return m

    mats = {"gray": mat("/World/m/bgray", (0.52, 0.53, 0.56), 0.32, 1.0),       # the rig's stock bolt materials
            "black": mat("/World/m/bblack", (0.07, 0.07, 0.08), 0.42, 0.9),
            "floor": mat("/World/m/floor", (0.22, 0.22, 0.23), 0.9, 0.0)}

    def mesh(path, m):
        pts, cnt, idx, nrm = m
        g = UsdGeom.Mesh.Define(stage, path)
        g.CreatePointsAttr(Vt.Vec3fArray.FromNumpy(pts.astype(np.float32)))
        g.CreateFaceVertexCountsAttr(Vt.IntArray.FromNumpy(cnt))
        g.CreateFaceVertexIndicesAttr(Vt.IntArray.FromNumpy(idx))
        g.CreateNormalsAttr(Vt.Vec3fArray.FromNumpy(nrm.astype(np.float32)))
        g.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
        g.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
        return g

    def bolt(path, color, translate, rotate_xyz):
        c = spec[color]
        xf = UsdGeom.Xform.Define(stage, path)
        xf.AddTranslateOp().Set(Gf.Vec3d(*translate))
        xf.AddRotateXYZOp().Set(Gf.Vec3f(*rotate_xyz))
        mesh(f"{path}/thread", bolt_visual.threaded_shaft_mesh(c["d_m"], c["length_m"], spec["pitch_m"]))
        mesh(f"{path}/head", bolt_visual.button_head_mesh(c, c["d_m"]) if c["head"] == "button"
             else bolt_visual.socket_cap_head_mesh(c, c["d_m"]))
        UsdShade.MaterialBindingAPI.Apply(xf.GetPrim()).Bind(mats[color])

    floor = UsdGeom.Cube.Define(stage, "/World/floor")
    floor.CreateSizeAttr(1.0)
    UsdGeom.Xformable(floor).AddTranslateOp().Set(Gf.Vec3d(0, 0, -0.5))
    UsdShade.MaterialBindingAPI.Apply(floor.GetPrim()).Bind(mats["floor"])
    for k, color in enumerate(("gray", "black")):
        y = -0.03 + 0.06 * k
        c = spec[color]
        R = c["head_d_m"] / 2
        bolt(f"/World/{color}_lying", color, (-0.02, y, R), (0, 0, 20))              # on its side
        # standing on its tip, head up, so the top views show the socket and the dome
        bolt(f"/World/{color}_standing", color, (0.025, y, c["length_m"]), (0, 90, 0))
    key = UsdLux.DistantLight.Define(stage, "/World/key")
    key.CreateIntensityAttr(900.0)
    key.CreateAngleAttr(8.0)
    UsdGeom.Xformable(key.GetPrim()).AddRotateXYZOp().Set(Gf.Vec3f(-40, 0, 30))
    UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(350.0)

    views = {"top": ((0.0, 0.0, 0.20), (0.0, 0.0, 0.0)),
             "oblique": ((0.14, -0.14, 0.12), (0.0, 0.0, 0.005)),
             "side": ((0.0, -0.20, 0.03), (0.0, 0.0, 0.008))}
    world.reset()
    tiles = []
    for name, (eye, target) in views.items():
        cam = rep.create.camera(position=eye, look_at=target, focal_length=35.0, clipping_range=(0.005, 10))
        rp = rep.create.render_product(cam, (800, 600))
        ann = rep.AnnotatorRegistry.get_annotator("rgb")
        ann.attach([rp])
        arr = np.asarray([])
        for _ in range(30):
            rep.orchestrator.step(rt_subframes=8, pause_timeline=False)
            arr = np.asarray(ann.get_data())
        img = Image.fromarray(arr[..., :3].astype("uint8"))
        ImageDraw.Draw(img).text((8, 8), f"{spec['name']} -- {name}", fill=(255, 255, 0))
        tiles.append(img)
        ann.detach([rp])
    sheet = Image.new("RGB", (800 * len(tiles), 600))
    for i, t in enumerate(tiles):
        sheet.paste(t, (800 * i, 0))
    out = pathlib.Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    sheet.save(out)
    print(f"wrote {out}")
    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
