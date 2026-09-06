"""Render the wrist camera view from the CAD-derived D405 lens pose.

Lens pose recovered from the gripper mesh (scripts/fit_d405.py): the D405 stereo pair
sits at (x = -9.17, y = 46.01) and (x = +9.17, y = 46.01) mm in the tool frame, giving
an 18.34 mm baseline against the D405's 18 mm nominal. Module face at z = 119.3 mm, so
the optical axis runs along the tool +z (the approach direction, toward the grasp point).
NOTE: the round dome at y ~ 73 is the fisheye lens, which this rig does not use.

Camera frame convention: USD cameras look down their local -Z with +Y up, so with
  Z_cam = -z_tool,  Y_cam = +y_tool,  X_cam = -x_tool     (= Ry(180 deg))
the optical axis points along +z_tool and the fingers -- which sit near y_tool ~ 0,
i.e. below the lens -- land at the BOTTOM of frame, matching the real wrist images.

This is the CAD nominal hand-eye. It does not capture assembly error, but it is a
measured starting point rather than the eyeballed guess used for the montage.

Run:  OMNI_KIT_ACCEPT_EULA=YES .venv-isaac/bin/python scripts/render_wrist_cam.py
"""

import argparse
import math
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent

# CAD-derived lens pose in the tool / attachment_site frame (metres)
# Colour stream comes from the LEFT imager (operator-confirmed), where left/right are
# as seen from the camera. With optical axis = +z_tool and up = +y_tool, image-right is
# (forward x up) = z_hat x y_hat = -x_tool, so camera-left is +x_tool: the colour imager
# is the one at x = +9.17 mm.
LENS_XYZ = (+0.00917, 0.04601, 0.1193)
# D405 optics: MEASURED, not the datasheet. The 87 deg spec gives aperture 20.955 =
# fx 335.96 px, but the intrinsics the collected episodes actually carry
# (observations/<side>/camera_calib/color_intrinsics, both wrists, several sessions)
# are fx 393.32/393.78, fy 392.19/392.80 -- a 78.2 deg horizontal FOV. The policy's only
# spatial grounding is this image, so a 17.2% projection error is an aim error.
FOCAL_MM, H_APERTURE, V_APERTURE = 11.0, 17.8885, 13.4524
RES = (640, 480)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", choices=["aligned", "random"], default="aligned")
    ap.add_argument("--scene", default=None)
    ap.add_argument("--h-aperture", type=float, default=H_APERTURE)
    ap.add_argument("--v-aperture", type=float, default=V_APERTURE)
    ap.add_argument("--tag", default="")
    args = ap.parse_args()
    scene = pathlib.Path(args.scene) if args.scene else ROOT / f"assets/scene_{args.layout}.usd"

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})

    import numpy as np
    import omni.replicator.core as rep
    from isaacsim.core.api import World
    from isaacsim.core.prims import RigidPrim
    from isaacsim.core.utils.stage import open_stage, get_current_stage
    from PIL import Image
    from pxr import Gf, UsdGeom, UsdPhysics

    open_stage(str(scene))
    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 240.0)
    stage = get_current_stage()
    world.reset()
    for _ in range(5):
        world.step(render=True)

    out_paths = {}
    for side in ("left", "right"):
        root = f"/World/cell/{side}_arm/robot"
        tool_path = None
        for prim in stage.Traverse():
            p = str(prim.GetPath())
            if p.startswith(root) and prim.GetName() == "tool" and prim.HasAPI(
                UsdPhysics.RigidBodyAPI
            ):
                tool_path = p
                break
        if tool_path is None:
            print(f"{side}: tool body not found")
            continue

        view = RigidPrim(prim_paths_expr=tool_path, name=f"toolv_{side}")
        view.initialize()
        pos, quat = view.get_world_poses()
        t = np.asarray(pos)[0]
        w, x, y, z = np.asarray(quat)[0]
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ])

        cam_pos = t + R @ np.array(LENS_XYZ)
        # Ry(180): X_cam=-x_tool, Y_cam=+y_tool, Z_cam=-z_tool
        R_cam = R @ np.array([[-1.0, 0, 0], [0, 1.0, 0], [0, 0, -1.0]])

        cam_path = f"/World/wrist_cam_{side}"
        cam = UsdGeom.Camera.Define(stage, cam_path)
        cam.CreateFocalLengthAttr(FOCAL_MM)
        cam.CreateHorizontalApertureAttr(args.h_aperture)
        cam.CreateVerticalApertureAttr(args.v_aperture)
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.005, 100.0))
        M = Gf.Matrix4d()
        M.SetIdentity()
        for r in range(3):
            for c in range(3):
                M[r][c] = float(R_cam[c][r])
        M.SetTranslateOnly(Gf.Vec3d(*[float(v) for v in cam_pos]))
        UsdGeom.Xformable(cam).MakeMatrixXform().Set(M)

        print(f"{side:5s} lens world pos = ({cam_pos[0]:.4f}, {cam_pos[1]:.4f}, {cam_pos[2]:.4f})")
        print(f"      optical axis   = {np.round(-R_cam[:, 2], 3)}")

        rp = rep.create.render_product(cam_path, RES)
        ann = rep.AnnotatorRegistry.get_annotator("rgb")
        ann.attach([rp])
        arr = np.asarray([])
        for _ in range(20):
            rep.orchestrator.step(rt_subframes=4, pause_timeline=False)
            arr = np.asarray(ann.get_data())
            if arr.size:
                break
        if arr.size:
            out = ROOT / f"outputs/wristcam_{args.layout}_{side}{args.tag}.png"
            Image.fromarray(arr[..., :3].astype("uint8")).save(out)
            out_paths[side] = out
            print(f"      wrote {out.name}")
        ann.detach([rp])

    app.close()
    return 0 if len(out_paths) == 2 else 1


if __name__ == "__main__":
    raise SystemExit(main())
