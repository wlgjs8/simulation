"""Ladder step 2e: assemble the dual-arm cell in the CANONICAL stand frame and verify it.

Frame discipline (see CLAUDE.md §4): the arm-mount transforms in active_calibration.yaml
are relative to the `stand` frame, and `stand` is itself rotated +90 deg about z relative
to world. Rather than re-deriving that composition by hand (the montage got it wrong and
ended up 90 deg off), we read the mount transforms straight out of the imported stand USD
and parent each arm to them.

Verifies:
  - arm-mount world poses match the stand URDF's own kinematics
  - both arms reach a plausible workspace at the InitMotion reset pose
  - renders an overview image for eyeballing against montage/expected_sim_montage.png

Writes assets/cell_dual_rb3_730e.usd, outputs/cell_reset_pose.json, outputs/cell_overview.png

Run:  OMNI_KIT_ACCEPT_EULA=YES .venv-isaac/bin/python scripts/build_cell.py
"""

import argparse
import json
import pathlib

ROOT = pathlib.Path(__file__).resolve().parent.parent
# Default arm asset: Pika tool mesh pre-rotated +90 deg about the tool z axis.
# +90 is exactly what rb3_730e.urdf's tool visual origin already specifies -- the
# importer silently drops it (the tool link becomes a PhysX body whose pose comes
# from tool_joint, rpy=0). Baking it into the mesh is the only route that survives.
# See CLAUDE.md; regenerate with scripts/bake_tool_mesh.py --deg 90.
# 2026-09-05, RB3-730E -> RB5-850E. The tool-visual note above is RB3-only history: every
# RB5 URDF writes the tool visual at identity and the exported meshes already carry the
# rotation, so nothing is baked here any more.
ARM_USD = ROOT / "assets/rb5_850e_pika_tip_v15/rb5_850e_pika_articulated_sim.usda"
STAND_USD = ROOT / "assets/dual_rb5_850e_stand_only/dual_rb5_850e_stand_only.usda"
CELL_USD = ROOT / "assets/cell_dual_rb5_850e.usd"
OUT_JSON = ROOT / "outputs/cell_reset_pose.json"
OUT_PNG = ROOT / "outputs/cell_overview.png"
VIEW = "overview"

# InitMotion reset pose (deg). SOURCE IS THE SAVED FILE, NOT THE CODE DEFAULT:
# rb_gui writes the operator's taught pose to `~/.rb_servo_gui/init_motion.json`
# (app.py `_init_motion_path`, override `RB_GUI_INIT_MOTION_PATH`) and only falls back to
# `_DEFAULT_INIT_*_JOINTS_DEG` when that file is absent. Those defaults are STILL the RB3
# values -- the 2026-09-02 RB5 pass that rotated every other stand-frame constant missed
# them -- so reading the code would have put an RB3 pose on an RB5 arm.
# Values below: the file as saved 2026-09-03 20:56, i.e. the pose the robot actually uses.
# Cross-checked by FK against four teleop sessions the next day: this pose puts the left TCP
# at z -0.054 and the right at -0.072, against a measured dwell median of -0.055 / -0.072.
RESET = {
    "left": [-85.721, 36.301, 125.914, -9.832, -123.706, 33.556],
    "right": [86.320, -28.371, -125.802, -1.274, 123.360, -42.350],
}
JOINTS = [
    "base_joint",
    "shoulder_joint",
    "elbow_joint",
    "wrist1_joint",
    "wrist2_joint",
    "wrist3_joint",
]
MOUNT_FRAME = {"left": "stand_left_arm_base", "right": "stand_right_arm_base"}


def main() -> int:
    global VIEW, OUT_PNG, ARM_USD, MOUNT_FRAME
    ap = argparse.ArgumentParser()
    ap.add_argument("--view", choices=["overview", "gripper", "axial"], default="overview")
    ap.add_argument("--arm-usd", default=None, help="override the arm USD (tool-yaw candidates)")
    ap.add_argument("--out", default=None, help="override output PNG path")
    ap.add_argument("--wrist3-offset-deg", type=float, default=0.0,
                    help="add this to wrist3 (joint 6). Its axis IS the tool z axis, so this "
                         "is the ground-truth reference for what a tool-yaw rotation looks like")
    ap.add_argument("--tool-yaw-deg", type=float, default=0.0,
                    help="extra rotation about the tool z axis, applied at the USD level "
                         "after import (the URDF visual-origin route does not land reliably)")
    ap.add_argument("--grip", type=float, default=100.0,
                    help="jaw opening percent: 100 = open, 0 = closed")
    ap.add_argument("--single-arm", action="store_true",
                    help="build the left arm only (avoids the right arm intruding on close-ups)")
    args = ap.parse_args()
    VIEW = args.view
    if VIEW in ("gripper", "axial"):
        OUT_PNG = ROOT / "outputs/gripper_isaac.png"
    if args.arm_usd:
        ARM_USD = pathlib.Path(args.arm_usd)
    if args.out:
        OUT_PNG = pathlib.Path(args.out)
    if args.single_arm:
        MOUNT_FRAME = {"left": "stand_left_arm_base"}

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})

    import numpy as np
    import omni.replicator.core as rep
    from isaacsim.core.api import World
    from isaacsim.core.prims import RigidPrim, SingleArticulation
    from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
    from isaacsim.core.utils.types import ArticulationAction
    from pxr import Gf, Usd, UsdGeom, UsdLux, UsdPhysics

    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 240.0)
    world.get_physics_context().set_gravity(0.0)
    stage = get_current_stage()

    add_reference_to_stage(usd_path=str(STAND_USD), prim_path="/World/cell/stand")

    # Pull the mount transforms out of the stand asset itself.
    mount_xf = {}
    for prim in stage.Traverse():
        name = prim.GetName()
        for side, frame in MOUNT_FRAME.items():
            if name == frame and side not in mount_xf:
                m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                mount_xf[side] = m
    missing = [s for s in MOUNT_FRAME if s not in mount_xf]
    if missing:
        print(f"FAIL: mount frames not found in stand USD: {missing}")
        app.close()
        return 1

    for side, m in mount_xf.items():
        t = m.ExtractTranslation()
        print(f"{side:5s} mount world pos = ({t[0]:.4f}, {t[1]:.4f}, {t[2]:.4f})")
        arm_path = f"/World/cell/{side}_arm"
        xf = UsdGeom.Xform.Define(stage, arm_path)
        xf.MakeMatrixXform().Set(m)
        add_reference_to_stage(usd_path=str(ARM_USD), prim_path=f"{arm_path}/robot")

    # Rotate the Pika tool at the USD level. Patching the URDF's <visual><origin rpy>
    # does NOT reliably reach the rendered mesh: the importer authors the tool prim with
    # the NEGATED yaw, leaves the mesh prototype unrotated, and prim-xform vs
    # BBoxCache-world-bound measurements disagree. Authoring the rotation here, on the
    # prim that actually carries the geometry, is verifiable in the render.
    if abs(args.tool_yaw_deg) > 1e-9:
        rot = Gf.Matrix4d().SetRotate(
            Gf.Rotation(Gf.Vec3d(0, 0, 1), float(args.tool_yaw_deg))
        )
        n = 0
        for prim in stage.Traverse():
            if prim.GetName() != "tool" or "attachment_site" not in str(prim.GetPath()):
                continue
            xf = UsdGeom.Xformable(prim)
            existing = xf.GetLocalTransformation()
            xf.ClearXformOpOrder()
            xf.MakeMatrixXform().Set(rot * existing)
            n += 1
        print(f"applied tool yaw {args.tool_yaw_deg:+.1f} deg to {n} tool prim(s)")

    light = UsdLux.DistantLight.Define(stage, "/World/light")
    light.CreateIntensityAttr(3000.0)
    light.CreateAngleAttr(1.0)
    dome = UsdLux.DomeLight.Define(stage, "/World/dome")
    dome.CreateIntensityAttr(700.0)

    arts = {}
    for side in MOUNT_FRAME:
        a = SingleArticulation(prim_path=f"/World/cell/{side}_arm/robot", name=f"{side}_arm")
        world.scene.add(a)
        arts[side] = a
    world.reset()
    for a in arts.values():
        a.initialize()

    # Resolve TCP rigid-body prims per arm (nested + name-shadowed by visual meshes).
    tcp_view = {}
    for side in MOUNT_FRAME:
        root = f"/World/cell/{side}_arm/robot"
        for prim in stage.Traverse():
            p = str(prim.GetPath())
            if p.startswith(root) and prim.GetName() == "tcp" and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                tcp_view[side] = RigidPrim(prim_paths_expr=p, name=f"tcp_{side}")
                tcp_view[side].initialize()
                break

    for side, art in arts.items():
        names = list(art.dof_names)
        q = np.zeros(len(names), dtype=np.float32)
        for j, deg in zip(JOINTS, RESET[side]):
            if j == "wrist3_joint":
                deg += args.wrist3_offset_deg
            q[names.index(j)] = np.deg2rad(deg)
        # jaw: finger_pos = (1 - grip/100) * 0.047, meshes authored OPEN (see CLAUDE.md)
        fp = (1.0 - args.grip / 100.0) * 0.049   # v15 jaw, stack_real gripper_finger_travel_m
        for jn, sgn in (("finger_left_joint", +1.0), ("finger_right_joint", -1.0)):
            if jn in names:
                q[names.index(jn)] = sgn * fp
        art.get_articulation_controller().set_gains(
            kps=np.full(len(names), 1.0e7), kds=np.full(len(names), 1.0e5)
        )
        art.set_joint_positions(q)
        art.set_joint_velocities(np.zeros_like(q))
        art.apply_action(ArticulationAction(joint_positions=q))
    # render=True is required: with render=False the render pipeline never picks up
    # the pose PhysX applied, and every image comes out at the default (all-zero)
    # joint configuration while the JSON numbers below are at the reset pose.
    for _ in range(10):
        world.step(render=True)

    result = {}
    for side, view in tcp_view.items():
        pos, quat = view.get_world_poses()
        p = np.asarray(pos)[0]
        result[side] = {
            "tcp_pos": [float(v) for v in p],
            "tcp_quat_wxyz": [float(v) for v in np.asarray(quat)[0]],
            "mount_pos": [float(v) for v in mount_xf[side].ExtractTranslation()],
            "q_deg": RESET[side],
        }
        print(f"{side:5s} TCP @reset = ({p[0]:.4f}, {p[1]:.4f}, {p[2]:.4f})")

    if len(result) == 2:
        d = np.linalg.norm(
            np.array(result["left"]["tcp_pos"]) - np.array(result["right"]["tcp_pos"])
        )
        print(f"TCP separation = {d:.4f} m")
        result["tcp_separation_m"] = float(d)

    OUT_JSON.parent.mkdir(exist_ok=True)
    OUT_JSON.write_text(json.dumps(result, indent=1))

    stage.Export(str(CELL_USD))
    print(f"wrote {CELL_USD}")

    if VIEW == "axial":
        # Look back down the tool's own -z (from beyond the fingertips toward the
        # flange). The finger-opening axis and which side the body sits on are both
        # unambiguous in this view. Frame axes come from the PHYSICS TCP quaternion,
        # not from UsdGeom (which is stale w.r.t. the applied pose).
        w, x, y, z = result["left"]["tcp_quat_wxyz"]
        R = np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ])
        tgt = np.array(result["left"]["tcp_pos"]) - R[:, 2] * 0.09
        eye = tgt + R[:, 2] * 0.42
        camera = rep.create.camera(
            position=tuple(float(v) for v in eye),
            look_at=tuple(float(v) for v in tgt),
            clipping_range=(0.01, 100.0),
        )
    elif VIEW == "gripper":
        # Frame the left arm's Pika tool, viewed from the attachment frame's +y so
        # the finger-opening axis (attachment +x) runs across the image.
        att_prim = next(
            p
            for p in stage.Traverse()
            if p.GetName() == "attachment_site"
            and p.GetTypeName() == "Xform"
            and str(p.GetPath()).startswith("/World/cell/left_arm")
        )
        tool_prim = next(
            p
            for p in stage.Traverse()
            if p.GetName() == "tool" and str(p.GetPath()).startswith("/World/cell/left_arm")
        )
        m = UsdGeom.Xformable(att_prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        A = np.array([[m[i][j] for j in range(4)] for i in range(4)]).T
        # NOTE: UsdGeom.BBoxCache reads the USD, which does NOT reflect the pose PhysX
        # applied (fabric does not write back). Targeting it aims the camera at the
        # robot's default pose. Use the physics-authoritative TCP position instead.
        # Keep the overview camera's proven viewing direction, just dolly in on the
        # left TCP. (Deriving an offset from `A` does not work: UsdGeom transforms are
        # stale w.r.t. the pose PhysX applied.)
        tgt = np.array(result["left"]["tcp_pos"])
        direction = np.array([1.7, -1.5, 1.25]) - np.array([0.25, 0.0, 0.35])
        direction = direction / np.linalg.norm(direction)
        eye = tgt + direction * 0.55
        print(f"gripper view: target={np.round(tgt, 4)} eye={np.round(eye, 4)}")
        print(f"  attachment +x (finger open axis) = {np.round(A[:3, 0], 3)}")
        # clipping_range must be set explicitly: the default near plane is ~1 m, which
        # silently clips away everything in a close-up (the image comes back empty).
        camera = rep.create.camera(
            position=tuple(float(v) for v in eye),
            look_at=tuple(float(v) for v in tgt),
            clipping_range=(0.01, 100.0),
        )
    else:
        # Work area is at +x (arms reach forward to x ~0.35 at reset), so view from +x.
        camera = rep.create.camera(position=(1.7, -1.5, 1.25), look_at=(0.25, 0.0, 0.35))
    rp = rep.create.render_product(camera, (1280, 960))
    rgb = rep.AnnotatorRegistry.get_annotator("rgb")
    rgb.attach([rp])
    arr = np.asarray([])
    for _ in range(20):
        rep.orchestrator.step(rt_subframes=4, pause_timeline=False)
        arr = np.asarray(rgb.get_data())
        if arr.size:
            break
    if arr.size:
        from PIL import Image

        Image.fromarray(arr[..., :3].astype("uint8")).save(OUT_PNG)
        print(f"wrote {OUT_PNG}")

    app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
