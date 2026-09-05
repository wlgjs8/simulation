"""Ladder step 3: table + place boxes + M12x25 bolts on top of the verified cell.

Coordinates are the montage layout mapped into the CANONICAL frame. The montage was
authored with the work area at -y; canonical puts it at +x, i.e. Rz(+90): (x, y) -> (-y, x).

  montage                        canonical
  gray pile   (+0.16, -0.47)  -> (0.47, +0.16)   left arm side  (+y)
  black pile  (-0.16, -0.47)  -> (0.47, -0.16)   right arm side (-y)
  gray box    (+0.17, -0.80)  -> (0.80, +0.17)
  green box   (-0.17, -0.80)  -> (0.80, -0.17)

Top view convention: camera looks down -z with world +x pointing DOWN in frame, so the
robot sits at the top and the boxes at the bottom -- matching the montage. World +y is
then image-RIGHT, so the gray box/gray bolts (+y, left arm) appear right and the green
box/black bolts (-y, right arm) appear left, exactly as in expected_sim_montage.png.

Gates:
  - every bolt lies within the recommended reach shell of its assigned arm
  - bolts rest on the table after settling (no fall-through, no explosion)

Run:  OMNI_KIT_ACCEPT_EULA=YES .venv-isaac/bin/python scripts/build_scene.py --layout aligned
"""

import argparse
import json
import math
import os
import pathlib
import work_surface

ROOT = pathlib.Path(__file__).resolve().parent.parent
# Same PIKA_TIP knob as eval_closed_loop.py -- see the note there. Kept in sync so a scene
# preview or a wrist-cam render never shows a different gripper than the one being scored.
PIKA_TIP = os.environ.get("PIKA_TIP", "v15").lower()
# Track eval_closed_loop.py's jaw model. This file used to carry its own local 0.047, so a
# preview rendered with the v15 asset would have closed 2 mm/side short of the jaw being
# scored -- the scene builder and the scorer must not disagree about what "closed" is.
FINGER_TRAVEL_M = float(os.environ.get(
    "FINGER_TRAVEL_M", "0.049" if PIKA_TIP == "v15" else "0.047"))
# Track eval_closed_loop.py: SIM_ARM picks the arm (RB5-850E since 2026-09-05) and the
# preview must never draw a different robot or stand than the scorer runs.
SIM_ARM = os.environ.get("SIM_ARM", "rb5_850e").lower()
ARM_USD = ROOT / (f"assets/{SIM_ARM}_pika_tip_v15/{SIM_ARM}_pika_articulated_sim.usda"
                  if PIKA_TIP == "v15" else
                  f"assets/{SIM_ARM}_pika_articulated_sim/{SIM_ARM}_pika_articulated_sim.usda")
STAND_USD = ROOT / "assets/dual_rb5_850e_stand_only/dual_rb5_850e_stand_only.usda"
# RB5-850E envelope: r_min 0.1706 / r_max 1.2526 recommended (RB3 was 0.1331 / 1.0583).
# NOTE the ENFORCED shell on the robot is tighter -- stack_real.yaml safety.reach_constraint
# r_max_m 1.150 / r_min_m 0.175 -- after a documented 1.250 -> 0.980 -> 1.050 -> 1.150
# sequence. Use the envelope for "can the arm physically get there", the config for "will the
# robot let it".
REACH = (
    pathlib.Path.home()
    / "workspace/robotics_lab/rb_servo_server/descriptions/reach_envelope_rb5_850e.json"
)

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

# M12 x 25 socket-head cap screw
SHAFT_R, SHAFT_L = 0.006, 0.025
HEAD_R, HEAD_L = 0.0092, 0.012

# TABLE HEIGHT. z = 0 is the STAND frame origin, not the table. On the RB3 cell the two
# were assumed coincident -- an assumption inherited from the montage and never measured.
# The RB5 build makes it false and by a lot: the stand is raised on a 280 mm rectangular
# riser the operator assembled between the table and the stand base (operator, 2026-09-05).
# The stand's own base plate reaches 15 mm below the stand origin, so the table top sits at
#     -0.015 - 0.280 = -0.295
# CORROBORATION, from four teleop sessions on 2026-09-04 (594k ticks of
# {left,right}_tcp_actual_stand_* in robotics_lab/logs): over the pile region (x < 0.60) the
# TCP bottoms out at p1 = -0.283 (left) / -0.291 (right). Fingertips stopping a few mm above
# the table with a 12 mm bolt between them is exactly right. The old z = 0 put this rig's
# whole scene 295 mm too high.
TABLE_Z = -0.295
WORK_SURFACE = work_surface.load('foam')
PICK_SURFACE_Z = work_surface.top_z(WORK_SURFACE, TABLE_Z)
RISER_H = 0.280           # table top -> stand base plate
TABLE = dict(cx=0.55, cy=0.0, hx=0.50, hy=0.55, thick=0.012)
# RISER. Same xy as the stand's FLOOR-BOLTED BASE PLATE, by operator instruction -- not the
# whole stand silhouette. The first cut used the mesh bbox (401.5 x 521.4 mm) and was far too
# big: that bbox is dominated by the upper arm-mounting plate, which overhangs the base.
# Measured by slicing dual_rb5_850e_stand_ver2.stl at its bottom: from z = -0.015 the section
# is CONSTANT at x [-0.120, +0.120], y [-0.150, +0.150] -- exactly 240 x 300 mm, centred on
# the stand axis. It is a real collidable object: the arms reach 280 mm below the stand now,
# so leaving it out would let them swing through the thing that is actually in the way.
RISER = dict(cx=0.0, cy=0.0, hx=0.120, hy=0.150)
# PILE / BOX, from the same 594k-tick teleop extract. Deep points (z < -0.24, i.e. actual
# picks) cluster at x p50 0.458 / 0.452 and y p50 +0.051 (left, gray) / -0.041 (right, black).
# THE PILES ARE NEARLY TOUCHING ON THE REAL CELL -- 92 mm apart, against the 320 mm this rig
# had. That is a genuine difficulty change, not a coordinate fix: it raises the nearest-bolt
# ambiguity the T1 metrics measure, so compare margin buckets, not raw grasp counts, across
# the port.
PILE_X = 0.455
PILE_DY = 0.046
# Box centres are UNCHANGED. The release points measured at x 0.762/0.767 and y +0.253/-0.310
# both fall inside a 380 x 240 box centred here, so the data corroborates the existing
# placement rather than contradicting it; releases simply are not at the box centre.
# See eval_closed_loop.py: moved out 2026-09-05 because the old placement put the box lip
# over the pick region and the arms fouled it. Values are the measured release clusters.
BOX_X = 0.765
BOX_DY = 0.260
# NPC NTC-321, measured: 380 x 240 x 105 mm outer, 20 mm side wall, 6.5 mm floor,
# plus a 30 mm sponge insert. Source: robotics_lab commit 83c5458^,
# camera_server/stereo_worker/box_detect.py -> BOX_DIMS / _open_tray_model().
# The sponge top (36.5 mm above the table) is the actual place surface -- that file
# notes the head camera sees the sponge top, not the floor (local-z ~ -0.019).
# Long side (380 mm) runs along world y, i.e. HORIZONTAL in the top view, matching the
# real cell. hw is the x half-extent, hd the y half-extent.
BOX = dict(
    hw=0.120,          # 240 mm along x (short side, vertical in top view)
    hd=0.190,          # 380 mm along y (long side, horizontal in top view)
    wall_h=0.0525,     # 105 mm outer height / 2
    t=0.020,           # 20 mm side wall
    floor_t=0.0065,    # 6.5 mm floor
    sponge_h=0.030,    # 30 mm sponge insert
)

# left arm handles gray bolts, right arm handles black
ARM_OF_COLOR = {"gray": "left", "black": "right"}


def bolt_poses(layout: str, n_per: int, seed: int):
    if 'size_m' in WORK_SURFACE:
        return work_surface.placed_bolts(WORK_SURFACE, layout, n_per, seed, PILE_X, PILE_DY)
    import numpy as np

    rng = np.random.default_rng(seed)
    out = []
    if layout == "aligned":
        for color, sy in (("gray", +1.0), ("black", -1.0)):
            for _ in range(n_per):
                out.append(
                    (
                        color,
                        PILE_X + rng.normal(0, 0.045),
                        sy * PILE_DY + rng.normal(0, 0.05),
                        rng.uniform(0, math.pi),
                    )
                )
    else:
        colors = ["gray"] * n_per + ["black"] * n_per
        rng.shuffle(colors)
        for color in colors:
            out.append(
                (
                    color,
                    PILE_X + rng.normal(0, 0.07),
                    rng.uniform(-0.30, 0.30),
                    rng.uniform(0, math.pi),
                )
            )
    return out


def main() -> int:
    global MOUNT_FRAME
    global WORK_SURFACE, PICK_SURFACE_Z
    ap = argparse.ArgumentParser()
    ap.add_argument("--layout", choices=["aligned", "random"], default="aligned")
    ap.add_argument("--n-per-color", type=int, default=11)
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument('--work-surface', choices=['foam','bare'], default='foam')
    ap.add_argument("--settle-steps", type=int, default=240)
    ap.add_argument("--only", choices=["left", "right"], default=None,
                    help="build a single arm, so a wrist view contains only that arm")
    ap.add_argument("--hide-stand", action="store_true",
                    help="keep the stand for its mount frames but hide it, so a wrist "
                         "view contains nothing but that arm's own gripper")
    ap.add_argument("--bare", action="store_true",
                    help="omit table/boxes/bolts: anything left in a wrist view is the "
                         "gripper itself, so left and right must look identical")
    ap.add_argument("--grip", type=float, default=100.0,
                    help="jaw opening percent: 100 = open, 0 = closed (real convention)")
    ap.add_argument("--wrist-cam", action="store_true",
                    help="also render the D405 wrist views (CAD-derived pose)")
    args = ap.parse_args()
    WORK_SURFACE = work_surface.load(args.work_surface)
    PICK_SURFACE_Z = work_surface.top_z(WORK_SURFACE, TABLE_Z)
    global MOUNT_FRAME
    if args.only:
        MOUNT_FRAME = {args.only: MOUNT_FRAME[args.only]}

    from isaacsim import SimulationApp

    app = SimulationApp({"headless": True})

    import numpy as np
    import omni.replicator.core as rep
    from isaacsim.core.api import World
    from isaacsim.core.prims import RigidPrim, SingleArticulation
    from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
    from isaacsim.core.utils.types import ArticulationAction
    from isaacsim.sensors.camera import Camera
    from PIL import Image
    from pxr import Gf, Sdf, Usd, UsdGeom, UsdLux, UsdPhysics

    world = World(stage_units_in_meters=1.0, physics_dt=1.0 / 240.0)
    stage = get_current_stage()

    def rgb_material(path, rgb, rough=0.5, metallic=0.0):
        from pxr import UsdShade

        mat = UsdShade.Material.Define(stage, path)
        sh = UsdShade.Shader.Define(stage, path + "/Shader")
        sh.CreateIdAttr("UsdPreviewSurface")
        sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgb))
        sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(rough)
        sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(metallic)
        mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
        return mat

    # Metallic where the real parts are metal. The bolts previously read as white
    # because a bright non-metallic diffuse saturates under the key light; a metal
    # workflow needs a much darker base colour, since metals get their brightness from
    # specular reflection rather than diffuse albedo.
    MAT = {
        "table": rgb_material("/World/mat/table", (0.42, 0.44, 0.45), 0.42, metallic=0.65),
        "riser": rgb_material("/World/mat/riser", (0.05, 0.05, 0.055), 0.55, metallic=0.35),
        "green": rgb_material("/World/mat/green", (0.10, 0.38, 0.27), 0.45),
        "boxgray": rgb_material("/World/mat/boxgray", (0.46, 0.47, 0.49), 0.50, metallic=0.35),
        "insert": rgb_material("/World/mat/insert", (0.26, 0.26, 0.28), 0.85),
        # zinc/stainless fastener: dark base + full metallic reads as grey, not white
        "bolt_gray": rgb_material("/World/mat/bolt_gray", (0.52, 0.53, 0.56), 0.32, metallic=1.0),
        # black-oxide fastener
        "bolt_black": rgb_material("/World/mat/bolt_black", (0.07, 0.07, 0.08), 0.42, metallic=0.9),
    }

    def bind(prim, key):
        from pxr import UsdShade

        UsdShade.MaterialBindingAPI(prim).Bind(MAT[key])

    def static_box(path, center, half, key):
        cube = UsdGeom.Cube.Define(stage, path)
        cube.CreateSizeAttr(2.0)
        UsdGeom.Xformable(cube).AddTransformOp().Set(
            Gf.Matrix4d().SetScale(Gf.Vec3d(*half)) * Gf.Matrix4d().SetTranslate(Gf.Vec3d(*center))
        )
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        bind(cube.GetPrim(), key)
        return cube

    # ---- stand + arms -----------------------------------------------------
    add_reference_to_stage(usd_path=str(STAND_USD), prim_path="/World/cell/stand")
    mount_xf = {}
    for prim in stage.Traverse():
        for side, frame in MOUNT_FRAME.items():
            if prim.GetName() == frame and side not in mount_xf:
                mount_xf[side] = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(
                    Usd.TimeCode.Default()
                )
    for side, m in mount_xf.items():
        p = f"/World/cell/{side}_arm"
        UsdGeom.Xform.Define(stage, p).MakeMatrixXform().Set(m)
        add_reference_to_stage(usd_path=str(ARM_USD), prim_path=f"{p}/robot")

    # D405 wrist cameras. Pose is set through the Camera sensor API with
    # camera_axes="ros" (+Y up, +Z forward), so an identity orientation in the tool frame
    # gives optical axis = +z_tool and up = +y_tool. Hand-authoring the USD matrix instead
    # produced frames that were aimed backwards -- do not go back to that.
    LENS = (0.00917, 0.04601, 0.11930)   # colour (left) imager, from scripts/fit_d405.py
    wrist_cams = {}
    if args.wrist_cam:
        for side in MOUNT_FRAME:
            wrist_cams[side] = None  # filled in after the tool prims resolve

    # ---- table ------------------------------------------------------------
    if not args.bare:
        work_surface.build(stage, WORK_SURFACE, TABLE_Z)
        static_box(
        "/World/scene/table",
            (TABLE["cx"], TABLE["cy"], TABLE_Z - TABLE["thick"]),
            (TABLE["hx"], TABLE["hy"], TABLE["thick"]),
            "table",
        )
        # The 280 mm riser the stand is bolted to (operator, 2026-09-05). Same xy as the
        # stand footprint, measured off dual_rb5_850e_stand_ver2.stl. Drawn here for the same
        # reason the scorer collides with it: the arms reach below the stand now.
        static_box(
            "/World/scene/riser",
            (RISER["cx"], RISER["cy"], TABLE_Z + RISER_H / 2),
            (RISER["hx"], RISER["hy"], RISER_H / 2),
            "riser",
        )

    # ---- place boxes ------------------------------------------------------
    b = BOX
    for name, sy, wall_key in ([] if args.bare else [("box_gray", +1.0, "boxgray"), ("box_green", -1.0, "green")]):
        cy = sy * BOX_DY
        root = f"/World/scene/{name}"
        UsdGeom.Xform.Define(stage, root)
        static_box(
            f"{root}/floor",
            (BOX_X, cy, TABLE_Z + b["floor_t"] / 2),
            (b["hw"], b["hd"], b["floor_t"] / 2),
            "boxgray" if wall_key == "boxgray" else "green",
        )
        static_box(
            f"{root}/sponge",
            (BOX_X, cy, TABLE_Z + b["floor_t"] + b["sponge_h"] / 2),
            (b["hw"] - b["t"], b["hd"] - b["t"], b["sponge_h"] / 2),
            "insert",
        )
        h = b["wall_h"]
        for tag, off, half in (
            # Outer dimensions are 240 x 380 mm; wall thickness is 20 mm.
            ("w_xm", (-b["hw"]+b["t"]/2, 0.0), (b["t"]/2, b["hd"], h)),
            ("w_xp", (+b["hw"]-b["t"]/2, 0.0), (b["t"]/2, b["hd"], h)),
            ("w_ym", (0.0, -b["hd"]+b["t"]/2), (b["hw"]-b["t"], b["t"]/2, h)),
            ("w_yp", (0.0, +b["hd"]-b["t"]/2), (b["hw"]-b["t"], b["t"]/2, h)),
        ):
            static_box(
                f"{root}/{tag}", (BOX_X + off[0], cy + off[1], TABLE_Z + h), half, wall_key
            )

    # ---- bolts ------------------------------------------------------------
    poses = [] if args.bare else bolt_poses(args.layout, args.n_per_color, args.seed)
    bolt_paths = []
    for i, (color, x, y, yaw) in enumerate(poses):
        path = f"/World/scene/bolt_{i:02d}"
        xf = UsdGeom.Xform.Define(stage, path)
        xf.MakeMatrixXform().Set(
            Gf.Matrix4d().SetRotate(Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(yaw)))
            * Gf.Matrix4d().SetTranslate(Gf.Vec3d(x, y, PICK_SURFACE_Z +
                (HEAD_R + 0.001 if 'size_m' in WORK_SURFACE else SHAFT_R + 0.002)))
        )
        prim = xf.GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(prim)
        UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(0.022)  # ~22 g for M12x25
        key = "bolt_gray" if color == "gray" else "bolt_black"
        for tag, r, half_l, cx in (
            ("shaft", SHAFT_R, SHAFT_L / 2, SHAFT_L / 2),
            ("head", HEAD_R, HEAD_L / 2, -HEAD_L / 2),
        ):
            cyl = UsdGeom.Cylinder.Define(stage, f"{path}/{tag}")
            cyl.CreateRadiusAttr(r)
            cyl.CreateHeightAttr(half_l * 2)
            cyl.CreateAxisAttr("X")
            UsdGeom.Xformable(cyl).AddTranslateOp().Set(Gf.Vec3d(cx, 0, 0))
            UsdPhysics.CollisionAPI.Apply(cyl.GetPrim())
            bind(cyl.GetPrim(), key)
        bolt_paths.append((path, color))

    if args.hide_stand:
        # The stand is not part of any articulation, so hiding it is safe here (unlike
        # hiding arm links, which also hides the tool and breaks physics queries).
        sp = stage.GetPrimAtPath("/World/cell/stand")
        if sp.IsValid():
            UsdGeom.Imageable(sp).MakeInvisible()
            print("  stand hidden")

    # Shadows were too hard: a distant light defaults to a ~0.5 deg angular size, which
    # gives razor-sharp edges. Widening it softens the penumbra, and shifting energy into
    # the dome fills the shadow interiors the way the real cell's diffuse lighting does.
    key = UsdLux.DistantLight.Define(stage, "/World/key")
    key.CreateIntensityAttr(600.0)
    key.CreateAngleAttr(12.0)
    UsdLux.DomeLight.Define(stage, "/World/dome").CreateIntensityAttr(520.0)

    arts = {}
    for side in MOUNT_FRAME:
        a = SingleArticulation(prim_path=f"/World/cell/{side}_arm/robot", name=f"{side}_arm")
        world.scene.add(a)
        arts[side] = a
    world.reset()
    for a in arts.values():
        a.initialize()

    if args.wrist_cam:
        for side in list(wrist_cams):
            root = f"/World/cell/{side}_arm/robot"
            tool_path = next(
                (
                    str(pr.GetPath())
                    for pr in stage.Traverse()
                    if str(pr.GetPath()).startswith(root)
                    and pr.GetName() == "tool"
                    and pr.HasAPI(UsdPhysics.RigidBodyAPI)
                ),
                None,
            )
            if tool_path is None:
                print(f"  {side}: tool body not found, no wrist cam")
                wrist_cams.pop(side)
                continue
            cam = Camera(prim_path=f"{tool_path}/wrist_cam", name=f"d405_{side}",
                         resolution=(640, 480))
            cam.initialize()
            # 180 deg roll about the optical axis (quat is scalar-first w,x,y,z).
            # Operator: this orientation matches the real wrist frames; removing the roll
            # made it worse.
            cam.set_local_pose(translation=np.array(LENS),
                               orientation=np.array([0.0, 0.0, 0.0, 1.0]),
                               camera_axes="ros")
            # optics set on the USD prim directly: the helper setters apply a stage-unit
            # conversion that muddies the 87 deg D405 FOV we want
            cam.prim.GetAttribute("focalLength").Set(11.0)
            cam.prim.GetAttribute("horizontalAperture").Set(20.955)
            cam.prim.GetAttribute("verticalAperture").Set(20.955 * 480 / 640)
            cam.prim.GetAttribute("clippingRange").Set(Gf.Vec2f(0.004, 100.0))
            wrist_cams[side] = cam
            print(f"  {side:5s} D405 camera at {cam.prim_path}")

    bolt_views = {p: RigidPrim(prim_paths_expr=p, name=f"bv_{i}") for i, (p, _) in enumerate(bolt_paths)}
    for v in bolt_views.values():
        v.initialize()

    for side, art in arts.items():
        names = list(art.dof_names)
        q = np.zeros(len(names), dtype=np.float32)
        for j, deg in zip(JOINTS, RESET[side]):
            q[names.index(j)] = np.deg2rad(deg)
        # Jaw BEFORE set_joint_positions: writing the finger targets afterwards left the
        # fingers teleported to 0 (open) with only the drive chasing the closed target,
        # and they never actually closed in the render. build_cell.py had the right order.
        finger_pos = (1.0 - args.grip / 100.0) * FINGER_TRAVEL_M
        for jn, sign in (("finger_left_joint", +1.0), ("finger_right_joint", -1.0)):
            if jn in names:
                q[names.index(jn)] = sign * finger_pos

        art.set_joint_positions(q)
        art.set_joint_velocities(np.zeros_like(q))
        # Gravity is ON here so the bolts settle, so the arms must be actively HELD at the
        # reset pose. Two things are needed and both were missing at first:
        #   1) a drive target (defaults to 0, so the arm is driven away from the pose), and
        #   2) nonzero drive gains -- the imported drives have ~zero stiffness, so a target
        #      alone produces no force and the arm simply falls onto the table.
        n = len(names)
        art.get_articulation_controller().set_gains(
            kps=np.full(n, 1.0e7), kds=np.full(n, 1.0e5)
        )
        art.apply_action(ArticulationAction(joint_positions=q))

    tcp_views = {}
    for side in MOUNT_FRAME:
        path = next(str(p.GetPath()) for p in stage.Traverse() if p.GetName() == 'tcp'
                    and f'/{side}_arm/' in str(p.GetPath()) and p.HasAPI(UsdPhysics.RigidBodyAPI))
        tcp_views[side] = RigidPrim(path, name='check_tcp_'+side, reset_xform_properties=False)
        tcp_views[side].initialize()
    world.step(render=False)
    reset_tcp_z = {s:float(np.asarray(v.get_world_poses()[0])[0,2]) for s,v in tcp_views.items()}
    for _ in range(args.settle_steps):
        world.step(render=True)

    # ---- gates ------------------------------------------------------------
    reach = json.loads(REACH.read_text())
    r_min, r_max = reach["r_min_recommended_m"], reach["r_max_recommended_m"]
    mounts = {s: np.array([m.ExtractTranslation()[i] for i in range(3)]) for s, m in mount_xf.items()}

    settled, unreachable = [], []
    for (path, color), _ in zip(bolt_paths, range(len(bolt_paths))):
        pos, _ = bolt_views[path].get_world_poses()
        p = np.asarray(pos)[0]
        settled.append(float(p[2]))
        arm = ARM_OF_COLOR[color]
        d = float(np.linalg.norm(p - mounts[arm]))
        if not (r_min <= d <= r_max):
            unreachable.append((path, color, arm, round(d, 3)))

    for side, art in arts.items():
        nm = list(art.dof_names)
        got = np.asarray(art.get_joint_positions())
        if "finger_left_joint" in nm:
            print(f"  {side:5s} jaw grip={args.grip:5.1f} -> finger L={got[nm.index('finger_left_joint')]:+.4f} "
                  f"R={got[nm.index('finger_right_joint')]:+.4f} m "
                  f"(target +-{(1 - args.grip / 100) * FINGER_TRAVEL_M:.4f})")

    tcp_z = {}
    for side in MOUNT_FRAME:
        root = f"/World/cell/{side}_arm/robot"
        for prim in stage.Traverse():
            pp = str(prim.GetPath())
            if pp.startswith(root) and prim.GetName() == "tcp" and prim.HasAPI(UsdPhysics.RigidBodyAPI):
                v = RigidPrim(prim_paths_expr=pp, name=f"tcpz_{side}")
                v.initialize()
                tcp_z[side] = float(np.asarray(v.get_world_poses()[0])[0][2])
                break

    z = np.array(settled) if settled else np.array([0.006])
    print(f"layout={args.layout}  bolts={len(bolt_paths)}")
    # reference: reset-pose TCP heights measured with gravity off (step 2)
    for side, expect in reset_tcp_z.items():
        got = tcp_z.get(side, float("nan"))
        print(f"  {side:5s} TCP z after settling = {got:.4f} (reset ref {expect:.4f}, "
              f"drop {expect - got:+.4f} m)")
    print(f"  settled z: min={z.min():.4f} max={z.max():.4f} mean={z.mean():.4f}; pick surface={PICK_SURFACE_Z:.4f}")
    print(f"  reach gate [{r_min:.3f}, {r_max:.3f}] m -> {len(unreachable)} outside")
    for u in unreachable[:5]:
        print(f"    OUT {u}")
    for side, mp in mounts.items():
        for tag, pt in (("pile", np.array([PILE_X, (1 if side == 'left' else -1) * PILE_DY, PICK_SURFACE_Z + HEAD_R])),
                        ("box", np.array([BOX_X, (1 if side == 'left' else -1) * BOX_DY, TABLE_Z + BOX['floor_t'] + BOX['sponge_h']]))):
            print(f"  {side:5s} -> {tag:4s} dist = {np.linalg.norm(pt - mp):.3f} m")

    held = all(abs(tcp_z.get(s_, -9) - e_) < 0.02 for s_, e_ in reset_tcp_z.items())
    if not args.bare:
        work_surface.verify_stage(stage,WORK_SURFACE,TABLE_Z)
        poses=[]
        for path,_ in bolt_paths:
            p,q=bolt_views[path].get_world_poses()
            poses.append(dict(p=np.asarray(p)[0].tolist(),q=np.asarray(q)[0].tolist()))
        work_surface.verify_bolts(WORK_SURFACE,TABLE_Z,poses)
    ok = held if args.bare else (not unreachable and held)
    print(f"  arms held at reset pose: {held}")
    print(f"gate: {'PASS' if ok else 'FAIL'}")

    # ---- D405 wrist captures ------------------------------------------------
    for side, cam in wrist_cams.items():
        if cam is None:
            continue
        img = None
        for _ in range(20):
            world.step(render=True)
            arr = cam.get_rgba()
            if arr is not None and np.asarray(arr).size:
                img = np.asarray(arr)
                break
        if img is None:
            print(f"  {side}: wrist camera returned no frame")
            continue
        tag = "tool" if args.hide_stand else ("solo" if args.only else ("bare" if args.bare else args.layout))
        o = ROOT / f"outputs/wristcam_{tag}_{side}.png"
        rgb = Image.fromarray(img[..., :3].astype("uint8"))
        rgb.save(o)

        # Policy-format frame: openpi resize_with_pad (openpi_client/image_tools.py).
        # 640x480 -> ratio max(640/224, 480/224) = 2.857 -> 224x168, then centred zero
        # padding of 28 rows top and bottom. The served config
        # pi05_pika_umi_wrist_velgrip_k1_h24_80k leaves resize_pad at its default True.
        ratio = max(rgb.width / 224, rgb.height / 224)
        rs = rgb.resize((int(rgb.width / ratio), int(rgb.height / ratio)), Image.BILINEAR)
        canvas = Image.new(rs.mode, (224, 224), 0)
        canvas.paste(rs, ((224 - rs.width) // 2, (224 - rs.height) // 2))
        op = ROOT / f"outputs/policyimg_{tag}_{side}.png"
        canvas.save(op)
        print(f"  {side:5s} wrote {o.name} + {op.name} ({rs.width}x{rs.height} padded to 224x224)")

    # ---- renders ----------------------------------------------------------
    # A straight-down look_at is degenerate (view dir parallel to world up), so the top
    # camera is nudged along +x and looks back toward -x. That puts the robot at the TOP of
    # frame with world +y to the image RIGHT -- i.e. gray box/gray bolts (left arm, +y) on
    # the right and green box/black bolts (right arm, -y) on the left, matching
    # expected_sim_montage.png. rep.create.camera(rotation=...) renders empty here.
    for tag, pos, look in (
        ("overview", (1.85, -1.35, 1.15), (0.55, 0.0, 0.15)),
        ("top", (1.12, 0.0, 1.62), (0.52, 0.0, 0.0)),
    ):
        cam = rep.create.camera(position=pos, look_at=look, clipping_range=(0.01, 100.0))
        rp = rep.create.render_product(cam, (1280, 960))
        ann = rep.AnnotatorRegistry.get_annotator("rgb")
        ann.attach([rp])
        arr = np.asarray([])
        for _ in range(20):
            rep.orchestrator.step(rt_subframes=4, pause_timeline=False)
            arr = np.asarray(ann.get_data())
            if arr.size:
                break
        out = ROOT / f"outputs/scene_{args.layout}_{tag}.png"
        Image.fromarray(arr[..., :3].astype("uint8")).save(out)
        print(f"wrote {out.name}")
        ann.detach([rp])

    stage.Export(str(ROOT / f"assets/scene_{args.layout}.usd"))
    app.close()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
