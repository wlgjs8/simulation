#!/usr/bin/env python3
"""Turn robotics_lab's RB5-850E display URDF into one Isaac Sim can actually simulate.

The RB3 counterpart is `make_articulated_urdf.py`; this is its RB5 replacement, written for
the 2026-09-05 RB3 -> RB5 port. Source is robotics_lab's OWN generated file
`descriptions/urdf/rb5_850e_pika_articulated.urdf` (itself produced by that repo's
`tools/make_rb5_850e_urdfs.py`, which guarantees the kinematics are bit-identical to
upstream). Nothing here moves a joint or a link -- every edit below is about making a
DISPLAY urdf survive a physics engine.

WHAT IT FIXES, and why each one is not optional
===============================================
1. `elbow_joint` carries `effort="0.0"` (line 211 of the source, and the same in
   `rb5_850e.urdf` and `dual_rb5_850e_ver3.urdf`). It is inherited from upstream and is
   harmless in robotics_lab because every consumer there is Pinocchio, which ignores effort.
   **Isaac reads it and gives J3 a zero torque limit, so the arm folds under its own
   weight.** Its five siblings all carry 20000.0; the elbow gets the same.
2. `link0` has no `<inertial>`. It is the articulation root and the cell fixes the base, so
   this is usually survivable -- but PhysX then invents mass properties and warns, and the
   rig has been bitten before by exactly this class of silent default (negative-mass marker
   links, CLAUDE.md section 10). Give it an explicit one.
3. The tool and both fingers have `<visual>` only, so PhysX would see NOTHING where the
   gripper is -- it could not touch a bolt, a box, or the table. Attach the convex hulls
   robotics_lab itself uses for its collision monitor (`stack_real.yaml` gripper_meshes).
4. Mesh paths are relative to the source file; resolve them absolute so the importer can be
   run from anywhere.

WHAT IT DELIBERATELY DOES NOT DO
================================
* No frame-marker spheres to shrink: unlike the RB3 file, this one has none.
* No tool-visual rotation. The RB3 monolithic urdf needed a +90 deg Z baked into the mesh
  (CLAUDE.md section 11); every RB5 file uses identity and the exported meshes already
  carry it.
* No tip swap. The v15 PLA+TPU tip is applied downstream by `build_tip_v15.py`, which reads
  its meshes from the rb5 tool directory already -- the tip is the same part on both arms
  and `boltv2`, the campaign the served checkpoints are trained on, was collected with it.
"""

import pathlib
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parent.parent
DESC = pathlib.Path.home() / "workspace/robotics_lab/rb_servo_server/descriptions"
SRC = DESC / "urdf/rb5_850e_pika_articulated.urdf"
OUT_URDF = ROOT / "assets/urdf"
NAME = "rb5_850e_pika_articulated_sim"

TOOL_DIR = DESC / "meshes/robots/rb5_850e/visual/tool"
HULLS = {
    "tool": TOOL_DIR / "pika_gripper_base_hull.STL",
    "finger_left": TOOL_DIR / "pika_finger_left_hull.STL",
    "finger_right": TOOL_DIR / "pika_finger_right_hull.STL",
}
# The five non-elbow arm joints all carry this in the source; the elbow is the odd one out.
ARM_EFFORT = "20000.0"
# link0 is the fixed root. The number only has to be finite and sane -- it is never
# accelerated -- so use the source's own link1 mass rather than inventing a scale.
LINK0_MASS = 4.147
LINK0_INERTIA = 0.05
# FRAME LINKS. `tcp`, `attachment_site` and the two ft_sensor frames are pure coordinate
# frames -- empty <link> elements with no inertial, visual or collision. The URDF importer
# turns a link with no content into a plain Xform, NOT a PhysX body, so `tcp` cannot be read
# back as a physics pose and every consumer that resolves it by RigidBodyAPI comes up empty
# (build_cell.py wrote an empty cell_reset_pose.json on the first RB5 build, 2026-09-05).
# The RB3 asset got bodies here only by accident: its frame links carried a marker sphere.
# That accident also cost it the "negative mass and unit inertia tensor" warnings in
# CLAUDE.md section 10, because PhysX then had a shape but no mass properties. Give them a
# small, VALID inertial instead: enough to be a body, small enough to be dynamically inert
# on a fixed joint.
FRAME_LINKS = ("tcp", "attachment_site", "ft_sensor_base", "ft_sensor_measurement")
FRAME_MASS = 1e-4
FRAME_INERTIA = 1e-8
FINGER_TRAVEL_M = 0.049   # stack_real.yaml gripper_finger_travel_m, and the v15 jaw


def main() -> int:
    if not SRC.exists():
        print(f"FAIL: missing {SRC}")
        return 1
    OUT_URDF.mkdir(parents=True, exist_ok=True)
    tree = ET.parse(SRC)
    root = tree.getroot()

    for mesh in root.iter("mesh"):
        fn = mesh.get("filename")
        if fn:
            mesh.set("filename", str((SRC.parent / fn).resolve()))

    fixed_effort = []
    for j in root.findall("joint"):
        lim = j.find("limit")
        if lim is None:
            continue
        if j.get("name", "").endswith("_joint") and lim.get("effort") == "0.0":
            lim.set("effort", ARM_EFFORT)
            fixed_effort.append(j.get("name"))

    def _inertial(mass, inertia, z=0.0):
        it = ET.Element("inertial")
        ET.SubElement(it, "origin", {"xyz": f"0.0 0.0 {z}", "rpy": "0.0 0.0 0.0"})
        ET.SubElement(it, "mass", {"value": str(mass)})
        ET.SubElement(it, "inertia", {"ixx": str(inertia), "ixy": "0.0", "ixz": "0.0",
                                      "iyy": str(inertia), "iyz": "0.0", "izz": str(inertia)})
        return it

    added_frames = []
    for link in root.findall("link"):
        if link.get("name") in FRAME_LINKS and link.find("inertial") is None:
            link.insert(0, _inertial(FRAME_MASS, FRAME_INERTIA))
            added_frames.append(link.get("name"))

    added_inertial = []
    for link in root.findall("link"):
        if link.get("name") == "link0" and link.find("inertial") is None:
            it = ET.Element("inertial")
            ET.SubElement(it, "origin", {"xyz": "0.0 0.0 0.05", "rpy": "0.0 0.0 0.0"})
            ET.SubElement(it, "mass", {"value": str(LINK0_MASS)})
            ET.SubElement(it, "inertia", {"ixx": str(LINK0_INERTIA), "ixy": "0.0",
                                          "ixz": "0.0", "iyy": str(LINK0_INERTIA),
                                          "iyz": "0.0", "izz": str(LINK0_INERTIA)})
            link.insert(0, it)
            added_inertial.append("link0")

    added_hull = []
    for link in root.findall("link"):
        name = link.get("name")
        if name in HULLS and not link.findall("collision"):
            hull = HULLS[name]
            if not hull.exists():
                print(f"FAIL: missing hull {hull}")
                return 1
            col = ET.SubElement(link, "collision")
            ET.SubElement(col, "origin", {"xyz": "0.0 0.0 0.0", "rpy": "0.0 0.0 0.0"})
            g = ET.SubElement(col, "geometry")
            ET.SubElement(g, "mesh", {"filename": str(hull.resolve()),
                                      "scale": "0.001 0.001 0.001"})
            added_hull.append(name)

    print(f"source            {SRC}")
    print(f"elbow effort fix  {fixed_effort or 'NONE FOUND -- check the source changed'}"
          f"  -> {ARM_EFFORT}")
    print(f"inertial added    {added_inertial or 'none needed'}")
    print(f"frame bodies      {added_frames or 'none needed'}  "
          f"(mass {FRAME_MASS} kg, so tcp is readable as a physics pose)")
    print(f"collision hulls   {sorted(added_hull)}")
    for j in root.findall("joint"):
        if j.get("name") in ("finger_left_joint", "finger_right_joint"):
            lim = j.find("limit")
            print(f"  {j.get('name'):20s} {j.get('type'):10s} "
                  f"axis={j.find('axis').get('xyz')}  "
                  f"limit=({lim.get('lower')}, {lim.get('upper')})")
    print(f"jaw model: finger_pos = (1 - grip/100) * {FINGER_TRAVEL_M}  "
          f"[grip 100 = open, 0 = closed]")

    if not fixed_effort:
        print("WARN: no zero-effort joint found. Either upstream fixed it (good, drop this "
              "step) or the joint was renamed (bad, J3 will collapse silently).")

    root.set("name", NAME)
    out = OUT_URDF / f"{NAME}.urdf"
    tree.write(out, encoding="utf-8", xml_declaration=True)
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
