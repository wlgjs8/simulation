#!/usr/bin/env python3
"""Extract a stand-only URDF from robotics_lab's dual_rb5_850e_ver3.urdf.

WHY NOT JUST IMPORT THE DUAL FILE. It carries both arms as well, and only as collision
hulls (its visuals were stripped by robotics_lab's generator). Referencing it into the cell
would put a second, invisible collider around each arm on top of the real one -- two bodies
in the same place is not a conservative default, it is a physics bug that renders correctly.

WHY NOT HAND-WRITE THE MOUNT POSES. Because CLAUDE.md section 4 says not to, and the RB3
history is the reason: the montage hand-composed the stand transform, got it 90 deg out, and
the error survived a visual check because the assembly was self-consistent. Everything here
is PARSED from the dual URDF -- the two arm-base origins, the stand visual, and the 20 CoACD
collision hulls -- so the only way this file can be wrong is if the source is.

What comes out: `world -> base -> stand`, the stand visual and its hulls, and the two
`stand_{left,right}_arm_base` frames the cell builder attaches arms to (same names as the
RB3 asset, so build_cell.py needs no new frame vocabulary).

On RB5, world == base == stand: `base_fixed` and `stand_fixed` are both identity in the
source, where the RB3 stand carried base->stand = +90 deg about Z. That rotation is the one
the 2026-09-02 robotics_lab swap had to unwind everywhere (rb_gui placeholders, the ROI box),
and it is why the arm-base joints below are already WORLD poses.
"""

import pathlib
import xml.etree.ElementTree as ET

ROOT = pathlib.Path(__file__).resolve().parent.parent
DESC = pathlib.Path.home() / "workspace/robotics_lab/rb_servo_server/descriptions"
SRC = DESC / "urdf/dual_rb5_850e_ver3.urdf"
OUT = ROOT / "assets/urdf/dual_rb5_850e_stand_only.urdf"
NAME = "dual_rb5_850e_stand_only"

KEEP_LINKS = ("base", "stand", "stand_collision",
              "stand_left_arm_base", "stand_right_arm_base")
KEEP_JOINTS = ("base_fixed", "stand_fixed", "stand_collision_fixed",
               "stand_left_arm_base_fixed", "stand_right_arm_base_fixed")


def main() -> int:
    if not SRC.exists():
        print(f"FAIL: missing {SRC}")
        return 1
    src = ET.parse(SRC).getroot()
    links = {l.get("name"): l for l in src.findall("link")}
    joints = {j.get("name"): j for j in src.findall("joint")}

    missing = [n for n in KEEP_LINKS if n not in links] + \
              [n for n in KEEP_JOINTS if n not in joints]
    if missing:
        print(f"FAIL: source is missing {missing} -- the stand was restructured upstream, "
              f"re-read dual_rb5_850e_ver3.urdf before editing this list.")
        return 1

    out = ET.Element("robot", {"name": NAME})
    ET.SubElement(out, "link", {"name": "world"})
    for n in KEEP_LINKS:
        out.append(links[n])
    for n in KEEP_JOINTS:
        out.append(joints[n])

    for mesh in out.iter("mesh"):
        fn = mesh.get("filename")
        if fn:
            mesh.set("filename", str((SRC.parent / fn).resolve()))

    n_vis = sum(len(l.findall("visual")) for l in out.findall("link"))
    n_col = sum(len(l.findall("collision")) for l in out.findall("link"))
    print(f"source        {SRC}")
    print(f"links {len(KEEP_LINKS)}  joints {len(KEEP_JOINTS)}  "
          f"visual {n_vis}  collision {n_col}")
    for n in KEEP_JOINTS:
        o = joints[n].find("origin")
        if o is not None:
            print(f"  {n:28s} xyz={o.get('xyz')}  rpy={o.get('rpy')}")
    OUT.parent.mkdir(parents=True, exist_ok=True)
    ET.ElementTree(out).write(OUT, encoding="utf-8", xml_declaration=True)
    print(f"wrote {OUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
