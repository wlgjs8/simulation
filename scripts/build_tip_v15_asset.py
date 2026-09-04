#!/usr/bin/env python3
"""USD half of the v15 fingertip swap -- called by build_tip_v15.py, not run directly.

The rig's arm asset is a URDF-importer bundle with a fixed composition chain:

    rb3_730e_pika_articulated_sim.usda      references payloads/base.usda,
                                            variantSet Physics = "physx"
      payloads/base.usda                    subLayers robot.usda; holds the link tree.
                                            Each finger LINK has two child Xforms that
                                            reference instances.usda (visual, collider).
        payloads/instances.usda             references geometries.usd, binds materials
          payloads/geometries.usd           the baked meshes
      payloads/Physics/physx.usda           subLayers physics.usda
        payloads/Physics/physics.usda       per-link PhysicsMassAPI / physics:mass

So the swap needs to touch exactly three things, and it does them in a COPY of the bundle
(assets/rb3_730e_pika_tip_v15/) so the original stays byte-identical. That matters twice
over: every validated number this project has was measured on the old tip, and stage 2
(TPU contact compliance) needs a stage-1 baseline to A/B against.

  1. a new payloads/tip_v15.usda carrying the spine/blade meshes, the convex colliders
     and the two materials
  2. payloads/base.usda: repoint each finger link's two children at it
  3. payloads/Physics/physics.usda: explicit finger mass (see build_tip_v15.py's MASS note)

(2) and (3) are TEXT edits, not pxr round-trips. Re-serialising base.usda through Usd
would reformat all 48k lines and destroy any ability to diff the copy against the original,
which is the one cheap check that this swap changed nothing else.

NAMING IS LOAD-BEARING. eval_closed_loop.py binds the fingertip/table friction material by
matching `"finger" in prim.GetName()` over every collider, so the new collision meshes are
named finger_pla_hull / finger_tpu_NNN. Rename them and FINGER_FRICTION silently stops
reaching the jaws -- the run would abort on the existing zero-collider guard rather than
lie, but it would still be a wasted run.
"""
from __future__ import annotations

import pathlib
import re
import shutil

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

FINGER_LINK = ("/rb3_730e_pika_articulated_sim/Geometry/world/link0/link1/link2/link3"
               "/link4/link5/link6/attachment_site")


def _mesh(stage, path, tm, collider=None, sdf_res=0):
    m = UsdGeom.Mesh.Define(stage, path)
    m.CreatePointsAttr([Gf.Vec3f(*p) for p in tm.vertices.astype(float)])
    m.CreateFaceVertexCountsAttr([3] * len(tm.faces))
    m.CreateFaceVertexIndicesAttr(tm.faces.reshape(-1).tolist())
    lo, hi = tm.bounds
    m.CreateExtentAttr([Gf.Vec3f(*lo.astype(float)), Gf.Vec3f(*hi.astype(float))])
    # STL is faceted by construction; asking USD to subdivide would round the arch away.
    m.CreateSubdivisionSchemeAttr(UsdGeom.Tokens.none)
    p = m.GetPrim()
    if collider:
        UsdPhysics.CollisionAPI.Apply(p)
        UsdPhysics.MeshCollisionAPI.Apply(p).CreateApproximationAttr().Set(collider)
        if collider == "sdf" and sdf_res:
            # PhysxSDFMeshCollisionAPI is an extension schema and is not importable outside
            # a running Kit app, so the attribute is authored directly and the API name is
            # appended to apiSchemas through Sdf. PhysX reads the token either way; applying
            # the typed schema would only add editor validation we cannot run here anyway.
            p.CreateAttribute("physxSDFMeshCollision:sdfResolution",
                              Sdf.ValueTypeNames.Int, custom=False).Set(sdf_res)
            spec = stage.GetRootLayer().GetPrimAtPath(p.GetPath())
            lst = spec.GetInfo("apiSchemas")
            lst.prependedItems = list(lst.prependedItems) + ["PhysxSDFMeshCollisionAPI"]
            spec.SetInfo("apiSchemas", lst)
        UsdGeom.Imageable(p).CreatePurposeAttr().Set(UsdGeom.Tokens.guide)
    return m


def _material(stage, path, linear_rgb):
    mat = UsdShade.Material.Define(stage, path)
    sh = UsdShade.Shader.Define(stage, f"{path}/PreviewSurface")
    sh.CreateIdAttr("UsdPreviewSurface")
    sh.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*linear_rgb))
    sh.CreateInput("metallic", Sdf.ValueTypeNames.Float).Set(0.0)
    sh.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
    sh.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(1.0)
    mat.CreateSurfaceOutput().ConnectToSource(sh.ConnectableAPI(), "surface")
    return mat


def _write_geometry_layer(path, parts, pla_hull, tpu_parts, tpu_linear, pla_linear,
                          mode, sdf_res):
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    root = UsdGeom.Scope.Define(stage, "/TipV15")
    stage.SetDefaultPrim(root.GetPrim())
    n_col = 0
    for side in ("left", "right"):
        UsdGeom.Xform.Define(stage, f"/TipV15/{side}_vis")
        mats = UsdGeom.Scope.Define(stage, f"/TipV15/{side}_vis/VisualMaterials")
        m_pla = _material(stage, f"{mats.GetPath()}/pla_dark", pla_linear)
        m_tpu = _material(stage, f"{mats.GetPath()}/tpu_yellow", tpu_linear)
        g = _mesh(stage, f"/TipV15/{side}_vis/finger_pla", parts[side]["pla"])
        UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(m_pla)
        g = _mesh(stage, f"/TipV15/{side}_vis/finger_tpu", parts[side]["tpu"])
        UsdShade.MaterialBindingAPI.Apply(g.GetPrim()).Bind(m_tpu)

        UsdGeom.Xform.Define(stage, f"/TipV15/{side}_col")
        # The spine never reaches an object: its own front face sits 7.9 mm behind the blade's
        # contact face, so a hull cannot add material anywhere a bolt can be. Hull it and
        # spend the collider budget on the blade.
        _mesh(stage, f"/TipV15/{side}_col/finger_pla_hull", pla_hull[side],
              collider="convexHull")
        n_col += 1
        if mode == "sdf":
            _mesh(stage, f"/TipV15/{side}_col/finger_tpu_sdf", parts[side]["tpu"],
                  collider="sdf", sdf_res=sdf_res)
            n_col += 1
        else:
            for i, p in enumerate(tpu_parts[side]):
                _mesh(stage, f"/TipV15/{side}_col/finger_tpu_{i:03d}", p,
                      collider="convexHull")
                n_col += 1
    stage.GetRootLayer().Save()
    return n_col


# The child Xform the URDF importer wrote under each finger link. Two per side: the visual
# and the collider, both instanceable, both scaled 0.001 because the meshes are in mm.
_OLD = """                                                def Xform "pika_finger_{side}{suffix}" (
                                                    instanceable = true
                                                    prepend references = @./instances.usda@</Instances/pika_finger_{side}{suffix}>
                                                )"""
_NEW = """                                                def Xform "{name}" (
                                                    prepend references = @./tip_v15.usda@</TipV15/{side}_{part}>
                                                )"""


def _patch_base(path):
    """Repoint both finger children at the new layer. Instancing is deliberately dropped:
    the original bundle instanced one collider per finger, this one has ~24, and a
    mis-composed instance proxy is far harder to diagnose than the few MB it would save
    across the two arms in the scene.

    Matched by REGEX, not by a literal block: the leading indentation encodes how deep the
    finger link sits in the USD, and that depth is a property of the ARM (RB5 has a longer
    link chain than RB3). Hardcoding it made this patch silently arm-specific -- it aborted
    on the first RB5 import, 2026-09-05. The indent is captured and reused so the rewritten
    block stays aligned with its neighbours.
    """
    txt = path.read_text()
    n = 0
    # WHICH VISUALS THE IMPORT ALREADY HAS. The RB3 display URDF draws one monolithic
    # `pika_finger_<side>`, so the v15 visual had to be grafted in. The RB5 display URDF
    # (robotics_lab make_rb5_850e_urdfs.py build_display) already references
    # pika_finger_<side>_{pla,tpu}.STL -- the printed tip is native there, and repointing it
    # would replace the correct meshes with an identical copy. Detected, not configured, so
    # a future arm cannot pick the wrong branch by forgetting a flag.
    native_v15 = '"pika_finger_left_pla"' in txt
    targets = ((("_hull", "tip_v15_col", "col"),) if native_v15
               else (("", "tip_v15_vis", "vis"), ("_hull", "tip_v15_col", "col")))
    print(f"  base.usda: v15 visuals {'already native (collider only)' if native_v15 else 'grafted'}")
    for side in ("left", "right"):
        for suffix, name, part in targets:
            pat = re.compile(
                r'^(?P<ind>[ \t]*)def Xform "pika_finger_' + side + suffix + r'" \(\n'
                r'(?P=ind)    instanceable = true\n'
                r'(?P=ind)    prepend references = @\./instances\.usda@</Instances/'
                + f"pika_finger_{side}{suffix}" + r'>\n'
                r'(?P=ind)\)',
                re.MULTILINE)
            m = pat.search(txt)
            if m is None:
                raise SystemExit(f"ABORT: base.usda has no pika_finger_{side}{suffix} "
                                 f"instance block to repoint -- the importer output "
                                 f"changed shape, not just its indentation.")
            ind = m.group("ind")
            new = (f'{ind}def Xform "{name}" (\n'
                   f'{ind}    prepend references = @./tip_v15.usda@</TipV15/{side}_{part}>\n'
                   f'{ind})')
            txt = txt[:m.start()] + new + txt[m.end():]
            n += 1
    path.write_text(txt)
    return n


_MASS_OLD = """                                            over "finger_{side}" (
                                                prepend apiSchemas = ["PhysicsRigidBodyAPI"]
                                            )
                                            {{
                                            }}"""
_MASS_NEW = """                                            over "finger_{side}" (
                                                prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]
                                            )
                                            {{
                                                float physics:mass = {mass:.6f}
                                            }}"""


def _patch_mass(path, mass):
    """Same indentation caveat as _patch_base -- match the `over` by regex, not by depth."""
    txt = path.read_text()
    for side in ("left", "right"):
        pat = re.compile(
            r'^(?P<ind>[ \t]*)over "finger_' + side + r'" \(\n'
            r'(?P=ind)    prepend apiSchemas = \["PhysicsRigidBodyAPI"\]\n'
            r'(?P=ind)\)\n'
            r'(?P=ind)\{\n'
            r'(?P=ind)\}',
            re.MULTILINE)
        m = pat.search(txt)
        if m is None:
            raise SystemExit(f"ABORT: physics.usda has no empty finger_{side} rigid-body "
                             f"block -- mass may already be authored; check before rebuilding.")
        ind = m.group("ind")
        new = (f'{ind}over "finger_{side}" (\n'
               f'{ind}    prepend apiSchemas = ["PhysicsRigidBodyAPI", "PhysicsMassAPI"]\n'
               f'{ind})\n'
               f'{ind}{{\n'
               f'{ind}    float physics:mass = {mass:.6f}\n'
               f'{ind}}}')
        txt = txt[:m.start()] + new + txt[m.end():]
    path.write_text(txt)


def build_asset(parts, pla_hull, tpu_parts, mass, src_asset, out_asset,
                tpu_linear, pla_linear, mode="sdf", sdf_res=256):
    print(f"\n== asset -> {out_asset.name}  (blade collision: {mode})")
    if out_asset.exists():
        shutil.rmtree(out_asset)
    shutil.copytree(src_asset, out_asset)
    # The bundle is named after its root file; keep the file name and the internal prim
    # name identical to the original so every relative reference inside still resolves and
    # a diff against the source directory shows only the three intended files.
    n_col = _write_geometry_layer(out_asset / "payloads/tip_v15.usda",
                                  parts, pla_hull, tpu_parts, tpu_linear, pla_linear,
                                  mode, sdf_res)
    n = _patch_base(out_asset / "payloads/base.usda")
    _patch_mass(out_asset / "payloads/Physics/physics.usda", mass)
    print(f"  payloads/tip_v15.usda: {n_col} colliders, 2 materials")
    print(f"  payloads/base.usda: repointed {n} finger children")
    print(f"  payloads/Physics/physics.usda: physics:mass = {mass:.6f} kg on both fingers")
    print(f"  root: {out_asset.name}/{src_asset.name}.usda")
    return out_asset
