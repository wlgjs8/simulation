import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import numpy as np, eval_closed_loop as E
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
from pxr import Gf, UsdGeom, UsdPhysics, Usd

world = World(stage_units_in_meters=1.0, physics_dt=E.PHYSICS_DT, rendering_dt=E.PHYSICS_DT)
stage = get_current_stage(); UsdGeom.Xform.Define(stage, "/World")
ps = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
ps.CreateGravityDirectionAttr().Set(Gf.Vec3f(0,0,-1)); ps.CreateGravityMagnitudeAttr().Set(9.81)
add_reference_to_stage(usd_path=str(E.ARM_USD), prim_path="/World/robot")
world.reset()
art = SingleArticulation(prim_path="/World/robot", name="arm"); art.initialize()

def wbox(prim):
    bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"], useExtentsHint=False)
    r = bc.ComputeWorldBound(prim).ComputeAlignedRange()
    return np.array(r.GetMin()), np.array(r.GetMax())

for nm in ("finger_left", "finger_right", "tcp", "attachment_site"):
    p = next((x for x in stage.Traverse() if x.GetName() == nm), None)
    if p is None: print(f"  {nm}: 없음"); continue
    t = UsdGeom.Xformable(p).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
    line = f"  {nm}: 원점 z={t[2]:.4f} xyz=({t[0]:+.4f},{t[1]:+.4f},{t[2]:.4f})"
    try:
        lo, hi = wbox(p); line += f" | bbox z {lo[2]:.4f}~{hi[2]:.4f}"
    except Exception as e: line += f" | bbox 실패"
    print(line, flush=True)
# 충돌 메시 하위까지
for x in stage.Traverse():
    if "finger_left" in str(x.GetPath()) and x.HasAPI(UsdPhysics.CollisionAPI):
        try:
            lo, hi = wbox(x)
            print(f"  [collider] {x.GetPath().name}: z {lo[2]:.4f}~{hi[2]:.4f}", flush=True)
        except Exception: pass
app.close()
