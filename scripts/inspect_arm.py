import os, sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})
import numpy as np, eval_closed_loop as E
from isaacsim.core.api import World
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
from isaacsim.core.utils.types import ArticulationAction
from pxr import Gf, UsdGeom, UsdPhysics, Usd

world = World(stage_units_in_meters=1.0, physics_dt=E.PHYSICS_DT, rendering_dt=E.PHYSICS_DT)
stage = get_current_stage()
UsdGeom.Xform.Define(stage, "/World")
ps = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
ps.CreateGravityDirectionAttr().Set(Gf.Vec3f(0,0,-1)); ps.CreateGravityMagnitudeAttr().Set(9.81)
add_reference_to_stage(usd_path=str(E.ARM_USD), prim_path="/World/robot")
world.reset()
art = SingleArticulation(prim_path="/World/robot", name="arm"); art.initialize()
names = list(art.dof_names)
print("  DOF names:", names, flush=True)
fl, fr = names.index("finger_left_joint"), names.index("finger_right_joint")
print("  FINGER_TRAVEL_M =", E.FINGER_TRAVEL_M, flush=True)

def xyz(prim):
    t = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).ExtractTranslation()
    return np.array([t[0],t[1],t[2]])

fp = {n: next(x for x in stage.Traverse() if x.GetName()==n and x.HasAPI(UsdPhysics.RigidBodyAPI))
      for n in ("finger_left","finger_right")}
for n,p in fp.items(): print(f"  {n}: {p.GetPath()}", flush=True)

for pct in (100.0, 50.0, 0.0):
    q = np.asarray(art.get_joint_positions(), dtype=np.float32).reshape(-1)
    f = (1.0 - pct/100.0) * E.FINGER_TRAVEL_M
    q[fl], q[fr] = +f, -f
    art.apply_action(ArticulationAction(joint_positions=q))
    for _ in range(400): world.step(render=False)
    qn = np.asarray(art.get_joint_positions(), dtype=np.float32).reshape(-1)
    gap = np.linalg.norm(xyz(fp["finger_left"]) - xyz(fp["finger_right"]))*1000
    print(f"  grip {pct:5.1f}%: 목표 fl={+f:+.4f} fr={-f:+.4f} | 실제 fl={qn[fl]:+.4f} fr={qn[fr]:+.4f} | 링크간격 {gap:.1f}mm", flush=True)
app.close()
