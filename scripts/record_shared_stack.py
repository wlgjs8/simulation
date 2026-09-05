"""Read-only pose and wrist-camera instrumentation of the shared Sim stack."""
import json
import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--diagnostics-dir', required=True, type=Path)
options, forwarded = parser.parse_known_args()
if '--shared-stack' not in forwarded:
    parser.error('recording requires --shared-stack; pass evaluation options after --diagnostics-dir')
OUT = options.diagnostics_dir.resolve()
OUT.mkdir(parents=True, exist_ok=False)
sys.argv = [sys.argv[0], *forwarded]
sys.path.insert(0, str(ROOT / 'scripts'))
import shared_stack

original_run = shared_stack.run_episode

def instrumented(args, settings, scene, world, rep, arts, cameras, tcp_views,
                 bolt_views, bolt_prims, overview, materials):
    import numpy as np
    import imageio.v2 as imageio
    from pxr import UsdGeom, Usd, Gf
    from isaacsim.core.prims import RigidPrim
    import shared_contact

    measured_time = [1_000_000_000]
    stream = (OUT / 'poses.jsonl').open('w')
    videos = {side:imageio.get_writer(str(OUT/f'{side}_wrist.mp4'),fps=30) for side in arts}
    fingers = {}
    geometry = {}
    for side in arts:
        fingers[side] = {}
        for name in ['finger_left','finger_right']:
            prim = next(p for p in world.stage.Traverse() if p.GetName()==name
                        and str(p.GetPath()).startswith(f'/World/cell/{side}_arm/'))
            view = RigidPrim(prim_paths_expr=str(prim.GetPath()),
                             name=f'diagnostic_{side}_{name}',reset_xform_properties=False)
            view.initialize()
            fingers[side][name]=view
            inv=UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default()).GetInverse()
            mesh=world.stage.GetPrimAtPath(str(prim.GetPath())+'/tip_v15_col/finger_tpu_sdf')
            tf=UsdGeom.Xformable(mesh).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            geometry[f'{side}_{name}']=np.array([inv.Transform(tf.Transform(Gf.Vec3d(*map(float,p))))
                                               for p in UsdGeom.Mesh(mesh).GetPointsAttr().Get()])
    np.savez_compressed(OUT/'tip_geometry.npz',**geometry)

    def pose(view):
        p,q=view.get_world_poses()
        return [*np.asarray(p)[0].tolist(),*np.asarray(q)[0].tolist()]

    original_sensor=shared_contact.ToolSensor
    class Sensor(original_sensor):
        def measure(self,side,time_ns,seq):
            result=super().measure(side,time_ns,seq)
            if side=='left':
                measured_time[0]=time_ns
                if seq%5==1:
                    arms={}
                    for s,art in arts.items():
                        q=np.asarray(art.get_joint_positions());names=list(art.dof_names)
                        arms[s]=dict(tcp=pose(tcp_views[s]),fingers={k:pose(v) for k,v in fingers[s].items()},
                                     finger_q_m=[float(q[names.index(j)]) for j in ['finger_left_joint','finger_right_joint']])
                    stream.write(json.dumps(dict(time_ns=time_ns,arms=arms,
                        bolts={p.split('/')[-1]:pose(v) for p,v in bolt_views.items()}))+'\n')
            return result
    shared_contact.ToolSensor=Sensor
    original_rgba={s:cam.get_rgba for s,cam in cameras.items()}
    last={s:-1 for s in cameras}
    def recorder(side):
        def get(*a,**kw):
            rgba=original_rgba[side](*a,**kw)
            if measured_time[0]!=last[side]:
                videos[side].append_data(np.asarray(rgba)[...,:3]);last[side]=measured_time[0]
            return rgba
        return get
    for s,cam in cameras.items():cam.get_rgba=recorder(s)
    try:
        return original_run(args,settings,scene,world,rep,arts,cameras,tcp_views,bolt_views,bolt_prims,overview,materials)
    finally:
        stream.close()
        for s,v in videos.items():v.close();cameras[s].get_rgba=original_rgba[s]
        shared_contact.ToolSensor=original_sensor

shared_stack.run_episode=instrumented
import eval_closed_loop
raise SystemExit(eval_closed_loop.main())
