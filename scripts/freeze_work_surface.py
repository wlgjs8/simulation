"""Create verified, settled pad scenes without connecting a policy or servo backend."""
import argparse
import json
import math
from pathlib import Path
import sys
import numpy as np
import eval_closed_loop as scene
import shared_stack


def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--output',required=True)
    ap.add_argument('--count',type=int,default=40)
    ap.add_argument('--seed',type=int,default=100)
    ap.add_argument('--layout',choices=['aligned','random'],default='aligned')
    args=ap.parse_args()
    target=Path(args.output)
    if target.exists() or args.count <= 0:
        ap.error('output must be new; count must be positive')

    def freeze(runtime, settings, scene, world, rep, arts, cameras, tcp_views,
               bolt_views, bolt_prims, overview, materials):
        from isaacsim.core.utils.types import ArticulationAction
        scene.work_surface.verify_stage(world.stage,scene.WORK_SURFACE,scene.TABLE_Z)
        for side,art in arts.items():
            names=list(art.dof_names);q=np.zeros(len(names),dtype=np.float32)
            q[[names.index(j) for j in scene.ARM_JOINTS]]=np.deg2rad(scene.RESET[side])
            art.set_joint_positions(q);art.set_joint_velocities(np.zeros_like(q))
            art.get_articulation_controller().set_gains(
                kps=np.full(len(names),settings['arm_stiffness']),
                kds=np.full(len(names),settings['arm_damping']))
            art.apply_action(ArticulationAction(joint_positions=q))
        captured={}
        for seed in range(args.seed,args.seed+args.count):
            placements=scene.bolt_poses(args.layout,runtime.n_per_color,seed)
            for path,(_,x,y,yaw) in zip(bolt_prims,placements):
                # dropped level from just above the largest head radius (a pre-tilted drop at the rest
                # angle was tried for the button heads and did not reduce their settling roll)
                bolt_views[path].set_world_poses(
                    np.array([[x,y,scene.PICK_SURFACE_Z+scene.bolt_max_radius()+.001]]),
                    np.array([[math.cos(yaw/2),0,0,math.sin(yaw/2)]]))
                bolt_views[path].set_velocities(np.zeros((1,6)))
            for _ in range(1000):world.step(render=False)
            def bolt_speeds():
                return [float(np.linalg.norm(np.asarray(bolt_views[path].get_velocities())[0,:3]))
                        for path in bolt_prims]
            # The historical cylinders always settled inside 1000 steps, so their scenes are unchanged.
            # A button-head bolt lies on its dome rim and shaft tip like a cone and can keep rolling
            # in place with no rolling friction; any roll angle is an equilibrium, so stepping on until
            # it stops (and freezing it at rest) is faithful. Capped so a genuinely unstable scene fails.
            extra=0
            while max(bolt_speeds())>.01 and extra<6000:
                for _ in range(250):world.step(render=False)
                extra+=250
            poses=[];speeds=bolt_speeds()
            for path in bolt_prims:
                p,q=bolt_views[path].get_world_poses()
                poses.append(dict(p=np.asarray(p)[0].tolist(),q=np.asarray(q)[0].tolist()))
            if extra:
                print(f'[freeze-pad] seed {seed}: settled after {1000+extra} steps', flush=True)
            colors=[p[0] for p in placements]
            check=scene.work_surface.verify_bolts(scene.WORK_SURFACE,scene.TABLE_Z,poses,
                                                  [scene.bolt_aabb_proxies(c) for c in colors])
            if max(speeds)>.01:
                slow=[(bolt_prims[i].rsplit('/',1)[-1],round(v,3)) for i,v in enumerate(speeds) if v>.01]
                raise RuntimeError(f'seed {seed}: bolts did not settle after {1000+extra} steps: {slow}')
            captured[str(seed)]=dict(layout=args.layout,colors=colors,
                work_surface=scene.WORK_SURFACE,bolts=poses,
                support_check=check,max_linear_speed_m_s=max(speeds))
            if extra:
                captured[str(seed)]['settle_steps']=1000+extra
            if scene.bolt_geometry_metadata() is not None:
                # frozen poses are only valid for the geometry that settled them; the rig refuses a mismatch
                captured[str(seed)]['bolt_geometry']=scene.bolt_geometry_metadata()
            print(f'[freeze-pad] seed {seed}: {check}',flush=True)
        target.parent.mkdir(parents=True,exist_ok=True)
        with target.open('x') as f:json.dump(captured,f,indent=1)
        print(f'[freeze-pad] wrote {target}',flush=True)
        return 0

    shared_stack.run_episode=freeze
    sys.argv=[sys.argv[0],'--shared-stack','--episodes','1','--shared-hold','--no-video',
              '--work-surface','foam','--seed',str(args.seed),'--layout',args.layout]
    return scene.main()


if __name__=='__main__':raise SystemExit(main())
