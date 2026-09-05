"""Hardware-free PhysX wrench validation; no policy/server connection or motion RPC.

Run with the Isaac Python environment, from the simulation directory:
  .venv-isaac/bin/python scripts/validate_shared_contact.py --tag wrench_validation
Applies known world-frame loads to each finger and checks the aggregate sensor
at the flange after subtracting the unloaded reading. Uses the production scene
and shared_contact payload/material configuration. Writes reviewable JSON.
"""
import json
import sys
import numpy as np

import eval_closed_loop as scene
import shared_stack


def validate(args, settings, scene, world, rep, arts, cameras, tcp_views,
             bolt_views, bolt_prims, overview, materials):
    from isaacsim.core.prims import RigidPrim
    from isaacsim.core.utils.types import ArticulationAction
    from scipy.spatial.transform import Rotation
    from shared_contact import ToolSensor
    from pxr import UsdPhysics, UsdShade

    out = scene.ROOT/'outputs/diagnostics'/args.tag
    out.mkdir(parents=True, exist_ok=False)
    indices = {}
    for side, art in arts.items():
        names = list(art.dof_names)
        indices[side] = np.array([names.index(j) for j in scene.ARM_JOINTS])
        fingers = [names.index(j) for j in ('finger_left_joint', 'finger_right_joint')]
        q = np.zeros(len(names), dtype=np.float32)
        q[indices[side]] = np.deg2rad(scene.RESET[side])
        art.set_joint_positions(q)
        art.set_joint_velocities(np.zeros_like(q))
        kp = np.full(len(names), settings['arm_stiffness'])
        kd = np.full(len(names), settings['arm_damping'])
        kp[fingers], kd[fingers] = settings['gripper_stiffness'], settings['gripper_damping']
        art.get_articulation_controller().set_gains(kps=kp, kds=kd)
        art.apply_action(ArticulationAction(joint_positions=q))
    world.step(render=False)
    sensor = ToolSensor(world, arts, settings,
                        (scene.ROOT/settings['robotics_lab']/settings['servo_config']).resolve(), indices)
    sensor.save(out/'plant_contact.json')
    bindings = {}
    for p in world.stage.Traverse():
        if p.GetName() == 'finger_tpu_sdf' and p.HasAPI(UsdPhysics.CollisionAPI):
            mat, _ = UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial('physics')
            bindings[str(p.GetPath())] = str(mat.GetPath())
    records = []
    for side in arts:
        flange_path = next(str(p.GetPath()) for p in world.stage.Traverse()
                           if p.GetName() == 'attachment_site' and f'/{side}_arm/' in str(p.GetPath()))
        flange = RigidPrim(flange_path, name=f'validation_flange_{side}', reset_xform_properties=False)
        flange.initialize()
        for finger in ('finger_left', 'finger_right'):
            load_body = RigidPrim(flange_path+'/'+finger, name=f'validation_load_{side}_{finger}',
                                  reset_xform_properties=False)
            load_body.initialize()
            for _ in range(300):
                world.step(render=False)

            def read_world():
                p, q = flange.get_world_poses()
                r = Rotation.from_quat(np.asarray(q)[0][[1, 2, 3, 0]])
                w = np.array(sensor.measure(side, 0, 0)['wrench'])
                return np.r_[r.apply(w[:3]), r.apply(w[3:])], np.asarray(p)[0]

            baseline = read_world()[0]
            tcp = np.asarray(tcp_views[side].get_world_poses()[0])[0]
            for axis in range(6):
                for sign in (-1, 1):
                    load = np.zeros(6)
                    load[axis] = sign*(10.0 if axis < 3 else 0.5)
                    samples = []
                    for tick in range(200):
                        load_body.apply_forces_and_torques_at_pos(
                            forces=load[:3][None].astype(np.float32),
                            torques=load[3:][None].astype(np.float32),
                            positions=tcp[None].astype(np.float32), is_global=True)
                        world.step(render=False)
                        if tick >= 150:
                            measured, p = read_world()
                            expected = np.r_[load[:3], load[3:]+np.cross(tcp-p, load[:3])]
                            samples.append(measured-baseline-expected)
                    error = np.mean(samples, axis=0)
                    records.append(dict(side=side, body=finger, axis=axis, sign=sign,
                                        applied_world=load.tolist(), error_world=error.tolist(),
                                        force_error_n=float(np.linalg.norm(error[:3])),
                                        torque_error_nm=float(np.linalg.norm(error[3:]))))
                    for _ in range(100):
                        world.step(render=False)
    result = dict(cases=records, collision_bindings=bindings,
                  max_force_error_n=max(r['force_error_n'] for r in records),
                  max_torque_error_nm=max(r['torque_error_nm'] for r in records),
                  tolerance_force_n=0.4, tolerance_torque_nm=0.05)
    result['passed'] = (len(bindings) == 4 and
                        result['max_force_error_n'] < result['tolerance_force_n'] and
                        result['max_torque_error_nm'] < result['tolerance_torque_nm'])
    (out/'wrench_validation.json').write_text(json.dumps(result, indent=2))
    print(json.dumps({k:v for k,v in result.items() if k not in ('cases', 'collision_bindings')}), flush=True)
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    shared_stack.run_episode = validate
    sys.argv += ['--shared-stack', '--shared-hold', '--no-video', '--episodes', '1']
    raise SystemExit(scene.main())
