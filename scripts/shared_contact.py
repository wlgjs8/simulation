"""PhysX tool inertia, compliant tips, and a six-axis sensor at the tool load path.

The force law is robotics_lab's. This module only configures/measures the plant.
Contact gains and effort limits are explicit provisional simulation parameters;
the aggregate payload mass/COM and sensor reference come from the source stack.
"""
import json
import math
from pathlib import Path
import numpy as np


def configure_materials(stage, settings):
    from pxr import PhysxSchema, Usd, UsdPhysics, UsdShade
    c = settings['contact']
    if not c['enabled']:
        if c['force_sensor']:
            raise ValueError('force sensor requires the configured contact/payload model')
        return
    for key in ['tip_stiffness_n_m', 'tip_damping_ns_m', 'settle_sec', 'tare_sec',
                'static_force_tolerance_n', 'static_torque_tolerance_nm']:
        if not math.isfinite(c[key]) or c[key] <= 0:
            raise ValueError(f'invalid contact setting: {key}')
    effort = np.asarray(c['arm_max_effort_nm'])
    if effort.shape != (6,) or not np.all(np.isfinite(effort) & (effort > 0)):
        raise ValueError('contact.arm_max_effort_nm requires six positive limits')
    material = UsdShade.Material.Define(stage, '/World/physmat_shared_tpu')
    UsdPhysics.MaterialAPI.Apply(material.GetPrim())
    api = PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim())
    api.CreateCompliantContactStiffnessAttr().Set(c['tip_stiffness_n_m'])
    api.CreateCompliantContactDampingAttr().Set(c['tip_damping_ns_m'])
    api.CreateCompliantContactAccelerationSpringAttr().Set(False)
    count = 0
    # The visual TPU prim is a sibling of tip_v15_col, not its ancestor.
    # De-instance only the collision reference so its TPU mesh can be authored
    # independently of the rigid PLA hull in the same reference.
    for prim in list(stage.Traverse()):
        if prim.GetName() == 'tip_v15_col':
            prim.SetInstanceable(False)
    for prim in stage.Traverse():
        if prim.HasAPI(UsdPhysics.RigidBodyAPI) and prim.GetName() in ('tool', 'finger_left', 'finger_right'):
            PhysxSchema.PhysxContactReportAPI.Apply(prim).CreateThresholdAttr().Set(0.0)
        if prim.GetName() == 'finger_tpu_sdf' and prim.HasAPI(UsdPhysics.CollisionAPI):
            UsdShade.MaterialBindingAPI.Apply(prim).Bind(
                material, bindingStrength=UsdShade.Tokens.strongerThanDescendants,
                materialPurpose='physics')
            count += 1
    if count != 4:
        raise RuntimeError(f'expected four TPU material bindings, got {count}')
    bound = [p for p in Usd.PrimRange(stage.GetPseudoRoot(), Usd.TraverseInstanceProxies())
             if p.GetName() == 'finger_tpu_sdf' and p.HasAPI(UsdPhysics.CollisionAPI)]
    if len(bound) != 4 or any(
            UsdShade.MaterialBindingAPI(p).ComputeBoundMaterial('physics')[0].GetPath()
            != material.GetPath() for p in bound):
        raise RuntimeError('TPU collision material binding verification failed')


class ContactRecorder:
    """Record actual collider/material pairs and solver impulses for diagnosis."""
    def __init__(self, path, clock):
        from omni.physx import get_physx_simulation_interface
        self.stream = Path(path).open('w')
        self.clock = clock
        self.subscription = get_physx_simulation_interface().subscribe_contact_report_events(self.record)

    def record(self, headers, data):
        from pxr import PhysicsSchemaTools
        for header in headers:
            if not header.num_contact_data:
                continue
            points = data[header.contact_data_offset:header.contact_data_offset+header.num_contact_data]
            # The callback occurs inside the world.step before the caller advances
            # the shared clock, so these are the samples for the upcoming tick.
            self.stream.write(json.dumps(dict(
                time_ns=self.clock.now_ns()+self.clock.dt_ns,
                collider0=str(PhysicsSchemaTools.intToSdfPath(header.collider0)),
                collider1=str(PhysicsSchemaTools.intToSdfPath(header.collider1)),
                material0=str(PhysicsSchemaTools.intToSdfPath(points[0].material0)),
                material1=str(PhysicsSchemaTools.intToSdfPath(points[0].material1)),
                count=len(points),
                force_world_n=(np.sum([list(p.impulse) for p in points],axis=0)/(self.clock.dt_ns*1e-9)).tolist(),
                min_separation_m=min(p.separation for p in points)))+'\n')

    def close(self):
        self.subscription = None
        self.stream.close()


class ToolSensor:
    def __init__(self, world, arts, settings, servo_config, arm_indices):
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
        import yaml
        self.world, self.arts = world, arts
        self.enabled = settings['contact']['force_sensor']
        self.rows = {}
        self.metadata = {'sensor': 'attachment_site_joint',
                         'input_frame': 'flange_at_flange',
                         'sign': 'negative incoming reaction = tool load on flange',
                         'contact': settings['contact'], 'payload': {}}
        if not settings['contact']['enabled']:
            return
        if settings['contact']['tool_mass_source'] != 'servo_config':
            raise ValueError('payload mass/COM must come from servo_config')
        ft = yaml.safe_load(Path(servo_config).read_text())['force_torque']
        # Use the same standard gravity as the pipeline's measured mass fits.
        world.get_physics_context().set_gravity(-9.80665)
        stage = world.stage
        for side, art in arts.items():
            view = art._articulation_view
            self.rows[side] = view._metadata.joint_indices['attachment_site_joint'] + 1
            joint = next(p for p in stage.Traverse()
                         if p.GetName() == 'attachment_site_joint' and f'/{side}_arm/' in str(p.GetPath()))
            flange_path = UsdPhysics.Joint(joint).GetBody1Rel().GetTargets()[0]
            flange = stage.GetPrimAtPath(flange_path)
            flange_xf = UsdGeom.Xformable(flange).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            to_flange = flange_xf.GetInverse()
            bodies = [p for p in stage.Traverse() if str(p.GetPath()).startswith(str(flange_path))
                      and p.HasAPI(UsdPhysics.RigidBodyAPI)]
            ids = [view.get_body_index(p.GetName()) for p in bodies]
            tool = next(p for p in bodies if p.GetName() == 'tool')
            tool_id = view.get_body_index('tool')
            masses = np.asarray(view.get_body_masses()).copy()
            coms, orientations = view.get_body_coms()
            coms = np.asarray(coms).copy()
            inertias = np.asarray(view.get_body_inertias()).copy()
            desired_mass = float(ft[side]['tool_mass_kg'])
            desired_com = (np.asarray(ft[side]['sensor_offset_mm']) +
                           np.asarray(ft[side]['tool_com_mm'])) * 1e-3
            weighted_other = np.zeros(3)
            other_mass = 0.0
            for prim, index in zip(bodies, ids):
                if index == tool_id:
                    continue
                xf = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                com = np.asarray(to_flange.Transform(xf.Transform(Gf.Vec3d(*coms[0,index].astype(float)))))
                other_mass += float(masses[0,index])
                weighted_other += masses[0,index] * com
            tool_mass = desired_mass-other_mass
            if tool_mass <= 0 or not np.all(np.isfinite(desired_com)):
                raise ValueError('measured payload cannot contain the simulated fingers')
            tool_com_flange = (desired_mass*desired_com-weighted_other)/tool_mass
            tool_xf = UsdGeom.Xformable(tool).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            tool_com = tool_xf.GetInverse().Transform(flange_xf.Transform(Gf.Vec3d(*tool_com_flange)))
            old_mass = float(masses[0,tool_id])
            inertias[0,tool_id] *= tool_mass/old_mass
            masses[0,tool_id] = tool_mass
            coms[0,tool_id] = np.asarray(tool_com)
            view.set_body_masses(masses)
            view.set_body_coms(coms, orientations)
            view.set_body_inertias(inertias)
            limits = np.asarray(view.get_max_efforts()).copy()
            limits[0,arm_indices[side]] = settings['contact']['arm_max_effort_nm']
            view.set_max_efforts(limits)
            actual_mass = float(np.asarray(view.get_body_masses())[0,ids].sum())
            if abs(actual_mass-desired_mass) > 1e-6:
                raise RuntimeError('PhysX payload mass write/read mismatch')
            self.metadata['payload'][side] = {
                'mass_kg': actual_mass, 'com_flange_m': desired_com.tolist(),
                'tool_body_mass_before_kg': old_mass, 'tool_body_mass_kg': tool_mass,
                'tool_body_com_m': list(tool_com),
                'inertia_status': 'geometry inertia scaled with mass; not identified on hardware',
                'arm_max_effort_nm': np.asarray(view.get_max_efforts())[0,arm_indices[side]].tolist()}

    def measure(self, side, time_ns, seq):
        reaction = np.asarray(self.arts[side].get_measured_joint_forces())[self.rows[side]]
        if reaction.shape != (6,) or not np.all(np.isfinite(reaction)):
            raise RuntimeError(f'{side}: invalid PhysX tool joint wrench')
        return {'frame': 'flange_at_flange', 'wrench': (-reaction).tolist(),
                'seq': seq, 'time_ns': time_ns, 'valid': True}

    def save(self, path):
        Path(path).write_text(json.dumps(self.metadata, indent=2))
