"""Shared pick-pad geometry, placement contract, and PhysX contact material."""
import hashlib
import json
import math
from pathlib import Path
import numpy as np

ROOT = Path(__file__).resolve().parents[1]


def load(mode):
    if mode == 'bare':
        return {'schema': 'simulation.work_surface.v1', 'name': 'bare_table'}
    if mode != 'foam':
        raise ValueError(f'unknown work surface: {mode}')
    profile = json.loads((ROOT/'config/work_surface.json').read_text())
    if (profile['schema'] != 'simulation.work_surface.v1' or
            np.asarray(profile['size_m']).shape != (3,) or
            not np.all(np.isfinite(profile['size_m'])) or min(profile['size_m']) <= 0):
        raise ValueError('invalid work surface dimensions')
    texture = ROOT/profile['texture']
    profile['texture_sha256'] = hashlib.sha256(texture.read_bytes()).hexdigest()
    return profile


def top_z(profile, table_z):
    return table_z + (profile['size_m'][2] if 'size_m' in profile else 0.0)


def check_frozen(frozen, profile):
    expected = frozen.get('work_surface', load('bare'))
    if expected != profile:
        raise ValueError('Frozen scene work surface differs from requested surface. '
                         'Use --work-surface bare for historical table scenes, or regenerate '
                         'the foam scene with scripts/freeze_work_surface.py.')


def contains(profile, x, y, margin=0.03):
    if 'size_m' not in profile:
        return True
    return all(abs(v-c) <= size/2-margin for v,c,size in
               zip((x,y), profile['center_xy_m'], profile['size_m'][:2]))


def placed_bolts(profile, layout, n_per, seed, pile_x, pile_dy, spread=1.0):
    """Place nonintersecting planar bolt footprints on the pad before a short settle."""
    rng = np.random.default_rng(seed)
    colors = ['gray']*n_per+['black']*n_per
    if layout == 'random':
        rng.shuffle(colors)
    out, footprints = [], []
    # Conservative oriented rectangle around M12x25 head+shaft, with 1 mm clearance.
    local = np.array([[-.013,-.0102],[.026,-.0102],[.026,.0102],[-.013,.0102]])
    def overlap(a,b):
        for polygon in (a,b):
            for edge in np.diff(np.vstack([polygon,polygon[0]]),axis=0)[:2]:
                axis = np.array([-edge[1],edge[0]])
                pa,pb = a@axis,b@axis
                if pa.max() < pb.min() or pb.max() < pa.min():
                    return False
        return True
    for color in colors:
        for _ in range(10000):
            x = pile_x+rng.normal(0,(.045 if layout=='aligned' else .07)*spread)
            y = ((1 if color=='gray' else -1)*pile_dy+rng.normal(0,.05*spread)
                 if layout=='aligned' else rng.uniform(-.22,.22))
            yaw = rng.uniform(0,math.pi)
            if not contains(profile,x,y):
                continue
            c,s = math.cos(yaw),math.sin(yaw)
            footprint = local@np.array([[c,s],[-s,c]])+np.array([x,y])
            if any(overlap(footprint,other) for other in footprints):
                continue
            out.append((color,x,y,yaw)); footprints.append(footprint)
            break
        else:
            raise ValueError('Could not fit requested bolts on pad without overlap')
    return out


def build(stage, profile, table_z):
    if 'size_m' not in profile:
        return
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics, UsdShade, PhysxSchema
    size=np.asarray(profile['size_m']); center=np.r_[profile['center_xy_m'],table_z+size[2]/2]
    points=np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],
                     [-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]])*size/2+center
    mesh=UsdGeom.Mesh.Define(stage,'/World/scene/pick_pad')
    mesh.CreatePointsAttr([Gf.Vec3f(*p) for p in points])
    mesh.CreateFaceVertexCountsAttr([4]*6)
    mesh.CreateFaceVertexIndicesAttr([3,2,1,0,4,5,6,7,0,1,5,4,1,2,6,5,2,3,7,6,3,0,4,7])
    mesh.CreateSubdivisionSchemeAttr('none')
    repeat=float(profile['texture_repeats'])
    uv=UsdGeom.PrimvarsAPI(mesh).CreatePrimvar('st',Sdf.ValueTypeNames.TexCoord2fArray,'faceVarying')
    uv.Set([Gf.Vec2f(0,0),Gf.Vec2f(repeat,0),Gf.Vec2f(repeat,repeat),Gf.Vec2f(0,repeat)]*6)
    mat=UsdShade.Material.Define(stage,'/World/m/pick_pad')
    shader=UsdShade.Shader.Define(stage,'/World/m/pick_pad/surface'); shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('roughness',Sdf.ValueTypeNames.Float).Set(profile['roughness'])
    shader.CreateInput('metallic',Sdf.ValueTypeNames.Float).Set(profile['metallic'])
    tex=UsdShade.Shader.Define(stage,'/World/m/pick_pad/texture');tex.CreateIdAttr('UsdUVTexture')
    tex.CreateInput('file',Sdf.ValueTypeNames.Asset).Set(str(ROOT/profile['texture']))
    tex.CreateInput('sourceColorSpace',Sdf.ValueTypeNames.Token).Set('sRGB')
    # The photographed patch includes camera illumination/white balance. Apply a
    # material-only tint to match its rendered appearance to the real wrist RGB.
    scale = profile.get('texture_rgb_scale', [1.0, 1.0, 1.0])
    tex.CreateInput('scale',Sdf.ValueTypeNames.Float4).Set(Gf.Vec4f(*scale,1.0))
    for axis in ('S','T'):
        tex.CreateInput('wrap'+axis,Sdf.ValueTypeNames.Token).Set('repeat')
    reader=UsdShade.Shader.Define(stage,'/World/m/pick_pad/uv');reader.CreateIdAttr('UsdPrimvarReader_float2')
    reader.CreateInput('varname',Sdf.ValueTypeNames.Token).Set('st')
    tex.CreateInput('st',Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(),'result')
    shader.CreateInput('diffuseColor',Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(),'rgb')
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(),'surface')
    UsdShade.MaterialBindingAPI.Apply(mesh.GetPrim()).Bind(mat)
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim()).CreateApproximationAttr('convexHull')
    physics=UsdShade.Material.Define(stage,'/World/physmat_pick_pad')
    c=profile['contact']; api=UsdPhysics.MaterialAPI.Apply(physics.GetPrim())
    api.CreateStaticFrictionAttr(c['static_friction']);api.CreateDynamicFrictionAttr(c['dynamic_friction'])
    api.CreateRestitutionAttr(c['restitution'])
    api=PhysxSchema.PhysxMaterialAPI.Apply(physics.GetPrim())
    api.CreateCompliantContactStiffnessAttr(c['stiffness_n_m'])
    api.CreateCompliantContactDampingAttr(c['damping_ns_m'])
    api.CreateCompliantContactAccelerationSpringAttr(False)
    UsdShade.MaterialBindingAPI(mesh.GetPrim()).Bind(physics,materialPurpose='physics')


def verify_stage(stage, profile, table_z):
    if 'size_m' not in profile:
        return {'name':'bare_table'}
    from pxr import UsdGeom, UsdShade, UsdPhysics, PhysxSchema
    prim=stage.GetPrimAtPath('/World/scene/pick_pad')
    points=np.asarray(UsdGeom.Mesh(prim).GetPointsAttr().Get())
    size=np.asarray(profile['size_m']);center=np.r_[profile['center_xy_m'],table_z+size[2]/2]
    lo,hi=points.min(axis=0),points.max(axis=0)
    if not np.allclose([lo,hi],[center-size/2,center+size/2],atol=1e-7,rtol=0):
        raise RuntimeError('pick pad dimensions/height differ from configuration')
    material,_=UsdShade.MaterialBindingAPI(prim).ComputeBoundMaterial('physics')
    if not material or str(material.GetPath()) != '/World/physmat_pick_pad' or not prim.HasAPI(UsdPhysics.CollisionAPI):
        raise RuntimeError('pick pad collision/material binding missing')
    api=PhysxSchema.PhysxMaterialAPI(material.GetPrim())
    k=api.GetCompliantContactStiffnessAttr().Get();d=api.GetCompliantContactDampingAttr().Get()
    if (k,d)!=(profile['contact']['stiffness_n_m'],profile['contact']['damping_ns_m']):
        raise RuntimeError('pick pad compliant material differs from configuration')
    return dict(min_m=lo.tolist(),max_m=hi.tolist(),size_m=(hi-lo).tolist(),
                physics_material=str(material.GetPath()),stiffness_n_m=k,damping_ns_m=d)


def verify_bolts(profile, table_z, poses):
    """Check settled whole bolt footprints and support height, not just their origins."""
    from scipy.spatial.transform import Rotation
    issues=[]; heights=[]
    for i,pose in enumerate(poses):
        p=np.asarray(pose['p']);q=np.asarray(pose['q']);r=Rotation.from_quat(q[[1,2,3,0]])
        # Exact world AABB of the two cylinders oriented along the bolt's local X.
        axis=r.apply([1,0,0]); lows=[]; highs=[]
        for cx,half,radius in ((.0125,.0125,.006),(-.006,.006,.0092)):
            center=p+r.apply([cx,0,0]); ext=np.abs(axis)*half+radius*np.sqrt(np.maximum(0,1-axis**2))
            lows.append(center-ext);highs.append(center+ext)
        lo=np.min(lows,axis=0);hi=np.max(highs,axis=0)
        if 'size_m' in profile:
            c=np.array(profile['center_xy_m']); h=np.array(profile['size_m'][:2])/2
            if np.any(lo[:2]<c-h) or np.any(hi[:2]>c+h):issues.append(f'bolt {i}: outside pad')
        bottom=lo[2]-top_z(profile,table_z);heights.append(float(bottom))
        if not -.002 <= bottom <= .003:issues.append(f'bolt {i}: not supported at pick surface ({bottom:.5f} m)')
    if issues:
        raise ValueError('; '.join(issues))
    return {'count':len(poses),'all_on_surface':True,'min_bottom_offset_m':min(heights),
            'max_bottom_offset_m':max(heights)}
