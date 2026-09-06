"""플랜트 파지 유지력 최소 프로브.

정책·IK·Ruckig·오라클 플래너를 전부 우회한다. 팔을 고정 자세로 두고 두 손가락 사이
공중에 M12 볼트를 놓은 뒤 그리퍼를 닫고, 중력만으로 몇 mm 미끄러지는지 잰다.
재는 것은 하나 -- "완벽히 정렬해 물었을 때 sim 플랜트가 볼트를 유지하는가".
실기 기준은 '잡으면 사실상 100% 유지'.

레버: BOLT_FRICTION / FINGER_FRICTION / GRIP_MAXF / GRIP_KP (eval_closed_loop 과 동일 이름)
"""
import os, sys, json
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from isaacsim import SimulationApp
app = SimulationApp({"headless": True})

import eval_closed_loop as E
from isaacsim.core.api import World
from isaacsim.core.prims import RigidPrim, SingleArticulation
from isaacsim.core.utils.stage import add_reference_to_stage, get_current_stage
from isaacsim.core.utils.types import ArticulationAction
from pxr import Gf, UsdGeom, UsdPhysics, UsdShade, Sdf

N        = int(os.environ.get("PROBE_TRIALS", "20"))
HOLD_S   = float(os.environ.get("PROBE_HOLD_S", "2.0"))
JITTER   = float(os.environ.get("PROBE_JITTER_MM", "2.0")) / 1000.0
SEED     = int(os.environ.get("PROBE_SEED", "0"))
OUT      = os.environ.get("PROBE_OUT", "/tmp/grasp_probe.json")
SLIP_TH  = float(os.environ.get("PROBE_SLIP_MM", "10.0"))
# 닫힘 목표(%). 실기 명령 p50 은 좌 10.1 / 우 6.9, 측정 조는 12.1 / 7.7 에서 스톨한다.
CLOSE_PCT = float(os.environ.get("PROBE_CLOSE", "8"))
# 조가 닫히는 시간. 이 동안 볼트는 제자리에 고정된다(바닥 지지의 대용).
CLOSE_S = float(os.environ.get("PROBE_CLOSE_S", "0.5"))

world = World(stage_units_in_meters=1.0, physics_dt=E.PHYSICS_DT, rendering_dt=E.PHYSICS_DT)
stage = get_current_stage()
UsdGeom.Xform.Define(stage, "/World")
# 팔 USD 만 로드하면 PhysicsScene 이 없어 물리가 전혀 돌지 않는다(대조군에서 볼트가 떨어지지
# 않는 것으로 확인됨). 명시적으로 만들고 중력을 켠다.
_ps = UsdPhysics.Scene.Define(stage, "/World/physicsScene")
_ps.CreateGravityDirectionAttr().Set(Gf.Vec3f(0.0, 0.0, -1.0))
_ps.CreateGravityMagnitudeAttr().Set(9.81)
add_reference_to_stage(usd_path=str(E.ARM_USD), prim_path="/World/robot")
print("  PhysicsScene 생성, 중력 9.81 m/s^2 (-z)", flush=True)


# 물성 레버 적용 (eval_closed_loop 과 같은 환경변수 이름)
def _mat(path, mu):
    m = UsdShade.Material.Define(stage, path)
    a = UsdPhysics.MaterialAPI.Apply(m.GetPrim())
    a.CreateStaticFrictionAttr().Set(mu); a.CreateDynamicFrictionAttr().Set(mu)
    a.CreateRestitutionAttr().Set(0.0)
    return m.GetPrim()

bolt_mat = _mat("/World/physmat_bolt", float(os.environ.get("BOLT_FRICTION", "0.5")))
fmu = os.environ.get("FINGER_FRICTION")
if fmu:
    fm = _mat("/World/physmat_finger", float(fmu))
    nb = 0
    for p in stage.Traverse():
        n = p.GetName()
        if p.HasAPI(UsdPhysics.CollisionAPI) and ("finger" in str(p.GetPath()).lower()):
            UsdShade.MaterialBindingAPI.Apply(p)
            UsdShade.MaterialBindingAPI(p).Bind(UsdShade.Material(fm),
                bindingStrength=UsdShade.Tokens.weakerThanDescendants, materialPurpose="physics")
            nb += 1
    print(f"  finger friction {fmu} -> {nb} collider", flush=True)

# 볼트 하나 (eval_closed_loop 과 동일 치수/질량)
BP = "/World/probe_bolt"
xf = UsdGeom.Xform.Define(stage, BP)
prim = xf.GetPrim()
UsdPhysics.RigidBodyAPI.Apply(prim)
UsdPhysics.MassAPI.Apply(prim).CreateMassAttr(0.022)
for tag, rr, hl, cx in (("shaft", E.SHAFT_R, E.SHAFT_L/2, E.SHAFT_L/2),
                        ("head", E.HEAD_R, E.HEAD_L/2, -E.HEAD_L/2)):
    cy = UsdGeom.Cylinder.Define(stage, f"{BP}/{tag}")
    cy.CreateRadiusAttr(rr); cy.CreateHeightAttr(hl*2); cy.CreateAxisAttr("X")
    UsdGeom.Xformable(cy).AddTranslateOp().Set(Gf.Vec3d(cx, 0, 0))
    UsdPhysics.CollisionAPI.Apply(cy.GetPrim())
    UsdShade.MaterialBindingAPI.Apply(cy.GetPrim())
    UsdShade.MaterialBindingAPI(cy.GetPrim()).Bind(UsdShade.Material(bolt_mat),
        bindingStrength=UsdShade.Tokens.weakerThanDescendants, materialPurpose="physics")
# 리그와 동일: 모든 프림을 만든 뒤 reset 을 단 한 번만 호출한다. 중간에 다시 reset 하면
# 나중에 추가된 rigid body 가 물리 등록에서 빠져 '떨어지지 않는 볼트'가 된다(대조군에서 확인).
world.reset()
bolt = RigidPrim(prim_paths_expr=BP, name="probe_bolt"); bolt.initialize()

# articulation 은 씬이 최종 확정된 뒤에 바인딩한다 (볼트 추가 후 reset 하면 핸들이 갈린다)
art = SingleArticulation(prim_path="/World/robot", name="arm"); art.initialize()
names = list(art.dof_names)
fl, fr = names.index("finger_left_joint"), names.index("finger_right_joint")
q_probe = np.asarray(art.get_joint_positions(), dtype=np.float32).reshape(-1)
print(f"  DOF {len(names)} | finger idx {fl},{fr} | q 길이 {q_probe.size}", flush=True)
if q_probe.size != len(names):
    raise SystemExit(f"ABORT: dof_names {len(names)} vs joint_positions {q_probe.size} 불일치")

# 그리퍼 구동 게인/힘 (eval_closed_loop 과 동일 정책)
# 손가락은 articulation 게인이 아니라 USD DriveAPI 로 구동된다. 진단 결과 드라이브
# stiffness 가 없어 명령을 무시했으므로(개방 100% 명령에도 관절이 안 움직임) 명시적으로 세운다.
_DS = float(os.environ.get("GRIP_DRIVE_STIFF", "1e5"))
_DD = float(os.environ.get("GRIP_DRIVE_DAMP", "1e3"))
_nd = 0
for prim in stage.Traverse():
    if prim.GetName() in ("finger_left_joint", "finger_right_joint"):
        dr = None
        for tok in ("linear", "transX", "transY", "transZ"):
            dr = UsdPhysics.DriveAPI.Get(prim, tok)
            if dr: break
        if not dr:
            dr = UsdPhysics.DriveAPI.Apply(prim, "linear")
        dr.CreateTypeAttr().Set("force")
        dr.CreateStiffnessAttr().Set(_DS)
        dr.CreateDampingAttr().Set(_DD)
        if os.environ.get("GRIP_MAXF"):
            dr.CreateMaxForceAttr().Set(float(os.environ["GRIP_MAXF"]))
        _nd += 1
print(f"  finger drive stiffness={_DS:g} damping={_DD:g} maxForce={os.environ.get('GRIP_MAXF','기본')} on {_nd} joint(s)", flush=True)
if _nd == 0:
    raise SystemExit("ABORT: finger 관절을 찾지 못함")

gkp, gmf = os.environ.get("GRIP_KP"), os.environ.get("GRIP_MAXF")
_kp = float(os.environ.get("TREMOR_KP", 1.0e7)); _kd = float(os.environ.get("TREMOR_KD", 1.0e5))
kps = np.full(len(names), _kp, dtype=np.float32); kds = np.full(len(names), _kd, dtype=np.float32)
if gkp:
    kps[[fl, fr]] = float(gkp); kds[[fl, fr]] = float(os.environ.get("GRIP_KD", float(gkp)*1e-2))
try:
    art._articulation_view.set_gains(kps=kps.reshape(1, -1), kds=kds.reshape(1, -1))
except Exception:
    try: art.get_articulation_controller().set_gains(kps=kps, kds=kds)
    except Exception as e: print("  게인 설정 생략:", e, flush=True)
# GRIP_MAXF 는 위의 DriveAPI maxForce 로 이미 적용됨 (SingleArticulation 에는 set_max_efforts 없음)

def set_grip(pct):
    q = np.asarray(art.get_joint_positions(), dtype=np.float32).reshape(-1)
    fp = (1.0 - pct/100.0) * E.FINGER_TRAVEL_M
    q[fl], q[fr] = +fp, -fp
    art.apply_action(ArticulationAction(joint_positions=q))

def step(n):
    for _ in range(n): world.step(render=False)

# 손가락 중점 = 파지 중심. 손가락은 articulation 링크라 RigidPrim 으로 잡히지 않으므로
# USD 월드 변환에서 직접 읽는다.
from pxr import Usd
_fp = {}
for nm in ("finger_left", "finger_right"):
    _fp[nm] = next(x for x in stage.Traverse()
                   if x.GetName() == nm and x.HasAPI(UsdPhysics.RigidBodyAPI))

def _world_xyz(prim):
    m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
    t = m.ExtractTranslation()
    return np.array([t[0], t[1], t[2]], dtype=np.float64)

set_grip(100.0); step(400)
# 파지 중심은 링크 원점이 아니라 TPU 파지면(finger_tpu_sdf)의 중앙이다.
# 링크 원점(z=1.0969)에 놓으면 볼트가 PLA 하우징 위에 얹혀 대조군이 무의미해진다.
from pxr import UsdGeom as _UG
_bc = _UG.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy", "guide"])
_tpu = [x for x in stage.Traverse()
        if x.GetName() == "finger_tpu_sdf" and x.HasAPI(UsdPhysics.CollisionAPI)]
if len(_tpu) >= 2:
    _rs = [_bc.ComputeWorldBound(t).ComputeAlignedRange() for t in _tpu]
    _mid = np.mean([[(r.GetMin()[i] + r.GetMax()[i]) / 2.0 for i in range(3)] for r in _rs], axis=0)
    c = np.array(_mid, dtype=np.float64)
    print(f"  TPU 파지면 {len(_tpu)}개 중앙 사용", flush=True)
else:
    c = (_world_xyz(_fp["finger_left"]) + _world_xyz(_fp["finger_right"])) / 2.0
    print(f"  [warn] TPU 콜라이더 {len(_tpu)}개 -- 링크 원점 사용", flush=True)
FULL_OPEN_M = float(os.environ.get("FULL_OPEN_M", "0.098"))
def jaw_gap_mm():
    q = np.asarray(art.get_joint_positions(), dtype=np.float32).reshape(-1)
    return (FULL_OPEN_M - (q[fl] - q[fr])) * 1000.0
gap0 = jaw_gap_mm()
print(f"  [진단] 개방(grip 100%) 후 조 간격 {gap0:.1f}mm (완전개방 {FULL_OPEN_M*1000:.0f}mm 기대)", flush=True)
print(f"  파지 중심 = ({c[0]:.4f}, {c[1]:.4f}, {c[2]:.4f})  개방 간격 {gap0:.1f}mm", flush=True)
if gap0 < 0.8 * FULL_OPEN_M * 1000.0:
    raise SystemExit(f"ABORT: 개방 조 간격 {gap0:.1f}mm < 기대치 -- 손가락이 명령을 따르지 않음")

# 자유낙하 검사: 볼트가 실제로 물리 시뮬레이션되는지 확인한 뒤에만 시행에 들어간다.
_fp_test = c + np.array([0.30, 0.30, 0.30])   # 그리퍼에서 충분히 이격
bolt.set_world_poses(positions=np.array([_fp_test], dtype=np.float32),
                     orientations=np.array([[1.0, 0, 0, 0]], dtype=np.float32))
bolt.set_velocities(np.zeros((1, 6), dtype=np.float32))
_traj = []
for _i in range(6):
    step(int(0.05 / E.PHYSICS_DT))
    _z = float(np.asarray(bolt.get_world_poses()[0])[0][2])
    _v = float(np.asarray(bolt.get_velocities())[0][2])
    _traj.append((0.05*(_i+1), (_fp_test[2]-_z)*1000.0, _v))
print("  [자유낙하 검사] t(s) 낙하(mm) vz(m/s)  | 이론 0.3s -> 44mm, vz -2.94", flush=True)
for t, d, v in _traj:
    print(f"    {t:.2f}  {d:7.2f}  {v:+.3f}", flush=True)
_drop = _traj[-1][1]
if _drop < 10.0:
    raise SystemExit(f"ABORT: 볼트가 자유낙하하지 않음({_drop:.2f}mm) -- 물리 미등록")

rng = np.random.default_rng(SEED)
res = []
for k in range(N):
    off = rng.uniform(-JITTER, JITTER, 3)
    yaw = rng.uniform(-np.pi, np.pi)
    pos = c + off
    set_grip(100.0)
    bolt.set_world_poses(positions=np.array([pos], dtype=np.float32),
                         orientations=np.array([[np.cos(yaw/2), 0, 0, np.sin(yaw/2)]], dtype=np.float32))
    bolt.set_velocities(np.zeros((1, 6), dtype=np.float32))
    step(4)
    z0 = float(np.asarray(bolt.get_world_poses()[0])[0][2])
    if os.environ.get("PROBE_NOCLOSE") != "1":
        set_grip(CLOSE_PCT)            # 닫기 (실기 명령 p50 ~7~10%)
    # 조가 닫히는 동안 볼트를 제자리에 유지한다(실기에서는 볼트가 바닥에 놓여 있다).
    # kinematic 토글은 이 Isaac 버전에 API 가 없어, 매 스텝 자세/속도를 되돌려 고정한다.
    _P0 = np.array([pos], dtype=np.float32)
    _Q0 = np.array([[np.cos(yaw/2), 0, 0, np.sin(yaw/2)]], dtype=np.float32)
    for _ in range(int(CLOSE_S / E.PHYSICS_DT)):
        bolt.set_world_poses(positions=_P0, orientations=_Q0)
        bolt.set_velocities(np.zeros((1, 6), dtype=np.float32))
        world.step(render=False)
    z0 = float(np.asarray(bolt.get_world_poses()[0])[0][2])
    z1 = float(np.asarray(bolt.get_world_poses()[0])[0][2])
    step(int(HOLD_S / E.PHYSICS_DT))   # 중력 유지
    z2 = float(np.asarray(bolt.get_world_poses()[0])[0][2])
    gap = jaw_gap_mm()
    slip = (z0 - z2) * 1000.0
    held = slip < SLIP_TH
    res.append({"trial": k, "slip_mm": round(slip, 2), "held": bool(held),
                "yaw_deg": round(float(np.degrees(yaw)), 1), "finger_gap_mm": round(gap, 2),
                "z0": round(z0, 5), "z_after_close": round(z1, 5), "z_end": round(z2, 5)})
    print(f"  {k:2d}: slip {slip:7.2f} mm  gap {gap:5.1f}mm  z {z0:.4f}->{z2:.4f}  {'HOLD' if held else 'DROP'}", flush=True)

held = sum(r["held"] for r in res)
slips = np.array([r["slip_mm"] for r in res])
summary = {"trials": N, "held": held, "retention_pct": round(100.0*held/max(N,1), 1),
           "slip_mm_p50": round(float(np.median(slips)), 2),
           "slip_mm_p90": round(float(np.percentile(slips, 90)), 2),
           "params": {k: os.environ.get(k) for k in
                      ("BOLT_FRICTION", "FINGER_FRICTION", "GRIP_MAXF", "GRIP_KP")},
           "trials_detail": res}
json.dump(summary, open(OUT, "w"), indent=2, default=lambda o: o.item() if hasattr(o, "item") else str(o))
print(f"\n  유지율 {summary['retention_pct']}%  ({held}/{N})  slip p50 {summary['slip_mm_p50']}mm", flush=True)
app.close()
