# simulation — Isaac Sim 기반 M12 볼트 pick-place 평가 리그

> 이 문서는 다음 세션의 에이전트(Opus 5)가 프로젝트 맥락을 파악하기 위한 핸드오프 문서다.
> 작성: 2026-08-17, MuJoCo 예상 몽타주 작업 세션에서.

## 0. 한 줄 요약

dual RB3-730E + Pika 리그의 **양팔 M12 볼트 pick-place task를 Isaac Sim에서 폐쇄루프로
평가하는 리그를 구축**한다. **1순위 목표는 task 성능 평가(특히 무작위 배치 조건)이며,
시뮬레이션 학습 데이터 생성은 의도적으로 후순위다** (평가와 학습이 같은 sim 분포를 공유하면
개선이 모델 발전인지 sim 과적합인지 구분 불가 — Genesis World 1.0 블로그의 eval-first 논지 채택).

## 1. 배경과 현재 상태

- **Task**: 좌완=회색 볼트 → (탑뷰) 우측 회색 박스, 우완=검은 볼트 → (탑뷰) 좌측 녹색 박스.
  볼트 규격 **M12 × 25mm 소켓헤드**. 정의: `~/workspace/robotics_lab/policy_runner/GOAL.md`
- **현재 성능**: 로컬 **:8001에 서빙 중인 pi0.5(openpi)** 가 **정렬 배치**(좌=회색, 우=검정 더미)에서
  **80~90% 성공률** (2026-08-17 기준, 운영자 판정).
- **이번 주 작업**: 두 색을 **무작위 혼합 배치**하고 성능 측정 → 개선. Isaac Sim 검증 리그를 병렬 구축.
- **중요한 사전 지식** (llm-wiki, 2026-07-11 판정): 수집 데이터 752 에피소드 전수 감사 결과
  시연이 "nearest-bolt-consistent" — geometry 조건부로 action ⊥ color. 즉 **기존 데이터로는
  색상 조건화가 학습 불가능(unlearnable)**. 따라서 무작위 배치 평가에서는 각 trial을
  "가까운 볼트 = 정답 색" **일치/불일치로 분류해 성공률을 분리 집계**할 것. 불일치 조건의
  성공률이 색상 이해도의 직접 측정치다. 무너지면 개선은 모델 튜닝이 아니라 새 데이터 수집이 정공법.
- 실기 평가는 현재 수동(operator_success 라벨)이고 자동 성공 판정기·물리 sim은 robotics_lab에 없다.
  Isaac 리그가 이 공백을 메우는 것이 존재 이유다.

## 2. 구축 사다리 (단계별 게이트, cm_bridge ladder 방식)

1. ~~**설치 + 스모크**~~ — **완료 (2026-08-17)**. 상세는 §9. Isaac Sim **6.0.1.0** / Python 3.12 /
   `.venv-isaac` (24GB). 스모크 통과: 헤드리스 부팅 + 물리 + 오프스크린 RGB 캡처.
2. ~~**로봇 임포트 + FK 검증**~~ — **완료 (2026-08-17)**. 상세는 §10.
   FK 교차검증 최대 오차 **0.00038 mm / 0.051°** (게이트 1mm/0.1° PASS).
   `assets/cell_dual_rb3_730e.usd`에 정본 좌표계 양팔+스탠드 조립 완료.
3. ~~**씬 구성**~~ — **완료 (2026-08-17)**. 상세는 §12. `scripts/build_scene.py`,
   정렬/무작위 두 레이아웃 + 도달성·안착 게이트 PASS.
4. **카메라 정합**: **선행 조건 = 손목캠 hand-eye 측정** (아래 §4 참고, 현재 unmeasured).
   이후 녹화된 실기 에피소드를 sim에서 kinematic replay → 실기 프레임과 나란히 비교하며
   재질/조명 수렴 (real-to-sim 권위 사다리: replay → paired → rank → substitution).
5. ~~**폐쇄 루프**~~ — **동작 확인 (2026-08-17)**. 상세는 §18.
   `scripts/eval_closed_loop.py`. 4 에피소드 예비 실행에서 정답 배치 5개 / 오배치 0개.
6. **정렬 배치 sim 성공률 vs 실기 80~90% 상관 확인** → 통과해야 무작위 배치 sim 결과를 신뢰.
   이후 무작위 배치 대량 평가.

## 3. 자산 경로 (robotics_lab 기준)

- 팔 MJCF: `rb_servo_server/descriptions/mjcf/rb3_730e/rb3_730e.xml`
- 양팔 조립 MJCF: `rb_servo_server/descriptions/mjcf/dual_rb3_730e_ver3.xml`
- Pika 포함 URDF(뷰어용): `rb_servo_server/descriptions/urdf/rb3_730e_pika_articulated.urdf`
  — **kinematics 정본은 rb3_730e.urdf** (pika_articulated는 GUI 전용, DOF 추가됨)
- 메시: `rb_servo_server/descriptions/meshes/` (visual .obj/.dae + collision coacd hull .stl,
  Pika 툴은 `robots/rb3_730e/visual/tool/*.STL`, **mm 단위 → scale 0.001**)
- 스탠드: `meshes/stands/dual_rb3_730e/dual_rb3_730e_stand_ver3.stl` (mm)
- 캘리브레이션: `calibration/active_calibration.yaml` (mount 변환 measured, 카메라 unmeasured)
- 실기 프레임: `outputs/bolt_episode_mp4/*.mp4`, `outputs/episode_videos/*_realsense_LR.mp4`

## 4. MuJoCo 몽타주 작업에서 확인한 함정 (반드시 읽을 것)

- **스탠드 90° 회전 합성**: 원본 dual MJCF는 `body stand euler z=+1.5708` 안에
  시각 메시 geom(euler z=−1.5708)과 collision 서브바디(`stand_base_col`, euler z=−1.5708)가
  들어 있고, **팔 마운트 바디는 회전 없는 body 프레임**에 있다. 즉 메시/콜리전과 마운트가
  서로 다른 서브프레임. 이걸 그대로 재조립하지 않으면 스탠드가 팔 대비 90° 돌아간다.
  몽타주(`montage/render_montage.py`)는 마운트를 world `(±0.1601, −0.1725, 0.5825)`에 직접 두고
  메시만 −90° 돌리는 방식으로 조립했다. 내부적으로는 자기일관적(스탠드-팔 상대 배치는 맞음)이지만
  **전체 조립이 정본 대비 z축 −90° 돌아가 있다. 몽타주 좌표를 Isaac에 그대로 쓰지 말 것.**
- **⚠️ 정본 stand 프레임 (2026-08-17 step 2에서 확정, 위 항목의 정정)**:
  `active_calibration.yaml`의 `T_stand_{left,right}_base`는 **`stand` 프레임 기준 상대값**이고
  (`parent: stand`로 명시돼 있음), 그 `stand` 프레임 자체가 `base`/world 대비 **z +1.57078 회전**돼
  있다. 따라서 팔 마운트의 **월드 좌표는 `(0.1725, ±0.1601, 0.5825)`** 이지
  `(±0.1601, −0.1725, ...)`가 아니다 (좌완 = +y, 우완 = −y).
  검증 출처: `mo_robot_descriptions/.../stands/urdf/dual_rb3_730e_stand/dual_rb3_730e_stand_ver3.urdf`
  의 부모 체인 `stand_left_arm_base <- stand <- base <- world`를 전개해 계산.
  **정책의 14-dim state가 "절대 stand-frame" 기준이므로, Isaac 월드 프레임이 정본과 어긋나면
  openpi에 잘못된 state를 먹이게 된다.** 조립은 반드시 스탠드 URDF 계층을 따를 것 —
  손으로 변환을 재작성하지 말고 `stand_{left,right}_arm_base` 프레임에 팔을 붙인다.
- **dual MJCF 로드 실패 요인**: `descriptions/camera/realsense_d435f.obj`를 참조하는데
  이 파일이 저장소에 없다. 그대로 로드하면 asset 에러 → 카메라 마운트 지오메트리는 빼거나 대체.
- **meshdir 상대경로**: `rb3_730e.xml`의 `meshdir="../../meshes"`는 파일 위치 기준.
  파일을 복사/이동하면 meshdir을 조정해야 한다 (montage에서는 `../meshes`로 수정 + 심링크).
- **Pika 툴 부착 변환** (pika_articulated.urdf에서 추출):
  `attachment_site = link6 + z 0.100`, `TCP = attachment_site + z 0.247642` (fingertip 평면),
  FT센서(RFT64)는 attachment_site z 0.015~0.045, +90° yaw가 CAD에 베이크됨.
  핑거 메시는 열린 상태가 베이크된 시각 전용 (좌/우 별도 STL).
- **Reset pose** (`rb_gui/rb_servo_gui/app.py:216-217`, InitMotion 기본값, deg,
  순서 base/shoulder/elbow/wrist1/wrist2/wrist3):
  - LEFT: `(259.0, 75.6, 129.5, −55.6, −131.2, −161.7)`
  - RIGHT: `(−253.7, −76.9, −127.6, 65.7, 143.7, 166.9)`
- **손목캠 hand-eye 미측정**: `active_calibration.yaml`이
  `geometry_valid_for_real_policy: false`, 좌/우 손목캠 `hand_eye_status: unmeasured`.
  몽타주의 카메라 pose는 실기 영상 보고 눈대중으로 맞춘 것이다
  (현재값: link6 pika_tool 프레임에서 `pos 0 −0.058 0.155, euler −3.02 0 0, fovy 108`).
  **wrist3 값에 따라 카메라 롤이 180° 뒤집혀 보일 수 있음을 실험으로 확인** —
  Isaac에서 real-to-sim 상관을 보려면 hand-eye 실측이 선행돼야 한다.
- **어안**: 정책 입력은 fisheye wrist RGB + **0.65 center-crop** (640×480 → 416×312,
  `policy_runner/config/flow_real_fisheye.yaml`). MuJoCo 프리뷰는 핀홀 근사만 했다.
- **씬 물체 스펙** (실측 아님, 영상 기반 추정 — 실측값 들어오면 교체):
  - 테이블: 회색 판금, 상면 z=0 (스탠드 베이스와 동일 평면)
  - 볼트: M12×25 SHCS (축 φ12×25, 머리 φ18×12). 은색(=회색)과 흑색.
  - 박스: 좌(탑뷰)=녹색, 우=회색. **치수는 §16의 실측값으로 대체됨**(추정 31×23×16cm는 폐기).
  - 볼트 더미: 정렬 시 회색 중심 (+0.16, −0.47) / 검정 (−0.16, −0.47), 무작위는 x∈[−0.3,0.3] 혼합

## 5. openpi 폐쇄루프 연결 계약

- 서버: 외부 openpi `serve_policy.py` websocket (로컬 **:8001**이 현재 pi0.5 프로덕션).
  robotics_lab 쪽 스킴은 `openpi://host:port` (`policy_runner/policy_runner/openpi_remote.py`).
- obs: `{left,right}_wrist_0_rgb` + **14-dim state** (팔당 pos3 + rotvec3 + grip/100, 절대 stand-frame)
  + 고정 prompt. 액션은 ee_local 델타 청크(30Hz).
- Isaac 리그는 flow-infer를 통째로 이식할 필요 없이, 같은 obs 계약으로 서버에 직접 접속해
  액션을 sim 로봇에 적용하는 standalone 스크립트면 충분하다.

## 6. 현재 작업물 (montage/)

- `expected_sim_montage.png` — **목표 구도 스펙** (사용자 승인됨: reset pose, 스탠드 방향,
  M12×25, 박스 색/위치/5cm 인서트, 색→박스 매핑 반영)
- `render_montage.py` — MuJoCo 씬 생성+렌더 (배치 로직/좌표/재질이 코드로 명시됨)
- `rb3_730e_pika/rb3_730e_pika.xml` — 팔 MJCF 사본 + Pika 툴 메시 + 손목캠 추가본
- `compose.py` — 몽타주 합성. 재생성:
  `MUJOCO_GL=egl ../.venv/bin/python render_montage.py && ../.venv/bin/python compose.py`
- `lr_ep_f1.png` 등 — 실기 손목캠 참조 프레임

venv 2개를 쓴다 (섞지 말 것):
- `.venv` — Python 3.11 + mujoco 3.11. 몽타주 프리뷰 전용.
- `.venv-isaac` — Python 3.12 + Isaac Sim 6.0.1.0. Isaac 작업 전용.

## 7. 환경

- 이 PC: RTX 5090 32GB (compute cap 12.0 = sm_120), 드라이버 595.84 (CUDA 13.2),
  Ubuntu 22.04 / glibc 2.35, 디스크 여유 ~950GB. `uv` 사용 가능.
  NVIDIA Vulkan ICD + `libnvidia-glcore.so.595.84` 확인됨 (Isaac의 Vulkan 렌더러 요건).
- **GPU를 프로덕션 추론 서버와 공유한다**: :8001 pi0.5 openpi 서버가 상주하며 **약 8.7GB 점유**
  (2026-08-17 확인, PID는 매번 다름). 남는 여유 ~24GB로 Isaac 동시 구동은 가능하지만,
  **실기 rollout 중 무거운 렌더링을 돌리면 프로덕션 추론에 영향**을 줄 수 있다. 실기 세션과
  겹치지 않게 스케줄링하거나, 대량 평가는 GPU 서버(8.8 / 8.13: 8× RTX PRO 6000)로 이전할 것.
- 실기와 같은 PC이므로 :8001 서버·캘리브레이션 재사용 가능. 시작은 로컬 권장.

## 8. 평가 설계 원칙 (1순위 목표의 구체화)

- 단일 성공률 대신 **섭동 축별 robustness profile**: 1차 축 = 배치(정렬 vs 무작위),
  무작위 내에서 nearest-bolt 일치/불일치 분리 집계. 이후 조명/배경/시점 축 확장.
- 오픈루프 지표(offline L2 등)는 모델 판별력이 없다는 것이 실기(grip-echo 진단)와
  Genesis 실측 양쪽에서 확인됨 — **폐쇄루프 성공률이 판정 기준**.
- sim 평가가 권위를 얻는 조건: §2-6단계의 실기-시뮬 상관 확인을 통과해야 한다.
  통과 전의 sim 수치는 참고용으로만 취급할 것.

## 9. 1단계 완료 기록 (2026-08-17)

**결과: PASS.** 헤드리스 부팅 + 물리 스텝 + 오프스크린 RGB 캡처 3종 모두 검증됨.
- 큐브가 z=1.0에서 낙하해 z=0.1000(스케일 0.2의 절반)에 정확히 안착 → 물리 정상
- RGB 애노테이터가 (720, 1280, 4) uint8 실제 프레임 반환 → 렌더 정상
- 산출물: `outputs/smoke_test.png`, 스크립트: `scripts/smoke_test.py`
- 실행: `OMNI_KIT_ACCEPT_EULA=YES .venv-isaac/bin/python scripts/smoke_test.py`

### 설치 절차 (재현용)
```bash
cd ~/workspace/simulation
uv venv --python 3.12 .venv-isaac
uv pip install --python .venv-isaac/bin/python --prerelease=allow "isaacsim[all,extscache]==6.0.1.0"
```

### 설치에서 걸린 함정 (다음 사람이 반복하지 말 것)
- **버전/Python이 문서 초안과 다르다**: 이 문서 초안은 "Isaac Sim 5.x + Python 3.11"이었으나,
  실제 최신은 **6.0.1.0이고 Python 3.12를 요구**한다 (`requires_python: ==3.12.*`).
  5.0/5.1은 Python 3.11, 6.x는 3.12. 6.x 휠이 `manylinux_2_35_x86_64`라 Ubuntu 22.04의
  glibc 2.35와 정확히 일치해서 6.0.1.0을 선택했다.
- **`--prerelease=allow`가 필수**: `isaacsim[all]`이 `tinyobjloader==2.0.0rc13`(프리릴리스)을
  끌어오기 때문에 플래그 없이는 resolver가 "requirements are unsatisfiable"로 실패한다.
  첫 시도가 이걸로 죽었다.
- **EULA**: 첫 실행에 `OMNI_KIT_ACCEPT_EULA=YES` 필요.
- **RGB 애노테이터가 빈 프레임을 반환**: `world.step(render=True)`만 60회 돌려도
  `annotator.get_data()`가 빈 배열이다. **`rep.orchestrator.step()`을 호출해야** 렌더 프로덕트에
  실제 프레임이 채워진다 (실측: orchestrator step 1회로 채워짐). 이걸 모르면 "렌더가 깨졌다"고
  오진하기 쉽다 — 첫 스모크 실행이 정확히 이 이유로 FAIL 했다.
- 첫 부팅은 셰이더 컴파일로 ~23초, 이후 캐시되어 빨라진다. Warp 커널 캐시는 `~/.cache/warp/`.
- 설치 용량: `.venv-isaac` **24GB**, isaacsim 관련 패키지 25개.

### 다음 단계(2단계) 준비 상태
- URDF 메시 참조 무결성 검증 완료: `rb3_730e.urdf` 56개 / `rb3_730e_pika_articulated.urdf` 58개
  **전부 해석됨(누락 0)**. scale은 `0.001`(mm 메시)과 `1.0`이 섞여 있으니 임포트 시 확인할 것.
- **kinematics 정본은 `rb3_730e.urdf`** (pika_articulated는 GUI 전용, 핑거 prismatic으로 DOF가 다름).
  FK 교차검증의 기준은 정본 쪽이어야 한다.

## 10. 2단계 완료 기록 (2026-08-17)

**결과: PASS.** URDF→USD 임포트 + FK 교차검증 + 정본 좌표계 양팔 조립까지 완료.

### FK 교차검증 결과
레퍼런스로 **Pinocchio 대신 MuJoCo(MJCF)** 를 썼다. Python pinocchio가 없고(rb_servo_server는 C++),
무엇보다 **MJCF와 URDF는 같은 CAD에서 나온 별개 파일**이라 둘의 일치가 "MJCF↔URDF 정합성 +
URDF→USD 임포트 충실도"를 한 번에 검증해주기 때문 — 같은 URDF를 두 번 읽는 것보다 강한 검사다.

27개 자세(zero + 좌/우 reset + 랜덤 24, elbow는 실제 물리한계 ±150°로 샘플링) × 7개 프레임 비교:

| | 최대 위치오차 | 최대 자세오차 |
|---|---|---|
| 전체 | **0.00038 mm** | **0.051°** |

서브마이크론 수준으로, float32 정밀도 한계에 해당한다. 게이트(1mm / 0.1°) 통과.
- 스크립트: `scripts/fk_dump_mujoco.py` → `scripts/fk_dump_isaac.py` → `scripts/fk_compare.py`
- 산출물: `outputs/fk_mujoco.json`, `outputs/fk_isaac.json`
- 재현: `.venv/bin/python scripts/fk_dump_mujoco.py` →
  `OMNI_KIT_ACCEPT_EULA=YES .venv-isaac/bin/python scripts/fk_dump_isaac.py` →
  `.venv/bin/python scripts/fk_compare.py`

### 조립 결과 (정본 좌표계)
- 마운트 월드 좌표 **(0.1725, ±0.1601, 0.5825)** — 좌완 +y, 우완 −y. §4의 정정 항목 참조.
- **작업영역은 +x 방향**이다. reset pose에서 TCP는
  좌 `(0.3592, 0.0987, 0.2100)`, 우 `(0.3328, −0.1852, 0.2240)`, TCP 간격 0.2854 m.
  → 3단계에서 테이블/볼트/박스는 **+x 쪽**에 배치해야 한다 (몽타주의 −y가 아님).
- 산출물: `assets/cell_dual_rb3_730e.usd`, `outputs/cell_reset_pose.json`,
  `outputs/cell_overview.png` (몽타주와 눈으로 대조 가능한 뷰)
- 스크립트: `scripts/build_cell.py` — 마운트 변환을 손으로 쓰지 않고
  **스탠드 USD의 `stand_{left,right}_arm_base` 프레임에서 직접 읽어** 팔을 붙인다. 이 방식을 유지할 것.

### Isaac Sim 6.0 API 함정 (5.x 예제와 다름)
- **URDF 임포트 API가 바뀌었다**: 구버전의 `omni.kit.commands.execute("URDFParseAndImportFile", ...)`가
  아니라 `from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig` →
  `URDFImporter(config).import_urdf(config)`. 인터넷의 5.x 예제 코드는 그대로 안 돌아간다.
- **`merge_fixed_joints=False` 필수**: True면 `attachment_site` / `tcp` / `tool` / `ft_sensor_*`
  프레임이 사라져 FK 검증도 카메라 부착도 불가능해진다. 현재 이 프레임들은 전부 USD에 살아 있고
  PhysX 링크로도 존재함(`body_names`에 나옴).
- **링크 프림 경로는 중첩 + 이름 충돌**: `/World/robot/Geometry/world/link0/link1/.../link6/
  attachment_site/tcp` 처럼 깊게 중첩되고, 각 링크 밑에 **같은 이름의 시각 메시 프림**이 또 있다
  (`link6/link6`). 이름으로 경로를 조립하면 틀린 프림을 잡는다.
  → **`prim.HasAPI(UsdPhysics.RigidBodyAPI)`로 필터링해서 경로를 해석**할 것 (`fk_dump_isaac.py` 참고).
- **`Articulation`에는 `get_link_poses()`가 없다** (`get_world_poses()`는 루트 pose만).
  per-link 월드 pose는 각 링크에 `RigidPrim` 뷰를 만들어 `get_world_poses()`로 읽는다.
- FK만 볼 때는 `set_gravity(0.0)` + `fix_base=True`로 두고 `set_joint_positions()` 후
  `world.step(render=False)` — 중력으로 인한 오차 없이 순수 기구학이 된다.

### 프레임 이름 함정 (반복 주의)
**MJCF의 site `tcp`와 URDF의 링크 `tcp`는 서로 다른 지점이다.**
- MJCF `site tcp` = link6 + z 0.100 = **URDF의 `attachment_site`**
- URDF `tcp` = attachment_site + z 0.247642 (Pika 손끝 평면) = link6 + z 0.347642

FK 비교는 `attachment_site`를 기준으로 했다. 두 모델에서 이름이 같은 `tcp`를 그냥 비교하면
247.6mm 오차가 나면서 "임포트가 깨졌다"고 오진하게 된다.
참고: 정본 `rb3_730e.urdf`에도 Pika TCP 오프셋과 `tool` 시각 메시가 이미 포함돼 있다
(그래서 `cell_overview.png`에 그리퍼가 보인다).

### 렌더링 함정 2가지 (2026-08-17 그리퍼 검증 중 발견, 둘 다 오진 유발)
- **카메라 near clip 기본값이 ~1 m다.** `rep.create.camera(...)`에 `clipping_range`를 안 주면
  **1 m 이내의 피사체가 통째로 잘려 이미지가 빈 화면으로 나온다.** 에러도 경고도 없다.
  클로즈업을 찍을 땐 반드시 `clipping_range=(0.01, 100.0)` 같은 값을 명시할 것.
  (오버뷰 카메라는 ~2.3 m라 멀쩡했고, 클로즈업만 계속 백지로 나와 "렌더가 깨졌다"고 오진했다.)
- **`UsdGeom.ComputeLocalToWorldTransform` / `UsdGeom.BBoxCache`는 USD를 읽으므로
  PhysX가 적용한 자세를 반영하지 않는다.** `set_joint_positions()` 후에도 이 API들은 기본 자세
  값을 돌려준다. 자세에 의존하는 계산(카메라 타깃, 프레임 축, 바운딩박스)은
  **`RigidPrim.get_world_poses()` 같은 물리 권위 소스**를 써야 한다.
  단, **렌더 자체는 물리 자세를 정상 반영한다** (fabric 경유). 즉 이미지는 맞고 USD 쿼리만 낡았다.
  (검증: 오버뷰 카메라로 reset TCP를 이미지에 투영한 좌표에 실제 그리퍼가 그려져 있음.
  `world.step(render=True)`로 바꿔도 이미지가 사실상 동일 — 평균 픽셀차 0.17.)

### 알려진 경고 (기능에는 영향 없으나 나중에 정리 필요)
- `Mesh '/__Prototype_.../pika_gripper' has corrupted data in primvar 'normal'` —
  pika_gripper.STL의 노멀 primvar 버퍼 크기 불일치. 렌더는 되지만 셰이딩 품질에 영향 가능.
- `attachment_site` / `tcp` / `tool` 링크가 **음수 질량 + 단위 관성텐서** 경고를 낸다.
  해당 URDF 링크에 `<inertial>` 블록이 없기 때문. 기구학/렌더에는 무해하지만,
  동역학을 켜고 접촉을 볼 때(3단계 이후)는 질량을 채워야 한다.

### Pika 툴 장착 회전 조사 결론 (2026-08-17)
사용자가 `cell_overview.png`에서 "6번 조인트의 툴이 90° 틀어져 보인다"고 지적해 조사한 결과:
- **두 URDF는 서로 다른 메시 export를 쓴다.** `rb3_730e.urdf`는 `pika_gripper.STL`
  (손가락 개폐축 = **Y**, bbox y ±107.5)에 **visual origin rpy z=+90°** 를 명시하고,
  `rb3_730e_pika_articulated.urdf`는 `pika_gripper_base.STL`(개폐축 = **X**)을 identity로 쓴다.
  두 export가 정확히 90° 다르고, +90°가 그 차이를 상쇄한다. **URDF는 자기일관적이다.**
- **Isaac 임포트 결과도 명세와 일치한다.** attachment_site 프레임에서 툴 형상의 실측 bbox는
  `x ±107.5 / y −43.9~+104.3 / z 0~247.6 mm` 로, URDF 명세(+90° 적용) 및 몽타주 빌드와 3자 일치.
  → **임포터가 회전을 넣거나 뺀 것이 아니다.**
- 참고: USD의 `tool` 프림 자체 Xform은 −90°로 보이지만, 인스턴스 메시 참조 쪽에 보정 변환이
  들어 있어 최종 월드 형상은 위 실측대로 맞다. 프림 Xform만 보고 판단하면 안 된다.
- **미해결**: 따라서 만약 실기 대비 90° 오차가 실재한다면 그것은 Isaac 파이프라인이 아니라
  **원본 URDF 자산(rb3_730e.urdf 또는 Pika 메시 export)의 문제**다. 저장소에 3인칭 실기 사진이
  없어(전부 손목캠) 하드웨어 대조를 못 했다. `outputs/gripper_isaac.png`(클로즈업)로
  운영자 확인이 필요하다. 확인되면 수정 지점은 `rb3_730e.urdf`의 tool visual origin 한 줄.

## 11. Pika 툴 90° 회전 — 근본원인과 수정 (2026-08-17, §10의 조사 결론을 뒤집음)

**운영자 확인 결과 최종 채택값은 툴 축 기준 +90°.** (270°도 시험했으나 실물 대비 180° 반대로 판정)
결론적으로 **`rb3_730e.urdf`의 tool visual origin `rpy z=+90°`는 원래부터 옳았고, 임포터가
그 값을 조용히 버리고 있었던 것**이 문제였다. robotics_lab 자산에는 오류가 없다.

### 근본원인 (측정으로 확정)
`merge_fixed_joints=False`로 임포트하면 URDF의 `tool` 링크가 **PhysX 강체**가 되고, 그 자세는
`tool_joint`(rpy=0)를 통해 **물리엔진이 소유**한다. 그 결과:
- URDF `<visual><origin rpy>`의 회전이 **렌더에 도달하지 못하고 버려진다**
- 임포트 후 USD의 tool 프림에 xform을 써 넣어도 **매 스텝 물리가 덮어쓴다**

즉 **Isaac은 Pika 툴을 URDF가 의도한 +90°가 아니라 0°로 그리고 있었다.** 이것이 90° 오차의 정체다.

동일 카메라·왼팔만으로 baseline 대비 렌더 픽셀 변화량 (`>10` 계조):

| 조작 | 변화 | 판정 |
|---|---|---|
| 조인트6(wrist3) +90° | **12.51%** | 실제로 회전 (기준값) |
| URDF visual origin yaw +90° | 0.44% | **반영 안 됨** |
| USD tool 프림에 xform +90° | 0.44% | **반영 안 됨** (물리가 덮어씀) |
| **메시 정점에 +90° 베이킹** | **13.04%** | 반영됨 ✅ |

`wrist3_joint` 축이 `(0,0,1)`이고 `attachment_site`/`tool`이 link6에서 rpy=0 순수 평행이동이므로
**툴 z축 = 조인트6 회전축**이다. 그래서 조인트6 회전이 "툴 yaw가 제대로 먹었을 때의 모습"의
정답지 역할을 한다 — 검증할 때 이 비교를 쓸 것.

### 수정 방법 (채택)
회전을 **메시 정점에 굽는 것**만이 임포터를 통과한다.
```bash
.venv/bin/python scripts/bake_tool_mesh.py --deg 90       # 회전된 STL + rpy=0 URDF 생성
OMNI_KIT_ACCEPT_EULA=YES .venv-isaac/bin/python scripts/import_urdf.py \
    --urdf assets/urdf/rb3_730e_bakedrzp90.urdf
```
- **프로젝트 기본 팔 자산 = `assets/rb3_730e_bakedrzp90/rb3_730e_bakedrzp90.usda`**
  (`build_cell.py`의 `ARM_USD` 기본값). robotics_lab 원본은 건드리지 않았다.
- 검증 산출물: `outputs/gripper_final.png`(축방향), `outputs/cell_overview.png`(양팔 반영),
  `outputs/tool_fix_verified.png`(0° vs 베이킹 vs 조인트6 3자 비교)

### robotics_lab 쪽 판정: 이상 없음
채택값 +90°가 `rb3_730e.urdf`의 tool visual origin(`rpy z=+90°`) 및
`collision_monitor.cpp`의 단일 hull 회전(`AngleAxisd(M_PI/2, UnitZ())` = +90°)과 **일치한다.**
따라서 robotics_lab 기하 체인은 정확하며, 실기 충돌 가드가 틀어져 있다는 우려는 해소됐다.
문제는 Isaac 임포터가 그 +90°를 버린 것뿐이고, 수정은 Isaac 쪽 자산에만 필요하다.
(참고: 커밋 `a0ee3f7` 메시지가 roll=pi 제거만 설명해 yaw의 출처가 불분명해 보였으나,
결과적으로 그 yaw 값은 옳았다.)

### 이 조사에서 내가 틀렸던 것 (같은 함정 반복 방지)
- §10에서 "Isaac 형상이 URDF 명세와 일치한다"고 bbox로 결론냈는데 **틀렸다.**
  `UsdGeom.BBoxCache`는 USD를 읽고 **렌더는 PhysX를 따른다.** 둘이 갈라진 상태였고
  나는 USD 쪽만 보고 판단했다. **자세/형상 검증은 반드시 렌더(또는 물리 권위 소스)로 교차확인할 것.**
- 여러 후보를 한 세션에 같은 위치로 겹쳐 놓고 비교하면 **PhysX가 서로 밀어내 자세가 달라진다.**
  후보 비교는 반드시 후보당 독립 프로세스로.
- 클로즈업에 양팔이 다 들어가면 서로 다른 팔의 그리퍼를 비교하게 된다 → `--single-arm` 사용.
- `UsdGeom.Imageable.MakeInvisible()`은 **상속**되므로 link6을 숨기면 그 하위 tool도 사라지고,
  게다가 이후 `get_world_poses()`가 "Failed to get rigid body transforms"로 실패한다.

## 12. 3단계 완료 기록 — 씬 구성 (2026-08-17)

**결과: PASS.** `scripts/build_scene.py --layout {aligned,random}` 로 테이블·박스·M12×25 볼트를
2단계의 검증된 셀 위에 올렸다. 산출물: `assets/scene_{layout}.usd`,
`outputs/scene_{layout}_{overview,top}.png`.

### 좌표 (정본 프레임) — 몽타주를 Rz(+90)로 옮긴 값
몽타주는 작업영역이 −y였고 정본은 +x이므로 `(x, y) → (−y, x)`:

| 대상 | 몽타주 | **정본** |
|---|---|---|
| 회색 볼트 더미 (좌완) | (+0.16, −0.47) | **(0.47, +0.16)** |
| 검정 볼트 더미 (우완) | (−0.16, −0.47) | **(0.47, −0.16)** |
| 회색 박스 (좌완 place) | (+0.17, −0.80) | **(0.80, +0.17)** |
| 녹색 박스 (우완 place) | (−0.17, −0.80) | **(0.80, −0.17)** |

- 테이블 상면 z=0 (스탠드 베이스와 동일 평면), 중심 (0.55, 0), 반크기 (0.50, 0.55), 두께 12mm
- 박스: 외벽 반크기 (0.155, 0.115), 벽높이 0.075, 두께 0.012, **짙은 회색 5cm 인서트 바닥**
- 볼트: 축 φ12×25 + 머리 φ18.4×12, 질량 22g, 강체(shaft+head 실린더 2개 복합)

### 게이트 결과
- **도달성**: 모든 볼트가 담당 팔 마운트 기준 권장 반경 `[0.133, 1.058] m` 안 (22/22 통과).
  거리 실측 — 마운트→더미 0.649 m, 마운트→박스 0.823 m (출처
  `descriptions/reach_envelope_rb3_730e.json`, TCP 프레임 기준이라 Pika 오프셋 포함).
- **안착**: 240스텝 후 볼트 z = 0.0091 m로 전부 수렴 (관통·발산 없음).

### 카메라 함정 2가지 (또 다른 오진 유발 지점)
- **`rep.create.camera(rotation=...)`는 이 환경에서 빈 이미지를 낸다.** `look_at`을 쓸 것.
- **완전한 수직 탑뷰는 시선이 world up과 평행해 degenerate**가 된다. 대신 카메라를 +x로 빼고
  −x를 바라보게 하면 (`pos (1.12,0,1.62) → look_at (0.52,0,0)`) **로봇이 화면 위, +y가 화면
  오른쪽**이 되어 몽타주 구도와 일치한다. 반대로 −x에서 +x를 보면 상하좌우가 뒤집힌다.
- 조명이 세면 재질이 날아가 **검은 볼트가 회색으로 보인다.** 현재값 DistantLight 1100 +
  DomeLight 260이 적정. 재질 판별이 중요한 렌더에서는 조명부터 의심할 것.

### 4단계로 넘기는 메모
- 이 씬은 아직 **실측이 아니라 몽타주 기반 추정**이다 (테이블 크기, 박스 치수·위치, 더미 산포).
  실측값이 들어오면 `build_scene.py` 상단 상수만 교체하면 된다.
- 볼트 재질은 UsdPreviewSurface 단색이다. 4단계에서 실기 프레임과 대조하며
  금속 반사·거칠기를 맞춰야 한다.

## 13. reset 자세 충돌 검사 + 그리퍼 콜라이더 추가 (2026-08-17)

운영자가 "몽타주에서 로봇이 스탠드/바닥과 충돌 상태"라고 지적해 검증한 결과.

### 결론: **reset 자세는 충돌이 없다.**
- **MuJoCo (정확한 narrow-phase, 신뢰 기준)**: dual MJCF + z=0 테이블 평면에서 **접촉 0개**.
  `scripts/check_collision_mujoco.py`. MJCF에는 팔 collision hull과 스탠드 충돌 박스가 모두 있다.
- **Isaac (PhysX 접촉력)**: 16개 링크 전부 `|F| = 0.000 N`. `scripts/check_clearance.py`.
- TCP는 테이블 위 **0.210 m (좌) / 0.224 m (우)** 로 여유가 크다.

### 함께 메운 구멍: 그리퍼가 물리적으로 투명했다
`rb3_730e.urdf`의 `tool` 링크에는 `<visual>`만 있고 **`<collision>`이 없다.** MJCF에는 툴 자체가
없다. 즉 **그리퍼는 Isaac·MuJoCo 양쪽 모두에서 충돌 검사에 아예 참여하지 않았다** — 테이블/스탠드/
볼트 어느 것과도 접촉할 수 없었다. (robotics_lab 실기는 `collision_monitor.cpp`가 런타임에
`pika_gripper_mesh` 등 hull을 붙여 검사하므로 실기 쪽에는 문제가 없다.)

`scripts/bake_tool_mesh.py`가 이제 **`pika_gripper_hull.STL`도 같은 각도로 구워 `<collision>`으로
추가**한다 (hull은 visual 메시와 동일 프레임이라 같은 회전이 맞다). 부수 효과로 PhysX가 콜라이더에서
질량을 산출하게 되어 **tool의 negative-mass 경고도 사라졌다.** 5단계에서 볼트를 집으려면 필수다.

### 이 과정에서 또 걸린 함정
- **`import_urdf.py`를 같은 이름으로 다시 돌리면 덮어쓰지 않고 `<name>_1/` 디렉토리를 새로 만든다.**
  그래서 재임포트 후에도 검사 스크립트는 **옛 자산**을 보고 있었다. 재임포트 전에 기존 디렉토리를
  지울 것.
- **조인트 드라이브 타깃 기본값은 0이다.** `set_joint_positions()`만 하면 상태만 바뀌고 드라이브가
  0으로 끌어당겨, 몇십 스텝만 지나도 자세가 무너진다(첫 검사에서 좌 TCP가 0.21 → −0.05로 처짐).
  정적 검사는 `set_gravity(0)` + `apply_action(ArticulationAction(joint_positions=q))`를 함께 쓸 것.
  (`SingleArticulation`에 `set_joint_position_targets`는 없다.)
- **USD 바운드로 기하 간섭을 재려던 시도는 실패했다** (`scripts/measure_clearance.py`).
  프로토타입 바운드는 **mm 단위**이고 스케일(0.001)이 프림 변환에 들어 있어서, 이를 미터 단위 물리
  pose와 섞으면 −459 m 같은 값이 나온다. **기하 검증은 MuJoCo로 하는 편이 안전하다** —
  이 저장소에는 제대로 된 collision hull이 있는 MJCF가 있다.

## 14. 손목 카메라(D405) 위치 — CAD 기하로 확정 (2026-08-17)

§4에 "손목캠 hand-eye 미측정이라 4단계 블로커"라고 적었으나, **그리퍼 CAD 메시에서 직접
렌즈 위치를 복원**해 공칭 hand-eye를 얻었다. 조립 오차는 못 잡지만 눈대중과는 차원이 다르다.

### 방법
`assets/meshes/pika_gripper_rzp90.STL`(= 툴/attachment 프레임)을 z축으로 얇게 잘라 x-y 단면을
그리고, 원형 피처를 클러스터링 후 원 피팅. 스크립트:
`find_wrist_camera.py`(광역 아틀라스) → `zoom_wrist_camera.py`(확대) → `fit_d405.py`(정밀).

### 결과 (툴 프레임, mm)
`z = 116` 단면에서 두 개구부가 깨끗하게 분리된다:

| 피처 | 중심 (x, y) | 반경 | rms |
|---|---|---|---|
| 이미저 A (툴 −x) | (−9.17, 46.01) | 5.65 | 0.22 |
| 이미저 B (툴 +x) | (+9.17, 46.01) | 5.65 | 0.22 |
| 장착 나사 ×2 | (±16.00, 54.51) | 2.00 | 0.00 |

- **베이스라인 18.34 mm** ↔ D405 공칭 **18 mm**. 이 일치가 D405임을 확정하는 근거다.
- **모듈 전면 z = 119.3 mm** (핑거 팁 평면 247.64 → 렌즈는 팁보다 **128.3 mm 뒤**)
- **광축 = 툴 +z** (접근 방향, 파지점을 향함)

### ⚠️ y≈73의 둥근 돔은 fisheye 렌즈다 (운영자 확인)
z 122~126에서 반경이 10→7.5→4 mm로 좁아지는 돔이 있는데, 처음에 이것을 D405로 잘못 지목했다.
**이 리그는 fisheye를 쓰지 않으므로 무시할 것.** D405는 그보다 아래(y≈46)의 납작한 모듈이다.

### 카메라 프레임 규약 (운영자 확인 반영)
- **컬러 스트림 = LEFT 이미저**, 그리고 left/right는 **카메라 시점 기준**이다.
- 광축 = +z_tool, 위 = +y_tool로 두면 image-right = forward × up = ẑ × ŷ = **−x_tool**.
  따라서 **카메라 시점의 left = +x_tool**, 즉 **컬러 광학 중심 = (+9.17, 46.01, 119.3) mm**.
- 위를 +y_tool로 잡는 근거: 손가락이 y≈0 (렌즈보다 46 mm 아래), x≈±50~105에 있으므로
  **화면 하단 좌우**에 잡힌다 — 실기 손목캠에서 핀레이 핑거가 하단 좌우에 보이는 것과 일치.
- USD 카메라는 로컬 −Z를 보고 +Y가 위이므로, 툴 프레임 대비 **Ry(180°)** 를 적용한다.

### 사용법
`scripts/render_wrist_cam.py`가 `tool` 링크의 **물리 pose**에서 위 오프셋을 적용해 카메라를
배치한다(USD 변환은 stale이므로 쓰지 말 것). D405 FOV 87°에 맞춰 focal 11.0 mm /
horizontal aperture 20.955 로 설정. 결과는 `outputs/wristcam_{layout}_{side}.png`,
실기 대조는 `scripts/compose_wristcam.py`.

### 카메라 구성 확정 (운영자)
- **fisheye는 실험에서 제외됐고, 이제 D405만 쓴다.** `policy_runner/config/flow_real_fisheye.yaml`
  이라는 이름과 그 안의 `wrist_source: fisheye` / 0.65 center-crop 기록은 **현재 구성과 다르다.**
  5단계에서 관측 계약을 짤 때 config 이름을 믿지 말 것.
- CAD 공칭이므로 조립 오차는 미반영. 실기 프레임과 렌더를 대조해 잔차를 봐야 한다.

## 15. 손목캠 뷰 디버깅 — 확인된 사실 (2026-08-17)

4단계 목표는 **손목캠 뷰를 실기와 맞추는 것**이고, 현재 남은 작업은 카메라 변환 행렬을
정확히 잡는 것이다. 아래는 이 과정에서 **측정으로 확정된** 사실들이다.

### 확정 1 — 좌우 팔의 손목캠 뷰가 달랐던 것은 실제 버그였고 수정됐다
운영자가 "left는 그리퍼 팁 2개, right는 1개가 보인다"고 지적. 씬 물체 → 반대편 팔 → 스탠드를
차례로 제거해 **자기 그리퍼만** 남기고 비교해도 좌우가 17.6% 달랐다.

- **원인**: 카메라를 `RigidPrim.get_world_poses()`가 주는 **PhysX 바디 프레임**에 수동으로
  오프셋을 곱해 배치했다. 이 프레임은 질량·관성 주축 기준이라 **기하가 동일해도 좌우 액터에서
  축 배정이 다르게 잡힐 수 있다.**
- **수정**: 카메라를 **tool 프림의 자식으로 붙여** 씬 그래프가 변환을 처리하게 했다.
  차이 **17.6% → 0.16%** (남은 건 월드 조명 각도 차이).
- **교훈**: 링크에 센서를 붙일 때 물리 바디 pose로 합성하지 말고 **프림 부모-자식 관계**를 쓸 것.
- 참고: 툴은 `pika_gripper.STL` 단일 강체라 **손가락 관절이 없다** — 개방 정도 차이는 원인이 아니다.

### 확정 2 — 그리퍼 하우징은 시야를 가리지 않는다 (STEP 불필요)
운영자가 STEP 파일(`~/workspace/pika_gripper_tip/Pika Gripper模型外发.STEP`)을 제공해
"렌즈를 가리는 하우징만 빼자"고 제안. 그 전에 기하로 검사한 결과:

```
carve cone: half-angle 50 deg, depth 30 mm from the lens  ->  removed 0 triangles (0.00%)
```
**렌즈 앞 50° 원뿔 30 mm 안에 재질이 하나도 없다.** 따라서 검은 화면의 원인은 하우징이 아니고,
STEP 기반 부품 제외는 이 문제에 **필요하지 않다**. (`scripts/carve_lens_view.py`가 이 검사를 한다.)

STEP 자체는 파싱 가능하다 — `cadquery`/OCP가 `~/workspace/pika_gripper_tip/.venv`에 설치돼 있고,
`STEPCAFControl_Reader` + `XCAFDoc_ShapeTool.GetComponents_s`로 **18개 부품을 이름과 함께** 읽었다
(`M-D080-950001-C`, `DECXIN-2M-2159V2-1`(카메라 모듈), `4310 감속모터`, `Pika Tracker PCBA V2` 등).
단, 이 STEP은 **핸들·트래커까지 포함한 전체 장치**(z −63~235)라 로봇에 붙는
`pika_gripper.STL`(z 0~247.6)과 좌표계가 다르다 — 부품 단위로 쓰려면 정합이 선행돼야 한다.

### 확정 3 — 단위 스케일 문제도 아니다
`link6` / `attachment_site` / `tool` 프림의 local·world 스케일이 **전부 1.0**이고
`attachment_site`의 local translation이 `(0, 0, 0.1)` m로 URDF와 일치한다. 즉 tool 프림 프레임은
**미터 단위**이고, 미터 단위 오프셋을 자식에 그대로 주면 된다.

### 미해결 → 카메라 API로 교체
손으로 작성한 USD 행렬에서 시선 방향이 뒤집혔다. 진단: 카메라를 손끝 너머(z=0.30 m)에 두고
렌더하니 **테이블이 아니라 그리퍼가 정면으로** 보였다(= 뒤를 보고 있었다). 그런데 방향을 뒤집어도
렌즈 위치에서는 여전히 검게 나와, 행/열 규약과 적용 시점을 특정하지 못했다.

**해결책: 행렬을 손으로 쓰지 말고 `isaacsim.sensors.camera.Camera`를 쓴다.**
`set_local_pose(translation, orientation, camera_axes=...)`가 축 규약을 문서화해 처리한다:

| camera_axes | 규약 |
|---|---|
| `world` | +Z up, **+X forward** |
| **`ros`** | **+Y up, +Z forward** ← 우리가 원하는 것 |
| `usd` | +Y up, −Z forward |

`camera_axes="ros"` + identity 쿼터니언이면 **광축 = +z_tool, 위 = +y_tool**이 되어 손가락이
화면 하단에 온다. 광학 파라미터(focalLength 11.0 / horizontalAperture 20.955 = D405 87°)는
헬퍼가 stage-unit 변환을 섞으므로 **USD 속성에 직접** 설정한다.

## 16. place 박스 실측 치수 (2026-08-17)

몽타주의 추정 치수를 **실측값으로 대체**했다. 출처는 robotics_lab의 삭제된 파일이다:

```
commit 83c5458^ : camera_server/stereo_worker/box_detect.py
BOX_DIMS = (0.380, 0.240, 0.105)   # NPC NTC-321 외부 실측(380×240×105mm). 측벽 20mm, 바닥 6.5mm.
_open_tray_model(dims=BOX_DIMS, wall=0.020, floor=0.0065, sponge=0.030, ...)
```

| 항목 | 몽타주 추정 (폐기) | **실측 (NPC NTC-321)** |
|---|---|---|
| 외부 W×D×H | 310×230×160 mm | **380×240×105 mm** |
| 측벽 두께 | 12 mm | **20 mm** |
| 바닥 | "짙은 회색 5cm 인서트" | **바닥 6.5 mm + 스펀지 30 mm** |

- **높이가 크게 다르다**: 160 → **105 mm**. 추정치가 50% 이상 높았다.
- 내가 "5cm 인서트"로 만든 것은 실제로는 **6.5 mm 바닥 + 30 mm 스펀지**다.
  `box_detect.py` 주석에 "카메라는 바닥이 아니라 **스펀지 윗면**을 보므로(측정 local-z≈−0.019)"라고
  적혀 있어, **테이블 기준 36.5 mm 높이의 스펀지 윗면이 실제 place 대상면**이다.
- 박스 x는 0.80 → **0.72** (운영자: reach 안이면 되지만 로봇에 가까운 편이 낫다).
- ICP 모델이 쓰던 참고값도 함께 기록: 긴변 0.38 m 축이 head 카메라 2D에 항상 보인다는 yaw prior,
  size gate long 0.16~0.60 m / short ≤ 0.42 m.

### 함께 확인된 사실: 배포 정책은 손목캠 전용
`docs/archive/head_stereo/README.md`에 명시 — "policy_runner's pi0.5 rollout is
**wrist-camera-only** (`camera.bundle.policy`)". head D435는 rb_gui 표시용으로만 남아 있고
`cm_bridge`는 perception 입력이 아예 없다. 즉 5단계 관측은 **손목 D405 2대만** 렌더하면 된다.
(fisheye는 운영자 확인대로 미사용.)

### 손목캠에 찍히던 작은 점 = URDF 프레임 마커 (2026-08-17)
운영자가 "그리퍼 팁 사이에 작은 점이 보인다"고 지적. 정체는 **`rb3_730e.urdf`의 프레임 마커**다:

```
link=tcp              visual SPHERE radius=0.0025  origin=0 0 0
link=attachment_site  visual SPHERE radius=0.0025  origin=0 0 0
```

`tcp`는 핑거 팁 평면(link6 + 347.6 mm)에 있으므로 **정확히 그리퍼 팁 사이**에 2.5 mm 점으로
렌더된다. 시각화 편의용 마커가 정책 관측 이미지에 섞여 들어가는 셈이라, 실기에는 없는 물체다.
`scripts/bake_tool_mesh.py`가 이제 생성 URDF에서 **이 두 sphere visual을 제거**한다
(로그에 `dropped 2 frame-marker sphere visual(s)`). 링크 자체는 남기므로 FK/카메라 부착은 그대로다.

박스 간격도 운영자 요청으로 넓혔다: 중심 간격 390 → **430 mm** (장변 380 mm + 50 mm 여유).

## 17. 관측 계약 확정 + 그리퍼 개폐 (2026-08-17)

### 관측 계약 — `~/workspace/openpi`에서 직접 확인, 블로커 해소
서빙 프로세스 인자에서 config를 특정했다:
```
serve_policy.py --port 8001 policy:checkpoint
  --policy.config pi05_pika_umi_wrist_velgrip_k1_h24_80k
  --policy.dir /home/plaif/workspace/pika_umi_models_v2/wrist_velgrip_k1_80k/79999
```

| 항목 | 값 | 출처 |
|---|---|---|
| image_keys | `("left_wrist_0_rgb", "right_wrist_0_rgb")` | config.py |
| drop_base_image | `True` (head/base 미사용) | config.py |
| include_depth | **`False`** — RGB만 | config.py |
| action_horizon | **24** | config.py |
| action_dim | 32 (14-dim 유효) | config.py |
| 이미지 전처리 | **`resize_with_pad` 224×224** | `resize_pad` 기본값 `True` (config.py:131) |

전처리 실제 동작 (`packages/openpi-client/src/openpi_client/image_tools.py`):
```
ratio = max(640/224, 480/224) = 2.857
resize BILINEAR -> 224 x 168
zero-pad 상하 각 28행 (letterbox, 좌우 패딩 0)
```
→ `build_scene.py --wrist-cam`이 원본 `wristcam_*.png`와 함께 정책 형식
`policyimg_*.png`(224×224)를 같이 저장한다.

주의: 팀이 추가한 `resize_no_pad`(비율 왜곡, 224행 전부 사용) 변형이 있으나
**서빙 config는 이를 쓰지 않는다**(`..._depth_nopad_h24` 두 개만 사용).
서빙 config 이름의 `velgrip`은 grip 채널이 velocity 계열임을 시사 — llm-wiki의 grip-echo 진단
(2026-08-16)과 함께 5단계에서 grip 연결 시 반드시 확인할 것.

### 그리퍼 개폐 — articulated 자산으로 전환 완료
`rb3_730e.urdf`의 통짜 `pika_gripper.STL`은 손가락이 못 움직여 grip 채널을 반영할 수 없었다.
**`rb3_730e_pika_articulated.urdf`** 로 전환했다 (`scripts/make_articulated_urdf.py`):

- `tool` = `pika_gripper_base.STL` — **flange 어댑터와 RFT64 FT 본체가 이 메시에 이미 포함**돼 있어
  별도 FT 메시(`mo_robot_descriptions/.../RFT64_6A01.stl`) 부착은 불필요(중복 위험).
  프레임은 URDF에 이미 있음: attachment_site → ft_sensor_base(+15 mm 어댑터) → ft_sensor_measurement(+30 mm 센서).
- `finger_left` / `finger_right` = 별도 메시 + **prismatic ±X, limit ±0.05 m**
- 이 메시들은 **+90° 회전이 export에 이미 구워져 있어** `bake_tool_mesh.py`의 회전 베이킹이 불필요해짐
- 스크립트가 추가하는 것: 툴·핑거 **collision hull 3종**, 마커 구체 축소(§16의 함정)
- **STEP 분해는 필요 없었다** — 필요한 분리가 이미 URDF에 있다.

**조 모델 (robotics_lab 런타임과 동일)**:
```
gripper_finger_travel_m: 0.047     # rb_servo_server/config/stack_{real,sim}.yaml
finger_pos = (1 - grip/100) * 0.047  # collision_monitor.cpp
grip 100 = 열림(메시가 authored된 상태), grip 0 = 47 mm 안쪽으로 닫힘
```
운영자 확인: 실기 grip은 닫힘 0 / 열림 95~100 (원형 모터 각도).

**검증 결과**: 8 DOF 임포트 확인(`base/shoulder/elbow/wrist1/wrist2/wrist3/finger_left/finger_right`),
관절이 목표에 정확히 도달(grip 0 → L=+0.0470, R=−0.0470), **핑거 링크 월드 좌표도 ±47 mm 이동**.

### 미해결: 손목캠 화면에서 핑거 이동이 안 보인다
관절·링크는 움직이는데 손목캠 렌더는 개폐 간 픽셀 차이 0.0%다. 화면 하단의 어두운 형상은
움직이지 않으므로 **핑거가 아니라 툴 베이스의 고정 레일**로 보이고, 실제 핑거 메시는 D405
화각의 가장자리 밖이거나 극단에 있을 가능성이 크다. 그런데 **실기 프레임에서는 핑거가 하단에
크게 보인다** — 따라서 카메라 pose나 핑거 형상 중 하나가 아직 실기와 다르다.

관련 단서: ~~실기 핑거는 AgileX Fin-Ray 소프트 핑거~~ → **정정 (운영자, 2026-08-19):
실기는 현재 강체 팁을 쓴다.** 따라서 sim의 강체 핑거는 하드웨어와 일치하며,
"실기의 유연 핑거가 높이 오차를 흡수한다"는 설명은 **성립하지 않는다**
(§20의 dz +26~42mm 계통 오차를 이걸로 설명하려던 가설은 폐기).
5단계 전에 이 차이를 확인할 것.

## 18. 5단계 폐쇄루프 (2026-08-17)

`scripts/eval_closed_loop.py` — Isaac 씬이 :8001의 pi0.5에 직접 접속해 30 Hz로 돈다.
운영자 결정: **(B) 충실형 컨트롤러**, **CM controller가 아닌 chunk follower** 사용.
근거는 pi0.5의 proprio에 velocity가 들어가므로 컨트롤러가 관측을 오염시킨다는 것.

### 배포 계약 (모두 코드에서 확인, 추정 금지)
운영자가 준 실기 실행 명령의 파라미터를 그대로 옮겼다:
```
ACTION_HORIZON=24  STITCH=boundary  CHUNK_EXECUTE_STEPS=4  CHUNK_ANCHOR=command
SPEED_SCALE=1.0    RTC=1  PREFETCH_AT=0  RTC_SCHEDULE=zeros  INCLUDE_DEPTH=0
VELPROPRIO_SOURCE=command   VELPROPRIO_SAMPLE=fixed_step   --proprio-mode velocity_grip
```
- prompt(고정): `openpi_remote.py:64` — "pick up the black bolt with the right arm and put it
  in the right box, then pick up the gray bolt with the left arm and put it in the left box"
- 이미지: 손목 2대 **원본 640×480**을 보낸다. 서버가 `resize_with_pad`를 하므로 미리 224로
  줄이면 이중 처리가 된다.

### ⚠️ velproprio 계산 — 세 번 틀렸던 지점
정답은 `openpi_remote.py:1502` 주석에 있다:
```
openpi _arm_velocity:  cur^-1 . (p_next - p_cur),   cur^-1 . R_next
```
| 항목 | 틀린 구현 | 정답 |
|---|---|---|
| 단위 | `Δ / policy_dt` | **나누지 않는다** — `fixed_step`은 단일 30 Hz 프레임 델타 |
| 프레임 | 월드 차분 | **이전 body 프레임** (`R_cur^T ·`) |
| 정렬 | 없음 | **R_align**(pika_rz180, RB TCP → EE tip) 적용 |

단위 오류만으로 정책에 **30배 부풀린 속도**가 들어갔고, 명령이 폭주했다(좌완 x 0.359 → −0.239).
수정 후 우완 추종오차 604 mm → 74 mm.

**출처는 `command`이지 measured가 아니다** — 방출된 절대 TcpPoseTarget 이력의 차분이며
hold 틱은 ZOH. openpi 쪽 주석은 "measured"라고 쓰여 있으나 실제 배포는 command다.

### 청크 적분
- 합성은 `flow_dataset.pose_compose_local`: `p + R_ref·Δp`, `q_ref ⊗ rotvec(Δr)`
- **청크 경계마다 재앵커링**(`flow_inference.py:1969`, `_steps_since_boundary == 0`).
  무한 누적하면 명령이 로봇에서 멀어진다.
- 스텝당 클램프 `max_velocity × policy_dt` (`_clamp_delta_step`)
- 액션의 grip은 서버가 /100로 주므로 ×100 → percent (절대값 규약)

### 컨트롤러
Ruckig 6-DOF 팔로워, 한계는 `stack_real.yaml`의 `ruckig_follower` 그대로:
`lin 0.45 / 12.0 / 4000`, `ang 0.90 / 40.0 / 8000`. 물리 500 Hz, 정책 30 Hz(17 substep).
IK는 **URDF 해석적 FK/Jacobian**(`SingleArticulation`에 `get_jacobians`가 없다 — 배치
`Articulation`에만 있음). 해석 FK vs 물리 TCP = **0.017 mm**로 검증됨.

### ⚠️ 떨림의 원인 = 관절 속도 한계 초과
운영자가 "실기는 안 떨리는데 sim은 떨린다"고 지적. 원인은 IK 증분 상한 ±0.05 rad/substep이
500 Hz에서 **25 rad/s**로 URDF 한계 **3.14 rad/s**의 8배였던 것. 연쇄:
포화 → 추종오차 누적(175 mm) → 청크 경계 재앵커링 점프 → velproprio 스파이크 → 진동.
**수정**: 상한을 `3.14 × dt = 0.0063 rad`로 낮추고 DLS 반복을 3회로 늘려 수렴 보상.
추종오차 **175 mm → 0.0~2.1 mm**.

### 첫 결과 (정렬 배치 4 에피소드 × 30초)
**정답 배치 5개 / 오배치 0개**, 3/4 에피소드에서 1개 이상 성공.
명령 궤적이 정상 pick-place 패턴을 그린다 (x 0.47 더미 → 0.65 박스, z 0.005 파지 후 상승,
grip 닫힘↔열림 순환). 볼트는 실기 밀도에 맞춰 **색당 10개 = 20개**.

### 남은 것
- 성공 판정이 "볼트 중심이 정답 박스 XY 안"으로 느슨하다. 초기 배치(x≈0.47)와 박스
  (x 0.60~0.84)가 겹치지 않아 현재는 문제없지만, 에피소드 시작 대비 변화량으로 세는 편이 엄밀하다.
- `_apply_chunk_crossfade`(실기의 청크 경계 crossfade)는 아직 미포팅.
- `attachment_site`/`tcp`의 음수 질량 경고 — 마커를 1e-5로 줄인 부작용, 기구학엔 무해.

## 19. 떨림 진단 종결 (2026-08-18) — 원인은 렌더 스텝이었다

운영자: "실기는 같은 모델·chunk follower로 안 떠는데 sim은 떤다. RTC 넣기 전에 이것부터."
그리고 **"제어기를 스무딩으로 조절하면 안 된다 — 제어기는 chunk를 최대한 추종하고,
떨림은 real처럼 물리로 잡아야 한다"** 는 원칙을 명시했다. 이 원칙을 지켜 해결했다.

### 결론: `World(rendering_dt=...)` 누락
```python
world = World(stage_units_in_meters=1.0, physics_dt=PHYSICS_DT)   # ← rendering_dt 누락
```
**`rendering_dt` 기본값은 1/60 s다.** physics_dt=1/500이므로 렌더 스텝 1회가 타임라인을
**물리 스텝 8.33개**만큼 진행시킨다. 비디오는 정책 틱마다(=30 Hz) 1프레임을 찍으므로,
**녹화할 때만 30 Hz로 로봇이 튀었다.** 수정: `rendering_dt=PHYSICS_DT`.

| 지표 | 수정 전 | 수정 후 | 교란 없는 기준선 |
|---|---|---|---|
| knot(30 Hz) 대역 에너지 | 20.8% | **1.8~3.4%** | — |
| 저크 RMS | 144~218 um | **19~74 um** | — |
| 측정/명령 저크 비 | 10.2x | **1.3~2.9x** | 0.8x |
| 서브스텝 최대 TCP 점프 | 6.58 mm | **0.99~1.81 mm** | 1.80 mm |

산술 일치가 진단의 결정타였다: 1/60 s x 0.4 m/s = 6.7 mm ↔ 실측 최대 점프 6.58 mm.

### 진단 방법 (재사용할 것)
1. **측정 주파수를 맞춰라.** 처음엔 TCP를 정책 틱(30 Hz)마다 기록했는데, 떨림은 33 ms knot
   **안에서** 일어나므로 완전히 에일리어싱됐다. A/B가 구분이 안 된 이유. **500 Hz(물리 스텝)로
   기록**하고, 속도 스펙트럼의 30 Hz+배음 대역 에너지 비율을 지표로 쓴다.
2. **명령 vs 측정 분리.** 명령 TCP(=FK(q_cmd))와 측정 TCP의 저크를 각각 재면 제어기 문제인지
   물리 문제인지 즉시 갈린다. 여기선 명령 17 um / 측정 179 um → 제어기 무죄.
3. **record/replay로 단일 변수 대조.** `TREMOR_DIAG=1`로 관절 명령 스트림을 기록하고
   `TREMOR_REPLAY=<npz>`로 재생하면 궤적이 서브스텝까지 동일(검증: 오차 0.00000000 rad)해서
   볼트 유무·렌더 유무 같은 단일 변수만 바꿔 비교할 수 있다.
   `scripts/analyze_tremor.py`가 저크 상위 5% vs 하위 50% 구간의 신호 대비를 출력한다.

### 배제된 가설 (데이터로 반박됨 — 다시 의심하지 말 것)
- **손목 특이점**: 떨림 순간 자코비안 `smin`이 오히려 같거나 높음 (x0.89~1.48).
- **IK 포화/미수렴**: `dq`, `res`가 DQ_MAX의 1/1000 이하.
- **접촉(볼트/강체 핑거)**: 동일 명령 재생, 볼트 20개 vs 0개 → 저크 16.2 vs 15.7 (사실상 동일).
  실기 Fin-Ray 유연 핑거 격차는 이 떨림의 원인이 **아니다**.
- **`pause_timeline=True`**: 시도했으나 **악화**(최대 점프 6.58 → 22.04 mm). 타임라인 정지/재개
  불연속만 추가할 뿐 dt 불일치를 못 고친다. `pause_timeline=False` 유지.

### 함께 잡은 실제 버그 2개 (둘 다 실기엔 있고 sim에만 없던 것)
1. **Ruckig 목표 속도 누락** — `target_velocity`를 0으로 두고 갱신하지 않아 33 ms knot마다
   **완전 정지**를 목표로 삼았다. 실기는 `cartesian_chunk_follower.cpp:301`에서
   `vf = (d_k + d_kp1) / (2*dt)` 중앙차분을 쓰고, `chunk_window.hpp`의 `reserve_R`
   (stack_real.yaml `reserve_steps: 4`)가 그 forward neighbor 확보용이다.
   → 청크 경계에서 horizon 전체를 절대 knot으로 적분해 두고 중앙차분 속도를 함께 준다.
   실측 knot 대역 에너지 20.8% → 12.9%. **이건 스무딩이 아니라 충실도다** (knot마다 멈추는 쪽이
   오히려 chunk가 함의하는 연속 운동에서 벗어난다).
2. **관절 한계 클램프 누락** — IK가 `qc`를 적분하며 한계를 안 봤다. 명령이 한계를 넘으면 PhysX는
   관절을 스톱에 고정하고 명령은 계속 적분되어, 명령 앵커가 `FK(q_cmd)`(실기와 같은
   `chunk_anchor=command`)이므로 격차가 **단조 증가해 635 mm**까지 벌어졌다.
   실측: 좌 j1이 에피소드의 74.3% 동안 한계 초과(최대 6.841 rad), 우 j3가 44.5%.
   실기 대응 코드는 `safety_filter.cpp:251` `clampJointLimits`.
   → `JOINT_LO/HI`(URDF 실제 가동범위; PhysX가 강제하는 값)로 클램프. 위반 0%, 편차 386 → 0.3 mm.
   관절별 속도 한계도 stack_real.yaml `dq_max_deg_s [170,170,170,240,240,320]`로 교체
   (기존엔 전 관절 일률 3.14 rad/s라 손목이 실제보다 느렸다).

### 이 세션에서 내가 틀렸던 것
- 지표를 정책 틱으로 재서 **측정하려는 현상에 눈이 먼 상태**로 A/B를 두 번 돌렸다.
- 자유공간 대조를 볼트만 빼고 돌려서, 정책이 잡을 대상을 잃고 도달 불가 명령을 내는 바람에
  **대조가 성립하지 않았다**(편차 141~350 mm). 단일 변수 대조는 record/replay로 해야 한다.
- 재생 소스로 관절 스트림이 없는 옛 npz를 지정해 두 런이 18초 만에 죽었는데, 종료코드를 즉시
  확인하지 않아 timeout 1200 s를 그대로 태웠다. → 시작 시 키를 검증하고 즉시 죽는 가드 추가.
- `pause_timeline=True`를 원인으로 지목했다가 악화시켰다. 증상 해석(렌더가 물리를 교란)은
  맞았지만 지점이 틀렸다.

## 20. std20 고정 평가 프로토콜 + T1 multimodality 계측 (2026-08-19)

운영자 결정: **모든 모델 실험·평가는 하나의 고정 세트로 채점한다.**

```bash
scripts/run_std20.sh <label> <host> <port> [aligned|random] [--rtc ...]
```
`--protocol std20` = **20 에피소드 / seed 100~119 / 30초 / 볼트 20개(색당 10)**.
프로토콜이 이 노브들을 강제로 덮어쓰므로 두 arm이 평가 설정 차이로 갈릴 수 없다.
집계는 `scripts/t1_report.py summary_<label>_<layout>.json`.

### T1 계측 필드 (close event마다) — sim만 가능한 측정
시뮬레이터는 모든 볼트의 ground-truth pose를 안다. 실기에는 이 계측기가 존재한 적이 없다.

| 필드 | 잡는 것 |
|---|---|
| `margin` = d(2nd nearest) − d(nearest) | **모호성 폭**. 판정축 = *grasp율 vs margin 곡선* |
| `t_seg`, `perp` | 조준점이 두 후보를 잇는 선분 위인가 = **모드 평균 서명** |
| `switches_1s`, `commit_lead_ticks` | 청크 경계 타깃 갈아타기 = **모드 스위치 서명** |
| `nearest_is_target_color` | 무작위 배치에서 잘못된 색 집기 |
| `same_color_crowding_m` (에피소드) | 씬 난이도 — 쉬운 씬과 동률 씬을 한 숫자로 묶지 않기 위해 |

타깃 추적은 TCP가 아니라 **청크 종점(`knots[-1]`, 24스텝 앞 의도)**의 최근접 볼트로 잰다.
TCP는 지평 전체만큼 늦어서 "지금 어느 볼트를 노리는가"를 못 본다.
볼트 20개는 매 틱 배치 `RigidPrim` 뷰로 읽는다(개별 읽기는 에피소드당 20×900 왕복).

**판정 규칙을 미리 못박음**(사후 합리화 방지): margin 버킷별 grasp율이 **평평하면
multimodality는 범인이 아니고 정밀도/그립이 범인**, **작은 margin에서 꺾이면 모드 문제**.

### ⚠️ 결론: std20은 **비디오 ON**으로 고정한다 (2026-08-19, 아래 함정의 후속)

비디오를 끄면 빨라지지만(30초 에피소드 ~4분 → ~2분) **결과가 달라진다.** 같은 코드·같은 seed 104:

| 런 | correct | closes | grasped | close dxy p50 |
|---|---|---|---|---|
| ab_rtc1 (과거, 비디오 ON) | 4 | 12 | 2 | 23.3 mm |
| ab_rtc0 (과거, 비디오 ON) | 2 | 45 | 3 | 26.6 mm |
| 수정본 + **비디오 ON** | **2** | 3 | 2 | **10.6 mm** |
| 수정본 + 비디오 OFF | **0** | 9 | 1 | **260 mm** |

손목캠 관측 이미지는 비디오 OFF에서도 **정상임을 덤프로 확인**했다(`EVAL_DUMP_OBS=<dir>`로
정책이 받는 프레임을 그대로 저장 — 회색/검정 더미·박스·핑거가 제대로 보이고 갱신됨).
그런데도 결과가 갈린다. 유력한 경로는 §19의 그 메커니즘이다: 녹화 시 `rep.orchestrator.step()`
호출 횟수가 달라지고, **orchestrator 스텝은 타임라인을 진행시킨다.**
이 프로젝트의 검증된 수치는 전부 비디오 ON에서 나왔으므로, 속도를 위해 평가 기반을 걸지 않는다.
(비디오 OFF는 나중에 n=20 동등성 시험을 통과하면 다시 검토.)

### ⚠️ 이 작업에서 터진 함정 — `--no-video`가 정책에 검은 화면을 먹였다
`rep.orchestrator.step()`이 **`if args.video:` 안에만** 있었다. 손목 카메라도 replicator
render product이므로 orchestrator가 스텝해야 애노테이터가 채워진다. 비디오를 끄면:

1. 손목캠 `get_rgba()`가 빈 배열 반환
2. `_observe()`가 그걸 **조용히** `np.zeros((480,640,4))`로 대체
3. 정책이 검은 이미지를 보고 그리퍼를 한 번도 안 닫음
4. **10 에피소드가 0/0으로 채점되고 정책 결과처럼 보였다**

부수 피해: "비디오 off가 3.5배 빠르다"는 실측도 무효였다 — 빨랐던 게 아니라 렌더를
아예 안 했던 것. (렌더를 고친 뒤의 정직한 값은 30초 에피소드 기준 ~2분 vs ~4분이고,
그마저 위 표의 이유로 채택하지 않는다.)

또 하나 고친 것: **씬 난이도(`same_color_crowding_m`)를 에피소드 끝에서 재고 있었다.**
팔이 더미를 흐트러뜨린 뒤라 같은 seed가 런마다 다른 난이도를 보고했다(35/35 vs 45/29).
난이도는 결과가 아니라 조건이므로 **정착 직후 시작 씬**에서 잰다.

**수정**: orchestrator 스텝을 `--video`에서 분리해 **매 틱 항상** 호출하고, 비디오일 때만
프레임을 적재한다. 오버뷰 render product(960×720)는 녹화할 때만 생성한다.
빈 프레임 대체는 이제 `blank_obs`로 계수되고, **첫 관측이 비면 즉시 abort**하며,
summary에 `blank_obs`가 0이 아니면 `!! INVALID RUN`을 찍는다.

**교훈(§19의 반복)**: 이 리그의 오진은 전부 "조용한 기본값"에서 나왔다 — 렌더 near clip 1m,
`rendering_dt` 기본 1/60, 그리고 이번의 zeros 폴백. **관측 경로의 폴백은 소리를 내야 한다.**

**과거 런은 무사하다**: post/ab_rtc/s5/nolang/v2lang/cmp_* 는 전부 비디오 ON으로 돌았고
(mp4가 있다) orchestrator가 매 틱 스텝했다. 관측 타이밍(틱 끝 렌더 → 다음 틱 관측,
33ms 카메라 지연)도 수정 후 그대로 보존했다.

## 21. 제어기 동일성 원칙 + stack_real.yaml 전수 대조 (2026-09-03)

### 원칙 (운영자 결정, 이 리그의 상위 규칙)

**이 리그의 제어기는 robotics_lab 실기 제어기와 같은 것이다. 파라미터까지 같아야 한다.
sim 성공률이 떨어지더라도 실기 제어기와 일치하는 쪽을 택한다.**

§19의 "떨림은 스무딩이 아니라 물리로 잡는다"와 충돌하지 않는다. 금지 대상은 **실기에 없는**
감쇠 가드를 sim에 새로 만들어 넣는 것이고, **실기가 이미 돌리고 있는 단**을 옮겨오는 것은
충실도다. 판정 기준은 "부드러워지는가"가 아니라 "`stack_real.yaml`에 있는가" 하나다.
어떤 값을 바꾸고 싶으면 sim에서 튜닝하지 말고 실기 config를 근거로 제시할 것.

### ⚠️ 정본 config는 HEAD가 아니라 `fad2cd4^` 다

`stack_real.yaml`과 `stack_sim.yaml`은 **2026-09-02에 둘 다 RB5-850E로 전환**됐다
(`fad2cd4` "Switch stack_real to the RB5-850E", `9b295d6` stack_sim). 이 리그는 **RB3-730E**다.
HEAD를 그대로 베끼면 다른 팔의 값을 넣게 된다. 반드시 이렇게 읽을 것:

```bash
git -C ~/workspace/robotics_lab show fad2cd4^:rb_servo_server/config/stack_real.yaml
```

### 전수 대조 결과 (2026-09-03 시점, 전부 반영 완료)

| 항목 | 실기 (RB3 프로파일) | 이 리그 (이전) | 조치 |
|---|---|---|---|
| `max_linear_velocity_m_s` | 0.45 | 0.45 | 일치 |
| `max_linear_accel_m_s2` | 12.0 | 12.0 | 일치 |
| `max_linear_jerk_m_s3` | **2000** | 4000 | 맞춤 (`LIN_JERK`) |
| `max_angular_velocity_rad_s` | 0.90 | 0.90 | 일치 |
| `max_angular_accel_rad_s2` | 40.0 | 40.0 | 일치 |
| `max_angular_jerk_rad_s3` | **4000** | 8000 | 맞춤 (`ANG_JERK`) |
| `output_smd.enable` | **true** | 단 자체가 없음 | 이식, 기본 ON (`OUTPUT_SMD`) |
| `output_smd` nf/zeta/ff | 3.5 / 2.5 Hz, 1.0, on | — | 동일값 |
| `af_damping_beta_lin/ang` | **1.0 / 1.0** | 가속 FF 자체가 없음 | 중앙 2차차분 af 전달 |
| `corner_velocity_scale` | **0.25** | 없음 | 이식 (`CORNER_VELOCITY_SCALE`) |
| `corner_deadband_lin/ang` | 0.0003 m / 0.0005 rad | 없음 | 이식 |
| `ik.damping` | **0.02** (λ²=4e-4) | 1e-4 균일 | SVD 선택적 DLS로 교체 |
| `ik.damping_max` / `singular_region_eps` | **0.08 / 0.10** | 없음 | 이식 |
| `ik.max_iterations` / `min_iterations` | 100 / 1 | 3회 고정 | 이식 |
| `ik.position_tolerance_m` | 0.00002 | 없음 | 이식 |
| `ik.orientation_tolerance_rad` | 0.0002 | 없음 | 이식 |
| `ik.max_step_deg` | [2,2,2,3,3,4] | 없음 | 이식 |
| `dq_max_deg_s` | [170,170,170,240,240,320] | 동일 | 일치 |
| elbow `q_max_deg` | ±150 | ±150 | 일치 |
| `smoothing_window` | 1 (= 꺼짐) | 없음(=1) | 일치 |
| `discard_head_steps` | 0 | 등가 처리 | 일치 |
| `reserve_steps` (flank ±1) | 4 | ±1 | 일치 |
| `SPEED_SCALE` | 1.0 | 1.0 | 일치 |

**여기에 없는 실기 단**: 컨트롤박스 전송 지연. 실기는 순수 데드타임이다 — `BOX_DELAY_TICKS`로
모사 가능하나 기본 0 (아래 참조).

### 각 항목의 근거 (실기 config 주석에서)

- **jerk 절반 (2026-08-28)**: "dt 2 ms에서 4000 m/s^3은 한 틱에 가속을 8 m/s^2 흔들어 12 m/s^2
  천장에 ~3 ms만에 닿는다 — near-step acceleration, broadband, 11-13 Hz 모드 직격."
  가속(12/40)은 낮추지 않는다. 실기 feasibility 비용은 conv 93.1→90.0% 수준으로 작았다.
- **output SMD**: IK 직전 500 Hz 상시 2차 임계감쇠 트래커 + 저역통과된 속도 FF.
  `H(s) = wn^2 (3s + wn) / (s + wn)^3`. 속도 FF가 1차 지연을 상쇄하므로 평범한 LPF가 아니다 —
  0.5-3 Hz는 1.06-1.30배로 통과하고 ~4.3 Hz 위부터 깎는다. 청크 경계 상태가 없다
  (per-chunk FIR은 5.74 Hz 경계 콤을 만들어 실기에서 기각됐다).
  2026-07-31 하드웨어 수용시험: 13-20 Hz 89-109x / 10-13 Hz 22-31x / 5-10 Hz 4.5-5.1x 감쇠,
  과제대역 1-5 Hz 1.15x 유지, 경로편차 p50 0.42 / p95 3.5 mm.
- **가속 FF**: "af는 요구가 아니라 **경계조건**이다. 작게 잡으면 싸지는 게 아니라, 실제로
  움직이는 것보다 평평하게 도착하도록 jerk를 쓰게 만든다." 이 리그는 af=0이었다 —
  §19에서 고친 `target_velocity=0` 버그의 한 미분 위.
- **IK 감쇠**: `dq = V diag(σ/(σ²+λᵢ²)) Uᵀ err`, `λᵢ² = 0.02²` (+ `0.08²(1-(σ/0.10)²)` if σ<0.10).
  이 리그는 λ²=1e-4 균일이라 기본 4배, 특이영역에서 최대 **68배** 덜 감쇠했다.
  실측 σ_min p10이 **0.07~0.16**이라 이 팔은 eps=0.10 경계에 상주한다.
  σ=0.03에서 역이득이 리그 30.0 vs 실기 4.2 (**7배**).

### 함께 고친 리그 자체 버그 (실기에는 없던 것)

1. **팔로워가 절대 rotvec을 Ruckig 축 3-5에 넣고 있었다.** 이 태스크 자세는 |rotvec|≈3.05~3.14로
   π 경계에 상주하고 `mat_to_rotvec`이 w>=0으로 접으므로, 20초 에피소드당 12~59회 3-벡터가
   대척점으로 점프했다(실제 회전 스텝 중앙값 0.34~0.64도인데 raw 차분 p99가 357~360도).
   → 실기 `cartesian_chunk_follower.cpp`처럼 **쿼터니언 기준 R0 + 접선 좌표 + 매 knot 재선형화**
   (`FOLLOWER_ROT=tangent` 기본, `abs`로 원복). 합성 knot 단위시험: abs는 참값 4.09도 대비
   152도를 돌고 최종 자세오차 152도, tangent는 4.19도 / 0.016도.
2. **`ACTION_MODE=anchored` knot 생성에 회전 클램프가 없었다** (위치만 클램프). delta 분기와
   실기에는 항상 있었다. 추가 후 knot 회전 스텝 p99 10.2 → 1.9도.
3. **velproprio 이력과 청크 앵커가 한 리스트(`cmd_hist`)를 공유했다.** 경계마다
   `cmd_hist[-1] = anchor(FK(q_cmd))`로 덮어써서, 경계 다음 틱의 velproprio가
   `knot − FK(q_cmd)`가 됐다. 주입량이 정상 tick 변위의 **14~32%**, 주기는 청크 경계율
   (execute 4 → 7.35 Hz). 실기는 분리돼 있다 —
   `openpi_remote._record_command_pose_history`는 방출된 TcpPoseTarget만 append-only로 쌓고
   (홀드는 ZOH) velproprio는 시간 기준 1스텝 lookback + scale 정규화를 쓴다.
   → `VELPROPRIO_ANCHOR_OVERWRITE=0` 기본(=분리), 1이면 과거 동작.

### ⚠️ `tremor_um`으로 떨림 A/B를 판정하지 말 것

`tremor_um`(2차차분)과 knot-band(30 Hz)는 **f² 가중**이라 떨림이 실제로 사는 **3-15 Hz에 눈이
멀었다**. 실측 knot 30 Hz 대역 기여는 0.0%였고, 위 수정 전후로 `tremor_um`이 둘 다 ~110 um로
똑같이 나온다. 대신 쓸 지표:

- **자유공간 3-15 Hz 잔차 RMS** (0.2 s 3차 추세 제거 후; `cmd z > 20 mm`로 접촉 구간 배제)
- **knot 회전 churn (deg/s)** 과 knot 스텝 p99 — 실기 액션 청크가 6.8~8.6 deg/s, p99 0.78~1.03도
- **짝지은 단별 감쇠** (`refpre` → `ref`) — 같은 런·같은 시점이라 유일하게 교란이 없다

### 배제된 가설 (데이터로 반박, 재의심 금지)

- **q_ref ↔ q_actual 간극**: tau≈7.7 ms 1차 지연(= PhysX `kd/kp`), 에너지 95~99%가 10 Hz 아래,
  2-15 Hz 플랜트 전달비 0.90~1.07로 **투명**. 되먹임 경로도 없다(IK는 q_cmd 위에서 적분,
  proprio·앵커 모두 command 소스).
- **드라이브 강성**: 같은 관절 명령을 재생하며 kp를 1e7→1e6→1e5로 100배 바꿔도 링 피크가
  235.6/205.5 Hz에 고정. 링크 관성 0.002~0.075 kg·m²에서 드라이브 고유진동수는 1.8~11 kHz로
  2 ms 스텝이 못 푸는 대역이다. 진폭 ~12 um(오버뷰 렌더에서 0.008 px)이라 눈에 보일 수 없다.
- **동역학 전체**: `DRIVE_MODE=kinematic`으로 팔 관절을 매 스텝 **직후** q_ref로 강제해
  q_actual==q_ref(cmd jerk == meas jerk, plant gain 1.00)로 만들어도 자유공간 3-15 Hz wiggle이
  0.99 → 0.89 mm로 남았다. (스텝 **전에만** 쓰면 한 스텝 적분이 남아 키네마틱이 아니다.)
- **컨트롤박스**: 필터가 아니라 **순수 지연**이다. 배포 펌웨어 v8.7.3 + queue_sync(5) +
  `servo_alpha 10`(내부 LPF OFF) 실측 — `box delay = RBACK queue fill + 1 tick`, sent→ref
  8.17/8.03 tk, end-to-end 11.14/11.13 tk (22.3 ms). `servo_t2_sec`은 hold time이고
  `docs/servo_backend_contract.md`가 "**not UR-style lookahead**"라고 못박고 있다.
  `BOX_DELAY_TICKS=8`로 이식해 지연을 22.5~31.2 ms로 맞춰도 떨림은 불변(전달비 여전히 ~1).
- **`smoothing_window`**: 실기도 **1(꺼짐)** 이라 차이가 아니다.

### 남은 격차 (아직 안 맞춘 것)

- **정책 청크의 회전 요구량**: 실기 액션 청크가 6.8~8.6 deg/s(스텝 p99 0.78~1.03도, `ANG_V*dt`
  1.72도 초과 **0.0%**)인데 이 리그의 anchored 40k는 12.8~63.2 deg/s(p99 3.4~10.2도, 초과 3~41%).
  약 3배다. 체크포인트 차이인지(실기 로그는 delta v1) 렌더 OOD인지 리그 디코딩 결함인지 미판정.
  가르는 방법: **:8001에 delta 체크포인트를 띄우고 delta 모드로 같은 측정**. GPU를 프로덕션
  추론과 공유하므로 스케줄링 주의.
- `BOX_DELAY_TICKS` 기본 0. 실측 기반이라 8이 충실하지만, 과거 모든 평가와의 비교 가능성 때문에
  기본값 변경은 보류. 켤 때 std20 동등성 확인 후.

### 새 환경변수 (전부 실기 값이 기본)

| 노브 | 기본 | 의미 |
|---|---|---|
| `OUTPUT_SMD` | **1** | `output_smd.enable`. 0이면 이식 전 리그 |
| `SMD_NF_LINEAR_HZ` / `SMD_NF_ANGULAR_HZ` | 3.5 / 2.5 | SMD 고유주파수 |
| `SMD_DAMPING_RATIO` / `SMD_VELOCITY_FF` / `SMD_VELOCITY_FF_LPF_HZ` | 1.0 / 1 / 0 | |
| `IK_MODE` | **real** | `legacy`면 λ²=1e-4 균일 DLS |
| `IK_DAMPING` / `IK_DAMPING_MAX` / `IK_SINGULAR_EPS` | 0.02 / 0.08 / 0.10 | |
| `IK_MAX_ITERS` / `IK_POS_TOL_M` / `IK_ORI_TOL_RAD` | 100 / 2e-5 / 2e-4 | |
| `LIN_JERK` / `ANG_JERK` | 2000 / 4000 | |
| `AF_BETA_LIN` / `AF_BETA_ANG` | 1.0 / 1.0 | 가속 FF 감쇠 |
| `CORNER_VELOCITY_SCALE` / `CORNER_DEADBAND_LIN_M` / `CORNER_DEADBAND_ANG_RAD` | 0.25 / 3e-4 / 5e-4 | |
| `FOLLOWER_ROT` | **tangent** | `abs`면 절대 rotvec (버그 재현용) |
| `VELPROPRIO_ANCHOR_OVERWRITE` | **0** | 1이면 앵커가 velproprio 이력을 덮어씀 |
| `BOX_DELAY_TICKS` | 0 | 컨트롤박스 FIFO 지연 [2 ms 틱]. 실측 8 |
| `DRIVE_MODE` | drive | `kinematic`은 진단 전용(파지가 죽는다) |

### 측정 결과 (n=2, 롤아웃이 갈라지므로 크기는 참고용)

자유공간 3-15 Hz **실측 잔차**가 이식 전 대비 **좌 1.136 → 0.513 mm (-54.8%) / 우 1.214 → 0.817 mm
(-32.7%)**. 관절 wiggle은 좌 -65.2%인데 우가 +33.9%로 갈리는데, n=2에 롤아웃이 갈라지므로
관절 쪽 부호는 아직 못 믿는다. **짝지은** SMD 감쇠(같은 런·같은 시점 `refpre`→`ref`)만이
교란이 없는 수치이고 **2.0~2.4배**로 4개 arm-run 전부 일관했다.
IK는 20 um 허용오차 내 **100% 수렴**(잔차 p50 0.7~3.5 um).

**과제 성적은 내려갔다**: correct 3→2, grasp 4→2, lifted 3→3 (중간 단계인 corner 가드 이전
구성은 1/2/2였다). n=2라 판정은 아니지만, **원칙상 성공률을 이유로 되돌리지 않는다.** SMD lag가 이 리그에서 p50 0.94~2.97 mm로
실기(0.84 quiet / 1.22 vibrating)의 1.1~3.5배인 것이 후보다 — lag는 `3a/wn²`이라 리그가 가속을
더 요구하는 만큼 더 문다. 즉 이것도 "리그가 실기보다 거친 명령을 받고 있다"는 위 미판정 항목의
증상이다. 확정은 **std20**으로.

산출물: `outputs/eval/compare_{rotfix,boxdelay,kinematic,fx,parity}_{00,01}.mp4`,
diag npz(`TREMOR_DIAG=1`)에 `qact`/`qdact`/`knot`/`ref`/`refpre` 채널 추가.

## 22. v15 PLA+TPU 팁 교체 — 1단계 형상/외관 (2026-09-04)

운영자: 실기 팁을 **PLA 스파인 + TPU 95A 블레이드(#F3E600 노랑)** 로 교체했고, **`data_v2`가
그 팁으로 수집됐다.** 즉 옛 통짜 검정 팁은 보수적 기본값이 아니라 **하드웨어와도 학습분포와도
어긋난 값**이었다 — 그것도 손목캠이 항상 보고 볼트에 유일하게 닿는 부품에서.

운영자 결정: **① 형상만(강체 유지) 먼저 커밋 → ② 접촉 파라미터로 95A 컴플라이언스.**
섞으면 형상 효과와 강성 효과를 못 가른다. 이 절은 ①이다.

### 자산은 robotics_lab에서 소비한다 (직접 유도 금지)

메시는 `robotics_lab/.../rb5_850e/visual/tool/pika_finger_{left,right}_{pla,tpu}.STL`,
그쪽 `tools/make_pika_tool_meshes.py`가 tool 프레임으로 생성한 것. rb5 경로가 맞다 —
Pika 툴은 양 팔 공통이고 이 메시들은 rb3 자산과 같은 attachment_site 프레임이다.

**직접 유도했다가 틀렸다.** v15 CAD만으로 tip 프레임은 정확히 복원했으나(같은
−58.990/−19.055/−44.190, Z=−34.39 seat 평면으로 확인) tip X=0을 **공장 핑거 접촉면**에
앵커링했다. 측당 ~2mm 오차. 징후는 보였는데 무시했다 — "스파인이 뒤쪽으로 1.99mm 튀어나옴".
뒤로 튀어나온 게 아니라 **부품이 다른 데 앉는다**(스파인 foot이 Gripper 핑거가 아니라
pika SENSE arm에서 유도됐기 때문).

### 조는 이제 CAD 포즈가 아니라 실측이다

robotics_lab이 캐리지 데이텀(M2 4개 + rail 나사)으로 1.991mm 인보드 시프트를 잡아 90.35mm를
예측했으나, **실측이 뒤집었다**:

> 2026-09-04 버니어, TPU 면 사이 **full-open ≈ 98mm**, 조는 접촉까지 닫힘.
> 90.35(시프트 적용)도, 94.33(미적용)도, 94.04(공장)도 아니다.

셋 다 공유한 미검증 전제: **벤더 CAD는 핑거를 open stop에 그리지 않는다**(측당 ~3.8mm 안쪽).
그리고 옛 travel 0.047도 실측이 아니라 **그 CAD 포즈가 0으로 닫히도록 고른 값**이었다.
미측정 숫자 둘이 서로 맞장구치고 있었던 것.

**⚠️ 이 리그의 귀결: `FINGER_TRAVEL_M` 0.047 → 0.049.** 스트로크는 실측 gap에서 따라온다
(닫힘에서 gap 0 ⇒ 98/2). `eval_closed_loop.py`의 조 모델·grip proprio·`achieved_gap_mm`가
전부 이 상수다. 0.047이면 sim 조가 측당 2mm 덜 닫히고, 18.4mm 볼트 머리에서 그건 파지와
헛발의 차이다. URDF prismatic 한계가 ±0.05라 0.049는 관절 손 안 대고 들어간다.
(`achieved_gap_mm`에 리터럴 `0.047`이 하나 더 박혀 있었다 — 상수로 교체했다.)

빌드는 두 숫자를 하드코딩하지 않는다. 메시에서 접촉면을 읽어 나머지를 유도하므로,
robotics_lab이 어디에 안착하든 **스크립트 재실행으로 따라온다.**

### 콜리전 — 여기가 교체의 실속이다

옛 콜라이더는 핑거 전체의 **단일 convex hull**(119,262 vs 실제 31,039 mm³ = 3.8배)이라
접촉면이 평면 하나였다. 재설계의 **2.0mm 유지 아치가 통째로 지워진다.**

블레이드는 **SDF 콜리전**(res 256, ~0.31mm/voxel), 스파인은 hull(자기 앞면이 접촉면보다
7.9mm 뒤라 hull이 물체 닿는 곳에 살을 못 붙임). **coacd 분해는 기본값에서 뺐다** —
같은 메시·같은 seed에 24/25/27/29 파트로 갈리고 한 번은 실제 표면보다 0.44mm 튀어나왔다.
재빌드마다 조가 움직이는 콜라이더는 이 리그에 설 자리가 없다. `TIP_COLLISION=decomp`로 A/B만.

**정지 원기둥 스톨 실측** (`verify_tip_v15.py`, 조를 완전히 닫으라고 명령):

| | 옛 팁 (평면 hull) | v15 (SDF, 아치) |
|---|---|---|
| full-open gap | 94.045 mm | **97.998** (실측 ~98) |
| M12 머리 ⌀18.4 안착 | 0.10 mm/side | **1.99 mm/side** |
| M12 샹크 ⌀12 안착 | 0.10 mm/side | **2.04 mm/side** |

설계 아치 깊이가 2.0mm다. **유지 피처가 이제 sim에 존재하고, 설계값 그대로 측정된다.**
옛 팁은 볼트에 자리를 0.1mm밖에 안 줬다 — 평면이니까.

### 질량

핑거 링크는 `PhysicsRigidBodyAPI`만 있고 질량이 없어 PhysX가 **3.8배 부푼 hull에서** 유도하고
있었다(119.3g). 콜라이더를 바꾸면 조용히 반토막 나므로 **명시적으로 박았다**: 55.74 g/finger
(PLA 35,862.7mm³ × 1240 + TPU 9,390mm³ × 1200). 솔리드 밀도 기준 —
**실제 인필률을 주면 `TIP_DENSITY_PLA`/`TIP_DENSITY_TPU`로 재빌드**한다.

### 손목캠에 핑거가 보인다 (§17 미해결 항목 해소)

§17의 "손목캠 화면에서 핑거 이동이 안 보인다"가 풀렸다. 새 팁은 §14가 실기에 대해 예측한
그대로 **화면 하단 좌우에 크게 잡힌다**(`outputs/tip_v15_wrist_g*.png`). 노란색이 정책 입력에
실제로 들어간다는 뜻이고, 이게 `data_v2`와의 시각 정합이다.

### 메시는 원본 그대로 쓴다 (운영자 지시)

`PLA_spine_v15.STL`은 깨끗한 솔리드가 아니다 — 외부 수정본 불리언 잔여물로 **5개 컴포넌트**,
foot boss에 **비다양체 에지 10개**, 부호 반대로 거의 상쇄되는 중복 평면 슬랩 쌍 + 뒤집힌
내부 공동 셸. 초기 구현은 이걸 제거했으나 **원복했다**: 운영자 지시이고, 근거가 타당하다 —
**이 파일이 실제로 출력된 파일이다.** 슬라이서가 이 삼각형들을 그대로 먹었으므로 로봇에 달린
물리적 부품은 잔여물 포함 이 메시가 의미하는 그것이다. 여기서 다듬으면 모델링 대상과 다른
부품이 된다.

"그대로 쓴다"도 안전한지는 확인하고 받았다:

| | 원본 | 정리본 | 판정 |
|---|---|---|---|
| convex hull 부피 | 91,684.911 | 91,684.911 | **비트 동일** → 콜라이더 무영향 |
| 메시 부피 | 35,909.5 | 35,862.7 | 0.13% → 질량 무의미 |
| 잔여물 위치 | 슬랩 쌍은 내부 벽면에 밀착(0.00mm), 뒤집힌 셸은 1.75~2.86mm 내부 | | 외곽 실루엣·렌더 무영향 |

즉 잔여물은 콜라이더에도, 외곽에도, 질량에도 못 닿는다. 다만 **빌드가 매번 컴포넌트 목록을
출력**하므로 파일 상태가 구전이 아니라 기록으로 남는다.

### 질량은 상한값이다

인필률을 모른다(운영자). 그래서 쓰는 밀도는 **솔리드 재료**이고 55.8 g/finger는 **상한**이다 —
실제 부품은 sparse 인필이 덜어낸 만큼 가볍다. 그래도 리그가 쓰던 119.3g(솔리드보다도 2배 이상)
보다는 훨씬 가깝다. 확정하려면 추측이 아니라 측정이 필요하다: robotics_lab이 이 교체 후
`ft identify` 재실행을 빚지고 있고, 그 값을 내려받으면 된다. 노브는 `TIP_DENSITY_PLA/TPU`.

### 기본값을 v15로 뒤집었다

`PIKA_TIP=v15`(기본) / `orig`. OUTPUT_SMD를 뒤집었을 때와 같은 논리다 — 이 리그는 로봇을
따라간다. **다만 기존 검증 수치는 전부 옛 팁에서 나왔다**: 자산·travel·콜라이더가 전부
움직였으므로 std20 기준선은 v15에서 다시 떠야 한다. `PIKA_TIP=orig`가 A/B 파트너다.

### 남은 것

- **②단계**: TPU 95A 접촉 파라미터. 지금 블레이드는 강체다.
- 질량 상한 55.8g → robotics_lab `ft identify` 실측이 오면 `TIP_DENSITY_*`로 재빌드.
- `make_articulated_urdf.py`는 이미 죽어 있다 — 참조하는 rb3 tool 메시가 사라졌다(rb5에만 있음).
  0.047도 거기 남아 있으나, 되살릴 때 rb5 경로와 함께 고칠 것.
- v15에서 std20 재기준선.

### 손목캠 확인 산출물 (2026-09-04)

정책이 실제로 받는 프레임으로 확인했다 (`EVAL_DUMP_OBS`, :8002 boltv2ph3_40k, 20초 롤아웃):

- `outputs/tipv15_handcam_sheet.png` — 좌/우 팔 × t=0~540 컨택트 시트. 전 구간에서 노란 팁이
  화면 하단에 잡히고 조 개폐가 프레임에 보인다.
- `outputs/tipv15_vs_orig_handcam.png` — 같은 동결 씬 t=0에서 `orig` vs `v15` 나란히.
  옛 팁은 특징 없는 검정 슬래브, v15는 hex 필드와 아치 윤곽이 보이는 노란 블레이드이고
  조 폭도 94.04 → 98.0으로 더 벌어져 있다.
- `outputs/eval/tipv15_hand_aligned_00.mp4` — 오버뷰 영상 (598 프레임).

`blank_obs 0`으로 유효 런이다.

## 23. sim이 "엉터리"였던 이유 = 액션 디코딩 계약 불일치 (2026-09-04)

운영자: ":8002 모델은 실기에서 가끔 성공하는데 sim에서는 아예 엉터리다. 팁은 새로 장착한
걸로 학습했으니 OOD는 아닐 텐데."

**OOD가 아니었다. 리그가 앵커 상대 웨이포인트를 체인 델타로 디코딩하고 있었다.**

### 증거

:8002가 서빙 중인 것은 `pi05_pika_umi_boltv2_anchAB_ph3_h24_40k`이고 openpi config가
**`action_mode="anchored"`** 다. 그런데 리그는 `ACTION_MODE`의 기본값인 `delta`로 돌았다.
같은 씬·같은 seed·같은 정책으로 20초 A/B:

| | `delta` (틀림) | `anchored` (맞음) |
|---|---|---|
| close 시 **dz p50** | **+270.6 mm** | **−5.9 mm** |
| close 시 dxy p50 | 59.6 mm | 111.6 mm |
| close 횟수 | 12 | 3 |

**dz +270mm** = 조를 볼트보다 27cm 위 허공에서 닫고 있었다. anchored로 디코딩하면 볼트
높이에서 닫는다. 손목캠 컨택트 시트에서도 delta 런은 t=180 이후 화면이 빈 테이블과 박스
벽뿐이었다 — 팔이 위로 떠나가고 있었던 것.

원리: anchored 청크의 24행은 **각각 앵커에서의 독립 오프셋**이다. delta 분기는 이걸
`cur_p = cur_p + cur_R·dl`로 **누적**하므로, 청크마다 웨이포인트 24개어치 오프셋이 쌓인다.
execute 4로 매 4틱 재앵커링해도 평균 오프셋 방향으로 계속 끌려간다.

### 왜 조용히 통과했나

이 런은 **건강해 보였다**: `blank_obs 0`, close 이벤트 기록됨, mp4 정상, 추론 지연 정상.
§20의 교훈("관측 경로의 폴백은 소리를 내야 한다")의 액션 경로 판이다. `ACTION_MODE`는 이쪽의
플래그이고 저쪽의 학습 선택인데 **둘이 맞는지 아무도 안 보고 있었다.**

### 가드 (구현됨)

openpi 웹소켓 metadata에는 `{action_horizon, action_dim}`뿐이라 서버에 물어볼 수 없다.
대신 **체크포인트 자신이 답을 갖고 있다** — openpi가 norm stats 옆에 학습 데이터셋 이름을
써 두고, 그 이름이 계약을 인코딩한다:

```
..._tcp_anchored_...  -> anchored      ..._tcp_gripabs_...  -> delta
```

이 머신의 전 체크포인트에서 확인: anchAB/anchored/boltv2/boltv2ph3 = anchored,
pad/v2_nolang/velgrip_real = gripabs. `_checkpoint_contract()`가 서빙 프로세스의
`--policy.dir`에서 이걸 읽고, **`ACTION_MODE`와 다르면 시작 시 abort**한다.
계약은 summary provenance(`checkpoint_contract`)에도 남는다.

### 집 규칙: 계약 문제는 경고가 아니라 종료다 (운영자, 2026-09-04)

**이 부류의 실패는 전부 시끄러운 에러 + 종료로 만든다.** 근거는 이번 사고 자체다 — 계약이
틀린 런은 `blank_obs 0`, close 이벤트 기록, mp4 정상, 지연 정상으로 **건강해 보이는 요약**을
남긴다. 200줄짜리 시작 로그 속의 WARNING은 침묵과 구분되지 않고, **숫자가 로그보다 오래 산다.**

`_abort_unverified()`로 통일했고 다음이 전부 종료 대상이다:

| 경로 | 이전 | 지금 |
|---|---|---|
| `ACTION_MODE` ↔ 체크포인트 학습 모드 불일치 | 없음(무검사) | **ABORT** |
| 체크포인트 계약을 못 읽음 | WARNING 후 진행 | **ABORT** |
| anchored인데 norm stats 없음 (RTC가 vanilla로 강등) | WARNING 후 진행 | **ABORT** |
| 서버 `action_horizon` ≠ 리그 `ACTION_HORIZON` | 무검사 | **ABORT** |
| 서버 `action_dim` < 14 | 무검사 | **ABORT** |
| 손목캠 빈 프레임 (**첫 관측 이후**) | 계수 후 요약에 `!! INVALID RUN` 표시, 채점은 계속 | **즉시 ABORT** |

마지막 줄이 특히 중요하다. 예전엔 첫 관측만 막았고 이후에 검은 프레임이 들어오면 남은 에피소드
내내 검은 이미지로 정책을 채점한 뒤 끝에 표시만 붙였다 — 그 표시를 읽을 때쯤이면 mp4와 점수가
이미 존재하고 멀쩡해 보인다.

### ⚠️ `raise SystemExit`은 이 프로세스를 끝내지 못한다

가드를 넣고 시험하다 발견했다. 계약 게이트가 **t+16초에 메시지를 찍고, 프로세스는 t+400초에
외부 timeout이 죽일 때까지 그대로 앉아 있었다.** Isaac의 `SimulationApp`이 non-daemon 스레드를
붙들고 있어서 처리되지 않은 `SystemExit`이 `main()`을 풀고 나와도 인터프리터가 못 나간다.
**찍고 멈추는 가드는 절반짜리다** — `run_std20.sh`는 `PIPESTATUS`를 보는데 아무것도 못 보고,
큐로 돌리는 스윕은 다음으로 넘어가지 않고 멈춘다.

그래서 `_die()`로 통일했다: 양쪽 스트림 flush → `os._exit(1)`. 앱 생성 이후의 모든 치명 경로
(계약 게이트·GRIP_MAXF·FINGER_FRICTION·scene-states 누락·빈 손목 프레임)가 이걸 쓴다.

**`SimulationApp.close()`를 먼저 부르면 안 된다.** Isaac이 `--/app/fastShutdown=True`로 돌아서
`close()`가 프로세스를 **상태 0으로** 직접 끝내버린다 — 첫 버전이 정확히 그랬고, abort를 찍고
18초에 종료하면서 **성공을 보고했다**. 런타임만 짧아진 같은 조용한 통과다. GPU 컨텍스트는
프로세스가 죽으면 커널이 회수하므로 close()가 지켜줄 것 중에 틀린 종료코드와 바꿀 만한 건 없다.
실측: 수정 후 `exit=1`, 17초.

탈출구는 **`ALLOW_UNVERIFIED_CONTRACT=1` 하나뿐**이고 명시적으로 타이핑해야 한다 —
STATE_MODE/ACTION_MODE는 애초에 인터페이스 실험용으로 만든 노브라(§18) 막기만 하는 게이트는
틀린 종류의 엄격함이지만, 기본값으로 새어나가면 이번 사고가 반복된다. 켜면 로그에
`!! UNVERIFIED CONTRACT ALLOWED ... This run is an experiment, not a score.`가 찍힌다.

### 함께 막은 지뢰: RTC norm stats 하드코딩

`RTC_NORM_STATS` 기본값이 **특정 옛 체크포인트의 경로로 하드코딩**돼 있었다. 다른 anchored
체크포인트를 서빙하면 그 파일이 존재하기 때문에 "없음" 경고가 안 뜨고 **틀린 정규화를 조용히
로드**한다. anchored RTC 재앵커링은 SE(3) 연산이라 이 모델의 action q01/q99가 필요하다
(§ dfb897a). 이제 **서빙 중인 체크포인트에서** 읽는다.

### 아직 안 끝난 것

`anchored`로 고쳐도 **dxy p50 111.6mm** 로 측면 조준이 크다(n=3 close라 표본은 약하다).
높이 문제는 디코딩이 전부였지만 측면 정확도는 별개 질문으로 남아 있다. 실기가 "가끔 성공"
하는 모델이라면 sim의 남은 격차는 여기서 봐야 한다 — §21의 미판정 항목(정책 청크 회전
요구량이 실기 대비 3배)과 같은 줄에 놓고 볼 것.

## 24. std20 3종 비교 — v15 팁 + anchored 계약 (2026-09-04)

§23의 계약 게이트를 넣은 뒤 처음 뜬 정식 기준선. 세 체크포인트 전부 **`blank_obs 0`**,
계약 `anchored` 검증됨, `PIKA_TIP=v15` / `FINGER_TRAVEL_M 0.049` / 동결 씬 / 비디오 ON.
런당 약 2시간, 순차 (`scripts/run_std20_three.sh`).

| 모델 | 포트 | 정답 | 오배치 | ≥1 ep | close | grasp | g% | **linked** | dxy p50 | **볼트높이 close** |
|---|---|---|---|---|---|---|---|---|---|---|
| `boltv2_ph3` (신규 v2 + phase3) | 8002 | **5** | 1 | 5/20 | 161 | 11 | 6.8% | **7** | 68.5 | **64%** |
| `boltv2` (신규 v2) | 8001 | **5** | 2 | 5/20 | 181 | 12 | 6.6% | 5 | 67.5 | 40% |
| `anchAB_olddata` (구 캠페인) | 8003 | 3 | 0 | 2/20 | **523** | 12 | 2.3% | 2 | 62.1 | 30% |

### ⚠️ `dz_p50`을 그대로 읽지 말 것 — 양봉이다

close의 dz는 **볼트 높이(진짜 파지 시도)와 허공(이송 중·박스 위, 750mm까지)** 두 봉우리로
갈린다. 중앙값은 두 봉우리의 비율만 반영해서 뒤집힌다 — boltv2_ph3 −3.7mm vs boltv2 +170mm는
모델 정밀도 차이가 **아니라** 허공 close 비율 차이(36% vs 60%)다.

쓸 지표는 **`|dz| < 30mm`인 close의 비율**("볼트높이 close")이다. `t1_report.py`가 아직
`dz_p50`을 싣고 있으니 개선 대상.

### 읽히는 것

- **신규 v2 캠페인이 구 캠페인을 이긴다**: 정답 5/5 vs 3, ≥1 성공 에피소드 5/5 vs 2,
  grasp-linked 7/5 vs 2. 다만 **구 모델은 팁 OOD 교락**이 있다(아래).
- **phase-3 데시메이션은 과제 점수를 안 올리고 헛발질을 줄인다**: 정답은 5로 동률인데
  오배치 1 vs 2, grasp-linked 7 vs 5, **볼트높이 close 64% vs 40%**. 같은 컴퓨트에서
  위상 다양성만 준 arm이므로 "더 깔끔해진다"가 이 실험의 답으로 보인다.
- **`anchAB`는 조를 523회 닫는다** — 다른 둘의 3.2배인데 30%만 볼트 높이다. 파지율 2.3%.
  점수가 낮은 게 아니라 **거동이 다르다**(허공에서 계속 여닫는다).
- 타깃 스위치/에피소드 중앙값 33 / 36 / 52.5 — 구 모델이 확연히 산만하다.

### margin 버킷별 grasp율 (§20의 사전 못박은 판정축)

| 모델 | 0–20mm | 20–50mm | 50–100mm | >100mm |
|---|---|---|---|---|
| `boltv2_ph3` | 3.7% (54) | 3.4% (58) | **15.8%** (38) | 9.1% (11) |
| `boltv2` | 3.8% (53) | 6.2% (65) | 9.4% (32) | 9.7% (31) |
| `anchAB_olddata` | 1.2% (169) | 3.1% (223) | 2.6% (76) | 1.8% (55) |

**신규 두 모델 모두 0–20mm 버킷이 최저**(3.7 / 3.8%)이고 margin이 커지면 오른다 —
§20 규칙의 "작은 margin에서 꺾임 = 모드 문제" 서명이 **두 arm에서 재현**됐다. 다만 상승이
단조롭지 않고(>100mm에서 다시 하락) 버킷 n이 11~65라 **시사이지 확정이 아니다**.
`anchAB`는 전 구간 평평하고 낮다 — 모드 문제가 아니라 그냥 못한다(또는 OOD).

### ⚠️ `anchAB` 해석의 교락: 팁 OOD

`anchAB`는 구 캠페인(프린트 팁 이전) 학습이고 sim은 v15 노란 팁을 렌더한다. **이 점수로
"옛 레시피가 나쁘다"와 "옛 모델이 팁 OOD다"를 가를 수 없다.** 배포 조건 자체는 정직하다 —
실기도 어느 체크포인트를 서빙하든 v15를 달고 있다. 가르려면 `anchAB`만
`PIKA_TIP=orig`로 재실행해 자기 v15 점수와 비교할 것(약 2시간).

### 남은 것

- `t1_report.py`에 볼트높이 close 비율 추가, `dz_p50` 강등.
- `anchAB` × `PIKA_TIP=orig` 대조군 (위 교락 분리용).
- §21의 미판정 항목(정책 청크 회전 요구량 실기 대비 3배)은 이 런들로도 아직 안 풀렸다.
