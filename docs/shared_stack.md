# Isaac에서 robotics_lab 공통 제어 스택 실행

2026-09-05 구현. 양팔 RB5-850E + Pika v15, 로컬 pi0.5(:8001).

2026-09-05 색상 수정: 초기 shared-stack reset이 볼트 visual material 할당을 빠뜨려
모든 볼트가 기본 회색으로 보였다. `shared_policy_02/03/04` 및
`shared_pi05_8001_20260905_210149`는 이 문제가 있는 녹화이며, 모델의 색상 구분이나
작업 성능 평가 근거로 사용하면 안 된다. 현재는 동결 씬의 `colors`를 사용해 기존
회색/검정 재질을 머리·몸통에 연결하고, 색상별 개수와 40개 실제 binding을 검증한다.
결과는 `scene_materials.json`과 `summary.json.scene`에 남긴다.

2026-09-06 Real 설정 대조: shared 설정에 과거 gripper close bias L=2/R=6이
남아 있었으나, 사용자가 확인한 Real 성공 실행과 현재 robotics_lab 기본값은 0/0이다.
`config/shared_stack.json`의 양손 bias를 0으로 수정했다. command gripper proprio는
보정 후 명령을 다음 추론에 되먹임하므로, bias는 실제 개방 폭뿐 아니라 정책의
후속 출력과 닫힘 시점에도 영향을 준다. `foam_color_8001/8002/8003_20260906_0609`
영상은 모두 옛 2/6 설정이며 Real 조건의 모델 성능 비교로 해석하면 안 된다.
같은 보정 폼/seed 100에서 bias만 0/0으로 바꾼 8001의 30초 실행은 왼팔로
회색 bolt_04를 들어 회색 박스 안에 놓았다. 오른팔 검정 볼트의 완료나 세 모델의
일반 성공률을 검증한 결과는 아니다. 상세 근거:
`outputs/diagnostics/foam_grasp_gap_20260906/report.md`.

최신 결과(2026-09-06): bias 0/0, 동일 foam seed 100, 모델별 30초에서
8001은 회색 1개, 8002는 안착 0개, 8003은 회색/검정 각 1개를 올바른 박스에 놓았다.
8002는 들어 올리지만 놓치는 동작이 있었다. 세 제어기 실행 모두 fault 없이 완료했다.
8001은 직전 bias 수정 검증 실행을 비교에 재사용했다. 한 장면의 한 번 실행 결과다.
[Git에 보존한 영상·검증·진단](results/20260906/README.md)과
[현재 자동 검증 결과](validation_20260906.md)를 참고한다.
아래 초기 개발 실행 및 bare-table fault 기록은 당시 조건의 이력이다.

```text
Isaac wrist RGB + measured joints / gripper / tool-joint F/T
  → policy_runner.main.run + OpenpiRemoteActionSource
  → 기존 command / chunk_overlay.v3 인코더
  → pipe RPC (실기 UDP 포트를 열지 않음)
  → C++ CommandServer + ChunkFrameReceiver
  → DualArmServoLoop: delta_preview → FollowerOutputSmd → 기존 force overlay → Pinocchio IK
  → 기존 관절 제한 / 속도·가속도 / 충돌 / 작업공간 / fault 처리
  → q_target_deg → radians → PhysX articulation drive
```

`eval_closed_loop.py --shared-stack`는 씬·카메라를 만든 직후 이 경로로 진입한다.
기존 Python Ruckig/IK 제어 객체와 기존 policy client는 생성하지 않는다.
기존 평가 경로는 `--shared-stack`이 없으면 유지된다.

## 실행

```bash
cd /home/plaif/workspace/robotics_lab
cmake -S rb_servo_server -B rb_servo_server/build
cmake --build rb_servo_server/build -j6

cd /home/plaif/workspace/simulation
scripts/run_shared_stack.sh --episode-sec 5 --tag my_shared_run \
  --scene-states assets/scene_states40_aligned_rb5_foam.json

# 정책 추론 없이 plant/시간/카메라 연결만 검증
scripts/run_shared_stack.sh --shared-hold --episode-sec 1 --no-video --tag my_hold

# 양팔·양쪽 손가락에 알려진 ±힘/±토크를 가해 센서 경로 검증 (정책/서버 미접속)
.venv-isaac/bin/python scripts/validate_shared_contact.py --tag wrench_check

# 현재 버전의 chunk_packets.jsonl을 가진 실행의 입력 재생; F/T/IK/안전은 계속 작동
scripts/run_shared_stack.sh --shared-replay outputs/shared_stack/my_shared_run \
  --episode-sec 5 --no-video --tag my_contact_replay

.venv-isaac/bin/python scripts/analyze_shared_stack.py outputs/shared_stack/my_shared_run
```

태그 디렉터리가 이미 있으면 덮어쓰지 않고 거부한다.
런처가 지정하는 기본 실행은 동결 씬의 5초이며 태그를 생략하면 시각으로 생성한다.
2026-09-06부터 기본 작업면과 동결 씬은 아래 폼 패드 프로필이다.
기본 policy 주소는
127.0.0.1:8001이며 `--host`/`--port`로 선택한다. 현재 런처는 로컬 서버의 체크포인트
설정에서 anchored 계약을 확인하며, 확인할 수 없거나 다르면 시작하지 않는다.
에피소드는 한 번씩 실행한다. 이 경로의 policy 설정은 `config/shared_stack.json`에서
관리한다. `--rtc` 등 기존 평가기 제어 옵션을 섞지 않는다.

Isaac venv의 추가 의존성: `h5py`, `websockets>=15,<16` 및 로컬 openpi-client의
PYTHONPATH. 이 PC에는 설치 완료. 설치가 필요하면:

```bash
uv pip install --python .venv-isaac/bin/python h5py 'websockets>=15,<16'
```

## 시간과 상태 계약

- 물리와 C++는 정확히 2 ms씩 진행한다. 양팔 측정값을 함께 보낸다.
- `init` 이후 `step.seq`는 정확히 +1, `time_ns`는 정확히 +2,000,000이다.
  반복·누락·비유한 관절값·잘못된 길이는 오류 종료한다.
- C++ command freshness, lease, follower 및 카메라 상태 판단은 같은 simulation
  monotonic clock을 사용한다. 일반 실기 실행에서는 기존 wall monotonic clock이다.
- policy_dt=33.4 ms와 카메라 30 Hz는 정수 17 substep으로 올림하지 않는다.
  카메라 렌더는 `delta_time=0.0`이고, 렌더 전후 물리 tick이 같아야 한다.
- 모델 추론은 기존 비동기 worker에서 실행한다. 응답은 측정한 서비스 지연이
  simulation time에서도 지난 뒤에만 공개한다. 느린 렌더링이 추론 지연을 지우지 않는다.
- `q_actual`/`dq_actual`은 PhysX 측정값, `q_ref`는 마지막으로 보낸 drive 목표이다.
  관절 위치 강제 설정은 reset에만 사용한다. 그리퍼도 PhysX finger joint의 측정값을
  0–100%로 변환해 C++의 기존 충돌 외곽 보간에 전달한다.
- 종료 시 runner가 Hold와 lease release를 보내고 마지막 C++ tick이 이를 소비한다.
- F/T 사용 시 정책 전에 1초 물리 안정화, 부호/중력 residual 검사, 기존 250샘플
  tare를 위한 0.6초가 있다. `episode_start_time_ns` 이후가 정책 시간이다.
  video/CSV에는 0.6초 tare 구간이 포함되며 `policy_elapsed_sec`에는 제외된다.

## 공유하는 설정과 명시적인 차이

`robotics_lab/rb_servo_server/config/stack_real.yaml`을 읽어 현재 follower/IK/안전 값을
그대로 사용한다. 파일을 수정하거나 실기 백엔드를 생성하지 않는다.
`rb_isaac_bridge`는 `BackendFactory`, box DH probe, network command receiver,
실물 그리퍼 드라이버를 실행하지 않는 별도 실행 파일이다. wire 상태의 기존
`observed_backend=mock`은 하드웨어 없는 경로라는 호환 라벨이며 실제 plant는
`plant=isaac_physx`로 구별한다. 새 `backend_type`/`run_mode` 별칭은 추가하지 않았다.

현재 차이는 다음과 같다.

| 항목 | Isaac 경로 |
|---|---|
| 서보 I/O | 외부 step의 direct I/O. 실제 컨트롤박스 worker/queue_sync/ServoJ 내부 응답은 모사하지 않음 |
| 충돌 검사 | 동일한 CollisionMonitor 계산을 매 tick 동기로 수행. 실기의 비동기 계산 지연은 미모사 |
| 힘 제어 | 실제 PhysX tool-joint wrench → 기존 FtPipeline/tare/coverage/force gate/overlay. 잡음 기반 전기센서 연결 검사 대신 2 ms 샘플 seq/time 검증 |
| 기구학 | Isaac USD와 일치하는 nominal URDF. 실기의 box DH 보정과는 별도 |
| PhysX drive | JSON의 arm stiffness/damping/최대 토크, finger stiffness/damping/max force. TPU 충돌 mesh의 compliant contact |
| 그리퍼 | 같은 정책의 gripper target을 실제 simulated finger drive로 전달. 실물 모터 내부 응답은 미식별 |
| RT·로그 | 실기 RT affinity 대신 외부 clock. C++ 계산 시간의 일부 기존 telemetry는 simulation clock상 0 |

arm stiffness=1e7, damping=1e5는 기존 sim의 기준값을 유지했다. finger는 유한 힘
5 N, stiffness=1e4, damping=100을 명시한다. 이 값들은 현재 **측정으로 식별된
RB5 모터 모델이라고 주장하지 않는다**. 알고리즘을 공유한 이후 plant를 식별하기 위한 출발점이다.
JSON의 policy 설정은 20260904_235947 boltv2_plain_40k 실기 실행을 기준으로
anchored/H24/execute4/runway4/crossfade0/command anchor/velocity_grip/command
proprio/fixed_step/RTC off/RGB only를 사용한다.

### F/T 및 접촉 모델 (2026-09-05 추가)

`config/shared_stack.json.contact`가 Sim 설정의 명시적 출처이다. 힘 제어 법칙과
35 mm actual-lead/40 mm force-deviation 등 공통 제한은 원본 stack을 그대로 쓴다.
`contact.force_sensor=false`는 F/T 없는 A/B 실험용이며 실기 YAML은 바꾸지 않는다.

- 전체 도구 하중이 통과하는 `attachment_site_joint`의 incoming reaction을 반전한다.
  `ft_sensor_measurement` leaf를 읽거나 접촉력 크기를 임의로 잘라 센서값을 만들지 않는다.
- source `force_torque`의 mass/COM에 맞춰 **모든 하류 rigid body의 합**을 조정한다.
  자동 계산된 tool body 질량이 약 1.619 kg이었던 것을 포함한 총 하중을
  왼쪽 0.7912 kg / 오른쪽 0.7822 kg으로 맞춘다. COM도 같은 source 값을 사용한다.
  tool inertia는 형상 관성을 질량비로 조정한 근사이며 하드웨어 식별값이 아니다.
- flange 기준 wrench를 source SRO로 옮기고 실제 전기 채널 축으로 인코딩한다.
  공통 FtPipeline이 이를 원래대로 해석한다. 250샘플 tare 이전 residual 검사로
  중력·부호 오류가 tare에 가려지는 것을 막는다.
- TPU visual은 충돌 mesh의 형제가므로 재질을 visual에 연결하면 물리에 적용되지 않는다.
  실제 `finger_tpu_sdf` 네 곳에 물리 재질을 연결하고 binding을 검증한다.
  PLA와 그리퍼 본체는 rigid contact를 유지한다.
- 기본 TPU stiffness=15000 N/m, damping=30 Ns/m, 관절 최대 토크
  [200,200,200,100,100,100] Nm는 **명시적 임시 Sim 파라미터**다.
  실측한 TPU/모터 특성이나 완성된 실기 동등 모델로 해석하지 않는다.
  센서값과 제어 법칙을 조정해 이 값을 숨기지 않는다.
- 수거함 wall의 20 mm를 half-extent로 넣었던 오류를 수정했다. 원래 생성된
  280×420 mm 외형을 source 실측 240×380 mm, 벽 두께 20 mm로 맞춘다.
  두 씬 생성기를 고쳤고 shared reset에서 생성된 USD 외형을 검사한다.
  **수거함 중심 위치는 여전히 release TCP 분포로 추정한 값**이다.
  소스: robotics_lab `83c5458^:camera_server/stereo_worker/box_detect.py:31,132`.
- 원본 stack은 external-box keep-out을 의도적으로 끄고 F/T를 사용한다.
  이 실행도 같은 설정이다. 환경 벽의 실제 접촉은 PhysX에서 발생하고 F/T로 들어온다.

## 출력과 검증 결과

`outputs/shared_stack/<tag>/`:

- `summary.json`: 설정 원본의 SHA-256, model 계약, 시간·청크 수·fault·종료 코드
- `joints.csv`: 500 Hz 양팔 actual/target/velocity와 실제 follower 이름
- `states.jsonl`: 카메라 cadence의 기존 C++ 상태 + `shared_control` 진단
- `policy_steps*.jsonl`: 기존 policy_runner의 step/chunk 기록
- `servo.log`, wrist PNG, `overview.mp4` (video 켠 경우)
- `scene_materials.json`: 색상별 볼트 수와 머리·몸통 visual material 연결 검증
- `plant_contact.json`: 실제 aggregate payload 질량/COM, tool body 조정, drive 한도
- `force.csv`: 500 Hz force coverage/tare/gate/deviation/actual-lead 및 wrench
- `contacts.jsonl`: 실제 충돌 collider/material 쌍, solver impulse/dt, 접촉 간격
- `chunk_packets.jsonl`: C++가 받은 v3 원본 패킷과 simulation 수신 시각
- `force_tracking.png`: 힘·gate·추종 오차 그래프 (분석 실행 후)
- 분석 실행 후 `tracking.png`, `tracking_metrics.json`

재생은 저장된 chunk delta/metadata와 정책 step의 target/gripper를 공통 제어기에
공급한다. 새 정책 추론이나 task score는 아니다. 예전 chunk-row 로그는 v3의
alignment/proprio metadata를 빠뜨렸으므로 현재 재생은 이를 만들어 넣지 않고 거부한다.
활성 `delta_preview` tick이 없는 실행도 성공 처리하지 않는다.

검증 기록:

| 실행 | 결과 |
|---|---|
| `shared_policy_02` | 3.002 sim 초, 1501 ticks, 21 chunks, fault 없음 |
| `shared_policy_04` | 5.002 sim 초, 2501 ticks, 36 chunks, 양팔 `delta_preview` 활성, 종료 0 |
| `shared_policy_03` | 15초 요청 중 8.564초에 왼팔 actual-lead fault로 안전 정지, 종료 4 |
| `shared_hold_final` | 최종 런처 Hold 시험: 0.502초, 251 ticks, fault 없음, 종료 0 |

reset에서 PhysX↔Pinocchio TCP 차이: 왼팔 0.000525 mm, 오른팔 0.000373 mm,
회전은 모두 0.00005° 미만. 5초 실행의 최대 관절 추종 오차는 왼팔 0.124°,
오른팔 0.336°. 3–15 Hz 추종 잔차 RMS는 관절별 약 0.006–0.014°였다.
이는 해당 실행의 측정치이며 기존 Python 경로 대비 감소율이나 실기 동등성 검증은 아니다.

접촉 실행에서는 왼팔 TCP가 테이블 높이(z≈−0.293 m)에 도달했고, 마지막 상태에서
손목 J6 actual/target 차이가 약 8°로 커졌다. C++가 `delta_preview_actual_lead_fault`로
멈췄다. **접촉·파지까지 실기와 같다는 검증은 아직 완료되지 않았다.** 힘 제어 제외의
한계, 접촉 재질·형상 및 plant 응답을 구분해 추가 측정해야 한다. 그 fault 한계를 늘리거나
관절 위치를 강제로 덮어써 우회하지 않았다.

자동 검증: C++ CTest 44개, 새 clock/bridge 테스트 7개(정지·reset·재무장 포함).
policy_runner 전체 테스트는
기존 `ChunkKnotFilterTest` 2개가 실패하며 변경 전 HEAD를 별도 디렉터리에 추출해
동일 실패를 재현했다. 해당 기존 필터나 테스트는 이번 변경에서 수정하지 않았다.

### F/T 및 치수 수정 검증 결과

- CTest 44/44, clock/bridge/F/T Python 11/11, 기록 재생 4/4 통과.
- PhysX 실제 부하 48 cases 통과: 최대 force error 0.038503 N,
  torque error 0.028067 Nm. 정지 자세 gravity 검사와 표준 tare도 통과.
- `ft_policy_1k`의 입력을 `ft_replay_1k_valid`로 재생했을 때 actual/target
  관절 기록이 완전히 같았다. 이 1 kN/m 실험값은 기본값으로 채택하지 않았다.
- 같은 입력/같은 15 kN/m profile에서 잘못된 40 mm box wall은 13.04초에
  actual-lead fault(최대 37.20 mm), 수정한 20 mm wall은 15.002초 완료
  (최대 22.56 mm, fault 없음). 이것은 고정 입력 비교이지 task 성공률이 아니다.
- 최종 8001 live 녹화는 30초 요청 중 정책 시간 **22.496초**에
  actual-lead fault로 종료 4. 왼쪽 force deviation은 40 mm fence에 오래
  도달했고, 그리퍼 본체/수거함 및 손끝/테이블 접촉이 함께 존재했다.
  보상/deadzone 이후 tool force peak도 약 **1348 N**으로 과도했다.
  **정지 문제와 실기 동등성은 아직 완전히 해결되지 않았다.**
- `ft_replay_15k`, `ft_replay_15k_02`는 개발 중 v3 metadata가 빠져 follower가
  활성화되지 않았던 무효 진단이다. 해당 summary에 invalid를 명시했으며 검증 근거에서 제외한다.

상세 결과: `outputs/diagnostics/shared_contact_20260905/report.md`.
최종 영상: `outputs/eval/shared_ft_pi05_8001_box20_final_aligned_00.mp4`
(960×720, 30 fps, tare 포함 23.067초). 실기 stack 설정/force 법칙/안전 한도는
변경하지 않았고 실물 로봇이나 그리퍼에 연결하지 않았다.

## 후속 작업

1. 실제 카메라 intrinsics/그리퍼 상대 pose를 확인해 현재 fx≈336px와 수집 데이터 fx≈393px 차이 보정.
2. bias 0 조건의 8002 운반 중 놓침과 8001 오른팔 실패를 기록된 접촉/폭/위치로 분석.
3. 동일 관절 목표 기록을 실기와 PhysX에 각각 재생한 자료로 drive 지연/응답 식별.
4. 수거함/패드 위치·gripper percent–개방 폭과 실제 셀의 정합 검증.
3. TPU 및 구동기 응답을 실측한 힘/변위·명령/측정 자료로 식별. 현재 센서 경로는
   양팔·양쪽 finger의 ±10 N / ±0.5 Nm 48가지 부하에서 최대 오차
   0.0386 N / 0.0281 Nm로 검증했으나, 이것은 실기 동역학 동등성 검증은 아니다.
4. box DH 보정 URDF를 사용할 때는 Isaac USD도 같은 기구학으로 생성하고 현재 FK gate로 검증.

## 폼 작업면 (2026-09-06)

`pika/data/data_20260904_233234/episode_000.hdf5`와
`pika/data/data_20260904_235111/episode_004.hdf5`의 양쪽 손목 RGB에서, 볼트가
금속 테이블 위의 어두운 회색 폼 패드에 놓여 있는 것을 확인했다.
수집 좌표는 `survive_world`이므로 raw pose를 stand 좌표로 간주해 패드 위치를
추정하지 않았다. 500×500×20mm 치수는 사용자가 제공한 값이다.

- `config/work_surface.json`: 치수, 위치, 텍스처, PhysX 접촉값의 단일 출처.
- `scripts/work_surface.py`: preview와 evaluator가 공유하는 패드 생성/배치/검증.
- 패드 중심 stand XY=(0.385, 0), 바닥 Z=-0.295m, 윗면 Z=-0.275m.
  riser와 15mm, 박스 가까운 벽과 10mm 간격을 두었다. XY는 영상과 기존 배치를
  바탕으로 한 추정이며 실제 셀 좌표를 계측한 값이 아니다. 박스는 테이블에 그대로 둔다.
- 실제 RGB의 물체 없는 128×128 영역을 텍스처로 사용한다.
  `assets/materials/pika_foam_patch.json`에 HDF5, 팔, 프레임, crop, hash를 기록한다.
  촬영된 명암/노이즈가 포함된 외관 근사이며 calibrated albedo는 아니다.
- 2026-09-06 색상 보정: `texture_rgb_scale=[0.70,0.75,0.95]`를 패드의
  UsdUVTexture에 적용해 Sim 조명에서 과하게 보이는 밝기와 노란 기를 줄였다.
  원본 텍스처/접촉값/형상은 그대로이며, frozen 40개 장면의 볼트 자세는 바꾸지
  않고 외관 프로필 메타데이터만 갱신했다. 같은 카메라의 정지 렌더로 비교했다.
- 패드는 정적 볼록 충돌체와 compliant contact를 사용한다. 강성 5000N/m,
  감쇠 30Ns/m, 정지/동마찰 0.6/0.5, 반발 0이다. **실측 폼 물성이나 재질 판별이
  아니다. 체적 변형·압축 후 복원·영구 눌림은 모사하지 않는다.**
  TPU 접촉값과 팔·그리퍼 drive, C++ FT/IK/안전 파라미터는 그대로다.
- 생성 시 볼트 footprint가 겹치지 않도록 pad 내부에 놓고, 2초 안정화한 자세를
  저장한다. 정책 관측/녹화는 그 이후 시작한다. 런타임에 볼트를 고정하거나 붙이지 않는다.
- 새 동결 장면: `assets/scene_states40_aligned_rb5_foam.json`, seeds 100–139,
  회색/검정 각 10개. 전체 800개 볼트의 실제 두 원통 형상 경계가 패드 안에 있고
  하단이 패드 윗면에 놓였는지 검증했다. 초기 위치가 달라졌으므로 기존 bare-table
  실행과 성공률을 같은 조건의 비교로 해석하지 않는다.
- 동결 파일에 작업면 프로필을 기록한다. old bare scene + foam, foam scene + bare는
  Isaac 시작 전에 거부한다. shared reset 뒤에도 전체 볼트 지지 상태를 다시 검증한다.
- `scene_materials.json`과 `summary.json.scene`에 pad 실제 크기/높이/물리 재질,
  작업면 프로필, 초기 볼트 자세, 지지 검증 결과가 남는다.

```bash
# 기본: 폼 패드 위의 동일한 고정 배치, 8001, 30초 녹화
scripts/run_shared_stack.sh --port 8001 --episode-sec 30 --tag foam_8001

# 과거 맨 테이블 장면 재현
scripts/run_shared_stack.sh --work-surface bare \
  --scene-states assets/scene_states40_aligned_rb5.json \
  --port 8001 --episode-sec 30 --tag bare_8001

# 기존 입력 재생은 그 입력이 기록된 작업면을 명시한다
scripts/run_shared_stack.sh --work-surface bare \
  --scene-states assets/scene_states40_aligned_rb5.json \
  --shared-replay outputs/shared_stack/shared_ft_pi05_8001_box20_final \
  --episode-sec 30 --tag replay_old_bare

# 새 고정 배치 생성: 정책/서보 서버에 연결하지 않음, 기존 파일 덮어쓰기 거부
OMNI_KIT_ACCEPT_EULA=YES \
PYTHONPATH=/home/plaif/workspace/openpi/packages/openpi-client/src \
  .venv-isaac/bin/python scripts/freeze_work_surface.py --count 40 \
  --output assets/new_foam_scenes.json
```

진단 자료: `outputs/diagnostics/foam_scene_20260906/`.

## 추가 진단 녹화와 비교 패키징

기본 녹화는 overview이며, 아래 래퍼는 같은 실행에 양쪽 손목 MP4와 100Hz
볼트/TCP/손가락 자세 및 tip 형상을 추가한다. 제어 입력과 물성을 바꾸지 않는다.
`--diagnostics-dir`와 `--tag`는 새 경로/태그를 사용한다.

```bash
OMNI_KIT_ACCEPT_EULA=YES \
PYTHONPATH=../openpi/packages/openpi-client/src \
.venv-isaac/bin/python scripts/record_shared_stack.py \
  --diagnostics-dir outputs/diagnostics/foam_next_8001 \
  --shared-stack --episodes 1 --episode-sec 30 --port 8001 \
  --scene-states assets/scene_states40_aligned_rb5_foam.json \
  --work-surface foam --layout aligned --seed 100 --n-per-color 10 \
  --tag foam_next_8001
```

`record_shared_stack.py`는 이번 8001/8002/8003 분석에 사용한 읽기 전용 계측을
저장소 CLI로 옮긴 것이다. 반복 실행 중인 태그/진단 파일을 덮어쓰지 않는다.

세 실행의 `manifest.json`을 가진 비교 디렉터리에는 다음 도구를 적용한다.
기록 형식은 `docs/results/20260906/manifest.json`과 같다. `models`에는
포트별 tag/directory/diagnostics/checkpoint/label, `snapshots`와 `source_hashes`에는
사용한 설정·자산·소스의 hash가 필요하다. 기록의 로컬 절대 경로는 다른 PC에서 조정해야 한다.

```bash
.venv-isaac/bin/python scripts/analyze_shared_task.py outputs/eval/<comparison>
.venv-isaac/bin/python scripts/package_shared_comparison.py outputs/eval/<comparison>
```

task 분석은 실제 볼트 상승 및 전체 형상이 올바른 박스 안에 남았는지를 검사한다.
패키징은 bias 0, 동일 제어/접촉/장면 설정, 초기 자세 차이, 영상 프레임 수를 검증한 뒤
개별 MP4와 시간 정렬 비교 MP4를 생성한다. 이 도구는 2026-09-06의 세 모델/폼/bias 0
프로토콜용이며, 다른 실험의 설정을 조용히 받아들이는 범용 성공 판정기는 아니다.
