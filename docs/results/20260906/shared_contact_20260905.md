# 공통 스택 F/T·접촉 모델 개선 결과 — 2026-09-05

Sim의 실제 tool 하중을 공통 C++ force pipeline에 연결하고, payload/접촉 재질 및
수거함 치수 오류를 수정했다. 고정 정책 입력에서 추종 오차가 줄었지만,
**최종 8001 live 실행은 정책 시간 22.496초에 정지했다. 완전 해결이나 task 성공으로
평가하지 않는다.**

## 구현

1. `attachment_site_joint`의 전체 도구 하중을 6축 F/T로 읽는다. 부호를 정한 뒤
   flange→SRO torque 이동, 기존 전기 채널 축 인코딩을 거쳐 기존 FtPipeline에 넣는다.
   원래 gravity compensation, deadzone, coverage, force gate, admittance, IK/안전을 사용한다.
   Fixed-joint incoming wrench의 기준은 [NVIDIA articulation force 문서](https://docs.isaacsim.omniverse.nvidia.com/5.0.0/sensors/isaacsim_sensors_physics_articulation_force.html)를 확인했고 실제 부하로 검증했다.
2. 원본 force config의 총 tool mass/COM에 맞춰 PhysX 하류 body의 합을 조정했다.
   기존 자동 계산 tool body 자체가 약 1.619 kg이었다. 수정 후 전체 payload는
   왼쪽 0.7912 kg, 오른쪽 0.7822 kg이다. inertia는 형상 기반 질량비 근사다.
3. TPU visual과 실제 collider가 형제인 구조를 확인했다. 네 `finger_tpu_sdf`에
   compliant physics material을 직접 연결하고 USD binding을 검증한다.
   [NVIDIA compliant contact 문서](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/rigid_bodies_articulations/rigid_bodies.html)의 스프링·댐퍼 모델을 사용한다. 본체/PLA는 rigid contact다.
4. 20 mm wall을 half-extent로 사용해 실제로는 40 mm 벽, 280×420 mm 외형이 되던
   두 씬 생성기를 수정했다. 20 mm 벽과 240×380×105 mm 외형을 생성·검증한다.
   근거는 robotics_lab `83c5458^:camera_server/stereo_worker/box_detect.py:31,132`의
   실측 외부 치수와 20 mm full wall이다. 박스 중심 자체는 아직 추정값이다.
5. `force.csv`, collider/material별 `contacts.jsonl`, 원본 v3 `chunk_packets.jsonl`을
   기록한다. 동일 입력 재생도 공통 policy_runner/lease/C++ follower/force/IK/안전을 통과한다.
   metadata를 누락한 옛 로그에서 유효한 proprio를 만들어 넣지 않으며,
   활성 delta_preview tick이 없으면 성공으로 처리하지 않는다.

## 고정 입력 비교

입력: `ft_policy_1k`에 저장된 8001의 원본 chunk delta/metadata 및 policy step/gripper.
이 이름의 1 kN/m 실험값은 기본값으로 채택하지 않았다. 같은 조건 재생
`ft_replay_1k_valid`에서 15,602개 arm-tick의 actual/target 관절 기록 최대 차이가
**0.0°**였다. 이후 두 비교는 모두 기본 15 kN/m contact profile을 사용하며
수거함 wall geometry만 다르다.

| 조건 | 정책 시간 | 최대 왼쪽 actual lead | 결과 |
|---|---:|---:|---|
| 수정 전 40 mm 벽, `ft_replay_15k_valid` | 13.038 s | 37.201 mm | fault, exit 4 |
| 수정 후 20 mm 벽, `ft_replay_box20` | 15.002 s | 22.557 mm | 요청 구간 완료, exit 0 |

![고정 입력 비교](contact_comparison.png)

추종 오차 감소는 관찰했지만 force quality 개선을 주장하지 않는다. 두 조건 모두
강한 접촉과 40 mm force-deviation fence 도달이 있었다. 자세한 수치는
[comparison.json](comparison.json)에 있다. 개발 중 `ft_replay_15k` 및
`ft_replay_15k_02`는 metadata 누락으로 follower가 비활성인 무효 실행이며 제외했다.

## 최종 8001 live 녹화

- 모델: `pi05_pika_umi_boltv2_anchAB_h24_40k`, `boltv2_plain_40k/39999`, localhost:8001.
- 실행: `scripts/run_shared_stack.sh --episode-sec 30 --tag shared_ft_pi05_8001_box20_final`.
- **22.496 s 정책 시간에 왼쪽 actual-lead fault**, exit 4. 167 chunks, 11,548 physics ticks.
- fault 시 actual lead 37.548 mm, 회전 0.0626073 rad. 35 mm 기존 기준을 유지했다.
- 왼쪽 보상/deadzone 후 force peak 1347.68 N, p95 1039.42 N. 정상 접촉 수준이 아니다.
  정책 시간의 약 58%에서 force deviation 40 mm fence에 도달했다.
- 실제 contact report에서 gripper base↔gray box 근접벽 약 1227 N,
  TPU↔table 약 1095 N peak를 확인했다. 서로 다른 시점의 pair peak이므로 더하지 않는다.
  TPU/table 최대 침투량 약 10.5 mm도 미해결 상태다.
- [녹화](/home/plaif/workspace/simulation/outputs/eval/shared_ft_pi05_8001_box20_final_aligned_00.mp4):
  960×720, 30 fps, 23.067초. 앞의 0.6초 tare 구간 포함.
- [실행 디렉터리](/home/plaif/workspace/simulation/outputs/shared_stack/shared_ft_pi05_8001_box20_final)
  에 원본 상태/관절/힘/충돌/패킷/설정 hash가 있다. 회색10/검정10, 40개 visual binding 확인.

## 설정과 schema

- 실기 stack/runner YAML은 수정하지 않았다.
  stack SHA-256 `98dd0045add8d83efd7f112f19af8335950cf9c5c9269d08d60c9fb3f8ea013e`.
  runner SHA-256 `08dff5c97a33b6bb12beec29c6dc1408c76ad439f35baedf257b51d62730c80b`.
- Sim JSON에 `contact` 명시 설정을 추가했다. 기본 sensor=true, tip stiffness=15000 N/m,
  damping=30 Ns/m, arm max effort=[200,200,200,100,100,100] Nm.
  이 값들은 실측한 모터/TPU 특성이 아닌 **임시 Sim 파라미터**다.
- reset 후 1 s settle, gravity residual 한도 0.5 N/0.1 Nm 검사, 기존 250샘플 tare를 위한
  0.6 s bootstrap. 검사 전 tare로 오류를 숨기지 않는다.
- bridge `init.force_sensor_enabled=true`이면 매 arm/tick에
  `{force_sensor:{valid:true,seq,time_ns,frame:"flange_at_flange",wrench:[6 SI values]}}`
  가 필요하다. 누락/오래된 값/잘못된 frame/비유한 값은 오류 종료한다.
- real의 전기적 noise liveness는 그대로다. externally stepped backend만
  transport의 seq/time 검증을 연결 판정으로 사용하며 bias/coverage를 자동 승인하지 않는다.
- 기존 external-box keep-out OFF도 그대로 유지했다. 이것은 source stack의 명시적
  F/T 대체 결정이다. 벽의 물리 접촉은 PhysX에 존재하며 센서에 들어온다.

## 변경 파일

이번 변경은 작업 시작 시 있던 공통 스택 변경 위에 추가했다. 기존 변경은 보존했다.

| 파일 | 이번 추가 |
|---|---|
| [shared_contact.py](/home/plaif/workspace/simulation/scripts/shared_contact.py) | payload, collider material, F/T, contact report |
| [shared_stack.py](/home/plaif/workspace/simulation/scripts/shared_stack.py) | sensor/tare bootstrap, geometry 검증, 원본 패킷 및 force logging |
| [shared_stack.json](/home/plaif/workspace/simulation/config/shared_stack.json) | 명시적 contact profile |
| [eval_closed_loop.py](/home/plaif/workspace/simulation/scripts/eval_closed_loop.py) | material 준비, replay 옵션, tray wall 수정 |
| [build_scene.py](/home/plaif/workspace/simulation/scripts/build_scene.py) | 같은 tray wall 수정 |
| [shared_replay.py](/home/plaif/workspace/simulation/scripts/shared_replay.py), [test_shared_replay.py](/home/plaif/workspace/simulation/scripts/test_shared_replay.py) | 원본 v3 입력 재생 및 검증 |
| [validate_shared_contact.py](/home/plaif/workspace/simulation/scripts/validate_shared_contact.py) | 물리 부하 48 cases |
| [analyze_shared_stack.py](/home/plaif/workspace/simulation/scripts/analyze_shared_stack.py) | 힘/gate/추종 그래프와 수치 |
| [isaac_bridge_main.cpp](/home/plaif/workspace/robotics_lab/rb_servo_server/src/isaac_bridge_main.cpp) | strict F/T packet, SRO 이동 및 축 인코딩 |
| [ft_pipeline.hpp](/home/plaif/workspace/robotics_lab/rb_servo_server/include/rb_servo/sensor/ft_pipeline.hpp), [ft_pipeline.cpp](/home/plaif/workspace/robotics_lab/rb_servo_server/src/sensor/ft_pipeline.cpp) | 외부 검증 연결 판정 |
| [dual_arm_servo_loop.cpp](/home/plaif/workspace/robotics_lab/rb_servo_server/src/control/dual_arm_servo_loop.cpp) | external stepping의 liveness 분기만 추가 |
| [test_force_control.cpp](/home/plaif/workspace/robotics_lab/rb_servo_server/tests/test_force_control.cpp), [test_isaac_transport.py](/home/plaif/workspace/robotics_lab/policy_runner/tests/test_isaac_transport.py) | invalid/stale/no-tare/frame 검증 |
| [Sim 문서](/home/plaif/workspace/simulation/docs/shared_stack.md), [bridge 문서](/home/plaif/workspace/robotics_lab/docs/isaac_shared_stack.md) | 계약, 실행, 검증과 한계 |

## 검증 및 생략

- C++ build 통과. `ctest --test-dir rb_servo_server/build --output-on-failure`: **44/44**.
- Python clock/bridge/F/T unittest: **11/11**. replay unittest: **4/4**.
- Isaac 실제 양팔·양쪽 finger에 ±10 N/±0.5 Nm를 가한 **48 cases 통과**.
  최대 오차 **0.038503 N / 0.028067 Nm**, 기준 0.4 N / 0.05 Nm.
  [원본 검증](/home/plaif/workspace/simulation/outputs/diagnostics/ft_wrench_01/wrench_validation.json).
  이는 reset 자세의 센서 load-path 검증이며 전체 작업공간 동역학 식별은 아니다.
- Isaac Hold/bootstrap 통과, 정책 입력 재생 및 8001 live 실행은 위 결과 그대로 기록.
- Python compile, 두 저장소 `git diff --check`, 최종 mp4 decode/치수/fps 확인 통과.
- 전체 Python suite는 이전 검증에서 변경 전 HEAD에도 재현된 ChunkKnotFilter 2개 실패가
  있어 이번에는 변경 범위 bridge/replay tests를 실행했다. 기존 필터는 건드리지 않았다.
- 별도 legacy build_scene 전체 렌더, 실기 모션/접촉 시험, 파지 성공률 평가는 하지 않았다.
  이번 범위는 공통 스택의 Sim 접촉 개선이며 하드웨어 시험 권한이나 동역학 식별 자료가 아니다.

## 남은 일과 안전 영향

가장 먼저 실물 수거함 **중심·회전·외형**과 gripper base collision hull을 같은 stand
좌표에서 정합해야 한다. 외부 치수 오류는 고쳤지만 중심은 TCP release 분포로 추정한
값이라 실물과의 간섭 관계를 확정할 수 없다. 다음으로 TPU force/displacement 및
actuator 명령/측정 응답을 식별하고, 동시 접촉과 policy의 접근/회전 동작을 재생해 검증한다.
현재 큰 힘을 해결하려고 임의로 수거함을 이동하거나 충돌을 제거하지 않았다.

실기 실행 파일 경로, 실기 YAML, force 제어 법칙/게인, IK, lease, 관절 제한,
35 mm actual-lead 및 40 mm force-deviation 한도는 변경하지 않았다.
추가한 연결 판정은 strict external stepping에서만 작동한다.
실물 로봇/그리퍼 연결 또는 모션은 실행하지 않았다. 이 결과는 Sim 개선 단계의
검증 자료이며 실기 실행 승인이나 정상 파지/접촉 성능을 뜻하지 않는다.
