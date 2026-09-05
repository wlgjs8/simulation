# 2026-09-05: shared-stack 추종 오차 정지 원인

이번 정지의 직접 원인은 **왼쪽 손가락이 테이블에 막힌 상태에서 명령이 계속 아래로 진행한 것**이다. 일반 이동 구간의 모터 지연, IK 실패, 볼트 끼임이 주원인이라는 가설은 이번 기록과 재생 실험에 맞지 않는다.

## 원본 기록

- Sim: `outputs/shared_stack/shared_pi05_8001_colors_fixed_20260905_211812`.
- 모델: :8001, `pi05_pika_umi_boltv2_anchAB_h24_40k`, `boltv2_plain_40k/39999`.
- 2 ms 고정 물리/C++ step, 63 chunks, 8.592초에서 종료 코드 4.
- 정지 이유: `left delta_preview_actual_lead_fault`, 위치 42.5209 mm, 각도 0.0406395 rad = 약 2.33도. 위치 기준 35 mm를 넘은 경우이며 각도 기준 5도는 넘지 않았다.
- 이 오차는 follower의 계획 위치와 실제 관절 FK 위치의 차이이다. IK 해의 잔차와 다르다.
- 마지막 30 Hz 상태(8.568초): 테이블 z = -0.295 m, 실제 TCP z = -0.293222 m, C++ 명령 z = -0.329588 m. 실제 TCP는 테이블 위 약 1.78 mm에 멈춰 있고 명령은 테이블 아래 약 34.59 mm이다. 3차원 계획 오차에는 수평 성분과 필터 앞/뒤 차이도 포함되므로 단순 z 차이가 fault의 42.52 mm와 같지는 않다.
- 같은 시점 IK: 성공, 위치 잔차 약 0.000595 mm, 최소 특이값 약 0.208. 관절/속도/가속도 clamp 모두 false. 마지막 팔꿈치 J3의 실제/목표 차이는 약 5.40도이다.
- C++ 환경 정보: floor constraint OFF, external-box 목록 없음. PhysX에 존재하는 테이블 접촉은 C++ 충돌 manifest에 없는 환경 접촉이다. 물리에서는 막히지만 기하학적 안전 제한으로 설명되지 않고, F/T도 공급하지 않아 실제 추종 오차로 남는다.
- 소스 설정의 해시가 summary에 기록되어 있다. :8001 추론은 약 52–74 ms 수준이며 이 정지는 추론 시간 초과나 state stale 오류가 아니다.

## 원인 분리 재생 실험

`replay.py`는 기존 eval의 동일 씬 생성 함수를 사용하되, 정책/C++/네트워크에 연결하지 않고 저장된 500 Hz 관절 목표만 PhysX drive로 재생한다. 원본에서 목표를 계산한 다음 tick에 적용한다. 그리퍼는 policy step 로그의 명령과 타임스탬프를 zero-order hold로 재생한다. 물리 dt, reset, drive, 모델 형상은 세 조건에서 동일하다. 각 조건은 새 Isaac 프로세스로 실행했다.

| 조건 | 왼팔 최대 관절 오차 | 오른팔 최대 관절 오차 | 왼손–테이블 최초 접촉 | 마지막 왼 TCP z |
|---|---:|---:|---:|---:|
| 원래 조건 | 5.3901도 | 0.2743도 | 8.054초 | -0.293375 m |
| 테이블 collider만 OFF | 0.2567도 | 0.2743도 | 없음 | -0.330412 m |
| 볼트 collider만 OFF | 5.3913도 | 0.2743도 | 8.054초 | -0.293352 m |

세 조건의 입력 `joints.csv` SHA-256은 `2f056dc8f8bb67b90cac7b30b099013c36cc8a52428c71b223b6aa3b0124287d`로 같다. 원래 조건 재생과 원본 녹화의 TCP z 차이는 30 Hz 비교에서 최대 0.0632 mm였다. 따라서 재생의 접촉 상태는 원본을 충분히 가깝게 재현했다.

테이블을 없애면 볼트도 낙하하는 부수 효과가 있으므로 볼트 collider만 제거한 세 번째 조건을 추가했다. 볼트가 없어도 같은 시각의 테이블 접촉과 큰 오차가 남았다. 이는 이 정지의 직접 방해물이 볼트가 아니라 테이블이라는 근거다.

이것은 **물리 원인을 분리한 open-loop 실험**이다. 안전 기준 변경이나 실제 closed-loop 완주를 의미하지 않는다. 진단 조건은 production config에 적용하지 않았다.

![접촉 전후의 TCP 높이, 계획 오차, 관절 오차](contact_diagnosis.png)

## Real과의 대조

같은 체크포인트와 anchored/H24/execute4/command proprio 설정의 실기 rollout `20260904_235947_boltv2_plain_40k`를 사용했다. 정책 로그의 monotonic 구간을 `robotics_lab/logs/servo_log_20260904_235421.csv`와 맞춰 95,723 tick, 191.444초를 추출했다. 서로 다른 장면/행동이므로 동일 궤적의 모터 응답 비교로 해석하지 않는다.

- 해당 실기 구간 fault 0회, 양팔 `delta_preview` 동작.
- 왼팔 위치 lead: 최대 24.97 mm, p99 13.15 mm. 오른팔 최대 21.53 mm.
- 실기 왼 TCP 최저 z = -0.287769 m. Sim에서 정지 직전의 명령(-0.329588 m)처럼 깊게 밀고 들어가는 실행은 아니었다.
- 실기 설정에 힘 제어가 켜져 있으나, 실제 `fc_covered`는 왼팔 처음 약 64.23초, 오른팔 처음 약 18.14초만 true였다. 나머지는 false이며 그 구간에서도 fault가 없었다. 따라서 **이번 차이를 전부 힘 제어 OFF 때문이라고 단정할 수 없다.**
- 실기 왼 force gate 최저 0.0456, 최대 force overlay 변위 약 0.646 mm. 실제 force compose가 적용된 tick은 약 24.5%였다. Sim은 F/T와 force overlay가 전체 OFF이며 접촉 시 계획을 감속시키는 force gate가 없다.

`real_matching_rollout.csv`, `real_metrics.json`에 대조 구간과 집계를 보존했다.

## 공유 코드인데 왜 접촉에서 달라지는가

1. **동일 모델이어도 입력과 실행 궤적이 다르다.** Sim의 영상/물체 배치/손끝 접촉 상태는 Real과 같지 않다. 이번 Sim은 실제 손끝이 막힌 뒤에도 하강하는 계획을 내보냈다. Real 비교 구간에서는 그렇게 깊이 진행하지 않았다. 특정 원인이 카메라 외부 파라미터인지, 렌더링 외관인지, 그리퍼/볼트 접촉 실패인지까지는 이 기록만으로 분리되지 않는다.
2. **알고리즘 공유와 접촉 피드백 공유는 다르다.** 현재 bridge는 힘 제어를 명시적으로 제외했고, 테이블을 C++ 환경 장애물로 등록하지 않았다. 따라서 IK는 테이블 아래 목표도 기구학적으로 풀 수 있고, PhysX는 손가락 관통을 막는다. 계획이 진행하지만 실제가 멈춰 lead가 쌓인다. 실기의 force gate는 접촉 방향 계획 진행을 감속시킬 수 있으나 Sim에 전달할 센서값이 없다.
3. **현재 PhysX plant는 실기 액추에이터/재질을 식별한 모델이 아니다.** arm stiffness 1e7, damping 1e5, USD angular acceleration drive maxForce 20000을 사용한다. v15의 TPU blade도 강체다. 재생에서 두 손가락–테이블 접촉력 크기 합은 순간 최대 약 53.6 kN으로 계산됐다. 이는 실기 센서로 측정한 힘이 아니라 `get_contact_force_matrix(dt=0.002)`로 읽은, 보정되지 않은 PhysX solver 출력이다. 현실적인 접촉 힘을 재현했다고 볼 수 없으며 drive effort 및 접촉 유연성 검증이 필요하다.
4. **re-anchor는 테이블 접촉을 해결하지 못한다.** 공통 코드가 계획을 마지막으로 보낸 목표의 FK에 다시 붙이지만 그 목표 자체가 테이블 아래다. 반복해도 실제 손끝과 차이가 줄지 않고, unexplained re-anchor 예산(2초 안 5회)을 소진하면 정지한다. Sim만 안전 기준이 더 엄격하게 설정된 것은 아니다.

PhysX의 drive는 위치/속도 오차에 따른 spring-damper effort이며 RB 컨트롤박스 자체가 아니다. stiffness, damping, effort limit은 별도로 맞춰야 한다. [NVIDIA Gain Tuner 문서](https://docs.isaacsim.omniverse.nvidia.com/latest/robot_setup/ext_isaacsim_robot_setup_gain_tuner.html), [PhysX joint drive 문서](https://nvidia-omniverse.github.io/PhysX/ovphysx/latest/simulation_setup/joints.html).

## 다음 수정의 우선순위

- 보정된 액추에이터 effort/응답과 손끝–테이블 접촉 유연성을 마련하고, tool 전체에 걸리는 simulated wrench를 로깅한다. 현재의 과도한 접촉력을 그대로 실기 F/T로 간주해서는 안 된다.
- 실제 sensor 축, TCP 기준점, 중력 보상, tare 계약에 맞춘 sim F/T를 기존 force-control/plan gate에 연결한다. 이 작업을 하거나 공통 기하 안전 계층에 명시적으로 환경을 반영해야, 물리 접촉이 기존 제어기에 전달된다.
- 실기와 동일한 손목 카메라 자세/내부 파라미터, TCP 높이, 그리퍼 개구량을 대조하고, 정책이 Sim에서 계속 하강하는 상위 원인을 분리한다.
- 실제 기록과 동일한 목표를 재생하는 자유 공간/접촉 응답을 확인한 다음 :8001 closed-loop를 다시 검증한다. fault 한계를 높이거나 관절 위치를 강제로 덮어써 완주시키는 방식은 원인을 해결하지 않는다.

## 작업 범위와 검증

- 생성 파일: 이 진단 디렉터리의 재생/분석 스크립트, CSV, JSON, 그래프, 로그 및 보고서.
- product 코드, schema, 안전 기준, tracked real/sim config 변경 없음. Real 동작/하드웨어 연결 없음.
- Isaac 재생 3개 모두 4,296 physics tick 완료, 종료 코드 0. 원본 대비 z 재현 오차와 동일 입력 해시 확인. 분석 실행 완료.
- 하드웨어를 구동한 A/B, 실제 F/T 센서 교정, 액추에이터 파라미터 식별, upstream 카메라/정책 원인 분리는 미실시. 이번 요청의 원인 조사 범위에서 확보할 수 있는 기록과 simulation으로 검증했다.
