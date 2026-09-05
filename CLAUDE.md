# simulation 현재 핸드오프

최신화: 2026-09-06. 시작점은 [README](README.md),
[공유 스택 계약](docs/shared_stack.md), [최신 결과](docs/results/20260906/README.md)다.

## 현재 기준

- 양팔 RB5-850E + Pika v15, 98mm 전체 개방 폭. RB3는 과거 자산/실험이다.
- 현재 경로는 `scripts/run_shared_stack.sh`다. robotics_lab policy_runner,
  C++ delta_preview/output SMD/FT/IK/안전 필터를 사용한다. 제어기 독립 재구현과 혼동하지 않는다.
- `config/shared_stack.json`: 2ms PhysX/C++ clock, anchored H24, execute/runway 4,
  command anchor/velocity/grip proprio, RTC off, **close bias L0/R0**.
- `config/work_surface.json`: 500×500×20mm 폼, 표면 Z=-0.275m,
  텍스처 RGB 배율 [0.70,0.75,0.95]. 위치/물성은 일부 추정값이다.
- `assets/scene_states40_aligned_rb5_foam.json`: 40 seeds, 색상별 10개,
  전체 볼트 형상의 패드 지지 검증. bare/foam 동결 씬을 섞으면 거부한다.
- 추론 서버 8001 plain / 8002 more-data / 8003 grip-only의 상세 체크포인트는
  결과 manifest에 있다. 실제 서버 식별은 매 실행 확인해야 한다.
- bias 0 기준 seed 100의 30초 결과는 8001 회색 1개, 8002 안착 0개,
  8003 회색·검정 각 1개다. 단일 장면 결과를 일반적인 모델 순위로 확대하지 않는다.

## 다시 반복하지 않을 것

- Sim에 남아 있던 과거 bias 2/6을 현재 Real 설정이라고 사용하지 않는다.
  command proprio에 보정값이 되먹임돼 닫힘 시점과 후속 정책 출력이 달라진다.
- 공유 코드와 실효 설정을 둘 다 비교한다. 원본 YAML의 hash와 runtime override를 기록한다.
- 카메라 렌더가 추가 물리 tick을 만들면 안 된다. 현재 `delta_time=0` 검사를 유지한다.
- nominal TCP 목표, 힘/안전 보정 후 목표, PhysX 실제 위치를 구분한다.
- 파지처럼 보이는 접촉과 실제 볼트 상승/올바른 박스 안착을 구분한다.
- C++ fault를 가리기 위해 lead/force 한계를 늘리거나 관절을 강제로 덮어쓰지 않는다.
- shared 실행은 별도 pipe-only bridge이며 물리 로봇/실물 그리퍼를 연결하지 않는다.

## 다음 우선순위

1. 실제 장착 카메라 intrinsics/그리퍼 상대 pose를 확인해 Sim 투영을 보정한다.
   최근 수집 데이터 fx≈393px, 현재 Sim fx≈336px 차이는 확인됐지만 아직 미수정이다.
2. 8002의 들어 올린 뒤 놓침과 8001 오른팔 실패를 접촉·그리퍼 폭·동작 전환으로 분석한다.
3. pad/box pose 및 그리퍼 percent–개방 폭·모터 응답·접촉 물성을 실측한다.
4. 같은 다중 장면에서 반복 평가하고 Real 결과와 상관을 검증한다.

## 이력

과거 전체 기록은 [이전 CLAUDE snapshot](docs/archive/CLAUDE_before_shared_stack_20260906.md)에
보존했다. 그 문서의 RB3, standalone Python follower, 예전 proprio/RTC/bias/성과 주장은
작성 당시의 실험 조건이다. 현재 기본값과 충돌하면 위 최신 문서를 따른다.
대표 검증 결과는 Git에 저장하고 원시 대용량 로그는 로컬 `outputs/`에 보존한다.
