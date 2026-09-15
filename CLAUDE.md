# simulation 현재 핸드오프

최신화: 2026-09-13. 시작점은 [README](README.md),
[공유 스택 계약](docs/shared_stack.md), [최신 결과](docs/results/20260907/README.md)다.

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
- **채점 광학은 실측 fx 393.55(조리개 17.8885 × 13.4524)로 고정한다.** bolt_v2(`data_v2`)
  체크포인트는 이 값으로만 채점한다. `EVAL_H_APERTURE`/`EVAL_V_APERTURE`는 명시적 A/B 전용이고,
  기본값을 벗어나면 실행 로그에 경고 배너가 찍힌다. 데이터시트 fx 335.96으로 매긴 보드
  (2026-09-06 6모델 보드 포함)는 순위로도 대조군으로도 쓰지 않는다.
- 현재 보드는 [2026-09-07 std40 fx393](docs/results/20260907/README.md)다. veldrop50 29,
  griponly 27, plain_r4 22, ph3_r5 20, ph3_r4 18, plain_r5 17. 광학 보정에 움직인 모델은
  griponly 하나뿐이다(13→27, p=0.022). velocity proprio가 마스킹된 모델만 투영 오차를 전부 떠안는다.
- **광도(조명·톤매퍼·재질)는 스톡 리그가 채점 기준이다.** 노브는 `EVAL_RTX`/`EVAL_MAT`/`EVAL_*_INTENSITY`,
  프리셋은 `EVAL_PHOTOMETRY`이고, shared-stack `summary.json`의 `photometry`에 기록된다. 09-12 피팅
  프리셋 `cell_fit_20260912_t013`은 전체 프레임 분포만 맞췄다. 같은 손목 뷰 사다리
  ([2026-09-13](docs/results/20260913/README.md))에서 **회색 볼트가 흰 덩어리로 포화**되고 박스 색이
  틀어져 재채점에 쓰지 않는다. 전 구간 W1은 A 대비 좌 -3.6 / 우 -1.1뿐이다.
  실기 배포 체크포인트(devjit40k) A–D 보드([2026-09-14](docs/results/20260914/README.md)): 안착은
  A 29 / B 14(p=0.035) / C 20 / D 23으로 어떤 변경도 늘리지 못했다. 다만 좌팔이 들어올린 회색 볼트를
  박스 밖에서 놓는 비율은 A 0.66 → D 0.20(p=0.001)으로 줄고, op0 세 조건 모두 우팔 파지가 무너진다.
  분리 보드([separation](docs/results/20260914/separation/README.md)): 우팔 붕괴 원인은 **op0 톤매퍼**다
  (D→G Iray로 우팔 안착 6→16, p=0.002). D의 좌팔 손실 원인은 **t013 회색 볼트 재질**이다(D→E 좌팔 안착
  19→37, p<0.001). 안착은 G 46 / E 44 / A 39 / F 31 / D 24이고, 운반 중 박스 밖 놓기는 A만 높다(0.46,
  나머지 0.16–0.29). G(t013 조명·재질 + Iray)가 후보지만, A 대비 안착 차이는 유의하지 않아 재현이 필요하다.
  **gpu13 재현([replication](docs/results/20260914/replication/README.md))**: 안착 G 53 / E 39 / A 33 / D 23,
  A→G p=0.007이다. 두 배치 합산 A→G 72→99(31/12, p=0.005), D→G 우팔 12→35, D→E 좌팔 37→69로 재현됐다.
  운반 중 박스 밖 놓기는 E에서 재현되지 않았고(0.16→0.40), t013 회색 볼트 재질이 있는 D·F·G에서만
  일관되게 낮다. 즉 볼트 재질에 반응한다. G가 채점 광도 후보이지만, 기준 교체는 아직 결정하지 않았다.
  **실기 후보 11개 G 보드([all11_G](docs/results/20260915/all11_G/README.md))**: devjit_r6 52 ≈ r6knormcrop 51 >
  ph3 43 > plain_first 39 > griponly 36 > griponly_r6 34 > plain_r4 29 > r6knorm·veldrop50 28 > **devjit_r7 25** > plain_r5 23.
  r7의 결손은 전부 좌팔이다(좌 박스 밖 놓기 0.74 vs r6 0.14). devjit_r6는 G 조건 세 배치에서 46/53/52로 안정적이다.
- **볼트 외형**: `EVAL_BOLT_VISUAL=threaded`는 렌더 전용 M12 나사산을 넣는다(충돌 원통은 숨겨서 유지,
  seed 100 초기 자세가 기존 보드와 완전히 같다). 기본값은 원통이다. 실기 회색 볼트는 **버튼 헤드**라서
  `EVAL_BOLT_GEOMETRY=config/bolts/iso_heads_20260914.json`(회색 돔 볼록 껍질 충돌체 16.4 g, 검정 널링·육각 렌더,
  둘 다 나사산)로 반영했다. **현재 형상은 실측 `config/bolts/measured_20260915.json`이다**: 회색 ISO 7380-1
  M12×20(머리 20.5×6.0, 13.7 g), 검정 ISO 4762 M12×25(머리 18.0×12, 22 g). 반드시
  `assets/scene_states40_aligned_rb5_foam_measured.json`과 함께 쓴다(형상이 다른 배치 파일은 거부된다).
  이 형상의 보드는 아직 없다([bolt_measured](docs/results/20260915/bolt_measured/README.md); 임시 형상
  `iso_heads_20260914`는 이전 기록이다). 손목 카메라는 Isaac 기본 DLSS
  Performance라 320×240에서 업스케일된다(원본 해상도는 `EVAL_RTX=/rtx/post/aa/op=4`).
  [bolt_threads](docs/results/20260914/bolt_threads/README.md)
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
- 보드 숫자를 **다른 동시 실행 수**의 보드와 비교하지 않는다. 같은 광학·같은 시드에서 동시 8은
  griponly 35, 동시 12는 13이 나온다. 서빙 정책의 샘플링 노이즈가 서버 프로세스당 단일 키
  스트림이고 동시 에피소드가 도착 순서로 나눠 쓰기 때문에, 같은 시드도 원리적으로 재현되지 않는다.
- 모델 순위를 안착 수 하나로만 읽지 않는다. `scripts/analyze_grasp_funnel.py --detail`로
  닫힘 시도 / 캐리 / 릴리즈 지점까지 본다. griponly는 캐리 최다·안착 최소였고, 원인은 파지가
  아니라 운반이었다.

## 다음 우선순위

1. **주점 정렬.** 렌더는 화면 중앙, 학습 카메라는 좌 (319.08, 229.52) / 우 (316.93, 238.20).
   좌 10.5px는 작동 거리에서 약 5mm이고, 광학 보정 후에도 **좌팔은 여섯 모델 모두 박스
   150~190mm 앞에서 그리퍼를 연다**. 노브는 `EVAL_PP_LEFT`/`EVAL_PP_RIGHT`로 있다.
   단, 실기 좌팔도 같은 실패(박스 위 24% vs 우팔 49%)라 sim 단독 결함은 아니다
   (`scripts/real_release_sites.py`).
2. **채점 재현성.** 서버당 클라이언트 1개로 돌리거나 요청별 시드를 넣는다. 지금은 같은 시드가
   재현되지 않아 보드 간 비교가 배치 고정에 의존한다.
3. **물체 단위 광도 보정.** 회색/검정 볼트·두 박스·바닥면·팁을 실기 프레임 영역과 맞춘다(전체
   프레임 CDF는 물체를 구속하지 못한다). 끝나면 `scripts/photometry_ladder.py`로 같은 뷰를 확인한
   뒤 스톡 대비 짝지은 std40을 돌리고, `analyze_grasp_funnel.py`의 들어올린 뒤 박스 밖 release로
   "운반 중 놓기"를 판정한다.
4. pad/box pose 및 그리퍼 percent–개방 폭·모터 응답·접촉 물성을 실측한다.
5. 좌팔 `ChunkFollowerFault`(각도 0.1rad 허용치) — 약 360런 중 3건, 전부 좌팔이다.
6. 같은 다중 장면에서 반복 평가하고 Real 결과와 상관을 검증한다.

## 이력

과거 전체 기록은 [이전 CLAUDE snapshot](docs/archive/CLAUDE_before_shared_stack_20260906.md)에
보존했다. 그 문서의 RB3, standalone Python follower, 예전 proprio/RTC/bias/성과 주장은
작성 당시의 실험 조건이다. 현재 기본값과 충돌하면 위 최신 문서를 따른다.
대표 검증 결과는 Git에 저장하고 원시 대용량 로그는 로컬 `outputs/`에 보존한다.
