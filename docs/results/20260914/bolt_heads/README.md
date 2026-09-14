# 볼트 머리 반영: 회색 = 버튼 헤드, 검정 = 소켓 캡 (2026-09-14, 임시 치수)

[나사산·머리 조사](../bolt_threads/README.md)에서 확인한 실기 볼트 종류를 sim에 반영했다. 치수는 M12
규격표 값을 **임시로** 넣었고, 실측(캘리퍼스) 후 설정 파일만 바꾸면 된다. 기본 리그는 그대로다.

## 무엇이 바뀌나

`EVAL_BOLT_GEOMETRY=config/bolts/iso_heads_20260914.json` + `--scene-states assets/scene_states40_aligned_rb5_foam_isoheads.json`

| | 검정 볼트 | 회색 볼트 |
|---|---|---|
| 머리 렌더 | 소켓 캡: 원통 18.4 × 12 mm, 옆면 널링 72줄, 모따기, 육각 구멍 s 10 · 깊이 6 | 버튼 헤드: 돔 지름 21 · 높이 6.6 mm(0.5 mm 테두리 + 원호 + 반경 5.8 mm 윗면), 육각 구멍 s 8 · 깊이 4.4 |
| 축 렌더 | M12 × 1.75 나사산(`bolt_visual.py`) | 같음 |
| 머리 충돌체 | 기존 원통(렌더에서만 숨김) → **물리 불변** | **돔의 볼록 껍질**(convexHull) → 물리 변경 |
| 질량 | 22 g(기존) | 16.4 g = 22 g × 충돌체 부피비(4494 / 6018 mm³) |

모델 확대 렌더: [`img/bolt_models_render.jpg`](img/bolt_models_render.jpg)(`scripts/render_bolt_models.py`).
실기 머리 사진: [`img/real_bolt_heads.jpg`](img/real_bolt_heads.jpg).

## 안전장치

- **초기 배치 파일 검사**: 배치 파일의 각 시드에는 정착에 쓴 형상의 이름과 sha256이 기록된다. 실행 형상과
  다르면 거부한다. 새 형상 + 기존 배치 파일, 기존 원통 + 새 배치 파일 둘 다 `ABORT`로 확인했다.
- **배치 제한**: 머리는 볼트 번호로 정해진다. aligned 배치에서는 모든 시드에서 0–9번이 회색, 10–19번이
  검정이다. random 배치는 거부한다.
- **색 검증**: shared-stack 리셋은 볼트 prim에 기록된 색(`simulation:boltColor`)과 장면의 색이 다르면 멈춘다.
- **기록**: `summary.json`의 `scene.bolt_geometry`에 이름·sha·색별 머리·질량이, `scene.bolt_proxies`에
  지지 검사용 경계 원통이 남는다.
- **형상 반영 범위**: 지지 검사(`work_surface.verify_bolts`), 고정 도구(`freeze_work_surface.py`),
  `analyze_shared_task.py`가 기록된 형상을 쓴다.
- **테스트**: `tests/test_bolt_visual.py`(11개)가 머리 외곽, 돔 단조성, 육각 치수, 충돌체 폐합·부피,
  질량비, 잘못된 스펙 거부를 고정한다.

## 새 초기 배치 (`scene_states40_aligned_rb5_foam_isoheads.json`)

같은 배치 규칙(시드별 x, y, 방향)으로 새 형상을 떨어뜨려 정착시켰다(`frozen_scene_change.txt`).

- 40/40 시드가 지지 검사를 통과했다.
- **14개 시드는 1000 스텝 안에 멈추지 않아 250 스텝씩 연장**했다(최대 1750). 누운 버튼 헤드 볼트는 돔 테두리와
  축 끝을 딛는 원뿔이라, 구름 마찰이 없는 PhysX에서 제자리에서 구른다. 어느 굴림 각에서든 평형이므로 멈춘
  자세를 고정해도 된다. 기존 원통 모드는 원래 1000 스텝 안에 멈췄으므로 결과가 바뀌지 않는다.
- 회색 볼트는 기존 배치 대비 xy가 중앙값 5.8 mm, 상위 10% 25.9 mm, 최대 78 mm 움직였다. 휴식 기울기로
  미리 기울여 떨어뜨리는 방식도 시험했지만 줄지 않아(5.8 / 25.4 mm) 되돌렸다.
- 검정 볼트는 형상이 같지만 최대 2.8 mm 달라졌다. 굴러온 회색 볼트와의 접촉이나 솔버 순서 차이로 보인다.
- 누운 자세의 축 기울기: 회색 9.7°, 검정 7.3°로 머리·축 반경 차이와 맞는다.

## 폐루프 확인 (gpu12, devjit40k 8044, seed 100·101, 사다리 A·G)

- 두 에피소드 모두 exit 0, fault 없음. 우팔은 검정 볼트를 집어 박스까지 옮겼다.
- 손목 뷰: [`img/handview_isoheads.jpg`](img/handview_isoheads.jpg),
  확대 [`img/handview_isoheads_crops.jpg`](img/handview_isoheads_crops.jpg). 회색 더미에 돔 머리가, 운반 중인
  검정 볼트 윗면에 육각 구멍이 실기처럼 보인다.

## 아직 아닌 것

- **실측 전**이다. 호칭경, 길이(특히 회색), 머리 지름·높이, 육각 크기, 질량을 재서 `config/bolts/`에 새 스펙을
  만들고 `freeze_work_surface.py`로 배치를 다시 고정한다.
- **보드는 아직 매기지 않았다.** 회색 볼트의 충돌체·질량·초기 배치가 바뀌었으므로 기존 보드와 비교하지 않고,
  새 기준(A·G 등)을 다시 채점해야 한다.
- `analyze_grasp_funnel.py`의 `REST_Z`(누운 볼트 원점 높이 = 9.2 mm 머리 반경)는 회색 버튼 헤드에서 약간
  달라진다. 들어올림 판정 여유(8 mm, 30 mm) 안이지만 보드 전에 확인할 것.
