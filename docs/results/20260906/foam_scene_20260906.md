# 최근 Pika 데이터에 맞춘 폼 작업면

2026-09-06 요청에 따라 기본 시뮬레이션 작업면을 500×500×20mm 어두운 회색 폼 패드로 변경했다. 볼트는 패드 위에 놓고 안정화한 상태에서 정책이 시작한다. Real 설정이나 제어 코드는 변경하지 않았다.

## 실제 데이터 확인

다음 두 HDF5의 양쪽 RealSense RGB에서 금속 테이블 위에 놓인 거친 표면의 어두운 패드와, 그 위에서 볼트를 집어 상자에 놓는 모습을 확인했다.

- `/home/plaif/workspace/pika/data/data_20260904_233234/episode_000.hdf5`
- `/home/plaif/workspace/pika/data/data_20260904_235111/episode_004.hdf5`

후자의 오른쪽 카메라 frame 404에서 물체가 없는 영역 (110,70)–(238,198)을 128×128 RGB 텍스처로 추출했다. 색상/노이즈/명암은 촬영 데이터에 포함된 값이다. 정밀한 반사율을 측정한 것은 아니다. 추출 근거는 `assets/materials/pika_foam_patch.json`, 원본 프레임과 contact sheet는 이 보고서 디렉터리에 있다.

500×500×20mm는 사용자가 제공한 치수이다. 사진만으로 EPS/EPP/EVA 등 재질 종류와 압축 물성을 확정하지 않았다. 수집 pose frame은 `survive_world`이므로 raw XYZ를 stand 좌표로 직접 사용하지 않았다.

## 구현

- 패드 stand 중심 XY=(0.385,0)m. 바닥 Z=-0.295m, 윗면 Z=-0.275m.
- XY는 기존 riser/박스 사이에 배치한 추정값이다. riser와 15mm, 박스 가까운 면과 10mm 떨어진다. 실제 패드 pose 계측치는 아니다.
- 프리뷰와 평가기에 같은 `work_surface.py`를 사용한다. 외관은 추출 텍스처, roughness 0.95, metallic 0이다.
- 정적 충돌체에 compliant contact를 설정했다. 강성 5000N/m, 감쇠 30Ns/m, 마찰 0.6/0.5, 반발 0은 조정 가능한 초기 근사값이다. 체적 변형이나 영구 눌림을 재현한 deformable foam 모델은 아니다.
- 새로운 배치는 겹치지 않는 볼트 footprint를 패드 내부에 배치한 뒤 2초 안정화한다. 40개 seed(100–139), 각 회색 10개/검정 10개의 실제 자세를 별도 파일에 저장했다. 기존 bare-table 파일은 보존했다.
- 정책이 시작한 후에는 실제 rigid-body 볼트가 움직이며, 접착/부착/위치 고정 방식으로 파지를 흉내 내지 않는다.
- 기존 테이블용 frozen scene을 foam에 불러오거나 그 반대로 사용하는 경우 Isaac 초기화 전에 오류로 거부한다. 공유 제어기 진입 전 pad 형상·재질과 볼트 전체 형상의 지지 상태를 검증한다.
- legacy preview의 높이 검증에서 과거 RB3 절대 Z 상수를 쓰던 부분도 실제 reset/작업면 기준으로 바꿨다.

## 검증

- Python compile 및 `git diff --check` 통과.
- PhysX에서 생성한 40개 장면/800개 볼트의 전체 두 원통 경계가 패드 안에 있고, 하단이 작업면에 닿아 있음을 검증했다.
- 새/옛 frozen scene의 양방향 작업면 불일치가 거부되며, 기존 장면은 bare 모드에서 승인됨을 확인했다.
- 실제 USD pad 크기 0.500×0.500×0.01999998m, 윗면 -0.275000006m. 물리 재질 `/World/physmat_pick_pad`와 강성/감쇠 readback 확인.
- 8001 (`boltv2_plain_40k/39999`) 정책을 30.002초, 15,301 physics ticks, 223 chunks 실행했다. 제어기 exit code 0, fault 없음. 최대 실제 선행 오차 L=2.76mm/R=3.76mm.
- 960×720, 30fps, 918프레임/30.6초 MP4를 끝까지 읽어 확인했다. 앞 0.6초는 tare다.
- **이번 실행은 그리퍼/볼트/패드 접촉 보고가 0건이며, 파지 성공을 확인한 실행이 아니다.** 배경/작업면/초기 배치 변경으로 같은 8001 정책의 동작도 달라졌다. 작업면 개선과 파지 성공률 개선은 구분해야 한다.
- 정책 결과·MP4 저장 완료 후 Kit 종료 단계에서 shell exit 143이 반환됐다. 위의 제어기 exit 0과 구분해 `verification.json`에 기록했다. 저장된 영상은 전체 프레임을 읽을 수 있었고, 이 종료 코드를 근거로 제어기 fault를 추정하지 않았다. Kit 종료 신호의 원인은 이번 장면 수정 범위에서 변경하지 않았다.

기존 bare 실행과는 작업 높이/배경뿐 아니라 초기 볼트 XY도 달라졌다. 이번 한 실행에서 기존 대비 개선율이나 물성 식별 결과를 주장하지 않는다.

## 변경 파일

- `config/work_surface.json`
- `scripts/work_surface.py`, `scripts/freeze_work_surface.py`
- `scripts/build_scene.py`, `scripts/eval_closed_loop.py`, `scripts/shared_stack.py`, `scripts/run_shared_stack.sh`
- `assets/materials/pika_foam_patch.png`, `assets/materials/pika_foam_patch.json`
- `assets/scene_states40_aligned_rb5_foam.json`
- `README.md`, `docs/shared_stack.md`

산출물:

- `outputs/eval/shared_pi05_8001_foam_20260906_aligned_00.mp4`
- `outputs/shared_stack/shared_pi05_8001_foam_20260906/`
- 본 디렉터리의 `real_sim_surface.png`, `overview_start.png`, `overview_end.png`, `verification.json`.

Real의 stack/runner config SHA-256은 이전과 동일하다. policy runner, C++ delta_preview/FT/IK/안전 한계, 팔/그리퍼 drive는 그대로다. 기존 dirty 변경도 보존했다. 이번 작업은 Sim 작업면과 초기 배치에 한정했다.

재실행:

```bash
cd /home/plaif/workspace/simulation
scripts/run_shared_stack.sh --port 8001 --episode-sec 30 --tag foam_next
```

다음 검증 대상은 패드 물성의 실측과 실제 패드 stand pose 보정, 그리고 새 관측에서 정책이 볼트에 도달해 닫힘·상승까지 진행하는지 여부다.
