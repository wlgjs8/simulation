# 2026-09-06 커밋 전 검증

## 변경 범위

현재 실행 경로는 `robotics_lab`의 policy runner와 C++ delta_preview/IK/안전 필터를
Isaac PhysX plant에 연결하는 공유 스택이다. simulation에는 실행·재생·접촉 진단,
500×500×20mm 폼 환경과 실제 RGB 기반 외관, bias 0/0, 동결 장면 40개,
녹화·비교 도구와 대표 결과를 보존했다. 기존 독립 Python 제어기 실험은 legacy로 남긴다.

의존 저장소의 변경은 `isaac_transport.py`, `rb_isaac_bridge`, 외부 2ms clock/step,
공유 command/chunk ingress, 외부 F/T 입력 및 해당 테스트/문서다.
정확한 버전은 [stack_versions.json](../config/stack_versions.json)에 기록한다.

## 이번 커밋 준비에서 다시 실행한 검사

| 검사 | 결과 |
|---|---|
| `cmake --build rb_servo_server/build -j6` | 성공 |
| `ctest --test-dir rb_servo_server/build --output-on-failure` | 44/44 통과, 73.74초 |
| Isaac transport/clock/F/T unittest | 11/11 통과 |
| simulation shared replay unittest | 4/4 통과 |
| `python -m compileall -q scripts` 및 새 녹화/비교/분석 CLI `--help` | 성공 |
| 새 CLI로 기존 세 실행의 비교 MP4 재생성 및 task metrics 재계산 | 성공, 기존 수치와 일치 |
| 전체 policy_runner unittest, 실제 OpenPI Python 환경 | 570개 실행, 기존 실패 2개, skip 3개 |

재패키징은 `outputs/validation/commit_20260906_uq_emhjk/`의 새 디렉터리에서 수행했다.
기존 비교 결과를 덮어쓰지 않았다. 세 실행의 코드·자산 SHA256, 설정·접촉 모델·장면 일치,
초기 볼트/팔 위치, MP4 프레임 수를 다시 검사했다. 세 개별 영상과 비교 영상은
모두 918프레임/30fps다. 최종 올바른 박스 개수는 8001 회색 1/검정 0,
8002 0/0, 8003 1/1로 동일했다.

주요 Python 검사 명령은 다음과 같다. 첫 두 명령은 `robotics_lab/`,
마지막 두 명령은 `simulation/`에서 실행한다.

```bash
../simulation/.venv-isaac/bin/python -m unittest discover \
  -s policy_runner/tests -p 'test_isaac_transport.py' -v
../openpi/.venv/bin/python -m unittest discover -s policy_runner/tests
PYTHONPATH=scripts .venv-isaac/bin/python -m unittest scripts/test_shared_replay.py -v
.venv-isaac/bin/python -m compileall -q scripts
```

### 전체 policy runner의 기존 실패

`test_flow_inference_gripper_mode.ChunkKnotFilterTest`의 다음 두 테스트가 실패한다.

- `test_alternating_ripple_is_attenuated`: 표준편차 0.9977753이 기대 상한 0.2496535보다 큼.
- `test_context_carries_across_chunks_instead_of_edge_padding`: filter context가 `None`.

미수정 HEAD `62ee102eedeaa2faac787e39c42fe1e053da0066`의 `policy_runner/`를
`git archive`로 임시 디렉터리에 추출하고, 같은 OpenPI Python으로 해당 클래스
6개 테스트를 실행했다. **동일한 두 assertion과 수치가 그대로 실패했다.**
이번 공유 스택 변경으로 생긴 회귀는 아니며, 기존 필터 동작은 이 커밋에서 변경하지 않았다.

처음 전체 suite를 Isaac 가상환경으로 실행했을 때는 `pytest`/`zmq` 누락도 있었다.
필요 의존성이 있는 OpenPI 가상환경에서 다시 실행하여 위 두 기존 실패로 분리했다.
로컬 원시 로그는 `/tmp/simulation_commit_{build,ctest,isaac_tests,replay_tests,policy_full,baseline_tests}.log`에 남겼다.

## 앞선 실제 Isaac 실행에서 확보한 검증

이 항목은 이번 문서 정리에서 새로 추론한 결과가 아니라, 동일 소스/설정으로
앞서 완료한 실행이다. [결과 디렉터리](results/20260906/README.md)에 요약·영상·hash를 보존한다.

- PhysX 힘/토크 주입 48개 경우: 최대 오차 0.038503 N / 0.028067 Nm.
- 8001 원본 정책 입력 재생: 15,301 physics ticks에서 actual/target 관절 및 속도 차이 0.
- bias 0/0의 8001·8002·8003: 각각 정책 시간 30.002초, 제어기 fault 없이 완료.
- 8003은 결과 저장 후 Kit 종료에서 shell 143을 반환했다. 제어기 exit 0과 구분하며,
  저장 영상은 끝까지 디코딩한 상태다.

이번 커밋 준비에서는 GPU 정책 추론이나 실제 로봇 실행을 반복하지 않았다.
`record_shared_stack.py`는 이전 실행에서 사용한 계측을 CLI로 옮겼으며,
새 CLI의 문법·옵션은 확인했지만 옮긴 후의 Isaac 녹화 자체는 재실행하지 않았다.

## 설정·안전 범위와 남은 작업

실기 `stack_real.yaml`과 `flow_real_realsense.yaml`은 변경하지 않았다.
Real force/safety 한계를 완화하지 않았으며 실제 하드웨어에 연결하거나 명령하지 않았다.
브리지는 명시적으로 외부 stepping을 지원하는 PhysX backend만 사용한다.
OpenPI 서버/학습 코드와 체크포인트는 외부 실행 의존성이며 이 작업에서 커밋하지 않는다.
현재 OpenPI checkout에는 기존 미커밋 변경이 있으므로 HEAD만으로 서버 재현을 보장하지 않는다.

남은 작업은 카메라 intrinsics 보정(fx 약 336→393px), pad/box pose 실측,
그리퍼 모터·접촉 물성 식별, 8002 놓침/8001 오른팔 분석, 여러 장면의 반복 평가다.
이 테스트와 단일 장면 성공은 Real 동등성이나 물리 작업 승인을 뜻하지 않는다.
