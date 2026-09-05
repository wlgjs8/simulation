# 폼 패드 색상 보정

실제 Pika 최근 두 세션의 양쪽 손목 RGB를 확인하고, 패드 재질의 `texture_rgb_scale`을
`[0.70, 0.75, 0.95]`로 변경했다. 기존 Sim의 밝고 노란/갈색 기를 줄이고 어두운 회색으로
맞춘 외관 조정이다. 원본 PNG는 유지했으며, 사진의 조명/white balance를 포함하는 값이므로
정밀한 albedo 또는 조명 캘리브레이션 결과라고 해석하지 않는다.

변경: `config/work_surface.json`, `scripts/work_surface.py`,
`assets/scene_states40_aligned_rb5_foam.json`의 외관 메타데이터, `docs/shared_stack.md`.
기존 40개 장면/800개 볼트의 좌표·자세 및 나머지 레코드가 완전히 동일함을 확인했다.
패드 치수·위치·접촉 물성, 텍스처, Real 설정과 제어기는 변경하지 않았다.

최종 확인은 새 Isaac 프로세스에서 동일한 seed 100/reset/카메라로 수행했다.
40회 `delta_time=0` 렌더 후 이미지를 저장했고 렌더 중 물리 tick/관절값이
변하지 않았음을 검사했다. 같은 프로세스에서 재질을 반복 변경하면 렌더의 시간 누적
때문에 후보 비교에 잔상이 남는 것이 관찰돼, 최종 이미지는 새 프로세스에서 검증했다.
Python compile 및 diff whitespace 검사도 통과했다. 색상 변경이므로 새 정책 실행이나
성공률 측정은 하지 않았다.

- `before_after.png`: 동일 overview 카메라의 패드 부분 비교.
- `real_before_after.png`: 실제 RGB와 변경 전/후 손목 RGB. Real과 Sim의 손목 자세는 다르다.
- `after_overview.png`, `after_left.png`, `after_right.png`: 최종 실제 렌더.
- `real_samples.json`, `real_sample_rois.png`: 검토한 실사 영역.
- `work_surface_before.json`, `unchanged_layout.json`, `verification.json`: 원본 설정과 보존/검증 기록.

후속 작업면 기본 설정에 적용되어 있다. 이전에 녹화한 MP4는 당시 외관을 그대로 보존한다.
