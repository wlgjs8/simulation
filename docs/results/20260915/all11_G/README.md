# 실기 후보 11개 모델 std40 — G 조명 + 원통 볼트 (2026-09-14~15)

r7로 학습한 devjit과 그 전의 실기 후보 체크포인트 전부를 한 배치에서 같은 조건으로 채점했다.

## 조건

- 광도: `EVAL_PHOTOMETRY=config/photometry/sep_G_fit_iray_20260914.json`(t013 조명·재질 + Iray 톤매퍼, 두 배치에서 재현된 실기 근접 조건)
- 볼트: 기존 원통(`scene_states40_aligned_rb5_foam.json`). 새 버튼 헤드 형상은 치수 실측 전이라 쓰지 않았다.
- seed 100–139, 30 s, fx 393.55, `record_shared_stack.py`
- 가용 GPU가 7장이라 **모델당 동시 에피소드 1개**로 돌렸다. 이전 보드는 2개였다. 서버 11개와 Isaac 11개를 GPU당 3–4개로
  고르게 나눴다(`run_board_all11.sh`, `serve_models.sh`).
  - gpu13 GPU 0·5·6·7: devjit_r7, r6knormcrop, griponly_r6, plain_r4, ph3, griponly, plain_first
  - gpu12 GPU 5·6·7: devjit_r6, r6knorm, veldrop50, plain_r5
- 두 서버의 sim 코드 md5가 같다. `shared_stack.json`의 robotics_lab 경로도 `../robotics_lab_rb5`로 같다.
- 서빙은 `snap_devjit`이다. 9개 설정 블록의 md5가 실기 로컬 openpi와 같다.
- 로컬에서 옮긴 체크포인트 6개는 manifest md5·크기가 원본과 일치한다.
- 추론 지연(`inference_latency.txt`): 11개 모두 p50 69–74 ms, p90 88–97 ms. 모델·서버 간 차이가 거의 없다.
- 2026-09-14 12:59–17:45 UTC, 440/440 rc=0이다. fault 3건: r6knormcrop s123, ph3 s128, devjit_r6 s138.

| 표기 | 체크포인트 (로컬 `pika_umi_models_v2`) | 설정 |
|---|---|---|
| plain_first | `boltv2_40k` | `boltv2_anchAB_h24_40k` |
| plain_r4 | `boltv2_plain_40k` | `boltv2_anchAB_h24_40k` |
| plain_r5 | `boltv2_r5_40k` | `boltv2_anchAB_h24_40k` |
| ph3 | `boltv2ph3_40k` | `boltv2_anchAB_ph3_h24_40k` |
| griponly | `boltv2_griponly_40k` | `boltv2_anchAB_griponly_h24_40k` |
| veldrop50 | `boltv2_veldrop50_40k` | `boltv2_anchAB_veldrop50_h24_40k` |
| griponly_r6 | `boltv2_griponly_r6_40k` | `boltv2r6_anchAB_griponly_h24_40k` |
| r6knorm | `boltv2_griponly_r6knorm_40k` | `boltv2r6knorm_anchAB_griponly_h24_40k` |
| r6knormcrop | `boltv2_griponly_r6knormcrop_40k` | `boltv2r6knormcrop_anchAB_griponly_h24_40k` |
| devjit_r6 | `boltv2_griponly_devjit_40k` | `boltv2r6_anchAB_griponly_devjit_h24_40k` |
| devjit_r7 | `boltv2_griponly_devjit_r7_40k` | `boltv2r7_anchAB_griponly_devjit_h24_40k` |

## 결과 (`board_table.txt`, 재생성: `python3 board_table.py`)

| 순위 | 모델 | 안착 | 오안착 | 닫기 | 들어올림 | 좌 제 박스 | 좌 박스 밖 놓기 / 들어올림 [95% CI] | 우 제 박스 | 우 박스 밖 놓기 | devjit_r6 대비 시드 짝 |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | **devjit_r6** | **52** | 0 | 141 | 86 | **35** | 0.14 [0.04, 0.26] | 19 | 0.16 | — |
| 2 | **r6knormcrop** | **51** | 0 | 188 | 105 | 34 | 0.27 [0.13, 0.39] | **27** | 0.42 | 9/10, p=1.00 |
| 3 | ph3 | 43 | **5** | 203 | 54 | 16 | 0.00 | 25 | 0.03 | 13/17, p=0.59 |
| 4 | plain_first | 39 | 1 | 211 | 61 | 15 | 0.04 | 24 | 0.05 | 11/17, p=0.35 |
| 5 | griponly | 36 | 0 | 184 | 80 | 15 | 0.30 | 22 | 0.34 | 7/20, p=0.019 |
| 6 | griponly_r6 | 34 | 0 | 155 | 61 | 22 | 0.40 | 16 | 0.23 | 6/19, p=0.015 |
| 7 | plain_r4 | 29 | 0 | 185 | 61 | 10 | 0.00 | 19 | 0.00 | 6/23, p=0.002 |
| 8 | r6knorm | 28 | 0 | 147 | 45 | 12 | 0.06 | 17 | 0.15 | 6/23, p=0.002 |
| 8 | veldrop50 | 28 | 0 | 190 | 60 | 10 | 0.07 | 20 | 0.17 | 6/22, p=0.004 |
| 10 | **devjit_r7** | **25** | 0 | **256** | **115** | **11** | **0.74 [0.61, 0.84]** | 19 | 0.45 | 7/26, **p=0.001** |
| 11 | plain_r5 | 23 | 1 | 166 | 45 | 6 | 0.18 | 17 | 0.07 | 5/25, p<0.001 |

## 판정

1. **devjit_r6와 r6knormcrop이 공동 1위다**(52, 51, 시드 짝 9/10). 나머지 8개는 devjit_r6보다 유의하게 낮거나
   (p ≤ 0.019) 차이가 불확실하다(ph3, plain_first).
2. **devjit_r7은 devjit_r6보다 뚜렷이 낮다**(25 vs 52, 7/26, p=0.001).
   - **차이는 전부 좌팔(회색 볼트)에서 난다.** 좌팔 제 박스 안착 11 vs 35(4/23, p<0.001), 좌팔 박스 밖 놓기
     54 vs 7(20/4, p=0.002)이다. 우팔 안착은 19 vs 19로 같다(p=0.66).
   - r7 좌팔의 행동: 운반 101회 중 박스 위 release는 7회뿐이다. 들어올림 p50 78 mm, 운반 거리 p50 15 mm,
     쥐는 시간 p50 1.0 s, 박스까지 거리 p50 146 mm다. **집어서 조금 들고 1초 안에 더미 근처에 놓는** 동작이다.
     닫기 시도도 가장 많다(256).
   - 원인은 이 보드로 판단하지 못한다. r7에 새로 들어간 데이터(예: 좌팔이 닫힌 채 시작해 공중에서 열고
     다시 잡는 recovery 시연)가 이 행동과 관련 있는지는 데이터 쪽 확인이 필요하다(가설, 미검증).
3. **ph3는 오안착이 5건이다**(다른 모델 0–1). 다른 색 박스에 넣은 경우로, 색-박스 대응이 흔들린다.
4. **G 조건은 배치 간에 안정적이다.** devjit_r6(=devjit40k/39999)가 G 조건 세 배치에서 46(분리) / 53(재현) /
   52(이번)이 나왔다. 반면 기존 리그에서 같은 체크포인트는 29–39로 흔들렸다.

## 주의

- 모델당 동시 1개로 돌려서 이전 보드(동시 2개)와 절대값을 비교하지 않는다. 이 표 안에서만 비교한다.
- 볼트는 원통이다. 회색 버튼 헤드 반영 후 순위가 바뀔 수 있다(좌팔 차이가 회색 볼트에서 나므로 특히).
- 실기 성능과의 대응은 실기 롤아웃 기록으로 확인해야 한다.
