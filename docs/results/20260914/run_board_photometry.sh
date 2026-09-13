#!/usr/bin/env bash
# Photometry A-D board for the deployed checkpoint (boltv2_griponly_devjit_40k/39999, the model the
# robot runs at 80-90% pick & place). Same arrangement as the 2026-09-11 sample board: one server per
# condition, two episodes in flight per condition = 8, seeds 100-139, 30 s, measured optics.
# Every server holds the same checkpoint; a condition differs from another only by EVAL_PHOTOMETRY.
set -u
cd /home/plaif/workspace/simulation_rb5
ROOT=/home/plaif/eval_photoboard
LOG=/home/plaif/eval_photoboard_progress.log
mkdir -p $ROOT; : > $LOG
echo "$(date -u +%H:%M:%S) === photometry board: A 8044 / B 8051 / C 8052 / D 8053 (devjit40k 39999) ===" >> $LOG

run_one(){ # cond port seed gpu preset
  local cond=$1 port=$2 seed=$3 gpu=$4 preset=$5 out=$ROOT/${1}_s${3}
  rm -rf "$out" /home/plaif/workspace/simulation_rb5/outputs/shared_stack/pb_${cond}_s${seed}
  mkdir -p "$out"
  env OMNI_KIT_ACCEPT_EULA=YES PYTHONPATH=/home/plaif/workspace/openpi/packages/openpi-client/src \
      LD_LIBRARY_PATH=/home/plaif/simrig/lib CUDA_VISIBLE_DEVICES=$gpu EVAL_PHOTOMETRY="$preset" \
      timeout 900 /home/plaif/isaac_rig/.venv/bin/python -u scripts/record_shared_stack.py \
      --diagnostics-dir "$out/diag" --shared-stack --episodes 1 --episode-sec 30 --port $port \
      --scene-states /home/plaif/workspace/simulation_rb5/assets/scene_states40_aligned_rb5_foam.json \
      --seed $seed --n-per-color 10 --layout aligned --tag pb_${cond}_s${seed} > "$out/run.log" 2>&1
  echo "$(date -u +%H:%M:%S) ${cond} s${seed} rc=$? $(grep -m1 -oE 'fx [0-9.]+ px' "$out/run.log")" >> $LOG
}
cond_loop(){ # cond port gpu preset
  local cond=$1 port=$2 gpu=$3 preset=$4
  for seed in $(seq 100 139); do
    while [ "$(jobs -rp | wc -l)" -ge 2 ]; do sleep 5; done
    run_one "$cond" "$port" "$seed" "$gpu" "$preset" &
    sleep 2
  done
  wait
  echo "$(date -u +%H:%M:%S) === ${cond} 완료 ===" >> $LOG
}
cond_loop A 8044 0 "" &
sleep 4
cond_loop B 8051 1 config/photometry/ladder_B_tonemap_20260913.json &
sleep 4
cond_loop C 8052 3 config/photometry/ladder_C_light_20260913.json &
sleep 4
cond_loop D 8053 4 config/photometry/cell_fit_20260912_t013.json &
wait
echo "$(date -u +%H:%M:%S) === 전체 종료 ===" >> $LOG
