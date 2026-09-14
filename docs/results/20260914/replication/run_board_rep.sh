#!/usr/bin/env bash
# Replication board (2026-09-14, gpu13) of the separation board: A, D, E, G on the deployed checkpoint
# devjit40k/39999, same arrangement (one server per condition, two episodes in flight each = 8, seeds
# 100-139, 30 s). The rig is a copy of gpu12's simulation_rb5 / robotics_lab_rb5 / simrig, and the Isaac
# venv package list was matched to gpu12 (h5py 3.16.0, websockets 15.0.1).
set -u
cd /home/plaif/workspace/simulation_rb5
ROOT=/home/plaif/eval_repboard
LOG=/home/plaif/eval_repboard_progress.log
mkdir -p $ROOT; : > $LOG
echo "$(date -u +%H:%M:%S) === replication board (gpu13): A 8061 / D 8062 / E 8063 / G 8064 (devjit40k 39999) ===" >> $LOG

run_one(){ # cond port seed gpu preset
  local cond=$1 port=$2 seed=$3 gpu=$4 preset=$5 out=$ROOT/${1}_s${3}
  rm -rf "$out" /home/plaif/workspace/simulation_rb5/outputs/shared_stack/rb3_${cond}_s${seed}
  mkdir -p "$out"
  env OMNI_KIT_ACCEPT_EULA=YES PYTHONPATH=/home/plaif/workspace/openpi/packages/openpi-client/src \
      LD_LIBRARY_PATH=/home/plaif/simrig/lib CUDA_VISIBLE_DEVICES=$gpu EVAL_PHOTOMETRY="$preset" \
      timeout 900 /home/plaif/isaac_rig/.venv/bin/python -u scripts/record_shared_stack.py \
      --diagnostics-dir "$out/diag" --shared-stack --episodes 1 --episode-sec 30 --port $port \
      --scene-states /home/plaif/workspace/simulation_rb5/assets/scene_states40_aligned_rb5_foam.json \
      --seed $seed --n-per-color 10 --layout aligned --tag rb3_${cond}_s${seed} > "$out/run.log" 2>&1
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
cond_loop A 8061 5 "" &
sleep 4
cond_loop D 8062 6 config/photometry/cell_fit_20260912_t013.json &
sleep 4
cond_loop E 8063 7 config/photometry/sep_E_fit_stock_boltgray_20260914.json &
sleep 4
cond_loop G 8064 0 config/photometry/sep_G_fit_iray_20260914.json &
wait
echo "$(date -u +%H:%M:%S) === 전체 종료 ===" >> $LOG
