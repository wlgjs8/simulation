#!/usr/bin/env bash
# Separation board (2026-09-14) for the deployed checkpoint devjit40k/39999. A and D are the in-batch
# controls; E = D with the stock gray-bolt material, F = A with t013's gray-bolt material (left-arm
# question: is it the bolt), G = D with Kit's Iray tonemapper instead of the hard clamp (right-arm
# question: is it op 0). One server per condition, two episodes in flight each, seeds 100-139, 30 s.
set -u
cd /home/plaif/workspace/simulation_rb5
ROOT=/home/plaif/eval_sepboard
LOG=/home/plaif/eval_sepboard_progress.log
mkdir -p $ROOT; : > $LOG
echo "$(date -u +%H:%M:%S) === separation board: A 8044 / D 8051 / E 8052 / F 8053 / G 8054 (devjit40k 39999) ===" >> $LOG

run_one(){ # cond port seed gpu preset
  local cond=$1 port=$2 seed=$3 gpu=$4 preset=$5 out=$ROOT/${1}_s${3}
  rm -rf "$out" /home/plaif/workspace/simulation_rb5/outputs/shared_stack/sb2_${cond}_s${seed}
  mkdir -p "$out"
  env OMNI_KIT_ACCEPT_EULA=YES PYTHONPATH=/home/plaif/workspace/openpi/packages/openpi-client/src \
      LD_LIBRARY_PATH=/home/plaif/simrig/lib CUDA_VISIBLE_DEVICES=$gpu EVAL_PHOTOMETRY="$preset" \
      timeout 900 /home/plaif/isaac_rig/.venv/bin/python -u scripts/record_shared_stack.py \
      --diagnostics-dir "$out/diag" --shared-stack --episodes 1 --episode-sec 30 --port $port \
      --scene-states /home/plaif/workspace/simulation_rb5/assets/scene_states40_aligned_rb5_foam.json \
      --seed $seed --n-per-color 10 --layout aligned --tag sb2_${cond}_s${seed} > "$out/run.log" 2>&1
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
cond_loop A 8044 1 "" &
sleep 4
cond_loop D 8051 3 config/photometry/cell_fit_20260912_t013.json &
sleep 4
cond_loop E 8052 4 config/photometry/sep_E_fit_stock_boltgray_20260914.json &
sleep 4
cond_loop F 8053 0 config/photometry/sep_F_stock_fit_boltgray_20260914.json &
sleep 4
cond_loop G 8054 6 config/photometry/sep_G_fit_iray_20260914.json &
wait
echo "$(date -u +%H:%M:%S) === 전체 종료 ===" >> $LOG
