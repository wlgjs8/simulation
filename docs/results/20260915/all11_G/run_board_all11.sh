#!/usr/bin/env bash
# All real-deployment boltv2 checkpoints (r7 devjit + everything before it) on std40 under the G photometry
# (t013 lights/materials + Iray tonemap) with the historical cylinder bolts. Available GPUs forced one episode in
# flight per model (not two): 11 servers + 11 Isaac spread 3-4 per GPU across gpu13 (4 GPUs) and gpu12 (3 GPUs),
# so every model sees about the same GPU share. Seeds 100-139, 30 s, measured optics.
# usage: run_board_all11.sh <host-tag> "label port isaac_gpu" ...
set -u
cd /home/plaif/workspace/simulation_rb5
HOSTTAG=$1; shift
ROOT=/home/plaif/eval_all11
LOG=/home/plaif/eval_all11_${HOSTTAG}_progress.log
mkdir -p $ROOT; : > $LOG
echo "$(date -u +%H:%M:%S) === all11 board ($HOSTTAG, G photometry, 1 in flight/model): $* ===" >> $LOG
model_loop(){ # label port gpu
  local label=$1 port=$2 gpu=$3
  for seed in $(seq 100 139); do
    local out=$ROOT/${label}_s${seed}
    rm -rf "$out" /home/plaif/workspace/simulation_rb5/outputs/shared_stack/a11_${label}_s${seed}
    mkdir -p "$out"
    env OMNI_KIT_ACCEPT_EULA=YES PYTHONPATH=/home/plaif/workspace/openpi/packages/openpi-client/src \
        LD_LIBRARY_PATH=/home/plaif/simrig/lib CUDA_VISIBLE_DEVICES=$gpu \
        EVAL_PHOTOMETRY=config/photometry/sep_G_fit_iray_20260914.json \
        timeout 900 /home/plaif/isaac_rig/.venv/bin/python -u scripts/record_shared_stack.py \
        --diagnostics-dir "$out/diag" --shared-stack --episodes 1 --episode-sec 30 --port $port \
        --scene-states /home/plaif/workspace/simulation_rb5/assets/scene_states40_aligned_rb5_foam.json \
        --seed $seed --n-per-color 10 --layout aligned --tag a11_${label}_s${seed} > "$out/run.log" 2>&1
    echo "$(date -u +%H:%M:%S) ${label} s${seed} rc=$? $(grep -m1 -oE 'fx [0-9.]+ px' "$out/run.log")" >> $LOG
  done
  echo "$(date -u +%H:%M:%S) === ${label} 완료 ===" >> $LOG
}
for spec in "$@"; do
  set -- $spec
  model_loop "$1" "$2" "$3" &
  sleep 5
done
wait
echo "$(date -u +%H:%M:%S) === 전체 종료 ===" >> $LOG
