#!/usr/bin/env bash
set -u
cd /home/plaif/workspace/simulation_rb5
OUT=/home/plaif/op_sweep; mkdir -p $OUT; : > $OUT/progress.log
one(){ # tag rtx gpu
  local tag=$1
  rm -rf $OUT/$tag outputs/shared_stack/op_$tag; mkdir -p $OUT/$tag
  env OMNI_KIT_ACCEPT_EULA=YES PYTHONPATH=/home/plaif/workspace/openpi/packages/openpi-client/src \
      LD_LIBRARY_PATH=/home/plaif/simrig/lib CUDA_VISIBLE_DEVICES=$3 EVAL_RTX="$2" \
      timeout 600 /home/plaif/isaac_rig/.venv/bin/python -u scripts/record_shared_stack.py \
      --diagnostics-dir "$OUT/$tag/diag" --shared-stack --episodes 1 --episode-sec 4 --port 8044 \
      --scene-states /home/plaif/workspace/simulation_rb5/assets/scene_states40_aligned_rb5_foam.json \
      --seed 100 --n-per-color 10 --layout aligned --tag op_$tag > "$OUT/$tag/run.log" 2>&1
  echo "$(date -u +%H:%M:%S) $tag rc=$?" >> $OUT/progress.log
  grep -h "\[rtx\]" "$OUT/$tag/run.log" >> $OUT/progress.log
}
for base in 0 3 6; do
  one op$((base+0)) "/rtx/post/tonemap/op=$((base+0))" 0 &
  one op$((base+1)) "/rtx/post/tonemap/op=$((base+1))" 1 &
  one op$((base+2)) "/rtx/post/tonemap/op=$((base+2))" 4 &
  wait
done
echo DONE >> $OUT/progress.log
