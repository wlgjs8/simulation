#!/usr/bin/env bash
set -u
cd /home/plaif/workspace/simulation_rb5
OUT=/home/plaif/rough_op0; mkdir -p $OUT; : > $OUT/progress.log
MATTE="table.metallic=0;riser.metallic=0;boxgray.metallic=0;table.rgb=0.36,0.38,0.39;boxgray.rgb=0.40,0.41,0.43"
one(){ local tag=$1; local gpu=$2
  rm -rf $OUT/$tag outputs/shared_stack/ro_$tag; mkdir -p $OUT/$tag
  env OMNI_KIT_ACCEPT_EULA=YES PYTHONPATH=/home/plaif/workspace/openpi/packages/openpi-client/src \
      LD_LIBRARY_PATH=/home/plaif/simrig/lib CUDA_VISIBLE_DEVICES=$gpu \
      EVAL_RTX="/rtx/post/tonemap/op=0" EVAL_MAT="$MATTE" \
      EVAL_KEY_INTENSITY=0.89 EVAL_KEY_ANGLE=32 EVAL_DOME_INTENSITY=2.93 \
      EVAL_SUN_INTENSITY=0.90 EVAL_SUN_ANGLE=3.2 EVAL_SUN_AZIMUTH=330 EVAL_SUN_ELEVATION=29 \
      timeout 600 /home/plaif/isaac_rig/.venv/bin/python -u scripts/record_shared_stack.py \
      --diagnostics-dir "$OUT/$tag/diag" --shared-stack --episodes 1 --episode-sec 4 --port 8044 \
      --scene-states /home/plaif/workspace/simulation_rb5/assets/scene_states40_aligned_rb5_foam.json \
      --seed 100 --n-per-color 10 --layout aligned --tag ro_$tag > "$OUT/$tag/run.log" 2>&1
  echo "$(date -u +%H:%M:%S) $tag rc=$?" >> $OUT/progress.log
}
for r in 0.03 0.10 0.25 0.50; do
  python3 /home/plaif/set_tip_finish.py $r >> $OUT/progress.log
  one r$r 0
done
python3 /home/plaif/set_tip_finish.py 0.5 >> $OUT/progress.log
echo DONE >> $OUT/progress.log
