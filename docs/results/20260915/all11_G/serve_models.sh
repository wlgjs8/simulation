#!/usr/bin/env bash
# usage: serve_models.sh "port gpu config_suffix ckpt_dir" ...   (config = pi05_pika_umi_<suffix>, code = snap_devjit)
set -u
cd /home/plaif/workspace/openpi
for spec in "$@"; do
  set -- $spec
  setsid nohup env XLA_PYTHON_CLIENT_PREALLOCATE=false PYTHONPATH=/home/plaif/boltv2/snap_devjit/openpi_snap/src \
    CUDA_VISIBLE_DEVICES=$2 .venv/bin/python /home/plaif/boltv2/snap_devjit/openpi_snap/scripts/serve_policy.py \
    --num-medoid-samples 1 --port $1 policy:checkpoint --policy.config pi05_pika_umi_$3 --policy.dir $4 \
    > /home/plaif/serve_all11_$1.log 2>&1 < /dev/null &
done
