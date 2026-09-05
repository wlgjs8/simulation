#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export OMNI_KIT_ACCEPT_EULA=YES
export PYTHONPATH="$PWD/../openpi/packages/openpi-client/src${PYTHONPATH:+:$PYTHONPATH}"
exec .venv-isaac/bin/python -u scripts/eval_closed_loop.py --shared-stack --episodes 1 \
  --episode-sec 5 --scene-states "$PWD/assets/scene_states40_aligned_rb5_foam.json" \
  --tag "shared_$(date +%Y%m%d_%H%M%S)" "$@"
