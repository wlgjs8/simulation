#!/usr/bin/env bash
# std20 on the three checkpoints currently served locally, one after another.
#
# ORDER IS DELIBERATE. boltv2 and boltv2ph3 are the same data campaign differing only in the
# phase-3 decimation, so they are the pair that has to BOTH exist for the comparison to mean
# anything; they go first. anchAB is a reference point, not a comparison partner, so it goes
# last -- if the queue is interrupted, the pair survives.
#
# SEQUENTIAL, NOT PARALLEL. Two Isaac instances plus the three policy servers exhaust the
# 5090's 32 GB: measured 2026-09-04, a second instance died with "Buffer creation failed for
# the device: 0" and took the renderer down with it. One at a time.
#
# ACTION_MODE=anchored for all three -- every one of these checkpoints is trained anchored
# (dataset names carry _tcp_anchored_). The rig's contract gate would abort otherwise, which
# is the point, but setting it here means the queue does not stop on the first run.
#
# TIP: this scores on PIKA_TIP=v15, the tip the robot wears. Note the confound that leaves on
# anchAB: it was trained on the OLD campaign, before the printed tip, so v15 puts it out of
# distribution on the fingers. That is also true on the real robot today -- the hardware wears
# v15 whichever checkpoint you serve -- so v15 is the deployment-faithful condition for all
# three. To separate "older recipe is worse" from "older model is tip-OOD", re-run anchAB
# alone with PIKA_TIP=orig and compare against its own v15 score.
set -uo pipefail
SIM=/home/plaif/workspace/simulation
cd "$SIM" || exit 1

export PYTHONPATH=/home/plaif/workspace/openpi/packages/openpi-client/src
export OMNI_KIT_ACCEPT_EULA=YES
export ACTION_MODE=anchored
SCENES=assets/scene_states40_aligned.json      # superset of std20's seeds 100-119
LAYOUT=${LAYOUT:-aligned}
LOG=/home/plaif/std20_three.log

# label            port  what it is
RUNS=(
  "boltv2_ph3      8002  new bolt_v2 campaign + phase-3 decimation"
  "boltv2          8001  new bolt_v2 campaign"
  "anchAB_olddata  8003  previous champion, OLD campaign (tip-OOD, see header)"
)

say() { echo "=== $(date -u +%H:%M:%S) $* ===" | tee -a "$LOG"; }
say "std20 x3 START  layout=$LAYOUT  tip=${PIKA_TIP:-v15}  action_mode=$ACTION_MODE"

rc_all=0
for row in "${RUNS[@]}"; do
  read -r label port desc <<<"$row"
  say "RUN $label (:$port) -- $desc"
  PROTOCOL=std20 scripts/run_std20.sh "$label" 127.0.0.1 "$port" "$LAYOUT" \
      --scene-states "$SCENES" --rtc
  rc=$?
  say "END $label rc=$rc"
  [ "$rc" -ne 0 ] && rc_all=$rc
done

say "std20 x3 DONE (worst rc=$rc_all)"
say "compare: .venv-isaac/bin/python scripts/t1_report.py --table outputs/eval/summary_{boltv2_ph3,boltv2,anchAB_olddata}_${LAYOUT}.json"
exit "$rc_all"
