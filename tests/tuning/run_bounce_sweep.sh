#!/bin/bash
# Phase 1.2 bounce / vertical-smoothness sweep (docs/LOCOMOTION_IMPROVEMENTS.md #1.2):
# run the shipped G1 walk with candidate cost edits that target the residual
# 4-40 Hz pelvis vertical acceleration (acc_hf ~1.16, baseline 2026-11-24), scored by
# compare_bounce.py against the shipped-config baseline.
#
# usage: run_bounce_sweep.sh [vx_mps]        (default 0.5)
#   env: OUT (default <repo>/tuning_results), WS (colcon workspace; default
#        /root/ocs2_ros2_ws in the container), SKIP_BUILD=1, BASELINE=<run dir>
#   baseline: $BASELINE if set, else existing $OUT/blend_on, else $OUT/blend_off
#             (shipped dynamics; blend does not change bounce), else a fresh
#             run_case.sh bounce_base default run.
# writes: $OUT/bounce_<tag>/{diag.json,metrics.json,...}, $OUT/bounce_<tag>.yaml,
#         $OUT/bounce_sweep_report.txt
#
# Cases (edit in place as the sweep evolves):
#   accz10 / accz100 : r.baseAccelerations z x10 / x100 (never swept by the benchmark)
#   z100             : q+qFinal basePose z 50 -> 100 (300 fell; 100 untested)
#   vz10             : q+qFinal baseVelocity vz 5 -> 10 (benchmark tried 20/30: roll trade-off)
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WS=${WS:-/root/ocs2_ros2_ws}
OUT=${OUT:-$REPO/tuning_results}
VX=${1:-0.5}
mkdir -p "$OUT"

if [ "${SKIP_BUILD:-0}" != 1 ]; then
  echo "== colcon build (legged_robot_mpc_controller)"
  bash -c "source /opt/ros/jazzy/setup.bash && cd '$WS' && colcon build --packages-select legged_robot_mpc_controller" \
    || { echo "RUN_BOUNCE_SWEEP: colcon build failed"; exit 1; }
fi

CASES=(
  "accz10|ocs2.costs.r.baseAccelerations=[0.005,0.005,0.05,0.005,0.005,0.005]"
  "accz100|ocs2.costs.r.baseAccelerations=[0.005,0.005,0.5,0.005,0.005,0.005]"
  "z100|ocs2.costs.q.basePose=[0.0,0.0,100.0,0.0,100.0,100.0]|ocs2.costs.qFinal.basePose=[0.0,0.0,100.0,0.0,100.0,100.0]"
  "vz10|ocs2.costs.q.baseVelocity=[30.0,15.0,10.0,30.0,30.0,30.0]|ocs2.costs.qFinal.baseVelocity=[30.0,15.0,10.0,30.0,30.0,30.0]"
)

if [ -n "${BASELINE:-}" ]; then
  BL=$BASELINE
elif [ -f "$OUT/blend_on/metrics.json" ]; then
  BL=$OUT/blend_on
elif [ -f "$OUT/blend_off/metrics.json" ]; then
  BL=$OUT/blend_off
else
  echo "== baseline: fresh shipped-config run"
  OUT=$OUT bash "$HERE/run_case.sh" bounce_base default "$VX" || exit 1
  BL=$OUT/bounce_base
fi
echo "== baseline: $BL"

CANDS=()
for spec in "${CASES[@]}"; do
  tag=${spec%%|*}
  edits=${spec#*|}
  IFS='|' read -r -a arr <<< "$edits"
  echo "== config bounce_$tag"
  python3 "$HERE/make_config.py" "$OUT/bounce_$tag.yaml" \
    --in "$REPO/config/g1/ros2_controllers.yaml" "${arr[@]}" || exit 1
  echo "== run bounce_$tag"
  OUT=$OUT bash "$HERE/run_case.sh" "bounce_$tag" "$OUT/bounce_$tag.yaml" "$VX" || exit 1
  CANDS+=("$OUT/bounce_$tag")
done

echo "== score"
python3 "$HERE/compare_bounce.py" "$BL" "${CANDS[@]}" | tee "$OUT/bounce_sweep_report.txt"
rc=${PIPESTATUS[0]}
echo "report -> $OUT/bounce_sweep_report.txt"
exit $rc
