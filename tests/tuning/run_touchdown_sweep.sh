#!/bin/bash
# Phase 1.3 touchdown-softening sweep (docs/LOCOMOTION_IMPROVEMENTS.md #1.3):
# G1 walk with candidate swing-touchdown edits targeting the servo touchdown peak
# (legs pd_tau_p99.9 = 50.25 Nm, baseline 2026-11-24), scored by compare_touchdown.py.
#
# usage: run_touchdown_sweep.sh [vx_mps]        (default 0.5)
#   env: OUT (default <repo>/tuning_results), WS (colcon workspace; default
#        /root/ocs2_ros2_ws in the container), SKIP_BUILD=1, BASELINE=<run dir>
#   baseline: $BASELINE if set, else existing $OUT/blend_on, else $OUT/blend_off,
#             else a fresh run_case.sh td_base default run.
# writes: $OUT/td_<tag>/{diag.json,metrics.json,...}, $OUT/td_<tag>.yaml,
#         $OUT/touchdown_sweep_report.txt
#
# Recon notes (swing code, main-era quintic in tree):
#   - node velocity = min(1, swingDuration/swingTimeScale) * config  -> walk: 0.75 * config
#   - default touchDownVelocity 0.0; z-up world: negative = still descending at contact
#     (sign ambiguous in the docs) -> sweep BOTH signs at the same magnitude.
#   - impactProximityFactor* are boundary slopes of the velocity-error gating cubic
#     (value 1 at both ends, mid 0): larger touchdown slope = velocity tracked
#     more tightly approaching contact.
# Cases:
#   tdvn05 / tdvp05 : touchDownVelocity -0.05 / +0.05 (effective -0.0375 / +0.0375 m/s)
#   impf06 / impf10 : impactProximityFactorTouchDownVelocity 0.3 -> 0.6 / 1.0
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
    || { echo "RUN_TOUCHDOWN_SWEEP: colcon build failed"; exit 1; }
fi

CASES=(
  "tdvn05|ocs2.gait.swingTrajectory.touchDownVelocity=-0.05"
  "tdvp05|ocs2.gait.swingTrajectory.touchDownVelocity=0.05"
  "impf06|ocs2.gait.swingTrajectory.impactProximityFactorTouchDownVelocity=0.6"
  "impf10|ocs2.gait.swingTrajectory.impactProximityFactorTouchDownVelocity=1.0"
)

if [ -n "${BASELINE:-}" ]; then
  BL=$BASELINE
elif [ -f "$OUT/blend_on/metrics.json" ]; then
  BL=$OUT/blend_on
elif [ -f "$OUT/blend_off/metrics.json" ]; then
  BL=$OUT/blend_off
else
  echo "== baseline: fresh shipped-config run"
  OUT=$OUT bash "$HERE/run_case.sh" td_base default "$VX" || exit 1
  BL=$OUT/td_base
fi
echo "== baseline: $BL"

CANDS=()
for spec in "${CASES[@]}"; do
  tag=${spec%%|*}
  edits=${spec#*|}
  IFS='|' read -r -a arr <<< "$edits"
  echo "== config td_$tag"
  python3 "$HERE/make_config.py" "$OUT/td_$tag.yaml" \
    --in "$REPO/config/g1/ros2_controllers.yaml" "${arr[@]}" || exit 1
  echo "== run td_$tag"
  OUT=$OUT bash "$HERE/run_case.sh" "td_$tag" "$OUT/td_$tag.yaml" "$VX" || exit 1
  CANDS+=("$OUT/td_$tag")
done

echo "== score"
python3 "$HERE/compare_touchdown.py" "$BL" "${CANDS[@]}" | tee "$OUT/touchdown_sweep_report.txt"
rc=${PIPESTATUS[0]}
echo "report -> $OUT/touchdown_sweep_report.txt"
exit $rc
