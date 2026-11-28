#!/bin/bash
# A/B the solve-handover command blend (control.solveBlendSeconds, Phase 1.1) in the
# ros_jazzy container: the same walk run twice -- blend OFF (0.0) vs the shipped value
# -- scored by plot_tracking.py and gated by compare_blend_ab.py.
#
# usage: run_blend_ab.sh [vx_mps]        (default 0.5)
#   env: OUT (results root, default <repo>/tuning_results), HEIGHT, PROFILE, LAUNCH_ARGS
#        (as run_case.sh), SKIP_BUILD=1 (skip colcon build), RUN_CONTRACT=0 (skip the
#        static contract test), BLEND_ON_SECONDS (default 0.005)
# writes: $OUT/{blend_off,blend_on}/... and $OUT/blend_ab_report.txt
#
# Flow: colcon build -> contract test -> make_config (off/on, tracking 500 Hz) ->
#       run_case.sh x2 -> compare_blend_ab.py gates.
# Methodology step 'test' for docs/LOCOMOTION_IMPROVEMENTS.md #1.1; results are
# documented in docs/results/policy_swap_blend.md.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WS=${WS:-/root/ocs2_ros2_ws}
OUT=${OUT:-$REPO/tuning_results}
VX=${1:-0.5}
ON_S=${BLEND_ON_SECONDS:-0.005}
mkdir -p "$OUT"

if [ "${SKIP_BUILD:-0}" != 1 ]; then
  echo "== colcon build (legged_robot_mpc_controller)"
  bash -c "source /opt/ros/jazzy/setup.bash && cd '$WS' && colcon build --packages-select legged_robot_mpc_controller" \
    || { echo "RUN_BLEND_AB: colcon build failed"; exit 1; }
fi

if [ "${RUN_CONTRACT:-1}" = 1 ]; then
  echo "== contract test"
  if python3 -c 'import pytest, yaml' 2>/dev/null; then
    python3 -m pytest "$REPO/tests/wb_id_policy_blend_contract_test.py" -q \
      || { echo "RUN_BLEND_AB: contract test failed"; exit 1; }
  else
    echo "RUN_BLEND_AB: pytest/yaml unavailable in this container -- contract test skipped (RUN_CONTRACT=0 to silence)"
  fi
fi

echo "== configs (tracking 500 Hz)"
python3 "$HERE/make_config.py" "$OUT/blend_off.yaml" \
  --in "$REPO/config/g1/ros2_controllers.yaml" control.solveBlendSeconds=0.0
python3 "$HERE/make_config.py" "$OUT/blend_on.yaml" \
  --in "$REPO/config/g1/ros2_controllers.yaml" control.solveBlendSeconds=$ON_S

echo "== run blend_off"
OUT=$OUT bash "$HERE/run_case.sh" blend_off "$OUT/blend_off.yaml" "$VX" || exit 1
echo "== run blend_on"
OUT=$OUT bash "$HERE/run_case.sh" blend_on "$OUT/blend_on.yaml" "$VX" || exit 1

echo "== score"
python3 "$HERE/compare_blend_ab.py" "$OUT/blend_off" "$OUT/blend_on" | tee "$OUT/blend_ab_report.txt"
rc=${PIPESTATUS[0]}
echo "report -> $OUT/blend_ab_report.txt"
exit $rc
