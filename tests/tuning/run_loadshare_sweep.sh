#!/bin/bash
# Phase 1.3 structural (D2/D3) sweep: the double-support load-share cost
# (docs/LOCOMOTION_IMPROVEMENTS.md #1.3 force-rate lever).
# G1 walk with candidate ocs2.costs.contactLoadShareWeight values, scored by
# compare_loadshare.py (primary: legs.cmd_tau_jump_at_switch <= 0.85x, D3).
#
# usage: run_loadshare_sweep.sh [vx_mps]        (default 0.5)
#   env: OUT (default <repo>/tuning_results), WS (colcon workspace; default
#        /root/ocs2_ros2_ws in the container), SKIP_BUILD=1, BASELINE=<run dir>
#   baseline: $BASELINE if set, else a FRESH run_case.sh ls_base default run —
#             required because shipped config changed (touchDownVelocity -0.05),
#             so the old blend_on dirs are stale for jump/peak comparisons.
# writes: $OUT/ls_<tag>/{diag.json,metrics.json,...}, $OUT/ls_<tag>.yaml,
#         $OUT/loadshare_sweep_report.txt
#
# Cases: log-spaced weights around the existing fz input-cost scale
# (r.contactWrenches fz = 0.01 * rScaling = 1e-5 per N^2):
#   1e-6 (sub-noise) .. 1e-3 (dominant). weight 0 = cost not registered.
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
    || { echo "RUN_LOADSHARE_SWEEP: colcon build failed"; exit 1; }
fi

CASES=(
  "ls_w1e6|ocs2.costs.contactLoadShareWeight=0.000001"
  "ls_w1e5|ocs2.costs.contactLoadShareWeight=0.00001"
  "ls_w1e4|ocs2.costs.contactLoadShareWeight=0.0001"
  "ls_w1e3|ocs2.costs.contactLoadShareWeight=0.001"
)

# diag <dir>: one-glance failure evidence for a run that never produced metrics.json.
diag() {
  local d=$1
  echo "-- $d"
  [ -f "$d/summary.txt" ] && tail -n 8 "$d/summary.txt"
  grep -aE "FAULT|Traceback|on_configure failed|Segmentation|what\(\)|no locomotion|RUN:" "$d/sim.log" 2>/dev/null | tail -n 8
}

if [ -n "${BASELINE:-}" ]; then
  BL=$BASELINE
else
  # The baseline MUST go through make_config too: the shipped config has no
  # diagnostics.trackingRate, so a plain `default` run bags the tracking topic at
  # 0 Hz and plot_tracking writes no diag.json (jump/peak gates unreadable).
  # make_config adds trackingRate=500 — diagnostics only, dynamics unchanged.
  echo "== baseline config ls_base.yaml (shipped + diagnostics.trackingRate)"
  python3 "$HERE/make_config.py" "$OUT/ls_base.yaml" \
    --in "$REPO/config/g1/ros2_controllers.yaml" || exit 1
  echo "== baseline: fresh shipped-config run (ls_base)"
  OUT=$OUT bash "$HERE/run_case.sh" ls_base "$OUT/ls_base.yaml" "$VX" || exit 1
  BL=$OUT/ls_base
fi
echo "== baseline: $BL"
if [ ! -f "$BL/metrics.json" ] || [ ! -f "$BL/diag.json" ]; then
  # Abort early: with no baseline the sweep cannot be scored (the cost is OFF at
  # weight 0, so candidates would only repeat an environment failure).
  { echo "BASELINE FAILED: $BL needs both metrics.json and diag.json (got: $(ls "$BL" 2>/dev/null | tr '\n' ' '))"
    diag "$BL"; } | tee "$OUT/loadshare_sweep_report.txt"
  exit 1
fi

CANDS=()
for spec in "${CASES[@]}"; do
  tag=${spec%%|*}
  edits=${spec#*|}
  IFS='|' read -r -a arr <<< "$edits"
  echo "== config $tag"
  python3 "$HERE/make_config.py" "$OUT/$tag.yaml" \
    --in "$REPO/config/g1/ros2_controllers.yaml" "${arr[@]}" || exit 1
  echo "== run $tag"
  OUT=$OUT bash "$HERE/run_case.sh" "$tag" "$OUT/$tag.yaml" "$VX" || exit 1
  if [ ! -f "$OUT/$tag/metrics.json" ] || [ ! -f "$OUT/$tag/diag.json" ]; then
    echo "WARN: $tag missing metrics.json/diag.json (scored as ERROR):"
    diag "$OUT/$tag"
  fi
  CANDS+=("$OUT/$tag")
done

echo "== score"
python3 "$HERE/compare_loadshare.py" "$BL" "${CANDS[@]}" | tee "$OUT/loadshare_sweep_report.txt"
rc=${PIPESTATUS[0]}
echo "report -> $OUT/loadshare_sweep_report.txt"
exit $rc
