#!/bin/bash
# Phase 1.5 gait-transition sweep: gait.phaseTransitionStanceTime (docs/LOCOMOTION_IMPROVEMENTS.md #1.5).
# GaitScheduleUpdater splices a new template in at the event before an LF phase; for walk->trot
# that deletes the walk's last double support and inserts phaseTransitionStanceTime seconds of
# stance instead (shipped 0.0 -> swing straight into swing); for trot->walk the stance is
# inserted after a swing. stand<->gait changes are unaffected (the outgoing mode is stance).
#
# The 0.5 m/s probe sits on the walk->trot escalation edge (walk band tops at 0.45 m/s, trot
# engages once the measured speed exceeds 0.40), so this sweep defaults to 0.6 m/s to make the
# ramp cross walk->trot on the way up and trot->walk on the way down; plot_tracking.py's `gait`
# section verifies that the transitions happened and scores them (compare_gaitsplice.py).
#
# usage: run_gaitsplice_sweep.sh [vx_mps]        (default 0.6)
#   env: OUT (default <repo>/tuning_results), WS (colcon workspace; default /root/ocs2_ros2_ws
#        in the container), SKIP_BUILD=1, BASELINE=<run dir> (must be a run at the same vx)
# writes: $OUT/<tag>/{diag.json,metrics.json,...}, $OUT/<tag>.yaml, $OUT/gaitsplice_sweep_report.txt
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WS=${WS:-/root/ocs2_ros2_ws}
OUT=${OUT:-$REPO/tuning_results}
VX=${1:-0.6}
mkdir -p "$OUT"

if [ "${SKIP_BUILD:-0}" != 1 ]; then
  echo "== colcon build (legged_robot_mpc_controller)"
  bash -c "source /opt/ros/jazzy/setup.bash && cd '$WS' && colcon build --packages-select legged_robot_mpc_controller" \
    || { echo "RUN_GAITSPLICE_SWEEP: colcon build failed"; exit 1; }
fi

CASES=(
  "gs_t005|ocs2.gait.phaseTransitionStanceTime=0.05"
  "gs_t010|ocs2.gait.phaseTransitionStanceTime=0.1"
  "gs_t015|ocs2.gait.phaseTransitionStanceTime=0.15"
  "gs_t020|ocs2.gait.phaseTransitionStanceTime=0.2"
)

diag() {
  local d=$1
  [ -f "$d/summary.txt" ] && { echo "--- $d/summary.txt"; cat "$d/summary.txt"; }
  [ -f "$d/sim.log" ] && { echo "--- $d/sim.log (FAULT/Traceback/error lines)";
    grep -aE "FAULT|Traceback|Error|error|failed" "$d/sim.log" | head -20; }
}

if [ -n "${BASELINE:-}" ]; then
  BL=$BASELINE
else
  echo "== baseline config gs_base.yaml (shipped + diagnostics.trackingRate)"
  python3 "$HERE/make_config.py" "$OUT/gs_base.yaml" --in "$REPO/config/g1/ros2_controllers.yaml" || exit 1
  echo "== baseline: fresh shipped run at vx=$VX (gs_base)"
  OUT=$OUT bash "$HERE/run_case.sh" gs_base "$OUT/gs_base.yaml" "$VX" || exit 1
  BL=$OUT/gs_base
fi
echo "== baseline: $BL"
if [ ! -f "$BL/metrics.json" ] || [ ! -f "$BL/diag.json" ]; then
  { echo "BASELINE FAILED: $BL needs both metrics.json and diag.json (got: $(ls "$BL" 2>/dev/null | tr '\n' ' '))"
    diag "$BL"; } | tee "$OUT/gaitsplice_sweep_report.txt"
  exit 1
fi
NCH=$(python3 -c "import json; print(json.load(open('$BL/diag.json')).get('gait', {}).get('changes_locomotion', 0))")
if [ "$NCH" = "0" ]; then
  { echo "BASELINE HAS NO GAIT-TO-GAIT TRANSITION at vx=$VX: $(python3 -c "import json; print(json.load(open('$BL/diag.json')).get('gait', {}).get('sequence'))")"
    echo "phaseTransitionStanceTime cannot act; re-run with a higher vx (walk->trot needs measured speed > 0.40 m/s)."
    diag "$BL"; } | tee "$OUT/gaitsplice_sweep_report.txt"
  exit 1
fi
echo "== baseline has $NCH gait-to-gait transition(s)"

CANDS=()
for spec in "${CASES[@]}"; do
  tag=${spec%%|*}
  edits=${spec#*|}
  echo "== config $tag"
  python3 "$HERE/make_config.py" "$OUT/$tag.yaml" --in "$REPO/config/g1/ros2_controllers.yaml" "$edits" || exit 1
  echo "== run $tag"
  OUT=$OUT bash "$HERE/run_case.sh" "$tag" "$OUT/$tag.yaml" "$VX" || exit 1
  if [ ! -f "$OUT/$tag/metrics.json" ] || [ ! -f "$OUT/$tag/diag.json" ]; then
    echo "WARN: $tag missing metrics.json/diag.json (scored as ERROR):"
    diag "$OUT/$tag"
  fi
  CANDS+=("$OUT/$tag")
done

echo
echo "== score"
python3 "$HERE/compare_gaitsplice.py" "$BL" "${CANDS[@]}" | tee "$OUT/gaitsplice_sweep_report.txt"
