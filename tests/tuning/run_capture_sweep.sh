#!/bin/bash
# Phase 2.4 capture-point / Raibert foothold sweep (docs/LOCOMOTION_IMPROVEMENTS.md Phase 2 #4).
# The G1 shipped config leaves foot placement entirely to the optimizer
# (ocs2.costs.captureFootPlacement.enabled=false). Candidates switch the flat-ground xy
# foothold reference on with increasing capture-point gain:
#   cp_g0   enabled, gain 0.0  (Raibert neutral point only, stanceTimeGain 0.5)
#   cp_g03  gain 0.3
#   cp_g06  gain 0.6
#   cp_g10  gain 1.0
# Set A (default): PROFILE=vx @ 0.5 m/s — primaries roll_rms <= 0.85x | acc_hf <= 0.92x |
#   base.roll_track_err_deg <= 0.85x, default guards (compare_ab.py).
# Set B (SETS=B): PROFILE=stops (start/stop balance) with cp_g0 and cp_g06 — primaries
#   command.transient_roll_max_deg | transient_pitch_max_deg | hold acc_hf; guards survived,
#   vx, solve, roll.
# No C++ change: SKIP_BUILD=1 is fine if the install is current.
#
# usage: run_capture_sweep.sh [vx_mps]   env: OUT, WS, SKIP_BUILD=1, SETS="A B" (default A)
# writes: $OUT/<tag>/..., $OUT/<tag>.yaml, $OUT/capture_sweep_report.txt
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WS=${WS:-/root/ocs2_ros2_ws}
OUT=${OUT:-$REPO/tuning_results}
VX=${1:-0.5}
SETS=${SETS:-"A"}
mkdir -p "$OUT"

if [ "${SKIP_BUILD:-0}" != 1 ]; then
  echo "== colcon build (legged_robot_mpc_controller)"
  bash -c "source /opt/ros/jazzy/setup.bash && cd '$WS' && colcon build --packages-select legged_robot_mpc_controller" \
    || { echo "RUN_CAPTURE_SWEEP: colcon build failed"; exit 1; }
fi

ON="ocs2.costs.captureFootPlacement.enabled=true"
G="ocs2.costs.captureFootPlacement.gain"

diag() {
  local d=$1
  [ -f "$d/summary.txt" ] && { echo "--- $d/summary.txt"; cat "$d/summary.txt"; }
  [ -f "$d/sim.log" ] && { echo "--- $d/sim.log (FAULT/Traceback/error lines)";
    grep -aE "FAULT|Traceback|Error|error|failed" "$d/sim.log" | head -20; }
}

run() {
  local tag=$1 profile=$2; shift 2
  echo "== config $tag"
  python3 "$HERE/make_config.py" "$OUT/$tag.yaml" --in "$REPO/config/g1/ros2_controllers.yaml" "$@" || exit 1
  echo "== run $tag ($profile)"
  OUT=$OUT PROFILE=$profile bash "$HERE/run_case.sh" "$tag" "$OUT/$tag.yaml" "$VX" || exit 1
  if [ ! -f "$OUT/$tag/metrics.json" ] || [ ! -f "$OUT/$tag/diag.json" ]; then
    echo "WARN: $tag missing metrics.json/diag.json:"; diag "$OUT/$tag"
  fi
}

: > "$OUT/capture_sweep_report.txt"
if [[ " $SETS " == *" A "* ]]; then
  run cp_base vx
  run cp_g0 vx "$ON" "$G=0.0"
  run cp_g03 vx "$ON" "$G=0.3"
  run cp_g06 vx "$ON" "$G=0.6"
  run cp_g10 vx "$ON" "$G=1.0"
  echo; echo "== score set A (steady walk)"
  python3 "$HERE/compare_ab.py" \
    --primary walk.roll_rms_deg:0.85 --primary walk.pelvis_acc_hf:0.92 --primary base.roll_track_err_deg:0.85 \
    --show walk.vy_mean --show walk.yaw_change_deg --show legs.track_err_q_deg \
    "$OUT/cp_base" "$OUT/cp_g0" "$OUT/cp_g03" "$OUT/cp_g06" "$OUT/cp_g10" | tee -a "$OUT/capture_sweep_report.txt"
fi
if [[ " $SETS " == *" B "* ]]; then
  run cps_base stops
  run cps_g0 stops "$ON" "$G=0.0"
  run cps_g06 stops "$ON" "$G=0.6"
  echo; echo "== score set B (start/stop balance)"
  python3 "$HERE/compare_ab.py" --no-default-guards \
    --primary command.transient_roll_max_deg:0.85 --primary command.transient_pitch_max_deg:0.85 \
    --primary walk.pelvis_acc_hf:0.92 \
    --guard walk.vx_mean:min:0.90:0 --guard solver.solve_hz:min:0.90:0 --guard walk.roll_rms_deg:max:1.15:0.05 \
    --show command.transient_z_pp_mm_max \
    "$OUT/cps_base" "$OUT/cps_g0" "$OUT/cps_g06" | tee -a "$OUT/capture_sweep_report.txt"
fi
