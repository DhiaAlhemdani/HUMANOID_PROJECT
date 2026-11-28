#!/bin/bash
# Phase 1.6 command & upper-body polish sweep (docs/LOCOMOTION_IMPROVEMENTS.md #1.6).
# Two run sets, one build (the S-curve slew limiter is new C++ — do NOT use SKIP_BUILD=1 the
# first time):
#   A  PROFILE=vx @ 0.5 m/s (steady walking + gentle probe ramps):
#        po_base      shipped
#        po_fb        solver.sqp.useFeedbackPolicy=true   (LQR gains between solves; D3 lever)
#        po_jerk1     reference.velocityCommandJerkLimits=[1.0, 1.0, 0.0, 4.0]  (S-curve slew)
#        po_fb_jerk1  both
#      scored: primaries jump@switch <= 0.85x | acc_hf <= 0.92x | command.transient_pitch_max_deg
#      <= 0.85x; default guards.
#   B  PROFILE=stops @ 0.5 m/s (command steps 0<->0.5 every 4 s / 3 s — where a slew limiter acts):
#        ps_base, ps_jerk1 (1.0 m/s^3), ps_jerk05 (0.5 m/s^3)
#      scored: primaries command.transient_pitch_max_deg | transient_acc_hf_max | transient_z_pp_mm_max
#      <= 0.85x; guards survived, vx >= 0.90x, solve >= 0.90x, steady acc_hf/roll not worse.
#
# usage: run_polish_sweep.sh [vx_mps]        (default 0.5)
#   env: OUT, WS, SKIP_BUILD=1, SETS="A B" (default both)
# writes: $OUT/<tag>/..., $OUT/<tag>.yaml, $OUT/polish_sweep_report.txt
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WS=${WS:-/root/ocs2_ros2_ws}
OUT=${OUT:-$REPO/tuning_results}
VX=${1:-0.5}
SETS=${SETS:-"A B"}
mkdir -p "$OUT"

if [ "${SKIP_BUILD:-0}" != 1 ]; then
  echo "== colcon build (legged_robot_mpc_controller)"
  bash -c "source /opt/ros/jazzy/setup.bash && cd '$WS' && colcon build --packages-select legged_robot_mpc_controller" \
    || { echo "RUN_POLISH_SWEEP: colcon build failed"; exit 1; }
fi

JERK1="ocs2.reference.velocityCommandJerkLimits=[1.0, 1.0, 0.0, 4.0]"
JERK05="ocs2.reference.velocityCommandJerkLimits=[0.5, 0.5, 0.0, 2.0]"
FB="ocs2.solver.sqp.useFeedbackPolicy=true"

diag() {
  local d=$1
  [ -f "$d/summary.txt" ] && { echo "--- $d/summary.txt"; cat "$d/summary.txt"; }
  [ -f "$d/sim.log" ] && { echo "--- $d/sim.log (FAULT/Traceback/error lines)";
    grep -aE "FAULT|Traceback|Error|error|failed" "$d/sim.log" | head -20; }
}

# run <tag> <profile> <edits...>
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

: > "$OUT/polish_sweep_report.txt"
if [[ " $SETS " == *" A "* ]]; then
  run po_base vx
  run po_fb vx "$FB"
  run po_jerk1 vx "$JERK1"
  run po_fb_jerk1 vx "$FB" "$JERK1"
  echo; echo "== score set A (steady walk + probe ramps)"
  python3 "$HERE/compare_ab.py" \
    --primary legs.cmd_tau_jump_at_switch:0.85 --primary walk.pelvis_acc_hf:0.92 \
    --primary command.transient_pitch_max_deg:0.85 \
    --show legs.pd_tau_hf --show command.transient_z_pp_mm_max \
    "$OUT/po_base" "$OUT/po_fb" "$OUT/po_jerk1" "$OUT/po_fb_jerk1" | tee -a "$OUT/polish_sweep_report.txt"
fi
if [[ " $SETS " == *" B "* ]]; then
  run ps_base stops
  run ps_jerk1 stops "$JERK1"
  run ps_jerk05 stops "$JERK05"
  echo; echo "== score set B (command steps)"
  python3 "$HERE/compare_ab.py" --no-default-guards \
    --primary command.transient_pitch_max_deg:0.85 --primary command.transient_acc_hf_max:0.85 \
    --primary command.transient_z_pp_mm_max:0.85 \
    --guard walk.vx_mean:min:0.90:0 --guard solver.solve_hz:min:0.90:0 \
    --guard walk.pelvis_acc_hf:max:1.10:0 --guard walk.roll_rms_deg:max:1.15:0.05 \
    --show command.transient_roll_max_deg \
    "$OUT/ps_base" "$OUT/ps_jerk1" "$OUT/ps_jerk05" | tee -a "$OUT/polish_sweep_report.txt"
fi
