#!/bin/bash
# Phase 2.B S1 sweep 3 (docs/results/wbc_s1_sweep.md, "Sweep 3"). Sweep 2's spike anatomy: every
# hip-pitch peak (86 Nm, 2 ms wide, 17 per run) sits on the leg that just touched down, 0 ms after
# the plan's contact switch, and is 55 Nm of layer correction over a 30 Nm plan torque -- the layer
# swaps its contact set with the plan mode while the robot is still in the old contact state.
# One lever, two settings, from the default (soft) set:
#   wb3_soft     the defaults (control: sweep 1/2 noise floor)
#   wb3_fade30   contactBlendSeconds 0.03: layer torque faded into the plan torque +-30 ms around
#                every planned contact switch (smoothstep)
#   wb3_fade60   contactBlendSeconds 0.06
# Gates unchanged from sweep 2 (declared before sweep 2 ran, not moved):
# Set A (default): PROFILE=vx @ 0.5 -- primaries acc_hf <= 0.92x | legs.meas_tau_hf <= 0.85x |
#   legs.tau_sat_pct <= 0.5x+0.02 | base.roll_track_err_deg <= 0.85x; guards as listed in GUARDS.
# Set B (SETS=B): PROFILE=stops with both fades -- transient pitch is a primary, not-worse 1.1x.
# C++ change: needs the build (no SKIP_BUILD=1 unless the install is already current).
#
# usage: run_wbc_sweep3.sh [vx_mps]   env: OUT, WS, SKIP_BUILD=1, SETS="A B" (default A)
# writes: $OUT/<tag>/..., $OUT/<tag>.yaml, $OUT/wbc_sweep3_report.txt (+ per-set _A/_B copies)
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
    || { echo "RUN_WBC_SWEEP3: colcon build failed"; exit 1; }
fi

WB=control.wholeBody
ON="$WB.enabled=true"
FADE30=("$WB.contactBlendSeconds=0.03")
FADE60=("$WB.contactBlendSeconds=0.06")

diag() {
  local d=$1
  [ -f "$d/summary.txt" ] && { echo "--- $d/summary.txt"; cat "$d/summary.txt"; }
  [ -f "$d/sim.log" ] && { echo "--- $d/sim.log (FAULT/Traceback/error/whole-body lines)";
    grep -aE "FAULT|Traceback|Error|error|failed|whole-body" "$d/sim.log" | head -20; }
}

run() {
  local tag=$1 profile=$2; shift 2
  echo "== config $tag"
  python3 "$HERE/make_config.py" "$OUT/$tag.yaml" --in "$REPO/config/g1/ros2_controllers.yaml" "$@" || exit 1
  local attempt
  for attempt in 1 2; do
    echo "== run $tag ($profile) attempt $attempt"
    OUT=$OUT PROFILE=$profile bash "$HERE/run_case.sh" "$tag" "$OUT/$tag.yaml" "$VX" || exit 1
    [ -f "$OUT/$tag/metrics.json" ] && [ -f "$OUT/$tag/diag.json" ] && break
    echo "WARN: $tag produced no metrics (sim did not come up?):"; diag "$OUT/$tag"
    if [ $attempt = 1 ]; then
      # A stale simulator / controller manager from the previous case is the usual cause.
      pkill -9 -f "ros2 launch legged_robot_mpc_controller" 2>/dev/null; pkill -9 -x mujoco_ros2_con 2>/dev/null
      pkill -9 -f controller_manager 2>/dev/null; pkill -9 -f robot_state_publisher 2>/dev/null; sleep 5
      echo "== retrying $tag"
    fi
  done
  [ -f "$OUT/$tag/sim.log" ] && grep -aE "whole-body feedback (layer on|solve invalid)" "$OUT/$tag/sim.log" | tail -2
}

GUARDS=(--no-default-guards
  --guard walk.roll_rms_deg:max:1.15:0.05 --guard walk.pitch_rms_deg:max:1.15:0.05
  --guard legs.cmd_tau_jump_at_switch:max:1.15:0.02 --guard legs.track_err_q_deg:max:1.5:0.1
  --guard base.plan_z_sd_mm:max:1.10:0.5 --guard base.z_track_err_mm:max:1.10:0.5
  --guard base.pitch_track_err_deg:max:1.2:0.05 --guard walk.z_sd_mm:max:1.15:0.3
  --guard walk.arm_swing_pp_deg:max:1.20:1.0 --guard walk.vx_mean:min:0.90:0 --guard solver.solve_hz:min:0.90:0)

REPORT="$OUT/wbc_sweep3_report.txt"
if [[ " $SETS " == *" A "* ]]; then
  run wb3_base vx
  run wb3_soft vx "$ON"
  run wb3_fade30 vx "$ON" "${FADE30[@]}"
  run wb3_fade60 vx "$ON" "${FADE60[@]}"
  echo; echo "== score set A (steady walk)"
  python3 "$HERE/compare_ab.py" "${GUARDS[@]}" \
    --primary walk.pelvis_acc_hf:0.92 --primary legs.meas_tau_hf:0.85 --primary legs.tau_sat_pct:0.5:0.02 \
    --primary base.roll_track_err_deg:0.85 \
    --show base.pitch_track_err_deg --show walk.pitch_rms_deg --show legs.track_err_q_deg --show walk.z_sd_mm \
    "$OUT/wb3_base" "$OUT/wb3_soft" "$OUT/wb3_fade30" "$OUT/wb3_fade60" \
    | tee "$OUT/wbc_sweep3_report_A.txt" | tee -a "$REPORT"
  echo; echo "== per-joint diagnostics"
  python3 "$HERE/wbc_diag.py" "$OUT/wb3_base" "$OUT/wb3_soft" "$OUT/wb3_fade30" "$OUT/wb3_fade60" \
    > "$OUT/wbc_diag3_A.txt" && echo "wrote $OUT/wbc_diag3_A.txt"
fi
if [[ " $SETS " == *" B "* ]]; then
  run wbs3_base stops
  run wbs3_fade30 stops "$ON" "${FADE30[@]}"
  run wbs3_fade60 stops "$ON" "${FADE60[@]}"
  echo; echo "== score set B (start/stop balance)"
  python3 "$HERE/compare_ab.py" --no-default-guards \
    --primary command.transient_roll_max_deg:0.85 --primary command.transient_pitch_max_deg:0.85 \
    --primary walk.pelvis_acc_hf:0.92 --primary legs.tau_sat_pct:0.5:0.02 \
    --guard walk.vx_mean:min:0.90:0 --guard solver.solve_hz:min:0.90:0 --guard walk.roll_rms_deg:max:1.15:0.05 \
    --guard command.transient_z_pp_mm_max:max:1.15:2.0 \
    --show command.transient_z_pp_mm_max --show legs.meas_tau_hf \
    "$OUT/wbs3_base" "$OUT/wbs3_fade30" "$OUT/wbs3_fade60" \
    | tee "$OUT/wbc_sweep3_report_B.txt" | tee -a "$REPORT"
  python3 "$HERE/wbc_diag.py" "$OUT/wbs3_base" "$OUT/wbs3_fade30" "$OUT/wbs3_fade60" > "$OUT/wbc_diag3_B.txt" && echo "wrote $OUT/wbc_diag3_B.txt"
fi
