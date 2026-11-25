#!/bin/bash
# Phase 2.B S1 sweep 2 (docs/results/wbc_s1_sweep.md, "Decisions"). Sweep 1's diagnostics: the
# pelvis base task folds the torso at the waist (posture weight 1 vs base 10) -> pitch drift,
# hip-pitch torque peaks 86 Nm, falls at the stiffer gains. One change per candidate, all from
# the (now default) soft set:
#   wb2_soft     the defaults (repeat of sweep 1's wb_s1_soft: noise floor)
#   wb2_waist    waist joints' posture weight 50 (others 1)
#   wb2_torso    torso_link orientation task (kp 100/100/25, w 10) replaces the pelvis pitch/roll
#                task (base kp/kd pitch, roll = 0; z and yaw kept)
#   wb2_lowbase  pelvis pitch/roll task x0.25 (kp 25, kd 10) - the stiffness-ladder direction
# Set A (default): PROFILE=vx @ 0.5 — primaries acc_hf <= 0.92x | legs.meas_tau_hf <= 0.85x |
#   legs.tau_sat_pct <= 0.5x+0.02 | base.roll_track_err_deg <= 0.85x. Guards as sweep 1 plus
#   walk.z_sd_mm <= 1.15x+0.3, walk.pitch_rms_deg <= 1.15x+0.05, base.pitch_track_err_deg <= 1.2x+0.05.
# Set B (SETS=B): PROFILE=stops with wb2_waist and wb2_torso — transient pitch is a primary and
#   may not exceed 1.1x the baseline.
# C++ change: needs the build (no SKIP_BUILD=1 unless the install is already current).
#
# usage: run_wbc_sweep2.sh [vx_mps]   env: OUT, WS, SKIP_BUILD=1, SETS="A B" (default A)
# writes: $OUT/<tag>/..., $OUT/<tag>.yaml, $OUT/wbc_sweep2_report.txt (+ per-set _A/_B copies)
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
    || { echo "RUN_WBC_SWEEP2: colcon build failed"; exit 1; }
fi

WB=control.wholeBody
ON="$WB.enabled=true"
# robot.jointNames order: 6 left leg, 6 right leg, waist yaw/roll/pitch, 4 left arm, 4 right arm.
WAIST=("$WB.jointWeights=[1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,50.0,50.0,50.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0,1.0]")
TORSO=("$WB.orientationLink=torso_link" "$WB.baseKp=[0.0,0.0,50.0,25.0,0.0,0.0]" "$WB.baseKd=[0.0,0.0,14.0,10.0,0.0,0.0]")
LOWBASE=("$WB.baseKp=[0.0,0.0,50.0,25.0,25.0,25.0]" "$WB.baseKd=[0.0,0.0,14.0,10.0,10.0,10.0]")

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

REPORT="$OUT/wbc_sweep2_report.txt"
if [[ " $SETS " == *" A "* ]]; then
  run wb2_base vx
  run wb2_soft vx "$ON"
  run wb2_waist vx "$ON" "${WAIST[@]}"
  run wb2_torso vx "$ON" "${TORSO[@]}"
  run wb2_lowbase vx "$ON" "${LOWBASE[@]}"
  echo; echo "== score set A (steady walk)"
  python3 "$HERE/compare_ab.py" "${GUARDS[@]}" \
    --primary walk.pelvis_acc_hf:0.92 --primary legs.meas_tau_hf:0.85 --primary legs.tau_sat_pct:0.5:0.02 \
    --primary base.roll_track_err_deg:0.85 \
    --show base.pitch_track_err_deg --show walk.pitch_rms_deg --show legs.track_err_q_deg --show walk.z_sd_mm \
    "$OUT/wb2_base" "$OUT/wb2_soft" "$OUT/wb2_waist" "$OUT/wb2_torso" "$OUT/wb2_lowbase" \
    | tee "$OUT/wbc_sweep2_report_A.txt" | tee -a "$REPORT"
  echo; echo "== per-joint diagnostics"
  python3 "$HERE/wbc_diag.py" "$OUT/wb2_base" "$OUT/wb2_soft" "$OUT/wb2_waist" "$OUT/wb2_torso" "$OUT/wb2_lowbase" \
    > "$OUT/wbc_diag2_A.txt" && echo "wrote $OUT/wbc_diag2_A.txt"
fi
if [[ " $SETS " == *" B "* ]]; then
  run wbs2_base stops
  run wbs2_waist stops "$ON" "${WAIST[@]}"
  run wbs2_torso stops "$ON" "${TORSO[@]}"
  echo; echo "== score set B (start/stop balance)"
  python3 "$HERE/compare_ab.py" --no-default-guards \
    --primary command.transient_roll_max_deg:0.85 --primary command.transient_pitch_max_deg:0.85 \
    --primary walk.pelvis_acc_hf:0.92 --primary legs.tau_sat_pct:0.5:0.02 \
    --guard walk.vx_mean:min:0.90:0 --guard solver.solve_hz:min:0.90:0 --guard walk.roll_rms_deg:max:1.15:0.05 \
    --guard command.transient_z_pp_mm_max:max:1.15:2.0 \
    --show command.transient_z_pp_mm_max --show legs.meas_tau_hf \
    "$OUT/wbs2_base" "$OUT/wbs2_waist" "$OUT/wbs2_torso" \
    | tee "$OUT/wbc_sweep2_report_B.txt" | tee -a "$REPORT"
  python3 "$HERE/wbc_diag.py" "$OUT/wbs2_base" "$OUT/wbs2_waist" "$OUT/wbs2_torso" > "$OUT/wbc_diag2_B.txt" && echo "wrote $OUT/wbc_diag2_B.txt"
fi
