#!/bin/bash
# Phase 2.B S1 whole-body feedback layer sweep (docs/design/wbc_layer.md §5).
# Baseline: shipped servo path (joint PD kp 200 / kd 20, feedforward policy_torque).
# Candidates switch control.wholeBody.enabled on (contact-consistent torque at the measured
# state, residual PD kp 20 / kd 3) and vary the task stiffness and the residual servo:
#   wb_s1        shipped wholeBody gains (base kp z 100, pitch/roll 200; joint 100; swing 200)
#   wb_s1_soft   base and swing task gains x0.5 (damping x0.71)
#   wb_s1_stiff  base and swing task gains x2   (damping x1.41)
#   wb_s1_pd50   shipped task gains, residual PD kp 50 / kd 5
# Set A (default): PROFILE=vx @ 0.5 m/s — primaries acc_hf <= 0.92x | legs.meas_tau_hf <= 0.85x |
#   legs.tau_sat_pct <= 0.5x | base.roll_track_err_deg <= 0.85x. Explicit guards (the default
#   set minus pd_tau_hf, which is not comparable across servo architectures, with joint tracking
#   relaxed to 1.5x + 0.1 deg: the layer is meant to trade joint stiffness for contact consistency).
# Set B (SETS=B): PROFILE=stops with wb_s1 and wb_s1_soft — start/stop balance.
# C++ change: needs the build (no SKIP_BUILD=1 unless the install is already current).
#
# usage: run_wbc_sweep.sh [vx_mps]   env: OUT, WS, SKIP_BUILD=1, SETS="A B" (default A)
# writes: $OUT/<tag>/..., $OUT/<tag>.yaml, $OUT/wbc_sweep_report.txt (+ per-set _A/_B copies)
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
    || { echo "RUN_WBC_SWEEP: colcon build failed"; exit 1; }
fi

WB=control.wholeBody
ON="$WB.enabled=true"
SOFT=("$WB.baseKp=[0.0,0.0,50.0,25.0,100.0,100.0]" "$WB.baseKd=[0.0,0.0,14.0,10.0,20.0,20.0]"
      "$WB.swingKp=[100.0,100.0,100.0]" "$WB.swingKd=[20.0,20.0,20.0]")
STIFF=("$WB.baseKp=[0.0,0.0,200.0,100.0,400.0,400.0]" "$WB.baseKd=[0.0,0.0,28.0,20.0,40.0,40.0]"
       "$WB.swingKp=[400.0,400.0,400.0]" "$WB.swingKd=[40.0,40.0,40.0]")
PD50=("$WB.residualKp=50.0" "$WB.residualKd=5.0")

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
  echo "== run $tag ($profile)"
  OUT=$OUT PROFILE=$profile bash "$HERE/run_case.sh" "$tag" "$OUT/$tag.yaml" "$VX" || exit 1
  if [ ! -f "$OUT/$tag/metrics.json" ] || [ ! -f "$OUT/$tag/diag.json" ]; then
    echo "WARN: $tag missing metrics.json/diag.json:"; diag "$OUT/$tag"
  fi
  # The layer's own health: fallback ticks and the first engage line.
  [ -f "$OUT/$tag/sim.log" ] && grep -aE "whole-body feedback (layer on|solve invalid)" "$OUT/$tag/sim.log" | tail -2
}

GUARDS=(--no-default-guards
  --guard walk.roll_rms_deg:max:1.15:0.05 --guard legs.cmd_tau_jump_at_switch:max:1.15:0.02
  --guard legs.track_err_q_deg:max:1.5:0.1 --guard base.plan_z_sd_mm:max:1.10:0.5
  --guard base.z_track_err_mm:max:1.10:0.5 --guard walk.arm_swing_pp_deg:max:1.20:1.0
  --guard walk.vx_mean:min:0.90:0 --guard solver.solve_hz:min:0.90:0)

REPORT="$OUT/wbc_sweep_report.txt"
if [[ " $SETS " == *" A "* ]]; then
  run wb_base vx
  run wb_s1 vx "$ON"
  run wb_s1_soft vx "$ON" "${SOFT[@]}"
  run wb_s1_stiff vx "$ON" "${STIFF[@]}"
  run wb_s1_pd50 vx "$ON" "${PD50[@]}"
  echo; echo "== score set A (steady walk)"
  python3 "$HERE/compare_ab.py" "${GUARDS[@]}" \
    --primary walk.pelvis_acc_hf:0.92 --primary legs.meas_tau_hf:0.85 --primary legs.tau_sat_pct:0.5:0.02 \
    --primary base.roll_track_err_deg:0.85 \
    --show base.pitch_track_err_deg --show legs.track_err_q_deg --show legs.pd_tau_hf --show walk.z_sd_mm \
    "$OUT/wb_base" "$OUT/wb_s1" "$OUT/wb_s1_soft" "$OUT/wb_s1_stiff" "$OUT/wb_s1_pd50" \
    | tee "$OUT/wbc_sweep_report_A.txt" | tee -a "$REPORT"
fi
if [[ " $SETS " == *" B "* ]]; then
  run wbs_base stops
  run wbs_s1 stops "$ON"
  run wbs_s1_soft stops "$ON" "${SOFT[@]}"
  echo; echo "== score set B (start/stop balance)"
  python3 "$HERE/compare_ab.py" --no-default-guards \
    --primary command.transient_roll_max_deg:0.85 --primary command.transient_pitch_max_deg:0.85 \
    --primary walk.pelvis_acc_hf:0.92 --primary legs.tau_sat_pct:0.5:0.02 \
    --guard walk.vx_mean:min:0.90:0 --guard solver.solve_hz:min:0.90:0 --guard walk.roll_rms_deg:max:1.15:0.05 \
    --show command.transient_z_pp_mm_max --show legs.meas_tau_hf \
    "$OUT/wbs_base" "$OUT/wbs_s1" "$OUT/wbs_s1_soft" \
    | tee "$OUT/wbc_sweep_report_B.txt" | tee -a "$REPORT"
fi
