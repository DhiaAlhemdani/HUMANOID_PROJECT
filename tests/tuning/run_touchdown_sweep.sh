#!/bin/bash
# Phase 2.C sweep 1 (docs/design/touchdown_impact.md §4, docs/results/touchdown_timing_sweep.md).
# The shipped swing planner aims the touchdown 1 mm below the ground (swingTrajectory.
# touchDownHeightOffset -0.001, inherited from upstream, never swept): the foot reaches the real
# ground 14-20 ms before the plan's contact event and stands there at planned zero wrench; measured
# as touchdown.early_ms 20 (trot) / 14 (walk) and a base-vz mismatch of 42-57 % of the planned vz
# rms over the first 60 ms. Config-only candidates, shipped controller otherwise (WBC on, fade 0.03):
#   td_base    shipped (-0.001)
#   td_off0    touchDownHeightOffset 0.0     (aim at the ground)
#   td_offp1   touchDownHeightOffset +0.001  (aim 1 mm above: a late landing, the ladder's other side)
#   td_offm05  touchDownHeightOffset -0.0005 (half the shipped margin: monotonicity check)
# Set A (default): PROFILE=vx @ 0.5 (trot) -- primaries touchdown.abs_early_ms <= 0.4x |
#   touchdown.base_vz_mismatch <= 0.80x | walk.pelvis_acc_hf <= 0.92x | legs.meas_tau_hf <= 0.90x.
#   Guards: the sweep-3 set + foot z tracking (foot_*.z_track_err_mm <= 1.2x+0.3) + solve_hz.
# Set B (SETS=B): PROFILE=stops with td_off0 (and td_offp1) -- transient pitch/roll not-worse 1.1x.
# Config only: SKIP_BUILD=1 is fine when the install is current.
#
# usage: run_touchdown_sweep.sh [vx_mps]   env: OUT, WS, SKIP_BUILD=1, SETS="A B" (default A)
# writes: $OUT/<tag>/..., $OUT/<tag>.yaml, $OUT/touchdown_sweep_report.txt (+ per-set _A/_B copies)
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
    || { echo "RUN_TOUCHDOWN_SWEEP: colcon build failed"; exit 1; }
fi

ST=ocs2.gait.swingTrajectory
OFF0=("$ST.touchDownHeightOffset=0.0")
OFFP1=("$ST.touchDownHeightOffset=0.001")
OFFM05=("$ST.touchDownHeightOffset=-0.0005")

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
  [ -f "$OUT/$tag/sim.log" ] && grep -aE "whole-body feedback (layer on|solve invalid)|touchDownHeightOffset" "$OUT/$tag/sim.log" | tail -2
}

GUARDS=(--no-default-guards
  --guard walk.roll_rms_deg:max:1.15:0.05 --guard walk.pitch_rms_deg:max:1.15:0.05
  --guard legs.cmd_tau_jump_at_switch:max:1.15:0.02 --guard legs.track_err_q_deg:max:1.2:0.05
  --guard base.plan_z_sd_mm:max:1.10:0.5 --guard base.z_track_err_mm:max:1.10:0.5
  --guard base.pitch_track_err_deg:max:1.2:0.05 --guard walk.z_sd_mm:max:1.15:0.3
  --guard walk.arm_swing_pp_deg:max:1.20:1.0 --guard walk.vx_mean:min:0.90:0 --guard solver.solve_hz:min:0.90:0)

REPORT="$OUT/touchdown_sweep_report.txt"
if [[ " $SETS " == *" A "* ]]; then
  run td_base vx
  run td_off0 vx "${OFF0[@]}"
  run td_offp1 vx "${OFFP1[@]}"
  run td_offm05 vx "${OFFM05[@]}"
  echo; echo "== score set A (steady walk)"
  python3 "$HERE/compare_ab.py" "${GUARDS[@]}" \
    --guard foot_foot_l_contact.z_track_err_mm:max:1.2:0.3 --guard foot_foot_r_contact.z_track_err_mm:max:1.2:0.3 \
    --primary touchdown.abs_early_ms:0.4 --primary touchdown.base_vz_mismatch:0.80 \
    --primary walk.pelvis_acc_hf:0.92 --primary legs.meas_tau_hf:0.90 \
    --show touchdown.early_ms --show touchdown.leg_cmd_minus_plan_peak --show legs.cmd_tau_jump_at_switch \
    --show base.pitch_track_err_deg --show legs.tau_sat_pct \
    "$OUT/td_base" "$OUT/td_off0" "$OUT/td_offp1" "$OUT/td_offm05" \
    | tee "$OUT/touchdown_sweep_report_A.txt" | tee -a "$REPORT"
  echo; echo "== per-joint diagnostics"
  python3 "$HERE/wbc_diag.py" "$OUT/td_base" "$OUT/td_off0" "$OUT/td_offp1" "$OUT/td_offm05" \
    > "$OUT/touchdown_diag_A.txt" && echo "wrote $OUT/touchdown_diag_A.txt"
fi
if [[ " $SETS " == *" B "* ]]; then
  run tds_base stops
  run tds_off0 stops "${OFF0[@]}"
  run tds_offp1 stops "${OFFP1[@]}"
  echo; echo "== score set B (start/stop balance)"
  python3 "$HERE/compare_ab.py" --no-default-guards \
    --primary touchdown.abs_early_ms:0.4 --primary touchdown.base_vz_mismatch:0.80 \
    --primary command.transient_pitch_max_deg:0.85 --primary command.transient_roll_max_deg:0.85 \
    --primary walk.pelvis_acc_hf:0.92 \
    --guard walk.vx_mean:min:0.90:0 --guard solver.solve_hz:min:0.90:0 --guard walk.roll_rms_deg:max:1.15:0.05 \
    --guard command.transient_z_pp_mm_max:max:1.15:2.0 --guard legs.tau_sat_pct:max:1.0:0.02 \
    --show touchdown.early_ms --show legs.meas_tau_hf --show command.transient_z_pp_mm_max \
    "$OUT/tds_base" "$OUT/tds_off0" "$OUT/tds_offp1" \
    | tee "$OUT/touchdown_sweep_report_B.txt" | tee -a "$REPORT"
  python3 "$HERE/wbc_diag.py" "$OUT/tds_base" "$OUT/tds_off0" "$OUT/tds_offp1" > "$OUT/touchdown_diag_B.txt" && echo "wrote $OUT/touchdown_diag_B.txt"
fi
