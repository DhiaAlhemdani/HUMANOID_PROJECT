#!/bin/bash
# Phase 2.C sweep 4 (docs/results/touchdown_timing_sweep.md "Sweep 4"): the seam at the contact event.
# Sweep 3 shipped the swing-only height gain (swingPositionErrorGain_z 200, stance 50): landing
# 20 -> 6 ms early, 0.26 -> 0.19 m/s, acc_hf -27 %. Its one guard failure, legs.cmd_tau_jump_at_switch
# 0.121 -> 0.190, is entirely located (jump_anatomy.py) at policy switches within 40 ms of a planned
# contact change: 0.78 Nm there vs 0.27 with the shared gain and 0.36 shipped-before; elsewhere all
# three are 0.07. Mechanism: the foot still lands 6-8 ms early (the -1 mm touchDownHeightOffset is now
# the whole residual: 1 mm at 0.15 m/s), so re-solves between the real contact and t_e start from a
# stopped foot with the swing equality still active -- kp 200 on that error vs kp 50 in the stance
# form one node later is the seam. Candidates remove the residual earliness instead of softening kp:
#   so_base     shipped (swing 200 / stance 50, offset -0.001)  -- also the sk200 repeat
#   so_off0     + swingTrajectory.touchDownHeightOffset 0.0
#   so_offp05   + touchDownHeightOffset +0.0005
# (sweep 1 tried the offset alone; its stops verdict rested on base_vz_mismatch, since shown to be
# 23-46 % noisy, and the sag it was papering over is gone.) Config only: SKIP_BUILD=1 is fine.
# Set G (default): PROFILE=vx @ 0.5 -- primaries legs.cmd_tau_jump_at_switch <= 0.8x |
#   touchdown.abs_early_ms <= 0.5x | walk.pelvis_acc_hf <= 0.92x; guards sweep-1 set A minus the jump
#   guard (it is the primary) + landing_speed / meas_tau_hf / tau_sat not-worse.
# Set H (SETS=H): PROFILE=stops with so_off0 -- + transient pitch/roll 0.85x.
# Fallback if the offset does not close the seam: blend the swing gain to the stance value over the
# last ~40 ms of swing (planner side), not a return to the shared gain.
#
# usage: run_touchdown_sweep4.sh [vx_mps]   env: OUT, WS, SKIP_BUILD=1, SETS="G H" (default G)
# writes: $OUT/<tag>/..., $OUT/<tag>.yaml, $OUT/touchdown_sweep4_report.txt (+ per-set _G/_H copies)
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WS=${WS:-/root/ocs2_ros2_ws}
OUT=${OUT:-$REPO/tuning_results}
VX=${1:-0.5}
SETS=${SETS:-"G"}
mkdir -p "$OUT"

if [ "${SKIP_BUILD:-0}" != 1 ]; then
  echo "== colcon build (legged_robot_mpc_controller)"
  bash -c "source /opt/ros/jazzy/setup.bash && cd '$WS' && colcon build --packages-select legged_robot_mpc_controller" \
    || { echo "RUN_TOUCHDOWN_SWEEP: colcon build failed"; exit 1; }
fi

ST=ocs2.gait.swingTrajectory
OFF0=("$ST.touchDownHeightOffset=0.0")
OFFP05=("$ST.touchDownHeightOffset=0.0005")

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
  --guard legs.track_err_q_deg:max:1.2:0.05
  --guard base.plan_z_sd_mm:max:1.10:0.5 --guard base.z_track_err_mm:max:1.10:0.5
  --guard base.pitch_track_err_deg:max:1.2:0.05 --guard walk.z_sd_mm:max:1.15:0.3
  --guard walk.arm_swing_pp_deg:max:1.20:1.0 --guard walk.vx_mean:min:0.90:0 --guard solver.solve_hz:min:0.90:0)

REPORT="$OUT/touchdown_sweep4_report.txt"
if [[ " $SETS " == *" G "* ]]; then
  run so_base vx
  run so_off0 vx "${OFF0[@]}"
  run so_offp05 vx "${OFFP05[@]}"
  echo; echo "== score set G (steady walk)"
  python3 "$HERE/compare_ab.py" "${GUARDS[@]}" \
    --guard foot_foot_l_contact.z_track_err_mm:max:1.2:0.3 --guard foot_foot_r_contact.z_track_err_mm:max:1.2:0.3 \
    --guard legs.meas_tau_hf:max:1.10:0 --guard legs.tau_sat_pct:max:1.0:0.02 --guard touchdown.landing_speed:max:1.10:0 \
    --primary legs.cmd_tau_jump_at_switch:0.8 --primary touchdown.abs_early_ms:0.5 --primary walk.pelvis_acc_hf:0.92 \
    --show touchdown.early_ms --show touchdown.impact_early_ms --show touchdown.landing_speed --show touchdown.base_vz_mismatch \
    --show legs.meas_tau_hf --show walk.z_sd_mm --show solver.solve_hz \
    "$OUT/so_base" "$OUT/so_off0" "$OUT/so_offp05" \
    | tee "$OUT/touchdown_sweep4_report_G.txt" | tee -a "$REPORT"
  python3 "$HERE/wbc_diag.py" "$OUT/so_base" "$OUT/so_off0" "$OUT/so_offp05" > "$OUT/touchdown_diag_G.txt" && echo "wrote $OUT/touchdown_diag_G.txt"
  python3 "$HERE/touchdown_trace.py" "$OUT/so_base" "$OUT/so_off0" "$OUT/so_offp05" > "$OUT/touchdown_trace_G.txt" \
    && grep -E "^===|decel_onset" "$OUT/touchdown_trace_G.txt"
  echo; echo "== jump anatomy"
  python3 "$HERE/jump_anatomy.py" "$OUT/fk_base" "$OUT/so_base" "$OUT/so_off0" "$OUT/so_offp05" 2>/dev/null | tee -a "$REPORT"
fi
if [[ " $SETS " == *" H "* ]]; then
  run sos_base stops
  run sos_off0 stops "${OFF0[@]}"
  echo; echo "== score set H (start/stop balance)"
  python3 "$HERE/compare_ab.py" --no-default-guards \
    --primary legs.cmd_tau_jump_at_switch:0.8 --primary touchdown.abs_early_ms:0.5 \
    --primary command.transient_pitch_max_deg:0.85 --primary command.transient_roll_max_deg:0.85 \
    --primary walk.pelvis_acc_hf:0.92 \
    --guard walk.vx_mean:min:0.90:0 --guard solver.solve_hz:min:0.90:0 --guard walk.roll_rms_deg:max:1.15:0.05 \
    --guard command.transient_z_pp_mm_max:max:1.15:2.0 --guard legs.tau_sat_pct:max:1.0:0.02 --guard legs.meas_tau_hf:max:1.10:0 \
    --guard touchdown.landing_speed:max:1.10:0 \
    --show touchdown.early_ms --show touchdown.landing_speed --show touchdown.base_vz_mismatch --show command.transient_z_pp_mm_max --show walk.z_sd_mm \
    "$OUT/sos_base" "$OUT/sos_off0" \
    | tee "$OUT/touchdown_sweep4_report_H.txt" | tee -a "$REPORT"
  python3 "$HERE/wbc_diag.py" "$OUT/sos_base" "$OUT/sos_off0" > "$OUT/touchdown_diag_H.txt" && echo "wrote $OUT/touchdown_diag_H.txt"
  python3 "$HERE/touchdown_trace.py" "$OUT/sos_base" "$OUT/sos_off0" > "$OUT/touchdown_trace_H.txt" \
    && grep -E "^===|decel_onset" "$OUT/touchdown_trace_H.txt"
  echo; echo "== jump anatomy"
  python3 "$HERE/jump_anatomy.py" "$OUT/fks_base" "$OUT/sos_base" "$OUT/sos_off0" 2>/dev/null | tee -a "$REPORT"
fi
