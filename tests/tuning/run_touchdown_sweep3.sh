#!/bin/bash
# Phase 2.C sweep 3 (docs/results/touchdown_timing_sweep.md "Sweep 3"): the swing-only position gain.
# Sweep 2 raised constraints.footMotionEquality.positionErrorGain_z 50 -> 200/400/625 and moved the
# landing from 20 ms early at 0.26 m/s to 8/6/4 ms at 0.19 m/s (acc_hf -26/-31/-33 %); 200 and 400
# won the steady walk, 400 regressed the stops' pitch transient 1.33x, 625 failed walk z sd. That
# parameter is shared by the stance height correction (the reason it was lowered 250 -> 50 for ramps),
# so the gain is now split: swingPositionErrorGain_z (new, g1 ships 200) for the swing equality,
# positionErrorGain_z (stays 50) for stance. This sweep confirms the split reproduces sweep 2 with the
# stance gain untouched, and asks whether the stops regression at 400 came from the stance side:
#   sk200   swing 200 / stance 50  (= the shipped g1 config)
#   sk400   swing 400 / stance 50
# Baselines: sweep 2's fk_base / fks_base (same session, identical behaviour: swing gain < 0 falls
# back to the stance gain); they are run here only if missing. C++ changed: do NOT use SKIP_BUILD=1.
# Set E (default): PROFILE=vx @ 0.5 -- sweep-2 set C gates. Set F (SETS=F): PROFILE=stops -- set D gates.
#
# usage: run_touchdown_sweep3.sh [vx_mps]   env: OUT, WS, SETS="E F" (default E)
# writes: $OUT/<tag>/..., $OUT/<tag>.yaml, $OUT/touchdown_sweep3_report.txt (+ per-set _E/_F copies)
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WS=${WS:-/root/ocs2_ros2_ws}
OUT=${OUT:-$REPO/tuning_results}
VX=${1:-0.5}
SETS=${SETS:-"E"}
mkdir -p "$OUT"

if [ "${SKIP_BUILD:-0}" != 1 ]; then
  echo "== colcon build (legged_robot_mpc_controller)"
  bash -c "source /opt/ros/jazzy/setup.bash && cd '$WS' && colcon build --packages-select legged_robot_mpc_controller" \
    || { echo "RUN_TOUCHDOWN_SWEEP: colcon build failed"; exit 1; }
fi

FM=ocs2.constraints.footMotionEquality
SK200=("$FM.swingPositionErrorGain_z=200.0" "$FM.positionErrorGain_z=50.0")
SK400=("$FM.swingPositionErrorGain_z=400.0" "$FM.positionErrorGain_z=50.0")
SKBASE=("$FM.swingPositionErrorGain_z=50.0" "$FM.positionErrorGain_z=50.0")  # sweep-2 baseline behaviour

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
  [ -f "$OUT/$tag/sim.log" ] && grep -aE "whole-body feedback (layer on|solve invalid)|swingPositionErrorGain_z" "$OUT/$tag/sim.log" | tail -2
}

GUARDS=(--no-default-guards
  --guard walk.roll_rms_deg:max:1.15:0.05 --guard walk.pitch_rms_deg:max:1.15:0.05
  --guard legs.cmd_tau_jump_at_switch:max:1.15:0.02 --guard legs.track_err_q_deg:max:1.2:0.05
  --guard base.plan_z_sd_mm:max:1.10:0.5 --guard base.z_track_err_mm:max:1.10:0.5
  --guard base.pitch_track_err_deg:max:1.2:0.05 --guard walk.z_sd_mm:max:1.15:0.3
  --guard walk.arm_swing_pp_deg:max:1.20:1.0 --guard walk.vx_mean:min:0.90:0 --guard solver.solve_hz:min:0.90:0)

REPORT="$OUT/touchdown_sweep3_report.txt"
if [[ " $SETS " == *" E "* ]]; then
  [ -f "$OUT/fk_base/diag.json" ] || run fk_base vx "${SKBASE[@]}"
  run sk200 vx "${SK200[@]}"
  run sk400 vx "${SK400[@]}"
  echo; echo "== score set E (steady walk)"
  python3 "$HERE/compare_ab.py" "${GUARDS[@]}" \
    --guard foot_foot_l_contact.z_track_err_mm:max:1.2:0.3 --guard foot_foot_r_contact.z_track_err_mm:max:1.2:0.3 \
    --guard legs.meas_tau_hf:max:1.10:0 --guard legs.tau_sat_pct:max:1.0:0.02 \
    --primary touchdown.abs_early_ms:0.5 --primary touchdown.landing_speed:0.7 --primary walk.pelvis_acc_hf:0.92 \
    --show touchdown.early_ms --show touchdown.impact_early_ms --show touchdown.base_vz_mismatch \
    --show legs.meas_tau_hf --show legs.cmd_tau_jump_at_switch --show walk.z_sd_mm --show solver.solve_hz \
    "$OUT/fk_base" "$OUT/sk200" "$OUT/sk400" \
    | tee "$OUT/touchdown_sweep3_report_E.txt" | tee -a "$REPORT"
  python3 "$HERE/wbc_diag.py" "$OUT/fk_base" "$OUT/sk200" "$OUT/sk400" > "$OUT/touchdown_diag_E.txt" && echo "wrote $OUT/touchdown_diag_E.txt"
  python3 "$HERE/touchdown_trace.py" "$OUT/fk_base" "$OUT/sk200" "$OUT/sk400" > "$OUT/touchdown_trace_E.txt" \
    && grep -E "^===|decel_onset" "$OUT/touchdown_trace_E.txt"
fi
if [[ " $SETS " == *" F "* ]]; then
  [ -f "$OUT/fks_base/diag.json" ] || run fks_base stops "${SKBASE[@]}"
  run sks200 stops "${SK200[@]}"
  run sks400 stops "${SK400[@]}"
  echo; echo "== score set F (start/stop balance)"
  python3 "$HERE/compare_ab.py" --no-default-guards \
    --primary touchdown.abs_early_ms:0.5 --primary touchdown.landing_speed:0.7 \
    --primary command.transient_pitch_max_deg:0.85 --primary command.transient_roll_max_deg:0.85 \
    --primary walk.pelvis_acc_hf:0.92 \
    --guard walk.vx_mean:min:0.90:0 --guard solver.solve_hz:min:0.90:0 --guard walk.roll_rms_deg:max:1.15:0.05 \
    --guard command.transient_z_pp_mm_max:max:1.15:2.0 --guard legs.tau_sat_pct:max:1.0:0.02 --guard legs.meas_tau_hf:max:1.10:0 \
    --show touchdown.early_ms --show touchdown.impact_early_ms --show touchdown.base_vz_mismatch --show command.transient_z_pp_mm_max --show walk.z_sd_mm \
    "$OUT/fks_base" "$OUT/sks200" "$OUT/sks400" \
    | tee "$OUT/touchdown_sweep3_report_F.txt" | tee -a "$REPORT"
  python3 "$HERE/wbc_diag.py" "$OUT/fks_base" "$OUT/sks200" "$OUT/sks400" > "$OUT/touchdown_diag_F.txt" && echo "wrote $OUT/touchdown_diag_F.txt"
  python3 "$HERE/touchdown_trace.py" "$OUT/fks_base" "$OUT/sks200" "$OUT/sks400" > "$OUT/touchdown_trace_F.txt" \
    && grep -E "^===|decel_onset" "$OUT/touchdown_trace_F.txt"
fi
