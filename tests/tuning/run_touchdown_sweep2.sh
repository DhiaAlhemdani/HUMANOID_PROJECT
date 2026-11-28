#!/bin/bash
# Phase 2.C sweep 2 (docs/results/touchdown_timing_sweep.md "Sweep 2"): the swing-foot position gain.
# touchdown_trace.py on the sweep-1 recordings showed the robot tracks the MPC's foot plan to within
# 0.2 mm all the way down, and the *plan* reaches the ground ~40 ms before the planned contact
# event at 0.24-0.34 m/s (5-7x swingTrajectory.touchDownVelocity); the ground stops the foot early.
# The swing foot follows its reference spline through the hard acceleration equality
#   a_z = a_ref - kd (v - v_ref) - kp (z - z_ref)   (constraints.footMotionEquality, kp 50, kd 50)
# whose error poles are -1 and -49 rad/s: a height error, once made (20 ms zero-order-hold
# accelerations integrate a decelerating spline low), decays with a 1 s time constant -- never
# within a swing. Candidates raise kp so the slow pole moves inside the descent:
#   fk_base   shipped (kp 50, kd 50: poles -1 / -49)
#   fk200     kp 200 (poles -4.4 / -45.6)
#   fk400     kp 400 (poles -10 / -40)
#   fk625     kp 625 (critical, double pole -25)
# The same gain also scales the stance height correction (shared parameter); the stance foot z
# tracking guard covers that. Config only: SKIP_BUILD=1 is fine when the install is current.
# Set C (default): PROFILE=vx @ 0.5 (trot) -- primaries touchdown.abs_early_ms <= 0.5x |
#   touchdown.landing_speed <= 0.7x | walk.pelvis_acc_hf <= 0.92x; guards: sweep-1 set A +
#   legs.meas_tau_hf not-worse 1.1x + legs.tau_sat_pct not-worse.
# Set D (SETS=D): PROFILE=stops with fk200 and fk400 -- + transient pitch/roll 0.85x.
# If kp moves early_ms/landing_speed little, the sag is not position-feedback-limited and the next
# step is the C++ reference diagnostic (log z_ref/vz_ref next to plan_foot) before any further tuning.
#
# usage: run_touchdown_sweep2.sh [vx_mps]   env: OUT, WS, SKIP_BUILD=1, SETS="C D" (default C)
# writes: $OUT/<tag>/..., $OUT/<tag>.yaml, $OUT/touchdown_sweep2_report.txt (+ per-set _C/_D copies)
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WS=${WS:-/root/ocs2_ros2_ws}
OUT=${OUT:-$REPO/tuning_results}
VX=${1:-0.5}
SETS=${SETS:-"C"}
mkdir -p "$OUT"

if [ "${SKIP_BUILD:-0}" != 1 ]; then
  echo "== colcon build (legged_robot_mpc_controller)"
  bash -c "source /opt/ros/jazzy/setup.bash && cd '$WS' && colcon build --packages-select legged_robot_mpc_controller" \
    || { echo "RUN_TOUCHDOWN_SWEEP: colcon build failed"; exit 1; }
fi

FM=ocs2.constraints.footMotionEquality
KP200=("$FM.positionErrorGain_z=200.0")
KP400=("$FM.positionErrorGain_z=400.0")
KP625=("$FM.positionErrorGain_z=625.0")

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
  [ -f "$OUT/$tag/sim.log" ] && grep -aE "whole-body feedback (layer on|solve invalid)|positionErrorGain_z" "$OUT/$tag/sim.log" | tail -2
}

GUARDS=(--no-default-guards
  --guard walk.roll_rms_deg:max:1.15:0.05 --guard walk.pitch_rms_deg:max:1.15:0.05
  --guard legs.cmd_tau_jump_at_switch:max:1.15:0.02 --guard legs.track_err_q_deg:max:1.2:0.05
  --guard base.plan_z_sd_mm:max:1.10:0.5 --guard base.z_track_err_mm:max:1.10:0.5
  --guard base.pitch_track_err_deg:max:1.2:0.05 --guard walk.z_sd_mm:max:1.15:0.3
  --guard walk.arm_swing_pp_deg:max:1.20:1.0 --guard walk.vx_mean:min:0.90:0 --guard solver.solve_hz:min:0.90:0)

REPORT="$OUT/touchdown_sweep2_report.txt"
if [[ " $SETS " == *" C "* ]]; then
  run fk_base vx
  run fk200 vx "${KP200[@]}"
  run fk400 vx "${KP400[@]}"
  run fk625 vx "${KP625[@]}"
  echo; echo "== score set C (steady walk)"
  python3 "$HERE/compare_ab.py" "${GUARDS[@]}" \
    --guard foot_foot_l_contact.z_track_err_mm:max:1.2:0.3 --guard foot_foot_r_contact.z_track_err_mm:max:1.2:0.3 \
    --guard legs.meas_tau_hf:max:1.10:0 --guard legs.tau_sat_pct:max:1.0:0.02 \
    --primary touchdown.abs_early_ms:0.5 --primary touchdown.landing_speed:0.7 --primary walk.pelvis_acc_hf:0.92 \
    --show touchdown.early_ms --show touchdown.impact_early_ms --show touchdown.base_vz_mismatch \
    --show legs.meas_tau_hf --show legs.cmd_tau_jump_at_switch --show base.pitch_track_err_deg --show solver.solve_hz \
    "$OUT/fk_base" "$OUT/fk200" "$OUT/fk400" "$OUT/fk625" \
    | tee "$OUT/touchdown_sweep2_report_C.txt" | tee -a "$REPORT"
  echo; echo "== per-joint diagnostics"
  python3 "$HERE/wbc_diag.py" "$OUT/fk_base" "$OUT/fk200" "$OUT/fk400" "$OUT/fk625" \
    > "$OUT/touchdown_diag_C.txt" && echo "wrote $OUT/touchdown_diag_C.txt"
  echo; echo "== touchdown traces"
  python3 "$HERE/touchdown_trace.py" "$OUT/fk_base" "$OUT/fk200" "$OUT/fk400" "$OUT/fk625" > "$OUT/touchdown_trace_C.txt" \
    && grep -E "^===|decel_onset" "$OUT/touchdown_trace_C.txt"
fi
if [[ " $SETS " == *" D "* ]]; then
  run fks_base stops
  run fks200 stops "${KP200[@]}"
  run fks400 stops "${KP400[@]}"
  echo; echo "== score set D (start/stop balance)"
  python3 "$HERE/compare_ab.py" --no-default-guards \
    --primary touchdown.abs_early_ms:0.5 --primary touchdown.landing_speed:0.7 \
    --primary command.transient_pitch_max_deg:0.85 --primary command.transient_roll_max_deg:0.85 \
    --primary walk.pelvis_acc_hf:0.92 \
    --guard walk.vx_mean:min:0.90:0 --guard solver.solve_hz:min:0.90:0 --guard walk.roll_rms_deg:max:1.15:0.05 \
    --guard command.transient_z_pp_mm_max:max:1.15:2.0 --guard legs.tau_sat_pct:max:1.0:0.02 --guard legs.meas_tau_hf:max:1.10:0 \
    --show touchdown.early_ms --show touchdown.impact_early_ms --show touchdown.base_vz_mismatch --show command.transient_z_pp_mm_max \
    "$OUT/fks_base" "$OUT/fks200" "$OUT/fks400" \
    | tee "$OUT/touchdown_sweep2_report_D.txt" | tee -a "$REPORT"
  python3 "$HERE/wbc_diag.py" "$OUT/fks_base" "$OUT/fks200" "$OUT/fks400" > "$OUT/touchdown_diag_D.txt" && echo "wrote $OUT/touchdown_diag_D.txt"
  python3 "$HERE/touchdown_trace.py" "$OUT/fks_base" "$OUT/fks200" "$OUT/fks400" > "$OUT/touchdown_trace_D.txt" \
    && grep -E "^===|decel_onset" "$OUT/touchdown_trace_D.txt"
fi
