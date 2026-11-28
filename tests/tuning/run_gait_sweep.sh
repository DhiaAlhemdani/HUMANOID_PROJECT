#!/bin/bash
# Gait sweep (docs/results/gait_sweep.md): double support against the contact-switch torque step.
# Every trace of the shipped controller's remaining walk transient ends at the contact event: the
# plan moves a leg from swing to stance in one node, so ~60 Nm of hip-pitch torque and the whole
# weight transfer happen in one tick (docs/results/touchdown_timing_sweep.md, "Resolved"). At 0.5 m/s
# the ladder selects `trot` ([LF, RF], 0.5 s supports, zero double support), which makes that step
# instantaneous by construction. The `walk` template has a STANCE phase between supports -- a window
# in which the plan can move the weight across. Candidates (all config only, SKIP_BUILD=1 is fine):
#   gw_base      shipped: trot at 0.5 m/s
#   gw_walk      walk/trot boundary 0.45 -> 0.55 m/s: the stock walk template at 0.5
#                (0.3 s swing + 0.1 s double support, 0.8 s cycle, swingHeight 0.08; note the 0.3 s
#                swing is scaled 0.75x by swingTimeScale 0.4)
#   gw_walk41    boundary + walk template at the trot's cadence: [LF, STANCE, RF, STANCE]
#                [0, 0.4, 0.5, 0.9, 1.0], swingHeight 0.09 -- the last 0.1 s of each trot support
#                becomes double support, step period and swing height unchanged (cleanest A/B)
#   gw_walk3515  as walk41 with 0.15 s double support: [0, 0.35, 0.5, 0.85, 1.0]
# The boundary move keeps the command at 0.5 below walk's top (escalation needs command >= top), so
# the trot never engages; trot's lower bound follows to 0.50 for a consistent ladder.
# Set I (default): PROFILE=vx @ 0.5 -- primaries walk.pelvis_acc_hf <= 0.92x |
#   legs.meas_tau_hf <= 0.90x | legs.plan_tau_step_at_contact_change <= 0.7x (new metric: the step
#   itself); guards the standard set + touchdown.landing_speed / tau_sat / cmd_tau_jump not-worse,
#   vx_mean >= 0.9x (a gait that cannot hold the speed loses).
# Set J (SETS=J): PROFILE=stops with gw_walk41 and gw_walk -- + transient pitch/roll 0.85x.
#
# usage: run_gait_sweep.sh [vx_mps]   env: OUT, WS, SKIP_BUILD=1, SETS="I J" (default I)
# writes: $OUT/<tag>/..., $OUT/<tag>.yaml, $OUT/gait_sweep_report.txt (+ per-set _I/_J copies)
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WS=${WS:-/root/ocs2_ros2_ws}
OUT=${OUT:-$REPO/tuning_results}
VX=${1:-0.5}
SETS=${SETS:-"I"}
mkdir -p "$OUT"

if [ "${SKIP_BUILD:-0}" != 1 ]; then
  echo "== colcon build (legged_robot_mpc_controller)"
  bash -c "source /opt/ros/jazzy/setup.bash && cd '$WS' && colcon build --packages-select legged_robot_mpc_controller" \
    || { echo "RUN_GAIT_SWEEP: colcon build failed"; exit 1; }
fi

TR=ocs2.gait.transitions
TP=ocs2.gait.templates
BAND=("$TR.walk={linearVelocity: [0.02, 0.55], yawRate: [0.05, 10.0], measuredMargin: [0.05, 0.05]}"
      "$TR.trot={linearVelocity: [0.50, 0.75], yawRate: [0.65, 10.0], measuredMargin: [0.2, 0.2]}")
WALK41=("${BAND[@]}" "$TP.walk={modeSequence: [LF, STANCE, RF, STANCE], switchingTimes: [0.0, 0.4, 0.5, 0.9, 1.0], swingHeight: 0.09}")
WALK3515=("${BAND[@]}" "$TP.walk={modeSequence: [LF, STANCE, RF, STANCE], switchingTimes: [0.0, 0.35, 0.5, 0.85, 1.0], swingHeight: 0.09}")

diag() {
  local d=$1
  [ -f "$d/summary.txt" ] && { echo "--- $d/summary.txt"; cat "$d/summary.txt"; }
  [ -f "$d/sim.log" ] && { echo "--- $d/sim.log (FAULT/Traceback/error/gait lines)";
    grep -aE "FAULT|Traceback|Error|error|failed|[Gg]ait" "$d/sim.log" | head -20; }
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
      pkill -9 -f "ros2 launch legged_robot_mpc_controller" 2>/dev/null; pkill -9 -x mujoco_ros2_con 2>/dev/null
      pkill -9 -f controller_manager 2>/dev/null; pkill -9 -f robot_state_publisher 2>/dev/null; sleep 5
      echo "== retrying $tag"
    fi
  done
  # Which gait the ladder actually ran (the point of the band move).
  [ -f "$OUT/$tag/sim.log" ] && grep -aiE "gait (change|transition|switch)|Linear limits" "$OUT/$tag/sim.log" | tail -4
  python3 - "$OUT/$tag/diag.json" <<'EOF'
import json, sys
d = json.load(open(sys.argv[1])); g = d.get("gait", {})
print(f"   gait: {g.get('sequence', '?')} {g.get('templates_s', '')} | contact changes/s {d.get('contact_changes_per_s', '?')}"
      f" | plan tau step at contact change {d.get('legs', {}).get('plan_tau_step_at_contact_change', '?')} Nm"
      f" (max {d.get('legs', {}).get('plan_tau_step_at_contact_change_max', '?')})")
EOF
}

GUARDS=(--no-default-guards
  --guard walk.roll_rms_deg:max:1.15:0.05 --guard walk.pitch_rms_deg:max:1.15:0.05
  --guard legs.track_err_q_deg:max:1.2:0.05
  --guard base.plan_z_sd_mm:max:1.10:0.5 --guard base.z_track_err_mm:max:1.10:0.5
  --guard base.pitch_track_err_deg:max:1.2:0.05 --guard walk.z_sd_mm:max:1.15:0.3
  --guard walk.arm_swing_pp_deg:max:1.20:1.0 --guard walk.vx_mean:min:0.90:0 --guard solver.solve_hz:min:0.90:0
  --guard legs.cmd_tau_jump_at_switch:max:1.15:0.02 --guard legs.tau_sat_pct:max:1.0:0.02
  --guard touchdown.landing_speed:max:1.10:0)

REPORT="$OUT/gait_sweep_report.txt"
if [[ " $SETS " == *" I "* ]]; then
  run gw_base vx
  run gw_walk vx "${BAND[@]}"
  run gw_walk41 vx "${WALK41[@]}"
  run gw_walk3515 vx "${WALK3515[@]}"
  echo; echo "== score set I (steady walk)"
  python3 "$HERE/compare_ab.py" "${GUARDS[@]}" \
    --guard foot_foot_l_contact.z_track_err_mm:max:1.2:0.3 --guard foot_foot_r_contact.z_track_err_mm:max:1.2:0.3 \
    --primary walk.pelvis_acc_hf:0.92 --primary legs.meas_tau_hf:0.90 --primary legs.plan_tau_step_at_contact_change:0.7 \
    --show legs.cmd_tau_step_at_contact_change --show legs.plan_tau_step_at_contact_change_max --show contact_changes_per_s \
    --show touchdown.early_ms --show touchdown.landing_speed --show walk.roll_rms_deg --show walk.pitch_rms_deg \
    --show walk.z_sd_mm --show walk.vx_mean --show legs.tau_sat_pct --show solver.solve_hz \
    "$OUT/gw_base" "$OUT/gw_walk" "$OUT/gw_walk41" "$OUT/gw_walk3515" \
    | tee "$OUT/gait_sweep_report_I.txt" | tee -a "$REPORT"
  python3 "$HERE/wbc_diag.py" "$OUT/gw_base" "$OUT/gw_walk" "$OUT/gw_walk41" "$OUT/gw_walk3515" > "$OUT/gait_diag_I.txt" && echo "wrote $OUT/gait_diag_I.txt"
  python3 "$HERE/touchdown_trace.py" "$OUT/gw_base" "$OUT/gw_walk" "$OUT/gw_walk41" "$OUT/gw_walk3515" > "$OUT/gait_trace_I.txt" \
    && grep -E "^===" "$OUT/gait_trace_I.txt"
  echo; echo "== jump anatomy (largest-jump trace shows the contact-change step)"
  python3 "$HERE/jump_anatomy.py" "$OUT/gw_base" "$OUT/gw_walk41" 2>/dev/null | tee -a "$REPORT"
fi
if [[ " $SETS " == *" J "* ]]; then
  run gws_base stops
  run gws_walk stops "${BAND[@]}"
  run gws_walk41 stops "${WALK41[@]}"
  echo; echo "== score set J (start/stop balance)"
  python3 "$HERE/compare_ab.py" --no-default-guards \
    --primary command.transient_pitch_max_deg:0.85 --primary command.transient_roll_max_deg:0.85 \
    --primary walk.pelvis_acc_hf:0.92 --primary legs.plan_tau_step_at_contact_change:0.7 \
    --guard walk.vx_mean:min:0.90:0 --guard solver.solve_hz:min:0.90:0 --guard walk.roll_rms_deg:max:1.15:0.05 \
    --guard command.transient_z_pp_mm_max:max:1.15:2.0 --guard legs.tau_sat_pct:max:1.0:0.02 --guard legs.meas_tau_hf:max:1.10:0 \
    --guard touchdown.landing_speed:max:1.10:0 --guard legs.cmd_tau_jump_at_switch:max:1.15:0.02 \
    --show command.transient_acc_hf_max --show command.transient_z_pp_mm_max --show touchdown.early_ms --show walk.z_sd_mm \
    --show legs.cmd_tau_step_at_contact_change \
    "$OUT/gws_base" "$OUT/gws_walk" "$OUT/gws_walk41" \
    | tee "$OUT/gait_sweep_report_J.txt" | tee -a "$REPORT"
  python3 "$HERE/wbc_diag.py" "$OUT/gws_base" "$OUT/gws_walk" "$OUT/gws_walk41" > "$OUT/gait_diag_J.txt" && echo "wrote $OUT/gait_diag_J.txt"
fi
