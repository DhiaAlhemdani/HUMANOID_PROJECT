#!/bin/bash
# Phase 1.4 foot / contact-model sweep (docs/LOCOMOTION_IMPROVEMENTS.md #1.4).
# G1 walk with sole-model variants (make_model.py, MODEL= for run_case.sh) and/or CoP-rectangle
# config variants (make_config.py), scored by compare_footcontact.py (any-of primaries:
# acc_hf <= 0.92x, roll <= 0.85x, PD peak <= 0.90x; all guards must pass).
#
# usage: run_footcontact_sweep.sh [vx_mps]        (default 0.5)
#   env: OUT (default <repo>/tuning_results), WS (colcon workspace; default
#        /root/ocs2_ros2_ws in the container), SKIP_BUILD=1, BASELINE=<run dir>
#   baseline: $BASELINE if set, else a FRESH fc_base run (shipped model, shipped config via
#             make_config -> diagnostics.trackingRate; needed for diag.json).
# writes: $OUT/<tag>/{diag.json,metrics.json,robot_model.xml?,...}, $OUT/<tag>.yaml,
#         $OUT/<tag>_model.xml, $OUT/footcontact_sweep_report.txt
#
# Shipped facts (recon 2026-11-24): sole = four r=5 mm spheres, hull x[-0.05,0.12] y[+-0.025..0.03];
# MPC rectangle = x[-0.055,0.125] y[+-0.03] (5 mm OUTSIDE the physical footprint per edge);
# the robot's solref 0.001 is inert (floor has priority=1 -> MuJoCo default 20 ms contact).
# Cases  (tag | make_model args or "-" | make_config edits or "-"):
#   fc_rectfit    config only: rectangle fitted inside the sphere hull (5 mm CoP margin)
#   fc_box        model only: one box sole per foot, sphere hull (shape change, same footprint)
#   fc_box_rectfit box sole + fitted rectangle (plant and plan agree exactly)
#   fc_boxrect    box sole grown to the shipped rectangle (plant made to match the plan)
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WS=${WS:-/root/ocs2_ros2_ws}
OUT=${OUT:-$REPO/tuning_results}
VX=${1:-0.5}
mkdir -p "$OUT"

if [ "${SKIP_BUILD:-0}" != 1 ]; then
  echo "== colcon build (legged_robot_mpc_controller)"
  bash -c "source /opt/ros/jazzy/setup.bash && cd '$WS' && colcon build --packages-select legged_robot_mpc_controller" \
    || { echo "RUN_FOOTCONTACT_SWEEP: colcon build failed"; exit 1; }
fi

RECTFIT="ocs2.model.contactGeometry.rectangle=[0.085, -0.085, 0.025, -0.025]"
CASES=(
  "fc_rectfit|-|$RECTFIT"
  "fc_box|--sole box --hull spheres|-"
  "fc_box_rectfit|--sole box --hull spheres|$RECTFIT"
  "fc_boxrect|--sole box --hull rectangle|-"
)

# diag <dir>: one-glance failure evidence for a run that never produced metrics/diag.
diag() {
  local d=$1
  [ -f "$d/summary.txt" ] && { echo "--- $d/summary.txt"; cat "$d/summary.txt"; }
  [ -f "$d/sim.log" ] && { echo "--- $d/sim.log (FAULT/Traceback/error lines)";
    grep -aE "FAULT|Traceback|Error|error|failed" "$d/sim.log" | head -20; }
}

if [ -n "${BASELINE:-}" ]; then
  BL=$BASELINE
else
  echo "== baseline config fc_base.yaml (shipped + diagnostics.trackingRate)"
  python3 "$HERE/make_config.py" "$OUT/fc_base.yaml" --in "$REPO/config/g1/ros2_controllers.yaml" || exit 1
  echo "== baseline: fresh shipped-model run (fc_base)"
  OUT=$OUT bash "$HERE/run_case.sh" fc_base "$OUT/fc_base.yaml" "$VX" || exit 1
  BL=$OUT/fc_base
fi
echo "== baseline: $BL"
if [ ! -f "$BL/metrics.json" ] || [ ! -f "$BL/diag.json" ]; then
  { echo "BASELINE FAILED: $BL needs both metrics.json and diag.json (got: $(ls "$BL" 2>/dev/null | tr '\n' ' '))"
    diag "$BL"; } | tee "$OUT/footcontact_sweep_report.txt"
  exit 1
fi

CANDS=()
for spec in "${CASES[@]}"; do
  IFS='|' read -r tag model edits <<< "$spec"
  echo "== config $tag"
  if [ "$edits" = "-" ]; then
    python3 "$HERE/make_config.py" "$OUT/$tag.yaml" --in "$REPO/config/g1/ros2_controllers.yaml" || exit 1
  else
    python3 "$HERE/make_config.py" "$OUT/$tag.yaml" --in "$REPO/config/g1/ros2_controllers.yaml" "$edits" || exit 1
  fi
  MODEL_ARG=
  if [ "$model" != "-" ]; then
    # shellcheck disable=SC2086
    python3 "$HERE/make_model.py" "$OUT/${tag}_model.xml" $model || exit 1
    MODEL_ARG=$OUT/${tag}_model.xml
  fi
  echo "== run $tag${MODEL_ARG:+ (model $MODEL_ARG)}"
  OUT=$OUT MODEL=$MODEL_ARG bash "$HERE/run_case.sh" "$tag" "$OUT/$tag.yaml" "$VX" || exit 1
  if [ ! -f "$OUT/$tag/metrics.json" ] || [ ! -f "$OUT/$tag/diag.json" ]; then
    echo "WARN: $tag missing metrics.json/diag.json (scored as ERROR):"
    diag "$OUT/$tag"
  fi
  CANDS+=("$OUT/$tag")
done

echo
echo "== score"
python3 "$HERE/compare_footcontact.py" "$BL" "${CANDS[@]}" | tee "$OUT/footcontact_sweep_report.txt"
