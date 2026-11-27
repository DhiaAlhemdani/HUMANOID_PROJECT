#!/bin/bash
# Command-space benchmark: run every script in tests/tuning/suite for one config, score by segment,
# and (optionally) compare against a baseline tag.
#
# usage: run_suite.sh <tag> <controllers.yaml | default> [baseline_tag]
#   env: OUT, WS, SKIP_BUILD=1, TERRAIN (none | suite_mild | suite_rough, the uneven fields of config/terrains/uneven_suite_*_params.yaml),
#        HEIGHT (commanded pelvis height, default 0.785 = the shipped defaultBaseHeight), SCRIPTS="speeds turns lateral stops omni backward" (default all), VIDEO=1 (render
#        <case>/replay.mp4 with render_replay.py; VIDEO_PY=<python with mujoco imageio imageio-ffmpeg pillow>,
#        e.g. a venv: python3 -m venv ~/.venvs/replay && ~/.venvs/replay/bin/pip install mujoco imageio imageio-ffmpeg pillow numpy),
#        RESCORE=1 (no simulation: re-run score_suite.py / videos / comparison on existing case dirs)
# "default" as the config writes $OUT/<tag>.yaml from the shipped config with diagnostics.trackingRate 500
# (the shipped file has tracking off, and tau_rate / gait modes need it).
# writes: $OUT/<tag>__<script>/{data.npz, suite.json, suite.png, replay.mp4}, $OUT/<tag>_suite.txt,
#         and with a baseline $OUT/<tag>_vs_<baseline>.txt (compare_suite.py)
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WS=${WS:-/root/ocs2_ros2_ws}
if [ ! -r "$WS/install/setup.bash" ]; then
  echo "$(basename "$0"): no readable colcon workspace at WS=$WS (install/setup.bash missing or not readable); export WS=<your colcon ws, e.g. ~/Documents/GitHub/humanoid_ws>" >&2; exit 1
fi
OUT=${OUT:-$REPO/tuning_results}
TAG=$1; CFG=$2; BASE=${3:-}
SCRIPTS=${SCRIPTS:-"speeds turns lateral stops omni backward"}
mkdir -p "$OUT"
if [ "$CFG" = default ]; then
  python3 "$HERE/make_config.py" "$OUT/$TAG.yaml" --in "$REPO/config/g1/ros2_controllers.yaml" || exit 1
  CFG=$OUT/$TAG.yaml
fi
VIDEO_PY=${VIDEO_PY:-python3}
if [ "${RESCORE:-0}" != 1 ] && [ ! -f "$CFG" ]; then
  echo "RUN_SUITE: controllers config '$CFG' does not exist (usage: run_suite.sh <tag> <controllers.yaml | default> [baseline_tag]; SCRIPTS=\"speeds omni\" picks scripts)"; exit 1
fi

if [ "${SKIP_BUILD:-0}" != 1 ]; then
  echo "== colcon build (legged_robot_mpc_controller)"
  bash -c "source /opt/ros/jazzy/setup.bash && cd '$WS' && colcon build --packages-select legged_robot_mpc_controller" \
    || { echo "RUN_SUITE: colcon build failed"; exit 1; }
fi

REPORT="$OUT/${TAG}_suite.txt"; : > "$REPORT"
for s in $SCRIPTS; do
  case=${TAG}__$s
  for attempt in 1 2; do
    if [ "${RESCORE:-0}" = 1 ]; then
      python3 "$HERE/score_suite.py" "$OUT/$case" "$case" 2>&1 | grep -E "SUITE|^SEG|Traceback|Error" > "$OUT/$case/suite.txt"; cat "$OUT/$case/suite.txt"
      grep -E "^SUITE" "$OUT/$case/suite.txt" > "$OUT/$case/summary.txt.suite" ; break
    fi
    echo "== run $case attempt $attempt"
    OUT=$OUT PROFILE=script SCRIPT=$HERE/suite/$s.json HEIGHT=${HEIGHT:-0.785} bash "$HERE/run_case.sh" "$case" "$CFG" 0.5 || exit 1
    [ -f "$OUT/$case/suite.json" ] && break
    echo "WARN: $case produced no suite.json:"; [ -f "$OUT/$case/summary.txt" ] && cat "$OUT/$case/summary.txt"
    if [ $attempt = 1 ]; then
      pkill -9 -f "ros2 launch legged_robot_mpc_controller" 2>/dev/null; pkill -9 -x mujoco_ros2_con 2>/dev/null
      pkill -9 -f controller_manager 2>/dev/null; pkill -9 -f robot_state_publisher 2>/dev/null; sleep 5
    fi
  done
  { grep -hE "^SUITE" "$OUT/$case/summary.txt.suite" 2>/dev/null || grep -hE "^SUITE" "$OUT/$case/summary.txt"; } >> "$REPORT"
  if [ "${VIDEO:-0}" = 1 ]; then
    echo "== video $case"
    "$VIDEO_PY" "$HERE/render_replay.py" "$OUT/$case" "$OUT/$case/replay.mp4" 2>&1 | tail -1
  fi
done
echo; echo "== suite $TAG"; cat "$REPORT"
if [ -n "$BASE" ]; then
  echo; echo "== $TAG vs $BASE"
  python3 "$HERE/compare_suite.py" "$OUT" "$BASE" "$TAG" --scripts $SCRIPTS | tee "$OUT/${TAG}_vs_${BASE}.txt"
fi
