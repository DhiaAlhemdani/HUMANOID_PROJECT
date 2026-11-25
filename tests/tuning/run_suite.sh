#!/bin/bash
# Command-space benchmark: run every script in tests/tuning/suite for one config, score by segment,
# and (optionally) compare against a baseline tag.
#
# usage: run_suite.sh <tag> <controllers.yaml | default> [baseline_tag]
#   env: OUT, WS, SKIP_BUILD=1, SCRIPTS="speeds turns lateral stops omni" (default all), VIDEO=1 (render
#        <case>/replay.mp4 with render_replay.py, needs mujoco + imageio-ffmpeg in the python env)
# writes: $OUT/<tag>__<script>/{data.npz, suite.json, suite.png, replay.mp4}, $OUT/<tag>_suite.txt,
#         and with a baseline $OUT/<tag>_vs_<baseline>.txt (compare_suite.py)
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WS=${WS:-/root/ocs2_ros2_ws}
OUT=${OUT:-$REPO/tuning_results}
TAG=$1; CFG=$2; BASE=${3:-}
SCRIPTS=${SCRIPTS:-"speeds turns lateral stops omni"}
mkdir -p "$OUT"
[ "$CFG" = default ] && CFG=$REPO/config/g1/ros2_controllers.yaml

if [ "${SKIP_BUILD:-0}" != 1 ]; then
  echo "== colcon build (legged_robot_mpc_controller)"
  bash -c "source /opt/ros/jazzy/setup.bash && cd '$WS' && colcon build --packages-select legged_robot_mpc_controller" \
    || { echo "RUN_SUITE: colcon build failed"; exit 1; }
fi

REPORT="$OUT/${TAG}_suite.txt"; : > "$REPORT"
for s in $SCRIPTS; do
  case=${TAG}__$s
  for attempt in 1 2; do
    echo "== run $case attempt $attempt"
    OUT=$OUT PROFILE=script SCRIPT=$HERE/suite/$s.json bash "$HERE/run_case.sh" "$case" "$CFG" 0.5 || exit 1
    [ -f "$OUT/$case/suite.json" ] && break
    echo "WARN: $case produced no suite.json:"; [ -f "$OUT/$case/summary.txt" ] && cat "$OUT/$case/summary.txt"
    if [ $attempt = 1 ]; then
      pkill -9 -f "ros2 launch legged_robot_mpc_controller" 2>/dev/null; pkill -9 -x mujoco_ros2_con 2>/dev/null
      pkill -9 -f controller_manager 2>/dev/null; pkill -9 -f robot_state_publisher 2>/dev/null; sleep 5
    fi
  done
  grep -E "^SUITE" "$OUT/$case/summary.txt" >> "$REPORT"
  if [ "${VIDEO:-0}" = 1 ]; then
    python3 "$HERE/render_replay.py" "$OUT/$case" "$OUT/$case/replay.mp4" 2>&1 | tail -1
  fi
done
echo; echo "== suite $TAG"; cat "$REPORT"
if [ -n "$BASE" ]; then
  echo; echo "== $TAG vs $BASE"
  python3 "$HERE/compare_suite.py" "$OUT" "$BASE" "$TAG" --scripts $SCRIPTS | tee "$OUT/${TAG}_vs_${BASE}.txt"
fi
