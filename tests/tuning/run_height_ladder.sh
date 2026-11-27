#!/bin/bash
# Height ladder: the same suite script(s) for one config at several commanded pelvis heights, compared
# across heights (the top entry is the baseline). Answers "does the gait hold up at this height?"
#
# usage: run_height_ladder.sh <tag> <controllers.yaml | default> [height ...]
#   default heights: 0.785 0.75 0.72 0.68 (the shipped default first; it is the baseline of the comparison)
#   env: as run_suite.sh (OUT, WS, SKIP_BUILD=1, SCRIPTS - default "speeds" here, VIDEO=1)
# writes: $OUT/<tag>_h<height>__<script>/... per height (run_suite.sh), $OUT/<tag>_heights.txt (compare_suite.py
#         across heights; read z_m in the limbs table: the walking height is a cost compromise, not a setpoint)
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
OUT=${OUT:-$REPO/tuning_results}
TAG=$1; CFG=$2; shift 2
HEIGHTS=${*:-"0.785 0.75 0.72 0.68"}
SCRIPTS=${SCRIPTS:-speeds}
if [ "$CFG" = default ]; then
  python3 "$HERE/make_config.py" "$OUT/$TAG.yaml" --in "$REPO/config/g1/ros2_controllers.yaml" || exit 1
  CFG=$OUT/$TAG.yaml
fi
tags=()
first=1
for h in $HEIGHTS; do
  t=${TAG}_h$h
  tags+=("$t")
  echo "== height $h -> $t"
  # one build at most (the first height); the rest reuse it
  SKIP_BUILD=$([ $first = 1 ] && echo "${SKIP_BUILD:-0}" || echo 1) HEIGHT=$h SCRIPTS="$SCRIPTS" OUT=$OUT \
    bash "$HERE/run_suite.sh" "$t" "$CFG" || { echo "HEIGHT_LADDER: $t failed"; exit 1; }
  first=0
done
# shellcheck disable=SC2086
python3 "$HERE/compare_suite.py" "$OUT" "${tags[@]}" --scripts $SCRIPTS | tee "$OUT/${TAG}_heights.txt"
