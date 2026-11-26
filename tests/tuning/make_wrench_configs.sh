#!/bin/bash
# Step A configs: human gait law + wrench-as-state layout with a uniform wrench-rate weight.
#
# usage: make_wrench_configs.sh <out.yaml> <rate weight> [first-node scale (default 0.05)] [arms]
#   arms: "arms" stiffens the arm posture x10 (q/qFinal jointPositions, q/qFinal jointVelocities and
#         r.jointAccelerations of the 8 arm joints), to keep the optimizer from using the arms as a
#         flywheel once the ground-reaction rate is priced (2026-11-25: arm sway at 1e-3).
# Joint order (robot.jointNames): 6 left leg, 6 right leg, 3 waist, 4 left arm, 4 right arm.
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$1; W=$2; SCALE=${3:-0.05}; ARMS=${4:-}
R12="[$W, $W, $W, $W, $W, $W, $W, $W, $W, $W, $W, $W]"
EDITS=("ocs2.gait.continuous.enabled=true" "ocs2.model.wrenchState=true"
       "ocs2.costs.wrenchRateFirstNodeScale=$SCALE" "ocs2.costs.r.contactWrenchRates=$R12")
if [ "$ARMS" = arms ]; then
  POS="[0.05, 0.2, 15.0, 0.1, 0.1, 0.1, 0.05, 0.2, 15.0, 0.1, 0.1, 0.1, 2.0, 2.0, 2.0, 50.0, 50.0, 20.0, 40.0, 50.0, 50.0, 20.0, 40.0]"
  POSF="[0.05, 0.1, 30.0, 0.1, 0.1, 0.1, 0.05, 0.1, 30.0, 0.1, 0.1, 0.1, 2.0, 2.0, 2.0, 50.0, 50.0, 20.0, 40.0, 50.0, 50.0, 20.0, 40.0]"
  VEL="[0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.2, 0.2, 0.2, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6]"
  ACC="[0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05]"
  VELF="[0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]"
  EDITS+=("ocs2.costs.q.jointPositions=$POS" "ocs2.costs.qFinal.jointPositions=$POSF"
          "ocs2.costs.q.jointVelocities=$VEL" "ocs2.costs.qFinal.jointVelocities=$VELF"
          "ocs2.costs.r.jointAccelerations=$ACC")
fi
python3 "$HERE/make_config.py" "$OUT" "${EDITS[@]}"
