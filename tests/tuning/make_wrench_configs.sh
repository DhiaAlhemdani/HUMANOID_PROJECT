#!/bin/bash
# Step A configs: human gait law + wrench-as-state layout with a uniform wrench-rate weight.
#
# usage: make_wrench_configs.sh <out.yaml> <rate weight> [first-node scale (default 0.05)] [features]
#   features: comma-separated subset of
#     arms   stiffen the arm posture x10 (q/qFinal jointPositions, q/qFinal jointVelocities and
#            r.jointAccelerations of the 8 arm joints): with the ground-reaction rate priced the
#            optimizer otherwise uses the arms as a flywheel (2026-11-25: arm shake at 1e-3).
#     swing  reference.armSwing.enabled = true: the arms get a human, gait-locked swing reference
#            (left arm forward with the right leg) instead of a frozen nominal posture.
#     waist  waist posture x10 (2 -> 20, velocity 0.2 -> 1.0): the torso is the other cheap flywheel
#            (upper body leaning back).
#     vel    base velocity weight x3 (vx 30 -> 100, vy 15 -> 50): with the rate priced the optimizer
#            lags the command (step_l -10 %, hold v_err 0.12 -> 0.16 at 1e-3) because the velocity
#            error is cheap next to the force rate.
#     human  reference.humanGait.enabled = true (roadmap step H): human walking kinematics as the
#            reference for legs, pelvis, trunk and arms. Sets the leg joint weights that make the leg
#            curves count (hip pitch 10, knee 10, ankle pitch 3; shipped: 0.05 / 0.1 / 0.1), the
#            standing pelvis height for kneeOffset 0.3 (0.785 m) and a straighter elbow (elbowMean
#            -0.4 rad). Combine with `arms,waist` for the arm / trunk weights. Not with `swing`.
#            env HUMAN_LEGW="hp kn ap" overrides the three leg weights (default "10 10 3");
#            HUMAN_BASE=false turns the pelvis height / roll template off (default on: the height
#            reference is the stance legs' FK, so legs and base ask for the same thing);
#            HUMAN_LEGS / HUMAN_ARMS / HUMAN_WAIST=false switch template parts off, HUMAN_MAXSCALE caps
#            the amplitude scale (1.2; 0.6 freezes the template at its 0.45 m/s amplitude);
#            HUMAN_SWING="z1 z2 z3 z4" sets the gait law's swing-height knots at 0 / 0.45 / 0.8 /
#            1.15 m/s (default "0.06 0.065 0.09 0.11": the ankle height the swing-leg curves imply
#            at those speeds, so the swing-foot task and the knee curve ask for the same leg).
#            NOTE the suite COMMANDS the pelvis height (tune_probe.py, default 0.795 = a straight
#            leg): run the human configs with HEIGHT=0.785 so the height and the 0.3 rad knee agree,
#            otherwise the height weight (50) and the knee weight pull against each other.
# Joint order (robot.jointNames): 6 left leg, 6 right leg, 3 waist, 4 left arm, 4 right arm.
set -eu
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$1; W=$2; SCALE=${3:-0.05}; FEAT=",${4:-},"
# rcl YAML: an integer in a double list is an error and 1e-4 is a *string*; write plain decimals.
W=$(python3 -c "import sys; s=format(float(sys.argv[1]), 'f').rstrip('0'); print(s + '0' if s.endswith('.') else s)" "$W")
R12="[$W, $W, $W, $W, $W, $W, $W, $W, $W, $W, $W, $W]"
EDITS=("ocs2.gait.continuous.enabled=true" "ocs2.model.wrenchState=true"
       "ocs2.costs.wrenchRateFirstNodeScale=$SCALE" "ocs2.costs.r.contactWrenchRates=$R12")
has() { [[ "$FEAT" == *",$1,"* ]]; }
LEGP="0.05, 0.2, 15.0, 0.1, 0.1, 0.1, 0.05, 0.2, 15.0, 0.1, 0.1, 0.1"
LEGPF="0.05, 0.1, 30.0, 0.1, 0.1, 0.1, 0.05, 0.1, 30.0, 0.1, 0.1, 0.1"
# The terminal costs track the raw posture (defaultJointState), not the template: with `human` the
# terminal leg / arm position weights stay at the shipped values so the last node does not pull back
# towards the crouch and the unswung arms.
if has human; then
  # ROS 2 YAML lists must be one type: every weight is written as a double (10 -> 10.0).
  # With the leg curves off the leg weights stay at the shipped (weak) values: the posture is then the crouch.
  if [ "${HUMAN_LEGS:-true}" = true ]; then LEGW_DEFAULT="10 10 3"; else LEGW_DEFAULT="0.05 0.1 0.1"; fi
  read -r HP KN AP <<< "$(printf '%.3f %.3f %.3f' ${HUMAN_LEGW:-$LEGW_DEFAULT})"
  read -r Z1 Z2 Z3 Z4 <<< "$(printf '%.3f %.3f %.3f %.3f' ${HUMAN_SWING:-0.06 0.065 0.09 0.11})"
  LEGP="$HP, 0.2, 15.0, $KN, $AP, 0.1, $HP, 0.2, 15.0, $KN, $AP, 0.1"
  MAXS=$(printf '%.3f' ${HUMAN_MAXSCALE:-1.2})
  EDITS+=("ocs2.reference.humanGait.enabled=true" "ocs2.reference.humanGait.elbowMean=-0.4"
          "ocs2.reference.humanGait.legs=${HUMAN_LEGS:-true}" "ocs2.reference.humanGait.arms=${HUMAN_ARMS:-true}"
          "ocs2.reference.humanGait.waist=${HUMAN_WAIST:-true}" "ocs2.reference.humanGait.base=${HUMAN_BASE:-true}"
          "ocs2.reference.humanGait.maxScale=$MAXS" "ocs2.reference.defaultBaseHeight=0.785"
          "ocs2.gait.continuous.swingHeight=[$Z1, $Z2, $Z3, $Z4]")
fi
LEGV="0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001, 0.001"
LEGA="0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005"
WP="2.0, 2.0, 2.0"; WV="0.2, 0.2, 0.2"
ARMP="5.0, 5.0, 2.0, 4.0, 5.0, 5.0, 2.0, 4.0"; ARMPF="$ARMP"; ARMV="0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06, 0.06"
ARMVF="0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02"; ARMA="0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005, 0.005"
if has arms; then
  ARMP="50.0, 50.0, 20.0, 40.0, 50.0, 50.0, 20.0, 40.0"; ARMV="0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6"
  ARMVF="0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2"; ARMA="0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05"
  if ! has human; then ARMPF="$ARMP"; fi
fi
if has waist; then WP="20.0, 20.0, 20.0"; WV="1.0, 1.0, 1.0"; fi
if has arms || has waist || has human; then
  EDITS+=("ocs2.costs.q.jointPositions=[$LEGP, $WP, $ARMP]" "ocs2.costs.qFinal.jointPositions=[$LEGPF, $WP, $ARMPF]"
          "ocs2.costs.q.jointVelocities=[$LEGV, $WV, $ARMV]" "ocs2.costs.qFinal.jointVelocities=[$LEGV, $WV, $ARMVF]"
          "ocs2.costs.r.jointAccelerations=[$LEGA, 0.005, 0.005, 0.005, $ARMA]")
fi
if has swing; then
  if has human; then echo "make_wrench_configs: 'swing' and 'human' both add an arm swing; drop 'swing'" >&2; exit 1; fi
  EDITS+=("ocs2.reference.armSwing.enabled=true")
fi
if has vel; then
  EDITS+=("ocs2.costs.q.baseVelocity=[100.0, 50.0, 5.0, 30.0, 30.0, 30.0]"
          "ocs2.costs.qFinal.baseVelocity=[100.0, 50.0, 5.0, 30.0, 30.0, 30.0]")
fi
python3 "$HERE/make_config.py" "$OUT" "${EDITS[@]}"
