#!/bin/bash
# One WB-ID tuning experiment in the ros_jazzy container: launch WB-ID on generated flat ground with a
# controllers yaml, run the stand -> walk -> stop probe, score it and plot it.
#
# usage: run_case.sh <tag> <controllers.yaml | default> [vx_mps]
#   env: HEIGHT (commanded pelvis height, default 0.785), LAUNCH_ARGS (extra robot.launch.py args), OUT (results root, default
#        <repo>/tuning_results), RVIZ=true SHOTS=1 (RViz screenshots during the walk),
#        HOLD_SECONDS (constant-command duration, default 20),
#        MODEL (variant robot MJCF from make_model.py; default = shipped g1_29dof.xml)
# writes: $OUT/<tag>/{sim.log, data.npz, metrics.json, diag.json, *.png, config.yaml, summary.txt}
# The streams are recorded by ros2 bag (a Python subscriber in this container drops most of the
# 1 kHz data), converted to data.npz, and the bag is deleted.
# See docs/wb_id_tuning_benchmark.md.
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
REPO=$(cd "$HERE/../.." && pwd)
WS=${WS:-/root/ocs2_ros2_ws}
SRC="source /opt/ros/jazzy/setup.bash && source $WS/install/setup.bash"
TAG=$1; CFG=$2; VX=${3:-0.5}
OUT=${OUT:-$REPO/tuning_results}; D=$OUT/$TAG; mkdir -p "$D"; rm -rf "${D:?}"/*
[ "$CFG" = default ] && CFG=$REPO/config/g1/ros2_controllers.yaml
cp "$CFG" "$D/config.yaml"
[ -n "${MODEL:-}" ] && cp "$MODEL" "$D/robot_model.xml"
python3 "$REPO/tests/mujoco_scene.py" --robot-name g1 --output "$D/scene.xml" ${MODEL:+--robot-model "$D/robot_model.xml"}
pkill -9 -x mujoco_ros2_con 2>/dev/null; pkill -9 -x rviz2 2>/dev/null; sleep 2
# LAUNCH_TIMEOUT / LOCO_WAIT (env): raise both (e.g. 900 / 600) the first time a config needs new CppAD code (model.wrenchState).
setsid nohup timeout ${LAUNCH_TIMEOUT:-240} bash -c "cd $WS && $SRC && ros2 launch legged_robot_mpc_controller robot.launch.py robotName:=g1 \
  mpcControllerName:=humanoid_whole_body_inverse_dynamics_mpc_controller mujocoModelFile:=$D/scene.xml \
  rviz:=${RVIZ:-false} ${RVIZCFG:+rvizconfig:=$RVIZCFG} mujoco_headless:=true baseCommandDevice:=none controllersFile:=$D/config.yaml ${LAUNCH_ARGS:-}" > "$D/sim.log" 2>&1 < /dev/null &
w=0
until grep -qE -- "-> locomotion|first MPC advance:|bring-up FAULT|on_configure failed|init failed" "$D/sim.log" 2>/dev/null; do
  sleep 1; w=$((w+1)); [ $w -gt ${LOCO_WAIT:-120} ] && { echo "RUN: no locomotion after ${LOCO_WAIT:-120} s"; break; }
done > "$D/summary.txt"
if grep -q "no locomotion" "$D/summary.txt"; then
  # Why: the launch's last lines, and anything that died, in the summary (sim.log is long).
  { echo "--- sim.log: process exits / errors"; grep -aE "process has died|exited with code|died|Segmentation|what\(\)|error|Error" "$D/sim.log" | grep -v simulation_reset | head -12;
    echo "--- sim.log: tail"; tail -15 "$D/sim.log"; } >> "$D/summary.txt"
fi
sleep 2
# PROFILE (env): vx | vy | yaw | stops, see tune_probe.py; the command scale of that axis.
KEY=maxDisplacementVelocityX; [ "${PROFILE:-vx}" = vy ] && KEY=maxDisplacementVelocityY; [ "${PROFILE:-vx}" = yaw ] && KEY=maxRotationVelocity
VMAX=$(python3 -c "import yaml; c=yaml.safe_load(open('$D/config.yaml')); print(c['humanoid_whole_body_inverse_dynamics_mpc_controller']['ros__parameters']['ocs2']['reference']['$KEY'])")
# PROFILE=script (SCRIPT=<json>, tests/tuning/suite/*.json): all three command scales, scored by score_suite.py.
read -r VMAX_X VMAX_Y VMAX_YAW < <(python3 -c "import yaml; r=yaml.safe_load(open('$D/config.yaml'))['humanoid_whole_body_inverse_dynamics_mpc_controller']['ros__parameters']['ocs2']['reference']; print(r['maxDisplacementVelocityX'], r['maxDisplacementVelocityY'], r['maxRotationVelocity'])")
export VMAX_X VMAX_Y VMAX_YAW SCRIPT=${SCRIPT:-}
C=/humanoid_whole_body_inverse_dynamics_mpc_controller
setsid bash -c "$SRC && exec ros2 bag record -s mcap -o $D/bag --topics /joint_states /mujoco/ground_truth/odom $C/tracking $C/solver $C/tracking_names $C/solver_names" > "$D/bag.log" 2>&1 < /dev/null &
sleep 2
{
  bash -c "$SRC && PROFILE=${PROFILE:-vx} python3 $HERE/tune_probe.py $D/run $VX $VMAX ${HEIGHT:-0.785}" 2>&1 | grep -E "PROBE|Traceback|Error"
  # SIGTERM, not SIGINT: a background job of a non-interactive shell starts with SIGINT ignored.
  BP=$(pgrep -f "[r]os2 bag record -s mcap -o $D/bag"); kill -TERM $BP 2>/dev/null
  for i in $(seq 1 30); do kill -0 $BP 2>/dev/null || break; sleep 1; done
  bash -c "$SRC && python3 $HERE/bag_to_npz.py $D/bag $D/run_meta.npz $D/run_data.npz" 2>&1 | grep -E "^bag:|Traceback|Error"
  [ -f "$D/run_data.npz" ] && rm -rf "$D/bag" "$D/run_meta.npz"
  if [ "${PROFILE:-vx}" = script ]; then
    [ -f "$D/run_data.npz" ] && python3 "$HERE/score_suite.py" "$D/run" "$TAG" 2>&1 | grep -E "SUITE|^SEG|Traceback|Error"
  else
    [ -f "$D/run_data.npz" ] && python3 "$HERE/plot_tracking.py" "$D/run" "$TAG" 2>&1 | grep -E "SUMMARY|DIAG|Traceback|Error"
  fi
  grep -aoE "MPC_TIMING\].*utilization=[0-9.]+%|MPC_HEALTH\] equality SSE mean=[^|]*" "$D/sim.log" | tail -2 | sed -E 's/latest advance.*target period=[0-9.]+ ms, //'
  grep -aoE "bring-up FAULT: trunk tilt [0-9.]+|on_configure failed.*" "$D/sim.log" | head -1
} | tee -a "$D/summary.txt"
pkill -INT -f "ros2 launch legged_robot_mpc_controller" 2>/dev/null; sleep 4
pkill -9 -x mujoco_ros2_con 2>/dev/null; pkill -9 -x rviz2 2>/dev/null; pkill -9 -x static_transfor 2>/dev/null; pkill -9 -x robot_state_pub 2>/dev/null
for f in "$D"/run_shot*.xwd; do [ -s "$f" ] && python3 "$HERE/xwd2png.py" "$f" "${f%.xwd}.png" && rm -f "$f"; done
for f in "$D"/run_*; do mv "$f" "$D/${f##*/run_}"; done
echo "RUN_DONE $TAG -> $D"
