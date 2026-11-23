#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEPENDENCY="${1:-${ROOT}/legged_robot_mpc_controller_dependencies/mujoco_ros2_control}"
PATCH_FILE="${ROOT}/patches/mujoco_ros2_control_reset.patch"
FOLLOWUP_PATCH="${ROOT}/patches/mujoco_ros2_control_reset_followup.patch"
SNAPSHOT_PATCH="${ROOT}/patches/mujoco_ros2_control_reset_followup2.patch"

if [[ ! -d "${DEPENDENCY}/.git" ]]; then
  echo "mujoco_ros2_control checkout not found: ${DEPENDENCY}" >&2
  echo "Usage: $0 [path/to/mujoco_ros2_control]" >&2
  exit 2
fi

if git -C "${DEPENDENCY}" apply --ignore-whitespace --reverse --check "${PATCH_FILE}" 2>/dev/null; then
  echo "Coordinated reset patch is already current."
  exit 0
fi

if git -C "${DEPENDENCY}" apply --ignore-whitespace --check "${PATCH_FILE}" 2>/dev/null; then
  git -C "${DEPENDENCY}" apply --ignore-whitespace "${PATCH_FILE}"
  echo "Applied coordinated reset support. Rebuild mujoco_ros2_control and this package."
  exit 0
fi

# Upgrade the first reset patch with frame-loop pacing protection if necessary.
if git -C "${DEPENDENCY}" apply --ignore-whitespace --check "${FOLLOWUP_PATCH}" 2>/dev/null; then
  git -C "${DEPENDENCY}" apply --ignore-whitespace "${FOLLOWUP_PATCH}"
fi

# Preserve the launch-time spawn pose (including initial_pose.yaml overrides), rather than
# resetting only to the model's raw MJCF qpos0.
if git -C "${DEPENDENCY}" apply --ignore-whitespace --check "${SNAPSHOT_PATCH}" 2>/dev/null; then
  git -C "${DEPENDENCY}" apply --ignore-whitespace "${SNAPSHOT_PATCH}"
  echo "Upgraded coordinated reset to restore the launch-time robot pose. Rebuild both packages."
  exit 0
fi

echo "The reset patch does not match this mujoco_ros2_control checkout." >&2
echo "Check that it uses the jazzy revision recorded in legged_robot_mpc_controller.repos." >&2
exit 1
