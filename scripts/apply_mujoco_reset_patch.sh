#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEPENDENCY="${1:-${ROOT}/legged_robot_mpc_controller_dependencies/mujoco_ros2_control}"
PATCH_FILE="${ROOT}/patches/mujoco_ros2_control_reset.patch"

if [[ ! -d "${DEPENDENCY}/.git" ]]; then
  echo "mujoco_ros2_control checkout not found: ${DEPENDENCY}" >&2
  echo "Usage: $0 [path/to/mujoco_ros2_control]" >&2
  exit 2
fi
if git -C "${DEPENDENCY}" apply --ignore-whitespace --reverse --check "${PATCH_FILE}" 2>/dev/null; then
  echo "Coordinated reset patch is already applied."
  exit 0
fi
if ! git -C "${DEPENDENCY}" apply --ignore-whitespace --check "${PATCH_FILE}"; then
  echo "The reset patch does not match this mujoco_ros2_control checkout." >&2
  echo "Check that it uses the jazzy revision recorded in legged_robot_mpc_controller.repos." >&2
  exit 1
fi
git -C "${DEPENDENCY}" apply --ignore-whitespace "${PATCH_FILE}"
echo "Applied coordinated reset support. Rebuild mujoco_ros2_control and this package."
