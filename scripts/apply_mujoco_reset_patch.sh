#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEPENDENCY="${1:-${ROOT}/legged_robot_mpc_controller_dependencies/mujoco_ros2_control}"
PATCH_FILE="${ROOT}/patches/mujoco_ros2_control_builtin_reset.patch"

if [[ ! -d "${DEPENDENCY}/.git" ]]; then
  echo "mujoco_ros2_control checkout not found: ${DEPENDENCY}" >&2
  echo "Usage: $0 [path/to/mujoco_ros2_control]" >&2
  exit 2
fi

if git -C "${DEPENDENCY}" apply --reverse --check "${PATCH_FILE}" 2>/dev/null; then
  echo "Built-in MuJoCo reset integration is already applied."
  exit 0
fi

if git -C "${DEPENDENCY}" apply --check "${PATCH_FILE}" 2>/dev/null; then
  git -C "${DEPENDENCY}" apply "${PATCH_FILE}"
  echo "Applied built-in MuJoCo reset integration. Rebuild mujoco_ros2_control and this package."
  exit 0
fi

echo "Reset patch does not match this mujoco_ros2_control checkout; verify it uses the jazzy revision." >&2
exit 1
