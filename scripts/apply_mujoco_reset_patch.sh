#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DEPENDENCY="${1:-${ROOT}/legged_robot_mpc_controller_dependencies/mujoco_ros2_control}"
PATCH_FILE="${ROOT}/patches/mujoco_ros2_control_builtin_reset.patch"
LEGACY_PATCHES=(
  "${ROOT}/patches/legacy/mujoco_ros2_control_builtin_reset_v1.patch"
  "${ROOT}/patches/legacy/mujoco_ros2_control_reset_snapshot.patch"
  "${ROOT}/patches/legacy/mujoco_ros2_control_reset_paced.patch"
  "${ROOT}/patches/legacy/mujoco_ros2_control_reset_initial.patch"
)

if [[ ! -d "${DEPENDENCY}/.git" ]]; then
  echo "mujoco_ros2_control checkout not found: ${DEPENDENCY}" >&2
  echo "Usage: $0 [path/to/mujoco_ros2_control]" >&2
  exit 2
fi

if git -C "${DEPENDENCY}" apply --reverse --check "${PATCH_FILE}" 2>/dev/null; then
  echo "Built-in MuJoCo reset integration is already applied."
  exit 0
fi

# Upgrade checkouts patched by earlier reset revisions: remove the matching old patch
# first, then apply the smaller built-in-reset integration to the clean Jazzy source.
for legacy_patch in "${LEGACY_PATCHES[@]}"; do
  if git -C "${DEPENDENCY}" apply --reverse --check --ignore-whitespace "${legacy_patch}" 2>/dev/null; then
    git -C "${DEPENDENCY}" apply --reverse --ignore-whitespace "${legacy_patch}"
    echo "Removed a previous coordinated-reset patch before upgrading."
    break
  fi
done

if git -C "${DEPENDENCY}" apply --check "${PATCH_FILE}" 2>/dev/null; then
  git -C "${DEPENDENCY}" apply "${PATCH_FILE}"
  echo "Applied built-in MuJoCo reset integration. Rebuild mujoco_ros2_control and this package."
  exit 0
fi

echo "Reset patch does not match this mujoco_ros2_control checkout." >&2
echo "The helper supports clean Jazzy checkouts and the earlier reset patches from this repository." >&2
exit 1
