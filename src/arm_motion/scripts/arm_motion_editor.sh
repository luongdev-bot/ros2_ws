#!/usr/bin/env bash
# Launcher for the arm action group editor.
#
# Sources ROS and the workspace, then starts the Qt editor. Installed to the
# package share directory and symlinked onto the Desktop by
# scripts/install_desktop_shortcut.sh.

set -euo pipefail

ROS_DISTRO_DEFAULT="humble"
WORKSPACE="${ARM_MOTION_WS:-$HOME/ros2_ws}"

if [[ -f "/opt/ros/${ROS_DISTRO:-$ROS_DISTRO_DEFAULT}/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO:-$ROS_DISTRO_DEFAULT}/setup.bash"
else
  echo "ROS not found at /opt/ros/${ROS_DISTRO:-$ROS_DISTRO_DEFAULT}" >&2
  exit 1
fi

if [[ -f "${WORKSPACE}/install/setup.bash" ]]; then
  # shellcheck disable=SC1090
  source "${WORKSPACE}/install/setup.bash"
else
  echo "Workspace not built: ${WORKSPACE}/install/setup.bash is missing." >&2
  echo "Run: cd ${WORKSPACE} && colcon build" >&2
  exit 1
fi

exec ros2 run arm_motion arm_motion_editor "$@"
