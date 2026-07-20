#!/usr/bin/env bash
# Thin wrapper: delegate to the workspace launcher, which opens the editor
# (and the Gazebo sim it needs) in its own terminal windows.
#
# The old version sourced ROS under `set -u`, which aborts because ROS setup
# files reference unset variables. The workspace launcher sources ROS only
# inside child shells, so it does not hit that.
#
# Kept for the packaged/desktop path; the real logic lives in
# $WS/scripts/run_arm_editor.sh.

WS="${ARM_MOTION_WS:-$HOME/ros2_ws}"
LAUNCHER="$WS/scripts/run_arm_editor.sh"

if [ -x "$LAUNCHER" ]; then
  exec "$LAUNCHER" "$@"
fi

# Fallback: run the editor directly if the workspace launcher is missing.
# NOTE: no `set -u` here — ROS setup files are not nounset-safe.
# shellcheck disable=SC1090,SC1091
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash" || {
  echo "ROS not found at /opt/ros/${ROS_DISTRO:-humble}" >&2
  exit 1
}
if [ ! -f "$WS/install/setup.bash" ]; then
  echo "Workspace not built: $WS/install/setup.bash is missing." >&2
  echo "Run: cd $WS && colcon build" >&2
  exit 1
fi
# shellcheck disable=SC1090,SC1091
source "$WS/install/setup.bash"
export MACHINE_TYPE="${MACHINE_TYPE:-JetRover_Mecanum}"
export LIDAR_TYPE="${LIDAR_TYPE:-A1}"
exec ros2 run arm_motion arm_motion_editor "$@"
