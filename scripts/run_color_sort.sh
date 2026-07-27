#!/usr/bin/env bash
# Launch the closed-loop colour-sorting demo. The grasp executor computes all
# arm motion with IK, so no external motion library or playback service is used.
#
# Usage:
#   run_color_sort.sh
#   run_color_sort.sh auto_grasp:=true
set -u

WS="${ARM_MOTION_WS:-$HOME/ros2_ws}"

ENV_SETUP="source /opt/ros/humble/setup.bash; source '$WS/install/setup.bash'; export MACHINE_TYPE=JetRover_Mecanum; export LIDAR_TYPE=A1"

# Report failures through the desktop when this is started without a terminal.
die() {
  local msg="$1"
  echo "$msg" >&2
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --title="Colour sort" --text="$msg" 2>/dev/null
  elif command -v notify-send >/dev/null 2>&1; then
    notify-send "Colour sort" "$msg" 2>/dev/null
  elif command -v xmessage >/dev/null 2>&1; then
    xmessage -center "$msg" 2>/dev/null
  fi
  exit 1
}

if [ ! -f "$WS/install/setup.bash" ]; then
  die "Workspace not built: $WS/install/setup.bash is missing. Run: cd $WS && colcon build"
fi

command -v gnome-terminal >/dev/null || die "gnome-terminal is not installed."

if [ "$#" -gt 1 ]; then
  die "Usage: $0 [auto_grasp:=true|auto_grasp:=false]"
fi

AUTO_GRASP_ARG="${1:-}"
case "$AUTO_GRASP_ARG" in
  ""|auto_grasp:=true|auto_grasp:=false) ;;
  *) die "Usage: $0 [auto_grasp:=true|auto_grasp:=false]" ;;
esac

open_term() {
  local title="$1"
  shift
  gnome-terminal --title="$title" -- bash -c "$*; echo; echo '--- process exited (press Enter to close) ---'; read _" &
}

LAUNCH_COMMAND="$ENV_SETUP; ros2 launch jetrover_grasp grasp_demo.launch.py"
if [ -n "$AUTO_GRASP_ARG" ]; then
  LAUNCH_COMMAND="$LAUNCH_COMMAND $AUTO_GRASP_ARG"
fi

open_term "Closed-loop colour sort" "$LAUNCH_COMMAND"

echo "Started 1 terminal: Gazebo, colour perception, and IK grasp executor."
echo
echo "Trigger one pick-place cycle with:"
echo "  ros2 service call /grasp_executor/grasp_next std_srvs/srv/Trigger {}"
echo
echo "Watch detections with:"
echo "  ros2 topic echo /color_pick/detections"
