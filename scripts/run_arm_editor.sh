#!/usr/bin/env bash
# Launch the Arm Action Group editor (and the Gazebo sim + controllers it needs)
# each in its OWN terminal window, so you can teach and replay arm motions.
# Usage:  run_arm_editor.sh
#
#   Terminal 1: Gazebo + controllers   Terminal 2: arm_motion_server + editor
#
# If a simulation is ALREADY running, only the editor terminal is opened.
#
# NOTE: ROS setup files reference unset variables, so this script sources them
# only inside child `bash -c` shells (which do not run under `set -u`), never
# at the top level. That was the bug in the old package launcher.
set -u

WS="${ARM_MOTION_WS:-$HOME/ros2_ws}"

ENV_SETUP="source /opt/ros/humble/setup.bash; source '$WS/install/setup.bash'; export MACHINE_TYPE=JetRover_Mecanum; export LIDAR_TYPE=A1"
# `ros2 control list_controllers` BLOCKS when no controller_manager exists yet,
# so wait for that node first (a non-blocking check) before ever calling it;
# and bound the call with `timeout` so an unhealthy manager cannot hang forever.
WAIT_CTRL="echo 'Waiting for the controller_manager...'; until ros2 node list 2>/dev/null | grep -qx /controller_manager; do sleep 2; done; echo 'Waiting for arm_controller to become active...'; until timeout 5 ros2 control list_controllers 2>/dev/null | grep -q 'arm_controller.*active'; do sleep 2; done; sleep 1"

# Report a failure the user can actually see: a launcher started from a desktop
# icon (Terminal=false) has no console, so a bare `exit 1` looks like "nothing
# happened". Try the GUI dialog tools in turn before giving up.
die() {
  local msg="$1"
  echo "$msg" >&2
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --title="Arm editor" --text="$msg" 2>/dev/null
  elif command -v notify-send >/dev/null 2>&1; then
    notify-send "Arm editor" "$msg" 2>/dev/null
  elif command -v xmessage >/dev/null 2>&1; then
    xmessage -center "$msg" 2>/dev/null
  fi
  exit 1
}

if [ ! -f "$WS/install/setup.bash" ]; then
  die "Workspace not built: $WS/install/setup.bash is missing. Run: cd $WS && colcon build"
fi

command -v gnome-terminal >/dev/null || die "gnome-terminal is not installed."

open_term() {
  local title="$1"; shift
  gnome-terminal --title="$title" -- bash -c "$*; echo; echo '--- process exited (press Enter to close) ---'; read _" &
}

# Is a Gazebo simulation already up? Probe the controller_manager node with a
# non-blocking `node list` — never `list_controllers`, which blocks when no
# controller_manager is running yet and would freeze this launcher.
already_up=$(bash -c "$ENV_SETUP; ros2 node list 2>/dev/null | grep -qx /controller_manager && echo yes || echo no")

if [ "$already_up" = "yes" ]; then
  open_term "Arm editor" "$ENV_SETUP; ros2 launch arm_motion arm_motion.launch.py editor:=true"
  echo "Simulation already running - started the arm editor terminal only."
else
  open_term "Gazebo (arm)" "$ENV_SETUP; ros2 launch jetrover_gazebo gazebo_moveit.launch.py"
  open_term "Arm editor"  "$ENV_SETUP; $WAIT_CTRL; ros2 launch arm_motion arm_motion.launch.py editor:=true"
  echo "Started 2 terminals: Gazebo, Arm editor."
fi
