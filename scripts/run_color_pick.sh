#!/usr/bin/env bash
# Launch the colour-block pick demo: the robot watches its camera and, when it
# confirms a coloured block, plays the matching action group from ~/ActionGroups.
# Usage:  run_color_pick.sh
#
#   Terminal 1: Gazebo (colour-blocks world) + controllers
#   Terminal 2: arm_motion_server  (serves PlayMotion from the .d6a library)
#   Terminal 3: color_pick         (LAB detection -> PlayMotion goals)
#
# If a simulation is ALREADY running, only terminals 2 and 3 are opened.
#
# NOTE: ROS setup files reference unset variables, so this script sources them
# only inside child `bash -c` shells (which do not run under `set -u`), never
# at the top level. Same rule as run_arm_editor.sh.
set -u

WS="${ARM_MOTION_WS:-$HOME/ros2_ws}"

ENV_SETUP="source /opt/ros/humble/setup.bash; source '$WS/install/setup.bash'; export MACHINE_TYPE=JetRover_Mecanum; export LIDAR_TYPE=A1"

# `ros2 control list_controllers` BLOCKS when no controller_manager exists yet,
# so wait for that node first (a non-blocking check) before ever calling it;
# and bound the call with `timeout` so an unhealthy manager cannot hang forever.
WAIT_CTRL="echo 'Waiting for the controller_manager...'; until ros2 node list 2>/dev/null | grep -qx /controller_manager; do sleep 2; done; echo 'Waiting for arm_controller to become active...'; until timeout 5 ros2 control list_controllers 2>/dev/null | grep -q 'arm_controller.*active'; do sleep 2; done; sleep 1"

# color_pick sends PlayMotion goals, so it is useless until arm_motion_server
# is actually advertising that action. Wait for the node, not just the clock.
WAIT_ARM="echo 'Waiting for arm_motion_server...'; until ros2 node list 2>/dev/null | grep -qx /arm_motion_server; do sleep 2; done; sleep 1"

# Report a failure the user can actually see: a launcher started from a desktop
# icon (Terminal=false) has no console, so a bare `exit 1` looks like "nothing
# happened". Try the GUI dialog tools in turn before giving up.
die() {
  local msg="$1"
  echo "$msg" >&2
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --title="Colour pick" --text="$msg" 2>/dev/null
  elif command -v notify-send >/dev/null 2>&1; then
    notify-send "Colour pick" "$msg" 2>/dev/null
  elif command -v xmessage >/dev/null 2>&1; then
    xmessage -center "$msg" 2>/dev/null
  fi
  exit 1
}

if [ ! -f "$WS/install/setup.bash" ]; then
  die "Workspace not built: $WS/install/setup.bash is missing. Run: cd $WS && colcon build"
fi

WORLD="$WS/install/jetrover_gazebo/share/jetrover_gazebo/worlds/color_blocks_world.sdf"
if [ ! -f "$WORLD" ]; then
  die "Colour-blocks world not installed: $WORLD is missing. Run: cd $WS && colcon build --packages-select jetrover_gazebo"
fi

# The action groups live outside the workspace, so a missing library is a
# configuration problem the user must fix - not something colcon can.
LIBRARY_DIR="${ARM_MOTION_LIBRARY_DIR:-$HOME/ActionGroups}"
if [ ! -d "$LIBRARY_DIR" ]; then
  die "Action group library not found: $LIBRARY_DIR"
fi

command -v gnome-terminal >/dev/null || die "gnome-terminal is not installed."

open_term() {
  local title="$1"; shift
  gnome-terminal --title="$title" -- bash -c "$*; echo; echo '--- process exited (press Enter to close) ---'; read _" &
}

# Is a Gazebo simulation already up? Probe the controller_manager node with a
# non-blocking `node list` - never `list_controllers`, which blocks when no
# controller_manager is running yet and would freeze this launcher.
already_up=$(bash -c "$ENV_SETUP; ros2 node list 2>/dev/null | grep -qx /controller_manager && echo yes || echo no")

if [ "$already_up" = "yes" ]; then
  open_term "Arm motion server" "$ENV_SETUP; ros2 launch arm_motion arm_motion.launch.py library_dir:='$LIBRARY_DIR'"
  open_term "Colour pick"       "$ENV_SETUP; $WAIT_ARM; ros2 launch arm_perception color_pick.launch.py"
  echo "Simulation already running - started the arm server and colour pick terminals."
else
  open_term "Gazebo (colour blocks)" "$ENV_SETUP; ros2 launch jetrover_gazebo gazebo_moveit.launch.py world:='$WORLD'"
  open_term "Arm motion server"      "$ENV_SETUP; $WAIT_CTRL; ros2 launch arm_motion arm_motion.launch.py library_dir:='$LIBRARY_DIR'"
  open_term "Colour pick"            "$ENV_SETUP; $WAIT_CTRL; $WAIT_ARM; ros2 launch arm_perception color_pick.launch.py"
  echo "Started 3 terminals: Gazebo, Arm motion server, Colour pick."
fi

echo
echo "Watch the detections with:"
echo "  ros2 topic echo /color_pick/detections"
echo "  ros2 run rqt_image_view rqt_image_view   # then pick /color_pick/debug_image"
echo
echo "Pause/resume triggering without stopping the node:"
echo "  ros2 service call /color_pick/enable std_srvs/srv/SetBool '{data: false}'"
