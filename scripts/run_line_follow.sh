#!/usr/bin/env bash
# Launch the figure-8 line-following demo: the robot starts on the track,
# moves its arm to line_follow_init, then drives using its depth camera.
# Usage:  run_line_follow.sh
#
#   Terminal 1: Gazebo (line-follow figure-8) + controllers
#   Terminal 2: arm_motion_server  (serves PlayMotion from the .d6a library)
#   Terminal 3: line_follow        (waits for /clock, then plays line_follow_init
#                                    before driving the base on success)
#   Terminal 4: Camera view        (rqt_image_view -> /line_follow/debug_image)
#
# If a simulation is ALREADY running, only terminals 2-4 are opened.
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

# The line_follow_init PlayMotion goal is useless until arm_motion_server is
# actually advertising that action. Wait for the node, not just the clock.
WAIT_ARM="echo 'Waiting for arm_motion_server...'; until ros2 node list 2>/dev/null | grep -qx /arm_motion_server; do sleep 2; done; sleep 1"

# arm_motion_server runs on simulated time and aborts the motion if /clock is
# still zero, so wait for Gazebo to publish at least one clock message first.
WAIT_CLOCK="echo 'Waiting for the simulation clock (/clock)...'; timeout 30 ros2 topic echo /clock --once >/dev/null 2>&1 || echo 'WARNING: /clock did not respond within 30s - continuing anyway'; sleep 1"

# Report a failure the user can actually see: a launcher started from a desktop
# icon (Terminal=false) has no console, so a bare `exit 1` looks like "nothing
# happened". Try the GUI dialog tools in turn before giving up.
die() {
  local msg="$1"
  echo "$msg" >&2
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --title="Line follow" --text="$msg" 2>/dev/null
  elif command -v notify-send >/dev/null 2>&1; then
    notify-send "Line follow" "$msg" 2>/dev/null
  elif command -v xmessage >/dev/null 2>&1; then
    xmessage -center "$msg" 2>/dev/null
  fi
  exit 1
}

if [ ! -f "$WS/install/setup.bash" ]; then
  die "Workspace not built: $WS/install/setup.bash is missing. Run: cd $WS && colcon build"
fi

WORLD="$WS/install/jetrover_gazebo/share/jetrover_gazebo/worlds/line_figure8_world.sdf"
if [ ! -f "$WORLD" ]; then
  die "Line-follow figure-8 world not installed: $WORLD is missing. Run: cd $WS && colcon build --packages-select jetrover_gazebo"
fi

# The action groups live outside the workspace, so a missing library is a
# configuration problem the user must fix - not something colcon can.
LIBRARY_DIR="${ARM_MOTION_LIBRARY_DIR:-$HOME/ActionGroups}"
if [ ! -d "$LIBRARY_DIR" ]; then
  die "Action group library not found: $LIBRARY_DIR"
fi

INIT_GROUP="$LIBRARY_DIR/line_follow_init.d6a"
if [ ! -f "$INIT_GROUP" ]; then
  die "Line-follow action group is missing from the library: $INIT_GROUP
Check or restore it in $LIBRARY_DIR."
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
  open_term "Arm motion server" "$ENV_SETUP; $WAIT_CTRL; ros2 launch arm_motion arm_motion.launch.py library_dir:='$LIBRARY_DIR'"
  open_term "Line follow"       "$ENV_SETUP; $WAIT_CTRL; $WAIT_ARM; $WAIT_CLOCK; ros2 action send_goal /arm_motion_server/play_motion arm_motion_interfaces/action/PlayMotion \"{motion_name: 'line_follow_init'}\" | tee /tmp/line_follow_init_goal.\$\$.log; grep -q 'status: SUCCEEDED' /tmp/line_follow_init_goal.\$\$.log && ros2 launch line_follow line_follow.launch.py || echo 'line_follow_init motion did not succeed - line_follow NOT started. See the goal result above and check the arm_motion_server terminal.' >&2"
  open_term "Camera view"       "$ENV_SETUP; ros2 run rqt_image_view rqt_image_view /line_follow/debug_image"
  echo "Simulation already running - started the arm server, line follow, and camera view terminals."
else
  open_term "Gazebo (line-follow figure-8)" "$ENV_SETUP; ros2 launch jetrover_gazebo gazebo_moveit.launch.py world:='$WORLD' spawn_x:=-1.5 spawn_y:=0.0 spawn_yaw:=1.5707963"
  open_term "Arm motion server"              "$ENV_SETUP; $WAIT_CTRL; ros2 launch arm_motion arm_motion.launch.py library_dir:='$LIBRARY_DIR'"
  open_term "Line follow"                    "$ENV_SETUP; $WAIT_CTRL; $WAIT_ARM; $WAIT_CLOCK; ros2 action send_goal /arm_motion_server/play_motion arm_motion_interfaces/action/PlayMotion \"{motion_name: 'line_follow_init'}\" | tee /tmp/line_follow_init_goal.\$\$.log; grep -q 'status: SUCCEEDED' /tmp/line_follow_init_goal.\$\$.log && ros2 launch line_follow line_follow.launch.py || echo 'line_follow_init motion did not succeed - line_follow NOT started. See the goal result above and check the arm_motion_server terminal.' >&2"
  open_term "Camera view"                    "$ENV_SETUP; ros2 run rqt_image_view rqt_image_view /line_follow/debug_image"
  echo "Started 4 terminals: Gazebo, Arm motion server, Line follow, Camera view."
fi

echo
echo "Watch the base command with:"
echo "  ros2 topic echo /cmd_vel"
echo "The camera view opens automatically in its own window."
echo
echo "Pause/resume line following without stopping the node:"
echo "  ros2 service call /line_follow/enable std_srvs/srv/SetBool '{data: false}'"
