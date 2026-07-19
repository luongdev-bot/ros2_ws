#!/usr/bin/env bash
# Kill the simulation started by this workspace's run scripts.
#
# Why this exists: each Gazebo instance runs its own clock and its own
# ros_gz_bridge, and every orphaned static_transform_publisher keeps republishing
# TF with the sim time of the run it belonged to. Two of those and tf2 sees
# timestamps jump backwards ("Detected jump back in time. Clearing TF buffer."),
# the robot jitters in RViz, and RTAB-Map can never resolve the camera TF.
#
# Matching is deliberately NARROW. Two earlier versions were wrong:
#   * `pgrep -x static_transform_publisher` matched nothing, because Linux
#     truncates the process name to 15 chars ("static_transfor") - orphans
#     silently piled up.
#   * Broad `pkill -f rviz2|rtabmap|spawner` killed unrelated work in other
#     terminals and workspaces, and even the shell invoking this script.
# So we match on the full command line (not truncated) AND require the process
# to belong to this workspace / this robot, never on a bare tool name.
#
# Run it by hand any time with:  bash ~/ros2_ws/scripts/stop_sim.sh

WS="${WS:-$HOME/ros2_ws}"
MYPID=$$

# Patterns that identify *this* simulation only. Killing the parent
# `ros2 launch ...` process makes it tear down its own child nodes, which is why
# those come first - they cover children (robot_state_publisher, ros_gz_bridge,
# rviz2, ...) whose own command lines live under /opt/ros and would otherwise
# need dangerously generic patterns to match.
PATTERNS=(
  "ros2 launch jetrover_gazebo"     # parent launch: sim
  "ros2 launch slam"                # parent launch: 2D / 3D SLAM
  "ros2 launch navigation"          # parent launch: Nav2
  "ros2 launch peripherals"         # parent launch: joystick
  "jetrover_gazebo"                 # our gazebo/gazebo_arm launches
  "jetrover_world"                  # the world file passed to gz
  "$WS/install"                     # anything launched from this workspace
  "jetrover/base_footprint/lidar"   # our static TF bridges (frame names)
  "jetrover/link4/depth_camera"
  # Killing the parent `ros2 launch` does NOT always take its children with it.
  # Orphaned nodes keep their own command line under /opt/ros with only a temp
  # params file, e.g.
  #   /opt/ros/humble/lib/ros_gz_bridge/parameter_bridge --params-file /tmp/...
  # so none of the patterns above can see them. A single surviving
  # parameter_bridge is enough to republish /clock and /tf from a dead Gazebo
  # and make tf2 jump back in time, which silently breaks SLAM.
  # Matching the full install path of the executables this workspace launches is
  # far narrower than a bare name like "rviz2" (which would match any command
  # line merely mentioning it). Caveat: if you run ANOTHER ROS project using
  # these same nodes at the same time, they will be stopped too.
  "lib/ros_gz_bridge/parameter_bridge"
  "lib/ros_gz_sim/create"
  "lib/robot_state_publisher/robot_state_publisher"
  "lib/tf2_ros/static_transform_publisher"
  "lib/controller_manager/spawner"
  "lib/slam_toolbox/sync_slam_toolbox_node"
  "lib/rtabmap_slam/rtabmap"
  "lib/rtabmap_sync/rgbd_sync"
  "lib/rviz2/rviz2"
  # Nav2 runs all its servers inside this container, and the joystick brings up
  # joy_node; both orphan the same way as parameter_bridge did.
  "lib/rclcpp_components/component_container_isolated"
  "lib/joy/joy_node"
)

collect() {
  for p in "${PATTERNS[@]}"; do
    pgrep -f -- "$p" 2>/dev/null
  done | sort -u | grep -v "^${MYPID}$"
}

pids=$(collect)
if [ -n "$pids" ]; then
  # shellcheck disable=SC2086
  kill -TERM $pids 2>/dev/null
  sleep 2
  pids=$(collect)
  # shellcheck disable=SC2086
  [ -n "$pids" ] && kill -KILL $pids 2>/dev/null
  sleep 1
fi

# The Gazebo server/GUI are ruby wrappers whose command line contains the world
# path, so they are covered above. Anything still holding the world file is a
# stale server that would keep publishing /clock.
left=$(collect | wc -l)
if [ "$left" -eq 0 ]; then
  echo "Previous simulation processes cleaned up."
else
  echo "WARNING: $left simulation process(es) still running after cleanup."
fi
