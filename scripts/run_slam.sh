#!/usr/bin/env bash
# Launch the Gazebo simulation + SLAM + keyboard teleop, each in its OWN
# terminal window.  Usage:  run_slam.sh 2d   |   run_slam.sh 3d
#
#   Terminal 1: Gazebo        - simulation (robot, sensors, drive plugin)
#   Terminal 2: SLAM          - slam_toolbox (2d) or RTAB-Map (3d)
#   Terminal 3: Teleop        - keyboard w/a/s/d control (type in this window)
#
# Close a window to stop that component.
set -u

MODE="${1:-2d}"
WS="$HOME/ros2_ws"

# MACHINE_TYPE / LIDAR_TYPE are required by the robot xacro; without them
# robot_description is empty and the robot never spawns.
ENV_SETUP="source /opt/ros/humble/setup.bash; source '$WS/install/setup.bash'; export MACHINE_TYPE=JetRover_Mecanum; export LIDAR_TYPE=A1"
WAIT_SIM="echo 'Waiting for the simulation (/scan)...'; until ros2 topic list 2>/dev/null | grep -qx /scan; do sleep 1; done; sleep 2"

# 2D only needs the laser, so the plain sim is enough.
# 3D needs the depth camera, which is mounted on the ARM, so it needs:
#   - gazebo_moveit.launch.py: gz_ros2_control + arm_controller, and the static
#     TF depth_cam_frame -> jetrover/link4/depth_camera that RTAB-Map requires
#   - pose_arm_camera.sh: extends the arm so the camera looks forward
#     (at joint 0 the camera points straight up and RTAB-Map gets nothing)
# PREP is chained with && on purpose: if pose_arm_camera.sh fails the
# camera is still pointing up and RTAB-Map would map nothing, so SLAM must
# NOT start.
# use_gpu:=true -> CUDA ORB/FAST (source rtabmap built against CUDA OpenCV 4.10).
case "$MODE" in
  2d) SIM_LAUNCH="ros2 launch jetrover_gazebo gazebo.launch.py"
      SLAM_LAUNCH="ros2 launch slam slam.launch.py"
      SLAM_TITLE="SLAM 2D (slam_toolbox)"
      PREP="" ;;
  3d) SIM_LAUNCH="ros2 launch jetrover_gazebo gazebo_arm.launch.py"
      SLAM_LAUNCH="ros2 launch slam rtabmap_slam.launch.py use_gpu:=true"
      SLAM_TITLE="SLAM 3D (RTAB-Map, CUDA)"
      PREP="bash $WS/scripts/pose_arm_camera.sh &&" ;;
  *)  echo "Usage: $0 [2d|3d]"; exit 1 ;;
esac

# A leftover Gazebo from a previous run would publish a second /clock and
# /tf, making TF jump back in time and breaking RTAB-Map + RViz.
bash "$WS/scripts/stop_sim.sh"

command -v gnome-terminal >/dev/null || { echo "gnome-terminal is not installed"; exit 1; }

# Open a command in its own terminal window; keep the window open afterwards so
# errors stay readable.
open_term() {
  local title="$1"; shift
  gnome-terminal --title="$title" -- bash -c "$*; echo; echo '--- process exited ---'; exec bash" &
}

open_term "Gazebo"      "$ENV_SETUP; $SIM_LAUNCH"
open_term "$SLAM_TITLE" "$ENV_SETUP; $WAIT_SIM; $PREP $SLAM_LAUNCH"
open_term "Teleop (w/a/s/d)" "$ENV_SETUP; $WAIT_SIM; ros2 run peripherals teleop_key_control"

echo "Started 3 terminals: Gazebo, $SLAM_TITLE, Teleop."
