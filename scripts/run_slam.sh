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

# BOTH modes use gazebo_arm.launch.py, not the lighter gazebo.launch.py.
# gazebo.launch.py has no gz_ros2_control and no arm_controller, so nothing
# actuates or holds the five revolute arm joints: the arm collapses under
# gravity, and because /joint_states is bridged straight out of Gazebo, RViz
# faithfully renders the collapsed arm. 2D SLAM does not use the camera, but
# it still wants a robot that looks like the robot.
#
# The arm boots at the "horizontal" pose (slam_initial_positions.yaml, from
# ~/ActionGroups/horizontal.d6a) rather than the package default pick_init:
# pick_init swings joint1 90 deg and aims the depth camera down-right, so 3D
# SLAM mapped the floor beside the robot. Horizontal aims it forward and keeps
# every arm link above the lidar plane, so it is right for 2D as well.
#
# 3D additionally needs the depth camera, hence pose_arm_camera.sh - now only
# insurance in case gz_ros2_control ignores the initial positions. PREP is
# chained with && on purpose: if it fails the camera may be misaimed and
# RTAB-Map would map nothing, so SLAM must NOT start.
# use_gpu:=true -> CUDA ORB/FAST (source rtabmap built against CUDA OpenCV 4.10).
#
# Single-quoted so the substitution runs inside the spawned terminal, i.e.
# AFTER install/setup.bash has been sourced.
SLAM_ARM_POSE='$(ros2 pkg prefix jetrover_moveit_config)/share/jetrover_moveit_config/config/slam_initial_positions.yaml'
SIM_LAUNCH="ros2 launch jetrover_gazebo gazebo_arm.launch.py initial_positions_file:=$SLAM_ARM_POSE"

case "$MODE" in
  2d) SLAM_LAUNCH="ros2 launch slam slam.launch.py"
      SLAM_TITLE="SLAM 2D (slam_toolbox)"
      PREP="" ;;
  3d) SLAM_LAUNCH="ros2 launch slam rtabmap_slam.launch.py use_gpu:=true"
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
