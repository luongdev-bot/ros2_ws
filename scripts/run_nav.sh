#!/usr/bin/env bash
# Launch the Gazebo simulation + live SLAM + Nav2, each in its OWN terminal
# window. Set goals with the "Nav2 Goal" tool in RViz.
# Usage:  run_nav.sh 2d   |   run_nav.sh 3d
#
#   2d -> slam_toolbox provides map->odom, Nav2 navigates the live 2D map
#   3d -> RTAB-Map provides map->odom + grid, Nav2 navigates
#
#   Terminal 1: Gazebo    Terminal 2: SLAM    Terminal 3: Nav2 (+RViz)
set -u

MODE="${1:-2d}"
WS="$HOME/ros2_ws"

ENV_SETUP="source /opt/ros/humble/setup.bash; source '$WS/install/setup.bash'; export MACHINE_TYPE=JetRover_Mecanum; export LIDAR_TYPE=A1"
WAIT_SIM="echo 'Waiting for the simulation (/scan)...'; until ros2 topic list 2>/dev/null | grep -qx /scan; do sleep 1; done; sleep 2"
WAIT_MAP="echo 'Waiting for /map from SLAM...'; until ros2 topic list 2>/dev/null | grep -qx /map; do sleep 1; done; sleep 3"

# 3D uses the arm-mounted depth camera: it needs gazebo_arm.launch.py
# (arm_controller + the depth_cam static TF, no MoveIt) and the arm extended so
# the camera looks forward. See scripts/pose_arm_camera.sh.
case "$MODE" in
  2d) SIM_LAUNCH="ros2 launch jetrover_gazebo gazebo.launch.py"
      SLAM_LAUNCH="ros2 launch slam slam.launch.py use_rviz:=false"
      SLAM_TITLE="SLAM 2D (slam_toolbox)"
      PREP="" ;;
  3d) SIM_LAUNCH="ros2 launch jetrover_gazebo gazebo_arm.launch.py"
      SLAM_LAUNCH="ros2 launch slam rtabmap_slam.launch.py use_rviz:=false use_gpu:=true"
      SLAM_TITLE="SLAM 3D (RTAB-Map, CUDA)"
      PREP="bash $WS/scripts/pose_arm_camera.sh &&" ;;
  *)  echo "Usage: $0 [2d|3d]"; exit 1 ;;
esac

# A leftover Gazebo from a previous run would publish a second /clock and
# /tf, making TF jump back in time and breaking RTAB-Map + RViz.
bash "$WS/scripts/stop_sim.sh"

command -v gnome-terminal >/dev/null || { echo "gnome-terminal is not installed"; exit 1; }

open_term() {
  local title="$1"; shift
  gnome-terminal --title="$title" -- bash -c "$*; echo; echo '--- process exited ---'; exec bash" &
}

open_term "Gazebo"      "$ENV_SETUP; $SIM_LAUNCH"
open_term "$SLAM_TITLE" "$ENV_SETUP; $WAIT_SIM; $PREP $SLAM_LAUNCH"
open_term "Nav2 + RViz" "$ENV_SETUP; $WAIT_MAP; ros2 launch navigation navigation.launch.py use_sim_time:=true localization:=false use_rviz:=true"

echo "Started 3 terminals: Gazebo, $SLAM_TITLE, Nav2."
