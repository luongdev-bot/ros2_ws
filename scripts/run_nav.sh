#!/usr/bin/env bash
# Launch the Gazebo simulation + live SLAM + Nav2, each in its OWN terminal
# window. Set goals with the "Nav2 Goal" tool in RViz.
# Usage:  run_nav.sh [2d|3d] [world_id]
#
#   2d -> slam_toolbox provides map->odom, Nav2 navigates the live 2D map
#   3d -> RTAB-Map provides map->odom + grid, Nav2 navigates
#   world_id is optional; when omitted, the selected Gazebo launch file keeps
#   its existing default world. Example: run_nav.sh 2d turtlebot3_house
#
#   Terminal 1: Gazebo    Terminal 2: SLAM    Terminal 3: Nav2 (+RViz)
set -u

MODE="${1:-2d}"
WORLD_ID="${2:-}"
WS="$HOME/ros2_ws"

ENV_SETUP="source /opt/ros/humble/setup.bash; source '$WS/install/setup.bash'; export MACHINE_TYPE=JetRover_Mecanum; export LIDAR_TYPE=A1"
WAIT_SIM="echo 'Waiting for the simulation (/scan)...'; until ros2 topic list 2>/dev/null | grep -qx /scan; do sleep 1; done; sleep 2"
WAIT_MAP="echo 'Waiting for /map from SLAM...'; until ros2 topic list 2>/dev/null | grep -qx /map; do sleep 1; done; sleep 3"

# Report a failure the user can actually see when this launcher is started from
# a desktop icon without an attached console.
die() {
  local msg="$1"
  echo "$msg" >&2
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --title="JetRover SLAM + Nav2" --text="$msg" 2>/dev/null
  elif command -v notify-send >/dev/null 2>&1; then
    notify-send "JetRover SLAM + Nav2" "$msg" 2>/dev/null
  elif command -v xmessage >/dev/null 2>&1; then
    xmessage -center "$msg" 2>/dev/null
  fi
  exit 1
}

is_number() {
  [[ "$1" =~ ^[-+]?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][-+]?[0-9]+)?$ ]]
}

load_world_spawn() {
  local result status extra
  result="$(python3 -c '
import math
import sys

import yaml

try:
    with open(sys.argv[1]) as stream:
        data = yaml.safe_load(stream)
    if not isinstance(data, dict) or not isinstance(data.get("worlds"), list):
        raise ValueError("catalogue must contain a top-level worlds list")
    world = next(
        (item for item in data["worlds"]
         if isinstance(item, dict) and item.get("id") == sys.argv[2]),
        None,
    )
    if world is None:
        sys.exit(0)
    spawn = world.get("spawn")
    if not isinstance(spawn, dict):
        raise ValueError("matching entry has no spawn mapping")
    values = [spawn.get(key) for key in ("x", "y", "yaw")]
    if any(type(value) not in (int, float) or not math.isfinite(value)
           for value in values):
        raise ValueError("spawn x, y and yaw must be finite numbers")
    print(*values)
except Exception as exc:
    print(f"{type(exc).__name__}: {exc}", file=sys.stderr)
    sys.exit(2)
' "$CATALOG" "$WORLD_ID" 2>&1)"
  status=$?
  if [ "$status" -ne 0 ]; then
    die "Không thể đọc điểm spawn cho world '$WORLD_ID' từ catalogue '$CATALOG': $result"
  fi
  [ -n "$result" ] || return 0

  extra=""
  read -r SPAWN_X SPAWN_Y SPAWN_YAW extra <<< "$result"
  if [ -n "$extra" ] || ! is_number "$SPAWN_X" ||
      ! is_number "$SPAWN_Y" || ! is_number "$SPAWN_YAW"; then
    die "Điểm spawn của world '$WORLD_ID' không hợp lệ: '$result'. x, y và yaw phải là số."
  fi
}

if [ ! -f "$WS/install/setup.bash" ]; then
  die "Workspace not built: $WS/install/setup.bash is missing. Run: cd $WS && colcon build"
fi

SIM_WORLD_ARGS=""
GAZEBO_TITLE="Gazebo"
if [ -n "$WORLD_ID" ]; then
  WORLDS_DIR="$WS/install/jetrover_gazebo/share/jetrover_gazebo/worlds"
  WORLD="$WORLDS_DIR/$WORLD_ID.sdf"
  if [ ! -f "$WORLD" ]; then
    AVAILABLE_WORLDS=""
    for world_file in "$WORLDS_DIR"/*.sdf; do
      [ -e "$world_file" ] || continue
      AVAILABLE_WORLDS="$AVAILABLE_WORLDS ${world_file##*/}"
    done
    AVAILABLE_WORLDS="${AVAILABLE_WORLDS//.sdf/}"
    die "Không tìm thấy world '$WORLD_ID'. Các world hiện có:${AVAILABLE_WORLDS:- (không có)}. Hãy chạy: cd $WS && colcon build --packages-select jetrover_gazebo"
  fi

  SPAWN_X=0
  SPAWN_Y=0
  SPAWN_YAW=0
  CATALOG="$WS/install/jetrover_gazebo/share/jetrover_gazebo/config/world_catalog.yaml"
  load_world_spawn
  SIM_WORLD_ARGS=" world:='$WORLD' spawn_x:=$SPAWN_X spawn_y:=$SPAWN_Y spawn_yaw:=$SPAWN_YAW"
  GAZEBO_TITLE="Gazebo ($WORLD_ID)"
fi

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
  *)  die "Cách dùng: $0 [2d|3d] [world_id]" ;;
esac

if [ "$MODE" = "3d" ] && [ -n "$WORLD_ID" ]; then
  SIM_WORLD_ARGS="$SIM_WORLD_ARGS world_name:=$WORLD_ID"
fi

# A leftover Gazebo from a previous run would publish a second /clock and
# /tf, making TF jump back in time and breaking RTAB-Map + RViz.
bash "$WS/scripts/stop_sim.sh"

command -v gnome-terminal >/dev/null || die "gnome-terminal is not installed."

open_term() {
  local title="$1"; shift
  gnome-terminal --title="$title" -- bash -c "$*; echo; echo '--- process exited ---'; exec bash" &
}

open_term "$GAZEBO_TITLE" "$ENV_SETUP; $SIM_LAUNCH$SIM_WORLD_ARGS"
open_term "$SLAM_TITLE" "$ENV_SETUP; $WAIT_SIM; $PREP $SLAM_LAUNCH"
open_term "Nav2 + RViz" "$ENV_SETUP; $WAIT_MAP; ros2 launch navigation navigation.launch.py use_sim_time:=true localization:=false use_rviz:=true"

echo "Started 3 terminals: Gazebo, $SLAM_TITLE, Nav2."
