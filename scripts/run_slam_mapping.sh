#!/usr/bin/env bash
# Launch a selected simulation world, 2D SLAM, and keyboard teleop in separate
# terminal windows so the user can drive the robot and save a reusable map.
# Usage:  run_slam_mapping.sh [map_name] [world_id]
#
#   map_name defaults to "warehouse". Pass a different name (e.g. "aisle2")
#   to save a new map without overwriting a previously saved one.
#   world_id defaults to "warehouse" and must match an installed worlds/*.sdf
#   basename (e.g. "turtlebot3_house").
#
#   Terminal 1: Gazebo  - selected world simulation
#   Terminal 2: SLAM    - slam_toolbox with RViz
#   Terminal 3: Teleop  - interactive w/a/s/d keyboard control
#
# NOTE: ROS setup files reference unset variables, so this script sources them
# only inside child `bash -c` shells (which do not run under `set -u`), never
# at the top level. Same rule as run_line_follow.sh.
set -u

WS="$HOME/ros2_ws"
MAP_NAME="${1:-warehouse}"
WORLD_ID="${2:-warehouse}"

# Keep the name safe to embed unquoted in the printed save command and to use
# as a filename (map_saver_cli writes "$MAP_NAME.yaml"/"$MAP_NAME.pgm").
case "$MAP_NAME" in
  *[!A-Za-z0-9_-]*|"")
    echo "Tên map không hợp lệ: '$MAP_NAME'. Chỉ dùng chữ, số, '_' và '-' (ví dụ: aisle2)." >&2
    exit 1
    ;;
esac

ENV_SETUP="source /opt/ros/humble/setup.bash; source '$WS/install/setup.bash'; export MACHINE_TYPE=JetRover_Mecanum; export LIDAR_TYPE=A1"

# Wait for an actual simulated-clock message rather than merely seeing a topic
# name. Each probe is bounded so a half-started Gazebo cannot hang ros2 forever.
WAIT_SIM="echo 'Waiting for the simulation clock (/clock)...'; until timeout 5 ros2 topic echo /clock --once >/dev/null 2>&1; do sleep 2; done; echo 'Simulation clock detected.'; sleep 1"

# Report a failure the user can actually see when this launcher is started from
# a desktop icon without an attached console.
die() {
  local msg="$1"
  echo "$msg" >&2
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --title="JetRover SLAM mapping" --text="$msg" 2>/dev/null
  elif command -v notify-send >/dev/null 2>&1; then
    notify-send "JetRover SLAM mapping" "$msg" 2>/dev/null
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

MAPS_DIR="$WS/src/voice_llm_scenarios/maps"
if [ ! -d "$MAPS_DIR" ]; then
  die "Map output directory is missing: $MAPS_DIR"
fi

MAP_OUT="$MAPS_DIR/$MAP_NAME"
if [ -f "$MAP_OUT.yaml" ]; then
  echo "LƯU Ý: $MAP_OUT.yaml đã tồn tại và sẽ bị GHI ĐÈ khi bạn lưu bản đồ ở bước 3."
  echo "Nếu muốn giữ lại bản đồ cũ, hãy chạy lại: $0 <tên_map_mới>"
  echo
fi

command -v gnome-terminal >/dev/null || die "gnome-terminal is not installed."

# Commands inherit a real terminal. In particular, do not append a read prompt
# or redirect teleop: teleop_key_control must read the user's keystrokes itself.
open_term() {
  local title="$1"; shift
  gnome-terminal --title="$title" -- bash -c "$*" &
}

open_term "Gazebo ($WORLD_ID)"             "$ENV_SETUP; ros2 launch jetrover_gazebo gazebo.launch.py world:='$WORLD' spawn_x:=$SPAWN_X spawn_y:=$SPAWN_Y spawn_yaw:=$SPAWN_YAW"
open_term "SLAM (slam_toolbox + RViz)"     "$ENV_SETUP; $WAIT_SIM; ros2 launch slam slam.launch.py"
open_term "Teleop (w/a/s/d để lái)"       "$ENV_SETUP; $WAIT_SIM; ros2 run peripherals teleop_key_control"

echo "Đã mở 3 terminal: Gazebo, SLAM (kèm RViz), và Teleop."
echo
echo "1. Chuyển sang terminal Teleop và lái robot đi khắp world '$WORLD_ID' bằng các phím"
echo "   được teleop_key_control hướng dẫn. Hãy đi qua các phòng/lối đi và quay đầu ở"
echo "   các góc để lidar quét đủ 360 độ. Quá trình thường mất khoảng 5-10 phút."
echo
echo "2. Theo dõi bản đồ đang được dựng trong cửa sổ RViz do terminal SLAM tự mở."
echo
echo "3. Khi bản đồ đã bao phủ đủ khu vực cần dùng, MỞ MỘT TERMINAL MỚI (không phải"
echo "   một trong 3 terminal trên) và chạy đúng hai lệnh sau:"
echo
echo "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash"
echo "ros2 run nav2_map_server map_saver_cli -f $MAP_OUT"
echo
echo "   Lệnh lưu sẽ tạo $MAP_NAME.yaml và $MAP_NAME.pgm trong"
echo "   ~/ros2_ws/src/voice_llm_scenarios/maps/."
echo
echo "   Muốn lưu một bản đồ MỚI mà không ghi đè bản đồ cũ: chạy lại script này"
echo "   với tên khác, ví dụ:  $0 ten_map_moi"
echo
echo "4. Sau khi lưu bản đồ xong, có thể đóng cả 3 terminal Gazebo, SLAM và Teleop."
