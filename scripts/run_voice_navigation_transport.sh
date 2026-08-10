#!/usr/bin/env bash
# Launch the Vietnamese voice-controlled navigation demo. Gazebo,
# Nav2 localization, and the voice agent each run in their own terminal.
# Usage:  run_voice_navigation_transport.sh
#
#   Terminal 1: Gazebo (selected saved map)
#   Terminal 2: Nav2 + RViz
#   Terminal 3: Voice agent (Vietnamese microphone input and Piper speech)
set -u

WS="${ARM_MOTION_WS:-$HOME/ros2_ws}"

ENV_SETUP="source /opt/ros/humble/setup.bash; source '$WS/install/setup.bash'; export MACHINE_TYPE=JetRover_Mecanum; export LIDAR_TYPE=A1"

# Report failures through the desktop when this is started without a terminal.
die() {
  local msg="$1"
  echo "$msg" >&2
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --title="Voice navigation transport" --text="$msg" 2>/dev/null
  elif command -v notify-send >/dev/null 2>&1; then
    notify-send "Voice navigation transport" "$msg" 2>/dev/null
  elif command -v xmessage >/dev/null 2>&1; then
    xmessage -center "$msg" 2>/dev/null
  fi
  exit 1
}

if [ ! -f "$WS/install/setup.bash" ]; then
  die "Workspace not built: $WS/install/setup.bash is missing. Run: cd $WS && colcon build"
fi

OLLAMA_BIN="$HOME/.local/bin/ollama"
VOICE_PYTHON="$HOME/voice_llm_env/bin/python3"
PIPER_VOICE="$HOME/.local/share/piper-voices/vi_VN-vais1000-medium.onnx"

if [ ! -x "$OLLAMA_BIN" ]; then
  die "Ollama executable is missing or not executable: $OLLAMA_BIN"
fi
if [ ! -x "$VOICE_PYTHON" ]; then
  die "Voice-agent Python executable is missing or not executable: $VOICE_PYTHON"
fi
if [ ! -f "$PIPER_VOICE" ]; then
  die "Piper Vietnamese voice model is missing: $PIPER_VOICE"
fi

command -v gnome-terminal >/dev/null || die "gnome-terminal is not installed."

if ! curl -s --max-time 2 http://localhost:11434/api/version >/dev/null; then
  echo "Ollama is not responding; starting $OLLAMA_BIN serve..."
  nohup "$OLLAMA_BIN" serve > /tmp/ollama_serve_voice_demo.log 2>&1 &
  disown

  ollama_ready=false
  for _ in {1..15}; do
    sleep 1
    if curl -s --max-time 2 http://localhost:11434/api/version >/dev/null; then
      ollama_ready=true
      break
    fi
  done

  if [ "$ollama_ready" != "true" ]; then
    die "Không khởi động được Ollama. Kiểm tra ~/.local/bin/ollama và log /tmp/ollama_serve_voice_demo.log"
  fi
fi

LAUNCHER_MODULE=$(find "$WS/install" -name sim_launcher_gui.py -print -quit)
if [ -n "$LAUNCHER_MODULE" ]; then
  JETROVER_SITE_PACKAGES=$(dirname "$(dirname "$LAUNCHER_MODULE")")
  JETROVER_IMPORT_ROOT="$JETROVER_SITE_PACKAGES"
else
  # --symlink-install uses an egg-link instead of copying the Python module.
  JETROVER_EGG_LINK=$(find "$WS/install" -type f \
    -path '*/site-packages/jetrover-gazebo.egg-link' -print -quit)
  [ -n "$JETROVER_EGG_LINK" ] || die "Không tìm thấy bản cài sim_launcher_gui.py trong $WS/install."
  JETROVER_SITE_PACKAGES=$(dirname "$JETROVER_EGG_LINK")
  IFS= read -r JETROVER_IMPORT_ROOT < "$JETROVER_EGG_LINK"
  [ -f "$JETROVER_IMPORT_ROOT/jetrover_gazebo/sim_launcher_gui.py" ] || \
    die "Egg-link jetrover_gazebo không trỏ tới sim_launcher_gui.py hợp lệ: $JETROVER_EGG_LINK"
fi
JETROVER_PYTHONPATH="$JETROVER_SITE_PACKAGES:$JETROVER_IMPORT_ROOT${PYTHONPATH:+:$PYTHONPATH}"

if ! MAPS_JSON=$(PYTHONPATH="$JETROVER_PYTHONPATH" \
  JETROVER_WS="$WS" python3 -c '
import contextlib, json, sys
sys.path.insert(0, sys.argv[1])
from jetrover_gazebo.sim_launcher_gui import load_maps
with contextlib.redirect_stdout(sys.stderr):
    maps = load_maps()
print(json.dumps(maps, default=str, ensure_ascii=False))
' "$JETROVER_SITE_PACKAGES"); then
  die "Không đọc được registry map bằng sim_launcher_gui.load_maps()."
fi

if ! MAP_COUNT=$(MAPS_JSON="$MAPS_JSON" python3 -c \
  'import json, os; print(len(json.loads(os.environ["MAPS_JSON"])))'); then
  die "Dữ liệu registry map không phải JSON hợp lệ."
fi
if [ "$MAP_COUNT" -eq 0 ]; then
  die "Chưa có map nào được lưu. Dùng công cụ 'SLAM & Nav2 Launcher' (sim_launcher_gui) để tạo map trước."
fi

if ! MAP_OPTIONS_TEXT=$(MAPS_JSON="$MAPS_JSON" python3 -c '
import json, os
for entry in json.loads(os.environ["MAPS_JSON"]):
    label = entry.get("world_label", entry.get("world_file", "?"))
    mode = str(entry.get("slam_mode", "2d")).upper()
    print("{} — {} ({})".format(entry["map_name"], label, mode))
'); then
  die "Không tạo được danh sách map để lựa chọn."
fi
mapfile -t MAP_OPTIONS <<< "$MAP_OPTIONS_TEXT"

echo "Chọn map đã lưu để khởi động mô phỏng:"
PS3="Nhập số map: "
SELECTED_MAP_INDEX=""
select SELECTED_MAP_LABEL in "${MAP_OPTIONS[@]}"; do
  if [ -n "$SELECTED_MAP_LABEL" ]; then
    SELECTED_MAP_INDEX=$((REPLY - 1))
    break
  fi
  echo "Lựa chọn không hợp lệ. Hãy nhập số từ 1 đến $MAP_COUNT." >&2
done
[ -n "$SELECTED_MAP_INDEX" ] || die "Chưa chọn map."

JETROVER_WORLDS_DIR="$WS/install/jetrover_gazebo/share/jetrover_gazebo/worlds"
if ! SELECTED_VALUES=$(MAPS_JSON="$MAPS_JSON" SELECTED_MAP_INDEX="$SELECTED_MAP_INDEX" \
  JETROVER_WORLDS_DIR="$JETROVER_WORLDS_DIR" \
  PYTHONPATH="$JETROVER_PYTHONPATH" \
  JETROVER_WS="$WS" python3 -c '
import json, os, pathlib, shlex, sys
sys.path.insert(0, sys.argv[1])
from jetrover_gazebo.sim_launcher_gui import arm_pose_file
entry = json.loads(os.environ["MAPS_JSON"])[int(os.environ["SELECTED_MAP_INDEX"])]
spawn = entry["spawn"]
world_file = entry["world_file"]
values = {
    "SELECTED_WORLD_FILE": world_file,
    "SELECTED_WORLD_PATH": pathlib.Path(os.environ["JETROVER_WORLDS_DIR"]) / world_file,
    "SELECTED_SPAWN_X": spawn["x"],
    "SELECTED_SPAWN_Y": spawn["y"],
    "SELECTED_SPAWN_YAW": spawn["yaw"],
    "SELECTED_MAP_YAML": entry["yaml"],
    "SELECTED_MAP_NAME": entry["map_name"],
    "SELECTED_ARM_POSE_FILE": arm_pose_file("2d"),
}
for key, value in values.items():
    print(f"{key}={shlex.quote(str(value))}")
' "$JETROVER_SITE_PACKAGES"); then
  die "Không trích xuất được metadata của map đã chọn."
fi
eval "$SELECTED_VALUES"

[ -f "$SELECTED_WORLD_PATH" ] || die "World của map '$SELECTED_MAP_NAME' không tồn tại: $SELECTED_WORLD_PATH"
[ -f "$SELECTED_MAP_YAML" ] || die "map.yaml của map '$SELECTED_MAP_NAME' không tồn tại: $SELECTED_MAP_YAML"
[ -f "$SELECTED_ARM_POSE_FILE" ] || die "File tư thế tay máy 2D không tồn tại: $SELECTED_ARM_POSE_FILE"

SELECTED_WORLD_NAME=$(python3 -c "
import xml.etree.ElementTree as ET, sys
root = ET.parse(sys.argv[1]).getroot()
world = root.find('world')
print(world.get('name') if world is not None else '')
" "$SELECTED_WORLD_PATH")
[ -n "$SELECTED_WORLD_NAME" ] || die "Không đọc được tên world bên trong $SELECTED_WORLD_PATH"

LOCATIONS="$WS/install/voice_llm_scenarios/share/voice_llm_scenarios/config/locations_warehouse.yaml"

# Keep source checkouts usable before the next package rebuild. A current
# install places this file under share/voice_llm_scenarios/config.
if [ "$SELECTED_MAP_NAME" = "warehouse" ] && [ ! -f "$LOCATIONS" ]; then
  LOCATIONS="$WS/src/voice_llm_scenarios/config/locations_warehouse.yaml"
fi
if [ "$SELECTED_MAP_NAME" = "warehouse" ]; then
  [ -f "$LOCATIONS" ] || die "Warehouse locations file is missing: $LOCATIONS"
else
  echo "CẢNH BÁO: Map '$SELECTED_MAP_NAME' chưa có bảng địa điểm riêng — tool di chuyển theo tên địa điểm (move_to_location) sẽ không tìm thấy địa điểm nào, các tool khác vẫn hoạt động bình thường." >&2
  LOCATIONS=""
fi

printf -v SELECTED_WORLD_PATH_Q '%q' "$SELECTED_WORLD_PATH"
printf -v SELECTED_WORLD_NAME_Q '%q' "$SELECTED_WORLD_NAME"
printf -v SELECTED_SPAWN_X_Q '%q' "$SELECTED_SPAWN_X"
printf -v SELECTED_SPAWN_Y_Q '%q' "$SELECTED_SPAWN_Y"
printf -v SELECTED_SPAWN_YAW_Q '%q' "$SELECTED_SPAWN_YAW"
printf -v SELECTED_MAP_YAML_Q '%q' "$SELECTED_MAP_YAML"
printf -v SELECTED_ARM_POSE_FILE_Q '%q' "$SELECTED_ARM_POSE_FILE"
printf -v LOCATIONS_Q '%q' "$LOCATIONS"

WAIT_SIM="echo 'Waiting for the simulation (/scan)...'; until ros2 topic list 2>/dev/null | grep -qx /scan; do sleep 1; done; sleep 2"
WAIT_NAV="echo 'Waiting for Nav2 (/bt_navigator)...'; until ros2 node list 2>/dev/null | grep -qx /bt_navigator; do sleep 2; done; sleep 1"

open_term() {
  local title="$1"
  shift
  gnome-terminal --title="$title" -- bash -c "$*; echo; echo '--- process exited (press Enter to close) ---'; read _" &
}

open_term "Gazebo ($SELECTED_MAP_NAME)" "$ENV_SETUP; ros2 launch jetrover_gazebo gazebo_arm.launch.py world:=$SELECTED_WORLD_PATH_Q spawn_x:=$SELECTED_SPAWN_X_Q spawn_y:=$SELECTED_SPAWN_Y_Q spawn_yaw:=$SELECTED_SPAWN_YAW_Q initial_positions_file:=$SELECTED_ARM_POSE_FILE_Q world_name:=$SELECTED_WORLD_NAME_Q"
open_term "Nav2 + RViz" "$ENV_SETUP; $WAIT_SIM; ros2 launch navigation navigation.launch.py use_sim_time:=true localization:=true map:=$SELECTED_MAP_YAML_Q use_rviz:=true"
open_term "Voice agent (tiếng Việt)" "$ENV_SETUP; $WAIT_NAV; ros2 launch voice_llm_agent voice_agent.launch.py locations_yaml_path:=$LOCATIONS_Q"

echo "Đã mở 3 terminal:"
echo "  1. Gazebo (map: $SELECTED_MAP_NAME)"
echo "  2. Nav2 + RViz"
echo "  3. Voice agent (tiếng Việt)"
echo
echo "SYSTEM_PROMPT hiện tại của agent (nguyên văn):"
echo "Bạn là một robot thông minh trong môi trường mô phỏng."
echo "Bạn có tay máy, có thể di chuyển và thực hiện yêu cầu thông qua các công cụ"
echo "được cung cấp. Luôn chỉ trả lời bằng tiếng Việt, dù người dùng đặt câu hỏi"
echo "bằng bất kỳ ngôn ngữ nào khác. Hãy trả lời ngắn gọn, rõ ràng và chỉ gọi công"
echo "cụ khi cần để hoàn thành yêu cầu của người dùng."
echo
echo "Các tool có tác dụng thật trong kịch bản này:"
if [ -n "$LOCATIONS" ]; then
  echo "  move_to_location: đi tới 5 địa điểm: Điểm xuất phát, Khu trung tâm,"
  echo "    Góc tây nam, Khu vực kệ hàng, Khu tiếp nhận."
else
  echo "move_to_location sẽ không tìm thấy địa điểm nào cho map '$SELECTED_MAP_NAME' (đã cảnh báo ở trên)."
fi
echo "  describe_current_view: mô tả ảnh camera hiện tại."
echo "  get_object_box_distance: ước lượng khoảng cách tới vật thể trong khung hình."
echo "  robot_move_control: điều khiển vận tốc robot trực tiếp trong một khoảng thời gian."
echo
echo "Các tool arm_transport_function, line_following, object_track và"
echo "lidar_scan_detect LLM vẫn biết, nhưng không có tác dụng thật trong world này:"
echo "không có tay gắp, vạch kẻ hoặc cấu hình lidar phù hợp cho các chức năng đó."
