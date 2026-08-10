#!/usr/bin/env bash
# Launch the moving-object tracking demo. The grasp demo provides Gazebo,
# colour perception, and the grasp executor; block_wanderer moves the colour
# blocks; the voice agent handles Vietnamese instructions.
# Usage:  run_object_track_demo.sh
#
#   Terminal 1: Colour sort (Gazebo + grasp executor, automatic grasp disabled)
#   Terminal 2: Block wanderer (moves the colour blocks every four seconds)
#   Terminal 3: Voice agent (Vietnamese microphone input and Piper speech)
set -u

WS="${ARM_MOTION_WS:-$HOME/ros2_ws}"

ENV_SETUP="source /opt/ros/humble/setup.bash; source '$WS/install/setup.bash'; export MACHINE_TYPE=JetRover_Mecanum; export LIDAR_TYPE=A1"

# Report failures through the desktop when this is started without a terminal.
die() {
  local msg="$1"
  echo "$msg" >&2
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --title="Voice colour sort" --text="$msg" 2>/dev/null
  elif command -v notify-send >/dev/null 2>&1; then
    notify-send "Voice colour sort" "$msg" 2>/dev/null
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

open_term() {
  local title="$1"
  shift
  gnome-terminal --title="$title" -- bash -c "$*; echo; echo '--- process exited (press Enter to close) ---'; read _" &
}

open_term "Colour sort (Gazebo + grasp)"             "$ENV_SETUP; ros2 launch jetrover_grasp grasp_demo.launch.py auto_grasp:=false"
open_term "Block wanderer (di chuyển khối màu)"      "$ENV_SETUP; echo 'Waiting for grasp_executor...'; until ros2 node list 2>/dev/null | grep -qx /grasp_executor; do sleep 2; done; sleep 1; ros2 run jetrover_gazebo block_wanderer"
open_term "Voice agent (tiếng Việt)"                 "$ENV_SETUP; echo 'Waiting for grasp_executor...'; until ros2 node list 2>/dev/null | grep -qx /grasp_executor; do sleep 2; done; sleep 1; ros2 launch voice_llm_agent voice_agent.launch.py"

echo "Đã mở 3 terminal theo thứ tự: Colour sort (Gazebo + grasp), Block wanderer và Voice agent tiếng Việt."
echo
echo "Quy trình test object_track gồm 2 bước vì object_track cần một bounding box đầu vào,"
echo "nhưng hiện chưa có tool nào tự động cung cấp bounding box qua giọng nói. Đây là giới hạn"
echo "đã biết, không phải lỗi của script."
echo
echo "1. Trước tiên, có thể hỏi để lấy vị trí gần đúng, ví dụ nói:"
echo "     Vật màu đỏ đang ở đâu trong ảnh?"
echo "   Câu hỏi này dùng tool describe_current_view; LLM chỉ trả lời bằng mô tả, không cung cấp"
echo "   toạ độ pixel chính xác."
echo
echo "2. Cách đáng tin cậy hơn để tự test object_track ngay bây giờ là gọi thẳng topic ROS,"
echo "   không qua voice (các số dưới đây chỉ minh hoạ cú pháp):"
echo "     ros2 topic pub --once /tool_executor/user_utterance std_msgs/msg/String \"{data: 'Hãy bám theo vật trong khung [200,150,320,280]'}\""
echo "   Lệnh này publish thẳng lên topic mà voice_loop cũng dùng để LLM tự trích xuất box từ"
echo "   câu lệnh; nhờ vậy có thể bỏ qua bước ghi âm micro và test nhanh mà không cần nói to."
echo
echo "3. Ghi chú: để cải thiện triệt để, cần thêm một tool riêng xác định bounding box như"
echo "   obj_box_detect trong bản gốc Hiwonder. Tool này hiện chưa có trong bộ 8 tool và có thể"
echo "   được bổ sung ở bước sau nếu cần."
