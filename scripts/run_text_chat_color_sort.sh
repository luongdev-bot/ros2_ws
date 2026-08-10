#!/usr/bin/env bash
# Launch the text-chat-controlled colour-sorting demo. The grasp demo provides
# Gazebo, colour perception, and the grasp executor; automatic grasp is disabled.
# Usage:  run_text_chat_color_sort.sh
#
#   Terminal 1: Colour sort (Gazebo + grasp executor, automatic grasp disabled)
#   Terminal 2: Text chat agent (Vietnamese typed commands and Piper speech)
set -u

WS="${ARM_MOTION_WS:-$HOME/ros2_ws}"

ENV_SETUP="source /opt/ros/humble/setup.bash; source '$WS/install/setup.bash'; export MACHINE_TYPE=JetRover_Mecanum; export LIDAR_TYPE=A1"

# Report failures through the desktop when this is started without a terminal.
die() {
  local msg="$1"
  echo "$msg" >&2
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --title="Text chat colour sort" --text="$msg" 2>/dev/null
  elif command -v notify-send >/dev/null 2>&1; then
    notify-send "Text chat colour sort" "$msg" 2>/dev/null
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
  die "Text-chat Python executable is missing or not executable: $VOICE_PYTHON"
fi
if [ ! -f "$PIPER_VOICE" ]; then
  die "Piper Vietnamese voice model is missing: $PIPER_VOICE"
fi

command -v gnome-terminal >/dev/null || die "gnome-terminal is not installed."

if ! curl -s --max-time 2 http://localhost:11434/api/version >/dev/null; then
  echo "Ollama is not responding; starting $OLLAMA_BIN serve..."
  nohup "$OLLAMA_BIN" serve > /tmp/ollama_serve_text_chat_demo.log 2>&1 &
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
    die "Không khởi động được Ollama. Kiểm tra ~/.local/bin/ollama và log /tmp/ollama_serve_text_chat_demo.log"
  fi
fi

open_term() {
  local title="$1"
  shift
  gnome-terminal --title="$title" -- bash -c "$*; echo; echo '--- process exited (press Enter to close) ---'; read _" &
}

open_term "Colour sort (Gazebo + grasp)" "$ENV_SETUP; ros2 launch jetrover_grasp grasp_demo.launch.py auto_grasp:=false"
open_term "Text chat agent (tiếng Việt)" "$ENV_SETUP; echo 'Waiting for grasp_executor...'; until ros2 node list 2>/dev/null | grep -qx /grasp_executor; do sleep 2; done; sleep 1; ros2 launch voice_llm_agent text_chat_agent.launch.py"

echo "Đã mở 2 terminal: Colour sort (Gazebo + grasp) và Text chat agent tiếng Việt."
echo
echo "Hãy gõ lệnh vào cửa sổ chat hiện ra; câu trả lời sẽ hiện trong khung chat và được đọc qua loa."
echo
echo "Một số câu có thể thử:"
echo "  Hãy gắp vật màu đỏ"
echo "  Di chuyển tới trước trong 2 giây"
echo "  Bạn đang thấy gì phía trước?"
