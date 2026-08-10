#!/usr/bin/env bash
# Choose one Vietnamese voice-agent demo before its Gazebo world is launched.
# The script opens its own interactive terminal when started from a desktop
# shortcut whose Desktop Entry uses Terminal=false.
set -u

SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$SCRIPT_PATH")"

die() {
  local msg="$1"
  echo "$msg" >&2
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --title="Voice Agent" --text="$msg" 2>/dev/null
  elif command -v notify-send >/dev/null 2>&1; then
    notify-send "Voice Agent" "$msg" 2>/dev/null
  fi
  exit 1
}

command -v gnome-terminal >/dev/null || die "gnome-terminal is not installed."

if [ "${1:-}" != "--interactive" ] && [ ! -t 0 ]; then
  printf -v REEXEC_COMMAND '%q --interactive; exec bash' "$SCRIPT_PATH"
  exec gnome-terminal --title="Voice Agent (chọn kịch bản)" -- bash -c "$REEXEC_COMMAND"
fi

if [ "${1:-}" = "--interactive" ]; then
  shift
fi

[ -t 0 ] || die "Không có terminal tương tác để chọn kịch bản."

echo "Chọn một lần kịch bản/map trước khi Gazebo khởi động:"
PS3="Nhập lựa chọn [1-5]: "

select scenario in \
  "Function calling (gắp màu + di chuyển + camera) - color_sort_world" \
  "Navigation transport (di chuyển theo tên địa điểm) - warehouse" \
  "Transport & delivery (gắp + giao hàng) - warehouse" \
  "Road network (đi qua nhiều chặng theo đồ thị) - warehouse" \
  "Thoát"
do
  case "$REPLY" in
    1)
      echo "Khởi động Function calling trong color_sort_world."
      exec "$SCRIPT_DIR/run_voice_color_sort.sh"
      ;;
    2)
      echo "Khởi động Navigation transport trong warehouse."
      exec "$SCRIPT_DIR/run_voice_navigation_transport.sh"
      ;;
    3)
      echo "Khởi động Transport & delivery trong warehouse."
      exec "$SCRIPT_DIR/run_voice_transport_dietitianl.sh"
      ;;
    4)
      echo "Khởi động Road network nhiều chặng trong warehouse."
      exec "$SCRIPT_DIR/run_voice_road_network.sh"
      ;;
    5)
      echo "Đã thoát."
      exit 0
      ;;
    *)
      echo "Lựa chọn không hợp lệ; vui lòng nhập số từ 1 đến 5."
      ;;
  esac
done
