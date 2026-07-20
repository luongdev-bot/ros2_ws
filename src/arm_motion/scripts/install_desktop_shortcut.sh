#!/usr/bin/env bash
# Put an "Arm Action Editor" launcher on the Desktop and in the app menu.
#
# Usage:  ./install_desktop_shortcut.sh [workspace_dir]

set -euo pipefail

WORKSPACE="${1:-${ARM_MOTION_WS:-$HOME/ros2_ws}}"
# Prefer the workspace launcher (opens Gazebo + editor in their own terminals,
# matching run_joystick.sh); fall back to the packaged wrapper.
LAUNCHER="${WORKSPACE}/scripts/run_arm_editor.sh"
if [[ ! -f "${LAUNCHER}" ]]; then
  LAUNCHER="${WORKSPACE}/src/arm_motion/scripts/arm_motion_editor.sh"
fi

if [[ ! -f "${LAUNCHER}" ]]; then
  echo "Launcher not found: ${LAUNCHER}" >&2
  exit 1
fi
chmod +x "${LAUNCHER}"

# The Desktop directory is localised; ask xdg-user-dir when it is available.
if command -v xdg-user-dir >/dev/null 2>&1; then
  DESKTOP_DIR="$(xdg-user-dir DESKTOP)"
else
  DESKTOP_DIR="${HOME}/Desktop"
fi
mkdir -p "${DESKTOP_DIR}"

APPS_DIR="${HOME}/.local/share/applications"
mkdir -p "${APPS_DIR}"

ENTRY="${APPS_DIR}/arm-motion-editor.desktop"
cat >"${ENTRY}" <<DESKTOP
[Desktop Entry]
Type=Application
Version=1.0
Name=Arm Action Editor
Comment=Teach and replay robot arm action groups on Gazebo
Exec=${LAUNCHER}
Path=${WORKSPACE}
Terminal=false
Categories=Development;Robotics;
DESKTOP
chmod +x "${ENTRY}"

DESKTOP_COPY="${DESKTOP_DIR}/arm-motion-editor.desktop"
cp "${ENTRY}" "${DESKTOP_COPY}"
chmod +x "${DESKTOP_COPY}"

# GNOME requires desktop files dropped on the Desktop to be marked trusted.
if command -v gio >/dev/null 2>&1; then
  gio set "${DESKTOP_COPY}" metadata::trusted true 2>/dev/null || true
fi

echo "Installed:"
echo "  ${DESKTOP_COPY}"
echo "  ${ENTRY}"
echo
echo "If the Desktop icon shows a 'untrusted' prompt, right-click it and"
echo "choose 'Allow Launching'."
