#!/usr/bin/env bash
# Put a "Road Network (Voice, Vietnamese)" launcher on the Desktop and in the app menu.
#
# Usage:  ./install_voice_road_network_shortcut.sh [workspace_dir]

set -euo pipefail

WORKSPACE="${1:-${ARM_MOTION_WS:-$HOME/ros2_ws}}"
LAUNCHER="${WORKSPACE}/scripts/run_voice_road_network.sh"

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

# No Icon= line: the other launchers in this workspace use the default icon.
ENTRY="${APPS_DIR}/voice-road-network.desktop"
cat >"${ENTRY}" <<DESKTOP
[Desktop Entry]
Type=Application
Version=1.0
Name=Road Network (Voice, Vietnamese)
Comment=Navigate the mapped warehouse along a named road network using Vietnamese voice commands
Exec=${LAUNCHER}
Path=${WORKSPACE}
Terminal=false
Categories=Development;Robotics;
DESKTOP
chmod +x "${ENTRY}"

DESKTOP_COPY="${DESKTOP_DIR}/voice-road-network.desktop"
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
echo "If the Desktop icon shows an 'untrusted' prompt, right-click it and"
echo "choose 'Allow Launching'."
