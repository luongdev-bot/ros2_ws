#!/usr/bin/env bash
# Put a "SLAM & Nav2 Launcher" icon on the Desktop and in the app menu.
#
# Usage:  ./install_sim_gui_shortcut.sh [workspace_dir]
#
# The existing SLAM 2D / Nav2 2D icons run a fixed world. This one opens the
# GUI, where the world is chosen per run and a saved map remembers which world
# it came from - see ../src/simulations/jetrover_gazebo/README.md.

set -euo pipefail

WORKSPACE="${1:-${JETROVER_WS:-$HOME/ros2_ws}}"
LAUNCHER="${WORKSPACE}/scripts/run_sim_gui.sh"

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

# Desktop Entry quoting: a value used as an Exec argument must be wrapped in
# double quotes, with \ " ` $ backslash-escaped, or a path containing a space is
# split into several arguments and the shortcut silently fails to start.
desktop_quote() {
  local s=$1
  s=${s//\\/\\\\}
  s=${s//\"/\\\"}
  s=${s//\`/\\\`}
  s=${s//\$/\\\$}
  printf '"%s"' "$s"
}

QUOTED_LAUNCHER=$(desktop_quote "${LAUNCHER}")
QUOTED_WORKSPACE=$(desktop_quote "${WORKSPACE}")

# No Icon= line: the other launchers in this workspace use the default icon.
#
# Exec runs the wrapper through `env` so a non-default [workspace_dir] actually
# reaches it: run_sim_gui.sh resolves its workspace from JETROVER_WS, and Path=
# only sets the working directory - it would launch $HOME/ros2_ws regardless.
ENTRY="${APPS_DIR}/sim-launcher.desktop"
cat >"${ENTRY}" <<DESKTOP
[Desktop Entry]
Version=1.0
Type=Application
Name=SLAM & Nav2 Launcher
Comment=Choose a world, map it, then navigate that map in the world it came from
Exec=env JETROVER_WS=${QUOTED_WORKSPACE} ${QUOTED_LAUNCHER}
Path=${WORKSPACE}
Terminal=false
Categories=Development;
DESKTOP
chmod +x "${ENTRY}"

DESKTOP_COPY="${DESKTOP_DIR}/sim-launcher.desktop"
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
