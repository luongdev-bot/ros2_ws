#!/usr/bin/env bash
# Open the SLAM & Nav2 launcher GUI: pick a world, map it, save the map, then
# navigate on that map later without having to remember which world it came from.
#
#   Usage:  run_sim_gui.sh
#
#   Tab 1  choose a world -> opens Gazebo + SLAM + teleop, then saves the map
#          together with a note of which world produced it.
#   Tab 2  choose a saved map -> opens THAT map's world + Nav2 with AMCL.
#
# The bundled worlds pull their furniture from Gazebo Fuel. Run
# scripts/install_gazebo_worlds.sh once first, or the first launch stalls while
# Gazebo downloads meshes.
#
# NOTE: ROS setup files reference unset variables, so they are sourced only
# inside child shells, never at the top level under `set -u`. Same rule as
# run_arm_editor.sh / run_color_pick.sh.
set -u

WS="${JETROVER_WS:-$HOME/ros2_ws}"
export JETROVER_WS="$WS"

# Report failures somewhere the user can see them: launched from a desktop icon
# there is no console, so a bare `exit 1` looks like nothing happened.
die() {
  local msg="$1"
  echo "$msg" >&2
  if command -v zenity >/dev/null 2>&1; then
    zenity --error --title="SLAM & Nav2 launcher" --text="$msg" 2>/dev/null
  elif command -v notify-send >/dev/null 2>&1; then
    notify-send "SLAM & Nav2 launcher" "$msg" 2>/dev/null
  elif command -v xmessage >/dev/null 2>&1; then
    xmessage -center "$msg" 2>/dev/null
  fi
  exit 1
}

if [ ! -f "$WS/install/setup.bash" ]; then
  die "Workspace not built: $WS/install/setup.bash is missing. Run: cd $WS && colcon build"
fi

command -v gnome-terminal >/dev/null || die "gnome-terminal is not installed."

python3 -c 'import tkinter' 2>/dev/null || \
  die "python3-tk is not installed. Run: sudo apt install python3-tk"

WORLDS="$WS/install/jetrover_gazebo/share/jetrover_gazebo/worlds"
[ -d "$WORLDS" ] || \
  die "Worlds are not installed: $WORLDS is missing. Run: cd $WS && colcon build --packages-select jetrover_gazebo"

# The GUI lives in the jetrover_gazebo package; this script is the desktop-icon
# friendly entry point. It needs the workspace sourced for ament_index (to find
# the world catalogue) and for the map_saver_cli call it makes when saving a map.
exec bash -c "source /opt/ros/humble/setup.bash; source '$WS/install/setup.bash'; \
  exec ros2 run jetrover_gazebo sim_launcher_gui"
