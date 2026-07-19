#!/usr/bin/env bash
# Launch the Gazebo simulation + gamepad (joystick) teleop — no SLAM — each in
# its OWN terminal window. Drive the robot around with a physical controller.
# Usage:  run_joystick.sh
#
#   Terminal 1: Gazebo      Terminal 2: Joystick (joy_node + joystick_control)
#
# If a simulation is ALREADY running, only the joystick terminal is opened.
set -u

WS="$HOME/ros2_ws"

ENV_SETUP="source /opt/ros/humble/setup.bash; source '$WS/install/setup.bash'; export MACHINE_TYPE=JetRover_Mecanum; export LIDAR_TYPE=A1"
WAIT_SIM="echo 'Waiting for the simulation (/scan)...'; until ros2 topic list 2>/dev/null | grep -qx /scan; do sleep 1; done; sleep 2"

command -v gnome-terminal >/dev/null || { echo "gnome-terminal is not installed"; exit 1; }

open_term() {
  local title="$1"; shift
  gnome-terminal --title="$title" -- bash -c "$*; echo; echo '--- process exited ---'; exec bash" &
}

# Check for a running simulation BEFORE any cleanup: attaching the joystick to a
# sim the user already has open is the whole point of this branch, and calling
# stop_sim.sh first would kill that sim and make this unreachable.
already_up=$(bash -c "$ENV_SETUP; ros2 topic list 2>/dev/null | grep -qx /scan && echo yes || echo no")

if [ "$already_up" = "yes" ]; then
  open_term "Joystick" "$ENV_SETUP; ros2 launch peripherals joystick.launch.py"
  echo "Simulation already running - started the Joystick terminal only."
else
  # Only now is it safe to clear orphans: a leftover Gazebo would publish a
  # second /clock and /tf, making TF jump back in time.
  bash "$WS/scripts/stop_sim.sh"
  open_term "Gazebo"   "$ENV_SETUP; ros2 launch jetrover_gazebo gazebo.launch.py"
  open_term "Joystick" "$ENV_SETUP; $WAIT_SIM; ros2 launch peripherals joystick.launch.py"
  echo "Started 2 terminals: Gazebo, Joystick."
fi
