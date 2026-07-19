#!/usr/bin/env bash
# Move the arm so the depth camera looks forward — the sim equivalent of
# Hiwonder's `action_name: horizontal` that runs before 3D SLAM.
#
# The depth camera is mounted on link4 of the ARM (link4 -> camera_connect_link
# -> depth_cam_link -> depth_cam_frame). With all joints at 0 the camera points
# straight UP (viewing direction 0,0,1), which makes RTAB-Map useless.
#
# Pose below was picked by forward kinematics on the URDF:
#   joint2=0.80 joint3=0.00 joint4=0.80
#     -> camera at (0.277, 0.000, 0.456) in base_footprint
#     -> viewing direction (1.00, 0.00, -0.03): forward, 1.7 deg down
#     -> lowest arm link z = 0.227 m, safely above the lidar plane (0.157 m)
#
# Requires the arm controller, i.e. the sim must be started with
# gazebo_moveit.launch.py (gz_ros2_control + arm_controller), not gazebo.launch.py.
set -u

echo "Waiting for arm_controller to become active ..."
for i in $(seq 1 90); do
  ros2 control list_controllers 2>/dev/null | grep -q "arm_controller.*active" && break
  sleep 1
done

if ! ros2 control list_controllers 2>/dev/null | grep -q "arm_controller.*active"; then
  echo "ERROR: arm_controller never became active."
  echo "       Start the sim with gazebo_arm.launch.py (gz_ros2_control + spawners)."
  exit 1
fi

# Use the ACTION interface, not the plain topic: `ros2 topic pub` publishes as
# soon as it starts and the message is lost if the controller's subscription
# hasn't been discovered yet (that race silently left the arm at 0). The action
# client waits for the server before sending, so the goal always lands.
echo "Moving the arm to the camera-forward pose ..."
# Bounded: if the action server hangs or never returns a result the script must
# not block the caller forever (SLAM waits on this). Report the real outcome
# instead of unconditionally claiming success.
out=$(timeout 30 ros2 action send_goal /arm_controller/follow_joint_trajectory \
  control_msgs/action/FollowJointTrajectory \
"{trajectory: {joint_names: ['joint1','joint2','joint3','joint4','joint5'],
   points: [{positions: [0.0, 0.80, 0.0, 0.80, 0.0], time_from_start: {sec: 3, nanosec: 0}}]}}" 2>&1)
rc=$?

if [ $rc -eq 124 ]; then
  echo "WARNING: timed out waiting for the arm trajectory to finish."
  echo "         The depth camera may still be pointing up; 3D SLAM will see nothing."
  exit 1
elif echo "$out" | grep -q "SUCCEEDED"; then
  echo "Arm posed: depth camera now faces forward."
else
  echo "WARNING: arm trajectory did not succeed:"
  echo "$out" | tail -3
  exit 1
fi
