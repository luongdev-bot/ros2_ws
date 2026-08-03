# JetRover ROS 2 Simulation and Autonomy Workspace

This repository contains a ROS 2 Humble workspace for developing and testing a
Hiwonder JetRover with a Mecanum base, a five-degree-of-freedom arm, and a
gripper. It combines Gazebo Sim, MoveIt 2, Nav2, SLAM, camera-based perception,
autonomous grasping, line following, and voice-driven task execution in one
simulation-focused project.

The robot description and meshes are derived from the official
[Hiwonder JetRover repository](https://github.com/Hiwonder/JetRover), primarily
from the `Jetson_Orin_ros2` branch.

## Main capabilities

- JetRover visualization in RViz and physics simulation in Gazebo Fortress.
- Mecanum base control through `/cmd_vel`, with simulated LiDAR, IMU, RGB, and
  depth-camera data.
- MoveIt 2 motion planning and `ros2_control` trajectory execution for the arm
  and gripper.
- A Qt action-group editor and ROS 2 action server for teaching, storing, and
  replaying arm motions.
- Color and depth-based block detection, mobile grasp planning, pick-and-place,
  and automatic color sorting.
- Classical and YOLO segmentation-based line following on a figure-eight
  course.
- 2D SLAM with `slam_toolbox`, optional 3D SLAM with RTAB-Map, and Nav2
  navigation.
- A tool-calling voice-agent architecture with reusable scenario packages.
- Custom Gazebo plugins for stable gripper mimic-joint behavior and grasp
  attachment.

## Architecture

```text
Gazebo sensors ──> perception / SLAM ──> grasp, Nav2, or line following
      │                                            │
      └──────── robot state and TF                 ├──> /cmd_vel ──> base
                                                   └──> arm actions ──> controllers

Voice input ──> voice_llm_agent ──> validated ROS tools ──> robot capabilities
```

The Python packages use domain, application, and infrastructure layers so that
robotics logic can be tested without starting ROS 2 or Gazebo.

## Workspace layout

| Path or package | Purpose |
|---|---|
| `src/simulations/jetrover_description` | URDF/Xacro models, meshes, and RViz display launches |
| `src/simulations/jetrover_gazebo` | Gazebo worlds, bridges, sensors, controllers, and integrated bringup |
| `src/simulations/jetrover_gazebo_plugins` | Custom Gazebo system plugins |
| `src/simulations/jetrover_moveit_config` | MoveIt 2 planning and controller configuration |
| `src/arm_motion` | Qt motion editor and action-group playback server |
| `src/arm_motion_interfaces` | Custom messages, services, and actions for arm motion |
| `src/arm_perception` | Color-based block detection and target selection |
| `src/jetrover_kinematics` | Arm geometry and inverse kinematics |
| `src/jetrover_grasp` | Depth-guided grasping, mobile alignment, and sorting orchestration |
| `src/line_follow` | Classical and YOLO-based line detection and steering |
| `src/slam` | 2D and 3D SLAM launch configuration |
| `src/navigation` | Nav2 bringup for live SLAM and saved maps |
| `src/peripherals` | Joystick and peripheral integration |
| `src/voice_llm_agent` | Voice-agent domain model and ROS tool executor |
| `src/voice_llm_scenarios` | Scenario-specific launch wiring and road-network tools |
| `scripts` | Build, launch, setup, and maintenance helpers |

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Sim Fortress / Ignition Gazebo 6
- MoveIt 2 and `ros2_control`
- Nav2 and `slam_toolbox`
- Python 3.10 with OpenCV, NumPy, PyYAML, and PyQt5
- `rqt_image_view` for camera debugging

Optional components:

- An NVIDIA GPU for faster Gazebo sensor rendering and YOLO inference.
- RTAB-Map packages for 3D SLAM.
- A Python environment at `~/ultralytics_env` containing PyTorch and
  Ultralytics for `yolo_line_follow.launch.py`. The trained segmentation model
  is included in `src/line_follow/models/`.

Install ROS package dependencies after cloning:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

Each new terminal must source both the ROS 2 underlay and this workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## Robot configuration

Set the chassis and LiDAR type before launching the robot:

```bash
export MACHINE_TYPE=JetRover_Mecanum
export LIDAR_TYPE=A1
```

Other supported description values include `JetRover_Tank`,
`JetRover_Acker`, `G4`, `A2`, `S2L`, and `LD14P`. The current Gazebo drive
physics is validated for `JetRover_Mecanum`; the Tank and Ackermann variants
are currently intended for model visualization.

## Quick start

### RViz model display

```bash
ros2 launch jetrover_description sim_display.launch.py
```

### Gazebo simulation

```bash
ros2 launch jetrover_gazebo gazebo.launch.py
```

The main simulated interfaces include:

| Topic | Direction | Purpose |
|---|---|---|
| `/cmd_vel` | ROS 2 to Gazebo | Base velocity command |
| `/odom`, `/tf` | Gazebo to ROS 2 | Odometry and transforms |
| `/scan` | Gazebo to ROS 2 | LiDAR scan |
| `/imu` | Gazebo to ROS 2 | IMU data |
| `/depth_cam/image` | Gazebo to ROS 2 | RGB camera stream |
| `/depth_cam/depth_image` | Gazebo to ROS 2 | Depth image |
| `/depth_cam/camera_info` | Gazebo to ROS 2 | Camera calibration |
| `/joint_states` | Gazebo to ROS 2 | Arm, gripper, and wheel joint state |

### Gazebo with MoveIt 2

```bash
ros2 launch jetrover_gazebo gazebo_moveit.launch.py
```

This starts Gazebo, `controller_manager`, the arm and gripper trajectory
controllers, MoveIt 2, and RViz. Use the Motion Planning panel in RViz to plan
and execute collision-aware arm trajectories.

### Arm motion editor

```bash
./scripts/run_arm_editor.sh
```

Action groups are stored as `.d6a` SQLite files in `~/ActionGroups` by default.
Override the location with `ARM_MOTION_LIBRARY_DIR`.

### Color pick and autonomous sorting

```bash
./scripts/run_color_pick.sh
./scripts/run_color_sort.sh
```

Color sorting uses derived `*_release.d6a` action groups. Generate them from
the taught placement motions with:

```bash
python3 scripts/provision_color_pick_release_groups.py
```

Use `--force` to regenerate the derived groups after teaching new placement
poses.

### Line following

With the simulation and camera stream running, launch either detector:

```bash
ros2 launch line_follow line_follow.launch.py
ros2 launch line_follow yolo_line_follow.launch.py
```

The nodes publish steering commands on `/cmd_vel` and annotated camera frames
on their private `debug_image` topics. Pause or resume the YOLO controller with:

```bash
ros2 service call /yolo_line_follow/enable std_srvs/srv/SetBool \
  "{data: false}"
```

### SLAM and navigation

```bash
./scripts/run_slam.sh
./scripts/run_nav.sh
```

The `slam` package supports `slam_toolbox` for 2D mapping and RTAB-Map for 3D
mapping. RTAB-Map source trees are intentionally not tracked in this repository;
install the ROS packages or provide compatible local source checkouts.

## Testing

Run the ROS package test suites with:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test
colcon test-result --all
```

For focused Python development, run tests through the Python module entrypoint:

```bash
PYTHONPATH=src/line_follow:src/jetrover_grasp:src/jetrover_kinematics \
  python3 -m pytest -q \
  src/line_follow/test \
  src/jetrover_grasp/test \
  src/jetrover_kinematics/test
```

## GPU notes

The Gazebo launch files support NVIDIA PRIME render offload for the GUI and
GPU-backed sensors. Check utilization with:

```bash
nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv
```

MoveIt OctoMap processing and Nav2 costmaps remain CPU workloads; the GPU is
used primarily for Gazebo rendering, simulated sensors, and YOLO inference.

## Stop the simulation

```bash
./scripts/stop_sim.sh
```
