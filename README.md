# JetRover Simulation Workspace

Workspace mô phỏng robot **JetRover** (Hiwonder) — chassis Mecanum + tay máy 5
bậc tự do + kẹp, chạy trên **ROS 2 Humble** + **Gazebo Sim (Ignition Fortress
6.18)** + **MoveIt 2**. Nguồn gốc URDF/mesh: chính chủ
[github.com/Hiwonder/JetRover](https://github.com/Hiwonder/JetRover)
(branch `Jetson_Orin_ros2`).

## Cấu trúc

```
src/simulations/
├── jetrover_description/   # URDF/xacro + mesh + RViz display
├── jetrover_gazebo/         # World, bridge config, launch tích hợp Gazebo
└── jetrover_moveit_config/  # SRDF, controller, planning — sinh bởi MoveIt Setup Assistant
```

## Build

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Biến môi trường bắt buộc

Mọi launch bên dưới đều cần khai báo loại chassis + loại LiDAR trước khi
chạy (đọc bởi `jetrover.xacro`/`jetrover_sim.xacro`):

```bash
export MACHINE_TYPE=JetRover_Mecanum   # hoặc JetRover_Tank / JetRover_Acker
export LIDAR_TYPE=A1                   # hoặc G4 / A2 / S2L / LD14P
```

> ⚠️ Chỉ **JetRover_Mecanum** có bánh xe quay được trong vật lý Gazebo (khớp
> `continuous`). Tank/Acker vẫn hiển thị đúng hình dạng trong RViz nhưng
> bánh xe hiện là khớp `fixed` (chỉ để hiển thị, chưa lái được trong Gazebo).

## 3 kịch bản chạy

### 1. Chỉ xem mô hình 3D trong RViz (nhẹ nhất, không cần vật lý)

```bash
ros2 launch jetrover_description sim_display.launch.py
```

Dùng `robot_state_publisher` + node tự viết `default_joint_state_publisher`
(thay cho `joint_state_publisher_gui`) + `rviz2`. Không cần Gazebo.

### 2. Vật lý Gazebo đầy đủ (bánh xe lăn thật, LiDAR/IMU/camera có dữ liệu)

```bash
ros2 launch jetrover_gazebo gazebo.launch.py
```

Sinh ra: world với 2 khối vật cản, robot spawn bằng plugin
`MecanumDrive` + `OdometryPublisher` + `JointStatePublisher` + cảm biến
`gpu_lidar`/`imu`/`rgbd_camera`. Topic ROS2 chính:

| Topic | Chiều | Ghi chú |
|---|---|---|
| `/cmd_vel` | ROS→Gazebo | `geometry_msgs/Twist` |
| `/odom`, `/tf` | Gazebo→ROS | odom→base_footprint |
| `/scan` | Gazebo→ROS | LiDAR |
| `/imu` | Gazebo→ROS | |
| `/depth_cam/image`, `/depth_cam/depth_image`, `/depth_cam/camera_info` | Gazebo→ROS | camera depth |
| `/joint_states` | Gazebo→ROS | tất cả khớp không-fixed |

Test lái thử:
```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.3}}"
```

### 3. Gazebo + MoveIt (né vật cản khi lập kế hoạch chuyển động tay máy)

```bash
ros2 launch jetrover_gazebo gazebo_moveit.launch.py
```

Gồm tất cả kịch bản 2, cộng thêm:
- `gz_ros2_control` host `controller_manager` bên trong tiến trình Gazebo,
  điều khiển 5 khớp tay (`joint1..joint5`) + kẹp (`r_joint`) qua
  `arm_controller`/`gripper_controller` (`JointTrajectoryController`).
- `move_group` + RViz **MotionPlanning** panel — kéo-thả mục tiêu, bấm
  **Plan & Execute**, tay máy tự lập quỹ đạo tránh 2 khối vật cản trong
  world và tự-va-chạm (self-collision) nhờ Octomap từ camera depth
  (`/depth_cam/depth_image`).
- GPU NVIDIA được ép dùng cho render Gazebo (xem phần GPU bên dưới).

Đổi chassis: `export MACHINE_TYPE=JetRover_Tank` (v.v.) trước khi launch —
lưu ý phần lái vật lý chỉ hoạt động với Mecanum như đã nói ở trên.

## GPU

Máy dùng đồ họa lai (Intel iGPU + NVIDIA rời, chế độ PRIME on-demand). Cả
2 launch Gazebo (`gazebo.launch.py`, `gazebo_moveit.launch.py`) đã tự set
`__NV_PRIME_RENDER_OFFLOAD=1` + `__GLX_VENDOR_LIBRARY_NAME=nvidia` để ép
render (GUI Gazebo, camera depth, LiDAR `gpu_lidar`) chạy trên GPU rời thay
vì Intel iGPU. Kiểm tra đang dùng đúng GPU:

```bash
nvidia-smi --query-gpu=utilization.gpu,memory.used --format=csv
```

**Octomap (MoveIt) và costmap (Nav2) luôn chạy CPU** — đây là kiến trúc
chuẩn của ROS 2/MoveIt, không có bản GPU chính thức cho phần này trong
Humble; chỉ việc *tạo* dữ liệu cảm biến (raycasting LiDAR, render ảnh
camera) mới đẩy được sang GPU.

## Các lỗi đã gặp & cách sửa (tham khảo khi tự chỉnh sửa thêm)

| Vấn đề | Nguyên nhân | Đã sửa ở đâu |
|---|---|---|
| Mesh không hiện trong Gazebo | `package://` bị đổi thành `model://`, thiếu `GZ_SIM_RESOURCE_PATH` | `gazebo.launch.py`, `gazebo_moveit.launch.py` |
| Bánh xe quay nhưng xe không chạy | Khớp bánh xe là `fixed` (chỉ để hiển thị) | `jetrover_description/urdf/car_mecanum.gazebo.urdf.xacro` (bản copy đổi sang `continuous`) |
| Tay máy "xệ" dưới trọng lực | Thiếu `position_proportional_gain` cho `gz_ros2_control` | `jetrover_moveit_config/config/jetrover.ros2_control.xacro` |
| Ngón kẹp "gãy"/đung đưa | Gazebo Fortress (6.18) không hỗ trợ vật lý cho tag `<mimic>` | Thêm `<dynamics damping friction>` vào 5 khớp mimic, `jetrover_description/urdf/gripper.urdf.xacro` |
| `move_group` crash khi khởi động | `max_velocity`/`max_acceleration` là số nguyên thay vì số thực trong YAML | `jetrover_moveit_config/config/joint_limits.yaml` |
| Octomap không nhận dữ liệu camera | Frame TF của camera trong Gazebo (`jetrover/link4/depth_camera`) khác tên link URDF (`depth_cam_frame`) | Static TF bridge trong `gazebo_moveit.launch.py` |
| Tên package dính ký tự xuống dòng | Lỗi khi gõ đường dẫn lưu trong MoveIt Setup Assistant | Đã quét & sửa toàn bộ `jetrover_moveit_config` |

## Nguồn dữ liệu gốc

Toàn bộ workspace JetRover đầy đủ (driver phần cứng thật, SLAM, Nav2,
tích hợp LLM...) nằm trong file zip gốc trên USB:
`ros2_ws-20260717T132335Z-1-002.zip`. Workspace này chỉ giữ lại phần liên
quan tới mô phỏng (`simulations/`) — các phần khác đã bị xóa vì thiếu
`package.xml`/`setup.py` (không build được, do lỗi export zip).
