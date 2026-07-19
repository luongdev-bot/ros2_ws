# slam

2D và 3D SLAM cho mô phỏng JetRover trong Gazebo (Ignition), port lại từ
package `slam` gốc của Hiwonder nhưng bỏ phần bringup phần cứng
(`peripherals` / `driver`) và nhắm vào các topic/frame mà `jetrover_gazebo`
phát ra.

## Phụ thuộc runtime

```bash
sudo apt update
sudo apt install -y \
  ros-humble-slam-toolbox \
  ros-humble-rtabmap-slam \
  ros-humble-rtabmap-sync \
  ros-humble-nav2-common
```

## Cách chạy

Trước tiên bật mô phỏng (cửa sổ 1):

```bash
source install/setup.bash
export MACHINE_TYPE=JetRover_Mecanum        # để có drive/odometry plugin
ros2 launch jetrover_gazebo gazebo.launch.py
```

### 2D SLAM (slam_toolbox) — cửa sổ 2
```bash
source install/setup.bash
ros2 launch slam slam.launch.py            # use_sim_time:=true, mở sẵn RViz
```
Lái robot (`/cmd_vel`) bằng teleop để dựng bản đồ, rồi lưu:
```bash
ros2 run nav2_map_server map_saver_cli -f ~/map_01
```

### 3D SLAM (RTAB-Map) — thay cho cửa sổ 2
```bash
source install/setup.bash
ros2 launch slam rtabmap_slam.launch.py    # RGBD + laser, use_sim_time:=true
```

## Nguồn topic/frame (jetrover_gazebo/config/gz_bridge.yaml)

| Dùng cho | Topic sim | Frame |
|----------|-----------|-------|
| laser 2D | `/scan` | `lidar_frame` |
| odom | `/odom` (+ TF odom→base_footprint) | `odom` / `base_footprint` |
| RGB | `/depth_cam/image` | `depth_cam_frame` |
| depth | `/depth_cam/depth_image` | `depth_cam_frame` |
| camera_info | `/depth_cam/camera_info` | `depth_cam_frame` |

## Khác biệt so với bản Hiwonder gốc

- Bỏ `launch/include/robot.launch.py` (bringup phần cứng thật) — trong sim
  mọi cảm biến/odometry do Gazebo cung cấp.
- Bỏ các biến môi trường Hiwonder (`need_compile`, `MASTER`, `HOST`,
  namespace theo robot). Mặc định `use_sim_time:=true`.
- RTAB-Map: tắt GPU ORB/FAST (bản apt build CPU-only) và remap sang đúng
  topic depth camera của sim.
