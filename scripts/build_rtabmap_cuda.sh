#!/usr/bin/env bash
# Build rtabmap (core) + rtabmap_sync/rtabmap_slam from source against the
# CUDA-enabled OpenCV 4.10 in /usr/local, so RTAB-Map can use GPU ORB/FAST
# (then launch with  use_gpu:=true ).
#
# Only 6 packages are built: rtabmap, rtabmap_msgs, rtabmap_conversions,
# rtabmap_util, rtabmap_sync, rtabmap_slam. realsense/velodyne packages are
# skipped, so their apt deps are not needed.
#
# Qt IS required: rtabmap_util does find_package(RTABMap COMPONENTS gui REQUIRED),
# so the core must be built with Qt (do NOT pass -DWITH_QT=OFF). Needs:
#   sudo apt install -y qtbase5-private-dev libqt5svg5-dev
# Only the standalone app/tools executables are skipped (BUILD_APP/BUILD_TOOLS).
#
# Memory note: rtabmap_sync instantiates a LOT of message-sync templates and is
# the usual OOM culprit (that is what killed the previous attempt). Keep the job
# count low. If it still gets killed, drop to MAKEFLAGS="-j1".
set -e

source /opt/ros/humble/setup.bash
cd "$HOME/ros2_ws"

export MAKEFLAGS="-j2"

colcon build \
  --packages-up-to rtabmap_slam rtabmap_sync \
  --parallel-workers 1 \
  --event-handlers console_direct+ \
  --cmake-args \
    -DCMAKE_BUILD_TYPE=RelWithDebInfo \
    -DOpenCV_DIR=/usr/local/lib/cmake/opencv4 \
    -DBUILD_APP=OFF \
    -DBUILD_TOOLS=OFF

echo
echo "=== Done. Verify it linked against the CUDA OpenCV 4.10: ==="
ldd install/rtabmap/lib/librtabmap_core.so* 2>/dev/null | grep -i opencv | head -3
