#!/bin/bash
# ZED2i Camera Launch with Virtual Display

# Start virtual display
Xvfb :100 -screen 0 1920x1080x24 &
XVFB_PID=$!
sleep 2

# Set display
export DISPLAY=:100

# Set environment
export ZED_SDK_ROOT=/usr/local/zed
export LD_LIBRARY_PATH=/usr/local/zed/lib:$LD_LIBRARY_PATH

# Source ROS2
source /opt/ros/humble/setup.bash

# Source ZED workspace
source /workspace/openclaw_holoagent/zed_ros2_ws/install/setup.bash

echo "Virtual display started on :100"
echo "Starting ZED2i camera..."

# # Launch ZED2i camera
# ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zed2i

# Launch ZED2i camera with pose estimation
ros2 launch zed_wrapper zed_camera.launch.py \
    camera_model:=zed2i \
    pos_tracking:=true \
    publish_tf:=true \
    depth_mode:=NEURAL

