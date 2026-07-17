#!/usr/bin/env bash

# Record the minimum topic set required for LiDAR-camera calibration.
# Recommended usage:
# 1. Start Livox and ZED drivers first.
# 2. Move the robot or calibration target so both sensors observe shared structure.
# 3. Stop recording after enough overlapping views are collected.
#
# Storage format: MCAP
#
# Recorded topics:
#   /livox/lidar                                   : Livox point cloud
#   /livox/imu                                     : Livox IMU
#   /zed/zed_node/left/color/rect/image            : ZED left RGB image
#   /zed/zed_node/left/color/rect/image/camera_info : Left camera intrinsics

ros2 bag record --storage mcap \
    /livox/lidar \
    /livox/imu \
    /zed/zed_node/left/color/rect/image \
    /zed/zed_node/left/color/rect/image/camera_info