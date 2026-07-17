#!/usr/bin/env bash

# Record the full sensor set used for SLAM / mapping data collection.
# Recommended usage:
# 1. Start all required sensor drivers first.
# 2. Run this script before moving the robot.
# 3. Stop recording after the traversal is complete.
#
# Storage format: MCAP
#
# Recorded topics:
#   /livox/lidar                                   : Livox point cloud
#   /livox/imu                                     : Livox IMU
#   /robot_imu                                     : Robot body IMU
#   /robot_odom                                    : Robot odometry
#   /zed/zed_node/left/color/rect/image            : ZED left RGB image
#   /zed/zed_node/right/color/rect/image           : ZED right RGB image
#   /zed/zed_node/depth/depth_registered           : ZED registered depth
#   /zed/zed_node/left/color/rect/image/camera_info  : Left camera intrinsics
#   /zed/zed_node/right/color/rect/image/camera_info : Right camera intrinsics
#   /zed/zed_node/imu/data                         : ZED IMU
#   /zed/zed_node/odom                             : ZED odometry
#   /zed/zed_node/pose                             : ZED pose estimate

ros2 bag record --storage mcap \
    /livox/lidar \
    /livox/imu \
    /robot_imu \
    /robot_odom \
    /zed/zed_node/left/color/rect/image \
    /zed/zed_node/right/color/rect/image \
    /zed/zed_node/depth/depth_registered \
    /zed/zed_node/left/color/rect/image/camera_info \
    /zed/zed_node/right/color/rect/image/camera_info \
    /zed/zed_node/imu/data \
    /zed/zed_node/odom \
    /zed/zed_node/pose \
    # $(ros2 topic list | grep '^/zed')