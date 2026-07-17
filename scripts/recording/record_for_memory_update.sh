#!/usr/bin/env bash

# Record the sensor set required for memory / map update workflows.
# Recommended usage:
# 1. Start all localization and sensor nodes first.
# 2. Run this script before collecting the update trajectory.
# 3. Stop recording after the target area has been fully revisited.
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
#   /zed/zed_node/left/color/rect/image/camera_info : Left camera intrinsics
#   /zed/zed_node/imu/data                         : ZED IMU
#   /zed/zed_node/odom                             : ZED odometry
#   /zed/zed_node/pose                             : ZED pose estimate
#   /tf                                            : Dynamic transforms
#   /tf_static                                     : Static transforms
#   /pose                                          : External pose topic used by update pipeline

ros2 bag record --storage mcap \
    /livox/lidar \
    /livox/imu \
    /robot_imu \
    /robot_odom \
    /zed/zed_node/left/color/rect/image \
    /zed/zed_node/right/color/rect/image \
    /zed/zed_node/depth/depth_registered \
    /zed/zed_node/left/color/rect/image/camera_info \
    /zed/zed_node/imu/data \
    /zed/zed_node/odom \
    /zed/zed_node/pose \
    /tf \
    /tf_static \
    /pose \
    # /relo_world_cloud \
    # $(ros2 topic list | grep '^/zed')