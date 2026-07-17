#!/usr/bin/env bash

# record_imu.sh
#
# Record IMU topics for intrinsic calibration.
# Keep the robot fully static for 2h+ during recording.
# Use the recorded data to analyze IMU Allan variance.
# Reference: https://github.com/gaowenliang/imu_utils
#
# Storage: MCAP
#
# Recorded topics:
#   /livox/imu               : Livox IMU
#   /dog_imu_raw             : Raw body IMU
#   /secondary_imu           : Secondary IMU
#   /zed/zed_node/imu/data   : ZED IMU

ros2 bag record --storage mcap \
    /livox/imu \
    /dog_imu_raw \
    /secondary_imu \
    /zed/zed_node/imu/data
