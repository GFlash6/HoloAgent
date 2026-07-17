#!/bin/bash

set -e

SESSION_NAME="robot_sensors"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INIT_ENV="${SCRIPT_DIR}/init_env.sh"

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    tmux kill-session -t "${SESSION_NAME}"
    echo "Session '${SESSION_NAME}' has been deleted."
fi

tmux new-session -d -s "${SESSION_NAME}" -n zed_tab

tmux send-keys -t "${SESSION_NAME}:zed_tab" "source ${INIT_ENV}" C-m
tmux send-keys -t "${SESSION_NAME}:zed_tab" "unset ASAN_OPTIONS" C-m
tmux send-keys -t "${SESSION_NAME}:zed_tab" "bash ${SCRIPT_DIR}/start_zedsdk.sh" C-m

tmux new-window -t "${SESSION_NAME}" -n robot_odom_tab
tmux send-keys -t "${SESSION_NAME}:robot_odom_tab" "source ${INIT_ENV}" C-m
tmux send-keys -t "${SESSION_NAME}:robot_odom_tab" "unset ASAN_OPTIONS" C-m
tmux send-keys -t "${SESSION_NAME}:robot_odom_tab" "taskset -c 4-7 ros2 run robot_odom imu_extractor" C-m

tmux new-window -t "${SESSION_NAME}" -n lidar_tab
tmux send-keys -t "${SESSION_NAME}:lidar_tab" "source ${INIT_ENV}" C-m
tmux send-keys -t "${SESSION_NAME}:lidar_tab" "unset ASAN_OPTIONS" C-m
tmux send-keys -t "${SESSION_NAME}:lidar_tab" "taskset -c 4-7 ros2 launch livox_ros_driver2 msg_MID360_launch.py" C-m

if [[ "${RUN_IN_BACKGROUND:-0}" != "1" ]]; then
    tmux attach-session -t "${SESSION_NAME}"
fi
