#!/bin/bash

set -e

SESSION_NAME="robot_nav"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INIT_ENV="${SCRIPT_DIR}/init_env.sh"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    tmux kill-session -t "${SESSION_NAME}"
    echo "Session '${SESSION_NAME}' has been deleted."
fi

tmux new-session -d -s "${SESSION_NAME}" -n nav

# Pane 0: navigation2 bringup
tmux send-keys -t "${SESSION_NAME}:0" "source ${INIT_ENV}" C-m
tmux send-keys -t "${SESSION_NAME}:0" "unset ASAN_OPTIONS" C-m
tmux send-keys -t "${SESSION_NAME}:0" "ros2 launch nav_bringup g1_navigation2_launch.py" C-m

# Pane 1: nav executor
tmux split-window -h -t "${SESSION_NAME}:0"
tmux send-keys -t "${SESSION_NAME}:0.1" "source ${INIT_ENV}" C-m
tmux send-keys -t "${SESSION_NAME}:0.1" "unset ASAN_OPTIONS" C-m
tmux send-keys -t "${SESSION_NAME}:0.1" "ros2 run nav_executor nav_executor_node" C-m

# Pane 2: ROS <-> HTTP bridge
tmux split-window -v -t "${SESSION_NAME}:0"
tmux send-keys -t "${SESSION_NAME}:0.2" "source ${INIT_ENV}" C-m
tmux send-keys -t "${SESSION_NAME}:0.2" "unset ASAN_OPTIONS" C-m
tmux send-keys -t "${SESSION_NAME}:0.2" "taskset -c 4-7 ros2 run robot_bridge robot_bridge_node" C-m

# Pane 3: multi robot fastapi control
tmux split-window -v -t "${SESSION_NAME}:0.1"
tmux send-keys -t "${SESSION_NAME}:0.3" "source ${INIT_ENV}" C-m
tmux send-keys -t "${SESSION_NAME}:0.3" "unset ASAN_OPTIONS" C-m
tmux send-keys -t "${SESSION_NAME}:0.3" "cd ${REPO_ROOT}/agentic_robot/services/src/multi_robot_ctl" C-m
tmux send-keys -t "${SESSION_NAME}:0.3" "taskset -c 4-7 python3 fastapi_control.py" C-m

tmux select-layout -t "${SESSION_NAME}:0" tiled

if [[ "${RUN_IN_BACKGROUND:-0}" != "1" ]]; then
    tmux attach-session -t "${SESSION_NAME}"
fi
