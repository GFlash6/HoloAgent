#!/bin/bash

set -e

SESSION_NAME="robot_nav_bridge"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INIT_ENV="${SCRIPT_DIR}/init_env.sh"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    tmux kill-session -t "${SESSION_NAME}"
    echo "Session '${SESSION_NAME}' has been deleted."
fi

tmux new-session -d -s "${SESSION_NAME}" -n bridge

# Pane 0: ROS <-> HTTP bridge
tmux send-keys -t "${SESSION_NAME}:0" "source ${INIT_ENV}" C-m
tmux send-keys -t "${SESSION_NAME}:0" "unset ASAN_OPTIONS" C-m
tmux send-keys -t "${SESSION_NAME}:0" "taskset -c 4-7 ros2 run robot_bridge robot_bridge_node" C-m

# Pane 1: multi robot fastapi control
tmux split-window -h -t "${SESSION_NAME}:0"
tmux send-keys -t "${SESSION_NAME}:0.1" "source ${INIT_ENV}" C-m
tmux send-keys -t "${SESSION_NAME}:0.1" "unset ASAN_OPTIONS" C-m
tmux send-keys -t "${SESSION_NAME}:0.1" "cd ${REPO_ROOT}/agentic_robot/services/src/multi_robot_ctl" C-m
tmux send-keys -t "${SESSION_NAME}:0.1" "taskset -c 4-7 python3 fastapi_control.py" C-m

tmux select-layout -t "${SESSION_NAME}:0" even-horizontal
tmux attach-session -t "${SESSION_NAME}"