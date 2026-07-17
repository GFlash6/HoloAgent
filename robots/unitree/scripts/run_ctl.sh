#!/bin/bash

set -e

SESSION_NAME="robot_ctl"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INIT_ENV="${SCRIPT_DIR}/init_env.sh"

send_cmd() {
    local target="$1"
    shift
    tmux send-keys -t "${target}" "$*" C-m
}

setup_common_env() {
    local target="$1"
    send_cmd "${target}" "source ${INIT_ENV}"
    send_cmd "${target}" "unset ASAN_OPTIONS"
}

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    tmux kill-session -t "${SESSION_NAME}"
    echo "Session '${SESSION_NAME}' has been deleted."
fi

tmux new-session -d -s "${SESSION_NAME}" -n ctl

# Pane 0: arm 发布命令节点
setup_common_env "${SESSION_NAME}:0"
send_cmd "${SESSION_NAME}:0" "rm -f /tmp/arm_fifo"
send_cmd "${SESSION_NAME}:0" "mkfifo /tmp/arm_fifo"
send_cmd "${SESSION_NAME}:0" "ros2 run g1_arm g1_pubcmd_node"

# Pane 1: arm 接收命令节点
tmux split-window -h -t "${SESSION_NAME}:0"
setup_common_env "${SESSION_NAME}:0.1"
send_cmd "${SESSION_NAME}:0.1" "sleep 3 && ros2 run g1_arm g1_getcmd_node"

# Pane 2: 底盘速度接收节点
tmux split-window -v -t "${SESSION_NAME}:0"
setup_common_env "${SESSION_NAME}:0.2"
send_cmd "${SESSION_NAME}:0.2" "rm -f /tmp/vel_fifo"
send_cmd "${SESSION_NAME}:0.2" "mkfifo /tmp/vel_fifo"
send_cmd "${SESSION_NAME}:0.2" "ros2 run g1_move g1_getvel_node"

# Pane 3: 底盘速度执行节点
tmux split-window -v -t "${SESSION_NAME}:0.1"
setup_common_env "${SESSION_NAME}:0.3"
send_cmd "${SESSION_NAME}:0.3" "ros2 run g1_move g1_pubvel_node"

tmux select-layout -t "${SESSION_NAME}:0" tiled

if [[ "${RUN_IN_BACKGROUND:-0}" != "1" ]]; then
    tmux attach-session -t "${SESSION_NAME}"
fi
