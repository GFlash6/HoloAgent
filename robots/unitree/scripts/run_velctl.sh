#!/bin/bash

# Unitree 本体速度跟踪接口启动脚本：
# - Pane 0 负责接收外部速度指令并写入 /tmp/vel_fifo
# - Pane 1 负责读取 /tmp/vel_fifo 并下发到底盘速度控制节点
# 该脚本仅用于速度控制链路，不包含定位、导航或机械臂相关功能。

set -e

SESSION_NAME="robot_velctl"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INIT_ENV="${SCRIPT_DIR}/init_env.sh"

send_cmd() {
    local target="$1"
    local cmd="$2"
    tmux send-keys -t "${target}" "${cmd}" C-m
}

setup_common_env() {
    local target="$1"
    send_cmd "${target}" "1"
    send_cmd "${target}" "source ${INIT_ENV}"
    send_cmd "${target}" "unset ASAN_OPTIONS"
}

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    tmux kill-session -t "${SESSION_NAME}"
    echo "Session '${SESSION_NAME}' has been deleted."
fi

tmux new-session -d -s "${SESSION_NAME}" -n vel

# Pane 0: 速度跟踪指令写入端
# 接收上层速度控制命令，并通过 FIFO 提供给本体速度执行节点
setup_common_env "${SESSION_NAME}:0"
send_cmd "${SESSION_NAME}:0" "rm -f /tmp/vel_fifo"
send_cmd "${SESSION_NAME}:0" "mkfifo /tmp/vel_fifo"
send_cmd "${SESSION_NAME}:0" "ros2 run g1_move g1_getvel_node"

# Pane 1: 速度跟踪指令执行端
# 从 FIFO 读取速度指令，并发送给 Unitree 本体速度控制接口
tmux split-window -h -t "${SESSION_NAME}:0"
setup_common_env "${SESSION_NAME}:0.1"
send_cmd "${SESSION_NAME}:0.1" "ros2 run g1_move g1_pubvel_node"

tmux select-layout -t "${SESSION_NAME}:0" even-horizontal
tmux attach-session -t "${SESSION_NAME}"