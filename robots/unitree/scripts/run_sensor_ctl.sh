#!/usr/bin/env bash

set -e

SESSION_NAME="robot_nav_ctl"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"
INIT_ENV="${SCRIPT_DIR}/init_env.sh"

kill_existing_session() {
    if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
        tmux kill-session -t "${SESSION_NAME}"
        echo "Session '${SESSION_NAME}' has been deleted."
    fi
}

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

kill_existing_session
tmux new-session -d -s "${SESSION_NAME}" -n nav

# Pane 0: arm control bridge
setup_common_env "${SESSION_NAME}:0"
send_cmd "${SESSION_NAME}:0" "cd ${REPO_ROOT}"
send_cmd "${SESSION_NAME}:0" "bash robots/unitree/scripts/run_armctl.sh"

# Pane 1: sensors
tmux split-window -h -t "${SESSION_NAME}:0"
setup_common_env "${SESSION_NAME}:0.1"
send_cmd "${SESSION_NAME}:0.1" "cd ${REPO_ROOT}"
send_cmd "${SESSION_NAME}:0.1" "bash robots/unitree/scripts/run_sensors.sh"

# Pane 2: velocity fifo writer
tmux split-window -v -t "${SESSION_NAME}:0"
send_cmd "${SESSION_NAME}:0.2" "[ -p /tmp/vel_fifo ] && rm /tmp/vel_fifo"
send_cmd "${SESSION_NAME}:0.2" "mkfifo /tmp/vel_fifo"
setup_common_env "${SESSION_NAME}:0.2"
send_cmd "${SESSION_NAME}:0.2" "ros2 run g1_move g1_getvel_node"

# Pane 3: velocity fifo reader
tmux split-window -v -t "${SESSION_NAME}:0"
setup_common_env "${SESSION_NAME}:0.3"
send_cmd "${SESSION_NAME}:0.3" "ros2 run g1_move g1_pubvel_node"

tmux select-layout -t "${SESSION_NAME}:0" tiled
tmux attach-session -t "${SESSION_NAME}"