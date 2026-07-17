#!/bin/bash

set -e

SESSION_NAME="robot_localization"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INIT_ENV="${SCRIPT_DIR}/init_env.sh"

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    tmux kill-session -t "${SESSION_NAME}"
    echo "Session '${SESSION_NAME}' has been deleted."
fi

tmux new-session -d -s "${SESSION_NAME}" -n localization

# Pane 0: FastLIVO 重定位
tmux send-keys -t "${SESSION_NAME}:0" "source ${INIT_ENV}" C-m
tmux send-keys -t "${SESSION_NAME}:0" "unset ASAN_OPTIONS" C-m
tmux send-keys -t "${SESSION_NAME}:0" "taskset -c 4-7 ros2 launch fast_livo online_reloc_g1.launch.py use_rviz:=True" C-m

# Pane 1: FastLIVO 激光里程计
tmux split-window -h -t "${SESSION_NAME}:0"
tmux send-keys -t "${SESSION_NAME}:0.1" "source ${INIT_ENV}" C-m
tmux send-keys -t "${SESSION_NAME}:0.1" "unset ASAN_OPTIONS" C-m
tmux send-keys -t "${SESSION_NAME}:0.1" "taskset -c 4-7 ros2 launch fast_livo online_livo.launch.py" C-m

tmux select-layout -t "${SESSION_NAME}:0" even-horizontal

if [[ "${RUN_IN_BACKGROUND:-0}" != "1" ]]; then
    tmux attach-session -t "${SESSION_NAME}"
fi
