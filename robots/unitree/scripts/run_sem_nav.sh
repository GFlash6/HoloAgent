#!/bin/bash

set -e

SESSION_NAME="robot_sem_nav"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INIT_ENV="${SCRIPT_DIR}/init_env.sh"
CONDA_SH="/root/miniconda3/etc/profile.d/conda.sh"
CONDA_ENV_NAME="holoagent_semantic_mapping"

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    tmux kill-session -t "${SESSION_NAME}"
    echo "Session '${SESSION_NAME}' has been deleted."
fi

tmux new-session -d -s "${SESSION_NAME}" -n sem_nav

# Pane 0: 语义目标节点
tmux send-keys -t "${SESSION_NAME}:0" "source ${INIT_ENV}" C-m
tmux send-keys -t "${SESSION_NAME}:0" "source ${CONDA_SH}" C-m
tmux send-keys -t "${SESSION_NAME}:0" "conda activate ${CONDA_ENV_NAME}" C-m
tmux send-keys -t "${SESSION_NAME}:0" "unset ASAN_OPTIONS" C-m
tmux send-keys -t "${SESSION_NAME}:0" "taskset -c 4-7 ros2 run semantic_goal semantic_goal_node" C-m

if [[ "${RUN_IN_BACKGROUND:-0}" != "1" ]]; then
    tmux attach-session -t "${SESSION_NAME}"
fi
