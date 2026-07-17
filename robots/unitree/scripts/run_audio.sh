#!/bin/bash

set -e

SESSION_NAME="robot_audio"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INIT_ENV="${SCRIPT_DIR}/init_env.sh"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    tmux kill-session -t "${SESSION_NAME}"
    echo "Session '${SESSION_NAME}' has been deleted."
fi

tmux new-session -d -s "${SESSION_NAME}" -n audio

tmux send-keys -t "${SESSION_NAME}:0" "source ${INIT_ENV}" C-m
tmux send-keys -t "${SESSION_NAME}:0" "unset ASAN_OPTIONS" C-m
tmux send-keys -t "${SESSION_NAME}:0" "cd ${REPO_ROOT}" C-m
tmux send-keys -t "${SESSION_NAME}:0" "bash scripts/audio/start_audio_ctl.sh" C-m

if [[ "${RUN_IN_BACKGROUND:-0}" != "1" ]]; then
    tmux attach-session -t "${SESSION_NAME}"
fi
