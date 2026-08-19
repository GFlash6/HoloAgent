#!/usr/bin/env bash
set -e

SESSION_NAME="robot_agent"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INIT_ENV="${SCRIPT_DIR}/init_env.sh"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

tmux kill-session -t "${SESSION_NAME}" 2>/dev/null || true
tmux new-session -d -s "${SESSION_NAME}" -n agent
tmux send-keys -t "${SESSION_NAME}:0" "source ${INIT_ENV}" C-m
tmux send-keys -t "${SESSION_NAME}:0" "cd ${REPO_ROOT}" C-m
tmux send-keys -t "${SESSION_NAME}:0" "bash scripts/agent/start_agent.sh" C-m

if [[ "${RUN_IN_BACKGROUND:-0}" != "1" ]]; then
    tmux attach-session -t "${SESSION_NAME}"
fi
