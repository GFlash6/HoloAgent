#!/usr/bin/env bash

set -e

SESSION_NAME="holoagent_workflow"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
CONTAINER_NAME="holoagent_running"

send_cmd() {
    local target="$1"
    shift
    tmux send-keys -t "${target}" "$*" C-m
}

docker_exec_cmd() {
    local inner_cmd="$1"
    printf 'docker exec -it %s /bin/bash -lc %q' "${CONTAINER_NAME}" "cd ${REPO_ROOT} && ${inner_cmd}"
}

if tmux has-session -t "${SESSION_NAME}" 2>/dev/null; then
    tmux kill-session -t "${SESSION_NAME}"
    echo "Session '${SESSION_NAME}' has been deleted."
fi

docker start "${CONTAINER_NAME}" >/dev/null 2>&1 || true
xhost + >/dev/null 2>&1 || true

tmux new-session -d -s "${SESSION_NAME}" -n workflow

# 2x2 布局
tmux split-window -h -t "${SESSION_NAME}:0"
tmux split-window -v -t "${SESSION_NAME}:0"
tmux split-window -v -t "${SESSION_NAME}:0.1"

# 左上：定位
send_cmd "${SESSION_NAME}:0.0" "1"
send_cmd "${SESSION_NAME}:0.0" "$(docker_exec_cmd 'bash robots/unitree/scripts/run_localization.sh')"

# 右上：导航
send_cmd "${SESSION_NAME}:0.1" "1"
send_cmd "${SESSION_NAME}:0.1" "$(docker_exec_cmd 'bash robots/unitree/scripts/run_nav.sh')"

# 左下：传感器与运控
send_cmd "${SESSION_NAME}:0.2" "1"
send_cmd "${SESSION_NAME}:0.2" "$(docker_exec_cmd 'bash robots/unitree/scripts/run_sensor_ctl.sh')"

# 右下：relative_nav 节点
send_cmd "${SESSION_NAME}:0.3" "1"
send_cmd "${SESSION_NAME}:0.3" "$(docker_exec_cmd 'source robots/unitree/scripts/init_env.sh && unset ASAN_OPTIONS && taskset -c 4-7 ros2 run relative_goal relative_goal_node')"

tmux select-layout -t "${SESSION_NAME}:0" tiled
tmux attach-session -t "${SESSION_NAME}"