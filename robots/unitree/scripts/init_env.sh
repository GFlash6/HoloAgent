#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

source_if_exists() {
    local setup_path="$1"
    if [[ -f "${setup_path}" ]]; then
        # shellcheck disable=SC1090
        source "${setup_path}"
    else
        echo "[WARN] setup file not found: ${setup_path}"
    fi
}

source_if_exists "/opt/ros/humble/setup.bash"
source_if_exists "${REPO_ROOT}/agentic_robot/thirdparty/install/setup.bash"
source_if_exists "${REPO_ROOT}/agentic_robot/core/install/setup.bash"
source_if_exists "${REPO_ROOT}/agentic_robot/services/install/setup.bash"
source_if_exists "${REPO_ROOT}/robots/unitree/install/setup.bash"
source_if_exists "${REPO_ROOT}/agentic_robot/chatbot/g1/.venv/bin/activate"
