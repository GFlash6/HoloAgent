#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

if ! pgrep redis-server > /dev/null; then
    echo "Starting Redis..."
    redis-server --daemonize yes
else
    echo "Redis already running"
fi

source /root/miniconda3/etc/profile.d/conda.sh
conda activate holoagent_py38

echo "Using python: $(which python)"

bash "${REPO_ROOT}/agentic_robot/core/src/perception/scripts/start_gpu_inference.sh"