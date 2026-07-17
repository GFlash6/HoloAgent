#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

source /opt/ros/humble/setup.bash
cd "${REPO_ROOT}/agentic_robot/chatbot/g1"
python g1chat_demo.py