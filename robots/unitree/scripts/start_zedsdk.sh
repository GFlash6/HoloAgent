#!/bin/bash

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
INIT_ENV="${SCRIPT_DIR}/init_env.sh"

source "${INIT_ENV}"
unset ASAN_OPTIONS

ros2 launch zed_wrapper zed_camera.launch.py camera_model:=zedx