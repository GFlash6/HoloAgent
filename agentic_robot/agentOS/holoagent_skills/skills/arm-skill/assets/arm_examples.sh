#!/usr/bin/env bash
set -e

ROBOT_URL="${ROBOT_URL:-http://127.0.0.1:8000}"

echo "Example: trigger wave"
echo "curl -X POST ${ROBOT_URL}/api/arm/high_wave"

echo "Example: trigger reset"
echo "curl -X POST ${ROBOT_URL}/api/arm/reset"
    