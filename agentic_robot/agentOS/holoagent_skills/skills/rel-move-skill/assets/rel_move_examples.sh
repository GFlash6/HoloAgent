#!/usr/bin/env bash
set -e

ROBOT_URL="${ROBOT_URL:-http://127.0.0.1:8000}"

echo "Example: move forward 1.0 and turn left"
echo "curl -X POST ${ROBOT_URL}/api/relative_nav -H 'Content-Type: application/json' -d '{"cmd":"1.0,0.0,90"}'"

echo "Example: move backward 1.0 and turn right"
echo "curl -X POST ${ROBOT_URL}/api/relative_nav -H 'Content-Type: application/json' -d '{"cmd":"-1.0,0.0,-90"}'"
