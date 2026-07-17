#!/usr/bin/env bash
set -e

ROBOT_URL="${ROBOT_URL:-http://127.0.0.1:8000}"

echo "Example: navigate to a semantic target"
echo "curl -X POST ${ROBOT_URL}/api/semantic_nav -H 'Content-Type: application/json' -d '{"cmd":"1F,meeting_room,table"}'"

echo "Example: navigate to another semantic target"
echo "curl -X POST ${ROBOT_URL}/api/semantic_nav -H 'Content-Type: application/json' -d '{"cmd":"2F,lab,charging_station"}'"
