#!/usr/bin/env bash
set -e

ROBOT_URL="${ROBOT_URL:-http://127.0.0.1:8000}"
MASTER_URL="${MASTER_URL:-http://127.0.0.1:8080}"

echo "[robot_bridge] health"
echo "curl ${ROBOT_URL}/health"

echo "[robot_bridge] named navigation"
echo "curl -X POST ${ROBOT_URL}/api/navigation/one_point_1"

echo "[robot_bridge] relative navigation"
echo "curl -X POST ${ROBOT_URL}/api/relative_nav -H 'Content-Type: application/json' -d '{"cmd":"forward,left,30"}'"

echo "[robot_bridge] semantic navigation"
echo "curl -X POST ${ROBOT_URL}/api/semantic_nav -H 'Content-Type: application/json' -d '{"cmd":"1F,meeting_room,table"}'"

echo "[robot_bridge] arm skill"
echo "curl -X POST ${ROBOT_URL}/api/arm/wave"

echo "[multi_robot_ctl] fan-out navigation"
echo "curl -X POST ${MASTER_URL}/trigger/one_point_1"

echo "[multi_robot_ctl] targeted stop"
echo "curl -X POST '${MASTER_URL}/trigger/stop?robot_id=11'"
