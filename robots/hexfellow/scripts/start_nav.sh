#!/bin/bash
# HexFellow Navigation Stack Launcher
#
# Usage:
#   ./start_nav.sh [--map <map.yaml>] [--params <nav_params.yaml>] [--no-rviz] [extra ros2 args...]
#
# Environment overrides (lower priority than CLI flags):
#   MAP_FILE   - Full path to grid map yaml
#   PARAMS_FILE - Full path to Nav2 params yaml
#   NAV_WS     - ROS2 overlay workspace install dir (default: auto-detected)
#
# Examples:
#   ./start_nav.sh
#   ./start_nav.sh --map /data/maps/office.yaml
#   ./start_nav.sh --no-rviz
#   MAP_FILE=/tmp/mymap.yaml ./start_nav.sh

set -e

# ── Locate this script's directory ──────────────────────────────────────────
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HEXFELLOW_DIR="$(dirname "$SCRIPT_DIR")"

# ── Defaults ─────────────────────────────────────────────────────────────────
MAP_FILE="${MAP_FILE:-/home/workspace/test_map/all_maps/sh3_513/map/grid_map.yaml}"
PARAMS_FILE="${PARAMS_FILE:-${HEXFELLOW_DIR}/config/nav_params.yaml}"
RVIZ_CONFIG="${HEXFELLOW_DIR}/config/rviz/hexfellow_nav.rviz"
USE_RVIZ="true"

# ── Parse CLI arguments ───────────────────────────────────────────────────────
EXTRA_ARGS=()
while [[ $# -gt 0 ]]; do
    case "$1" in
        --map)      MAP_FILE="$2";    shift 2 ;;
        --params)   PARAMS_FILE="$2"; shift 2 ;;
        --no-rviz)  USE_RVIZ="false"; shift   ;;
        *)          EXTRA_ARGS+=("$1"); shift  ;;
    esac
done

# ── Validate required files ───────────────────────────────────────────────────
if [[ ! -f "$MAP_FILE" ]]; then
    echo "[ERROR] Map file not found: $MAP_FILE"
    echo "  Set MAP_FILE env var or use: --map <path>"
    exit 1
fi

if [[ ! -f "$PARAMS_FILE" ]]; then
    echo "[ERROR] Params file not found: $PARAMS_FILE"
    exit 1
fi

# ── Source ROS 2 base environment ─────────────────────────────────────────────
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ ! -f "$ROS_SETUP" ]]; then
    echo "[ERROR] ROS setup not found: $ROS_SETUP"
    echo "  Expected: /opt/ros/<distro>/setup.bash"
    exit 1
fi
# shellcheck source=/dev/null
source "$ROS_SETUP"

# ── Source the compiled nav_bringup workspace ─────────────────────────────────
# Auto-detect workspace: look for install/setup.bash relative to repo layout
#   <repo>/agentic_robot/core/install/setup.bash
if [[ -n "$NAV_WS" ]]; then
    WS_SETUP="${NAV_WS}/setup.bash"
else
    REPO_CORE="$(cd "${HEXFELLOW_DIR}/../../agentic_robot/core" 2>/dev/null && pwd)" || true
    WS_SETUP="${REPO_CORE}/install/setup.bash"
fi

if [[ -f "$WS_SETUP" ]]; then
    # shellcheck source=/dev/null
    source "$WS_SETUP"
    echo "[INFO] Sourced workspace: $WS_SETUP"
else
    echo "[WARN] nav_bringup workspace not found: $WS_SETUP"
    echo "  Build first:"
    echo "    cd agentic_robot/core"
    echo "    colcon build --packages-select nav_bringup"
    echo "  Or set: NAV_WS=/path/to/install"
fi

# ── Print launch summary ──────────────────────────────────────────────────────
echo "================================================================"
echo " HexFellow Navigation Stack"
echo "----------------------------------------------------------------"
echo "  Map    : $MAP_FILE"
echo "  Params : $PARAMS_FILE"
echo "  CmdVel : cmd_vel -> /chassis/cmd_vel (topic_tools relay)"
echo "  Odom   : /chassis/odom (nav_params.yaml)"
if [[ "$USE_RVIZ" == "true" ]]; then
    echo "  RViz   : $RVIZ_CONFIG"
else
    echo "  RViz   : disabled"
fi
echo "================================================================"

# ── Launch ────────────────────────────────────────────────────────────────────
exec ros2 launch nav_bringup hexfellow_navigation2.launch.py \
    "map:=${MAP_FILE}" \
    "params_file:=${PARAMS_FILE}" \
    "rviz_config:=${RVIZ_CONFIG}" \
    "use_rviz:=${USE_RVIZ}" \
    "${EXTRA_ARGS[@]}"
