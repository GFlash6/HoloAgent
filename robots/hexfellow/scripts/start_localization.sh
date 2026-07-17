#!/bin/bash
# start_localization.sh — HexFellow 定位启动脚本（fast_livo）
#
# 启动顺序：
#   1. online_reloc — 基于已建地图的重定位节点（先启动，提供初始位姿）
#   2. online_livo  — 实时 LiDAR-Inertial-Visual Odometry（跟随 reloc 启动）
#
# 等待就绪的方法：
#   监听 /pose topic，有消息输出即表示定位初始化完成。
#   ros2 topic echo /pose --once
#
# 环境变量覆盖（可选）：
#   CORE_WS    - agentic_robot core install 目录
#   MAP_PATH   - 建图数据路径（fast_livo config 中指定，通常不需要覆盖）
#   USE_RVIZ   - true/false（默认 false，节省资源）
#
# 用法：
#   ./start_localization.sh            # 启动全部（打印命令，配合 start_all.sh 使用）
#   ./start_localization.sh reloc      # 仅启动 reloc
#   ./start_localization.sh livo       # 仅启动 livo

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
HEXFELLOW_DIR="$(dirname "$SCRIPT_DIR")"

# ── Workspace paths ───────────────────────────────────────────────────────────
CORE_WS="${CORE_WS:-$(cd "${HEXFELLOW_DIR}/../../agentic_robot/core" 2>/dev/null && pwd)/install}"
USE_RVIZ="${USE_RVIZ:-false}"

ROS_DISTRO="${ROS_DISTRO:-humble}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

if [[ -f "${CORE_WS}/setup.bash" ]]; then
    source "${CORE_WS}/setup.bash"
    echo "[INFO] Sourced core workspace: ${CORE_WS}"
else
    echo "[WARN] Core workspace not found: ${CORE_WS}/setup.bash"
    echo "       Build first: cd agentic_robot/core && colcon build --packages-select fast_livo"
fi

echo "================================================================"
echo " HexFellow Localization (fast_livo)"
echo "  Core WS  : $CORE_WS"
echo "  use_rviz : $USE_RVIZ"
echo "  Pose topic: /pose"
echo "================================================================"

TARGET="${1:-all}"

case "$TARGET" in
    reloc)
        echo "[INFO] Starting online_reloc..."
        exec ros2 launch fast_livo online_reloc_hexfellow.launch.py use_rviz:="${USE_RVIZ}"
        ;;
    livo)
        echo "[INFO] Starting online_livo..."
        exec ros2 launch fast_livo online_livo.launch.py use_rviz:="${USE_RVIZ}"
        ;;
    all|*)
        echo "[INFO] Localization commands (run each in separate pane — use start_all.sh for tmux)"
        echo "  Reloc: ros2 launch fast_livo online_reloc_hexfellow.launch.py use_rviz:=${USE_RVIZ}"
        echo "  Livo : ros2 launch fast_livo online_livo.launch.py  use_rviz:=${USE_RVIZ}"
        echo ""
        echo "Wait for localization ready:"
        echo "  ros2 topic echo /pose --once"
        echo ""
        echo "Tip: Use start_all.sh for automatic tmux layout with /pose readiness check."
        ;;
esac
