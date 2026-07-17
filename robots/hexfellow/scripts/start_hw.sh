#!/bin/bash
# start_hw.sh — HexFellow 硬件启动脚本
#
# 启动顺序：
#   1. Livox MID360 雷达
#   2. 底盘 + 升降台（multi_bringup，topic 在 /chassis/ 与 /lift/ 命名空间）
#
# 环境变量覆盖（可选）：
#   LIVOX_WS    - Livox 驱动工作区 install 目录
#   HEX_WS      - hexfellow 硬件工作区 install 目录
#   CHASSIS_URL - 底盘连接地址（默认 10.42.0.32:8439）
#   LIFT_URL    - 升降台连接地址（默认 10.42.0.18:8439）
#
# 用法：
#   ./start_hw.sh                        # 使用默认参数
#   CHASSIS_URL=192.168.1.10:8439 ./start_hw.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# ── Workspace paths ───────────────────────────────────────────────────────────
LIVOX_WS="${LIVOX_WS:-/home/workspace/Livox/livox_ws/install}"
HEX_WS="${HEX_WS:-/home/workspace/hex_ws/install}"
CHASSIS_URL="${CHASSIS_URL:-10.42.0.32:8439}"
LIFT_URL="${LIFT_URL:-10.42.0.18:8439}"

ROS_DISTRO="${ROS_DISTRO:-humble}"
source "/opt/ros/${ROS_DISTRO}/setup.bash"

for ws in "$LIVOX_WS" "$HEX_WS"; do
    if [[ -f "${ws}/setup.bash" ]]; then
        source "${ws}/setup.bash"
    else
        echo "[WARN] Workspace not found: ${ws}/setup.bash"
    fi
done

echo "================================================================"
echo " HexFellow Hardware"
echo "  Livox WS   : $LIVOX_WS"
echo "  Hex WS     : $HEX_WS"
echo "  Chassis URL: $CHASSIS_URL"
echo "  Lift URL   : $LIFT_URL"
echo "  Topics     : /chassis/cmd_vel  /chassis/odom  /lift/joint_cmd  /lift/motor_states"
echo "================================================================"

# ── 选择启动目标（可带参数）─────────────────────────────────────────────────
TARGET="${1:-all}"

case "$TARGET" in
    livox)
        echo "[INFO] Starting Livox MID360..."
        exec ros2 launch livox_ros_driver2 msg_MID360_launch.py
        ;;
    chassis|multi)
        echo "[INFO] Starting chassis + lift (multi_bringup)..."
        exec ros2 launch hex_device_ros_wrapper multi_bringup.launch.py \
            enable_chassis_bridge:=true \
            enable_lift_bridge:=true \
            chassis_url:="${CHASSIS_URL}" \
            lift_url:="${LIFT_URL}"
        ;;
    all|*)
        echo "[INFO] Starting all hardware (run each in separate pane — use start_all.sh for tmux)"
        echo "  Livox : ros2 launch livox_ros_driver2 msg_MID360_launch.py"
        echo "  HW    : ros2 launch hex_device_ros_wrapper multi_bringup.launch.py \\"
        echo "             enable_chassis_bridge:=true enable_lift_bridge:=true \\"
        echo "             chassis_url:=${CHASSIS_URL} lift_url:=${LIFT_URL}"
        echo ""
        echo "Tip: Use start_all.sh for automatic tmux layout."
        ;;
esac
