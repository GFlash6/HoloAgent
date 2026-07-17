#!/bin/bash
# start_all.sh — HexFellow 全栈 tmux 编排脚本
#
# 用法：
#   ./start_all.sh        # 创建 tmux session，打印 attach 指令后退出
#   ./start_all.sh --stop # 关闭 tmux session（杀掉所有进程）
#
# tmux session 布局（session 名: hexfellow）：
#
#   Window 0 [hw]  — 硬件
#     ├── pane-left  : Livox MID360 雷达（立即启动）
#     └── pane-right : HexFellow 底盘（立即启动）
#
#   Window 1 [loc] — 定位
#     ├── pane-left  : fast_livo online_reloc（等待硬件 8s 后启动）
#     └── pane-right : fast_livo online_livo（reloc 启动后再等 5s）
#
#   Window 2 [nav] — 导航
#     ├── pane-left  : Nav2 stack（轮询 /pose，定位就绪后启动）
#     └── pane-right : nav_executor（Nav2 起来后再启动）
#
# Ctrl+C 行为：
#   - 在外层脚本（未 attach 状态）按 Ctrl+C → 杀掉 tmux session
#   - 已 attach 到 tmux 后，Ctrl+C 作用于当前 pane；按 Ctrl+B D 可 detach
#   - Detach 后 session 依然存活，可重新 attach：tmux attach -t hexfellow
#
# 环境变量覆盖（可选）：
#   SESSION      tmux session 名         （默认 hexfellow）
#   ROS_DISTRO   ROS 发行版              （默认 humble）
#   LIVOX_WS     Livox 驱动 install 目录 （默认 /home/workspace/Livox/livox_ws/install）
#   HEX_WS       hexfellow 硬件 install  （默认 /home/workspace/hex_ws/install）
#   CORE_WS      core install 目录       （默认 自动推断）
#   CHASSIS_URL  底盘 IP:port            （默认 10.42.0.254:8439）
#   LIFT_URL     升降台 IP:port          （默认 10.42.0.39:8439）
#   MAP_FILE     地图 yaml 路径          （默认 sh3_513 grid_map.yaml）
#   MAP_NAME     signals.yaml 地图名     （默认 sh3_513）
#   USE_RVIZ     定位节点是否开 rviz     （默认 true）
#   POSE_TIMEOUT 等待 /pose 最大秒数     （默认 180）

# ─────────────────────────────────────────────────────────────────────────────
# 配置（外层脚本和 pane 子命令均可用）
# ─────────────────────────────────────────────────────────────────────────────
THIS_SCRIPT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/$(basename "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(dirname "$THIS_SCRIPT")"
HEXFELLOW_DIR="$(dirname "$SCRIPT_DIR")"
REPO_ROOT="$(cd "${HEXFELLOW_DIR}/../.." 2>/dev/null && pwd)"

SESSION="${SESSION:-hexfellow}"
ROS_DISTRO="${ROS_DISTRO:-humble}"
LIVOX_WS="${LIVOX_WS:-/home/workspace/Livox/livox_ws/install}"
HEX_WS="${HEX_WS:-/home/workspace/hex_ws/install}"
CORE_WS="${CORE_WS:-${REPO_ROOT}/agentic_robot/core/install}"
CHASSIS_URL="${CHASSIS_URL:-10.42.0.254:8439}"
LIFT_URL="${LIFT_URL:-10.42.0.39:8439}"
MAP_FILE="${MAP_FILE:-/home/workspace/test_map/all_maps/sh3_520/map/grid_map.yaml}"
MAP_NAME="${MAP_NAME:-sh3_520}"
USE_RVIZ="${USE_RVIZ:-true}"
POSE_TIMEOUT="${POSE_TIMEOUT:-180}"

# ─────────────────────────────────────────────────────────────────────────────
# 工具：source 工作区
# ─────────────────────────────────────────────────────────────────────────────
_src() {
    # 用法：_src <ws1> [ws2] ...  — source ROS base + 各工作区
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
    for ws in "$@"; do
        if [[ -f "${ws}/setup.bash" ]]; then
            source "${ws}/setup.bash"
        else
            echo "[WARN] workspace not found: ${ws}/setup.bash"
        fi
    done
}

# ─────────────────────────────────────────────────────────────────────────────
# Pane 子命令（由 tmux pane 内部调用，含顺序等待逻辑）
# ─────────────────────────────────────────────────────────────────────────────

_pane_livox() {
    _src "$LIVOX_WS"
    echo "[HW] Livox MID360 starting..."
    exec ros2 launch livox_ros_driver2 msg_MID360_launch.py
}

_pane_hw_multi() {
    _src "$HEX_WS"
    echo "[HW] Starting chassis + lift (chassis=${CHASSIS_URL}  lift=${LIFT_URL})..."
    exec ros2 launch hex_device_ros_wrapper multi_bringup.launch.py \
        enable_chassis_bridge:=true \
        enable_lift_bridge:=true \
        chassis_url:="${CHASSIS_URL}" \
        lift_url:="${LIFT_URL}"
}

_pane_reloc() {
    _src "$CORE_WS"
    echo "[LOC] Waiting 8s for hardware to initialize..."
    sleep 8
    echo "[LOC] Starting fast_livo online_reloc..."
    exec ros2 launch fast_livo online_reloc_hexfellow.launch.py use_rviz:="${USE_RVIZ}"
}

_pane_livo() {
    _src "$CORE_WS"
    echo "[LOC] Waiting 13s (hw 8s + reloc init 5s)..."
    sleep 13
    echo "[LOC] Starting fast_livo online_livo..."
    exec ros2 launch fast_livo online_livo.launch.py use_rviz:="${USE_RVIZ}"
}

_pane_nav2() {
    _src "$CORE_WS"
    echo "[NAV] Waiting for /pose topic (localization ready check, timeout=${POSE_TIMEOUT}s)..."
    local elapsed=0
    while [[ $elapsed -lt $POSE_TIMEOUT ]]; do
        if ros2 topic echo /pose --once --no-daemon 2>/dev/null | grep -q 'position'; then
            break
        fi
        printf '  [NAV] /pose not ready (%ds / %ds)\r' "$elapsed" "$POSE_TIMEOUT"
        sleep 3
        elapsed=$((elapsed + 3))
    done
    echo ""
    if [[ $elapsed -ge $POSE_TIMEOUT ]]; then
        echo "[WARN] /pose not received in ${POSE_TIMEOUT}s — starting Nav2 anyway."
    else
        echo "[NAV] /pose received — localization ready. Starting Nav2..."
    fi
    exec bash "${SCRIPT_DIR}/start_nav.sh" --map "${MAP_FILE}"
}

_pane_executor() {
    _src "$CORE_WS"
    echo "[NAV] nav_executor waiting for Nav2 (30s grace period)..."
    sleep 30
    echo "[NAV] Starting nav_executor (robot=hexfellow map=${MAP_NAME})..."
    exec ros2 run nav_executor nav_executor_node \
        --ros-args \
        -p robot_name:=hexfellow \
        -p map_name:="${MAP_NAME}"
}

# ─────────────────────────────────────────────────────────────────────────────
# --stop：关闭 tmux session（统一关闭入口）
# ─────────────────────────────────────────────────────────────────────────────
_stop() {
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        tmux kill-session -t "$SESSION"
        echo "[INFO] tmux session '$SESSION' killed."
    else
        echo "[INFO] No tmux session named '$SESSION' found."
    fi
}

# ─────────────────────────────────────────────────────────────────────────────
# 子命令路由（pane 内部调用）
# ─────────────────────────────────────────────────────────────────────────────
case "${1:-}" in
    --pane-livox)     _pane_livox;     exit ;;
    --pane-hw-multi)  _pane_hw_multi;  exit ;;
    --pane-reloc)     _pane_reloc;     exit ;;
    --pane-livo)      _pane_livo;      exit ;;
    --pane-nav2)      _pane_nav2;      exit ;;
    --pane-executor)  _pane_executor;  exit ;;
    --stop|-s)        _stop;           exit ;;
esac

# ─────────────────────────────────────────────────────────────────────────────
# 主流程：创建 tmux session
# ─────────────────────────────────────────────────────────────────────────────
# 清理旧 session
if tmux has-session -t "$SESSION" 2>/dev/null; then
    echo "[INFO] Killing existing session '$SESSION'..."
    tmux kill-session -t "$SESSION"
fi

# Ctrl+C / SIGTERM 在 attach 之前触发：清理 session
trap '_stop; exit 1' INT TERM

echo "================================================================"
echo " HexFellow Full Stack"
echo "  Session    : $SESSION"
echo "  Chassis URL: $CHASSIS_URL"
# echo "  Lift URL   : $LIFT_URL"
echo "  Map file   : $MAP_FILE"
echo "  Map name   : $MAP_NAME"
echo "  Core WS    : $CORE_WS"
echo "================================================================"

# ── Window 0: hw ─────────────────────────────────────────────────────────────
# pane 0 (left) : Livox MID360 雷达
# pane 1 (right): multi_bringup（底盘 + 升降台硬件驱动）
tmux new-session -d -s "$SESSION" -n "hw"

tmux send-keys -t "${SESSION}:hw.0" \
    "bash '${THIS_SCRIPT}' --pane-livox" Enter

tmux split-window -h -t "${SESSION}:hw"
tmux send-keys -t "${SESSION}:hw.1" \
    "bash '${THIS_SCRIPT}' --pane-hw-multi" Enter

tmux select-layout -t "${SESSION}:hw" even-horizontal

# ── Window 1: loc ─────────────────────────────────────────────────────────────
# pane 0 (left) : fast_livo online_reloc（等硬件 8s）
# pane 1 (right): fast_livo online_livo（等 reloc 再 5s）
tmux new-window -t "$SESSION" -n "loc"

tmux send-keys -t "${SESSION}:loc.0" \
    "bash '${THIS_SCRIPT}' --pane-reloc" Enter

tmux split-window -h -t "${SESSION}:loc"
tmux send-keys -t "${SESSION}:loc.1" \
    "bash '${THIS_SCRIPT}' --pane-livo" Enter

tmux select-layout -t "${SESSION}:loc" even-horizontal

# ── Window 2: nav ─────────────────────────────────────────────────────────────
# pane 0 (left) : Nav2 stack（等 /pose 就绪）
# pane 1 (right): nav_executor（Nav2 起来后 30s）
tmux new-window -t "$SESSION" -n "nav"

tmux send-keys -t "${SESSION}:nav.0" \
    "bash '${THIS_SCRIPT}' --pane-nav2" Enter

tmux split-window -h -t "${SESSION}:nav"
tmux send-keys -t "${SESSION}:nav.1" \
    "bash '${THIS_SCRIPT}' --pane-executor" Enter

tmux select-layout -t "${SESSION}:nav" even-horizontal

# ── 默认切换到 loc 窗口（方便观察定位状态）─────────────────────────────────
tmux select-window -t "${SESSION}:loc"

# Session 创建完成，移除 trap 并退出
trap - INT TERM

echo ""
echo "  ✔ Session '$SESSION' is ready."
echo ""
echo "  Windows:"
echo "    0 [hw]   — Livox | multi_bringup (chassis + lift 硬件驱动)"
echo "    1 [loc]  — online_reloc | online_livo       (等待硬件 8-13s)"
echo "    2 [nav]  — Nav2 | nav_executor              (等 /pose 就绪后启动)"
echo ""
echo "  Attach   :  tmux attach -t $SESSION"
echo "  Stop all :  bash '${THIS_SCRIPT}' --stop"
echo "================================================================"
