#!/usr/bin/env bash
# nav_then_trigger.sh
#
# 在宿主机执行，通过 docker exec 依次完成：
#   1. 在 holoagent_hexfellow 容器内运行 send_nav_goal.py 1，等待导航完成。
#   2. 导航成功后，在 holobrain 容器内以 ROS_DOMAIN_ID=39 发送
#      ros2 service call /robot/inference_service/enable。
#
# 用法：
#   bash nav_then_trigger.sh

set -euo pipefail

# ── 配置 ──────────────────────────────────────────────────────────────────────
NAV_CONTAINER="holoagent_hexfellow"
NAV_SCRIPT="/workspace/holoagent_ns/agentic_robot_system/robots/hexfellow/scripts/send_nav_goal.py"
NAV_LOOPS=1

SVC_CONTAINER="holobrain"
SVC_DOMAIN_ID=39
SVC_NAME="/robot/inference_service/enable"
SVC_TYPE="std_srvs/srv/Trigger"
SVC_ARGS="{}"
# ─────────────────────────────────────────────────────────────────────────────

log()  { echo -e "\033[1;34m[nav_then_trigger]\033[0m $*"; }
ok()   { echo -e "\033[1;32m[nav_then_trigger]\033[0m $*"; }
warn() { echo -e "\033[1;33m[nav_then_trigger]\033[0m $*"; }
err()  { echo -e "\033[1;31m[nav_then_trigger]\033[0m $*" >&2; }

# ── 检查容器是否在运行 ────────────────────────────────────────────────────────
check_container() {
    local name="$1"
    if ! docker ps --format '{{.Names}}' | grep -q "^${name}$"; then
        err "容器 [${name}] 未在运行，请先启动它。"
        docker ps -a --format 'table {{.Names}}\t{{.Status}}' | grep "${name}" || true
        exit 1
    fi
}

log "检查容器状态..."
check_container "${NAV_CONTAINER}"
check_container "${SVC_CONTAINER}"
ok "两个容器均在运行。"

# ── Step 1：在 holoagent_hexfellow 容器内执行导航，阻塞等待 ──────────────────
log "开始导航（容器: ${NAV_CONTAINER}）..."
log "  命令: python3 ${NAV_SCRIPT} ${NAV_LOOPS}"

NAV_EXIT=0
docker exec "${NAV_CONTAINER}" python3 "${NAV_SCRIPT}" "${NAV_LOOPS}" || NAV_EXIT=$?

if [[ ${NAV_EXIT} -ne 0 ]]; then
    err "导航脚本退出码: ${NAV_EXIT}，导航可能未成功完成。"
    read -r -p "是否仍要继续发送 inference_service enable？[y/N] " ans
    [[ "${ans,,}" == "y" ]] || exit "${NAV_EXIT}"
else
    ok "导航完成。"
fi

# ── Step 2：在 holobrain 容器内发送 service call ──────────────────────────────
INNER_CMD="export ROS_DOMAIN_ID=${SVC_DOMAIN_ID} && ros2 service call ${SVC_NAME} ${SVC_TYPE} '${SVC_ARGS}'"

log "发送 inference_service enable（容器: ${SVC_CONTAINER}，DOMAIN_ID=${SVC_DOMAIN_ID}）..."
log "  命令: ${INNER_CMD}"

# 尝试新开终端窗口；若无图形环境则直接在当前终端执行
open_terminal_or_exec() {
    local exec_cmd="docker exec -it ${SVC_CONTAINER} bash -c \"${INNER_CMD}\""
    local hold="echo; echo '── 完成，按 Enter 关闭 ──'; read"

    if [[ -n "${DISPLAY:-}${WAYLAND_DISPLAY:-}" ]]; then
        for term in gnome-terminal xterm konsole xfce4-terminal; do
            if command -v "${term}" &>/dev/null; then
                case "${term}" in
                    gnome-terminal)
                        gnome-terminal -- bash -c "${exec_cmd}; ${hold}" &
                        ;;
                    xterm)
                        xterm -hold -e bash -c "${exec_cmd}" &
                        ;;
                    konsole)
                        konsole -e bash -c "${exec_cmd}; ${hold}" &
                        ;;
                    xfce4-terminal)
                        xfce4-terminal -e "bash -c '${exec_cmd}; ${hold}'" &
                        ;;
                esac
                ok "已在新终端窗口中启动 service call（${term}）。"
                return 0
            fi
        done
    fi

    warn "未检测到图形终端，直接在当前终端执行..."
    docker exec -it "${SVC_CONTAINER}" bash -c "${INNER_CMD}"
}

open_terminal_or_exec

ok "脚本结束。"
