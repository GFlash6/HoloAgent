#!/usr/bin/env bash
set -euo pipefail

# 采样间隔（秒）
INTERVAL="${1:-1}"

# 目标脚本与匹配关键字
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGET_RUN_SCRIPT="${SCRIPT_DIR}/run_replica.sh"
TARGET_PATTERN="run_ovo_mapping_replica.py"

if [[ ! -f "${TARGET_RUN_SCRIPT}" ]]; then
  echo "找不到脚本: ${TARGET_RUN_SCRIPT}"
  exit 1
fi

# 启动任务
bash "${TARGET_RUN_SCRIPT}" &
RUNNER_PID=$!

echo "Started ${TARGET_RUN_SCRIPT} (pid=${RUNNER_PID}), sampling every ${INTERVAL}s..."

SAMPLES=0
TOTAL_VRAM_MB=0
MAX_VRAM_MB=0
MAX_RAM_MB=0

# 按 pid 列表求和 RSS(kB)
sum_rss_kb_by_pids() {
  local pids="$1"
  local sum_kb=0
  local p
  for p in ${pids}; do
    if [[ -r "/proc/${p}/status" ]]; then
      # VmRSS:   12345 kB
      local rss_kb
      rss_kb="$(awk '/VmRSS:/ {print $2}' "/proc/${p}/status" 2>/dev/null || echo 0)"
      rss_kb="${rss_kb:-0}"
      sum_kb=$((sum_kb + rss_kb))
    fi
  done
  echo "${sum_kb}"
}
# 将当前命名空间中的 pid 映射到宿主机 pid（取 NSpid 最后一个）
to_host_pid() {
  local pid="$1"
  if [[ -r "/proc/${pid}/status" ]]; then
    awk '
      /^NSpid:/ {
        # NSpid: <host_pid> <container_pid> ...
        if (NF >= 2) print $2;
        else print "'"${pid}"'";
        found=1
      }
      END {
        if (!found) print "'"${pid}"'"
      }
    ' "/proc/${pid}/status" 2>/dev/null
  else
    echo "${pid}"
  fi
}
pid_candidates() {
  local pid="$1"
  local out="${pid}"
  if [[ -r "/proc/${pid}/status" ]]; then
    local ns
    ns="$(awk '
      /^(NSpid|NStgid):/ {
        for (i=2; i<=NF; i++) print $i
      }
    ' "/proc/${pid}/status" 2>/dev/null || true)"
    if [[ -n "${ns}" ]]; then
      out="${out} ${ns}"
    fi
  fi
  # 去重后输出为一行
  echo "${out}" | tr ' ' '\n' | awk 'NF' | sort -u | xargs
}
sum_vram_mb_by_pids() {
  # 在 Docker 中无法按 PID 精确过滤，如果独占 GPU，直接读取 GPU 整体占用
  nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits 2>/dev/null | awk '{sum+=$1} END {print sum+0}'
}
# ...existing code...

while true; do
  # 匹配所有 run_ovo_mapping_replica.py 相关进程
  PIDS="$(pgrep -f "${TARGET_PATTERN}" || true)"

  # 任务结束条件：启动器结束且目标进程不存在
  if ! kill -0 "${RUNNER_PID}" 2>/dev/null && [[ -z "${PIDS}" ]]; then
    break
  fi

  if [[ -n "${PIDS}" ]]; then
    RAM_KB="$(sum_rss_kb_by_pids "${PIDS}")"
    RAM_MB=$((RAM_KB / 1024))

    VRAM_MB="$(sum_vram_mb_by_pids "${PIDS}")"
    printf "PIDs: %s | RAM: %d MB | vRAM: %d MB\n" "${PIDS}" "${RAM_MB}" "${VRAM_MB}"

    TOTAL_VRAM_MB=$((TOTAL_VRAM_MB + VRAM_MB))
    SAMPLES=$((SAMPLES + 1))

    (( VRAM_MB > MAX_VRAM_MB )) && MAX_VRAM_MB="${VRAM_MB}"
    (( RAM_MB  > MAX_RAM_MB  )) && MAX_RAM_MB="${RAM_MB}"
  fi

  sleep "${INTERVAL}"
done

# 等待 run_replica.sh 正常退出，拿到退出码
wait "${RUNNER_PID}" || true

if (( SAMPLES > 0 )); then
  AVG_VRAM_MB=$((TOTAL_VRAM_MB / SAMPLES))
else
  AVG_VRAM_MB=0
fi

echo "================ Memory Summary ================"
echo "Target Pattern : ${TARGET_PATTERN}"
echo "Samples        : ${SAMPLES}"
echo "Avg vRAM (MB)  : ${AVG_VRAM_MB}"
echo "Max vRAM (MB)  : ${MAX_VRAM_MB}"
echo "Max RAM  (MB)  : ${MAX_RAM_MB}"
echo "================================================"
#save summary to file
SUMMARY_FILE="${SCRIPT_DIR}/memory_summary.txt"
{
  echo "Memory Usage Summary for pattern: ${TARGET_PATTERN}"
  echo "Samples: ${SAMPLES}"
  echo "Average vRAM (MB): ${AVG_VRAM_MB}"
  echo "Max vRAM (MB): ${MAX_VRAM_MB}"
  echo "Max RAM (MB): ${MAX_RAM_MB}"
} > "${SUMMARY_FILE}"
echo "Summary saved to ${SUMMARY_FILE}"