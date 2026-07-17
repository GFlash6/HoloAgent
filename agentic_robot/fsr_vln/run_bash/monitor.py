#!/usr/bin/env python3
import psutil
import time
import argparse
from pynvml import (
    nvmlInit,
    nvmlDeviceGetCount,
    nvmlDeviceGetHandleByIndex,
    nvmlDeviceGetUtilizationRates,
    nvmlDeviceGetMemoryInfo,
    nvmlShutdown
)
import signal
import sys

cpu_list = []
total_mem_list = []
max_mem_list = []
gpu_util_list = []
gpu_mem_list = []


def get_gpu_usage():
    """返回每个 GPU 的使用率和显存."""
    gpu_info = []
    try:
        nvmlInit()
        gpu_count = nvmlDeviceGetCount()
        for i in range(gpu_count):
            handle = nvmlDeviceGetHandleByIndex(i)
            util = nvmlDeviceGetUtilizationRates(handle)
            mem = nvmlDeviceGetMemoryInfo(handle)
            gpu_info.append({
                "gpu_index": i,
                "gpu_util": util.gpu,
                "gpu_mem_MB": mem.used / 1024 / 1024
            })
        nvmlShutdown()
    except Exception:
        return []
    return gpu_info


def find_node_pids(node_name):
    """查找包含该 ROS2 节点名的进程（匹配 __node:=name 或可执行名）"""
    pid_list = []
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            cmd = ' '.join(proc.info.get('cmdline') or [])
            name = proc.info.get('name') or ''
            if f"__node:={node_name}" in cmd or node_name in cmd or node_name in name:
                pid_list.append(proc.info['pid'])
        except Exception:
            continue
    return pid_list


def print_statistics(output_file):
    if not cpu_list:
        msg = "没有采样数据。"
        print(msg)
        with open(output_file, "a") as f:
            f.write(msg + "\n")
        return

    stats_msg = (
        "\n=== 节点资源使用统计 ===\n"
        f"CPU(total):   avg={sum(cpu_list)/len(cpu_list):.1f}%, max={max(cpu_list):.1f}%, min={min(cpu_list):.1f}%\n"
        f"MEM(total):   avg={sum(total_mem_list)/len(total_mem_list):.1f} MB, max={max(total_mem_list):.1f} MB, min={min(total_mem_list):.1f} MB\n"
        f"MEM(max pid): avg={sum(max_mem_list)/len(max_mem_list):.1f} MB, max={max(max_mem_list):.1f} MB, min={min(max_mem_list):.1f} MB\n"
        f"GPU(util):    avg={sum(gpu_util_list)/len(gpu_util_list):.1f}%, max={max(gpu_util_list)}, min={min(gpu_util_list)}\n"
        f"GPU_mem(MB):  avg={sum(gpu_mem_list)/len(gpu_mem_list):.1f}, max={max(gpu_mem_list):.1f}, min={min(gpu_mem_list):.1f}\n")

    print(stats_msg)
    with open(output_file, "a") as f:
        f.write(stats_msg)


def monitor(node_name, interval, output_file):
    pid_list = find_node_pids(node_name)
    if not pid_list:
        print(f"未找到节点 {node_name}")
        return

    # 打印表头：total_cpu_percent 为该节点进程总和
    header = "time,total_cpu_percent,total_mem(MB),proc_count,maxPID,maxPID_cpu,maxPID_mem(MB),GPU_index,GPU_util,GPU_mem(MB)"
    print(header)
    with open(output_file, "w") as f:
        f.write(header + "\n")

    # 预热采样（第一次返回基于系统启动到现在的平均，需先调用一次）
    procs = []
    for pid in pid_list:
        try:
            p = psutil.Process(pid)
            p.cpu_percent(None)
            procs.append(p)
        except Exception:
            continue

    def signal_handler(sig, frame):
        print_statistics(output_file)
        sys.exit(0)

    import atexit
    atexit.register(print_statistics, output_file)

    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    while True:
        time.sleep(max(0.0, interval))
        t = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime())
        gpu_info = get_gpu_usage()

        # 更新进程列表（进程可能退出/重启）
        alive = []
        for p in procs:
            if p.is_running():
                alive.append(p)
        # 如果有新进程匹配到，也加入并预热
        current_pids = set(p.pid for p in alive)
        for pid in find_node_pids(node_name):
            if pid not in current_pids:
                try:
                    p = psutil.Process(pid)
                    p.cpu_percent(None)
                    alive.append(p)
                except Exception:
                    pass
        procs = alive

        total_cpu = 0.0
        total_mem_mb = 0.0
        max_cpu = -1.0
        max_pid = None
        max_mem_mb = 0.0

        for p in procs:
            try:
                cpu = p.cpu_percent(None)   # 相对上次调用的增量百分比
                mem_mb = p.memory_info().rss / 1024 / 1024  # 物理内存(MB)
            except Exception:
                continue
            total_cpu += cpu
            total_mem_mb += mem_mb
            if cpu > max_cpu:
                max_cpu = cpu
                max_pid = p.pid
                max_mem_mb = mem_mb

        # GPU 取设备上最高利用率（无法到进程/线程级）
        if gpu_info:
            max_gpu = max(gpu_info, key=lambda x: x['gpu_util'])
        else:
            max_gpu = {'gpu_index': -1, 'gpu_util': 0, 'gpu_mem_MB': 0}

        cpu_list.append(total_cpu)
        total_mem_list.append(total_mem_mb)
        max_mem_list.append(max_mem_mb)
        gpu_util_list.append(max_gpu['gpu_util'])
        gpu_mem_list.append(max_gpu['gpu_mem_MB'])

        line = f"{t},{total_cpu:.1f},{total_mem_mb:.1f},{len(procs)},{max_pid},{max_cpu:.1f},{max_mem_mb:.1f},{max_gpu['gpu_index']},{max_gpu['gpu_util']},{max_gpu['gpu_mem_MB']:.1f}"
        print(line)
        with open(output_file, "a") as f:
            f.write(line + "\n")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Monitor ROS2 node CPU/GPU usage (sum of PIDs)")
    parser.add_argument(
        "--node_name",
        type=str,
        required=True,
        help="ROS2 node name to monitor")
    parser.add_argument(
        "--interval",
        type=float,
        default=0.5,
        help="Sampling interval (s)")
    parser.add_argument(
        "--output",
        type=str,
        required=True,
        help="Output CSV file path")
    args = parser.parse_args()

    monitor(args.node_name, args.interval, args.output)
