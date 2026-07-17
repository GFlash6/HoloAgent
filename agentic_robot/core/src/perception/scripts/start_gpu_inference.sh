#!/bin/bash
# 启动 YOLO-E GPU 推理节点 (Python 3.8 + CUDA)
# 需要先激活 conda 环境: conda activate holoagent_py38

# 检查 conda 环境
if ! conda info --envs | grep -q "holoagent_py38"; then
    echo "Error: conda environment 'holoagent_py38' not found"
    echo "Please run: conda activate holoagent_py38"
    exit 1
fi

# 检查 Redis 是否运行
if ! pgrep -x "redis-server" > /dev/null; then
    echo "Starting Redis server..."
    redis-server --daemonize yes
    sleep 1
fi

# 项目路径
PROJECT_DIR="/agentic_robot/core/src/perception"

cd "$PROJECT_DIR"

echo "Starting YOLO-E GPU Inference Node..."
echo "Project directory: $PROJECT_DIR"
echo "Config: config/config.yaml"

python3 scripts/yolo_inference_node.py --config config/config.yaml