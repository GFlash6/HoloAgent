# Sandbox Tests for Long-Horizon Tasks

This directory contains dry-run and validation scripts for long-horizon natural-language task execution.

## Files

- `long_horizon_text_runner.py`: core planner and executor
- `test_single_robot_long_instruction.py`: single-robot dry-run example
- `test_multi_robot_long_instruction.py`: multi-robot dry-run example

## Features

Supported modes:

- `single_robot`: sequential task execution for one robot
- `multi_robot`: collaborative task execution across multiple robots

Core capabilities:

- accept long-form natural-language instructions
- call Qwen to generate a DAG
- validate nodes, dependencies, cycles, and action legality
- perform virtual execution before real execution
- write YAML monitoring and result files

## Dependencies

```bash
pip install openai requests pyyaml
```

## Environment Variables

### Qwen configuration

```bash
export QWEN_API_KEY=your_key
# Optional overrides:
export QWEN_MODEL=qwen3.7-plus
export QWEN_BASE_URL=https://dashscope.aliyuncs.com/compatible-mode/v1
```

### Robot and control center endpoints

```bash
export ROBOT_11_URL=http://192.168.124.101:8000
export ROBOT_12_URL=http://192.168.124.102:8000
export ROBOT_13_URL=http://192.168.124.103:8000
export ROBOT_14_URL=http://192.168.124.104:8000
export ROBOT_15_URL=http://192.168.124.105:8000
export ROBOT_16_URL=http://192.168.124.106:8000
export MULTI_ROBOT_CONTROL_CENTER_URL=http://127.0.0.1:8080
```

## Usage

Single-robot dry run:

```bash
python3 agentic_robot/agentOS/sandbox_test/test_single_robot_long_instruction.py
```

Multi-robot dry run:

```bash
python3 agentic_robot/agentOS/sandbox_test/test_multi_robot_long_instruction.py
```

Run the core script directly:

```bash
python3 agentic_robot/agentOS/sandbox_test/long_horizon_text_runner.py \
  --mode single_robot \
  --instruction "Robot 11 goes to point 1, waves, then moves to point 2 for a high-five, and finally returns to point 3 to wave goodbye." \
  --dry-run \
  --output-root agentic_robot/agentOS/sandbox_test/output
```

## Output Files

Each run creates a session directory that typically contains:

- `monitor.yaml`
- `dag_plan.yaml`
- `virtual_validation.yaml`
- `execution_result.yaml`

## Notes

- `execution_result.yaml` is usually marked as skipped during dry-run mode.
- Use these scripts to validate planning quality before enabling real robot execution.
