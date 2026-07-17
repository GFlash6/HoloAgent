# Agentic Mode Runbook

This document describes how to run the repository in **agentic mode**, where a language model plans tasks and dispatches registered robot skills.

## Scope

Agentic mode is intended for:

- natural-language task decomposition
- skill selection and execution
- single-robot or multi-robot orchestration
- long-horizon task validation and dry-run testing

The implementation is centered around `agentic_robot/agentOS/`.

## Key Directories

- `agentic_robot/agentOS/holoagent_skills/`: skill registry, skill metadata, examples, and CRUD helpers
- `agentic_robot/agentOS/run_dameon/`: background daemon helpers
- `agentic_robot/agentOS/sandbox_test/`: long-horizon planning and dry-run scripts
- `agentic_robot/services/src/robot_bridge/`: robot-side HTTP-to-ROS bridge
- `agentic_robot/services/src/multi_robot_ctl/`: multi-robot control center examples

## Typical Startup Sequence

1. Prepare the ROS 2 environment and build the required packages.
2. Start robot-side ROS nodes and hardware adapters.
3. Start `robot_bridge` on each robot.
4. Start the control center if multi-robot orchestration is required.
5. Prepare LLM credentials and robot endpoint environment variables.
6. Run the agentic test or integration entry point.

## Environment Preparation

Example environment variables:

```bash
export GPT_PROVIDER=azure
export AZURE_OPENAI_API_KEY=your_key
export AZURE_OPENAI_ENDPOINT=https://your-endpoint.openai.azure.com/
export AZURE_OPENAI_API_VERSION=2024-02-15-preview
export AZURE_OPENAI_DEPLOYMENT=your_deployment_name

export ROBOT_11_URL=http://192.168.124.101:8000
export ROBOT_12_URL=http://192.168.124.102:8000
export ROBOT_13_URL=http://192.168.124.103:8000
export MULTI_ROBOT_CONTROL_CENTER_URL=http://127.0.0.1:8080
```

## Recommended Entry Points

### Long-horizon single-robot dry run

```bash
python3 agentic_robot/agentOS/sandbox_test/test_single_robot_long_instruction.py
```

### Long-horizon multi-robot dry run

```bash
python3 agentic_robot/agentOS/sandbox_test/test_multi_robot_long_instruction.py
```

### Integration demo launcher

```bash
bash scripts/intergation/run_holoagent_pipeline.sh
```

## Related Documentation

- [`README.md`](README.md): repository overview
- [`README_demo.md`](README_demo.md): demo-mode runbook
- [`agentic_robot/agentOS/README.md`](agentic_robot/agentOS/README.md): AgentOS overview
- [`agentic_robot/agentOS/holoagent_skills/README.md`](agentic_robot/agentOS/holoagent_skills/README.md): skill system documentation
