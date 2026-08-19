# AgentOS Overview

`agentic_robot/agentOS` contains the runtime-facing components used by HoloAgent for task planning, skill registration, execution, and validation. The primary path is natural-language task → Qwen DAG → validation → registered skill execution; audio is an optional input adapter.

## Directory Layout

```text
agentOS/
├── holoagent_agent.py # Main task planner and skill dispatcher
├── holoagent_skills/   # Skill registry, skill docs, examples, and CRUD helpers
├── run_dameon/         # Background daemon launcher and notes
└── sandbox_test/       # Long-horizon planning and dry-run validation scripts
```

## Components

### `holoagent_skills`

Provides the project-maintained skill organization model:

- one directory per skill under `skills/`
- `SKILL.md` as the machine-facing skill specification
- `README.md` as the human-facing usage guide
- `assets/` for examples and prompt snippets
- `scripts/` for executable helpers used by the skill

Utility scripts under `holoagent_skills/scripts/` support listing, creation, inspection, deletion, validation, and legacy document synchronization.

### `run_dameon`

Contains the background daemon helper used to keep robot-side services alive for longer-running workflows.

### `sandbox_test`

Contains long-horizon text instruction runners and dry-run examples for single-robot and multi-robot task planning.

## Recommended Reading Order

1. [`holoagent_skills/README.md`](holoagent_skills/README.md)
2. [`run_dameon/README.md`](run_dameon/README.md)
3. [`sandbox_test/README.md`](sandbox_test/README.md)

## Typical Usage

### Run the main Agent

```bash
python3 agentic_robot/agentOS/holoagent_agent.py --dry-run \
  --task "向前移动0.5米，然后挥手"
```

### Validate the skill registry

```bash
python3 agentic_robot/agentOS/holoagent_skills/scripts/validate_skills.py
```

### List registered skills

```bash
python3 agentic_robot/agentOS/holoagent_skills/scripts/list_skills.py
```

### Run long-horizon dry-run tests

```bash
python3 agentic_robot/agentOS/sandbox_test/test_single_robot_long_instruction.py
python3 agentic_robot/agentOS/sandbox_test/test_multi_robot_long_instruction.py
```

## Notes

- `SKILL.md` is the authoritative skill contract for the agent runtime.
- `README.md` files are intended for operators and developers.
- The directory name `run_dameon` is preserved for compatibility with the current repository layout.
