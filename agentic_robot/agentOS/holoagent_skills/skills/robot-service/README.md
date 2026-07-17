# Robot Service Skill

`robot-service` provides a generic way to call robot-side HTTP services from the skill layer.

## Purpose

Use this skill when the agent needs to invoke a robot-side API that does not require a dedicated skill wrapper.

## Files

```text
robot-service/
├── SKILL.md
├── README.md
├── assets/
│   └── service_examples.sh
└── scripts/
    └── service_request.py
```

## Usage

```bash
python3 agentic_robot/agentOS/holoagent_skills/skills/robot-service/scripts/service_request.py \
  --method POST \
  --url http://127.0.0.1:8000/api/navigation/one_point_1
```

Example with JSON body:

```bash
python3 agentic_robot/agentOS/holoagent_skills/skills/robot-service/scripts/service_request.py \
  --method POST \
  --url http://127.0.0.1:8000/api/relative_nav \
  --json {cmd:1.0,0.0,0}
```

## Examples

See:

```bash
bash agentic_robot/agentOS/holoagent_skills/skills/robot-service/assets/service_examples.sh
```

## Notes

- This skill is intentionally generic.
- Prefer dedicated skills when a stable, well-defined action already exists.
