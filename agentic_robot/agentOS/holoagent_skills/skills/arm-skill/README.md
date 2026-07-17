# Arm Skill

`arm-skill` provides a simple wrapper for triggering robot arm actions through the configured robot-side interface.

## Purpose

Use this skill when the agent needs to trigger a named arm behavior, such as waving, greeting, or other predefined arm motions.

## Files

```text
arm-skill/
├── SKILL.md
├── README.md
├── assets/
│   └── arm_examples.sh
└── scripts/
    └── trigger_arm_skill.py
```

## Usage

Run the helper script directly:

```bash
python3 agentic_robot/agentOS/holoagent_skills/skills/arm-skill/scripts/trigger_arm_skill.py \
  --robot-url http://127.0.0.1:8000 \
  --skill wave_above_head
```

## Example Requests

See:

```bash
bash agentic_robot/agentOS/holoagent_skills/skills/arm-skill/assets/arm_examples.sh
```

## Expected Inputs

- robot base URL
- arm skill name exposed by the robot-side bridge

## Notes

- The exact arm skill names depend on the robot-side service configuration.
- This skill is typically used together with `robot_bridge`.
