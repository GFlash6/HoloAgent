# Relative Move Skill

`rel-move-skill` sends relative navigation commands to a robot-side endpoint.

## Purpose

Use this skill when the agent needs to move the robot relative to its current pose instead of navigating to a named waypoint or semantic target.

## Files

```text
rel-move-skill/
├── SKILL.md
├── README.md
├── assets/
└── scripts/
    └── relative_move.py
```

## Usage

```bash
python3 agentic_robot/agentOS/holoagent_skills/skills/rel-move-skill/scripts/relative_move.py \
  --robot-url http://127.0.0.1:8000 \
  --cmd "1.0,0.0,0"
```

The command format is:

```text
forward,left,degrees
```

Example:

- `1.0,0.0,0`: move forward 1 meter
- `0.0,0.5,0`: move 0.5 meter to the left
- `0.0,0.0,90`: rotate 90 degrees counterclockwise

## Notes

- The robot-side bridge must expose the relative navigation endpoint.
- The downstream ROS node is expected to convert the relative command into a global navigation target.
