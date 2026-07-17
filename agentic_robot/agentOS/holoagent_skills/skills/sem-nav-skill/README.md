# Semantic Navigation Skill

`sem-nav-skill` sends semantic navigation queries to the robot-side interface.

## Purpose

Use this skill when the agent needs to navigate toward a semantic target such as a room, object, or floor/object combination.

## Files

```text
sem-nav-skill/
├── SKILL.md
├── README.md
├── assets/
└── scripts/
    └── semantic_nav.py
```

## Usage

```bash
python3 agentic_robot/agentOS/holoagent_skills/skills/sem-nav-skill/scripts/semantic_nav.py \
  --robot-url http://127.0.0.1:8000 \
  --cmd "unknown,unknown,coffee machine"
```

Recommended semantic query format:

```text
floor,room,object
```

Examples:

- `1F,pantry,coffee machine`
- `unknown,meeting room,whiteboard`
- `unknown,unknown,charging station`

## Notes

- The semantic navigation backend must already be configured and reachable.
- Query quality depends on the scene graph or semantic map available on the robot.
