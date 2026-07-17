---
name: rel-move-skill
description: |
  Skill for relative robot movement commands through the documented OpenClaw interfaces.

  **Use this skill when:**
  - The user asks the robot to move forward, backward, left, or right relatively
  - The task requires a short relative adjustment instead of named-point navigation
  - The user asks about `/api/relative_nav` or `/relative_nav`
---

# Relative Move Skill

Use this skill for relative movement commands.

## Preferred Interfaces

1. HTTP:
   - `POST /api/relative_nav`
   - JSON body: `{"cmd":"forward,left,degrees"}`

2. ROS topic:
   - `/relative_nav`

## Workflow

1. Confirm the target relative pose with the user or calling system.
2. Interpret the request as three values: forward displacement, left displacement, and heading rotation.
3. Send the request through the documented HTTP or ROS interface.
4. Monitor the response and stop for clarification if the target pose is ambiguous.
5. Report the executed relative target and the returned status.

## Safety Rules

- Confirm the movement target before execution.
- Use relative movement only for short, explicit adjustments.
- Treat the three core parameters as: forward meters, left meters, and relative heading rotation degrees.
- If the command is ambiguous, clarify before acting.

## Example

See `assets/rel_move_examples.sh`.
