---
name: arm-skill
description: |
  Skill for triggering predefined robot arm actions through the official OpenClaw control interfaces.

  **Use this skill when:**
  - The user explicitly requests an arm action
  - The task requires a predefined arm skill such as wave, grasp, or reset
  - The user asks how to trigger arm actions through HTTP or ROS topics
  - The task involves manipulation but only through documented arm skill interfaces
---

# Arm Skill

Use this skill for predefined arm actions.

## Preferred Interfaces

1. HTTP:
   - `POST /api/arm/{skill}`

2. ROS topic:
   - `arm_signal_pub`

## Workflow

1. Confirm the requested arm action name and target robot are explicit.
2. Check that the robot service or ROS interface for arm control is available.
3. Prefer a predefined arm skill such as `release_arm`, `high_five`, `shake_hand`, or another documented action.
4. Trigger the action through the documented HTTP or ROS interface.
5. Observe the response and report success, failure, or the need for clarification.

## Safety Rules

- Confirm the requested arm action is explicit and safe before execution.
- Prefer predefined skill names instead of free-form motion generation.
- Do not expose FIFO internals to the user.
- Do not trigger arm motion when the workspace, nearby people, or collision risk is unclear.
- Remember that the arm automatically executes `release_arm` after a gesture for safety.
- Respect FSM constraints for arm actions and check robot state when needed.

## Example

See `assets/arm_examples.sh`.
