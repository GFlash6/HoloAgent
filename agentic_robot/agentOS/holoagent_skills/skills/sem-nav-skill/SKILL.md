---
name: sem-nav-skill
description: |
  Skill for semantic navigation requests using floor, room, and object level targets.

  **Use this skill when:**
  - The user asks the robot to go to a room, landmark, or object
  - The task refers to semantic targets instead of waypoint IDs
  - The user asks about `/api/semantic_nav` or `/chat_loc_pub`
---

# Semantic Navigation Skill

Use this skill for semantic navigation requests.

## Preferred Interfaces

1. HTTP:
   - `POST /api/semantic_nav`
   - JSON body: `{"cmd":"floor,room,object"}`

2. ROS topic:
   - `/chat_loc_pub`

## Workflow

1. Extract the semantic target as floor, room, and object level information.
2. Confirm the target is specific enough for the robot to execute.
3. Send the request through the documented HTTP or ROS interface.
4. Monitor the returned status and stop for clarification if the target is ambiguous.
5. Report the semantic target and execution result.

## Safety Rules

- Confirm the semantic target is explicit enough to execute.
- Prefer semantic navigation only when the task refers to rooms, floors, or objects.
- If the target is ambiguous, ask for clarification.
- Do not fabricate room names, floor labels, or object identifiers that are not provided.

## Example

See `assets/sem_nav_examples.sh`.
