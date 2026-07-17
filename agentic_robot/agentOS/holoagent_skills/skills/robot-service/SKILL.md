---
name: robot-service
description: |
  Canonical skill for OpenClaw robot service interfaces, covering robot_bridge on each robot
  and multi_robot_ctl on the control center.

  **Use this skill when:**
  - The user wants to trigger robot actions through HTTP services
  - The user asks about robot_bridge or multi_robot_ctl
  - The task involves remote navigation, arm control, or semantic navigation
  - The task requires multi-robot fan-out control
  - The user needs the official external control surface instead of internal FIFO details
---

# Robot Service

This skill documents the canonical external control surface for OpenClaw robots.

## Architecture

### Layer 1: robot_bridge on each robot

Default robot-side HTTP server:

- host: `0.0.0.0`
- port: `8000`

Supported mappings:

1. Navigation signal
   - `POST /api/navigation/{name}`
   - ROS topic: `chat_signal_pub`

2. Relative navigation
   - `POST /api/relative_nav`
   - ROS topic: `/relative_nav`
   - JSON body: `{"cmd": "forward,left,degrees"}`

3. Semantic navigation
   - `POST /api/semantic_nav`
   - ROS topic: `/chat_loc_pub`
   - JSON body: `{"cmd": "floor,room,object"}`

4. Arm skill
   - `POST /api/arm/{skill}`
   - ROS topic: `arm_signal_pub`

5. Stop navigation
   - `POST /api/navigation/stop`
   - ROS topic: `chat_signal_pub`

Health check:

- `GET /health`

### Layer 2: multi_robot_ctl on the master machine

Default control-center port:

- `8080`

Supported fan-out endpoints:

- `POST /trigger/one_point_1`
- `POST /trigger/one_point_2`
- `POST /trigger/one_point_3`
- `POST /trigger/one_point_4`
- `POST /trigger/multi_point_1`
- `POST /trigger/multi_point_2`
- `POST /trigger/stop`

Optional query parameter:

- `robot_id=<id>`
- `robot_id=all`

## Workflow

1. Identify whether the request should go to a single robot bridge or the multi-robot control layer.
2. Confirm the target endpoint, robot scope, and payload format before sending the request.
3. Prefer documented HTTP endpoints and only fall back to ROS topics when appropriate for the integration.
4. Execute one service request at a time and inspect the returned status.
5. Report the endpoint, payload, and result clearly for downstream workflow tracking.

## Safety Rules

- If the target robot is ambiguous, ask which robot should execute.
- If the command is unsafe or unclear, do not trigger the service.
- If unsure, stop and clarify.

## Examples

See:

- `README.md`
- `assets/service_examples.sh`