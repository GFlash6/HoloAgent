---
name: workflow
description: |
  High-level workflow skill for long-horizon task decomposition, dependency management,
  monitoring, and replanning in OpenClaw / HoloAgent robot systems.

  **Use this skill when:**
  - The user gives a multi-step task
  - The task spans navigation, manipulation, monitoring, or recovery
  - The task involves one or more robots
  - The task requires dependency ordering or replanning
  - The user asks for long-horizon execution logic or orchestration
---

# Workflow

This skill is the high-level execution policy for OpenClaw robot systems.

It does not directly replace low-level action skills. Instead, it decides:

- what should be done first
- which robot should do it
- what prerequisites must be checked
- when to wait, monitor, stop, or replan

## Workflow

This skill orchestrates long-horizon execution by decomposing tasks, selecting lower-level skills, checking prerequisites, and monitoring progress.

## Single-Robot Workflow

Default bringup dependency order:

1. start sensors
2. start localization
3. start navigation
4. start navigation bridge
5. start semantic navigation if needed
6. start voice interaction if needed
7. start recording if needed
8. execute the user task
9. monitor until success, failure, or cancellation

## Multi-Robot Workflow

When multiple robots are involved:

1. classify the collaboration mode
   - independent parallel execution
   - coordinated relay execution
   - leader-follower execution
   - shared-goal collaborative execution
2. assign a clear role to each robot
3. decompose the global task into robot-local subtasks
4. enforce dependency ordering
5. monitor local and global status
6. isolate failures and replan safely

## Required Task Structure

Before execution, convert the request into:

- task_goal
- task_type
- robot_scope
- required_skills
- prerequisites
- execution_steps
- success_criteria
- monitoring_signals
- abort_conditions

## Execution Rules

- Always check prerequisites before each major step.
- Execute one logically consistent step at a time.
- Verify the result before continuing.
- If the task is ambiguous, clarify before acting.
- Never invent unavailable robot capabilities.

## Monitoring Rules

At minimum monitor:

- sensor availability
- localization health
- navigation progress
- action completion
- robot responsiveness
- timeout
- safety events

## Replanning Rules

Replan only when:

- the original plan is blocked
- the environment changed
- a robot failed
- the user changed the goal
- a safer equivalent route exists

## Safety Rules

- Never skip prerequisite checks for sensing, localization, navigation, or manipulation.
- Do not invent unavailable robot capabilities or undocumented recovery actions.
- If the task is ambiguous, unsafe, or under-specified, stop and clarify before execution.
- Replan conservatively and preserve human safety, robot safety, and environment safety.

## Reporting Format

Use this concise structure for long-horizon tasks:

- Goal:
- Scope:
- Plan:
- Current Step:
- Status:
- Risk:
- Next Action:
