# Workflow Skill

`workflow` documents how to compose multiple low-level skills into a higher-level task sequence.

## Purpose

Use this skill when a task requires ordered execution across navigation, arm actions, service calls, or multi-step robot behaviors.

## Typical Pattern

A workflow usually combines:

1. navigation or semantic navigation
2. optional relative adjustment
3. arm or service action
4. completion signaling or follow-up action

## Files

```text
workflow/
├── SKILL.md
├── README.md
├── assets/
└── scripts/
```

## Example Workflow

A simple greeting workflow might look like:

1. navigate to a waypoint
2. perform a final relative adjustment
3. trigger `wave_above_head`
4. wait for completion feedback

## Notes

- `workflow` is a composition-oriented skill rather than a single endpoint wrapper.
- Keep workflow definitions deterministic and easy to validate.
