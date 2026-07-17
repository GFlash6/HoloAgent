# HoloAgent Skill CRUD Tutorial

This document describes the directory conventions, maintenance workflow, repository-level scripts, and usage entry points for the skills under `agentic_robot/agentOS/holoagent_skills/`.

---

## 1. Directory Conventions

Each skill should live under `skills/<skill-name>/`. The recommended structure is:

```text
skills/<skill-name>/
├── SKILL.md
├── README.md
├── scripts/
│   └── *.py
└── assets/
    └── *.sh
```

Where:

- `SKILL.md`: Required. Defines the skill metadata, trigger conditions, workflow, and safety rules.
- `README.md`: Recommended. Documents the skill interface, examples, and usage constraints.
- `scripts/`: Recommended. Stores Python helper scripts.
- `assets/`: Recommended. Stores shell examples, command templates, and reference materials.

Repository-level layout:

```text
agentic_robot/agentOS/holoagent_skills/
├── README.md
├── docs/
│   └── skill_crud_tutorial.md
├── scripts/
│   ├── create_skill.py
│   ├── delete_skill.py
│   ├── list_skills.py
│   ├── show_skill.py
│   ├── sync_legacy_docs.py
│   └── validate_skills.py
└── skills/
```

---

## 2. `SKILL.md` Authoring Requirements

A valid `SKILL.md` should include at least the following:

1. YAML front matter
2. `name`
3. `description`
4. `**Use this skill when:**`
5. `## Workflow`
6. `## Safety Rules`

Recommended template:

```md
---
name: demo-skill
description: |
  Describe what this skill does.

  **Use this skill when:**
  - The user explicitly asks for this capability
  - The task matches the documented interface of this skill
---

# Demo Skill

## Purpose

Describe the purpose of this skill.

## When to Use

- Trigger condition 1
- Trigger condition 2

## Interfaces

- HTTP:
- ROS topic:
- Script entry:

## Workflow

1. Check prerequisites.
2. Explain the action plan.
3. Execute or guide the user safely.
4. Verify the result.

## Safety Rules

- Confirm risky actions before execution.
- Do not invent unavailable capabilities.

## Examples

- See `README.md`
- See `assets/examples.sh`
```

---

## 3. Creating a New Skill

### 3.1 Create a Skill with the Scaffold Script

```bash
python3 agentic_robot/agentOS/holoagent_skills/scripts/create_skill.py demo-skill
```

This command generates:

```text
skills/demo-skill/
├── SKILL.md
├── README.md
├── scripts/
│   └── demo_skill.py
└── assets/
    └── examples.sh
```

After creation, it is recommended to immediately fill in:

- Trigger conditions and safety rules in `SKILL.md`
- Interface documentation in `README.md`
- Real request logic in `scripts/*.py`
- Example commands in `assets/*.sh`

### 3.2 Create a Skill Manually

```bash
mkdir -p agentic_robot/agentOS/holoagent_skills/skills/demo-skill/{scripts,assets}
touch agentic_robot/agentOS/holoagent_skills/skills/demo-skill/SKILL.md
touch agentic_robot/agentOS/holoagent_skills/skills/demo-skill/README.md
```

---

## 4. Querying Skills

### 4.1 List All Skills

```bash
python3 agentic_robot/agentOS/holoagent_skills/scripts/list_skills.py
```

Use this command to:

- See which skills currently exist in the repository
- Check whether each skill includes `README.md`, `scripts/`, and `assets/`
- Quickly review skill descriptions

### 4.2 Show Details for a Specific Skill

```bash
python3 agentic_robot/agentOS/holoagent_skills/scripts/show_skill.py arm-skill
```

Use this command to:

- Locate the skill directory
- View the `SKILL.md` description
- Inspect the scripts and example files under the skill
- Review the section structure of `SKILL.md`

### 4.3 Validate Skill Structure

```bash
python3 agentic_robot/agentOS/holoagent_skills/scripts/validate_skills.py
```

The validation checks include:

- Whether `SKILL.md` exists
- Whether YAML front matter is present
- Whether `description: |` is present
- Whether `**Use this skill when:**` is present
- Whether `## Workflow` is present
- Whether `## Safety Rules` is present

---

## 5. Updating a Skill

When updating a skill, the recommended order is:

1. Update `SKILL.md`
2. Update `README.md`
3. Update `scripts/*.py`
4. Update `assets/*.sh`
5. Run the validation script to confirm the structure is still correct

Example:

```bash
python3 agentic_robot/agentOS/holoagent_skills/scripts/validate_skills.py
python3 agentic_robot/agentOS/holoagent_skills/scripts/show_skill.py rel-move-skill
```

Update guidelines:

- If the meaning of interface parameters changes, you must update all of the following:
  - `SKILL.md`
  - `README.md`
  - Python script argument documentation
  - Shell example commands
- If the skill name changes, prefer creating a new skill and migrating the content instead of renaming the directory directly
- If legacy compatibility documents must be preserved, run the sync script after the update

---

## 6. Deleting a Skill

```bash
python3 agentic_robot/agentOS/holoagent_skills/scripts/delete_skill.py demo-skill
```

Notes:

- This script deletes `skills/demo-skill/`
- Before deletion, confirm that no other documentation or workflow still depends on the skill

After deletion, it is recommended to run:

```bash
python3 agentic_robot/agentOS/holoagent_skills/scripts/list_skills.py
python3 agentic_robot/agentOS/holoagent_skills/scripts/validate_skills.py
```

---

## 7. Syncing Legacy Compatibility Documents

The repository root keeps the following compatibility entry files:

- `ArmSkill.md`
- `RelMoveSkill.md`
- `RobotService.md`
- `SemNavSkill.md`
- `WorkFlow.md`

After `skills/<skill-name>/README.md` is updated, run:

```bash
python3 agentic_robot/agentOS/holoagent_skills/scripts/sync_legacy_docs.py
```

This script syncs the corresponding skill `README.md` content into the legacy Markdown files at the repository root.

---

## 8. Current Skill Usage

### 8.1 `workflow`

**Purpose**: Long-horizon task orchestration, multi-step execution, dependency checks, and failure recovery.

**Primary entry points**:

- `skills/workflow/SKILL.md`
- `skills/workflow/README.md`

**Usage pattern**:

1. Break the user task into multiple steps
2. Decide which underlying skill is needed for each step
3. Record prerequisites, execution order, success criteria, and fallback strategy
4. Invoke the underlying skills in sequence

Suitable for:

- Multi-step tasks
- Multi-robot coordination tasks
- Tasks that require execution-state monitoring

---

### 8.2 `robot-service`

**Purpose**: Provides a unified wrapper for robot HTTP service requests.

**Primary entry points**:

- `skills/robot-service/SKILL.md`
- `skills/robot-service/README.md`
- `skills/robot-service/scripts/service_request.py`
- `skills/robot-service/assets/service_examples.sh`

**Python script example**:

```bash
python3 agentic_robot/agentOS/holoagent_skills/skills/robot-service/scripts/service_request.py \
  --endpoint /api/relative_nav \
  --payload '{"forward": 0.5, "left": 0.0, "rotation": 0.0}' \
  --dry-run
```

**Shell example**:

```bash
bash agentic_robot/agentOS/holoagent_skills/skills/robot-service/assets/service_examples.sh
```

---

### 8.3 `arm-skill`

**Purpose**: Triggers predefined robotic arm actions.

**Primary entry points**:

- `skills/arm-skill/SKILL.md`
- `skills/arm-skill/README.md`
- `skills/arm-skill/scripts/trigger_arm_skill.py`
- `skills/arm-skill/assets/arm_examples.sh`

**Python script example**:

```bash
python3 agentic_robot/agentOS/holoagent_skills/skills/arm-skill/scripts/trigger_arm_skill.py \
  --skill high_wave \
  --dry-run
```

**Shell example**:

```bash
bash agentic_robot/agentOS/holoagent_skills/skills/arm-skill/assets/arm_examples.sh
```

---

### 8.4 `rel-move-skill`

**Purpose**: Sends a relative pose target.

The three core parameters are:

- `forward`: How many meters to move forward relative to the current position
- `left`: How many meters to move left relative to the current position
- `rotation`: How many degrees to rotate relative to the current heading

**Primary entry points**:

- `skills/rel-move-skill/SKILL.md`
- `skills/rel-move-skill/README.md`
- `skills/rel-move-skill/scripts/relative_move.py`
- `skills/rel-move-skill/assets/rel_move_examples.sh`

**Python script example**:

```bash
python3 agentic_robot/agentOS/holoagent_skills/skills/rel-move-skill/scripts/relative_move.py \
  --forward 0.8 \
  --left 0.2 \
  --rotation 30 \
  --dry-run
```

**Shell example**:

```bash
bash agentic_robot/agentOS/holoagent_skills/skills/rel-move-skill/assets/rel_move_examples.sh
```

---

### 8.5 `sem-nav-skill`

**Purpose**: Sends a semantic navigation request.

**Primary entry points**:

- `skills/sem-nav-skill/SKILL.md`
- `skills/sem-nav-skill/README.md`
- `skills/sem-nav-skill/scripts/semantic_nav.py`
- `skills/sem-nav-skill/assets/sem_nav_examples.sh`

**Python script example**:

```bash
python3 agentic_robot/agentOS/holoagent_skills/skills/sem-nav-skill/scripts/semantic_nav.py \
  --floor 3F \
  --room meeting_room \
  --object charging_station \
  --dry-run
```

**Shell example**:

```bash
bash agentic_robot/agentOS/holoagent_skills/skills/sem-nav-skill/assets/sem_nav_examples.sh
```

---

## 9. Recommended Maintenance Workflow

Use the following workflow to maintain the skill repository:

1. Create or update a skill
2. Update the corresponding `README.md`
3. Update the corresponding `scripts/` and `assets/`
4. Run `validate_skills.py`
5. Run `show_skill.py <skill-name>` to inspect the result
6. If legacy entry compatibility is required, run `sync_legacy_docs.py`

---

## 10. Common Command Reference

```bash
# List all skills
python3 agentic_robot/agentOS/holoagent_skills/scripts/list_skills.py

# Show a specific skill
python3 agentic_robot/agentOS/holoagent_skills/scripts/show_skill.py arm-skill

# Create a skill
python3 agentic_robot/agentOS/holoagent_skills/scripts/create_skill.py demo-skill

# Delete a skill
python3 agentic_robot/agentOS/holoagent_skills/scripts/delete_skill.py demo-skill

# Validate the skill repository
python3 agentic_robot/agentOS/holoagent_skills/scripts/validate_skills.py

# Sync legacy compatibility documents
python3 agentic_robot/agentOS/holoagent_skills/scripts/sync_legacy_docs.py