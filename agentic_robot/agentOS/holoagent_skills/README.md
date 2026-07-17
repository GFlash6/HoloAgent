# HoloAgent Skills

This directory contains the HoloAgent skill registry and the supporting tooling used to manage skill definitions in a consistent, repository-local format.

## Goals

The skill system is designed to provide:

- a stable directory layout for each skill
- machine-readable skill specifications
- human-readable usage documentation
- executable helper scripts and examples
- CRUD tooling for skill lifecycle management

## Directory Layout

```text
holoagent_skills/
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
    ├── arm-skill/
    ├── rel-move-skill/
    ├── robot-service/
    ├── sem-nav-skill/
    └── workflow/
```

## Skill Directory Contract

Each skill directory should follow this structure:

```text
skills/<skill-name>/
├── SKILL.md          # Machine-facing skill specification
├── README.md         # Human-facing usage guide
├── assets/           # Example prompts, shell snippets, or reference files
└── scripts/          # Python or shell helpers used by the skill
```

## Built-in Skills

- `arm-skill`: trigger robot arm actions
- `rel-move-skill`: execute relative movement commands
- `robot-service`: call robot-side HTTP services
- `sem-nav-skill`: perform semantic navigation requests
- `workflow`: compose multi-step workflows

## Management Scripts

### List all skills

```bash
python3 agentic_robot/agentOS/holoagent_skills/scripts/list_skills.py
```

### Show one skill

```bash
python3 agentic_robot/agentOS/holoagent_skills/scripts/show_skill.py arm-skill
```

### Create a new skill scaffold

```bash
python3 agentic_robot/agentOS/holoagent_skills/scripts/create_skill.py demo-skill
```

### Delete a skill

```bash
python3 agentic_robot/agentOS/holoagent_skills/scripts/delete_skill.py demo-skill
```

### Validate the registry

```bash
python3 agentic_robot/agentOS/holoagent_skills/scripts/validate_skills.py
```

### Sync legacy markdown documents

```bash
python3 agentic_robot/agentOS/holoagent_skills/scripts/sync_legacy_docs.py
```

## Documentation

- CRUD tutorial: [`docs/skill_crud_tutorial.md`](docs/skill_crud_tutorial.md)
- Per-skill usage guides: see each `skills/<name>/README.md`
- Per-skill runtime specs: see each `skills/<name>/SKILL.md`

## Notes

- Keep `SKILL.md` concise and runtime-oriented.
- Keep `README.md` focused on operator and developer usage.
- Put runnable examples in `assets/` or `scripts/`, not inline-only documentation.
