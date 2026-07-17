#!/usr/bin/env python3
"""Create a new HoloAgent skill scaffold."""

from __future__ import annotations

from pathlib import Path
import re
import sys


SKILL_TEMPLATE = """---
name: {skill_name}
description: |
  Describe what this skill does.

  **Use this skill when:**
  - The user explicitly asks for {skill_name}
  - The task matches the capability of this skill
  - The task requires a documented robot interface or workflow handled by this skill
---

# {title}

## Purpose

Describe the purpose of this skill.

## When to Use

- Add concrete trigger conditions here.
- Explain what user intent should map to this skill.

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

- Do not invent unavailable robot capabilities.
- Confirm risky actions before execution.
- Prefer documented interfaces over internal implementation details.

## Examples

- See `README.md`
- See `assets/examples.sh`
"""

README_TEMPLATE = """# {title} 使用说明

## 1. 功能简介

请在这里说明该 skill 的核心能力。

## 2. 适用场景

- 场景 1
- 场景 2

## 3. 推荐接口

### HTTP

请补充对应接口。

### ROS Topic

请补充对应 topic。

### Python 脚本

```bash
python3 agentic_robot/agentOS/holoagent_skills/skills/{skill_name}/scripts/{script_name} --help
```

## 4. 示例脚本

```bash
bash agentic_robot/agentOS/holoagent_skills/skills/{skill_name}/assets/examples.sh
```

## 5. 使用约束

- 补充安全约束
- 补充前置条件
"""

SCRIPT_TEMPLATE = """#!/usr/bin/env python3
\"\"\"Example helper script for {skill_name}.\"\"\"

from __future__ import annotations

import argparse
import json


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Helper script for {skill_name}")
    parser.add_argument("--dry-run", action="store_true", help="Print request without executing")
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    payload = {{
        "skill": "{skill_name}",
        "dry_run": args.dry_run,
    }}
    print(json.dumps(payload, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
"""

SHELL_TEMPLATE = """#!/usr/bin/env bash
set -euo pipefail

echo "[INFO] Example commands for {skill_name}"
echo "python3 agentic_robot/agentOS/holoagent_skills/skills/{skill_name}/scripts/{script_name} --dry-run"
"""


def slugify(name: str) -> str:
    slug = re.sub(r"[^a-zA-Z0-9_-]+", "-", name.strip()).strip("-").lower()
    if not slug:
        raise ValueError("skill name cannot be empty")
    return slug


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: create_skill.py <skill-name>")
        return 1

    repo_root = Path(__file__).resolve().parents[1]
    skills_dir = repo_root / "skills"
    skill_name = slugify(sys.argv[1])
    skill_dir = skills_dir / skill_name
    script_name = f"{skill_name.replace('-', '_')}.py"

    if skill_dir.exists():
        print(f"Skill already exists: {skill_dir}")
        return 1

    (skill_dir / "assets").mkdir(parents=True, exist_ok=False)
    (skill_dir / "scripts").mkdir(parents=True, exist_ok=False)

    skill_md = skill_dir / "SKILL.md"
    readme_md = skill_dir / "README.md"
    script_py = skill_dir / "scripts" / script_name
    example_sh = skill_dir / "assets" / "examples.sh"

    skill_md.write_text(
        SKILL_TEMPLATE.format(
            skill_name=skill_name,
            title=skill_name.replace("-", " ").title(),
        ),
        encoding="utf-8",
    )
    readme_md.write_text(
        README_TEMPLATE.format(
            skill_name=skill_name,
            title=skill_name.replace("-", " ").title(),
            script_name=script_name,
        ),
        encoding="utf-8",
    )
    script_py.write_text(
        SCRIPT_TEMPLATE.format(skill_name=skill_name),
        encoding="utf-8",
    )
    example_sh.write_text(
        SHELL_TEMPLATE.format(skill_name=skill_name, script_name=script_name),
        encoding="utf-8",
    )

    print(f"Created skill scaffold: {skill_dir}")
    print(f"- {skill_md}")
    print(f"- {readme_md}")
    print(f"- {script_py}")
    print(f"- {example_sh}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
