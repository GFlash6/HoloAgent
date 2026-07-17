#!/usr/bin/env python3
"""List all HoloAgent skills in the repository."""

from __future__ import annotations

from pathlib import Path
import re


def extract_description(content: str) -> str:
    match = re.search(r"description:\s*\|\n((?:[ \t]+.*\n?)*)", content)
    if not match:
        return ""

    lines = []
    for line in match.group(1).splitlines():
        stripped = line.strip()
        if stripped.startswith("**Use this skill when:**"):
            break
        if stripped:
            lines.append(stripped)
    return " ".join(lines).strip()


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    skills_dir = repo_root / "skills"

    if not skills_dir.exists():
        print(f"skills directory not found: {skills_dir}")
        return 1

    skill_dirs = sorted(
        path for path in skills_dir.iterdir() if path.is_dir() and (path / "SKILL.md").exists()
    )

    if not skill_dirs:
        print("No skills found.")
        return 0

    print("HoloAgent skills:")
    for skill_dir in skill_dirs:
        skill_md = skill_dir / "SKILL.md"
        readme_md = skill_dir / "README.md"
        scripts_dir = skill_dir / "scripts"
        assets_dir = skill_dir / "assets"
        description = extract_description(skill_md.read_text(encoding="utf-8"))

        print(f"- {skill_dir.name}")
        if description:
            print(f"  description: {description}")
        print(f"  skill: {skill_md}")
        print(f"  readme: {'yes' if readme_md.exists() else 'no'}")
        print(f"  scripts: {'yes' if scripts_dir.exists() else 'no'}")
        print(f"  assets: {'yes' if assets_dir.exists() else 'no'}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
