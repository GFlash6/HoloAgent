#!/usr/bin/env python3
"""Delete a HoloAgent skill directory safely."""

from __future__ import annotations

from pathlib import Path
import re
import shutil
import sys


def slugify(name: str) -> str:
    slug = re.sub(r"[^a-zA-Z0-9_-]+", "-", name.strip()).strip("-").lower()
    if not slug:
        raise ValueError("skill name cannot be empty")
    return slug


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: delete_skill.py <skill-name>")
        return 1

    repo_root = Path(__file__).resolve().parents[1]
    skills_dir = repo_root / "skills"
    skill_name = slugify(sys.argv[1])
    skill_dir = skills_dir / skill_name

    if not skill_dir.exists():
        print(f"Skill not found: {skill_dir}")
        return 1

    if skill_dir.resolve().parent != skills_dir.resolve():
        print(f"Refusing to delete unexpected path: {skill_dir}")
        return 1

    shutil.rmtree(skill_dir)
    print(f"Deleted skill: {skill_dir}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
