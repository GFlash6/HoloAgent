#!/usr/bin/env python3
"""Validate HoloAgent skill repository layout."""

from __future__ import annotations

from pathlib import Path


REQUIRED_SKILL_SECTIONS = [
    "# ",
    "## ",
]

RECOMMENDED_SKILL_DIRS = [
    "assets",
    "scripts",
]


def main() -> int:
    repo_root = Path(__file__).resolve().parents[1]
    skills_dir = repo_root / "skills"

    if not skills_dir.exists():
        print(f"[ERROR] skills directory not found: {skills_dir}")
        return 1

    errors: list[str] = []
    warnings: list[str] = []
    skill_dirs = sorted(path for path in skills_dir.iterdir() if path.is_dir())

    if not skill_dirs:
        errors.append("No skill directories found under skills/")

    for skill_dir in skill_dirs:
        skill_md = skill_dir / "SKILL.md"
        readme_md = skill_dir / "README.md"

        if not skill_md.exists():
            errors.append(f"{skill_dir.name}: missing SKILL.md")
            continue

        content = skill_md.read_text(encoding="utf-8")
        if not content.startswith("---\nname:"):
            errors.append(
                f"{skill_dir.name}: SKILL.md missing YAML front matter header")
        if "description: |" not in content:
            errors.append(
                f"{skill_dir.name}: SKILL.md missing description block")
        if "**Use this skill when:**" not in content:
            errors.append(
                f"{skill_dir.name}: SKILL.md missing trigger guidance")
        if "## Workflow" not in content:
            errors.append(
                f"{skill_dir.name}: SKILL.md missing workflow section")
        if "## Safety Rules" not in content:
            errors.append(
                f"{skill_dir.name}: SKILL.md missing safety rules section")

        for marker in REQUIRED_SKILL_SECTIONS:
            if marker not in content:
                errors.append(
                    f"{skill_dir.name}: SKILL.md missing heading marker {marker!r}")

        if not readme_md.exists():
            warnings.append(f"{skill_dir.name}: missing README.md")

        for dirname in RECOMMENDED_SKILL_DIRS:
            if not (skill_dir / dirname).exists():
                warnings.append(
                    f"{skill_dir.name}: missing recommended directory {dirname}/")

    if errors:
        print("HoloAgent skill validation failed:")
        for error in errors:
            print(f"- {error}")
        if warnings:
            print("\nWarnings:")
            for warning in warnings:
                print(f"- {warning}")
        return 1

    print("HoloAgent skill validation passed.")
    for skill_dir in skill_dirs:
        print(f"- {skill_dir.name}")

    if warnings:
        print("\nWarnings:")
        for warning in warnings:
            print(f"- {warning}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
