#!/usr/bin/env python3
"""Show detailed information for a HoloAgent skill."""

from __future__ import annotations

from pathlib import Path
import re
import sys


def extract_front_matter(content: str) -> tuple[str, str]:
    if not content.startswith("---\n"):
        return "", content

    parts = content.split("---\n", 2)
    if len(parts) < 3:
        return "", content
    return parts[1], parts[2].lstrip("\n")


def extract_description(front_matter: str) -> str:
    match = re.search(r"description:\s*\|\n((?:[ \t]+.*\n?)*)", front_matter)
    if not match:
        return ""

    lines = []
    for line in match.group(1).splitlines():
        lines.append(line.strip())
    return "\n".join(line for line in lines if line).strip()


def main() -> int:
    if len(sys.argv) != 2:
        print("Usage: show_skill.py <skill-name>")
        return 1

    repo_root = Path(__file__).resolve().parents[1]
    skill_name = sys.argv[1].strip()
    skill_dir = repo_root / "skills" / skill_name
    skill_md = skill_dir / "SKILL.md"
    readme_md = skill_dir / "README.md"
    scripts_dir = skill_dir / "scripts"
    assets_dir = skill_dir / "assets"

    if not skill_dir.exists():
        print(f"Skill not found: {skill_dir}")
        return 1

    if not skill_md.exists():
        print(f"SKILL.md not found: {skill_md}")
        return 1

    content = skill_md.read_text(encoding="utf-8")
    front_matter, body = extract_front_matter(content)
    description = extract_description(front_matter)

    print(f"Skill: {skill_name}")
    print(f"Directory: {skill_dir}")
    print(f"SKILL.md: {skill_md}")

    if description:
        print("\nDescription:")
        print(description)

    print("\nFiles:")
    print(f"- README.md: {'yes' if readme_md.exists() else 'no'}")
    print(f"- scripts/: {'yes' if scripts_dir.exists() else 'no'}")
    print(f"- assets/: {'yes' if assets_dir.exists() else 'no'}")

    if scripts_dir.exists():
        script_files = sorted(
            path.name for path in scripts_dir.iterdir() if path.is_file())
        if script_files:
            print("\nScripts:")
            for name in script_files:
                print(f"- {name}")

    if assets_dir.exists():
        asset_files = sorted(
            path.name for path in assets_dir.iterdir() if path.is_file())
        if asset_files:
            print("\nAssets:")
            for name in asset_files:
                print(f"- {name}")

    headings = []
    for line in body.splitlines():
        if line.startswith("## "):
            headings.append(line[3:].strip())

    if headings:
        print("\nSections:")
        for heading in headings:
            print(f"- {heading}")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
