#!/usr/bin/env python3
import argparse
import subprocess
import time
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[3]
UNITREE_SCRIPT_DIR = REPO_ROOT / "robots" / "unitree" / "scripts"

GROUPS = {
    "sensors": UNITREE_SCRIPT_DIR / "run_sensors.sh",
    "localization": UNITREE_SCRIPT_DIR / "run_localization.sh",
    "nav": UNITREE_SCRIPT_DIR / "run_nav.sh",
    "agent": UNITREE_SCRIPT_DIR / "run_agent.sh",
    "audio": UNITREE_SCRIPT_DIR / "run_audio.sh",
    "ctl": UNITREE_SCRIPT_DIR / "run_ctl.sh",
}

DEFAULT_GROUPS = ["sensors", "localization", "nav", "ctl", "agent"]


def start_group(group_name: str) -> None:
    script_path = GROUPS[group_name]
    if not script_path.exists():
        raise FileNotFoundError(f"script not found: {script_path}")

    subprocess.run(
        ["bash", str(script_path)],
        check=True,
        env={
            **dict(__import__("os").environ),
            "RUN_IN_BACKGROUND": "1",
        },
    )


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Start G1 background daemon groups for long-running robot services."
    )
    parser.add_argument(
        "--groups",
        nargs="+",
        choices=sorted(GROUPS.keys()),
        default=DEFAULT_GROUPS,
        help="Groups to start. Default: sensors localization nav audio ctl",
    )
    parser.add_argument(
        "--interval",
        type=float,
        default=2.0,
        help="Delay in seconds between group startups.",
    )
    args = parser.parse_args()

    print("Starting G1 background daemon groups:")
    for group_name in args.groups:
        script_path = GROUPS[group_name]
        print(f"  - {group_name}: {script_path}")
        start_group(group_name)
        time.sleep(args.interval)

    print("All requested groups have been launched in tmux background sessions.")
    print("Use tmux ls to inspect sessions and tmux attach -t <session> when needed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
