#!/usr/bin/env python3
"""HoloAgent main runtime: plan a task DAG and dispatch registered skills."""

from __future__ import annotations

import argparse
from pathlib import Path

from sandbox_test.long_horizon_text_runner import (
    DEFAULT_OUTPUT_ROOT,
    LongHorizonTextRunner,
)


def run_task(task: str, mode: str, dry_run: bool, output_root: Path) -> int:
    runner = LongHorizonTextRunner(mode, dry_run, output_root)
    return runner.run(task)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="HoloAgent task planner and skill dispatcher"
    )
    parser.add_argument("--task", help="Run one task and exit; omit for interactive mode")
    parser.add_argument(
        "--mode", choices=["single_robot", "multi_robot"], default="single_robot"
    )
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT))
    args = parser.parse_args()
    output_root = Path(args.output_root)

    if args.task:
        return run_task(args.task, args.mode, args.dry_run, output_root)

    print("HoloAgent task mode. Enter a task, or Ctrl-D to exit.")
    while True:
        try:
            task = input("task> ").strip()
        except EOFError:
            print()
            return 0
        if task:
            code = run_task(task, args.mode, args.dry_run, output_root)
            print(f"task_result={code}")


if __name__ == "__main__":
    raise SystemExit(main())
