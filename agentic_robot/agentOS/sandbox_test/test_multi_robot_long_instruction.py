#!/usr/bin/env python3
"""
多机多任务纯文本长指令测试脚本。

特点：
- 直接输入纯文本长指令
- 调用 long_horizon_text_runner.py
- 默认先 dry-run，只做 DAG 静态校验，不触发机器人
- 监控文件中会明确写入验证结论
"""

from __future__ import annotations

from pathlib import Path

from long_horizon_text_runner import LongHorizonTextRunner


TEST_INSTRUCTION = (
    "11机器和12机器同时去点位1，11机器到达后高挥手打招呼，"
    "12机器到达后和用户击掌，全部完成后同时挥手再见，然后回到点位2"
)


def main() -> int:
    runner = LongHorizonTextRunner(
        mode="multi_robot",
        dry_run=True,
        output_root=Path(__file__).resolve().parent / "output",
    )
    return runner.run(TEST_INSTRUCTION)


if __name__ == "__main__":
    raise SystemExit(main())
