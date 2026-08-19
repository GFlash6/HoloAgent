#!/usr/bin/env python3
"""
单机多任务纯文本长指令测试脚本。

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
    "11机器先去点位1，到了以后高挥手打招呼，然后去点位2和用户击掌，"
    "最后回到点位3并转身挥手再见"
)


def main() -> int:
    runner = LongHorizonTextRunner(
        mode="single_robot",
        dry_run=True,
        output_root=Path(__file__).resolve().parent / "output",
    )
    return runner.run(TEST_INSTRUCTION)


if __name__ == "__main__":
    raise SystemExit(main())
