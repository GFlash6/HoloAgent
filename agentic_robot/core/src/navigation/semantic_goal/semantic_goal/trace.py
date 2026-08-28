"""Append-only JSONL traces for semantic-navigation experiments."""

from __future__ import annotations

import json
import threading
import time
from pathlib import Path


class JsonlTrace:
    def __init__(self, path: str, ros_time_ns=None) -> None:
        self.path = Path(path).expanduser() if path else None
        self.ros_time_ns = ros_time_ns
        self.lock = threading.Lock()
        if self.path:
            self.path.parent.mkdir(parents=True, exist_ok=True)

    def write(self, event: str, **fields) -> None:
        if not self.path:
            return
        record = {
            "event": event,
            "wall_time_ns": time.time_ns(),
            "monotonic_ns": time.monotonic_ns(),
            **fields,
        }
        if self.ros_time_ns is not None:
            record["ros_time_ns"] = int(self.ros_time_ns())
        line = json.dumps(record, ensure_ascii=False, allow_nan=False) + "\n"
        with self.lock, self.path.open("a", encoding="utf-8") as stream:
            stream.write(line)
