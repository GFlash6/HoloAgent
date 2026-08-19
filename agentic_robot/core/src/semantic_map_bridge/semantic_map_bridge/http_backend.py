"""Small, dependency-free client for the existing semantic HTTP processes."""

from __future__ import annotations

import json
import math
from urllib.request import Request, urlopen


def post_json(url: str, payload: dict, timeout: float) -> dict:
    request = Request(
        url=url,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    with urlopen(request, timeout=timeout) as response:
        result = json.loads(response.read().decode("utf-8"))
    if not isinstance(result, dict):
        raise ValueError("semantic backend returned a non-object response")
    return result


def parse_query_result(result: dict) -> dict:
    frame_id = str(result.get("frame_id", "map"))
    if frame_id != "map":
        raise ValueError(f"semantic backend returned unsupported frame {frame_id!r}")
    center = result.get("center_map")
    if not isinstance(center, (list, tuple)) or len(center) != 3:
        raise ValueError("semantic backend returned an invalid center_map")
    center = [float(value) for value in center]
    score = float(result.get("score", float("nan")))
    if not all(math.isfinite(value) for value in center) or not math.isfinite(score):
        raise ValueError("semantic backend returned non-finite coordinates or score")
    status = str(result.get("status", "FOUND"))
    return {
        "found": status == "FOUND",
        "object_id": str(result.get("object_id", "")),
        "center": center,
        "score": score,
        "observation_count": max(0, int(result.get("observation_count", 0))),
        "source_timestamp_ms": max(0, int(result.get("source_timestamp_ms", 0))),
        "detail": str(result.get("detail", status)),
    }
