"""Small, dependency-free client for the existing semantic HTTP processes."""

from __future__ import annotations

import json
import math
from urllib.error import HTTPError
from urllib.request import Request, urlopen


class SemanticBackendError(RuntimeError):
    pass


def post_json(url: str, payload: dict, timeout: float) -> dict:
    request = Request(
        url=url,
        data=json.dumps(payload).encode("utf-8"),
        headers={"Content-Type": "application/json"},
        method="POST",
    )
    try:
        with urlopen(request, timeout=timeout) as response:
            result = json.loads(response.read().decode("utf-8"))
    except HTTPError as exc:
        try:
            body = json.loads(exc.read().decode("utf-8"))
            detail = body.get("detail", str(body))
        except (UnicodeDecodeError, json.JSONDecodeError):
            detail = exc.reason
        raise SemanticBackendError(f"HTTP {exc.code}: {detail}") from exc
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
    raw_second_score = result.get("second_score")
    has_second_score = raw_second_score is not None
    second_score = float(raw_second_score) if has_second_score else 0.0
    if has_second_score and not math.isfinite(second_score):
        raise ValueError("semantic backend returned a non-finite second_score")
    status = str(result.get("status", "FOUND"))
    raw_min_margin = result.get("backend_min_margin")
    has_backend_min_margin = raw_min_margin is not None
    return {
        "found": status == "FOUND",
        "object_id": str(result.get("object_id", "")),
        "center": center,
        "score": score,
        "has_second_score": has_second_score,
        "second_score": second_score,
        "passes_backend_threshold": bool(
            result.get("passes_backend_threshold", status == "FOUND")
        ),
        "backend_min_score": float(result.get("backend_min_score", 0.0)),
        "has_backend_min_margin": has_backend_min_margin,
        "backend_min_margin": (
            float(raw_min_margin) if has_backend_min_margin else 0.0
        ),
        "observation_count": max(0, int(result.get("observation_count", 0))),
        "source_timestamp_ms": max(0, int(result.get("source_timestamp_ms", 0))),
        "detail": str(result.get("detail", status)),
    }
