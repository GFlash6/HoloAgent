"""Pure localization health classification."""

import math


def localization_state(
    registration_success: bool | None,
    score: float,
    consecutive_failures: int,
    failure_limit: int,
) -> str:
    if registration_success is None or not math.isfinite(score):
        return "INITIALIZING"
    if not registration_success and consecutive_failures >= failure_limit:
        return "LOST"
    if not registration_success:
        return "DEGRADED"
    return "TRACKING"
