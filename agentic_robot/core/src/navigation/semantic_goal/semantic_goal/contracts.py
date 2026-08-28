"""Load only independently certified execution contracts."""

from __future__ import annotations

import json
import math
from pathlib import Path


class ContractGate:
    def __init__(self, policy_path: str) -> None:
        self.path = Path(policy_path).expanduser() if policy_path else None
        self.layers = {}
        if not self.path:
            return
        document = json.loads(self.path.read_text(encoding="utf-8"))
        if not document.get("deployable"):
            raise ValueError("contract policy is not independently certified")
        self.layers = dict(document.get("layers", {}))
        if not self.layers:
            raise ValueError("contract policy has no layers")

    @property
    def enabled(self) -> bool:
        return bool(self.layers)

    def evaluate(self, stage: str, evidence: dict) -> dict:
        checked, failed = [], []
        for name, contract in self.layers.items():
            if contract.get("stage") != stage:
                continue
            metric = str(contract["metric"])
            value = evidence.get(metric)
            passed = (
                value is not None
                and math.isfinite(float(value))
                and float(value) <= float(contract["threshold"])
            )
            result = {
                "name": name,
                "metric": metric,
                "value": value,
                "threshold": float(contract["threshold"]),
                "passed": passed,
            }
            checked.append(result)
            if not passed:
                failed.append(result)
        interventions = sorted(
            {
                str(self.layers[result["name"]].get("intervention", "abstain"))
                for result in failed
            }
        )
        return {
            "stage": stage,
            "passed": not failed,
            "checked": checked,
            "failed": failed,
            "interventions": interventions,
        }


class ProgressStallMonitor:
    """Detect lack of meaningful Nav2 progress using navigation-clock time."""

    def __init__(self, timeout_sec: float, min_progress_m: float) -> None:
        if timeout_sec <= 0 or min_progress_m <= 0:
            raise ValueError("stall monitor parameters must be positive")
        self.timeout_sec = timeout_sec
        self.min_progress_m = min_progress_m
        self.best_distance = math.inf
        self.last_progress_time: float | None = None

    def observe(self, distance: float, navigation_time: float) -> bool:
        if not (math.isfinite(distance) and math.isfinite(navigation_time)):
            return False
        if distance < 0 or navigation_time < 0:
            return False
        if self.last_progress_time is None:
            self.best_distance = distance
            self.last_progress_time = navigation_time
            return False
        if distance <= self.best_distance - self.min_progress_m:
            self.best_distance = distance
            self.last_progress_time = navigation_time
            return False
        return navigation_time - self.last_progress_time >= self.timeout_sec
