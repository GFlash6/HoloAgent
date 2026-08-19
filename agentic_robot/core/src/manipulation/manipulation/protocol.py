"""Legacy G1 arm result parsing."""


def parse_arm_result(value: str) -> tuple[bool, str, str] | None:
    parts = value.split(":", maxsplit=2)
    if len(parts) < 2 or parts[0] not in {"arm_finish", "arm_failed"}:
        return None
    return parts[0] == "arm_finish", parts[1], parts[2] if len(parts) == 3 else ""
