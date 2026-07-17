# fastapi_control.py
from fastapi import FastAPI
import os
import requests
import threading
import uvicorn
from datetime import datetime, timezone

app = FastAPI()

# 机器人地址，可用 ROBOT_11_URL 等环境变量覆盖

# # 机器人地址-HORIZON-WLAN（另一套网段示例，需自行替换主机名）
# ROBOT_11 = os.getenv("ROBOT_11_URL", "http://<robot-11-host>:8000")
# ROBOT_12 = os.getenv("ROBOT_12_URL", "http://<robot-12-host>:8000")
# ROBOT_13 = os.getenv("ROBOT_13_URL", "http://<robot-13-host>:8000") # 控制中心(主控机器),跑fastapi_control.py的机器
# ROBOT_14 = os.getenv("ROBOT_14_URL", "http://<robot-14-host>:8000")
# ROBOT_15 = os.getenv("ROBOT_15_URL", "http://<robot-15-host>:8000")
# ROBOT_16 = os.getenv("ROBOT_16_URL", "http://<robot-16-host>:8000")
# # 机器人地址-Local5G
ROBOT_11 = os.getenv("ROBOT_11_URL", "http://192.168.124.101:8000")
ROBOT_12 = os.getenv("ROBOT_12_URL", "http://192.168.124.102:8000")
# 控制中心(主控机器),跑fastapi_control.py的机器
ROBOT_13 = os.getenv("ROBOT_13_URL", "http://192.168.124.103:8000")
ROBOT_14 = os.getenv("ROBOT_14_URL", "http://192.168.124.104:8000")
ROBOT_15 = os.getenv("ROBOT_15_URL", "http://192.168.124.105:8000")
ROBOT_16 = os.getenv("ROBOT_16_URL", "http://192.168.124.106:8000")

# ===== 新增：状态 =====
EXPECTED_ROBOTS = {
    int(robot_id)
    for robot_id in os.getenv("EXPECTED_ROBOTS", "11, 12, 13, 14, 15, 16").split(",")
    if robot_id.strip()
}
reached_robots = set()
reached_robot_targets = {}
state_lock = threading.Lock()
ROBOT_URLS = {
    11: ROBOT_11,
    12: ROBOT_12,
    13: ROBOT_13,
    14: ROBOT_14,
    15: ROBOT_15,
    16: ROBOT_16,
}
# =====================


def call_robot(url):
    try:
        response = requests.post(url, timeout=3)
        response.raise_for_status()
        try:
            return response.json()
        except ValueError:
            return {"success": response.ok, "status_code": response.status_code}
    except requests.RequestException as exc:
        return {"error": "failed", "detail": str(exc), "url": url}


def normalize_robot_selector(robot_id: str | int | None):
    if robot_id is None:
        return sorted(ROBOT_URLS.keys())

    robot_id = str(robot_id).strip()
    if robot_id in {"all", "*"}:
        return sorted(ROBOT_URLS.keys())

    try:
        parsed_robot_id = int(robot_id)
    except ValueError:
        return []
    return [parsed_robot_id] if parsed_robot_id in ROBOT_URLS else []


def trigger_navigation(target: str, robot_id: str | int | None = None):
    resp = {}
    for selected_robot_id in normalize_robot_selector(robot_id):
        resp[str(selected_robot_id)] = call_robot(
            f"{ROBOT_URLS[selected_robot_id]}/api/navigation/{target}"
        )
    return resp


@app.post("/trigger/one_point_1")
async def trigger_one_point_1(robot_id: str | None = None):
    return trigger_navigation("one_point_1", robot_id)


@app.post("/trigger/one_point_2")
async def trigger_one_point_2(robot_id: str | None = None):
    return trigger_navigation("one_point_2", robot_id)


@app.post("/trigger/one_point_3")
async def trigger_one_point_3(robot_id: str | None = None):
    return trigger_navigation("one_point_3", robot_id)


@app.post("/trigger/one_point_4")
async def trigger_one_point_4(robot_id: str | None = None):
    return trigger_navigation("one_point_4", robot_id)


@app.post("/trigger/multi_point_1")
async def trigger_multi_point_1(robot_id: str | None = None):
    return trigger_navigation("multi_point_1", robot_id)


@app.post("/trigger/multi_point_2")
async def trigger_multi_point_2(robot_id: str | None = None):
    return trigger_navigation("multi_point_2", robot_id)


@app.post("/trigger/stop")
async def trigger_stop(robot_id: str | None = None):
    return trigger_navigation("stop", robot_id)

# ===== 新增接口：机器人到点上报 =====


@app.post("/robot_reached")
async def robot_reached(data: dict):
    robot_id = int(data["robot_id"])
    if robot_id not in ROBOT_URLS:
        return {
            "accepted": False,
            "reason": "unknown_robot_id",
            "robot_id": robot_id,
        }

    target = data.get("target") or data.get("waypoint_name")
    updated_at = data.get("updated_at") or datetime.now(
        timezone.utc).isoformat()

    with state_lock:
        reached_robots.add(robot_id)
        reached_robot_targets[robot_id] = {
            "target": target,
            "updated_at": updated_at,
        }
        expected_reached = EXPECTED_ROBOTS.issubset(reached_robots)

    if expected_reached:
        print("✅ all expected robots reached waypoint")

    return {
        "accepted": True,
        "reached": sorted(reached_robots),
        "all_reached": expected_reached,
        "robot_id": robot_id,
        "target": target,
        "updated_at": updated_at,
    }


@app.get("/robot_reached/{robot_id}")
async def get_robot_reached(robot_id: int, target: str | None = None):
    with state_lock:
        target_info = reached_robot_targets.get(robot_id, {})
        reached = robot_id in reached_robots
        stored_target = target_info.get("target")
        target_matched = target is None or stored_target == target

    return {
        "robot_id": robot_id,
        "reached": reached and target_matched,
        "target": stored_target,
        "target_matched": target_matched,
        "updated_at": target_info.get("updated_at"),
    }


@app.post("/reset_reached")
async def reset_reached(data: dict | None = None):
    robot_ids = []
    if isinstance(data, dict):
        robot_ids = [int(robot_id) for robot_id in data.get("robot_ids", [])]

    with state_lock:
        if robot_ids:
            for robot_id in robot_ids:
                reached_robots.discard(robot_id)
                reached_robot_targets.pop(robot_id, None)
        else:
            reached_robots.clear()
            reached_robot_targets.clear()

        remaining = sorted(reached_robots)

    return {
        "reset_robot_ids": robot_ids,
        "remaining_reached": remaining,
    }

if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8080)
