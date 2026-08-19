"""
robot_bridge_node.py — 通用 HTTP ↔ ROS 桥节点.

取代原 api_bridge + fastapi_robot 的两段结构，单进程实现：
  - FastAPI HTTP server（uvicorn 异步）
  - rclpy Node（ROS 通信）
双向桥接作用：接收 HTTP 指令转 ROS、接收 ROS 消息转 HTTP 上报。

端点映射由 bridge_config.yaml 驱动，支持 topic / service / action 三类。
反向通道（ROS → HTTP）同样在 yaml 中声明。

启动方式：
    ros2 run robot_bridge robot_bridge_node \
        --ros-args \
        -p config_path:=/path/to/bridge_config.yaml \
        -p robot_id:=13 \
        -p control_url:=http://192.168.124.103:8080

直接运行（非 ROS 启动）：
    python3 robot_bridge_node.py
"""

from __future__ import annotations

import json
import os
import pathlib
import threading
import time
from typing import Any, Optional

import requests
import rclpy
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
from rclpy.node import Node
from std_msgs.msg import String
import uvicorn
import yaml
from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse


# ---------------------------------------------------------------------------
# Config helpers
# ---------------------------------------------------------------------------

def _expand_env(value: str, extras: dict | None = None) -> str:
    """展开 ${VAR:-default} 形式的占位符。"""
    import re
    extras = extras or {}

    def _replace(m):
        var, _, default = m.group(1).partition(":-")
        if var.startswith("msg."):
            return m.group(0)
        return extras.get(var, os.environ.get(var, default))

    return re.sub(r"\$\{([^}]+)\}", _replace, str(value))


def _load_config(path: str, robot_id: int, control_url: str) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        raw = f.read()
    extras = {"ROBOT_ID": str(robot_id), "control_url": control_url,
              "CONTROL_URL": control_url}
    expanded = _expand_env(raw, extras)
    return yaml.safe_load(expanded) or {}


def _build_msg(
        msg_type_str: str,
        body_spec: dict,
        path_params: dict,
        json_body: dict):
    """
    根据 body_spec 构造 ROS 消息。

    body_spec: {ros_field: source}
      source 可以是：
        "@path.<var>"  — 取 URL 路径参数
        str literal    — 直接用该字符串作为 HTTP body 的 JSON key 来查值
    """
    pkg, _, type_name = msg_type_str.partition("/")
    # 动态 import
    import importlib
    mod = importlib.import_module(f"{pkg}.msg")
    MsgClass = getattr(mod, type_name)
    msg = MsgClass()

    for ros_field, source in body_spec.items():
        if isinstance(source, str) and source.startswith("@path."):
            var = source[len("@path."):]
            value = path_params.get(var, "")
        else:
            value = json_body.get(source, "") if json_body else source
        setattr(msg, ros_field, value)

    return msg


def _build_action_goal(
        action_type_str: str,
        body_spec: dict,
        path_params: dict,
        json_body: dict):
    """Build an action Goal using the same flat HTTP field mapping as topics."""
    pkg, _, type_name = action_type_str.partition("/")
    import importlib
    ActionClass = getattr(importlib.import_module(f"{pkg}.action"), type_name)
    goal = ActionClass.Goal()
    for goal_field, source in body_spec.items():
        if isinstance(source, str) and source.startswith("@path."):
            value = path_params.get(source[len("@path."):], "")
        else:
            value = json_body.get(source, "") if json_body else source
        setattr(goal, goal_field, value)
    return ActionClass, goal


# ---------------------------------------------------------------------------
# ROS Node
# ---------------------------------------------------------------------------

class RobotBridgeNode(Node):
    """ROS 侧：topic publishers + 反向 HTTP 回调订阅。"""

    def __init__(self, config: dict):
        super().__init__("robot_bridge_node")
        self._config = config
        self._robot_id = config.get("robot_id", 13)
        self._control_url = str(config.get("control_url", ""))

        # 预创建 topic publishers（type=topic 的 endpoint）
        # 注意：rclpy.Node 内部已经使用 self._publishers 保存 publisher 列表，
        # 这里不能覆写该属性，否则 create_publisher() 会因类型不匹配而崩溃。
        self._topic_publishers: dict[str, Any] = {}
        self._action_clients: dict[str, ActionClient] = {}
        self._action_types: dict[str, str] = {}
        self._action_goals: dict[str, Any] = {}
        self._action_states: dict[str, dict] = {}
        self._action_lock = threading.Lock()
        for ep in config.get("endpoints", []):
            ros = ep.get("ros", {})
            if ros.get("type") == "topic":
                name = ros["name"]
                msg_type_str = ros.get("msg_type", "std_msgs/String")
                if name not in self._topic_publishers:
                    pkg, _, type_name = msg_type_str.partition("/")
                    import importlib
                    mod = importlib.import_module(f"{pkg}.msg")
                    MsgClass = getattr(mod, type_name)
                    self._topic_publishers[name] = self.create_publisher(
                        MsgClass, name, 10)
                    self.get_logger().info(
                        f"Publisher created: {name} ({msg_type_str})")
            elif ros.get("type") == "action":
                name = ros["name"]
                action_type = ros["action_type"]
                pkg, _, type_name = action_type.partition("/")
                import importlib
                ActionClass = getattr(
                    importlib.import_module(f"{pkg}.action"), type_name)
                self._action_clients[name] = ActionClient(self, ActionClass, name)
                self._action_types[name] = action_type
                self.get_logger().info(
                    f"Action client created: {name} ({action_type})")

        # 反向订阅
        for rev in config.get("reverse", []):
            ros_sub = rev.get("ros_sub", "")
            msg_type_str = rev.get("msg_type", "std_msgs/String")
            if ros_sub:
                pkg, _, type_name = msg_type_str.partition("/")
                import importlib
                mod = importlib.import_module(f"{pkg}.msg")
                MsgClass = getattr(mod, type_name)
                http_post = str(rev.get("http_post", ""))
                payload_tmpl = rev.get("payload", {})

                def _make_cb(post_url, payload_template, robot_id):
                    def _cb(msg):
                        payload = {}
                        for k, v in payload_template.items():
                            v_str = str(v)
                            v_str = v_str.replace("${robot_id}", str(robot_id))
                            v_str = v_str.replace("${msg.data}", msg.data)
                            try:
                                payload[k] = int(v_str) if v_str.lstrip(
                                    "-").isdigit() else v_str
                            except Exception:
                                payload[k] = v_str
                        self.get_logger().info(
                            f"Reverse POST {post_url} payload={payload}")
                        try:
                            requests.post(post_url, json=payload, timeout=1.0)
                        except Exception as e:
                            self.get_logger().error(
                                f"Reverse POST failed: {e}")
                    return _cb

                self.create_subscription(
                    MsgClass, ros_sub,
                    _make_cb(http_post, payload_tmpl, self._robot_id),
                    10
                )
                self.get_logger().info(
                    f"Reverse subscriber: {ros_sub} → POST {http_post}")

    def publish_to_topic(self, topic_name: str, msg) -> bool:
        pub = self._topic_publishers.get(topic_name)
        if pub is None:
            return False
        pub.publish(msg)
        return True

    def send_action_goal(self, action_name: str, goal, timeout: float = 2.0) -> str:
        client = self._action_clients.get(action_name)
        if client is None:
            raise RuntimeError(f"Action client {action_name} is not configured")
        if not client.wait_for_server(timeout_sec=timeout):
            raise RuntimeError(f"Action server {action_name} is unavailable")
        accepted = threading.Event()
        outcome = {}

        def goal_response(future):
            try:
                handle = future.result()
                outcome["handle"] = handle
            except Exception as exc:
                outcome["error"] = exc
            accepted.set()

        client.send_goal_async(goal).add_done_callback(goal_response)
        if not accepted.wait(timeout):
            raise TimeoutError(f"Action goal response timed out: {action_name}")
        if "error" in outcome:
            raise RuntimeError(str(outcome["error"]))
        handle = outcome["handle"]
        if not handle.accepted:
            raise RuntimeError(f"Action goal rejected: {action_name}")
        goal_id = bytes(handle.goal_id.uuid).hex()
        with self._action_lock:
            self._action_goals[goal_id] = handle
            self._action_states[goal_id] = {"state": "accepted", "detail": ""}

        def action_result(future):
            try:
                wrapped = future.result()
                result = wrapped.result
                if wrapped.status == GoalStatus.STATUS_CANCELED:
                    state = "canceled"
                else:
                    state = (
                        "succeeded"
                        if bool(getattr(result, "success", False))
                        else "failed"
                    )
                detail = str(getattr(result, "detail", ""))
            except Exception as exc:
                state, detail = "failed", str(exc)
            with self._action_lock:
                self._action_states[goal_id] = {"state": state, "detail": detail}
                self._action_goals.pop(goal_id, None)

        handle.get_result_async().add_done_callback(action_result)
        return goal_id

    def action_state(self, goal_id: str) -> Optional[dict]:
        with self._action_lock:
            value = self._action_states.get(goal_id)
            return dict(value) if value is not None else None

    def cancel_action(self, goal_id: str, timeout: float = 2.0) -> bool:
        with self._action_lock:
            handle = self._action_goals.get(goal_id)
        if handle is None:
            return False
        completed = threading.Event()
        outcome = {}

        def canceled(future):
            try:
                outcome["accepted"] = bool(future.result().goals_canceling)
            except Exception:
                outcome["accepted"] = False
            completed.set()

        handle.cancel_goal_async().add_done_callback(canceled)
        if not completed.wait(timeout):
            return False
        if outcome.get("accepted"):
            with self._action_lock:
                self._action_states[goal_id] = {
                    "state": "canceling", "detail": "cancel requested"}
        return bool(outcome.get("accepted"))


# ---------------------------------------------------------------------------
# FastAPI app factory
# ---------------------------------------------------------------------------

def build_fastapi_app(ros_node: RobotBridgeNode, config: dict) -> FastAPI:
    app = FastAPI(title="robot_bridge", version="0.1.0")

    for ep in config.get("endpoints", []):
        http_spec = ep.get("http", {})
        ros_spec = ep.get("ros", {})
        method = http_spec.get("method", "POST").upper()
        path = http_spec.get("path", "/")
        ros_type = ros_spec.get("type", "topic")
        ros_name = ros_spec.get("name", "")
        msg_type = ros_spec.get("msg_type", "std_msgs/String")
        action_type = ros_spec.get("action_type", "")
        body_spec = ros_spec.get("body", {})

        # Closure to capture loop variables
        def _make_handler(r_type, r_name, m_type, a_type, b_spec):
            async def _handler(request: Request):
                try:
                    json_body = {}
                    try:
                        json_body = await request.json()
                    except Exception:
                        pass

                    if r_type == "topic":
                        msg = _build_msg(
                            m_type, b_spec, request.path_params, json_body)
                        ok = ros_node.publish_to_topic(r_name, msg)
                        if not ok:
                            raise HTTPException(
                                status_code=503, detail=f"Publisher {r_name} not ready")
                        return {"success": True}

                    elif r_type == "service":
                        # service 调用（预留，按需扩展）
                        return {"success": False,
                                "detail": "service calls not yet implemented"}

                    elif r_type == "action":
                        _, goal = _build_action_goal(
                            a_type, b_spec, request.path_params, json_body)
                        goal_id = ros_node.send_action_goal(r_name, goal)
                        return {"success": True, "goal_id": goal_id}

                    else:
                        raise HTTPException(
                            status_code=400, detail=f"Unknown ros type: {r_type}")

                except HTTPException:
                    raise
                except Exception as e:
                    ros_node.get_logger().error(f"Handler error: {e}")
                    raise HTTPException(status_code=500, detail=str(e))

            return _handler

        handler = _make_handler(
            ros_type, ros_name, msg_type, action_type, body_spec)

        # 注册路由（支持 {var} 路径参数）
        if method == "POST":
            app.add_api_route(path, handler, methods=["POST"])
        elif method == "GET":
            app.add_api_route(path, handler, methods=["GET"])

    @app.get("/health")
    async def health():
        return {"status": "ok", "node": "robot_bridge_node"}

    @app.get("/api/tasks/{goal_id}")
    async def task_status(goal_id: str):
        state = ros_node.action_state(goal_id)
        if state is None:
            raise HTTPException(status_code=404, detail="unknown action goal")
        return {"goal_id": goal_id, **state}

    @app.post("/api/tasks/{goal_id}/cancel")
    async def cancel_task(goal_id: str):
        if not ros_node.cancel_action(goal_id):
            raise HTTPException(
                status_code=404, detail="action goal is not active or cancel failed")
        return {"success": True, "goal_id": goal_id}

    return app


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)

    ROBOT_ID = int(os.getenv("ROBOT_ID", "13"))  # 本地机器ID
    CONTROL_URL = os.getenv("CONTROL_URL", "http://192.168.124.103:8080")
    # 临时节点，只为读取 ROS parameters
    _tmp = Node("_robot_bridge_param_reader")
    _tmp.declare_parameter("config_path", "")
    _tmp.declare_parameter("robot_id", ROBOT_ID)
    _tmp.declare_parameter("control_url", CONTROL_URL)

    config_path = _tmp.get_parameter(
        "config_path").get_parameter_value().string_value
    robot_id = _tmp.get_parameter(
        "robot_id").get_parameter_value().integer_value
    control_url = _tmp.get_parameter(
        "control_url").get_parameter_value().string_value
    _tmp.destroy_node()

    # 默认 config 路径
    if not config_path:
        _share = pathlib.Path(__file__).parent.parent / \
            "config" / "bridge_config.yaml"
        config_path = str(_share)

    config = _load_config(config_path, robot_id, control_url)
    config.setdefault("robot_id", robot_id)
    config.setdefault("control_url", control_url)

    ros_node = RobotBridgeNode(config)
    app = build_fastapi_app(ros_node, config)

    http_cfg = config.get("http", {})
    host = http_cfg.get("host", "0.0.0.0")
    port = int(http_cfg.get("port", 8000))

    # ROS spin 在独立线程
    ros_thread = threading.Thread(
        target=rclpy.spin, args=(
            ros_node,), daemon=True)
    ros_thread.start()

    ros_node.get_logger().info(
        f"robot_bridge HTTP server starting on {host}:{port}")
    try:
        uvicorn.run(app, host=host, port=port, log_level="info")
    finally:
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join(timeout=2.0)
        ros_node.destroy_node()


if __name__ == "__main__":
    main()
