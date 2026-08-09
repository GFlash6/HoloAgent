#!/usr/bin/env python3
"""
G1chat ROS2 节点：LLM 解析文本/语音指令 -> 生成多机 DAG -> 按依赖执行。

设计目标：
- 输入来源仍然复用 G1Chat，支持文本指令和语音识别后的 user 文本
- 使用大模型将自然语言拆解为多机器人 DAG
- DAG 节点支持按 robot_id、skill、target、depends_on 描述执行依赖
- 调度器按拓扑依赖并发执行多机任务
- 执行链路对接 multi_robot_ctl / robot_bridge 暴露的 HTTP 接口

多机技能调用参考：
- /workspace/r1_ws/agentic_robot_system-refactor-agent_reorg/agentic_robot/services/src/multi_robot_ctl/README.md
- /workspace/r1_ws/agentic_robot_system-refactor-agent_reorg/agentic_robot/services/src/robot_bridge/config/bridge_config.yaml

测试指令：
- “11机器和12机器同时去点位1，11机器到达后高挥手打招呼，12机器到达后和用户击掌，全部完成后同时挥手再见，然后回到点位2”
"""

import asyncio
import json
import os
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path
from queue import Empty, Queue
from typing import Dict, List, Optional, Set

import requests
import rclpy
import yaml
from openai import DefaultHttpxClient, OpenAI
from rclpy.node import Node
from std_msgs.msg import String

from g1 import G1Chat


MULTIROBOT_DAG_TEST_CASES = [
    {
        "name": "two_robot_parallel_nav_then_actions_then_return",
        "instruction": "11机器和12机器同时去点位1，11机器到达后高挥手打招呼，12机器到达后和用户击掌，全部完成后同时挥手再见，然后回到点位2",
        "expected_dag": {
            "description": "11/12 并行到点位1；11 到达后执行 wave_above_head；12 到达后执行 high_five；两者都完成后并行执行挥手再见；最后两机并行回点位2",
            "nodes": [
                {
                    "id": "r11_nav_p1",
                    "robot_id": 11,
                    "skill": "navigation",
                    "target": "one_point_1",
                    "depends_on": [],
                },
                {
                    "id": "r12_nav_p1",
                    "robot_id": 12,
                    "skill": "navigation",
                    "target": "one_point_1",
                    "depends_on": [],
                },
                {
                    "id": "r11_wave_greet",
                    "robot_id": 11,
                    "skill": "arm",
                    "target": "wave_above_head",
                    "depends_on": ["r11_nav_p1"],
                },
                {
                    "id": "r12_high_five",
                    "robot_id": 12,
                    "skill": "arm",
                    "target": "high_five",
                    "depends_on": ["r12_nav_p1"],
                },
                {
                    "id": "r11_wave_bye",
                    "robot_id": 11,
                    "skill": "arm",
                    "target": "wave_under_head",
                    "depends_on": ["r11_wave_greet", "r12_high_five"],
                },
                {
                    "id": "r12_wave_bye",
                    "robot_id": 12,
                    "skill": "arm",
                    "target": "wave_under_head",
                    "depends_on": ["r11_wave_greet", "r12_high_five"],
                },
                {
                    "id": "r11_nav_p2",
                    "robot_id": 11,
                    "skill": "navigation",
                    "target": "one_point_2",
                    "depends_on": ["r11_wave_bye", "r12_wave_bye"],
                },
                {
                    "id": "r12_nav_p2",
                    "robot_id": 12,
                    "skill": "navigation",
                    "target": "one_point_2",
                    "depends_on": ["r11_wave_bye", "r12_wave_bye"],
                },
            ],
        },
    }
]


class G1ChatNode(Node):
    """封装 G1Chat 的 ROS2 节点，并负责多机 DAG 规划与执行。"""

    _NAV_FINISH_SIGNAL = "nav_finish"
    _UNRECOGNIZED_COMMAND_SIGNAL = "unrecognized_command"
    _DAG_EXECUTOR_MAX_WORKERS = 8
    _HTTP_TIMEOUT_SEC = 10.0
    _NAV_WAIT_TIMEOUT_SEC = 120.0
    _ARM_SKILL_WAIT_SEC = 8.0
    _CONTROL_CENTER_TIMEOUT_SEC = 5.0

    def __init__(self) -> None:
        super().__init__("g1chat_multirobot_dag_node")

        self._chat = G1Chat()

        self._qa_pub = self.create_publisher(String, "chat_qa_pub", 10)
        self._location_pub = self.create_publisher(String, "chat_loc_pub", 10)
        self._signal_pub = self.create_publisher(String, "chat_signal_pub", 10)

        self._signal_sub = self.create_subscription(
            String, "waypoint_reached", self.waypoint_callback, 10
        )

        self._task_queue = Queue()
        self._nav_finish_events: Dict[int, threading.Event] = {}
        self._active_navigation_targets: Dict[int, str] = {}
        self._nav_state_lock = threading.Lock()

        self._task_memory_root = Path(
            os.getenv(
                "AGENTOS_TASK_MEMORY_ROOT",
                "/workspace/D-Robotics/agentic_robot_system/agentic_robot/agentOS/task_memory",
            )
        )
        self._task_memory_root.mkdir(parents=True, exist_ok=True)
        self._history_root = self._task_memory_root / "history"
        self._history_root.mkdir(parents=True, exist_ok=True)
        self._session_datetime = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._session_dir = self._task_memory_root / self._session_datetime
        self._session_dir.mkdir(parents=True, exist_ok=True)
        self._session_progress_path = self._session_dir / "session_progress.yaml"
        self._session_summary_path = self._session_dir / "session_summary.yaml"
        self._history_index_path = self._history_root / "history_index.yaml"
        self._session_lock = threading.Lock()
        self._command_counter = 0
        self._current_session_progress = {
            "session_datetime": self._session_datetime,
            "session_dir": str(self._session_dir),
            "created_at": datetime.now().isoformat(),
            "status": "initialized",
            "active_command_index": None,
            "commands": [],
        }

        self._robot_urls = {
            11: "http://192.168.124.101:8000",
            12: "http://192.168.124.102:8000",
            13: "http://192.168.124.103:8000",
            14: "http://192.168.124.104:8000",
            15: "http://192.168.124.105:8000",
            16: "http://192.168.124.106:8000",
        }
        self._control_center_url = os.getenv(
            "MULTI_ROBOT_CONTROL_CENTER_URL", "http://127.0.0.1:8080"
        ).rstrip("/")

        qwen_api_key = os.getenv("QWEN_API_KEY")
        if not qwen_api_key:
            raise RuntimeError("未配置 QWEN_API_KEY")
        self.gpt_model = os.getenv("QWEN_MODEL", "qwen3.7-plus")
        self.client = OpenAI(
            api_key=qwen_api_key,
            base_url=os.getenv(
                "QWEN_BASE_URL",
                "https://dashscope.aliyuncs.com/compatible-mode/v1",
            ),
            http_client=DefaultHttpxClient(
                proxy=os.getenv("HTTPS_PROXY") or os.getenv("https_proxy"),
                trust_env=False,
            ),
        )

        self._supported_navigation_targets = {
            "one_point_1",
            "one_point_2",
            "one_point_3",
            "one_point_4",
            "stop",
            # 这里可以添加更多支持的导航目标
        }

        self._supported_arm_targets = {
            "release_arm",
            "turn_back_wave",
            "blow_kiss_with_both_hands",
            "blow_kiss_with_left_hand",
            "blow_kiss_with_right_hand",
            "both_hands_up",
            "clamp",
            "high_five",
            "hug",
            "make_heart_with_both_hands",
            "make_heart_with_right_hand",
            "refuse",
            "right_hand_up",
            "ultraman_ray",
            "wave_under_head",
            "wave_above_head",
            "shake_hand",
            "one_point_1_waypoint_1",
            "box_left_hand_win",
            "box_right_hand_win",
            "box_both_hand_win",
            "right_hand_on_heart",
            "both_hands_up_deviate_right",
            "forward_push",
        }

        self._chat_thread = threading.Thread(
            target=self._run_chat_loop, daemon=True)
        self._chat_thread.start()

        self._bridge_thread = threading.Thread(
            target=self._bridge_text_queue, daemon=True
        )
        self._bridge_thread.start()

        self._task_worker_thread = threading.Thread(
            target=self._task_worker, daemon=True
        )
        self._task_worker_thread.start()

        self._write_session_summary()
        self._write_session_progress()
        self._append_history_index_entry(
            {
                "session_datetime": self._session_datetime,
                "session_dir": str(self._session_dir),
                "created_at": datetime.now().isoformat(),
                "status": "initialized",
                "result": "running",
            }
        )

        self.get_logger().info("g1chat 多机 DAG 节点已启动")
        self.get_logger().info(
            "已加载多机 DAG 测试用例: "
            + json.dumps(MULTIROBOT_DAG_TEST_CASES, ensure_ascii=False)
        )
        self.get_logger().info(f"task_memory 会话目录: {self._session_dir}")

    # -------------------- G1Chat 启动 --------------------

    def _run_chat_loop(self) -> None:
        """在独立线程中运行 G1Chat。"""

        async def _main():
            await self._chat.start()
            try:
                await asyncio.Future()
            except asyncio.CancelledError:
                pass
            finally:
                await self._chat.stop()

        asyncio.run(_main())

    # -------------------- temporal memory --------------------

    def _write_yaml_file(self, path: Path, data: dict) -> None:
        """将 YAML 数据写入文件。"""
        path.write_text(
            yaml.safe_dump(
                data,
                allow_unicode=True,
                sort_keys=False,
            ),
            encoding="utf-8",
        )

    def _read_yaml_file(self, path: Path) -> dict:
        """读取 YAML 文件，不存在时返回空字典。"""
        if not path.exists():
            return {}
        content = yaml.safe_load(path.read_text(encoding="utf-8"))
        return content if isinstance(content, dict) else {}

    def _append_session_event(self, event_type: str, payload: dict) -> None:
        """追加执行事件到 session_progress.yaml。"""
        event = {
            "timestamp": datetime.now().isoformat(),
            "event_type": event_type,
            "payload": payload,
        }
        with self._session_lock:
            events = self._current_session_progress.setdefault("events", [])
            events.append(event)
            self._write_session_progress()

    def _write_session_progress(self) -> None:
        """写入当前 session 的 DAG 执行进度。"""
        self._write_yaml_file(self._session_progress_path,
                              self._current_session_progress)

    def _write_session_summary(self) -> None:
        """写入当前 session 的概要信息。"""
        summary = {
            "session_datetime": self._session_datetime,
            "session_dir": str(self._session_dir),
            "created_at": datetime.now().isoformat(),
            "description": "LLM 多机 DAG 执行 task memory",
            "files": {
                "session_progress": str(self._session_progress_path),
                "history_index": str(self._history_index_path),
            },
            "control_center_url": self._control_center_url,
        }
        self._write_yaml_file(self._session_summary_path, summary)

    def _append_history_index_entry(self, entry: dict) -> None:
        """更新历史 session 索引。"""
        with self._session_lock:
            history_index = self._read_yaml_file(self._history_index_path)
            sessions = history_index.setdefault("sessions", [])
            sessions = [
                item
                for item in sessions
                if item.get("session_datetime") != entry.get("session_datetime")
            ]
            sessions.append(entry)
            history_index["sessions"] = sessions
            self._write_yaml_file(self._history_index_path, history_index)

    def _update_history_index_status(self, status: str, result: str) -> None:
        """更新当前 session 在历史索引中的状态。"""
        self._append_history_index_entry(
            {
                "session_datetime": self._session_datetime,
                "session_dir": str(self._session_dir),
                "updated_at": datetime.now().isoformat(),
                "status": status,
                "result": result,
            }
        )

    def _create_command_record_dir(self, user_text: str) -> tuple[int, Path]:
        """为单次用户指令创建记录目录。"""
        with self._session_lock:
            self._command_counter += 1
            command_index = self._command_counter

        command_dir = self._session_dir / f"command_{command_index:03d}"
        command_dir.mkdir(parents=True, exist_ok=True)

        instruction_payload = {
            "session_datetime": self._session_datetime,
            "command_index": command_index,
            "recorded_at": datetime.now().isoformat(),
            "instruction": user_text,
        }
        self._write_yaml_file(
            command_dir / "instruction.yaml",
            instruction_payload)
        with self._session_lock:
            self._current_session_progress["status"] = "planning"
            self._current_session_progress["active_command_index"] = command_index
            commands = self._current_session_progress.setdefault(
                "commands", [])
            commands.append(
                {
                    "command_index": command_index,
                    "instruction": user_text,
                    "command_dir": str(command_dir),
                    "status": "received",
                    "result": "pending",
                    "dag_node_count": 0,
                    "sandbox": {"status": "pending", "result": "pending"},
                    "execution": {"status": "pending", "result": "pending"},
                }
            )
            self._write_session_progress()
        self._append_session_event(
            "instruction_received",
            {
                "command_index": command_index,
                "instruction": user_text,
                "command_dir": str(command_dir),
            },
        )
        return command_index, command_dir

    def _record_dag_artifacts(
        self,
        command_index: int,
        command_dir: Path,
        user_text: str,
        dag: Optional[dict],
    ) -> None:
        """记录 DAG 规划结果到 temporal memory。"""
        dag_payload = {
            "session_datetime": self._session_datetime,
            "command_index": command_index,
            "instruction": user_text,
            "recorded_at": datetime.now().isoformat(),
            "dag": dag,
        }
        self._write_yaml_file(command_dir / "dag.yaml", dag_payload)
        self._update_command_progress(
            command_index,
            status="planned" if dag else "planning_failed",
            result="pending" if dag else "failed",
            dag_node_count=len(dag.get("nodes", [])) if dag else 0,
            dag_description=dag.get("description", "") if dag else "",
        )
        self._append_session_event(
            "dag_generated",
            {
                "command_index": command_index,
                "command_dir": str(command_dir),
                "dag_node_count": len(dag.get("nodes", [])) if dag else 0,
                "dag_description": dag.get("description", "") if dag else "",
            },
        )

    # -------------------- LLM DAG 规划 --------------------

    def _analyze_user_command(self, user_text: str) -> Optional[dict]:
        """使用 Qwen 将用户指令拆解为多机 DAG。"""
        system_prompt = f"""
你是一个多机器人任务规划器。你的职责是把用户自然语言指令转换为一个可执行 DAG(JSON)。

你只能输出合法 JSON，不要输出 markdown，不要输出解释，不要输出代码块。

输出 JSON 格式固定为：
{{
  "description": "任务描述",
  "nodes": [
    {{
      "id": "r11_nav_p1",
      "robot_id": 11,
      "skill": "navigation",
      "target": "one_point_1",
      "depends_on": []
    }},
    {{
      "id": "r11_wave",
      "robot_id": 11,
      "skill": "arm",
      "target": "wave_above_head",
      "depends_on": ["r11_nav_p1"]
    }}
  ]
}}

字段约束：
- id: 字符串，必须唯一
- robot_id: 整数，表示机器人编号，例如 11、12
- skill: 只能是以下之一：
  - "navigation"
  - "arm"
- target:
  - 当 skill="navigation" 时，只能是以下之一：
    {sorted(self._supported_navigation_targets)}
  - 当 skill="arm" 时，只能是以下之一：
    {sorted(self._supported_arm_targets)}
- depends_on: 字符串数组，表示当前节点依赖的前置节点 id
- 没有依赖时必须输出空数组 []

规划原则：
- 用户说“同时”时，相关节点应并行，不能互相依赖
- 用户说“到达后”“完成后”“然后”时，应正确建立 depends_on
- 不要凭空增加用户未提及的动作
- 如果用户要求无法可靠映射到上述 skill/target，输出：
  {{"description": "unrecognized", "nodes": []}}

多机 DAG 测试样例：
{json.dumps(MULTIROBOT_DAG_TEST_CASES, ensure_ascii=False, indent=2)}
"""

        try:
            response = self.client.responses.create(
                model=self.gpt_model,
                input=[
                    {"role": "system", "content": system_prompt},
                    {"role": "user", "content": user_text},
                ],
            )

            content = ""
            if hasattr(response, "output_text") and response.output_text:
                content = response.output_text.strip()
            elif hasattr(response, "output") and response.output:
                for item in response.output:
                    if getattr(item, "type", "") != "message":
                        continue
                    for part in getattr(item, "content", []):
                        text_value = getattr(part, "text", None)
                        if text_value:
                            content += text_value

            if not content:
                self.get_logger().warning("Qwen 返回内容为空。")
                return None

            dag = json.loads(content)
            return self._validate_and_normalize_dag(dag)
        except Exception as exc:
            self.get_logger().error(f"Qwen DAG 规划失败: {exc}")
            return None

    def _validate_and_normalize_dag(self, dag: dict) -> Optional[dict]:
        """校验并规范化 DAG。"""
        if not isinstance(dag, dict):
            return None

        description = str(dag.get("description", "")).strip()
        nodes = dag.get("nodes", [])
        if not isinstance(nodes, list):
            return None

        normalized_nodes: List[dict] = []
        node_ids: Set[str] = set()

        for node in nodes:
            if not isinstance(node, dict):
                continue

            node_id = str(node.get("id", "")).strip()
            if not node_id or node_id in node_ids:
                self.get_logger().warning(f"DAG 节点 id 非法或重复: {node_id}")
                return None

            try:
                robot_id = int(node.get("robot_id"))
            except Exception:
                self.get_logger().warning(f"DAG 节点 robot_id 非法: {node}")
                return None

            skill = str(node.get("skill", "")).strip()
            target = str(node.get("target", "")).strip()
            depends_on = node.get("depends_on", [])

            if robot_id not in self._robot_urls:
                self.get_logger().warning(f"不支持的 robot_id: {robot_id}")
                return None

            if skill not in {"navigation", "arm"}:
                self.get_logger().warning(f"不支持的 skill: {skill}")
                return None

            if skill == "navigation" and target not in self._supported_navigation_targets:
                self.get_logger().warning(f"不支持的 navigation target: {target}")
                return None

            if skill == "arm" and target not in self._supported_arm_targets:
                self.get_logger().warning(f"不支持的 arm target: {target}")
                return None

            if not isinstance(depends_on, list):
                self.get_logger().warning(f"depends_on 必须为数组: {node}")
                return None

            normalized_depends_on = [str(dep).strip()
                                     for dep in depends_on if str(dep).strip()]

            normalized_nodes.append(
                {
                    "id": node_id,
                    "robot_id": robot_id,
                    "skill": skill,
                    "target": target,
                    "depends_on": normalized_depends_on,
                }
            )
            node_ids.add(node_id)

        for node in normalized_nodes:
            for dep in node["depends_on"]:
                if dep not in node_ids:
                    self.get_logger().warning(
                        f"DAG 节点依赖不存在: node={node['id']} dep={dep}"
                    )
                    return None

        if normalized_nodes and self._has_cycle(normalized_nodes):
            self.get_logger().warning("DAG 存在循环依赖")
            return None

        return {
            "description": description or "multirobot_dag",
            "nodes": normalized_nodes,
        }

    def _has_cycle(self, nodes: List[dict]) -> bool:
        """检测 DAG 是否存在环。"""
        graph = {node["id"]: list(node["depends_on"]) for node in nodes}
        visiting = set()
        visited = set()

        def dfs(node_id: str) -> bool:
            if node_id in visiting:
                return True
            if node_id in visited:
                return False

            visiting.add(node_id)
            for dep in graph.get(node_id, []):
                if dfs(dep):
                    return True
            visiting.remove(node_id)
            visited.add(node_id)
            return False

        return any(dfs(node_id) for node_id in graph)

    def _update_command_progress(self, command_index: int, **updates) -> None:
        """更新当前 session 中某条命令的进度。"""
        with self._session_lock:
            for command in self._current_session_progress.get("commands", []):
                if command.get("command_index") == command_index:
                    command.update(updates)
                    break
            self._write_session_progress()

    def _run_dag_sandbox_self_test(
        self,
        dag: Optional[dict],
        command_index: int,
        command_dir: Path,
    ) -> bool:
        """在真实物理执行前，对 DAG 的同步与协作顺序做沙盒自测。"""
        sandbox_result = {
            "command_index": command_index,
            "recorded_at": datetime.now().isoformat(),
            "status": "passed",
            "checks": [],
        }

        if not dag or not dag.get("nodes"):
            sandbox_result["status"] = "failed"
            sandbox_result["checks"].append(
                {
                    "name": "dag_non_empty",
                    "passed": False,
                    "reason": "DAG 为空或无法识别",
                }
            )
            self._write_yaml_file(
                command_dir / "sandbox_result.yaml", sandbox_result)
            self._update_command_progress(
                command_index,
                sandbox={"status": "failed", "result": "failed"},
            )
            return False

        nodes = dag["nodes"]
        node_ids = {node["id"] for node in nodes}
        robot_busy_nodes: Dict[int, List[str]] = {}
        for node in nodes:
            robot_busy_nodes.setdefault(
                node["robot_id"], []).append(node["id"])

        for node in nodes:
            missing_deps = [dep for dep in node["depends_on"]
                            if dep not in node_ids]
            sandbox_result["checks"].append(
                {
                    "name": f"deps_exist::{node['id']}",
                    "passed": not missing_deps,
                    "missing_dependencies": missing_deps,
                }
            )
            if missing_deps:
                sandbox_result["status"] = "failed"

        for robot_id, robot_nodes in robot_busy_nodes.items():
            for node in nodes:
                if node["robot_id"] != robot_id:
                    continue
                conflicting = [
                    other["id"]
                    for other in nodes
                    if other["id"] != node["id"]
                    and other["robot_id"] == robot_id
                    and node["id"] not in other["depends_on"]
                    and other["id"] not in node["depends_on"]
                ]
                sandbox_result["checks"].append(
                    {
                        "name": f"single_robot_serialization::{node['id']}",
                        "passed": not conflicting,
                        "conflicting_nodes": conflicting,
                    }
                )
                if conflicting:
                    sandbox_result["status"] = "failed"

        self._write_yaml_file(
            command_dir / "sandbox_result.yaml", sandbox_result)
        self._update_command_progress(
            command_index,
            sandbox={
                "status": "passed" if sandbox_result["status"] == "passed" else "failed",
                "result": sandbox_result["status"],
            },
        )
        self._append_session_event(
            "dag_sandbox_self_test_completed",
            {
                "command_index": command_index,
                "status": sandbox_result["status"],
                "command_dir": str(command_dir),
            },
        )
        return sandbox_result["status"] == "passed"

    def _reset_control_center_reached_state(self, robot_ids: List[int]) -> None:
        """在新一轮导航前重置控制中心的到点状态。"""
        try:
            requests.post(
                f"{self._control_center_url}/reset_reached",
                json={"robot_ids": robot_ids},
                timeout=self._CONTROL_CENTER_TIMEOUT_SEC,
            )
        except Exception as exc:
            self.get_logger().warning(f"重置控制中心到点状态失败: {exc}")

    def _wait_for_robot_navigation_completion(self, robot_id: int, target: str) -> bool:
        """优先通过控制中心按 robot_id 轮询导航完成状态，必要时回退到本地事件。"""
        nav_finish_event = self._nav_finish_events.setdefault(
            robot_id, threading.Event())
        deadline = time.time() + self._NAV_WAIT_TIMEOUT_SEC

        while time.time() < deadline:
            try:
                response = requests.get(
                    f"{self._control_center_url}/robot_reached/{robot_id}",
                    params={"target": target},
                    timeout=self._CONTROL_CENTER_TIMEOUT_SEC,
                )
                if response.ok:
                    payload = response.json()
                    if payload.get("reached"):
                        nav_finish_event.set()
                        return True
            except Exception as exc:
                self.get_logger().warning(f"查询控制中心导航状态失败，回退到本地事件等待: {exc}")
                break

            if nav_finish_event.wait(timeout=0.5):
                return True

        remaining_timeout = max(0.0, deadline - time.time())
        return nav_finish_event.wait(timeout=remaining_timeout)

    # -------------------- DAG 执行 --------------------

    def _execute_dag(
        self,
        dag: Optional[dict],
        command_index: Optional[int] = None,
        command_dir: Optional[Path] = None,
    ) -> None:
        """按依赖关系执行多机 DAG。"""
        if not dag or not dag.get("nodes"):
            if command_index is not None:
                self._update_command_progress(
                    command_index,
                    status="unrecognized",
                    result="failed",
                    execution={"status": "not_started", "result": "failed"},
                )
            self._update_history_index_status("unrecognized", "failed")
            self._append_session_event(
                "dag_empty_or_unrecognized",
                {
                    "command_index": command_index,
                    "command_dir": str(command_dir) if command_dir else None,
                },
            )
            self._publish_unrecognized_command()
            return

        nodes = dag["nodes"]
        node_map = {node["id"]: node for node in nodes}
        pending = set(node_map.keys())
        completed = set()
        failed = set()

        self.get_logger().info(
            "开始执行多机 DAG: " + json.dumps(dag, ensure_ascii=False)
        )
        self._append_session_event(
            "dag_execution_started",
            {
                "command_index": command_index,
                "command_dir": str(command_dir) if command_dir else None,
                "dag_description": dag.get("description", ""),
                "pending_nodes": sorted(pending),
            },
        )

        with ThreadPoolExecutor(max_workers=self._DAG_EXECUTOR_MAX_WORKERS) as executor:
            while pending:
                ready_nodes = [
                    node_map[node_id]
                    for node_id in sorted(pending)
                    if all(dep in completed for dep in node_map[node_id]["depends_on"])
                ]

                if not ready_nodes:
                    self.get_logger().error(
                        f"DAG 无可执行节点，剩余节点可能存在非法依赖: {sorted(pending)}"
                    )
                    self._append_session_event(
                        "dag_execution_blocked",
                        {
                            "command_index": command_index,
                            "remaining_nodes": sorted(pending),
                        },
                    )
                    if command_index is not None:
                        self._update_command_progress(
                            command_index,
                            status="blocked",
                            result="failed",
                            execution={"status": "blocked",
                                       "result": "failed"},
                        )
                    self._update_history_index_status("blocked", "failed")
                    return

                future_to_node_id = {
                    executor.submit(self._execute_single_node, node): node["id"]
                    for node in ready_nodes
                }

                for node in ready_nodes:
                    pending.remove(node["id"])

                for future, node_id in future_to_node_id.items():
                    try:
                        success = future.result()
                    except Exception as exc:
                        self.get_logger().error(f"DAG 节点执行异常 {node_id}: {exc}")
                        success = False

                    if success:
                        completed.add(node_id)
                        self._append_session_event(
                            "dag_node_completed",
                            {
                                "command_index": command_index,
                                "node_id": node_id,
                                "completed_nodes": sorted(completed),
                            },
                        )
                    else:
                        failed.add(node_id)
                        self._append_session_event(
                            "dag_node_failed",
                            {
                                "command_index": command_index,
                                "node_id": node_id,
                                "failed_nodes": sorted(failed),
                            },
                        )

                if failed:
                    self.get_logger().error(
                        f"DAG 执行失败，失败节点: {sorted(failed)}，已停止后续调度"
                    )
                    if command_index is not None:
                        self._update_command_progress(
                            command_index,
                            status="failed",
                            result="failed",
                            execution={"status": "failed", "result": "failed"},
                        )
                    self._update_history_index_status("failed", "failed")
                    self._append_session_event(
                        "dag_execution_failed",
                        {
                            "command_index": command_index,
                            "failed_nodes": sorted(failed),
                            "completed_nodes": sorted(completed),
                        },
                    )
                    return

        self.get_logger().info("多机 DAG 执行完成")
        if command_index is not None:
            self._update_command_progress(
                command_index,
                status="completed",
                result="success",
                execution={"status": "completed", "result": "success"},
            )
        self._update_history_index_status("completed", "success")
        self._append_session_event(
            "dag_execution_completed",
            {
                "command_index": command_index,
                "completed_nodes": sorted(completed),
                "command_dir": str(command_dir) if command_dir else None,
            },
        )

    def _execute_single_node(self, node: dict) -> bool:
        """执行单个 DAG 节点。"""
        robot_id = node["robot_id"]
        skill = node["skill"]
        target = node["target"]
        node_id = node["id"]

        self.get_logger().info(
            f"执行 DAG 节点: id={node_id}, robot_id={robot_id}, skill={skill}, target={target}"
        )
        self._append_session_event(
            "dag_node_started",
            {
                "node_id": node_id,
                "robot_id": robot_id,
                "skill": skill,
                "target": target,
            },
        )

        if skill == "navigation":
            return self._call_robot_navigation(robot_id, target)

        if skill == "arm":
            return self._call_robot_arm(robot_id, target)

        self.get_logger().warning(f"未知 skill，忽略: {skill}")
        return False

    def _call_robot_navigation(self, robot_id: int, target: str) -> bool:
        """调用 robot_bridge 导航接口，并等待该机器人对应的 nav_finish。"""
        with self._nav_state_lock:
            nav_finish_event = self._nav_finish_events.setdefault(
                robot_id, threading.Event())
            nav_finish_event.clear()
            self._active_navigation_targets[robot_id] = target
        self._reset_control_center_reached_state([robot_id])

        base_url = self._robot_urls[robot_id]
        url = f"{base_url}/api/navigation/{target}"
        accepted = self._post_json(url)
        if not accepted:
            with self._nav_state_lock:
                self._active_navigation_targets.pop(robot_id, None)
            self._append_session_event(
                "navigation_dispatch_failed",
                {
                    "robot_id": robot_id,
                    "target": target,
                    "url": url,
                },
            )
            return False

        self._append_session_event(
            "navigation_dispatched",
            {
                "robot_id": robot_id,
                "target": target,
                "url": url,
                "wait_signal": self._NAV_FINISH_SIGNAL,
                "timeout_sec": self._NAV_WAIT_TIMEOUT_SEC,
            },
        )

        try:
            finished = self._wait_for_robot_navigation_completion(
                robot_id, target)
            if not finished:
                self.get_logger().warning(
                    f"等待机器人 {robot_id} 导航完成超时（{self._NAV_WAIT_TIMEOUT_SEC} 秒）: {target}"
                )
                self._append_session_event(
                    "navigation_wait_timeout",
                    {
                        "robot_id": robot_id,
                        "target": target,
                        "timeout_sec": self._NAV_WAIT_TIMEOUT_SEC,
                    },
                )
                return False

            self._append_session_event(
                "navigation_finished",
                {
                    "robot_id": robot_id,
                    "target": target,
                    "signal": self._NAV_FINISH_SIGNAL,
                },
            )
            return True
        finally:
            with self._nav_state_lock:
                self._active_navigation_targets.pop(robot_id, None)
                nav_finish_event.clear()

    def _call_robot_arm(self, robot_id: int, target: str) -> bool:
        """调用 robot_bridge 动作接口。"""
        base_url = self._robot_urls[robot_id]
        url = f"{base_url}/api/arm/{target}"
        ok = self._post_json(url)
        if ok:
            time.sleep(self._ARM_SKILL_WAIT_SEC)
        return ok

    def _post_json(self, url: str, payload: Optional[dict] = None) -> bool:
        """发送 HTTP POST 请求。"""
        try:
            response = requests.post(
                url, json=payload, timeout=self._HTTP_TIMEOUT_SEC)
            if response.ok:
                self.get_logger().info(f"HTTP POST 成功: {url}")
                return True

            self.get_logger().warning(
                f"HTTP POST 失败: url={url}, status={response.status_code}, body={response.text}"
            )
            return False
        except Exception as exc:
            self.get_logger().error(f"HTTP POST 异常: url={url}, error={exc}")
            return False

    # -------------------- 其它辅助 --------------------

    def _publish_unrecognized_command(self) -> None:
        """反馈无法识别指令。"""
        msg = String()
        msg.data = self._UNRECOGNIZED_COMMAND_SIGNAL
        self._signal_pub.publish(msg)
        self.get_logger().warning(f"无法识别用户指令，已发布信号: {msg.data}")

    def waypoint_callback(self, msg: String) -> None:
        """订阅 waypoint_reached，并在收到 nav_finish 时释放当前正在导航的机器人等待。"""
        received_data = msg.data.strip()
        if not received_data:
            return

        self._append_session_event(
            "waypoint_reached_signal",
            {
                "signal": received_data,
            },
        )

        if received_data != self._NAV_FINISH_SIGNAL:
            return

        with self._nav_state_lock:
            active_robot_ids = list(self._active_navigation_targets.keys())
            if len(active_robot_ids) != 1:
                self.get_logger().warning(
                    "收到 nav_finish，但当前活跃导航机器人数量不是 1，优先依赖控制中心 robot_id 状态归属: "
                    f"{active_robot_ids}"
                )
                self._append_session_event(
                    "navigation_finish_signal_ambiguous",
                    {
                        "signal": received_data,
                        "active_robot_ids": active_robot_ids,
                    },
                )
                return

            robot_id = active_robot_ids[0]
            nav_finish_event = self._nav_finish_events.get(robot_id)
            if nav_finish_event is None:
                return

            nav_finish_event.set()
            self._append_session_event(
                "navigation_finish_event_set",
                {
                    "robot_id": robot_id,
                    "target": self._active_navigation_targets.get(robot_id),
                    "signal": received_data,
                },
            )

    def _is_wake_word_only(self, user_text: str) -> bool:
        """激活词不进入任务拆解，也不输出无法识别。"""
        normalized_text = (
            user_text.strip()
            .replace(" ", "")
            .replace("。", "")
            .replace("，", "")
            .replace(",", "")
            .replace(".", "")
            .replace("！", "")
            .replace("!", "")
            .replace("？", "")
            .replace("?", "")
        )
        return normalized_text in {"地瓜地瓜"}

    def _enqueue_user_command(self, user_text: str) -> None:
        """将用户文本加入串行任务队列。"""
        self._task_queue.put(user_text)

    def _task_worker(self) -> None:
        """串行处理用户指令：先规划 DAG，再执行 DAG。"""
        while rclpy.ok():
            try:
                user_text = self._task_queue.get(timeout=0.1)
            except Empty:
                continue

            if not isinstance(user_text, str) or not user_text.strip():
                continue

            user_text = user_text.strip()
            if self._is_wake_word_only(user_text):
                continue

            command_index, command_dir = self._create_command_record_dir(
                user_text)
            dag = self._analyze_user_command(user_text)
            self._record_dag_artifacts(
                command_index, command_dir, user_text, dag)

            if dag and len(dag.get("nodes", [])) >= 2:
                sandbox_ok = self._run_dag_sandbox_self_test(
                    dag, command_index, command_dir
                )
                if not sandbox_ok:
                    self._update_command_progress(
                        command_index,
                        status="sandbox_failed",
                        result="failed",
                        execution={"status": "blocked",
                                   "result": "not_started"},
                    )
                    self._update_history_index_status(
                        "sandbox_failed", "failed")
                    continue

            self._update_command_progress(
                command_index,
                status="executing",
                result="running",
                execution={"status": "running", "result": "running"},
            )
            self._update_history_index_status("executing", "running")
            self._execute_dag(
                dag,
                command_index=command_index,
                command_dir=command_dir)

    # -------------------- ROS <-> G1Chat 桥接 --------------------

    def _bridge_text_queue(self) -> None:
        """持续从 G1Chat.text_queue 取数据并发布到对应 topic。"""
        q = self._chat.text_queue

        while rclpy.ok():
            try:
                item = q.get(timeout=0.1)
            except Empty:
                continue

            if not isinstance(item, str) or not item:
                continue

            if item.startswith("user:") or item.startswith("assistant:"):
                msg = String()
                msg.data = item
                self._qa_pub.publish(msg)

                if item.startswith("user:"):
                    user_text = item[len("user:"):].strip()
                    if user_text:
                        self._enqueue_user_command(user_text)
                continue

            if item.startswith("location:"):
                msg = String()
                msg.data = item[len("location:"):].strip()
                self._location_pub.publish(msg)
                continue

            msg = String()
            if item.startswith("signal:"):
                msg.data = item[len("signal:"):].strip()
            else:
                msg.data = item.strip()
            self._signal_pub.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = G1ChatNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
