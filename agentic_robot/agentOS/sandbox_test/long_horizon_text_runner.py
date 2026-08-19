#!/usr/bin/env python3
"""
纯文本长指令 DAG 执行器。

目标：
- 使用 LLM 将长指令拆解为单机/多机 DAG
- 在真实物理执行前，只做 DAG 依赖与串行约束的静态校验
- 将规划、验证、执行监控结果写入监控文件
"""

from __future__ import annotations

import argparse
import json
import os
import time
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Set

import requests
import yaml
from openai import DefaultHttpxClient, OpenAI


DEFAULT_OUTPUT_ROOT = Path(__file__).resolve().parents[1] / "task_runs"
SKILLS_ROOT = Path(__file__).resolve().parents[1] / "holoagent_skills" / "skills"
EXECUTABLE_SKILLS = ("sem-nav-skill", "rel-move-skill", "arm-skill")

SINGLE_ROBOT_TEST_CASES = [
    {
        "name": "single_robot_long_instruction",
        "instruction": "11机器先去点位1，到了以后高挥手打招呼，然后去点位2和用户击掌，最后回到点位3并挥手再见",
        "expected_dag": {
            "description": "11 机器串行执行导航、挥手、导航、击掌、导航、挥手再见",
            "nodes": [
                {"id": "r11_nav_p1", "robot_id": 11, "skill": "navigation",
                    "target": "one_point_1", "depends_on": []},
                {"id": "r11_wave_greet", "robot_id": 11, "skill": "arm-skill",
                    "target": "wave_above_head", "depends_on": ["r11_nav_p1"]},
                {"id": "r11_nav_p2", "robot_id": 11, "skill": "navigation",
                    "target": "one_point_2", "depends_on": ["r11_wave_greet"]},
                {"id": "r11_high_five", "robot_id": 11, "skill": "arm-skill",
                    "target": "high_five", "depends_on": ["r11_nav_p2"]},
                {"id": "r11_nav_p3", "robot_id": 11, "skill": "navigation",
                    "target": "one_point_3", "depends_on": ["r11_high_five"]},
                {"id": "r11_wave_bye", "robot_id": 11, "skill": "arm-skill",
                    "target": "wave_under_head", "depends_on": ["r11_nav_p3"]},
            ],
        },
    }
]

MULTI_ROBOT_TEST_CASES = [
    {
        "name": "multi_robot_long_instruction",
        "instruction": "11机器和12机器同时去点位1，11机器到达后高挥手打招呼，12机器到达后和用户击掌，全部完成后同时挥手再见，然后回到点位2",
        "expected_dag": {
            "description": "11/12 并行到点位1；11 到达后挥手；12 到达后击掌；全部完成后并行挥手再见；最后并行回点位2",
            "nodes": [
                {"id": "r11_nav_p1", "robot_id": 11, "skill": "navigation",
                    "target": "one_point_1", "depends_on": []},
                {"id": "r12_nav_p1", "robot_id": 12, "skill": "navigation",
                    "target": "one_point_1", "depends_on": []},
                {"id": "r11_wave_greet", "robot_id": 11, "skill": "arm-skill",
                    "target": "wave_above_head", "depends_on": ["r11_nav_p1"]},
                {"id": "r12_high_five", "robot_id": 12, "skill": "arm-skill",
                    "target": "high_five", "depends_on": ["r12_nav_p1"]},
                {"id": "r11_wave_bye", "robot_id": 11, "skill": "arm-skill",
                    "target": "wave_under_head", "depends_on": ["r11_wave_greet", "r12_high_five"]},
                {"id": "r12_wave_bye", "robot_id": 12, "skill": "arm-skill",
                    "target": "wave_under_head", "depends_on": ["r11_wave_greet", "r12_high_five"]},
                {"id": "r11_nav_p2", "robot_id": 11, "skill": "navigation",
                    "target": "one_point_2", "depends_on": ["r11_wave_bye", "r12_wave_bye"]},
                {"id": "r12_nav_p2", "robot_id": 12, "skill": "navigation",
                    "target": "one_point_2", "depends_on": ["r11_wave_bye", "r12_wave_bye"]},
            ],
        },
    }
]


class LongHorizonTextRunner:
    _HTTP_TIMEOUT_SEC = 10.0
    _CONTROL_CENTER_TIMEOUT_SEC = 5.0
    _NAV_WAIT_TIMEOUT_SEC = 120.0
    _DAG_EXECUTOR_MAX_WORKERS = 8

    def __init__(self, mode: str, dry_run: bool = False, output_root: Optional[Path] = None) -> None:
        self.mode = mode
        self.dry_run = dry_run
        self.output_root = output_root or DEFAULT_OUTPUT_ROOT
        self.output_root.mkdir(parents=True, exist_ok=True)

        self.session_datetime = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.session_dir = self.output_root / \
            f"{self.mode}_{self.session_datetime}"
        self.session_dir.mkdir(parents=True, exist_ok=True)

        self.monitor_path = self.session_dir / "monitor.yaml"
        self.plan_path = self.session_dir / "dag_plan.yaml"
        self.dag_validation_path = self.session_dir / "dag_validation.yaml"
        self.execution_path = self.session_dir / "execution_result.yaml"

        self.robot_urls = {
            11: os.getenv("ROBOT_11_URL", "http://192.168.124.101:8000"),
            12: os.getenv("ROBOT_12_URL", "http://192.168.124.102:8000"),
            13: os.getenv("ROBOT_13_URL", "http://192.168.124.103:8000"),
            14: os.getenv("ROBOT_14_URL", "http://192.168.124.104:8000"),
            15: os.getenv("ROBOT_15_URL", "http://192.168.124.105:8000"),
            16: os.getenv("ROBOT_16_URL", "http://192.168.124.106:8000"),
        }
        self.control_center_url = os.getenv(
            "MULTI_ROBOT_CONTROL_CENTER_URL", "http://127.0.0.1:8080"
        ).rstrip("/")
        self.default_robot_id = int(os.getenv("DEFAULT_ROBOT_ID", "13"))
        if self.default_robot_id not in self.robot_urls:
            raise ValueError("DEFAULT_ROBOT_ID must be one of 11/12/13/14/15/16")
        self.skill_catalog = "\n\n".join(
            (SKILLS_ROOT / name / "SKILL.md").read_text(encoding="utf-8")
            for name in EXECUTABLE_SKILLS
        )

        self.supported_navigation_targets = {
            "one_point_1",
            "one_point_2",
            "one_point_3",
            "one_point_4",
            "stop",
        }
        self.supported_arm_targets = {
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

        self.client, self.gpt_model = self._build_llm_client()
        self.monitor = {
            "session_datetime": self.session_datetime,
            "session_dir": str(self.session_dir),
            "mode": self.mode,
            "dry_run": self.dry_run,
            "created_at": datetime.now().isoformat(),
            "status": "initialized",
            "validation_conclusion": "pending",
            "execution_conclusion": "pending",
            "events": [],
            "subtasks": [],
            "artifacts": {
                "dag_plan": str(self.plan_path),
                "dag_validation": str(self.dag_validation_path),
                "execution_result": str(self.execution_path),
            },
        }
        self._write_yaml(self.monitor_path, self.monitor)

    def _build_llm_client(self):
        qwen_api_key = os.getenv("QWEN_API_KEY")
        if not qwen_api_key:
            raise RuntimeError("未配置 QWEN_API_KEY")
        qwen_model = os.getenv("QWEN_MODEL", "qwen3.7-plus")
        qwen_proxy = os.getenv("QWEN_PROXY")
        client = OpenAI(
            api_key=qwen_api_key,
            base_url=os.getenv(
                "QWEN_BASE_URL",
                "https://dashscope.aliyuncs.com/compatible-mode/v1",
            ),
            http_client=DefaultHttpxClient(
                proxy=qwen_proxy,
                trust_env=False,
            ),
        )
        return client, qwen_model

    def _write_yaml(self, path: Path, data: dict) -> None:
        path.write_text(yaml.safe_dump(data, allow_unicode=True,
                        sort_keys=False), encoding="utf-8")

    def _append_event(self, event_type: str, payload: dict) -> None:
        self.monitor["events"].append(
            {
                "timestamp": datetime.now().isoformat(),
                "event_type": event_type,
                "payload": payload,
            }
        )
        self._write_yaml(self.monitor_path, self.monitor)

    def _update_monitor(self, **updates) -> None:
        self.monitor.update(updates)
        self._write_yaml(self.monitor_path, self.monitor)

    def _record_subtask_update(self, record: dict) -> None:
        subtasks = self.monitor.setdefault("subtasks", [])
        node_id = record["node_id"]
        for index, existing in enumerate(subtasks):
            if existing.get("node_id") == node_id:
                subtasks[index] = {**existing, **record}
                break
        else:
            subtasks.append(record)
        self._write_yaml(self.monitor_path, self.monitor)

    def _build_system_prompt(self) -> str:
        mode_rule = (
            "当前任务模式是 single_robot，只允许规划单个 robot_id 的串行多任务 DAG。"
            if self.mode == "single_robot"
            else "当前任务模式是 multi_robot，允许规划多个 robot_id 的并发协作 DAG。"
        )
        return f"""
你是 HoloAgent 主任务规划器。你的职责是把用户自然语言任务拆解为调用已注册 skill 的可执行 DAG(JSON)。
你只能输出合法 JSON，不要输出 markdown，不要输出解释，不要输出代码块。

输出 JSON 格式固定为：
{{
  "description": "任务描述",
  "nodes": [
    {{
      "id": "r11_nav_p1",
      "robot_id": 11,
      "skill": "sem-nav-skill",
      "target": "一楼,实验室,充电桩",
      "depends_on": []
    }}
  ]
}}

字段约束：
- id: 字符串，必须唯一
- robot_id: 整数，必须是 11/12/13/14/15/16 之一
- skill 只能是以下已注册能力：
  - "sem-nav-skill": target 必须是 "floor,room,object"；用户只给出明确物体时使用 "current,current,object"
  - "rel-move-skill": target 必须是 "forward,left,degrees"
  - "arm-skill": target 必须是预定义动作名
  - "navigation": 兼容已有命名点导航，target 只能是 {sorted(self.supported_navigation_targets)}
- target:
  - skill="arm-skill" 时只能是 {sorted(self.supported_arm_targets)}
- depends_on: 字符串数组，没有依赖时必须输出 []
- 用户没有指定 robot_id 时使用 {self.default_robot_id}
- “同时”表示并行，不要互相依赖
- “然后”“之后”“完成后”“到达后”表示建立依赖
- 不要凭空增加用户未提及的动作
- 如果无法可靠映射，输出 {{"description": "unrecognized", "nodes": []}}
- {mode_rule}

注册 skill 组合样例：
{{
  "description": "去实验室充电桩附近，向前微调后挥手",
  "nodes": [
    {{"id":"semantic_nav","robot_id":{self.default_robot_id},"skill":"sem-nav-skill","target":"一楼,实验室,充电桩","depends_on":[]}},
    {{"id":"fine_tune","robot_id":{self.default_robot_id},"skill":"rel-move-skill","target":"0.3,0,0","depends_on":["semantic_nav"]}},
    {{"id":"wave","robot_id":{self.default_robot_id},"skill":"arm-skill","target":"wave_above_head","depends_on":["fine_tune"]}}
  ]
}}

以下 SKILL.md 是可执行能力的权威契约：
{self.skill_catalog}

单机长指令样例：
{json.dumps(SINGLE_ROBOT_TEST_CASES, ensure_ascii=False, indent=2)}

多机长指令样例：
{json.dumps(MULTI_ROBOT_TEST_CASES, ensure_ascii=False, indent=2)}
"""

    def analyze_instruction(self, instruction: str) -> Optional[dict]:
        self._append_event("instruction_received", {
                           "instruction": instruction})
        try:
            response = self.client.responses.create(
                model=self.gpt_model,
                input=[
                    {"role": "system", "content": self._build_system_prompt()},
                    {"role": "user", "content": instruction},
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
                self._append_event("dag_generation_failed", {
                                   "reason": "empty_response"})
                return None

            dag = self._extract_json_object(content)
            normalized = self._validate_and_normalize_dag(dag)
            self._write_yaml(
                self.plan_path,
                {
                    "instruction": instruction,
                    "recorded_at": datetime.now().isoformat(),
                    "dag": normalized,
                },
            )
            self._append_event(
                "dag_generated",
                {
                    "node_count": len(normalized.get("nodes", [])) if normalized else 0,
                    "description": normalized.get("description", "") if normalized else "",
                },
            )
            return normalized
        except Exception as exc:
            self._append_event("dag_generation_failed", {"reason": str(exc)})
            return None

    @staticmethod
    def _extract_json_object(content: str) -> dict:
        """Decode the first complete JSON object from a model response."""
        decoder = json.JSONDecoder()
        for index, char in enumerate(content):
            if char != "{":
                continue
            try:
                value, _ = decoder.raw_decode(content[index:])
            except json.JSONDecodeError:
                continue
            if isinstance(value, dict):
                return value
        raise ValueError("Qwen response did not contain a JSON object")

    def _validate_and_normalize_dag(self, dag: dict) -> Optional[dict]:
        if not isinstance(dag, dict):
            return None

        description = str(dag.get("description", "")).strip()
        nodes = dag.get("nodes", [])
        if not isinstance(nodes, list):
            return None

        normalized_nodes = []
        node_ids: Set[str] = set()
        robot_ids = set()

        for node in nodes:
            if not isinstance(node, dict):
                return None

            node_id = str(node.get("id", "")).strip()
            if not node_id or node_id in node_ids:
                return None

            try:
                robot_id = int(node.get("robot_id"))
            except Exception:
                return None

            skill = str(node.get("skill", "")).strip()
            target = str(node.get("target", "")).strip()
            depends_on = node.get("depends_on", [])

            if robot_id not in self.robot_urls:
                return None
            if skill not in {
                "navigation", "arm", "arm-skill", "rel-move-skill", "sem-nav-skill"
            }:
                return None
            if skill == "navigation" and target not in self.supported_navigation_targets:
                return None
            if skill in {"arm", "arm-skill"} and target not in self.supported_arm_targets:
                return None
            if skill == "rel-move-skill" and not self._valid_relative_target(target):
                return None
            if skill == "sem-nav-skill" and not self._valid_semantic_target(target):
                return None
            if not isinstance(depends_on, list):
                return None

            normalized_nodes.append(
                {
                    "id": node_id,
                    "robot_id": robot_id,
                    "skill": skill,
                    "target": target,
                    "depends_on": [str(dep).strip() for dep in depends_on if str(dep).strip()],
                }
            )
            node_ids.add(node_id)
            robot_ids.add(robot_id)

        if self.mode == "single_robot" and len(robot_ids) > 1:
            return None

        for node in normalized_nodes:
            for dep in node["depends_on"]:
                if dep not in node_ids:
                    return None

        if normalized_nodes and self._has_cycle(normalized_nodes):
            return None

        return {"description": description or f"{self.mode}_dag", "nodes": normalized_nodes}

    @staticmethod
    def _valid_relative_target(target: str) -> bool:
        parts = [part.strip() for part in target.split(",")]
        if len(parts) != 3:
            return False
        try:
            [float(part) for part in parts]
        except ValueError:
            return False
        return True

    @staticmethod
    def _valid_semantic_target(target: str) -> bool:
        return len([part for part in target.split(",") if part.strip()]) == 3

    def _has_cycle(self, nodes: List[dict]) -> bool:
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

    def _build_dependency_closure(self, nodes: List[dict]) -> dict[str, Set[str]]:
        graph = {node["id"]: list(node["depends_on"]) for node in nodes}
        closure_cache: dict[str, Set[str]] = {}

        def collect_dependencies(node_id: str) -> Set[str]:
            if node_id in closure_cache:
                return closure_cache[node_id]

            dependencies: Set[str] = set()
            for dep in graph.get(node_id, []):
                dependencies.add(dep)
                dependencies.update(collect_dependencies(dep))

            closure_cache[node_id] = dependencies
            return dependencies

        for node_id in graph:
            collect_dependencies(node_id)

        return closure_cache

    def validate_dag_static(self, dag: Optional[dict]) -> bool:
        result = {
            "recorded_at": datetime.now().isoformat(),
            "status": "passed",
            "conclusion": "DAG 静态校验通过：依赖和单机器人串行约束有效",
            "checks": [],
            "dag_execution_order": [],
        }

        if not dag or not dag.get("nodes"):
            result["status"] = "failed"
            result["conclusion"] = "DAG 静态校验失败：DAG 为空或无法识别，禁止真实执行"
            result["checks"].append(
                {"name": "dag_non_empty", "passed": False, "reason": "DAG 为空或无法识别"}
            )
            self._write_yaml(self.dag_validation_path, result)
            self._update_monitor(status="validation_failed",
                                 validation_conclusion=result["conclusion"])
            return False

        nodes = dag["nodes"]
        node_map = {node["id"]: node for node in nodes}
        dependency_closure = self._build_dependency_closure(nodes)
        pending = set(node_map.keys())
        completed = set()
        step_index = 0

        for node in nodes:
            missing_deps = [dep for dep in node["depends_on"]
                            if dep not in node_map]
            passed = not missing_deps
            result["checks"].append(
                {
                    "name": f"deps_exist::{node['id']}",
                    "passed": passed,
                    "missing_dependencies": missing_deps,
                }
            )
            if not passed:
                result["status"] = "failed"

        for node in nodes:
            conflicts = [
                other["id"]
                for other in nodes
                if other["id"] != node["id"]
                and other["robot_id"] == node["robot_id"]
                and node["id"] not in dependency_closure.get(other["id"], set())
                and other["id"] not in dependency_closure.get(node["id"], set())
            ]
            passed = not conflicts
            result["checks"].append(
                {
                    "name": f"single_robot_serialization::{node['id']}",
                    "passed": passed,
                    "conflicting_nodes": conflicts,
                }
            )
            if not passed:
                result["status"] = "failed"

        while pending and result["status"] == "passed":
            ready_nodes = [
                node_map[node_id]
                for node_id in sorted(pending)
                if all(dep in completed for dep in node_map[node_id]["depends_on"])
            ]
            if not ready_nodes:
                result["status"] = "failed"
                result["conclusion"] = "DAG 静态校验失败：存在无法推进的依赖阻塞，禁止真实执行"
                result["checks"].append(
                    {
                        "name": "topology_progress",
                        "passed": False,
                        "reason": f"剩余节点无法推进: {sorted(pending)}",
                    }
                )
                break

            step_index += 1
            result["dag_execution_order"].append(
                {
                    "step": step_index,
                    "parallel_nodes": [
                        {
                            "id": node["id"],
                            "robot_id": node["robot_id"],
                            "skill": node["skill"],
                            "target": node["target"],
                        }
                        for node in ready_nodes
                    ],
                }
            )
            for node in ready_nodes:
                pending.remove(node["id"])
                completed.add(node["id"])

        if result["status"] != "passed" and result["conclusion"].startswith("DAG 静态校验通过"):
            result["conclusion"] = "DAG 静态校验失败：存在依赖或串行约束问题，禁止真实执行"

        self._write_yaml(self.dag_validation_path, result)
        self._update_monitor(
            status="validated" if result["status"] == "passed" else "validation_failed",
            validation_conclusion=result["conclusion"],
        )
        self._append_event(
            "dag_validation_completed",
            {"status": result["status"], "conclusion": result["conclusion"]},
        )
        return result["status"] == "passed"

    def execute(self, dag: Optional[dict]) -> bool:
        if self.dry_run:
            result = {
                "recorded_at": datetime.now().isoformat(),
                "status": "skipped",
                "conclusion": "当前为 dry-run，仅完成 DAG 静态校验，未触发真实物理执行",
                "executed_nodes": [],
            }
            self._write_yaml(self.execution_path, result)
            self._update_monitor(status="dry_run_completed",
                                 execution_conclusion=result["conclusion"])
            self._append_event("physical_execution_skipped", result)
            return True

        if not dag or not dag.get("nodes"):
            result = {
                "recorded_at": datetime.now().isoformat(),
                "status": "failed",
                "conclusion": "真实执行失败：DAG 为空",
                "executed_nodes": [],
            }
            self._write_yaml(self.execution_path, result)
            self._update_monitor(status="execution_failed",
                                 execution_conclusion=result["conclusion"])
            return False

        nodes = dag["nodes"]
        node_map = {node["id"]: node for node in nodes}
        pending = set(node_map.keys())
        completed = set()
        failed = set()
        execution_trace = []

        with ThreadPoolExecutor(max_workers=self._DAG_EXECUTOR_MAX_WORKERS) as executor:
            while pending:
                ready_nodes = [
                    node_map[node_id]
                    for node_id in sorted(pending)
                    if all(dep in completed for dep in node_map[node_id]["depends_on"])
                ]
                if not ready_nodes:
                    break

                future_to_node_id = {
                    executor.submit(self._execute_single_node, node): node["id"]
                    for node in ready_nodes
                }
                for node in ready_nodes:
                    pending.remove(node["id"])

                for future, node_id in future_to_node_id.items():
                    success = future.result()
                    execution_trace.append(
                        {"node_id": node_id, "success": success})
                    if success:
                        completed.add(node_id)
                    else:
                        failed.add(node_id)

                if failed:
                    break

        success = not failed and not pending
        conclusion = (
            "真实执行完成：所有 DAG 节点已按验证后的顺序执行成功"
            if success
            else f"真实执行失败：失败节点={sorted(failed)}，剩余节点={sorted(pending)}"
        )
        result = {
            "recorded_at": datetime.now().isoformat(),
            "status": "passed" if success else "failed",
            "conclusion": conclusion,
            "completed_nodes": sorted(completed),
            "failed_nodes": sorted(failed),
            "remaining_nodes": sorted(pending),
            "execution_trace": execution_trace,
        }
        self._write_yaml(self.execution_path, result)
        self._update_monitor(
            status="completed" if success else "execution_failed",
            execution_conclusion=conclusion,
        )
        self._append_event(
            "physical_execution_completed",
            {"status": result["status"], "conclusion": conclusion},
        )
        return success

    def _execute_single_node(self, node: dict) -> bool:
        robot_id = node["robot_id"]
        skill = node["skill"]
        target = node["target"]
        started_at = datetime.now().isoformat()
        started_monotonic = time.time()

        self._record_subtask_update(
            {
                "node_id": node["id"],
                "robot_id": robot_id,
                "skill": skill,
                "target": target,
                "status": "running",
                "started_at": started_at,
            }
        )
        self._append_event(
            "node_execution_started",
            {
                "node_id": node["id"],
                "robot_id": robot_id,
                "skill": skill,
                "target": target,
                "started_at": started_at,
            },
        )

        if skill == "navigation":
            ok = self._call_robot_navigation(robot_id, target)
        elif skill in {"arm", "arm-skill"}:
            ok = self._call_robot_arm(robot_id, target)
        elif skill == "rel-move-skill":
            ok = self._call_navigation_skill(robot_id, "relative_nav", target)
        elif skill == "sem-nav-skill":
            ok = self._call_navigation_skill(robot_id, "semantic_nav", target)
        else:
            ok = False

        finished_at = datetime.now().isoformat()
        duration_sec = round(time.time() - started_monotonic, 3)
        status = "passed" if ok else "failed"
        self._record_subtask_update(
            {
                "node_id": node["id"],
                "robot_id": robot_id,
                "skill": skill,
                "target": target,
                "status": status,
                "started_at": started_at,
                "finished_at": finished_at,
                "duration_sec": duration_sec,
            }
        )
        self._append_event(
            "node_execution_finished",
            {
                "node_id": node["id"],
                "robot_id": robot_id,
                "skill": skill,
                "target": target,
                "success": ok,
                "finished_at": finished_at,
                "duration_sec": duration_sec,
            },
        )
        print(
            f"[subtask] node={node['id']} robot={robot_id} skill={skill} "
            f"target={target} status={status} duration_sec={duration_sec}"
        )
        return ok

    def _call_robot_navigation(self, robot_id: int, target: str) -> bool:
        self._append_event(
            "navigation_trigger_requested",
            {"robot_id": robot_id, "target": target},
        )
        self._reset_control_center_reached_state([robot_id])
        url = f"{self.control_center_url}/trigger/{target}?robot_id={robot_id}"
        if not self._post_json(url):
            self._append_event(
                "navigation_trigger_failed",
                {"robot_id": robot_id, "target": target},
            )
            return False
        self._append_event(
            "navigation_trigger_accepted",
            {"robot_id": robot_id, "target": target},
        )
        return self._wait_for_robot_navigation_completion(robot_id, target)

    def _call_robot_arm(self, robot_id: int, target: str) -> bool:
        self._append_event(
            "arm_trigger_requested",
            {"robot_id": robot_id, "target": target},
        )
        self._reset_control_center_reached_state([robot_id])
        url = f"{self.robot_urls[robot_id]}/api/arm/{target}"
        if not self._post_json(url):
            self._append_event(
                "arm_trigger_failed",
                {"robot_id": robot_id, "target": target},
            )
            return False
        self._append_event(
            "arm_trigger_accepted", {"robot_id": robot_id, "target": target}
        )
        return self._wait_for_robot_completion(robot_id, f"arm_finish:{target}")

    def _call_navigation_skill(self, robot_id: int, endpoint: str, target: str) -> bool:
        self._reset_control_center_reached_state([robot_id])
        url = f"{self.robot_urls[robot_id]}/api/{endpoint}"
        if not self._post_json(url, {"cmd": target}):
            return False
        return self._wait_for_robot_completion(robot_id, "nav_finish")

    def _post_json(self, url: str, payload: Optional[dict] = None) -> bool:
        try:
            response = requests.post(
                url, json=payload, timeout=self._HTTP_TIMEOUT_SEC)
            return response.ok
        except Exception:
            return False

    def _reset_control_center_reached_state(self, robot_ids: List[int]) -> None:
        try:
            requests.post(
                f"{self.control_center_url}/reset_reached",
                json={"robot_ids": robot_ids},
                timeout=self._CONTROL_CENTER_TIMEOUT_SEC,
            )
        except Exception:
            pass

    def _wait_for_robot_completion(self, robot_id: int, target: str) -> bool:
        wait_started_at = datetime.now().isoformat()
        self._append_event(
            "navigation_wait_started",
            {"robot_id": robot_id, "target": target, "started_at": wait_started_at},
        )
        deadline = time.time() + self._NAV_WAIT_TIMEOUT_SEC
        while time.time() < deadline:
            try:
                response = requests.get(
                    f"{self.control_center_url}/robot_reached/{robot_id}",
                    params={"target": target},
                    timeout=self._CONTROL_CENTER_TIMEOUT_SEC,
                )
                status = response.json() if response.ok else {}
                reported_target = status.get("target")
                if self._is_terminal_failure(reported_target):
                    self._append_event(
                        "navigation_wait_failed",
                        {"robot_id": robot_id, "target": reported_target},
                    )
                    return False
                if response.ok and status.get("reached"):
                    self._append_event(
                        "navigation_wait_completed",
                        {
                            "robot_id": robot_id,
                            "target": target,
                            "finished_at": datetime.now().isoformat(),
                            "completion_source": "nav_finish",
                        },
                    )
                    return True
            except Exception:
                pass
            time.sleep(0.5)

        self._append_event(
            "navigation_wait_timeout",
            {
                "robot_id": robot_id,
                "target": target,
                "finished_at": datetime.now().isoformat(),
                "timeout_sec": self._NAV_WAIT_TIMEOUT_SEC,
            },
        )
        if target == "nav_finish":
            self._cancel_navigation_and_confirm(robot_id)
        return False

    @staticmethod
    def _is_terminal_failure(reported_target: object) -> bool:
        value = str(reported_target or "")
        return value in {"nav_failed", "nav_canceled"} or value.startswith(
            "arm_failed:")

    def _cancel_navigation_and_confirm(self, robot_id: int) -> bool:
        cancel_url = f"{self.robot_urls[robot_id]}/api/navigation/stop"
        if not self._post_json(cancel_url):
            self._append_event(
                "navigation_cancel_failed", {"robot_id": robot_id, "reason": "request_failed"}
            )
            return False
        deadline = time.time() + 10.0
        while time.time() < deadline:
            try:
                response = requests.get(
                    f"{self.control_center_url}/robot_reached/{robot_id}",
                    timeout=self._CONTROL_CENTER_TIMEOUT_SEC,
                )
                if response.ok and response.json().get("target") == "nav_canceled":
                    self._append_event(
                        "navigation_cancel_confirmed", {"robot_id": robot_id}
                    )
                    return True
            except Exception:
                pass
            time.sleep(0.5)
        self._append_event(
            "navigation_cancel_failed", {"robot_id": robot_id, "reason": "confirmation_timeout"}
        )
        return False

    _wait_for_robot_navigation_completion = _wait_for_robot_completion

    def run(self, instruction: str) -> int:
        self._update_monitor(status="planning")
        dag = self.analyze_instruction(instruction)
        if not dag:
            self._update_monitor(
                status="planning_failed",
                validation_conclusion="DAG 规划失败，无法进入 DAG 静态校验",
                execution_conclusion="未执行",
            )
            return 1

        validation_ok = self.validate_dag_static(dag)
        if not validation_ok:
            self._update_monitor(
                execution_conclusion="DAG 静态校验未通过，真实执行被阻止",
            )
            return 2

        execution_ok = self.execute(dag)
        return 0 if execution_ok else 3


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="纯文本长指令 DAG 执行器")
    parser.add_argument("--instruction", required=True, help="纯文本长指令")
    parser.add_argument(
        "--mode", choices=["single_robot", "multi_robot"], required=True)
    parser.add_argument("--dry-run", action="store_true", help="仅做规划和 DAG 静态校验")
    parser.add_argument("--output-root", default=str(DEFAULT_OUTPUT_ROOT))
    return parser


def main() -> int:
    parser = build_arg_parser()
    args = parser.parse_args()
    runner = LongHorizonTextRunner(
        mode=args.mode,
        dry_run=args.dry_run,
        output_root=Path(args.output_root),
    )
    return runner.run(args.instruction)


if __name__ == "__main__":
    raise SystemExit(main())
