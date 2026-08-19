#!/usr/bin/env python3
"""Small dependency-free checks for the formal Agent skill runtime."""

from sandbox_test.long_horizon_text_runner import LongHorizonTextRunner


def make_runner() -> LongHorizonTextRunner:
    runner = LongHorizonTextRunner.__new__(LongHorizonTextRunner)
    runner.mode = "single_robot"
    runner.robot_urls = {13: "http://robot:8000"}
    runner.supported_navigation_targets = {"one_point_1", "stop"}
    runner.supported_arm_targets = {"wave_above_head"}
    return runner


def test_registered_skill_dag() -> None:
    runner = make_runner()
    dag = runner._validate_and_normalize_dag(
        {
            "description": "semantic nav then wave",
            "nodes": [
                {
                    "id": "nav",
                    "robot_id": 13,
                    "skill": "sem-nav-skill",
                    "target": "一楼,实验室,充电桩",
                    "depends_on": [],
                },
                {
                    "id": "move",
                    "robot_id": 13,
                    "skill": "rel-move-skill",
                    "target": "0.3,0,0",
                    "depends_on": ["nav"],
                },
                {
                    "id": "wave",
                    "robot_id": 13,
                    "skill": "arm-skill",
                    "target": "wave_above_head",
                    "depends_on": ["move"],
                },
            ],
        }
    )
    assert dag and [node["skill"] for node in dag["nodes"]] == [
        "sem-nav-skill",
        "rel-move-skill",
        "arm-skill",
    ]


def test_skill_http_dispatch() -> None:
    runner = make_runner()
    calls = []
    runner._reset_control_center_reached_state = lambda robot_ids: calls.append(
        ("reset", robot_ids)
    )
    runner._post_json = lambda url, payload=None: calls.append(
        ("post", url, payload)
    ) or True
    runner._wait_for_robot_completion = lambda robot_id, target: calls.append(
        ("wait", robot_id, target)
    ) or True

    assert runner._call_navigation_skill(13, "semantic_nav", "一楼,实验室,充电桩")
    assert calls[-2:] == [
        ("post", "http://robot:8000/api/semantic_nav", {"cmd": "一楼,实验室,充电桩"}),
        ("wait", 13, "nav_finish"),
    ]


def test_arm_requires_real_completion() -> None:
    runner = make_runner()
    calls = []
    runner._append_event = lambda *args: None
    runner._reset_control_center_reached_state = lambda robot_ids: calls.append(
        ("reset", robot_ids)
    )
    runner._post_json = lambda url, payload=None: calls.append(
        ("post", url, payload)
    ) or True
    runner._wait_for_robot_completion = lambda robot_id, target: calls.append(
        ("wait", robot_id, target)
    ) or True

    assert runner._call_robot_arm(13, "wave_above_head")
    assert calls[-2:] == [
        ("post", "http://robot:8000/api/arm/wave_above_head", None),
        ("wait", 13, "arm_finish:wave_above_head"),
    ]


def test_navigation_timeout_cancels() -> None:
    runner = make_runner()
    calls = []
    runner._append_event = lambda *args: None
    runner._NAV_WAIT_TIMEOUT_SEC = 0.0
    runner._cancel_navigation_and_confirm = lambda robot_id: calls.append(robot_id) or True

    assert not runner._wait_for_robot_completion(13, "nav_finish")
    assert calls == [13]


def test_stuck_warning_is_not_a_terminal_result() -> None:
    assert not LongHorizonTextRunner._is_terminal_failure("struck")
    assert LongHorizonTextRunner._is_terminal_failure("nav_failed")


def test_qwen_json_is_extracted_from_wrapped_output() -> None:
    payload = LongHorizonTextRunner._extract_json_object(
        'reasoning first\n```json\n{"description":"ok","nodes":[]}\n```')
    assert payload == {"description": "ok", "nodes": []}


if __name__ == "__main__":
    test_registered_skill_dag()
    test_skill_http_dispatch()
    test_arm_requires_real_completion()
    test_navigation_timeout_cancels()
    test_stuck_warning_is_not_a_terminal_result()
    test_qwen_json_is_extracted_from_wrapped_output()
    print("agent_runtime_tests: 6 passed")
