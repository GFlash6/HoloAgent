#!/usr/bin/env python3
"""
G1chat ROS2 节点, 单机言出法随instruction_follow_demo.

- 启动底层 G1Chat 服务（等价于 g1.py, 启动后会一直运行直到进程退出)
- 将 G1Chat 的 text_queue 映射为 ROS2 topic:
  - qa:      发布 user/assistant 文本对话，形如 "user:...", "assistant:..."
  - location:发布位置信息，形如 "location:{...json...}"
  - signal:  发布内部控制信号，形如 "signal:some_signal_name"
- 将 G1Chat 的 control_queue 映射为 ROS2 订阅 topic:
  - control: 订阅控制信号字符串，放入 control_queue

使用前请先安装 g1chat 的 whl 包，并在同一环境中运行。
"""

import json
import os
import threading
import time
from queue import Empty, Queue

import rclpy
from openai import AzureOpenAI, OpenAI
from rclpy.node import Node
from std_msgs.msg import String

from g1 import G1Chat


class G1ChatNode(Node):
    """封装 G1Chat 的 ROS2 节点."""

    _NAV_FINISH_SIGNAL = "nav_finish"  # navigation successed or stop by user
    _UNRECOGNIZED_COMMAND_SIGNAL = "unrecognized_command"
    _NAV_WAIT_TIMEOUT_SEC = 60.0
    _MOTION_TRACKING_WAIT_SEC = 8.0

    def __init__(self) -> None:
        super().__init__("g1chat_node")

        # 创建底层 G1Chat 实例
        self._chat = G1Chat()

        # 发布者：qa / location / signal / relative_nav / motion_tracking
        self._qa_pub = self.create_publisher(String, "chat_qa_pub", 10)
        self._location_pub = self.create_publisher(String, "chat_loc_pub", 10)
        self._signal_pub = self.create_publisher(String, "chat_signal_pub", 10)
        self._relative_nav_pub = self.create_publisher(
            String, "/relative_nav", 10)
        self._motion_tracking_pub = self.create_publisher(
            String, "/motion_tracking", 10)

        self._nav_finish_event = threading.Event()
        self._task_queue = Queue()

        # 订阅者：control
        self._signal_sub = self.create_subscription(
            String, "waypoint_reached", self.waypoint_callback, 10
        )

        # GPT 客户端初始化（参考 gpt_vision.py，兼容 OpenAI / Azure OpenAI）
        gpt_provider = os.getenv("GPT_PROVIDER", "azure").strip().lower()
        gpt_api_key = (
            os.getenv("AZURE_OPENAI_API_KEY")
            if gpt_provider == "azure"
            else os.getenv("OPENAI_API_KEY")
        )
        if not gpt_api_key:
            raise RuntimeError(
                "未配置 GPT API Key，请设置 AZURE_OPENAI_API_KEY 或 OPENAI_API_KEY"
            )

        if gpt_provider == "azure":
            azure_endpoint = os.getenv("AZURE_OPENAI_ENDPOINT")
            if not azure_endpoint:
                raise RuntimeError(
                    "未配置 Azure Endpoint，请设置 AZURE_OPENAI_ENDPOINT"
                )

            azure_api_version = os.getenv(
                "AZURE_OPENAI_API_VERSION", "2024-02-15-preview"
            )
            self.gpt_model = os.getenv(
                "AZURE_OPENAI_DEPLOYMENT",
                os.getenv("AZURE_OPENAI_MODEL", "gpt-4o"),
            )
            self.client = AzureOpenAI(
                azure_endpoint=azure_endpoint,
                api_key=gpt_api_key,
                api_version=azure_api_version,
            )
        else:
            self.gpt_model = os.getenv("OPENAI_MODEL", "gpt-4o")
            self.client = OpenAI(
                api_key=gpt_api_key,
                base_url=os.getenv("OPENAI_BASE_URL") or None,
            )

        # 框架层面可兼容 HoloMotion / [motion_tracking, velocity_tracking] 相关能力，用于离线参考动作播放和底盘移动速度跟踪。
        # 当前开源版本默认对接宇树原生运控动作接口完成动作执行，和生成式HoloMoiton/HoloBrain的集成还在内测中。
        # 如需扩展自定义全身运控技能或替换宇树官方速度跟踪接口，可基于如下项目需要自行集成适配或等下一步开源：
        # https://github.com/HorizonRobotics/HoloMotion(当前v1.3版本已提供在/离线参考动作跟踪能力接口和速度跟踪能力接口)
        self._unitree_supported_motion_tracking_commands = {
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

        # 启动底层 G1Chat 服务（在独立线程中运行 asyncio 循环）
        self._chat_thread = threading.Thread(
            target=self._run_chat_loop, daemon=True)
        self._chat_thread.start()

        # 启动队列 -> ROS topic 的桥接线程
        self._bridge_thread = threading.Thread(
            target=self._bridge_text_queue, daemon=True)
        self._bridge_thread.start()

        # 启动任务串行执行线程，避免阻塞 text_queue 桥接
        self._task_worker_thread = threading.Thread(
            target=self._task_worker, daemon=True)
        self._task_worker_thread.start()

        self.get_logger().info("g1chat_node 已启动")

    # -------------------- G1Chat 启动 --------------------

    def _run_chat_loop(self) -> None:
        """在独立线程中以 g1.py 的方式运行 G1Chat(直到进程结束)"""
        import asyncio

        async def _main():
            await self._chat.start()
            try:
                # 等价于 g1.py 中的 asyncio.Future()，一直运行直到进程退出
                await asyncio.Future()
            except asyncio.CancelledError:
                pass
            finally:
                await self._chat.stop()

        asyncio.run(_main())

    # -------------------- GPT 任务拆解 --------------------

    def _analyze_user_command(self, user_text: str):
        """使用 GPT 将用户指令拆解为 relative_nav / motion_tracking 任务序列。"""
        system_prompt = """
你是一个机器人任务拆解器。你只能把用户指令拆解为以下两类任务，并按顺序输出 JSON：

1. relative_nav: 相对导航，data 必须是字符串，格式为 "forward,left,degrees"
   - forward: 前进为正，后退为负，单位米
   - left: 左移为正，右移为负，单位米
   - degrees: 左转为正，右转为负，单位度
   - 如果用户未明确要求转向，则 degrees 填 0
   - 示例: "1.0,0.5,90"

2. motion_tracking: 全身运动跟踪，data 必须是以下受支持动作名之一：
   - "release_arm"
   - "turn_back_wave"
   - "blow_kiss_with_both_hands"
   - "blow_kiss_with_left_hand"
   - "blow_kiss_with_right_hand"
   - "both_hands_up"
   - "clamp"
   - "high_five"
   - "hug"
   - "make_heart_with_both_hands"
   - "make_heart_with_right_hand"
   - "refuse"
   - "right_hand_up"
   - "ultraman_ray"
   - "wave_under_head"
   - "wave_above_head"
   - "shake_hand"
   - "one_point_1_waypoint_1"
   - "box_left_hand_win"
   - "box_right_hand_win"
   - "box_both_hand_win"
   - "right_hand_on_heart"
   - "both_hands_up_deviate_right"
   - "forward_push"
   - 其中 "shake_hand" 与 "one_point_1_waypoint_1" 视为同类握手动作

输出要求：
- 只能输出合法 JSON，不要输出 markdown，不要输出解释
- JSON 格式固定为:
{
  "tasks": [
    {"type": "relative_nav", "data": "1.0,0.5,90"},
    {"type": "motion_tracking", "data": "high_five"}
  ]
}
- 如果无法可靠拆解为上述两类任务，输出:
{"tasks": []}
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
                self.get_logger().warning("GPT 返回内容为空。")
                return []

            result = json.loads(content)
            tasks = result.get("tasks", [])
            if not isinstance(tasks, list):
                return []

            valid_tasks = []
            for task in tasks:
                if not isinstance(task, dict):
                    continue
                task_type = str(task.get("type", "")).strip()
                data = str(task.get("data", "")).strip()
                if task_type not in ("relative_nav", "motion_tracking"):
                    continue
                if not data:
                    continue
                if (
                    task_type == "motion_tracking"
                    and data not in self._unitree_supported_motion_tracking_commands
                ):
                    continue
                valid_tasks.append({"type": task_type, "data": data})

            return valid_tasks
        except Exception as exc:
            self.get_logger().error(f"GPT 指令拆解失败: {exc}")
            return []

    def _publish_unrecognized_command(self) -> None:
        """反馈无法识别指令。"""
        msg = String()
        msg.data = self._UNRECOGNIZED_COMMAND_SIGNAL
        self._signal_pub.publish(msg)
        self.get_logger().warning(f"无法识别用户指令，已发布信号: {msg.data}")

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
        """串行处理用户指令，避免阻塞 text_queue 桥接线程。"""
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

            tasks = self._analyze_user_command(user_text)
            self._execute_tasks(tasks)

    def _execute_tasks(self, tasks) -> None:
        """按顺序执行 GPT 拆解出的任务。"""
        if not tasks:
            self._publish_unrecognized_command()
            return

        for task in tasks:
            task_type = task["type"]
            data = task["data"]
            msg = String()
            msg.data = data

            if task_type == "relative_nav":
                self._nav_finish_event.clear()
                self._relative_nav_pub.publish(msg)
                self.get_logger().info(f"relative_nav -> {data}")

                finished = self._nav_finish_event.wait(
                    timeout=self._NAV_WAIT_TIMEOUT_SEC
                )
                if not finished:
                    self.get_logger().warning(
                        f"等待导航完成超时（{self._NAV_WAIT_TIMEOUT_SEC} 秒）: {data}"
                    )
                    return
            elif task_type == "motion_tracking":
                self._motion_tracking_pub.publish(msg)
                self.get_logger().info(f"motion_tracking -> {data}")
                time.sleep(self._MOTION_TRACKING_WAIT_SEC)
            else:
                self.get_logger().warning(f"忽略未知任务类型: {task_type}")
                continue

    # -------------------- ROS <-> G1Chat 桥接 --------------------

    def _bridge_text_queue(self) -> None:
        """持续从 G1Chat.text_queue 取数据并发布到对应 topic."""
        q = self._chat.text_queue

        while rclpy.ok():
            try:
                item = q.get(timeout=0.1)
            except Empty:
                continue

            if not isinstance(item, str) or not item:
                continue

            # 形如 "user:xxx" / "assistant:yyy" / "location:{...}" /
            # "some_signal"
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

            # 其它非空字符串统一视为 signal
            msg = String()
            if item.startswith("signal:"):
                msg.data = item[len("signal:"):].strip()
            else:
                msg.data = item.strip()
            self._signal_pub.publish(msg)

    def waypoint_callback(self, msg: String) -> None:
        """订阅 waypoint_reached，并将消息内容写入 G1Chat.control_queue。"""
        received_data = msg.data.strip()
        if not received_data:
            return

        if received_data == self._NAV_FINISH_SIGNAL:
            self._nav_finish_event.set()

        self._chat.control_queue.put(received_data)


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
