#!/usr/bin/env python3
"""
G1chat ROS2 节点.

- 启动底层 G1Chat 服务（等价于 g1.py, 启动后会一直运行直到进程退出)
- 将 G1Chat 的 text_queue 映射为 ROS2 topic:
  - qa:      发布 user/assistant 文本对话，形如 "user:...", "assistant:..."
  - location:发布位置信息，形如 "location:{...json...}"
  - signal:  发布内部控制信号，形如 "signal:some_signal_name"
- 将 G1Chat 的 control_queue 映射为 ROS2 订阅 topic:
  - control: 订阅控制信号字符串，放入 control_queue

使用前请先安装 g1chat 的 whl 包，并在同一环境中运行。
"""

import threading
from queue import Empty

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from g1 import G1Chat


class G1ChatNode(Node):
    """封装 G1Chat 的 ROS2 节点."""

    def __init__(self) -> None:
        super().__init__("g1chat_node")

        # 创建底层 G1Chat 实例
        self._chat = G1Chat()

        # 发布者：qa / location / signal
        self._qa_pub = self.create_publisher(String, "chat_qa_pub", 10)
        self._location_pub = self.create_publisher(String, "chat_loc_pub", 10)
        self._signal_pub = self.create_publisher(String, "chat_signal_pub", 10)

        # 订阅者：control
        self._signal_sub = self.create_subscription(
            String, "waypoint_reached", self.waypoint_callback, 10)

        # 启动底层 G1Chat 服务（在独立线程中运行 asyncio 循环）
        self._chat_thread = threading.Thread(
            target=self._run_chat_loop, daemon=True)
        self._chat_thread.start()

        # 启动队列 -> ROS topic 的桥接线程
        self._bridge_thread = threading.Thread(
            target=self._bridge_text_queue, daemon=True)
        self._bridge_thread.start()

        self.get_logger().info("g1chat_node 已启动:G1Chat 服务运行中，队列与 ROS topic 已建立映射。")

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
                continue

            if item.startswith("location:"):
                msg = String()
                msg.data = item[len("location:"):].strip()
                self._location_pub.publish(msg)
                continue

            # 其它非空字符串统一视为 signal
            msg = String()
            # msg.data = item
            msg.data = item[len("signal:"):].strip()
            # print("_singal_pub msg.data: ", msg.data)
            self._signal_pub.publish(msg)

    def waypoint_callback(self, msg: String) -> None:
        """订阅 control topic, 将消息内容写入 G1Chat.control_queue."""
        received_data = msg.data.strip()
        if not received_data:
            return
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
