#!/usr/bin/env python3
"""
g1_tts ROS2 节点.

功能：
- 订阅 /voice/tts_message/ 接收文本字符串（中文或英文）
- 调用豆包 TTS 服务将文本转换为语音
- 使用音频设备播放语音
- 播放完成后，发布 bool 类型 true 到 /voice/tts_status/

使用前请确保音频模块已正确配置，参考 audio/ 目录下的相关文件。
"""

import asyncio
import copy
import json
import threading
import uuid
from queue import Empty, Queue

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String

from audio.audio_device import AudioDevice
from audio.env import SETTINGS
from audio.logging import default_logger as logger
from audio.misc import create_websocket_connection
from audio.volcengine_doubao_tts import (
    EventType,
    MsgType,
    finish_connection,
    finish_session,
    receive_message,
    start_connection,
    start_session,
    task_request,
    wait_for_event,
)

# 豆包 TTS 配置
TTS_APP_KEY = SETTINGS["tts"]["app_id"]
TTS_ACCESS_KEY = SETTINGS["tts"]["access_key"]
TTS_RESOURCE_ID = SETTINGS["tts"]["resource_id"]
TTS_ENDPOINT = SETTINGS["tts"]["endpoint"]
TTS_VOICE_TYPE = SETTINGS["tts"]["voice_type"]


class G1TTSNode(Node):
    """G1 TTS ROS2 节点."""

    def __init__(self) -> None:
        super().__init__("g1tts_node")

        # 订阅者：接收 TTS 文本消息
        self._tts_sub = self.create_subscription(
            String, "voice/tts_message", self.tts_message_callback, 10
        )

        # 发布者：TTS 播放完成状态
        self._status_pub = self.create_publisher(Bool, "voice/tts_status", 10)

        # 初始化音频设备
        self._audio_device = AudioDevice()
        self._audio_device.start_streams()

        # 消息队列，用于异步处理 TTS 请求
        self._tts_queue: Queue = Queue()
        self._is_processing = False

        # 启动 TTS 处理线程
        self._tts_thread = threading.Thread(target=self._tts_loop, daemon=True)
        self._tts_thread.start()

        self.get_logger().info("g1tts_node 已启动：订阅 /voice/tts_message/，播放完成后发布到 /voice/tts_status/")

    def tts_message_callback(self, msg: String) -> None:
        """接收 TTS 消息并加入队列."""
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(f"收到 TTS 请求: {text[:50]}...")
        self._tts_queue.put(text)

    def _tts_loop(self) -> None:
        """在独立线程中运行 asyncio 事件循环处理 TTS 请求."""
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        try:
            while rclpy.ok():
                try:
                    text = self._tts_queue.get(timeout=0.1)
                except Empty:
                    continue
                loop.run_until_complete(self._synthesize_and_play(text))
        except Exception as e:
            logger.error(f"TTS 处理线程异常: {e}")
        finally:
            loop.close()

    async def _synthesize_and_play(self, text: str) -> None:
        """调用豆包 TTS 服务并将语音播放."""
        self.get_logger().info("正在连接豆包 TTS 服务...")

        # 重启播放流以清空内核缓冲区，避免前一次播放的残留数据干扰
        if self._audio_device:
            logger.info("重启播放流以确保干净状态...")
            self._audio_device.clear_playback_queue()
            self._audio_device._stop_output()
            self._audio_device._open_output_stream()
            logger.info("播放流已重启")

        try:
            websocket = await create_websocket_connection(
                TTS_APP_KEY, TTS_ACCESS_KEY, TTS_RESOURCE_ID, TTS_ENDPOINT
            )

            # 按句号分割文本，逐句处理
            sentences = text.replace("。", ".").split(".")
            audio_received = False

            for sentence in sentences:
                sentence = sentence.strip()
                if not sentence:
                    continue

                # 构建基础请求参数
                base_request = {
                    "user": {
                        "uid": str(uuid.uuid4()),
                    },
                    "namespace": "BidirectionalTTS",
                    "req_params": {
                        "speaker": TTS_VOICE_TYPE,
                        "audio_params": {
                            "format": "pcm",
                            "sample_rate": self._audio_device.sample_rate,
                            "enable_timestamp": True,
                        },
                        "additions": json.dumps(
                            {
                                "disable_markdown_filter": False,
                            }
                        ),
                    },
                }

                # 启动会话
                start_session_request = copy.deepcopy(base_request)
                start_session_request["event"] = EventType.StartSession
                session_id = str(uuid.uuid4())
                await start_session(
                    websocket, json.dumps(
                        start_session_request).encode(), session_id
                )
                await wait_for_event(
                    websocket, MsgType.FullServerResponse, EventType.SessionStarted
                )

                # 逐字符发送文本
                async def send_chars():
                    for char in sentence:
                        synthesis_request = copy.deepcopy(base_request)
                        synthesis_request["event"] = EventType.TaskRequest
                        synthesis_request["req_params"]["text"] = char
                        await task_request(
                            websocket, json.dumps(
                                synthesis_request).encode(), session_id
                        )
                        await asyncio.sleep(0.005)

                    await finish_session(websocket, session_id)

                send_task = asyncio.create_task(send_chars())

                # 接收音频数据并播放
                audio_data = bytearray()
                playback_started = False

                while True:
                    msg = await receive_message(websocket)

                    if msg.type == MsgType.FullServerResponse:
                        if msg.event == EventType.SessionFinished:
                            break
                    elif msg.type == MsgType.AudioOnlyServer:
                        if not audio_received and len(msg.payload) > 0:
                            audio_received = True

                        audio_data.extend(msg.payload)

                        if self._audio_device and msg.payload:
                            try:
                                self._audio_device.put_playback_data(
                                    msg.payload)
                                if not playback_started:
                                    playback_started = True
                            except Exception as e:
                                logger.error(f"播放音频片段失败: {e}")
                    else:
                        raise RuntimeError(f"TTS conversion failed: {msg}")

                await send_task
                self.get_logger().info(f"会话完成，接收音频: {len(audio_data)} bytes")

                # 等待播放队列播放完成（每个句子处理完后都要等待）
                if self._audio_device:
                    q_size = self._audio_device.get_playback_queue_size()
                    aplay_ok = self._audio_device.output_stream and self._audio_device.output_stream.poll() is None
                    writer_ok = self._audio_device._writer_thread and self._audio_device._writer_thread.is_alive()
                    logger.info(
                        f"等待播放完成, queue={q_size}, aplay_alive={aplay_ok}, writer_alive={writer_ok}")
                    # 先等待队列有足够的数据
                    await asyncio.sleep(0.5)

                    # 然后等待队列播放完
                    max_wait = 100  # 最多等待 10 秒
                    wait_count = 0
                    while self._audio_device.get_playback_queue_size() > 0 and wait_count < max_wait:
                        await asyncio.sleep(0.1)  # 每 100ms 检查一次
                        wait_count += 1
                        if wait_count % 50 == 0:
                            q_now = self._audio_device.get_playback_queue_size()
                            aplay_ok = self._audio_device.output_stream and self._audio_device.output_stream.poll() is None
                            writer_ok = self._audio_device._writer_thread and self._audio_device._writer_thread.is_alive()
                            logger.info(
                                f"仍在等待播放, queue={q_now}, aplay_alive={aplay_ok}, writer_alive={writer_ok}, waited={wait_count*0.1:.1f}s")

                    # 额外等待一段时间确保最后的音频播放完
                    await asyncio.sleep(1.0)
                    logger.info("句子播放完成")

            # 检查是否接收到音频数据
            if not audio_received:
                raise RuntimeError("No audio data received")

            logger.info("所有音频播放完成")

            # 结束连接
            await finish_connection(websocket)
            await wait_for_event(
                websocket, MsgType.FullServerResponse, EventType.ConnectionFinished
            )
            await websocket.close()

        except Exception as e:
            logger.error(f"TTS 合成或播放失败: {e}")
            return

        # 发布播放完成状态
        self.get_logger().info("发布 TTS 完成状态")
        status_msg = Bool()
        status_msg.data = True
        self._status_pub.publish(status_msg)

    def destroy_node(self) -> None:
        """清理资源."""
        if self._audio_device:
            self._audio_device.cleanup()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = G1TTSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
