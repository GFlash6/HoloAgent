#!/usr/bin/env python3
"""豆包 TTS (Text-to-Speech) 测试脚本 通过 WebSocket 连接到豆包 TTS 服务，将文本转换为语音
支持实时音频播放功能."""

import argparse
import asyncio
import copy
import json
import uuid

from audio.env import SETTINGS
from audio.logging import default_logger as logger
from audio.audio_device import AudioDevice
from audio.misc import create_websocket_connection, ws_connect
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

TTS_APP_KEY = SETTINGS["tts"]["app_id"]               # 豆包语音合成KEY
TTS_ACCESS_KEY = SETTINGS["tts"]["access_key"]        # 豆包语音合成KEY
TTS_RESOURCE_ID = SETTINGS["tts"]["resource_id"]     # 豆包语音合成资源ID
TTS_ENDPOINT = SETTINGS["tts"]["endpoint"]           # 豆包语音合成endpoint
TTS_VOICE_TYPE = SETTINGS["tts"]["voice_type"]       # 豆包语音合成语音类型
TTS_RESOURCE_ID = SETTINGS["tts"]["resource_id"]     # 豆包语音合成资源ID
TTS_ENDPOINT = SETTINGS["tts"]["endpoint"]           # 豆包语音合成endpoint


async def main():
    """
    主函数：连接豆包 TTS 服务，将文本转换为语音.

    流程：
    1. 解析命令行参数
    2. 初始化音频设备
    3. 建立 WebSocket 连接
    4. 启动连接会话
    5. 按句子处理文本，逐字符发送并接收音频
    6. 实时播放音频
    7. 清理资源并关闭连接
    """
    parser = argparse.ArgumentParser(description='豆包 TTS 测试工具')
    parser.add_argument(
        "--text",
        default="你好，我是一个机器人，我可以帮助你完成各种任务。",
        help="Text to convert")
    parser.add_argument(
        "--repeat",
        type=int,
        default=2,
        help="播放text的次数 (默认: 4)")
    parser.add_argument(
        "--interval",
        type=float,
        default=2.0,
        help="每次播放之间的间隔秒数 (默认: 4.0)")
    args = parser.parse_args()

    # 初始化音频设备
    audio_device = AudioDevice()
    audio_device.start_streams()

    websocket = await create_websocket_connection(TTS_APP_KEY, TTS_ACCESS_KEY, TTS_RESOURCE_ID, TTS_ENDPOINT)

    try:
        for round_idx in range(args.repeat):
            if round_idx > 0:
                logger.info(
                    f"等待 {args.interval} 秒后开始第 {round_idx + 1}/{args.repeat} 次播放...")
                await asyncio.sleep(args.interval)

            logger.info(f"开始第 {round_idx + 1}/{args.repeat} 次播放")

            # 非首次播放时，重启 aplay 以清空内核缓冲区，避免 write 阻塞
            if round_idx > 0:
                logger.info("重启播放流以确保干净状态...")
                audio_device.clear_playback_queue()
                audio_device._stop_output()
                audio_device._open_output_stream()
                logger.info("播放流已重启")

            aplay_alive = audio_device.output_stream and audio_device.output_stream.poll() is None
            writer_alive = audio_device._writer_thread and audio_device._writer_thread.is_alive()
            logger.info(
                f"aplay状态: alive={aplay_alive}, writer_thread_alive={writer_alive}, "
                f"playback_queue_size={audio_device.get_playback_queue_size()}")

            # 按句号分割文本，逐句处理
            sentences = args.text.split("。")
            audio_received = False  # 标记是否接收到音频数据

            for i, sentence in enumerate(sentences):
                if not sentence:
                    continue

                # 构建基础请求参数（每个会话可以有不同的参数）
                base_request = {
                    "user": {
                        "uid": str(uuid.uuid4()),  # 生成唯一的用户 ID
                    },
                    "namespace": "BidirectionalTTS",
                    "req_params": {
                        "speaker": TTS_VOICE_TYPE,  # 语音类型
                        "audio_params": {
                            "format": "pcm",  # 音频编码格式
                            "sample_rate": audio_device.sample_rate,  # 服务器端采样率（与设备播放采样率保持一致）
                            "enable_timestamp": True,  # 启用时间戳
                        },
                        "additions": json.dumps(
                            {
                                "disable_markdown_filter": False,  # 不禁用 Markdown 过滤
                            }
                        ),
                    },
                }

                # 启动会话
                start_session_request = copy.deepcopy(base_request)
                start_session_request["event"] = EventType.StartSession
                session_id = str(uuid.uuid4())  # 生成唯一的会话 ID
                await start_session(
                    websocket, json.dumps(
                        start_session_request).encode(), session_id
                )
                await wait_for_event(
                    websocket, MsgType.FullServerResponse, EventType.SessionStarted
                )

                # 逐字符发送文本（异步函数）
                async def send_chars():
                    for char in sentence:
                        synthesis_request = copy.deepcopy(base_request)
                        synthesis_request["event"] = EventType.TaskRequest
                        synthesis_request["req_params"]["text"] = char
                        await task_request(
                            websocket, json.dumps(
                                synthesis_request).encode(), session_id
                        )
                        await asyncio.sleep(0.005)  # 字符间延迟 5ms，避免发送过快

                    # 发送会话结束请求
                    await finish_session(websocket, session_id)

                # 在后台任务中开始发送字符
                send_task = asyncio.create_task(send_chars())

                # 接收音频数据
                audio_data = bytearray()  # 累积所有接收到的音频数据
                playback_started = False  # 是否已开始播放

                # 循环接收消息直到会话结束
                while True:
                    msg = await receive_message(websocket)

                    if msg.type == MsgType.FullServerResponse:
                        # 处理服务器完整响应消息
                        if msg.event == EventType.SessionFinished:
                            break
                    elif msg.type == MsgType.AudioOnlyServer:
                        # 处理音频数据消息
                        if not audio_received and len(msg.payload) > 0:
                            audio_received = True  # 标记已接收到音频

                        # 收集所有音频数据
                        audio_data.extend(msg.payload)

                        # 实时播放音频片段（优化的批量转换策略）
                        if audio_device and msg.payload:
                            try:
                                # PCM 格式：直接放入播放队列
                                audio_device.put_playback_data(msg.payload)
                                if not playback_started:
                                    playback_started = True
                                # logger.info(f"添加PCM数据: {len(msg.payload)} bytes, 队列: {audio_device.get_playback_queue_size()}")
                            except Exception as e:
                                logger.error(f"播放音频片段失败: {e}")
                    else:
                        raise RuntimeError(f"TTS conversion failed: {msg}")

                # 等待字符发送任务完成
                await send_task

                # 等待播放队列播放完成
                if audio_device:
                    q_size = audio_device.get_playback_queue_size()
                    aplay_ok = audio_device.output_stream and audio_device.output_stream.poll() is None
                    writer_ok = audio_device._writer_thread and audio_device._writer_thread.is_alive()
                    logger.info(
                        f"等待播放完成, queue={q_size}, aplay_alive={aplay_ok}, writer_alive={writer_ok}")
                    # 先等待队列有足够的数据
                    await asyncio.sleep(0.5)

                    # 然后等待队列播放完
                    max_wait = 40  # 最多等待 30 秒
                    wait_count = 0
                    while audio_device.get_playback_queue_size() > 0 and wait_count < max_wait * 10:
                        await asyncio.sleep(0.1)  # 每 100ms 检查一次
                        wait_count += 1
                        if wait_count % 50 == 0:
                            q_now = audio_device.get_playback_queue_size()
                            aplay_ok = audio_device.output_stream and audio_device.output_stream.poll() is None
                            writer_ok = audio_device._writer_thread and audio_device._writer_thread.is_alive()
                            logger.info(
                                f"仍在等待播放, queue={q_now}, aplay_alive={aplay_ok}, writer_alive={writer_ok}, waited={wait_count*0.1:.1f}s")

                    # 额外等待一段时间确保最后的音频播放完
                    await asyncio.sleep(1.0)
                    logger.info("音频播放完成")

                logger.info(f"会话 {i} 完成,总接收: {len(audio_data)} bytes")

            # 检查是否接收到音频数据
            if not audio_received:
                raise RuntimeError("No audio data received")

            logger.info(f"第 {round_idx + 1}/{args.repeat} 次播放完成")

    finally:
        # 结束连接
        await finish_connection(websocket)
        msg = await wait_for_event(
            websocket, MsgType.FullServerResponse, EventType.ConnectionFinished
        )
        await websocket.close()
        logger.info("Connection closed")

        # 清理音频设备资源
        if audio_device:
            audio_device.cleanup()
            logger.info("音频设备已清理")


if __name__ == "__main__":
    asyncio.run(main())
