#!/usr/bin/env python3
"""
Qwen ASR + OpenClaw TUI
"""

import asyncio
import subprocess
import json
import uuid
import signal

# 导入你原有的模块
from audio.audio_device import AudioDevice
from audio.qwen_asr import AsrWsClient
from audio.misc import realtime_audio_generator  # 用你原来的函数


def send_to_openclaw(text: str):
    """发送消息到 OpenClaw TUI."""
    try:
        params = json.dumps({
            "message": text,
            "sessionKey": "main",
            "idempotencyKey": str(uuid.uuid4())
        })
        subprocess.run(
            ["openclaw", "gateway", "call", "chat.send", "--params", params],
            capture_output=True,
            text=True,
            timeout=10
        )
        print(f"\n🎤 {text}")
    except Exception as e:
        print(f"\n❌ {e}")


async def run_asr():
    """持续运行 ASR - 用你原来的方式"""
    audio = AudioDevice()
    audio.start_streams(input_only=True)

    asr_url = "wss://dashscope.aliyuncs.com/api-ws/v1/realtime?model=qwen3-asr-flash-realtime"
    last_text = ""

    print("🎤 持续录音中，按 Ctrl+C 停止")

    # 你原来的循环逻辑
    while True:
        try:
            async with AsrWsClient(asr_url, 200) as client:
                # 用你原来的生成器
                audio_stream = realtime_audio_generator(
                    audio,
                    duration_seconds=5,  # 每5秒一段，但外层 while True 会重新连接
                    chunk_duration_ms=200,
                    sample_rate=16000,
                )

                async for response in client.execute_stream(audio_stream):
                    resp = response.to_dict()
                    result = resp.get('payload_msg', {}).get('result', {})
                    text = result.get('text', '')

                    if not text:
                        continue

                    utterances = result.get('utterances', [])
                    is_final = utterances[0].get(
                        'definite', False) if utterances else False

                    if is_final and text.strip() and text != last_text:
                        last_text = text
                        print(f"\n✅ {text}")
                        send_to_openclaw(text)
                    elif not is_final:
                        print(f"\r📝 {text}", end="", flush=True)

        except Exception as e:
            print(f"\n⚠️ 重连: {e}")
            await asyncio.sleep(1)


def main():
    try:
        asyncio.run(run_asr())
    except KeyboardInterrupt:
        print("\n🛑 停止")


if __name__ == "__main__":
    main()
