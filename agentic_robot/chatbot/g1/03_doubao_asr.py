#!/usr/bin/env python3
"""豆包ASR测试脚本."""

import argparse
import asyncio

from audio.logging import default_logger as logger
from audio.audio_device import AudioDevice
from audio.volcengine_doubao_asr import AsrWsClient
from audio.misc import realtime_audio_generator


async def test_realtime_asr(url: str, seg_duration: int, duration: int):
    """
    测试实时录音ASR识别.

    Args:
        url: WebSocket服务器URL
        seg_duration: 音频片段时长(毫秒)
        duration: 录音时长(秒)
    """
    logger.info("=== 测试实时录音ASR ===")

    # 创建音频设备，配置录音参数
    audio_device = AudioDevice()

    try:
        # 仅启动录音流，不启动播放流（避免USB Full Speed带宽冲突导致回调停止）
        audio_device.start_streams(input_only=True)
        logger.info(f"开始录音，时长: {duration} 秒")
        logger.info("请开始说话...")

        # 创建ASR客户端并开始识别
        async with AsrWsClient(url, seg_duration) as client:
            # 创建实时音频生成器，将录音数据转换为WAV格式片段
            audio_stream = realtime_audio_generator(
                audio_device,
                duration_seconds=duration,
                chunk_duration_ms=seg_duration
            )

            # 开始实时ASR识别，处理音频流
            try:
                async for response in client.execute_stream(audio_stream):
                    # 解析响应数据
                    resp_dict = response.to_dict()
                    logger.debug(f"response: {response.to_dict()}")
                    # 检查响应中是否包含识别结果
                    if resp_dict.get(
                            'payload_msg') and 'result' in resp_dict['payload_msg']:
                        result = resp_dict['payload_msg']['result']

                        # 如果包含识别文本，则输出
                        if 'text' in result:
                            text = result['text']
                            # 获取识别结果的确定状态（是否为最终结果）
                            is_definite = result.get('utterances', [{}])[0].get(
                                'definite', False) if result.get('utterances') else False
                            logger.info(
                                f"[{'确定' if is_definite else '临时'}] {text}")
            except Exception as e:
                logger.error(f"实时ASR处理失败: {e}")
                raise

    finally:
        # 清理音频设备资源
        audio_device.cleanup()
        logger.info("音频设备已清理")


async def main():
    """主函数，解析命令行参数并执行相应的测试模式."""
    parser = argparse.ArgumentParser(description="豆包ASR WebSocket客户端测试工具")
    parser.add_argument(
        "--url",
        type=str,
        default="wss://openspeech.bytedance.com/api/v3/sauc/bigmodel_async",
        help="WebSocket服务器URL")
    parser.add_argument("--seg_duration", type=int, default=200,
                        help="音频片段时长(毫秒), 默认: 200")
    parser.add_argument("--duration", type=int, default=None,
                        help="录音时长(秒), 仅用于realtime模式, 默认: 10")
    args = parser.parse_args()

    await test_realtime_asr(args.url, args.seg_duration, args.duration)


if __name__ == "__main__":
    asyncio.run(main())
