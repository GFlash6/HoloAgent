import argparse
import time
import os
import wave
import numpy as np

from audio.audio_device import AudioDevice
from audio.logging import default_logger as logger


def record_and_playback(duration: float = 5.0, monitor: bool = True) -> None:
    """
    使用 AudioDevice 录音一段时间：
    1. 实时从扬声器监听
    2. 同时把录音内容保存为 WAV 文件
    """
    device = AudioDevice()

    try:
        device.start_streams()
        logger.info(f"开始录音并实时回放 {duration} 秒，请对着麦克风说话...")

        frames = []  # 经过限幅后的录音数据，用于最终保存

        # 目标采样点数
        target_frames = int(duration * device.sample_rate)
        collected_frames = 0

        # 录音阶段：从 recording_queue 取数据，同时立刻送入播放队列，实现实时监听
        # 流程：
        #   1. 先做软件限幅，避免数字削波带来的爆破音
        #   2. 保存到 frames（用于写 WAV）
        #   3. 对要实时播放的音频再做简单低通 + 衰减，减少啸叫和高频噪声
        # 循环直到累计的采样点数达到目标长度
        while collected_frames < target_frames:
            data = device.get_recorded_data(block=True, timeout=1.0)
            if data:
                # 步骤 1：软件限幅（limiter），先在 float32 域操作
                audio_f = np.frombuffer(
                    data, dtype=np.int16).astype(
                    np.float32)
                peak = np.max(np.abs(audio_f)) if audio_f.size > 0 else 0.0
                # 目标峰值略低于 int16 上限，留更多 headroom，进一步减少爆破感
                target_peak = 22000.0
                if peak > target_peak and peak > 0:
                    gain = target_peak / peak
                    audio_f *= gain
                # 转回 int16，作为“干净版本”保存和后续处理的基础
                limited_int16 = np.clip(
                    audio_f, -32768, 32767).astype(np.int16)

                # 步骤 2：保存到 frames，用于写 WAV（这样导出的文件也不会数字削波）
                frames.append(limited_int16.tobytes())
                collected_frames += limited_int16.size

                # 步骤 3：实时回放用的版本，再做简单低通 + 额外衰减
                play_f = limited_int16.astype(np.float32)
                if play_f.size > 4:
                    kernel = np.ones(5, dtype=np.float32) / 5.0
                    play_f = np.convolve(play_f, kernel, mode="same")
                if monitor:
                    # 监听音量再降低一点，减少声学反馈
                    play_f *= 1
                    play_int16 = np.clip(
                        play_f, -32768, 32767).astype(np.int16)
                    device.put_playback_data(play_int16.tobytes())

        # 录音时间结束后，把录音队列里尚未消费完的数据也读出来，避免只保存到一部分音频
        while True:
            extra = device.get_recorded_data(block=False)
            if not extra:
                break
            audio_f = np.frombuffer(extra, dtype=np.int16).astype(np.float32)
            peak = np.max(np.abs(audio_f)) if audio_f.size > 0 else 0.0
            target_peak = 22000.0
            if peak > target_peak and peak > 0:
                gain = target_peak / peak
                audio_f *= gain
            limited_int16 = np.clip(audio_f, -32768, 32767).astype(np.int16)
            frames.append(limited_int16.tobytes())

        logger.info(f"录音结束，期间采集到 {len(frames)} 个音频块（已实时回放）。")

        # 把录音内容保存为 WAV 文件（使用设备当前的采样率 / 通道 / 采样位宽）
        if frames:
            filename = f"record_{int(time.time())}.wav"
            filepath = os.path.abspath(filename)
            with wave.open(filepath, "wb") as wf:
                wf.setnchannels(device.channels)
                wf.setsampwidth(device.p.get_sample_size(device.format))
                wf.setframerate(device.sample_rate)
                all_data = b"".join(frames)
                wf.writeframes(all_data)

            bytes_per_second = 2 * device.channels * device.sample_rate
            approx_seconds = len(all_data) /\
                bytes_per_second if bytes_per_second > 0 else 0
            logger.info(
                f"录音内容已保存到: {filepath} "
                f"(按 header 估算时长约 {approx_seconds:.2f} 秒, sample_rate={device.sample_rate})")
        else:
            logger.info("未采集到任何音频数据，未生成录音文件。")

    finally:
        device.cleanup()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="测试 AudioDevice 录音并回放。")
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="录音时长(秒), 默认 5 秒")
    args = parser.parse_args()

    try:
        record_and_playback(
            duration=args.duration if args.duration is not None else 1000000000)
    except KeyboardInterrupt:
        logger.info("\n用户中断")
