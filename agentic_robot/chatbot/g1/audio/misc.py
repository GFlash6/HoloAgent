import pyaudio
import wave
import time
import uuid
import asyncio
import numpy as np
import websockets

from .logging import default_logger as logger
from .audio_device import AudioDevice
from .volcengine_doubao_tts import (
    EventType,
    MsgType,
    start_connection,
    wait_for_event,
)


def list_audio_devices(device: str = "USB"):
    """返回匹配的音频设备列表。"""
    device_list = []

    p = pyaudio.PyAudio()
    try:
        for i in range(p.get_device_count()):
            dev = p.get_device_info_by_index(i)
            if device == "all":
                device_list.append(dev)
            elif device in dev["name"]:
                device_list.append(dev)
    finally:
        p.terminate()

    return device_list


def resample_pcm(pcm_data, src_rate, dst_rate, channels):
    """
    使用线性插值将int16 PCM数据从src_rate重采样到dst_rate。

    Args:
        pcm_data: 原始PCM数据(bytes), int16格式
        src_rate: 原始采样率
        dst_rate: 目标采样率
        channels: 声道数

    Returns:
        重采样后的PCM数据(bytes), int16格式
    """
    if src_rate == dst_rate or not pcm_data:
        return pcm_data

    audio = np.frombuffer(pcm_data, dtype=np.int16)

    if channels > 1:
        # [num_samples * channels] -> [num_frames, channels]
        num_frames = audio.size // channels
        if num_frames == 0:
            return b""
        audio = audio[: num_frames * channels].reshape(num_frames, channels)
    else:
        if audio.size == 0:
            return b""
        num_frames = audio.size
        audio = audio.reshape(num_frames, 1)

    src_len = audio.shape[0]
    dst_len = max(1, int(round(src_len * dst_rate / src_rate)))

    # 构造时间轴，使用线性插值进行重采样
    x_old = np.linspace(0.0, 1.0, src_len, endpoint=False)
    x_new = np.linspace(0.0, 1.0, dst_len, endpoint=False)

    resampled = np.empty((dst_len, channels), dtype=np.float32)
    for ch in range(channels):
        resampled[:, ch] = np.interp(
            x_new, x_old, audio[:, ch].astype(np.float32))

    # 限幅并转换回int16
    resampled = np.clip(resampled, -32768, 32767).astype(np.int16)

    if channels == 1:
        resampled = resampled.reshape(-1)

    return resampled.tobytes()


def create_wav_chunk(pcm_data, sample_rate, channels):
    """
    创建WAV格式的音频数据.

    Args:
        pcm_data: PCM原始音频数据(bytes)
        sample_rate: 采样率(Hz)
        channels: 声道数

    Returns:
        WAV格式的音频数据(bytes)
    """
    import io

    # 创建内存缓冲区
    buffer = io.BytesIO()

    # 写入WAV文件头和数据
    with wave.open(buffer, 'wb') as wav_file:
        wav_file.setnchannels(channels)
        wav_file.setsampwidth(2)  # 16-bit，每个样本2字节
        wav_file.setframerate(sample_rate)
        wav_file.writeframes(pcm_data)

    # 返回完整的WAV文件数据
    return buffer.getvalue()


async def realtime_audio_generator(
        audio_device: AudioDevice,
        duration_seconds: int = None,
        chunk_duration_ms: int = 200,
        sample_rate: int = None):
    """
    实时音频生成器, 从AudioDevice录音并产生音频片段.

    Args:
        audio_device: AudioDevice实例
        duration_seconds: 录音时长(秒), None表示无限录音
        chunk_duration_ms: 每个音频片段的时长(毫秒)
        sample_rate: 目标采样率(Hz), None表示使用AudioDevice的采样率

    Yields:
        音频数据片段(bytes), WAV格式
    """
    # 获取音频设备参数
    device_sample_rate = audio_device.sample_rate
    channels = audio_device.channels
    chunk_size = audio_device.chunk_size

    # 目标输出采样率
    target_sample_rate = sample_rate if sample_rate else device_sample_rate

    # 计算每个chunk的字节数
    samples_per_chunk = chunk_size
    bytes_per_sample = 2  # int16格式，每个样本2字节
    bytes_per_chunk = samples_per_chunk * channels * bytes_per_sample

    # 计算需要累积多少个chunk才能达到目标时长
    target_bytes = int(
        device_sample_rate *
        channels *
        bytes_per_sample *
        chunk_duration_ms /
        1000)
    chunks_needed = max(1, target_bytes // bytes_per_chunk)

    # logger.info(
    #     f"实时录音配置: 设备采样率={device_sample_rate}, 输出采样率={target_sample_rate}, "
    #     f"块大小={chunk_size}, 每{chunk_duration_ms}ms发送一次, bytes_per_chunk={bytes_per_chunk}"
    # )
    # logger.info(f"每次发送需要累积 {chunks_needed} 个chunk, 约 {target_bytes} 字节")

    # 初始化录音状态
    loop = asyncio.get_event_loop()
    start_time = loop.time()
    accumulated_data = b''
    chunk_count = 0
    yield_count = 0
    total_get_count = 0
    timeout_count = 0
    consecutive_timeouts = 0
    last_yield_time = start_time

    try:
        while True:
            iter_start = loop.time()

            # 检查是否达到录音时长限制
            if duration_seconds is not None:
                elapsed = iter_start - start_time
                if elapsed >= duration_seconds:
                    logger.info(f"录音时长达到 {duration_seconds} 秒，停止录音")
                    break

            # 检查 arecord 进程是否仍然存活
            if audio_device.input_stream and audio_device.input_stream.poll() is not None:
                logger.error(
                    f"arecord进程已退出! returncode={audio_device.input_stream.returncode}")
                break

            # 异步获取录音数据，超时时间1秒
            total_get_count += 1
            audio_chunk = await audio_device.async_get_recorded_data(timeout=1.0)
            get_duration = loop.time() - iter_start

            if audio_chunk is None:
                timeout_count += 1
                consecutive_timeouts += 1
                logger.warning(
                    f"录音数据获取超时: get耗时={get_duration:.3f}s, "
                    f"连续超时={consecutive_timeouts}, 累计超时={timeout_count}/{total_get_count}, "
                    f"recording_queue_size={audio_device.get_recording_queue_size()}, "
                    f"arecord_alive={audio_device.input_stream.poll() is None if audio_device.input_stream else False}")
                if consecutive_timeouts >= 2:
                    read_age = time.monotonic() - audio_device._input_read_last_time
                    logger.error(
                        f"连续{consecutive_timeouts}次超时, 距上次读取={read_age:.1f}s, "
                        f"尝试重启录音流..."
                    )

                    recovered = False
                    for attempt in range(3):
                        ok = audio_device.restart_input_stream()
                        if not ok:
                            logger.error(f"重启尝试 {attempt+1}/3 失败")
                            continue

                        old_read_count = audio_device._input_read_count
                        for _ in range(10):
                            await asyncio.sleep(0.1)
                            if audio_device._input_read_count > old_read_count:
                                break

                        if audio_device._input_read_count > old_read_count:
                            logger.info(
                                f"重启尝试 {attempt+1}/3 成功, "
                                f"read_count={audio_device._input_read_count}")
                            recovered = True
                            break
                        else:
                            logger.warning(f"重启尝试 {attempt+1}/3: 读取未恢复, 再试")

                    if recovered:
                        consecutive_timeouts = 0
                        accumulated_data = b''
                        chunk_count = 0
                    else:
                        logger.error("3次重启均失败, 终止录音")
                        break
                continue

            consecutive_timeouts = 0

            # 累积音频数据
            accumulated_data += audio_chunk
            chunk_count += 1

            # 当累积足够的数据后，转换为WAV格式并发送
            if chunk_count >= chunks_needed:
                pcm_data = accumulated_data
                pcm_len = len(pcm_data)

                if target_sample_rate != device_sample_rate:
                    resample_start = loop.time()
                    pcm_data = resample_pcm(
                        pcm_data,
                        device_sample_rate,
                        target_sample_rate,
                        channels,
                    )
                    resample_cost = loop.time() - resample_start
                    logger.debug(
                        f"重采样耗时: {resample_cost:.4f}s, {pcm_len} -> {len(pcm_data)} bytes")

                wav_data = create_wav_chunk(
                    pcm_data, target_sample_rate, channels)
                yield_count += 1
                now = loop.time()
                gap = now - last_yield_time
                last_yield_time = now

                logger.debug(
                    f"yield音频包 #{yield_count}: wav_size={len(wav_data)}, pcm_size={pcm_len}, "
                    f"距上次yield={gap:.3f}s, 累积chunks={chunk_count}, "
                    f"queue_size={audio_device.get_recording_queue_size()}")

                if gap > (chunk_duration_ms / 1000) * 2:
                    pass
                    # logger.warning(
                    #     f"yield间隔过大! 期望<={chunk_duration_ms}ms, 实际={gap*1000:.0f}ms, "
                    #     f"可能导致服务端超时"
                    # )

                yield wav_data

                accumulated_data = b''
                chunk_count = 0

        # 发送剩余的音频数据
        if accumulated_data:
            pcm_data = accumulated_data
            if target_sample_rate != device_sample_rate:
                pcm_data = resample_pcm(
                    pcm_data,
                    device_sample_rate,
                    target_sample_rate,
                    channels,
                )
            wav_data = create_wav_chunk(pcm_data, target_sample_rate, channels)
            yield_count += 1
            logger.info(
                f"yield最后的音频包 #{yield_count}: wav_size={len(wav_data)}")
            yield wav_data

        logger.info(
            f"录音生成器结束: 总yield={yield_count}, 总get={total_get_count}, "
            f"超时次数={timeout_count}, 总时长={loop.time()-start_time:.1f}s"
        )

        # 发送结束信号
        yield None

    except Exception as e:
        logger.error(f"录音生成器错误: {type(e).__name__}: {e}", exc_info=True)
        raise


async def ws_connect(url, headers, **kwargs):
    """
    建立 WebSocket 连接，自动适配不同版本的请求头参数名。

    - websockets 14+：使用 additional_headers
    - websockets 13.x: 使用 extra_headers

    Args:
        url: WebSocket 服务地址
        headers: 请求头字典（如 X-Api-* 等）
        **kwargs: 其他传给 websockets.connect 的参数（如 max_size, ping_interval)

    Returns:
        WebSocket 连接对象
    """
    try:
        return await websockets.connect(url, additional_headers=headers, **kwargs)
    except TypeError:
        return await websockets.connect(url, extra_headers=headers, **kwargs)


def is_ws_connection_closed(conn):
    """
    判断 WebSocket 连接是否已关闭，兼容 13.x 与 14+。

    - websockets 13.x: 使用 connection.closed
    - websockets 14+: 使用 connection.state is State.CLOSED (无 .closed 属性)

    Args:
        conn: WebSocket 连接对象或 None

    Returns:
        True 表示无需使用conn 为 None)或连接已关闭, False 表示连接仍打开
    """
    if conn is None:
        return True
    if hasattr(conn, "closed"):
        return conn.closed
    try:
        from websockets.protocol import State
        return getattr(conn, "state", None) is State.CLOSED
    except Exception:
        return True


async def create_websocket_connection(
        tts_appid,
        tts_access_token,
        tts_resource_id,
        tts_endpoint):
    """
    创建 WebSocket 连接到 TTS 服务.

    Returns:
        websockets.WebSocketClientProtocol: WebSocket 连接对象

    Raises:
        Exception: 连接失败时抛出异常
    """
    # ========== 构建 WebSocket 连接请求头 ==========
    headers = {
        "X-Api-App-Key": tts_appid,  # TTS应用ID
        "X-Api-Access-Key": tts_access_token,  # TTS访问令牌
        "X-Api-Resource-Id": (tts_resource_id),  # TTS资源ID（模型ID）
        "X-Api-Connect-Id": str(uuid.uuid4()),  # 生成唯一的连接 ID
    }

    # ========== 连接到 WebSocket 服务器 ==========
    # 使用更宽松的 ping 配置，避免长时间运行后超时
    websocket = await ws_connect(
        tts_endpoint,
        headers,
        max_size=10 * 1024 * 1024,  # 最大消息大小10MB（用于接收音频数据）
        ping_interval=30,  # 每30秒发送一次ping保持连接活跃（从20秒增加到30秒）
        ping_timeout=20,   # ping超时时间20秒（从10秒增加到20秒，更宽松）
        close_timeout=10   # 关闭连接超时时间10秒
    )

    # ========== 启动连接 ==========
    await start_connection(websocket)
    await wait_for_event(
        websocket, MsgType.FullServerResponse, EventType.ConnectionStarted
    )

    return websocket

# async def realtime_audio_generator_forever(audio_device, chunk_duration_ms=200):
#     """无限持续生成音频数据"""
#     import pyaudio
#     import struct
#     import io
#     import wave

#     sample_width = pyaudio.get_sample_size(pyaudio.paInt16)
#     channels = audio_device.input_channels  # 注意属性名
#     sample_rate = audio_device.input_sample_rate

#     frames_per_chunk = int(sample_rate * chunk_duration_ms / 1000)
#     bytes_per_frame = sample_width * channels
#     bytes_per_chunk = frames_per_chunk * bytes_per_frame

#     audio_queue = audio_device.get_input_queue()

#     while True:
#         audio_frames = []
#         bytes_collected = 0

#         while bytes_collected < bytes_per_chunk:
#             data = await audio_queue.get()
#             audio_frames.append(data)
#             bytes_collected += len(data)

#         audio_data = b''.join(audio_frames)

#         # 包装成 WAV
#         with io.BytesIO() as wav_io:
#             with wave.open(wav_io, 'wb') as wav_file:
#                 wav_file.setnchannels(channels)
#                 wav_file.setsampwidth(sample_width)
#                 wav_file.setframerate(sample_rate)
#                 wav_file.writeframes(audio_data)
#             wav_data = wav_io.getvalue()

#         yield wav_data
