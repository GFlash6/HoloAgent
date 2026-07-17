import pyaudio
import queue
import asyncio
import time
import threading
import subprocess
import os
from typing import Union

from .logging import default_logger as logger


class AudioDevice:
    """音频设备管理类."""

    def __init__(
            self,
            device_name="USB Audio Device",
            channels=1,
            chunk_size=512):
        """
        初始化音频设备.

        Args:
            device_name: 设备名称, 同时支持麦克风和扬声器, 默认"USB Audio Device"
            channels: 声道数
            chunk_size: 音频块大小(samples per chunk), 默认512
        """
        self.device_name = device_name
        self.channels = channels
        self.chunk_size = chunk_size

        self.p = pyaudio.PyAudio()

        self.device_index = self._find_device_index_by_name(self.device_name)
        assert self.device_index is not None, f"未找到设备: {self.device_name}"
        logger.info(f"找到{self.device_name}设备, 索引: {self.device_index}")

        self.device_info = self.p.get_device_info_by_index(self.device_index)
        logger.debug(f"设备信息: {self.device_info}")
        self.format = pyaudio.paInt16
        self.sample_rate = int(
            self.device_info.get(
                "defaultSampleRate", 44100.0))

        # 获取 ALSA 硬件设备名（用于 arecord）
        self._alsa_device = self._get_alsa_device_name()
        logger.debug(f"ALSA设备名: {self._alsa_device}")

        # 音频流
        self.input_stream = None    # arecord 子进程（录音）
        self.output_stream = None   # aplay 子进程（播放）

        # 播放队列和同步录音队列
        self.playback_queue = queue.Queue()
        self.recording_queue = queue.Queue()

        # 标识音频流是否正在运行
        self.is_running = False

        # 录音读取线程
        self._reader_thread = None
        self._reader_running = False
        self._input_read_count = 0
        self._input_read_last_time = 0.0

        # 播放写入线程
        self._writer_thread = None
        self._writer_running = False

        # 异步录音队列
        self._async_queue = None
        self._async_loop = None

    def _find_device_index_by_name(
            self, name_keyword: str) -> Union[int, None]:
        """根据设备名关键字查找设备索引（模糊匹配，大小写不敏感）"""
        try:
            count = self.p.get_device_count()
            for i in range(count):
                info = self.p.get_device_info_by_index(i)
                device_name = str(info.get("name", ""))
                if name_keyword.lower() not in device_name.lower():
                    continue
                if info.get("maxInputChannels", 0) <= 0:
                    continue
                if info.get("maxOutputChannels", 0) <= 0:
                    continue
                return i
        except Exception as e:
            logger.error(f"根据关键字查找音频设备失败: {e}")
        return None

    def _get_alsa_device_name(self) -> str:
        """根据 PyAudio device_index 推断 ALSA 硬件设备名."""
        info = self.device_info
        name = info.get("name", "")
        # PyAudio 名字格式通常是 "USB Audio Device: - (hw:0,0)"
        if "hw:" in name:
            start = name.index("hw:")
            end = name.index(")", start) if ")" in name[start:] else len(name)
            return name[start:end]  # 返回hw:0,0
        # 回退：根据 card index 猜测
        try:
            count = self.p.get_device_count()
            card_idx = 0
            for i in range(count):
                dev = self.p.get_device_info_by_index(i)
                if dev.get(
                    "maxInputChannels",
                        0) > 0 and dev.get("hostApi") == 0:
                    if i == self.device_index:
                        return f"hw:{card_idx},0"
                    card_idx += 1
        except Exception:
            pass
        return "hw:0,0"

    def _start_arecord(self):
        """启动 arecord 子进程进行录音."""
        self.input_stream = subprocess.Popen(
            ['arecord', '-D', self._alsa_device,
             '-f', 'S16_LE',
             '-r', str(self.sample_rate),
             '-c', str(self.channels),
             '-t', 'raw',
             '--buffer-size=4096'],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0
        )
        logger.info(
            f"arecord 已启动: pid={self.input_stream.pid}, "
            f"device={self._alsa_device}, rate={self.sample_rate}, ch={self.channels}")

    def _reader_loop(self):
        """录音读取线程：从 arecord stdout 读取 PCM 数据， 通过 call_soon_threadsafe 传给
        asyncio.Queue。"""
        logger.info("录音读取线程已启动")
        bytes_per_chunk = self.chunk_size * self.channels * 2  # 16bit = 2 bytes
        fd = self.input_stream.stdout.fileno()

        while self._reader_running:
            try:
                data = os.read(fd, bytes_per_chunk)
                if not data:
                    if self._reader_running:
                        logger.warning("arecord stdout EOF")
                    break
                self._input_read_count += 1
                self._input_read_last_time = time.monotonic()
                if self._async_loop is not None and self._async_queue is not None:
                    self._async_loop.call_soon_threadsafe(
                        self._async_queue.put_nowait, data)
                else:
                    self.recording_queue.put_nowait(data)
            except OSError as e:
                if self._reader_running:
                    logger.warning(f"录音读取OSError: {e}")
                break
            except Exception as e:
                if self._reader_running:
                    logger.error(f"录音读取线程异常: {type(e).__name__}: {e}")
                break
        logger.info(f"录音读取线程已退出, 总读取={self._input_read_count}")

    def _start_aplay(self):
        """启动 aplay 子进程进行播放."""
        # 使用 plughw 替代 hw，自动处理声道数/格式转换
        alsa_play_device = self._alsa_device.replace("hw:", "plughw:")
        self.output_stream = subprocess.Popen(
            ['aplay', '-D', alsa_play_device,
             '-f', 'S16_LE',
             '-r', str(self.sample_rate),
             '-c', str(self.channels),
             '-t', 'raw',
             '--buffer-size=4096'],
            stdin=subprocess.PIPE, stderr=subprocess.PIPE, bufsize=0
        )
        # logger.info(f"aplay 已启动: pid={self.output_stream.pid}, "
        # f"device={alsa_play_device}, rate={self.sample_rate},
        # ch={self.channels}")

    def _writer_loop(self):
        """播放写入线程：从播放队列取 PCM 数据，写入 aplay stdin。"""
        # logger.info("播放写入线程已启动")
        write_count = 0
        drop_count = 0
        while self._writer_running:
            try:
                data = self.playback_queue.get(timeout=0.5)
                if self.output_stream and self.output_stream.poll() is None:
                    self.output_stream.stdin.write(data)
                    self.output_stream.stdin.flush()
                    write_count += 1
                else:
                    drop_count += 1
                    aplay_rc = self.output_stream.poll() if self.output_stream else "no_stream"
                    if drop_count <= 5 or drop_count % 50 == 0:
                        logger.warning(
                            f"播放写入跳过: aplay已退出(returncode={aplay_rc}), "
                            f"drop_count={drop_count}, data_len={len(data)}")
            except queue.Empty:
                continue
            except (OSError, BrokenPipeError) as e:
                if self._writer_running:
                    logger.warning(
                        f"播放写入OSError: {e}, write_count={write_count}")
                break
            except Exception as e:
                if self._writer_running:
                    logger.error(
                        f"播放写入线程异常: {type(e).__name__}: {e}, write_count={write_count}")
                break
        # logger.info(f"播放写入线程已退出, write_count={write_count}, drop_count={drop_count}")

    def _open_input_stream(self):
        """启动录音: arecord 子进程 + 读取线程."""
        self._start_arecord()
        self._input_read_count = 0
        self._input_read_last_time = time.monotonic()
        self._reader_running = True
        self._reader_thread = threading.Thread(
            target=self._reader_loop, daemon=True, name="audio-reader"
        )
        self._reader_thread.start()

    def _open_output_stream(self):
        """启动播放: aplay 子进程 + 写入线程."""
        self._start_aplay()
        self._writer_running = True
        self._writer_thread = threading.Thread(
            target=self._writer_loop, daemon=True, name="audio-writer"
        )
        self._writer_thread.start()

    def start_streams(self, input_only=False, output_only=False):
        """
        启动音频流.

        Args:
            input_only: 仅启动录音流
            output_only: 仅启动播放流
        """
        try:
            logger.info(
                f"正在启动音频流: device={self._alsa_device}, "
                f"sample_rate={self.sample_rate}, channels={self.channels}, "
                f"chunk_size={self.chunk_size}")

            if not output_only:
                self._open_input_stream()
                logger.info("录音流已打开 (arecord)")

            if not input_only:
                self._open_output_stream()
                logger.info("播放流已打开 (aplay)")

            self.is_running = True
            logger.info("音频流启动完成")

        except Exception as e:
            logger.error(f"启动音频流失败, {e}")
            self.is_running = False
            raise

    def _stop_input(self):
        """停止录音（停线程 + 杀 arecord 进程）"""
        self._reader_running = False

        # 先杀 arecord，让 reader 线程的 os.read 返回 EOF
        if self.input_stream and self.input_stream.poll() is None:
            self.input_stream.terminate()
            try:
                self.input_stream.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.input_stream.kill()
                self.input_stream.wait(timeout=1.0)
        self.input_stream = None

        if self._reader_thread and self._reader_thread.is_alive():
            self._reader_thread.join(timeout=1.0)
        self._reader_thread = None

    def restart_input_stream(self):
        """重启录音."""
        logger.warning("正在重启录音流...")
        try:
            self._stop_input()

            # 清空异步队列
            if self._async_queue is not None:
                while not self._async_queue.empty():
                    try:
                        self._async_queue.get_nowait()
                    except asyncio.QueueEmpty:
                        break

            self._open_input_stream()
            logger.info("录音流重启成功")
            return True
        except Exception as e:
            logger.error(f"录音流重启失败: {e}")
            return False

    def _stop_output(self):
        """停止播放（停线程 + 杀 aplay 进程）"""
        self._writer_running = False

        if self.output_stream and self.output_stream.poll() is None:
            # 先杀进程，让阻塞在 stdin.write() 的线程收到 BrokenPipeError
            self.output_stream.terminate()
            try:
                self.output_stream.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                self.output_stream.kill()
                self.output_stream.wait(timeout=1.0)
            try:
                self.output_stream.stdin.close()
            except Exception:
                pass
        self.output_stream = None

        if self._writer_thread and self._writer_thread.is_alive():
            self._writer_thread.join(timeout=2.0)
            if self._writer_thread.is_alive():
                logger.warning("播放写入线程未能在2秒内退出")
        self._writer_thread = None

    def stop_streams(self):
        """停止所有音频流."""
        self.is_running = False
        self._stop_input()
        self._stop_output()

        logger.info("音频流已停止")

    def cleanup(self):
        """清理资源."""
        self.stop_streams()
        try:
            self.p.terminate()
        except Exception:
            pass
        logger.info("资源已清理")

    def put_playback_data(self, audio_data):
        """添加音频数据到播放队列."""
        self.playback_queue.put(audio_data)

    def get_recorded_data(self, block=True, timeout=None):
        """从录音队列获取音频数据（同步方式）"""
        try:
            raw = self.recording_queue.get(block=block, timeout=timeout)
            return raw
        except queue.Empty:
            return None

    def clear_playback_queue(self):
        """清空播放队列."""
        while not self.playback_queue.empty():
            try:
                self.playback_queue.get_nowait()
            except queue.Empty:
                break

    def clear_playback_buffer(self):
        """清空当前播放缓冲（aplay 模式下重启 aplay 以清空内核缓冲）"""
        if self.output_stream and self.output_stream.poll() is None:
            self._stop_output()
            self._open_output_stream()

    def clear_recording_queue(self):
        """清空录音队列."""
        while not self.recording_queue.empty():
            try:
                self.recording_queue.get_nowait()
            except queue.Empty:
                break

    def get_recording_queue_size(self):
        """获取录音队列大小."""
        return self.recording_queue.qsize()

    def get_playback_queue_size(self):
        """获取播放队列大小."""
        return self.playback_queue.qsize()

    async def async_get_recorded_data(self, timeout=None):
        """异步方式从录音队列获取音频数据."""
        block_timeout = timeout if timeout else 1.0

        if self._async_queue is None:
            self._async_loop = asyncio.get_event_loop()
            self._async_queue = asyncio.Queue()

        try:
            raw = await asyncio.wait_for(self._async_queue.get(), timeout=block_timeout)
            return raw
        except asyncio.TimeoutError:
            async_qsize = self._async_queue.qsize() if self._async_queue else 0
            proc_alive = self.input_stream.poll() is None if self.input_stream else False
            read_count = self._input_read_count
            now = time.monotonic()
            read_age = now - self._input_read_last_time if self._input_read_last_time > 0 else -1
            reader_alive = self._reader_thread.is_alive() if self._reader_thread else False
            logger.warning(
                f"录音队列获取超时({block_timeout}s): async_qsize={async_qsize}, "
                f"arecord_alive={proc_alive}, is_running={self.is_running}, "
                f"read_count={read_count}, 距上次读取={read_age:.3f}s, "
                f"reader_alive={reader_alive}"
            )
            return None
        except Exception as e:
            logger.error(f"异步获取录音数据失败: {type(e).__name__}: {e}")
            return None
