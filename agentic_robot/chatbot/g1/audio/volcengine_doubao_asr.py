import asyncio
import aiohttp
import json
import struct
import gzip
import uuid
import os
import subprocess
from typing import List, Dict, Any, Tuple, AsyncGenerator

from .logging import default_logger as logger
from .env import SETTINGS

ASR_APP_KEY = SETTINGS["asr"]["app_id"]
ASR_ACCESS_KEY = SETTINGS["asr"]["access_key"]
END_WINDOW_SIZE = SETTINGS["asr"]["end_window_size"]     # 检测到600ms静音后触发断句
SAMPLE_RATE = SETTINGS["asr"]["rate"]       # 采样率

DEFAULT_SAMPLE_RATE = 16000


class ProtocolVersion:
    V1 = 0b0001


class MessageType:
    CLIENT_FULL_REQUEST = 0b0001
    CLIENT_AUDIO_ONLY_REQUEST = 0b0010
    SERVER_FULL_RESPONSE = 0b1001
    SERVER_ERROR_RESPONSE = 0b1111


class MessageTypeSpecificFlags:
    NO_SEQUENCE = 0b0000
    POS_SEQUENCE = 0b0001
    NEG_SEQUENCE = 0b0010
    NEG_WITH_SEQUENCE = 0b0011


class SerializationType:
    NO_SERIALIZATION = 0b0000
    JSON = 0b0001


class CompressionType:
    GZIP = 0b0001


class Config:
    def __init__(self):
        # 填入控制台获取的app id和access token
        app_id = ASR_APP_KEY
        access_key = ASR_ACCESS_KEY
        assert app_id and access_key, "ASR_APP_KEY and ASR_ACCESS_KEY must be set"
        self.auth = {
            "app_key": app_id,
            "access_key": access_key
        }

    @property
    def app_key(self) -> str:
        return self.auth["app_key"]

    @property
    def access_key(self) -> str:
        return self.auth["access_key"]


config = Config()


class CommonUtils:
    @staticmethod
    def gzip_compress(data: bytes) -> bytes:
        return gzip.compress(data)

    @staticmethod
    def gzip_decompress(data: bytes) -> bytes:
        return gzip.decompress(data)

    @staticmethod
    def judge_wav(data: bytes) -> bool:
        if len(data) < 44:
            return False
        return data[:4] == b'RIFF' and data[8:12] == b'WAVE'

    @staticmethod
    def convert_wav_with_path(audio_path: str,
                              sample_rate: int = DEFAULT_SAMPLE_RATE) -> bytes:
        try:
            cmd = [
                "ffmpeg", "-v", "quiet", "-y", "-i", audio_path,
                "-acodec", "pcm_s16le", "-ac", "1", "-ar", str(sample_rate),
                "-f", "wav", "-"
            ]
            result = subprocess.run(
                cmd,
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE)

            # 尝试删除原始文件
            try:
                os.remove(audio_path)
            except OSError as e:
                logger.warning(f"Failed to remove original file: {e}")

            return result.stdout
        except subprocess.CalledProcessError as e:
            logger.error(f"FFmpeg conversion failed: {e.stderr.decode()}")
            raise RuntimeError(f"Audio conversion failed: {e.stderr.decode()}")

    @staticmethod
    def read_wav_info(data: bytes) -> Tuple[int, int, int, int, bytes]:
        if len(data) < 44:
            raise ValueError("Invalid WAV file: too short")

        # 解析WAV头
        chunk_id = data[:4]
        if chunk_id != b'RIFF':
            raise ValueError("Invalid WAV file: not RIFF format")

        format_ = data[8:12]
        if format_ != b'WAVE':
            raise ValueError("Invalid WAV file: not WAVE format")

        # 解析fmt子块
        audio_format = struct.unpack('<H', data[20:22])[0]
        num_channels = struct.unpack('<H', data[22:24])[0]
        sample_rate = struct.unpack('<I', data[24:28])[0]
        bits_per_sample = struct.unpack('<H', data[34:36])[0]

        # 查找data子块
        pos = 36
        while pos < len(data) - 8:
            subchunk_id = data[pos:pos + 4]
            subchunk_size = struct.unpack('<I', data[pos + 4:pos + 8])[0]
            if subchunk_id == b'data':
                wave_data = data[pos + 8:pos + 8 + subchunk_size]
                return (
                    num_channels,
                    bits_per_sample // 8,
                    sample_rate,
                    subchunk_size // (num_channels * (bits_per_sample // 8)),
                    wave_data
                )
            pos += 8 + subchunk_size

        raise ValueError("Invalid WAV file: no data subchunk found")


class AsrRequestHeader:
    def __init__(self):
        self.message_type = MessageType.CLIENT_FULL_REQUEST
        self.message_type_specific_flags = MessageTypeSpecificFlags.POS_SEQUENCE
        self.serialization_type = SerializationType.JSON
        self.compression_type = CompressionType.GZIP
        self.reserved_data = bytes([0x00])

    def with_message_type(self, message_type: int) -> 'AsrRequestHeader':
        self.message_type = message_type
        return self

    def with_message_type_specific_flags(
            self, flags: int) -> 'AsrRequestHeader':
        self.message_type_specific_flags = flags
        return self

    def with_serialization_type(
            self, serialization_type: int) -> 'AsrRequestHeader':
        self.serialization_type = serialization_type
        return self

    def with_compression_type(
            self,
            compression_type: int) -> 'AsrRequestHeader':
        self.compression_type = compression_type
        return self

    def with_reserved_data(self, reserved_data: bytes) -> 'AsrRequestHeader':
        self.reserved_data = reserved_data
        return self

    def to_bytes(self) -> bytes:
        header = bytearray()
        header.append((ProtocolVersion.V1 << 4) | 1)
        header.append((self.message_type << 4) |
                      self.message_type_specific_flags)
        header.append((self.serialization_type << 4) | self.compression_type)
        header.extend(self.reserved_data)
        return bytes(header)

    @staticmethod
    def default_header() -> 'AsrRequestHeader':
        return AsrRequestHeader()


class RequestBuilder:
    @staticmethod
    def new_auth_headers() -> Dict[str, str]:
        reqid = str(uuid.uuid4())
        return {
            "X-Api-Resource-Id": "volc.bigasr.sauc.duration",
            "X-Api-Request-Id": reqid,
            "X-Api-Access-Key": config.access_key,
            "X-Api-App-Key": config.app_key
        }

    @staticmethod
    def new_full_client_request(seq: int) -> bytes:  # 添加seq参数
        header = AsrRequestHeader.default_header()\
            .with_message_type_specific_flags(MessageTypeSpecificFlags.POS_SEQUENCE)

        payload = {
            "user": {
                "uid": "demo_uid"
            },
            "audio": {
                "format": "wav",
                "codec": "raw",
                "rate": SAMPLE_RATE,    # 修改
                "bits": 16,
                "channel": 1
            },
            "request": {
                "model_name": "bigmodel",
                "enable_itn": True,
                "enable_punc": True,
                "enable_ddc": True,
                "show_utterances": True,
                "enable_nonstream": True,
                "end_window_size": END_WINDOW_SIZE,  # 修改
                "force_to_speech_time": 1,
                "result_type": "single"

            }
        }

        payload_bytes = json.dumps(payload).encode('utf-8')
        compressed_payload = CommonUtils.gzip_compress(payload_bytes)
        payload_size = len(compressed_payload)

        request = bytearray()
        request.extend(header.to_bytes())
        request.extend(struct.pack('>i', seq))  # 使用传入的seq
        request.extend(struct.pack('>I', payload_size))
        request.extend(compressed_payload)

        return bytes(request)

    @staticmethod
    def new_audio_only_request(
            seq: int,
            segment: bytes,
            is_last: bool = False) -> bytes:
        header = AsrRequestHeader.default_header()
        if is_last:  # 最后一个包特殊处理
            header.with_message_type_specific_flags(
                MessageTypeSpecificFlags.NEG_WITH_SEQUENCE)
            seq = -seq  # 设为负值
        else:
            header.with_message_type_specific_flags(
                MessageTypeSpecificFlags.POS_SEQUENCE)
        header.with_message_type(MessageType.CLIENT_AUDIO_ONLY_REQUEST)

        request = bytearray()
        request.extend(header.to_bytes())
        request.extend(struct.pack('>i', seq))

        compressed_segment = CommonUtils.gzip_compress(segment)
        request.extend(struct.pack('>I', len(compressed_segment)))
        request.extend(compressed_segment)

        return bytes(request)


class AsrResponse:
    def __init__(self):
        self.code = 0
        self.event = 0
        self.is_last_package = False
        self.payload_sequence = 0
        self.payload_size = 0
        self.payload_msg = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "code": self.code,
            "event": self.event,
            "is_last_package": self.is_last_package,
            "payload_sequence": self.payload_sequence,
            "payload_size": self.payload_size,
            "payload_msg": self.payload_msg
        }


class ResponseParser:
    @staticmethod
    def parse_response(msg: bytes) -> AsrResponse:
        response = AsrResponse()

        header_size = msg[0] & 0x0f
        message_type = msg[1] >> 4
        message_type_specific_flags = msg[1] & 0x0f
        serialization_method = msg[2] >> 4
        message_compression = msg[2] & 0x0f

        payload = msg[header_size * 4:]

        # 解析message_type_specific_flags
        if message_type_specific_flags & 0x01:
            response.payload_sequence = struct.unpack('>i', payload[:4])[0]
            payload = payload[4:]
        if message_type_specific_flags & 0x02:
            response.is_last_package = True
        if message_type_specific_flags & 0x04:
            response.event = struct.unpack('>i', payload[:4])[0]
            payload = payload[4:]

        # 解析message_type
        if message_type == MessageType.SERVER_FULL_RESPONSE:
            response.payload_size = struct.unpack('>I', payload[:4])[0]
            payload = payload[4:]
        elif message_type == MessageType.SERVER_ERROR_RESPONSE:
            response.code = struct.unpack('>i', payload[:4])[0]
            response.payload_size = struct.unpack('>I', payload[4:8])[0]
            payload = payload[8:]

        if not payload:
            return response

        # 解压缩
        if message_compression == CompressionType.GZIP:
            try:
                payload = CommonUtils.gzip_decompress(payload)
            except Exception as e:
                logger.error(f"Failed to decompress payload: {e}")
                return response

        # 解析payload
        try:
            if serialization_method == SerializationType.JSON:
                response.payload_msg = json.loads(payload.decode('utf-8'))
        except Exception as e:
            logger.error(f"Failed to parse payload: {e}")

        return response


class AsrWsClient:
    def __init__(self, url: str, segment_duration: int = 200):
        self.seq = 1
        self.url = url
        self.segment_duration = segment_duration
        self.conn = None
        self.session = None  # 添加session引用

    async def __aenter__(self):
        self.session = aiohttp.ClientSession()
        return self

    async def __aexit__(self, exc_type, exc, tb):
        if self.conn and not self.conn.closed:
            await self.conn.close()
        if self.session and not self.session.closed:
            await self.session.close()

    async def read_audio_data(self, file_path: str) -> bytes:
        try:
            with open(file_path, 'rb') as f:
                content = f.read()

            if not CommonUtils.judge_wav(content):
                logger.info("Converting audio to WAV format...")
                content = CommonUtils.convert_wav_with_path(
                    file_path, DEFAULT_SAMPLE_RATE)

            return content
        except Exception as e:
            logger.error(f"Failed to read audio data: {e}")
            raise

    def get_segment_size(self, content: bytes) -> int:
        try:
            channel_num, samp_width, frame_rate, _, _ = CommonUtils.read_wav_info(content)[
                :5]
            size_per_sec = channel_num * samp_width * frame_rate
            segment_size = size_per_sec * self.segment_duration // 1000
            return segment_size
        except Exception as e:
            logger.error(f"Failed to calculate segment size: {e}")
            raise

    async def create_connection(self) -> None:
        headers = RequestBuilder.new_auth_headers()
        try:
            self.conn = await self.session.ws_connect(  # 使用self.session
                self.url,
                headers=headers
            )
            logger.debug(f"Connected to {self.url}")
        except Exception as e:
            logger.error(f"Failed to connect to WebSocket: {e}")
            raise

    async def send_full_client_request(self) -> None:
        request = RequestBuilder.new_full_client_request(self.seq)
        self.seq += 1  # 发送后递增
        try:
            await self.conn.send_bytes(request)
            logger.debug(f"Sent full client request with seq: {self.seq-1}")

            msg = await self.conn.receive()
            if msg.type == aiohttp.WSMsgType.BINARY:
                response = ResponseParser.parse_response(msg.data)
                logger.debug(f"Received response: {response.to_dict()}")
            else:
                logger.error(f"Unexpected message type: {msg.type}")
        except Exception as e:
            logger.error(f"Failed to send full client request: {e}")
            raise

    async def send_messages(self, segment_size: int,
                            content: bytes) -> AsyncGenerator[None, None]:
        audio_segments = self.split_audio(content, segment_size)
        total_segments = len(audio_segments)

        for i, segment in enumerate(audio_segments):
            is_last = (i == total_segments - 1)
            request = RequestBuilder.new_audio_only_request(
                self.seq,
                segment,
                is_last=is_last
            )
            await self.conn.send_bytes(request)
            logger.info(
                f"Sent audio segment with seq: {self.seq} (last: {is_last})")

            if not is_last:
                self.seq += 1

            await asyncio.sleep(self.segment_duration / 1000)  # 逐个发送，间隔时间模拟实时流
            # 让出控制权，允许接受消息
            yield

    async def recv_messages(self) -> AsyncGenerator[AsrResponse, None]:
        try:
            async for msg in self.conn:
                if msg.type == aiohttp.WSMsgType.BINARY:
                    response = ResponseParser.parse_response(msg.data)
                    yield response

                    if response.is_last_package or response.code != 0:
                        break
                elif msg.type == aiohttp.WSMsgType.ERROR:
                    logger.error(f"WebSocket error: {msg.data}")
                    break
                elif msg.type == aiohttp.WSMsgType.CLOSED:
                    logger.info("WebSocket connection closed")
                    break
        except Exception as e:
            logger.error(f"Error receiving messages: {e}")
            raise

    async def start_audio_stream(
            self, segment_size: int, content: bytes) -> AsyncGenerator[AsrResponse, None]:
        async def sender():
            async for _ in self.send_messages(segment_size, content):
                pass

        # 启动发送和接收任务
        sender_task = asyncio.create_task(sender())

        try:
            async for response in self.recv_messages():
                yield response
        finally:
            sender_task.cancel()
            try:
                await sender_task
            except asyncio.CancelledError:
                pass

    @staticmethod
    def split_audio(data: bytes, segment_size: int) -> List[bytes]:
        if segment_size <= 0:
            return []

        segments = []
        for i in range(0, len(data), segment_size):
            end = i + segment_size
            if end > len(data):
                end = len(data)
            segments.append(data[i:end])
        return segments

    async def execute(
            self, file_path: str) -> AsyncGenerator[AsrResponse, None]:
        if not file_path:
            raise ValueError("File path is empty")

        if not self.url:
            raise ValueError("URL is empty")

        self.seq = 1

        try:
            # 1. 读取音频文件
            content = await self.read_audio_data(file_path)

            # 2. 计算分段大小
            segment_size = self.get_segment_size(content)

            # 3. 创建WebSocket连接
            await self.create_connection()

            # 4. 发送完整客户端请求
            await self.send_full_client_request()

            # 5. 启动音频流处理
            async for response in self.start_audio_stream(segment_size, content):
                yield response

        except Exception as e:
            logger.error(f"Error in ASR execution: {e}")
            raise
        finally:
            if self.conn:
                await self.conn.close()

    async def execute_stream(
            self, audio_stream) -> AsyncGenerator[AsrResponse, None]:
        """
        实时音频流ASR识别.

        Args:
            audio_stream: 异步生成器，产生音频数据片段(bytes)

        Yields:
            AsrResponse对象
        """
        if not self.url:
            raise ValueError("URL is empty")

        self.seq = 1

        try:
            # 1. 创建WebSocket连接
            await self.create_connection()

            # 2. 发送完整客户端请求
            await self.send_full_client_request()

            # 3. 启动实时音频流处理
            async for response in self.start_realtime_audio_stream(audio_stream):
                yield response

        except Exception as e:
            logger.error(f"Error in streaming ASR execution: {e}")
            raise
        finally:
            if self.conn:
                await self.conn.close()

    async def start_realtime_audio_stream(
            self, audio_stream) -> AsyncGenerator[AsrResponse, None]:
        """
        处理实时音频流.

        Args:
            audio_stream: 异步生成器，产生音频数据片段

        Yields:
            AsrResponse对象
        """

        async def sender():
            try:
                loop = asyncio.get_event_loop()
                chunk_idx = 0
                last_send_time = loop.time()
                sender_start = last_send_time

                async for audio_chunk in audio_stream:
                    now = loop.time()
                    wait_for_chunk = now - last_send_time

                    if audio_chunk is None:
                        logger.info(f"sender收到结束信号(None), 已发送{chunk_idx}个包, "
                                    f"总耗时={now-sender_start:.1f}s")
                        break

                    chunk_size = len(audio_chunk)
                    logger.debug(
                        f"sender发送包 #{chunk_idx}: seq={self.seq}, size={chunk_size}, "
                        f"等待chunk耗时={wait_for_chunk:.3f}s")

                    if wait_for_chunk > 3.0:
                        logger.error(
                            f"sender等待音频数据耗时过长: {wait_for_chunk:.3f}s, "
                            f"极可能导致服务端超时(阈值8s)!")

                    is_last = False
                    request = RequestBuilder.new_audio_only_request(
                        self.seq,
                        audio_chunk,
                        is_last=is_last
                    )

                    send_start = loop.time()
                    await self.conn.send_bytes(request)
                    send_cost = loop.time() - send_start

                    if send_cost > 0.5:
                        logger.warning(
                            f"send_bytes耗时过长: {send_cost:.3f}s, 网络可能拥堵")

                    self.seq += 1
                    chunk_idx += 1
                    last_send_time = loop.time()

                    await asyncio.sleep(0.1)

                # 发送最后一个空包表示结束
                request = RequestBuilder.new_audio_only_request(
                    self.seq,
                    b'',
                    is_last=True
                )
                await self.conn.send_bytes(request)
                logger.debug(
                    f"sender发送结束包: seq={self.seq}, 总发送={chunk_idx}个音频包")

            except asyncio.CancelledError:
                logger.warning(f"sender被取消, 已发送{chunk_idx}个包")
                raise
            except Exception as e:
                logger.error(
                    f"sender异常: {type(e).__name__}: {e}",
                    exc_info=True)
                raise

        sender_task = asyncio.create_task(sender())

        try:
            async for response in self.recv_messages():
                yield response
        finally:
            sender_task.cancel()
            try:
                await sender_task
            except asyncio.CancelledError:
                pass
