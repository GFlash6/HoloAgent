"""
G1Chat: 语音对话流水线.

实现 语音输入 -> ASR -> LLM -> TTS -> 播放语音 的完整流程
"""

import asyncio
import re
import os
import uuid
import json
import copy
import threading
import traceback
from datetime import datetime
from queue import Queue
from typing import Optional
import traceback
from openai import OpenAI

from audio.env import SETTINGS
from audio.audio_device import AudioDevice
from audio.logging import default_logger as logger
from audio.misc import realtime_audio_generator, is_ws_connection_closed, create_websocket_connection
from audio.volcengine_doubao_asr import AsrWsClient
from audio.volcengine_doubao_tts import (
    EventType,
    MsgType,
    finish_connection,
    finish_session,
    receive_message,
    start_session,
    task_request,
    wait_for_event,
)

AUDIO_DEVICE_NAME = SETTINGS["audio_device"]["device_name"]
AUDIO_DEVICE_CHANNELS = SETTINGS["audio_device"]["channels"]
AUDIO_DEVICE_CHUNK_SIZE = SETTINGS["audio_device"]["chunk_size"]

ASR_APP_KEY = SETTINGS["asr"]["app_id"]
ASR_APP_KEY = os.getenv("CHATBOT_ASR_APP_KEY", ASR_APP_KEY)
ASR_ACCESS_KEY = SETTINGS["asr"]["access_key"]
ASR_ACCESS_KEY = os.getenv("CHATBOT_ASR_ACCESS_KEY", ASR_ACCESS_KEY)
SILENCE_TIMEOUT_MS = SETTINGS["asr"]["end_window_size"]   # 静音超时时间(毫秒)，默认600ms
ASR_RATE = SETTINGS["asr"]["rate"]
ASR_SEG_DURATION = SETTINGS["asr"]["seg_duration"]
ASR_URL = SETTINGS["asr"]["url"]

ARK_API_KEY = SETTINGS["llm"]["ark_api_key"]
ARK_API_KEY = os.getenv("CHATBOT_ARK_API_KEY", ARK_API_KEY)
ARK_BASE_URL = SETTINGS["llm"]["ark_base_url"]
LLM_DOUBAO_MODEL = SETTINGS["llm"]["llm_doubao_model"]

TTS_APP_KEY = SETTINGS["tts"]["app_id"]                # 豆包语音合成KEY
TTS_APP_KEY = os.getenv("CHATBOT_TTS_APP_KEY", TTS_APP_KEY)
TTS_ACCESS_KEY = SETTINGS["tts"]["access_key"]  # 豆包语音合成KEY
TTS_ACCESS_KEY = os.getenv("CHATBOT_TTS_ACCESS_KEY", TTS_ACCESS_KEY)
TTS_RESOURCE_ID = SETTINGS["tts"]["resource_id"]
TTS_ENDPOINT = SETTINGS["tts"]["endpoint"]
TTS_VOICE_TYPE = SETTINGS["tts"]["voice_type"]

_SENTENCE_SPLIT = re.compile(r"([，,。.！？\n]+)")  # 用于流式回复按句切分

LANGUAGE = SETTINGS["language"]
WAKE_UP_TEXT = SETTINGS["wake_up_text"]
SLEEP_TEXT = SETTINGS["sleep_text"]
SYSTEM_PROMPT_ZH = open(
    os.path.expanduser(
        SETTINGS["system_prompt_zh_path"]),
    "r").read()
SYSTEM_PROMPT_EN = open(
    os.path.expanduser(
        SETTINGS["system_prompt_en_path"]),
    "r").read()
HOOKS = SETTINGS["hooks"]
WORK_DIR = os.path.expanduser(SETTINGS["work_dir"])
os.makedirs(WORK_DIR, exist_ok=True)


class G1Chat:
    """
    语音对话类：语音输入 -> ASR -> LLM -> TTS -> 播放.

    - 语音输入：通过麦克风录音，由 ASR 实时识别
    - ASR: 识别结果放入队列，由流水线取出
    - LLM: 调用豆包大模型生成回复
    - TTS: 将回复文本放入 TTS 队列
    - 播放: TTS 处理器将语音播放到扬声器
    """

    def __init__(self):

        # ========== 音频设备配置 ==========
        # 初始化音频设备，用于录音和播放
        self.audio_device = AudioDevice()
        # 启动音频设备的录音和播放流
        self.audio_device.start_streams()

        # ========== ASR客户端配置 ==========
        self.asr_url = ASR_URL  # ASR WebSocket服务地址
        self.asr_seg_duration = ASR_SEG_DURATION  # ASR分段时长（毫秒），每150ms发送一次，更早送首包
        self.asr_queue = Queue()  # ASR识别结果队列，存储识别到的文本
        self.asr_queue_event = None  # 异步事件，用于通知有新识别结果放入队列（在异步上下文中创建）
        self.asr_chat_id = 1  # ASR chat_id 计数器，用于标识不同的识别会话

        # ========== LLM客户端配置 ==========
        # 使用同步 OpenAI 客户端（与 doubao_llm 一致），在独立线程中拉流，首 token 延迟更低
        self._sync_llm_client = OpenAI(
            api_key=ARK_API_KEY, base_url=ARK_BASE_URL)

        # ========== TTS客户端配置 ==========
        self.tts_queue = Queue()  # TTS文本队列，存储待转换为语音的文本
        self.tts_queue_event = None  # 异步事件，用于通知有新文本放入队列（在异步上下文中创建）
        self.tts_chat_id = 1  # TTS chat_id 计数器，用于标识不同的TTS会话
        self.tts_appid = TTS_APP_KEY  # TTS应用ID（从环境变量获取）
        self.tts_access_token = TTS_ACCESS_KEY  # TTS访问令牌（从环境变量获取）
        self.tts_resource_id = TTS_RESOURCE_ID  # TTS资源ID，指定使用的TTS模型
        self.tts_voice_type = TTS_VOICE_TYPE  # TTS语音类型（中文男声）
        self.tts_encoding = "pcm"  # TTS音频编码格式
        self.tts_endpoint = TTS_ENDPOINT  # TTS WebSocket服务地址
        self.tts_sample_rate = self.audio_device.sample_rate  # TTS播放采样率（Hz）
        self.tts_running = False  # TTS处理器运行状态标志
        self.tts_processing = False  # 是否正在处理某条文本（已出队但尚未播放完）
        self.tts_generation = 0  # 用于支持中断播放：每次开始一段新的“对话回复”时自增

        # 多轮对话历史
        self._messages: Optional[list[dict[str, str]]] = None

        self.wakeup = False  # 是否唤醒
        self._running = False  # 是否运行
        # 文本队列, 用于传输信号, 例如停止信号, 挥手信号, 例如, signal:text, location:text,
        # user:text, assistant:text,
        self.text_queue = Queue()
        self.control_queue = Queue()  # 控制队列, 用于传输控制信号, 例如到达某地点后播放一段音频
        self._asr_task: Optional[asyncio.Task] = None  # ASR 任务
        self._tts_task: Optional[asyncio.Task] = None  # TTS 任务
        self._pipeline_task: Optional[asyncio.Task] = None  # 流水线任务
        self._control_task: Optional[asyncio.Task] = None  # 控制任务
        self.system_prompt_zh = SYSTEM_PROMPT_ZH
        self.system_prompt_en = SYSTEM_PROMPT_EN

    async def start_realtime_asr(
            self, silence_timeout_ms: int = SILENCE_TIMEOUT_MS):
        """
        启动实时ASR自动语音识别.

        持续录音并实时识别语音, 将识别结果放入asr_queue队列, 通过asr_queue_event事件通知有新识别结果放入队列.
        使用静音超时机制: 当超过指定时间没有识别到新文字时, 将累积的文本放入队列. 这样可以实现自动分段, 将一句话识别完成后立即放入队列.

        Args:
            silence_timeout_ms: 静音超时时间(毫秒), 默认600ms, 当超过此时间没有识别到新文字时, 将当前累积的识别结果放入队列

        Raises:
            Exception: ASR识别过程中发生错误时抛出异常
        """
        # ========== 初始化异步事件 ==========
        # 在异步上下文中创建事件，用于通知有新识别结果
        if self.asr_queue_event is None:
            self.asr_queue_event = asyncio.Event()

        # ========== 创建实时音频生成器 ==========
        try:
            # 从音频设备持续获取音频数据，转换为WAV格式并分块发送
            audio_stream = realtime_audio_generator(
                self.audio_device, chunk_duration_ms=self.asr_seg_duration)
        except Exception as e:
            logger.error(f"启动实时ASR失败: 创建实时音频生成器失败: {e}")
            logger.error(traceback.format_exc())
            # 大概率是硬件问题, 直接抛出异常
            raise

        # ========== 创建ASR客户端并进行实时识别 ==========
        try:
            async with AsrWsClient(self.asr_url, self.asr_seg_duration) as asr_client:
                last_text_time = None  # 最后一次收到识别文本的时间戳
                accumulated_text = ""  # 累积的识别文本, ASR服务会返回完整的累积文本
                last_is_definite = False  # 最后一次识别结果的确定状态

                # 超时检查任务：定期检查是否超过静音超时时间，如果是则将结果放入队列
                async def check_timeout():
                    """
                    超时检查任务.

                    每50ms检查一次, 如果超过silence_timeout_ms没有收到新文本,
                    且有累积文本且is_definite为True, 则将结果放入队列并重置状态.
                    """
                    nonlocal last_text_time, accumulated_text, last_is_definite
                    while True:
                        await asyncio.sleep(0.05)  # 每50ms检查一次，更快判定句结束
                        current_time = asyncio.get_event_loop().time()

                        # 如果超过静音超时时间没有收到新文本，且有累积文本且is_definite为True，则放入队列
                        if last_text_time is not None and accumulated_text and last_is_definite:
                            elapsed_ms = (current_time - last_text_time) * 1000
                            if elapsed_ms >= silence_timeout_ms:
                                # 将识别结果放入队列，并记录“静默判定完成”的时间戳
                                result = {
                                    "text": accumulated_text,
                                    "chat_id": self.asr_chat_id,
                                }
                                self.asr_queue.put(result)
                                self.asr_chat_id += 1

                                # 通知有新结果（唤醒等待队列的代码）
                                self.asr_queue_event.set()

                                # 重置状态，准备接收下一段识别文本
                                accumulated_text = ""
                                last_text_time = None
                                last_is_definite = False

                # 启动超时检查任务（后台运行）
                timeout_task = asyncio.create_task(check_timeout())

                # ========== 处理ASR响应流 ==========
                # 从ASR客户端接收识别结果（流式处理）
                async for response in asr_client.execute_stream(audio_stream):
                    # 解析响应数据为字典格式
                    resp_dict = response.to_dict()

                    # 检查响应中是否包含识别结果
                    if resp_dict.get(
                            'payload_msg') and 'result' in resp_dict['payload_msg']:
                        result = resp_dict['payload_msg']['result']

                        # 如果包含识别文本（ASR服务返回的文本是累积的完整文本）
                        if 'text' in result:
                            text = result['text']
                            current_time = asyncio.get_event_loop().time()

                            # 获取识别结果的确定状态（是否为最终结果）
                            is_definite = result.get('utterances', [{}])[0].get(
                                'definite', False) if result.get('utterances') else False

                            # 更新累积文本（使用最新的完整文本，ASR会不断更新完整文本）
                            accumulated_text = text
                            last_text_time = current_time  # 更新最后收到文本的时间
                            last_is_definite = is_definite  # 更新确定状态

        except Exception as e:
            logger.error(f"实时ASR处理失败: {e}")
            logger.error(traceback.format_exc())
            raise
        finally:
            # ========== 清理资源 ==========
            # 取消超时检查任务
            timeout_task.cancel()
            try:
                await timeout_task
            except asyncio.CancelledError:
                pass

            # 如果还有未处理的文本（识别结束时可能还有文本未放入队列），且is_definite为True，放入队列
            if accumulated_text and last_is_definite:
                result = {
                    "text": accumulated_text,
                    "chat_id": self.asr_chat_id,
                }
                self.asr_queue.put(result)
                # 通知有新结果
                self.asr_queue_event.set()

    async def _process_tts_text(
            self,
            websocket,
            text: str,
            generation_id: Optional[int] = None):
        """
        处理单个文本的 TTS 转换和播放.

        实现流式处理：一边接收音频数据一边播放，降低延迟。
        使用生产者-消费者模式：发送文本任务、接收音频任务、播放音频任务并行运行。

        Args:
            websocket: WebSocket 连接对象, 用于与TTS服务通信
            text: 要转换为语音的文本内容
            generation_id: 当前TTS会话的generation_id, 用于标识当前TTS会话是否为旧会话, 如果为旧会话, 则不播放音频数据
        """
        # 如果未显式传入，则使用当前 generation
        if generation_id is None:
            generation_id = self.tts_generation

        # ========== 构建基础请求参数 ==========
        # TTS服务的请求参数模板，包含用户信息、语音类型、音频格式等
        base_request = {
            "user": {
                "uid": str(uuid.uuid4()),  # 生成唯一用户ID
            },
            "namespace": "BidirectionalTTS",  # TTS命名空间
            "req_params": {
                "speaker": self.tts_voice_type,  # 语音类型（如：中文男声）
                "audio_params": {
                    "format": self.tts_encoding,  # 音频编码格式（mp3或pcm）
                    "sample_rate": self.tts_sample_rate,
                    "enable_timestamp": True,  # 启用时间戳
                },
                "additions": json.dumps(
                    {
                        "disable_markdown_filter": False,  # 不禁用Markdown过滤
                    }
                ),
            },
        }

        # ========== 启动TTS会话 ==========
        # 每个句子需要创建一个新的TTS会话
        start_session_request = copy.deepcopy(base_request)
        start_session_request["event"] = EventType.StartSession
        session_id = str(uuid.uuid4())  # 生成唯一的会话ID
        await start_session(
            websocket, json.dumps(start_session_request).encode(), session_id
        )
        # 等待会话启动确认
        await wait_for_event(
            websocket, MsgType.FullServerResponse, EventType.SessionStarted
        )

        # ========== 逐字符发送文本（异步函数） ==========
        # 流式输入：逐字符发送文本，TTS服务可以实时开始合成
        async def send_chars():
            """
            发送文本任务（生产者）

            逐字符发送文本到TTS服务, 字符间有短暂延迟, 模拟自然输入。
            """
            for char in text.strip():
                synthesis_request = copy.deepcopy(base_request)
                synthesis_request["event"] = EventType.TaskRequest
                synthesis_request["req_params"]["text"] = char
                await task_request(
                    websocket, json.dumps(
                        synthesis_request).encode(), session_id
                )
                await asyncio.sleep(0.005)  # 字符间延迟 5ms，避免发送过快

            # 发送会话结束请求（通知TTS服务文本发送完成）
            await finish_session(websocket, session_id)

        # ========== 创建异步队列用于存储接收到的音频数据 ==========
        # 生产者-消费者模式：接收任务将音频数据放入队列，播放任务从队列取出并播放
        audio_queue = asyncio.Queue()

        # ========== 接收音频数据的任务（生产者） ==========
        async def receive_audio_task():
            """
            接收音频数据任务（生产者）

            持续从WebSocket接收音频数据消息, 将音频数据放入队列。 当收到会话结束事件时, 发送结束标记(None)到队列。
            """
            try:
                while True:
                    # 添加超时处理，避免长时间阻塞导致连接超时
                    msg = await asyncio.wait_for(receive_message(websocket), timeout=40.0)

                    if msg.type == MsgType.FullServerResponse:
                        if msg.event == EventType.SessionFinished:
                            # 会话结束，发送结束标记到队列
                            await audio_queue.put(None)
                            break
                    elif msg.type == MsgType.AudioOnlyServer:
                        # 处理音频数据消息（TTS服务返回的音频数据）
                        if msg.payload:
                            # 将音频数据放入队列（供播放任务消费）
                            await audio_queue.put(msg.payload)
                    else:
                        # 未知消息类型，记录警告但继续处理
                        logger.warning(f"TTS: 收到未知消息类型: {msg.type}")
            except Exception as e:
                logger.error(f"TTS: 接收音频数据任务失败: {e}")
                logger.error(traceback.format_exc())
                raise

        # ========== 播放音频数据的任务（消费者） ==========
        async def playback_audio_task():
            """
            播放音频数据任务（消费者）

            持续从队列取出音频数据, 并播放。
            """
            try:
                while True:
                    # 从队列中获取音频数据，设置超时避免无限等待
                    try:
                        audio_data = await asyncio.wait_for(audio_queue.get(), timeout=1.0)
                    except asyncio.TimeoutError:
                        # 超时但会话可能还在进行，继续等待
                        continue

                    # 如果收到会话结束标记，则退出循环
                    if audio_data is None:
                        break

                    # 如果当前会话已经属于“旧 generation”，则不播放音频数据
                    if generation_id != self.tts_generation:
                        continue

                    self.audio_device.put_playback_data(audio_data)

            except Exception as e:
                logger.error(f"TTS: 播放音频数据任务失败: {e}")

        # ========== 启动接收和播放任务（并行运行） ==========
        # 三个任务并行运行：发送文本、接收音频、播放音频
        send_task = asyncio.create_task(send_chars())
        receive_task = asyncio.create_task(receive_audio_task())
        playback_task = asyncio.create_task(playback_audio_task())

        # ========== 等待所有任务完成 ==========
        try:
            await asyncio.gather(send_task, receive_task, playback_task)
        except Exception as e:
            logger.error(f"TTS: 处理任务失败: {e}")
            logger.error(traceback.format_exc())
            # 取消未完成的任务（清理资源）
            if not send_task.done():
                send_task.cancel()
            if not receive_task.done():
                receive_task.cancel()
            if not playback_task.done():
                playback_task.cancel()
            # 等待任务取消完成（确保资源清理完成）
            await asyncio.gather(send_task, receive_task, playback_task, return_exceptions=True)
            raise

    async def start_tts_processor(self):
        """
        启动 TTS文本转语音处理器.

        该方法会持续运行，从 tts_queue 中读取文本，调用 TTS 服务转换为语音并实时播放。
        使用 WebSocket 连接进行双向通信，支持流式处理（一边接收音频数据一边播放）。

        工作流程：
        1. 建立 WebSocket 连接到 TTS 服务
        2. 持续监听队列中的文本
        3. 对每个文本调用 TTS 服务转换为语音
        4. 实时接收音频数据并播放（流式处理，降低延迟）

        Raises:
            Exception: TTS处理过程中发生错误时抛出异常
        """
        # ========== 初始化异步事件 ==========
        # 在异步上下文中创建事件，用于通知有新文本放入队列
        if self.tts_queue_event is None:
            self.tts_queue_event = asyncio.Event()

        self.tts_running = True  # 设置运行标志

        websocket = None

        try:
            # ========== 主循环：支持自动重连 ==========
            while self.tts_running:
                try:
                    # ========== 建立或重新建立 WebSocket 连接 ==========
                    if websocket is None or is_ws_connection_closed(websocket):
                        websocket = await create_websocket_connection(self.tts_appid, self.tts_access_token, self.tts_resource_id, self.tts_endpoint)
                except Exception as e:
                    logger.error(f"TTS: 建立 WebSocket 连接失败: {e}")
                    logger.error(traceback.format_exc())
                    raise

                # 先检查队列，如果有数据立即处理（减少延迟）, 批量处理队列中的所有文本，确保及时响应
                processed_any = False
                while not self.tts_queue.empty():
                    text_data = self.tts_queue.get_nowait()
                    if text_data:
                        # 提取文本内容（支持字典格式或字符串格式）
                        text = text_data.get(
                            "text", "") if isinstance(
                            text_data, dict) else str(text_data)
                        if text:
                            self.tts_processing = True
                            await self._process_tts_text(websocket, text, generation_id=self.tts_generation)
                            self.tts_processing = False
                            processed_any = True

                # 如果已经处理了文本，立即继续循环检查下一个（保持高响应性）
                if processed_any:
                    continue

                # ========== 队列为空时，等待事件通知 ==========
                # 有新文本加入时会立即被唤醒，避免空转消耗CPU
                try:
                    await asyncio.wait_for(self.tts_queue_event.wait(), timeout=1.0)
                    self.tts_queue_event.clear()
                except asyncio.TimeoutError:
                    # 超时后继续循环检查（保持响应性，同时避免空转）
                    continue

        except Exception as e:
            logger.error(f"TTS处理器失败: {e}")
            logger.error(traceback.format_exc())
            # 内部错误, 向上抛出异常
            raise
        finally:
            # ========== 清理资源 ==========
            await finish_connection(websocket)
            await websocket.close()
            self.tts_running = False  # 重置运行标志
            self.tts_processing = False  # 重置处理标志
            logger.info("TTS: 处理器已停止")

    def _put_tts_text(self, text: str):
        """
        将文本放入 TTS 队列.

        将待转换为语音的文本放入队列, TTS处理器会从队列中取出并处理。
        这是一个线程安全的方法, 可以在任何线程中调用。

        Args:
            text: 要转换为语音的文本内容
        """
        # 将文本、chat_id 和 asr_end_ts 一起放入队列（字典格式）
        self.tts_queue.put(
            {
                "text": text,
                "chat_id": self.tts_chat_id,
            }
        )
        self.tts_chat_id += 1  # 递增chat_id计数器

        # 如果有事件对象，触发事件通知TTS处理器（唤醒等待的处理器）
        if self.tts_queue_event:
            self.tts_queue_event.set()

    def _interrupt_tts(self):
        """
        中断当前 TTS 播放:

        - 清空 TTS 文本队列
        - 清空底层音频播放队列和缓冲
        - 自增 generation, 让正在进行的 TTS 流尽快感知并退出
        """
        # 自增 generation，标记之前的所有 TTS 流为“过期”
        self.tts_generation += 1

        # 清空待播报的文本队列
        while not self.tts_queue.empty():
            try:
                self.tts_queue.get_nowait()
            except Exception:
                break

        # 清空底层播放队列和缓冲
        if self.audio_device:
            try:
                self.audio_device.clear_playback_queue()
                self.audio_device.clear_playback_buffer()
            except Exception:
                pass

        # 触发事件, 让处理协程尽快醒来检查 generation
        if self.tts_queue_event:
            self.tts_queue_event.set()

    async def call_llm(self, user_text: str) -> str:
        """
        调用 LLM 获取回复（异步流式，不阻塞事件循环）

        Args:
            user_text: 用户输入文本(ASR 结果)

        Returns:
            助手回复的完整文本
        """
        self._messages.append({"role": "user", "content": user_text})

        # LLM流式回复完整内容, 用于拼接LLM流式回复内容
        full_content: list[str] = []
        first_chunk_sent = False    # LLM流式回复首片段是否已发送到TTS队列中, 用于判断是否需要发送首片段, 如果已发送则不发送首片段
        first_buf = ""              # LLM流式回复首片段缓冲区, 用于累积首片段内容, 用于拼接首片段内容
        # LLM流式回复缓冲区(去除first_buf后的内容), 用于累积LLM流式回复内容, 用于拼接LLM流式回复内容
        buffer = ""
        MIN_FIRST_CHARS = 8         # 首片段最少字符数, 首片段最少字符数，尽量小以换取更快响应
        started_tts_session = False  # 标记本次 LLM 回复是否已经为 TTS 启动过一次“中断+新会话”

        # 同步客户端在独立线程拉流，用 asyncio.Queue + call_soon_threadsafe 递交给事件循环，
        # 避免 run_in_executor 与 ASR/control 争用默认线程池导致首 chunk 被延迟处理
        loop = asyncio.get_event_loop()
        chunk_async_queue: asyncio.Queue = asyncio.Queue()

        def sync_stream():
            stream = self._sync_llm_client.chat.completions.create(
                messages=self._messages,
                model=LLM_DOUBAO_MODEL,
                stream=True,
                extra_body={"thinking": {"type": "disabled"}},
            )
            for ch in stream:
                loop.call_soon_threadsafe(chunk_async_queue.put_nowait, ch)
            loop.call_soon_threadsafe(chunk_async_queue.put_nowait, None)

        t = threading.Thread(target=sync_stream, daemon=True)
        t.start()

        is_first_response = True    # 是否是LLM流式回复的第一包
        llm_response_type = None    # LLM 回复类型, 用于判断是位置信息还是音频信息
        while True:
            chunk = await chunk_async_queue.get()
            if chunk is None:
                break
            delta = (
                chunk.choices[0].delta.content or "") if chunk.choices else ""
            if not delta:
                continue

            # 判断LLM回复类型
            if is_first_response:
                is_first_response = False
                if "{" in delta:
                    llm_response_type = "location"
                else:
                    llm_response_type = "audio"

            # 首包快速首发：在还没有发送过任何 TTS 文本之前，
            # 只要累计到一定长度的内容就立即送入 TTS 队列，而不等待标点。
            if not first_chunk_sent:
                first_buf += delta
                if len(first_buf.strip()) >= MIN_FIRST_CHARS:
                    sentence = first_buf.strip()
                    if sentence:
                        full_content.append(sentence)
                        # 如果LLM回复类型是音频, 则直接发送给TTS队列
                        if llm_response_type == "audio":
                            # 新的一轮 LLM 回复到来时, 先中断之前未播完的语音,
                            # 再把当前回复送入 TTS 队列（只在本轮第一次触发）
                            if not started_tts_session:
                                self._interrupt_tts()
                                started_tts_session = True
                            self._put_tts_text(sentence)
                        first_chunk_sent = True
                        first_buf = ""
                        continue
                # 在首片段发送前，不进入按句切分逻辑，继续累积
                if not first_chunk_sent:
                    continue

            buffer += delta
            # 按句号、问号、感叹号、换行切分，有完整句就送 TTS
            parts = _SENTENCE_SPLIT.split(buffer)
            buffer = ""
            for i, part in enumerate(parts):
                if _SENTENCE_SPLIT.match(
                        part):  # 如果part是标点符号, 则把前一个句子与part拼接后发送给TTS队列
                    if i > 0 and parts[i - 1]:
                        sentence = (parts[i - 1] + part).strip()
                        if sentence:
                            full_content.append(sentence)
                            if llm_response_type == "audio":
                                if not started_tts_session:
                                    self._interrupt_tts()
                                    started_tts_session = True
                                self._put_tts_text(sentence)
                    buffer = ""
                    continue
                buffer = part   # 如果part不是完整句子, 则继续累积

        # 处理流结束时剩余的内容：
        # - 如果首片段尚未发送，则把首片段和 buffer 合并后一并发送；
        # - 否则只处理 buffer 中剩余的尾巴。
        if not first_chunk_sent:
            tail_source = first_buf + buffer
        else:
            tail_source = buffer

        tail = tail_source.strip()
        if tail:
            full_content.append(tail)
            if llm_response_type == "audio":
                if not started_tts_session:
                    self._interrupt_tts()
                    started_tts_session = True
                self._put_tts_text(tail)

        content = "".join(full_content) if full_content else ""
        if content:
            self._messages.append({"role": "assistant", "content": content})
        return content, llm_response_type

    async def pipeline_loop(self):
        """流水线协程：从 ASR 队列取文本 -> 调用 LLM -> 将回复放入 TTS 队列。"""
        if self.asr_queue_event is None:
            self.asr_queue_event = asyncio.Event()

        while self._running:
            try:
                await self.asr_queue_event.wait()
                self.asr_queue_event.clear()

                while not self.asr_queue.empty():
                    # ============== ASR ==============
                    result = self.asr_queue.get_nowait()
                    if not result or not isinstance(result, dict):
                        continue
                    text = result.get("text", "").strip()
                    if not text:
                        continue
                    logger.info(f"User: {result}")
                    self.text_queue.put(f"user:{text}")

                    # ============== WAKE_UP & SLEEP Hook ==============
                    if "wake_sleep_hooks" not in HOOKS or len(
                            HOOKS["wake_sleep_hooks"]) == 0:
                        if LANGUAGE == "zh":
                            response_text = "请正确配置唤醒和睡眠的钩子"
                        else:
                            response_text = "Please configure the wake_sleep_hooks correctly"
                        self._put_tts_text(response_text)
                        continue

                    if not self.wakeup and WAKE_UP_TEXT in text:
                        self.wakeup = True

                        if LANGUAGE == "zh":
                            response_text = HOOKS["wake_sleep_hooks"][0]["response_zh"]
                        elif LANGUAGE == "en":
                            response_text = HOOKS["wake_sleep_hooks"][0]["response_en"]
                        self._interrupt_tts()
                        self._put_tts_text(response_text)

                        self.text_queue.put(f"assistant:{response_text}")

                        self.text_queue.put(
                            HOOKS["wake_sleep_hooks"][0]["signal"])

                        # 更新对话历史
                        if LANGUAGE == "zh":
                            system_prompt = SYSTEM_PROMPT_ZH
                        else:
                            system_prompt = SYSTEM_PROMPT_EN
                        self._messages = [
                            {"role": "system", "content": system_prompt, }]
                        self._messages.append(
                            {"role": "user", "content": text})
                        self._messages.append(
                            {"role": "assistant", "content": response_text})

                        continue

                    if self.wakeup and SLEEP_TEXT in text:
                        self.wakeup = False

                        if LANGUAGE == "zh":
                            response_text = HOOKS["wake_sleep_hooks"][1]["response_zh"]
                        elif LANGUAGE == "en":
                            response_text = HOOKS["wake_sleep_hooks"][1]["response_en"]
                        self._interrupt_tts()
                        self._put_tts_text(response_text)

                        self.text_queue.put(f"assistant:{response_text}")

                        self.text_queue.put(
                            HOOKS["wake_sleep_hooks"][1]["signal"])

                        # 保存历史对话
                        save_history_dialog_path = os.path.join(
                            WORK_DIR, "dialogs", f"{datetime.now().strftime('%Y%m%d%H%M%S')}.json")
                        os.makedirs(
                            os.path.dirname(save_history_dialog_path),
                            exist_ok=True)
                        with open(save_history_dialog_path, "w") as f:
                            json.dump(self._messages, f)

                        # 清空对话历史
                        self._messages = []

                        continue

                    if not self.wakeup:
                        continue

                    # ============== ASR Hook ==============
                    if "asr_hooks" not in HOOKS:
                        if LANGUAGE == "zh":
                            response_text = "请正确配置语音识别的钩子"
                        else:
                            response_text = "Please configure the asr_hooks correctly"
                        self._put_tts_text(response_text)
                        continue

                    is_asr_hook_found = False
                    for hook in HOOKS["asr_hooks"]:
                        if hook['relate'] == "and":
                            if all(name.lower() in text.strip().lower()
                                   for name in hook['name']):
                                is_asr_hook_found = True
                            else:
                                is_asr_hook_found = False
                        elif hook['relate'] == "or":
                            if any(name.lower() in text.strip().lower()
                                   for name in hook['name']):
                                is_asr_hook_found = True
                            else:
                                is_asr_hook_found = False
                        if is_asr_hook_found:
                            if LANGUAGE == "zh":
                                response_text = hook['response_zh']
                            else:
                                response_text = hook['response_en']
                            self._interrupt_tts()
                            self._put_tts_text(response_text)

                            self.text_queue.put(f"assistant:{response_text}")

                            self.text_queue.put(hook["signal"])
                            is_asr_hook_found = True
                            break
                    if is_asr_hook_found:
                        continue

                    # ============== LLM ==============
                    try:
                        reply, llm_response_type = await self.call_llm(text)

                        self.text_queue.put(f"assistant:{reply.strip()}")
                        # logger.info(f"Assistant: {reply.strip()}")

                        # 如果返回的是location信息, 需要提取出来, 发送给queue_text
                        if llm_response_type == "location":
                            try:
                                if reply.startswith("{{"):
                                    location_info = json.loads(reply[1:-1])
                                elif reply.startswith("{"):
                                    location_info = json.loads(reply)
                                else:
                                    location_info = ""
                            except Exception as e:
                                if LANGUAGE == "zh":
                                    response_text = "抱歉, 我提取位置信息失败了, 请再说一次"
                                else:
                                    response_text = "Sorry, I'm having trouble extracting the location information. Please try again."
                                self._put_tts_text(response_text)
                                logger.error(
                                    f"提取位置信息失败: {traceback.format_exc()}")
                                location_info = ""
                            self.text_queue.put(
                                f"location:{json.dumps(location_info)}")

                            # ============== location Hook ==============
                            if "location_hooks" not in HOOKS:
                                if LANGUAGE == "zh":
                                    response_text = "请正确配置位置信息的钩子"
                                else:
                                    response_text = "Please configure the location_hooks correctly"
                                self._put_tts_text(response_text)
                                continue

                            is_location_hook_found = False
                            for hook in HOOKS["location_hooks"]:
                                if hook['relate'] == "and":
                                    if all(name.lower() in reply.strip().lower()
                                           for name in hook['name']):
                                        is_location_hook_found = True
                                    else:
                                        is_location_hook_found = False
                                elif hook['relate'] == "or":
                                    if any(name.lower() in reply.strip().lower()
                                           for name in hook['name']):
                                        is_location_hook_found = True
                                    else:
                                        is_location_hook_found = False
                                if is_location_hook_found:
                                    if LANGUAGE == "zh":
                                        response_text = hook['response_zh']
                                    else:
                                        response_text = hook['response_en']
                                    self._interrupt_tts()
                                    self._put_tts_text(response_text)

                                    self.text_queue.put(hook["signal"])
                                    is_location_hook_found = True
                                    break

                    except Exception as e:
                        logger.error(f"LLM 调用失败: {traceback.format_exc()}")
                        self._interrupt_tts()
                        self._put_tts_text("抱歉，我这边出错了，请再说一次。")

            except asyncio.CancelledError:
                break
            except Exception as e:
                logger.error(f"流水线异常: {traceback.format_exc()}")
                await asyncio.sleep(0.1)

    async def stop(self):
        """停止 ASR、流水线和 TTS 处理器，并清理音频设备。"""
        self._running = False
        for task in (
                self._pipeline_task,
                self._asr_task,
                self._tts_task,
                self._control_task):
            if task and not task.done():
                task.cancel()
                try:
                    await task
                except asyncio.CancelledError:
                    pass

        self._asr_task = None
        self._tts_task = None
        self._pipeline_task = None
        self._control_task = None
        if self.audio_device:
            self.audio_device.cleanup()
        logger.info("已停止，资源已清理")

    async def control_loop(self):
        """控制协程：从控制队列取信号 -> 执行控制。"""
        if "control_hooks" not in HOOKS or len(HOOKS["control_hooks"]) == 0:
            if LANGUAGE == "zh":
                response_text = "请正确配置控制信号的钩子"
            else:
                response_text = "Please configure the control_hooks correctly"
            self._put_tts_text(response_text)

        control_signals = HOOKS["control_signals"]

        while self._running:
            try:
                # Queue 是同步队列, 这里通过线程池避免阻塞事件循环
                loop = asyncio.get_event_loop()
                control_signal = await loop.run_in_executor(None, self.control_queue.get)
                if not control_signal:
                    continue

                if control_signal in control_signals:
                    if LANGUAGE == "zh":
                        response_text = HOOKS["control_hooks"][control_signal]["response_zh"]
                    else:
                        response_text = HOOKS["control_hooks"][control_signal]["response_en"]
                    self._interrupt_tts()
                    self._put_tts_text(response_text)
                    # add ending pose
                    # self.text_queue.put(HOOKS["control_hooks"][control_signal]["signal"])

                await asyncio.sleep(0.1)  # 等待0.1秒, 避免频繁读取控制队列

            except asyncio.CancelledError:
                break

    async def start(self):
        """启动语音对话：同时运行 ASR、TTS、AudioDevice、LLM等."""
        self._running = True
        self._asr_task = asyncio.create_task(
            self.start_realtime_asr())     # ASR 任务
        self._tts_task = asyncio.create_task(
            self.start_tts_processor())    # TTS 任务
        self._pipeline_task = asyncio.create_task(
            self.pipeline_loop())     # 流水线任务
        self._control_task = asyncio.create_task(
            self.control_loop())       # 控制任务, 接受控制信号
        logger.info("已启动：语音输入 -> ASR -> LLM -> TTS -> 播放")


async def main():
    """示例：运行 G1Chat 直到 Ctrl+C。"""
    chat = G1Chat()
    await chat.start()
    try:
        # 永久等待，Ctrl+C 会取消当前任务并触发 finally
        await asyncio.Future()
    except asyncio.CancelledError:
        logger.info("收到中断，正在停止...")
    finally:
        await chat.stop()


if __name__ == "__main__":
    asyncio.run(main())
