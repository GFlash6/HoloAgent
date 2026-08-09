"""Qwen realtime ASR client compatible with the G1Chat ASR loop."""

import asyncio
import base64
import io
import json
import os
import uuid
import wave
from typing import Any, AsyncGenerator, Dict

import aiohttp


class AsrResponse:
    """Keep the response shape consumed by ``G1Chat.start_realtime_asr``."""

    def __init__(self, text: str, definite: bool):
        self.text = text
        self.definite = definite

    def to_dict(self) -> Dict[str, Any]:
        return {
            "payload_msg": {
                "result": {
                    "text": self.text,
                    "utterances": [{"definite": self.definite}],
                }
            }
        }


class AsrWsClient:
    """Stream 16 kHz PCM audio to Qwen3-ASR-Flash-Realtime."""

    def __init__(self, url: str, segment_duration: int = 200):
        self.url = url
        self.segment_duration = segment_duration
        self.api_key = os.getenv("QWEN_API_KEY", "")
        self.session = None
        self.conn = None

    async def __aenter__(self):
        if not self.api_key:
            raise RuntimeError("未配置 QWEN_API_KEY")
        self.session = aiohttp.ClientSession(trust_env=True)
        return self

    async def __aexit__(self, exc_type, exc, tb):
        if self.conn and not self.conn.closed:
            await self.conn.close()
        if self.session and not self.session.closed:
            await self.session.close()

    @staticmethod
    def _pcm_from_wav_chunk(audio_chunk: bytes) -> bytes:
        """The recorder emits independent WAV chunks; Qwen expects raw PCM."""
        if not audio_chunk.startswith(b"RIFF"):
            return audio_chunk
        with wave.open(io.BytesIO(audio_chunk), "rb") as wav_file:
            if wav_file.getsampwidth() != 2 or wav_file.getnchannels() != 1:
                raise ValueError("Qwen ASR requires mono 16-bit PCM audio")
            if wav_file.getframerate() != 16000:
                raise ValueError("Qwen ASR requires 16000 Hz audio")
            return wav_file.readframes(wav_file.getnframes())

    async def _send_audio(self, audio_stream) -> None:
        async for audio_chunk in audio_stream:
            if audio_chunk is None:
                await self.conn.send_json(
                    {"type": "session.finish", "event_id": str(uuid.uuid4())}
                )
                return
            pcm = self._pcm_from_wav_chunk(audio_chunk)
            await self.conn.send_json(
                {
                    "type": "input_audio_buffer.append",
                    "event_id": str(uuid.uuid4()),
                    "audio": base64.b64encode(pcm).decode("ascii"),
                }
            )

    async def execute_stream(self, audio_stream) -> AsyncGenerator[AsrResponse, None]:
        headers = {"Authorization": f"Bearer {self.api_key}"}
        self.conn = await self.session.ws_connect(self.url, headers=headers)
        await self.conn.send_json(
            {
                "type": "session.update",
                "event_id": str(uuid.uuid4()),
                "session": {
                    "input_audio_format": "pcm",
                    "sample_rate": 16000,
                    "input_audio_transcription": {"language": "zh"},
                    "turn_detection": {
                        "type": "server_vad",
                        "threshold": 0.0,
                        "silence_duration_ms": 400,
                    },
                },
            }
        )
        sender = asyncio.create_task(self._send_audio(audio_stream))
        try:
            async for message in self.conn:
                if message.type == aiohttp.WSMsgType.ERROR:
                    raise self.conn.exception() or RuntimeError(
                        "Qwen ASR WebSocket error"
                    )
                if message.type != aiohttp.WSMsgType.TEXT:
                    continue
                event = json.loads(message.data)
                event_type = event.get("type")
                if event_type == "conversation.item.input_audio_transcription.text":
                    text = event.get("text", "") + event.get("stash", "")
                    if text:
                        yield AsrResponse(text, False)
                elif event_type == "conversation.item.input_audio_transcription.completed":
                    text = event.get("transcript", "")
                    if text:
                        yield AsrResponse(text, True)
                elif event_type in {
                    "error",
                    "conversation.item.input_audio_transcription.failed",
                }:
                    error = event.get("error", {})
                    raise RuntimeError(error.get("message", str(event)))
                elif event_type == "session.finished":
                    break
        finally:
            sender.cancel()
            try:
                await sender
            except asyncio.CancelledError:
                pass
            if self.conn and not self.conn.closed:
                await self.conn.close()
