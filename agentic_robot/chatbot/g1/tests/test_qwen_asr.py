import io
import wave

from audio.qwen_asr import AsrResponse, AsrWsClient


def test_pcm_from_wav_chunk_strips_header():
    pcm = b"\x01\x00\x02\x00" * 80
    buffer = io.BytesIO()
    with wave.open(buffer, "wb") as wav_file:
        wav_file.setnchannels(1)
        wav_file.setsampwidth(2)
        wav_file.setframerate(16000)
        wav_file.writeframes(pcm)

    assert AsrWsClient._pcm_from_wav_chunk(buffer.getvalue()) == pcm


def test_response_matches_g1chat_contract():
    result = AsrResponse("去点位一", True).to_dict()["payload_msg"]["result"]

    assert result == {
        "text": "去点位一",
        "utterances": [{"definite": True}],
    }
