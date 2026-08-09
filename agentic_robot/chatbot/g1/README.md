# Unitree G1 Chat Library

This directory contains the chat, ASR, TTS, and LLM integration utilities used for the Unitree G1 robot.

## Installation

```bash
sudo apt-get install -y portaudio19-dev
cd agentic_robot/chatbot/g1
uv sync -p 3.8
```

## Configuration

```bash
cd agentic_robot/chatbot/g1
mkdir -p ~/.config/g1
cp g1.json ~/.config/g1/
cp g1_system_prompt_en.txt ~/.config/g1/
cp g1_system_prompt_zh.txt ~/.config/g1/
export QWEN_API_KEY=your_key
```

## Source-Level Tests

```bash
# List audio devices
python 01_device_list.py

# Real-time audio loopback test
python 02_test_AudioDevice.py

# Qwen ASR
G1_SETTINGS_PATH=/home/unitree/.config/g1/g1.json python 03_doubao_asr.py

# Doubao TTS
G1_SETTINGS_PATH=/home/unitree/.config/g1/g1.json python 04_doubao_tts.py

# Qwen LLM
G1_SETTINGS_PATH=/home/unitree/.config/g1/g1.json python 05_doubao_llm.py

# Interactive G1 chat
G1_SETTINGS_PATH=/home/unitree/.config/g1/g1.json python g1.py
```

## ROS Integration

`g1chat_node.py` contains the ROS integration entry point and may require project-specific adaptation before deployment.

## Notes

- The runtime configuration is loaded from `G1_SETTINGS_PATH` when provided.
- `QWEN_MODEL` defaults to `qwen3.7-plus`; `QWEN_ASR_MODEL` defaults to `qwen3-asr-flash-realtime`.
- Audio device names and indexes may differ across machines.
