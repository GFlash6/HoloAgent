import time
import os
from openai import DefaultHttpxClient, OpenAI

QWEN_API_KEY = os.getenv("QWEN_API_KEY", "")
if not QWEN_API_KEY:
    raise RuntimeError("未配置 QWEN_API_KEY")

client = OpenAI(
    api_key=QWEN_API_KEY,
    base_url=os.getenv(
        "QWEN_BASE_URL", "https://dashscope.aliyuncs.com/compatible-mode/v1"
    ),
    http_client=DefaultHttpxClient(
        proxy=os.getenv("HTTPS_PROXY") or os.getenv("https_proxy"),
        trust_env=False,
    ),
)

start_time = time.time()
response = client.chat.completions.create(
    messages=[
        {"role": "system", "content": "你是个语音助手, 请用中文回答用户的问题"},
        {"role": "user", "content": "讲一个故事"},
    ],
    model=os.getenv("QWEN_MODEL", "qwen3.7-plus"),
    stream=True,  # True 是流逝返回，False是非流逝返回
    extra_body={"enable_thinking": False},
)

first_token_logged = False
print('*' * 100)
for chunk in response:
    delta = (chunk.choices[0].delta.content or "") if chunk.choices else ""
    if not first_token_logged and delta:
        print(f"\n首token耗时: {time.time() - start_time:.4f}秒")
        first_token_logged = True
    print(f"||{delta}", end="", flush=True)
print()
print('*' * 100)
