import time
from openai import OpenAI

from audio.env import SETTINGS

LLM_DOUBAO_MODEL = SETTINGS["llm"]["llm_doubao_model"]
ARK_API_KEY = SETTINGS["llm"]["ark_api_key"]
ARK_BASE_URL = SETTINGS["llm"]["ark_base_url"]

client = OpenAI(
    api_key=ARK_API_KEY,
    base_url=ARK_BASE_URL,
)

start_time = time.time()
response = client.chat.completions.create(
    messages=[
        {"role": "system", "content": "你是个语音助手, 请用中文回答用户的问题"},
        {"role": "user", "content": "讲一个故事"},
    ],
    model=LLM_DOUBAO_MODEL,
    stream=True,  # True 是流逝返回，False是非流逝返回
    extra_body={
        "thinking": {
            "type": "disabled"  # 不使用深度思考能力
            # "type": "enabled" # 使用深度思考能力
            # "type": "auto" # 模型自行判断是否使用深度思考能力
        }
    },
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
