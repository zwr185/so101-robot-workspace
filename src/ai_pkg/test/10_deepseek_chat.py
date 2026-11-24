"""与大模型多轮对话：使用 DeepSeek API，按 Ctrl+C 退出程序。"""

import os
import sys

from openai import OpenAI

MODEL = "deepseek-chat"  # 通用对话模型；深度思考可改为 "deepseek-reasoner"
SYSTEM_PROMPT = "你是一个乐于助人的中文助手。"


def main():
    api_key = os.environ.get("DEEPSEEK_API_KEY")
    if not api_key:
        print("未检测到环境变量 DEEPSEEK_API_KEY")
        print('请先执行：export DEEPSEEK_API_KEY="sk-你的密钥"')
        sys.exit(1)

    client = OpenAI(api_key=api_key, base_url="https://api.deepseek.com")

    # 保存多轮对话上下文
    messages = [{"role": "system", "content": SYSTEM_PROMPT}]

    print("已进入对话模式，输入内容回车发送，按 Ctrl+C 退出（也可输入 exit / q 退出）。")

    try:
        while True:
            try:
                user_input = input("\n你：").strip()
            except EOFError:
                break

            if not user_input:
                continue
            if user_input.lower() in ("exit", "quit", "q"):
                break

            messages.append({"role": "user", "content": user_input})

            try:
                response = client.chat.completions.create(
                    model=MODEL,
                    messages=messages,
                    stream=False,
                )
            except Exception as exc:
                # 请求失败时回滚本条消息，保持上下文干净
                messages.pop()
                print(f"请求失败：{exc}")
                continue

            reply = response.choices[0].message.content or ""
            messages.append({"role": "assistant", "content": reply})
            print(f"助手：{reply}")
    except KeyboardInterrupt:
        print("\n已退出对话。")

    print("再见。")


if __name__ == "__main__":
    main()
