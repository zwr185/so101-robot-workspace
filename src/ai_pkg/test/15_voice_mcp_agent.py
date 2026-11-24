"""语音指令 Agent：录音 → Whisper 识别 → DeepSeek 解析 → 通过 MCP 调用本地工具。

流程：
  1. 按 r 录音、按 s 停止（参考 01_audio_record.py）
  2. Whisper 将录音转为文本（参考 03_whisper_asr.py）
  3. 文本提交给 DeepSeek，并把本地 MCP Server 的工具注册为 function calling（参考 10_deepseek_chat.py）
  4. DeepSeek 请求调用工具时，由本脚本通过 MCP Client 回调 12_mcp_server.py 执行
  5. 工具结果回传给 DeepSeek，得到最终回答
"""

import asyncio
import json
import os
import select
import sys
import termios
import threading
import tty
import wave
from pathlib import Path

import pyaudio
import whisper
from fastmcp import Client
from openai import OpenAI

# 音频参数（与 01_audio_record.py 一致）
RATE = 16000              # 采样率 16kHz
CHANNELS = 1              # 单声道
FORMAT = pyaudio.paInt16  # 16 位量化
CHUNK = 1024              # 每次读取的帧数

BASE_DIR = Path(__file__).resolve().parent
WAV_FILE = BASE_DIR / "output.wav"
MCP_SERVER = BASE_DIR / "12_mcp_server.py"

WHISPER_MODEL = "medium"
SIMPLIFIED_PROMPT = "以下是普通话的句子，请使用简体中文输出。"

LLM_MODEL = "deepseek-chat"
SYSTEM_PROMPT = (
    "你是一个语音助手的决策模块。用户会通过语音提出请求，"
    "你必须调用提供的工具来完成任务，不要自行心算或编造结果。"
    "可用工具：add（加法）、subtract（减法）、multiply（乘法）、"
    "divide（除法）、robot_nod（控制机械臂点头）。"
    "例如用户说“帮我算一下三加五”就调用 add，"
    "说“让机械臂点个头”就调用 robot_nod。"
    "拿到工具结果后，用简洁的中文告诉用户最终答案。"
)

BANNER = """
=============== 语音指令 Agent ===============
  流程：录音 → Whisper 识别 → DeepSeek 解析 → MCP 调用本地工具
  r : 开始录音
  q : 退出程序
=============================================
"""

MAX_TOOL_ROUNDS = 5  # 单轮对话最多允许的工具调用轮数，防止死循环


class Recorder:
    """负责打开音频流、后台采集并在停止时写入 wav 文件。"""

    def __init__(self, audio):
        self.audio = audio
        self.stream = None
        self.thread = None
        self.frames = []
        self.recording = False

    @property
    def is_recording(self):
        return self.recording

    def start(self):
        if self.recording:
            return
        self.frames = []
        self.stream = self.audio.open(
            format=FORMAT,
            channels=CHANNELS,
            rate=RATE,
            input=True,
            frames_per_buffer=CHUNK,
        )
        self.recording = True
        self.thread = threading.Thread(target=self._capture, daemon=True)
        self.thread.start()

    def _capture(self):
        while self.recording:
            self.frames.append(self.stream.read(CHUNK, exception_on_overflow=False))

    def stop(self):
        if not self.recording:
            return
        self.recording = False
        self.thread.join()
        self.stream.stop_stream()
        self.stream.close()
        self.stream = None

        with wave.open(str(WAV_FILE), "wb") as wf:
            wf.setnchannels(CHANNELS)
            wf.setsampwidth(self.audio.get_sample_size(FORMAT))
            wf.setframerate(RATE)
            wf.writeframes(b"".join(self.frames))


def read_key(timeout=0.1):
    """非阻塞读取单个按键，无输入时返回 None。"""
    if select.select([sys.stdin], [], [], timeout)[0]:
        return sys.stdin.read(1).lower()
    return None


def to_openai_tools(mcp_tools):
    """把 MCP 工具描述转换为 OpenAI function calling 格式。"""
    tools = []
    for tool in mcp_tools:
        schema = tool.input_schema or {}
        tools.append(
            {
                "type": "function",
                "function": {
                    "name": tool.name,
                    "description": tool.description or "",
                    "parameters": {
                        "type": "object",
                        "properties": schema.get("properties", {}),
                        "required": schema.get("required", []),
                    },
                },
            }
        )
    return tools


async def ask_agent(user_text, messages, api_key):
    """把文本提交给 DeepSeek，若模型要求调用工具则通过 MCP 执行并回传结果。"""
    async with Client(MCP_SERVER) as mcp_client:
        mcp_tools = await mcp_client.list_tools()
        tools = to_openai_tools(mcp_tools)
        print(f"已连接本地 MCP Server，可用工具：{[tool.name for tool in mcp_tools]}")

        llm = OpenAI(api_key=api_key, base_url="https://api.deepseek.com")
        messages.append({"role": "user", "content": user_text})

        for _ in range(MAX_TOOL_ROUNDS):
            response = llm.chat.completions.create(
                model=LLM_MODEL,
                messages=messages,
                tools=tools,
                tool_choice="auto",
            )
            message = response.choices[0].message

            # 模型没有请求工具，直接返回回答
            if not message.tool_calls:
                messages.append({"role": "assistant", "content": message.content or ""})
                return message.content or ""

            # 记录助手的工具调用请求
            messages.append(
                {
                    "role": "assistant",
                    "content": message.content,
                    "tool_calls": [
                        {
                            "id": call.id,
                            "type": "function",
                            "function": {
                                "name": call.function.name,
                                "arguments": call.function.arguments,
                            },
                        }
                        for call in message.tool_calls
                    ],
                }
            )

            # 逐个执行工具，并把结果回传给模型
            for call in message.tool_calls:
                name = call.function.name
                try:
                    arguments = json.loads(call.function.arguments or "{}")
                except json.JSONDecodeError:
                    arguments = {}

                print(f"  -> 大模型请求调用 MCP 工具：{name}({arguments})")
                try:
                    result = await mcp_client.call_tool(name, arguments)
                    content = json.dumps(result.data, ensure_ascii=False)
                except Exception as exc:
                    content = json.dumps({"error": str(exc)}, ensure_ascii=False)
                print(f"  <- MCP 工具返回：{content}")

                messages.append(
                    {
                        "role": "tool",
                        "tool_call_id": call.id,
                        "content": content,
                    }
                )

        return "工具调用轮数超过上限，已停止本轮处理。"


def main():
    api_key = os.environ.get("DEEPSEEK_API_KEY")
    if not api_key:
        print("未检测到环境变量 DEEPSEEK_API_KEY")
        print('请先执行：export DEEPSEEK_API_KEY="sk-你的密钥"')
        sys.exit(1)

    # 优先使用系统完整版 ffmpeg，避免 PATH 中的精简版无法解码 wav
    os.environ["PATH"] = "/usr/bin" + os.pathsep + os.environ.get("PATH", "")

    print(f"正在加载 Whisper 模型：{WHISPER_MODEL} ...")
    model = whisper.load_model(WHISPER_MODEL)
    print("Whisper 模型加载完成。")

    audio = pyaudio.PyAudio()
    recorder = Recorder(audio)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    messages = [{"role": "system", "content": SYSTEM_PROMPT}]

    print(BANNER)
    print("下一步：按 r 开始录音，按 q 退出。")

    try:
        tty.setcbreak(fd)  # 单键输入，无需回车
        while True:
            key = read_key()
            if key is None:
                continue

            if key == "q":
                break

            if key != "r":
                continue

            # ---------- 1. 录音 ----------
            recorder.start()
            print("正在录音... 按 s 停止录音。")
            while recorder.is_recording:
                key = read_key()
                if key == "s":
                    recorder.stop()
                elif key == "q":
                    recorder.stop()
                    raise KeyboardInterrupt

            print("录音完成，正在使用 Whisper 识别...")

            # ---------- 2. 语音转文本 ----------
            result = model.transcribe(
                str(WAV_FILE),
                language="zh",
                initial_prompt=SIMPLIFIED_PROMPT,
            )
            text = result["text"].strip()
            if not text:
                print("没有识别到有效语音。下一步：按 r 重新录音，按 q 退出。")
                continue

            print(f"识别出的指令：{text}")
            print("正在请求 DeepSeek 解析指令并调用本地 MCP 工具...")

            # ---------- 3. 大模型解析 + MCP 工具调用 ----------
            history_length = len(messages)
            try:
                answer = asyncio.run(ask_agent(text, messages, api_key))
            except Exception as exc:
                del messages[history_length:]  # 失败时回滚本轮上下文
                print(f"处理失败：{exc}")
                print("下一步：按 r 重新开始本轮，按 q 退出。")
                continue

            print(f"\n最终回答：{answer}\n")
            print("本轮完成。下一步：按 r 发起新的语音指令，按 q 退出。")
    except KeyboardInterrupt:
        print("\n已退出。")
    finally:
        if recorder.is_recording:
            recorder.stop()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        audio.terminate()
        print("程序已退出")


if __name__ == "__main__":
    main()
