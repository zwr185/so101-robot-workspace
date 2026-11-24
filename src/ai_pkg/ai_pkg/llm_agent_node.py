"""LLM Agent 节点：录音 → Whisper 识别 → DeepSeek 解析 → 通过 MCP 调用工具。

流程：
  1. 按 r 录音、按 s 停止（参考 01_audio_record.py）
  2. Whisper 将录音转为文本（参考 03_whisper_asr.py）
  3. 文本提交给 DeepSeek，并把 MCP Server 的工具注册为 function calling（参考 10_deepseek_chat.py）
  4. DeepSeek 请求调用工具时，由本节点通过 MCP Client 调用 mcp_server_node 执行
  5. 工具结果回传给 DeepSeek，得到最终回答
  6. 用 edge-tts 播报最终回答（参考 06_edge_tts_demo.py）

运行方式：
  export DEEPSEEK_API_KEY="sk-你的密钥"
  ros2 run ai_pkg llm_agent_node

  默认由本节点自动以 stdio 方式拉起本地 mcp_server_node；
  若已单独启动 HTTP 模式的 MCP 服务，可指定 mcp_url 直接连接：
      ros2 run ai_pkg llm_agent_node \
          --ros-args -p mcp_url:=http://127.0.0.1:8000/mcp
"""

import asyncio
import json
import os
import select
import shutil
import subprocess
import sys
import termios
import threading
import tty
import wave
from contextlib import asynccontextmanager
from pathlib import Path

import edge_tts
import pyaudio
import rclpy
import whisper
from fastmcp import Client
from fastmcp.client.transports import StdioTransport
from openai import OpenAI
from rclpy.node import Node

# 音频参数（与 01_audio_record.py 一致）
RATE = 16000              # 采样率 16kHz
CHANNELS = 1              # 单声道
FORMAT = pyaudio.paInt16  # 16 位量化
CHUNK = 1024              # 每次读取的帧数

# 运行期临时文件统一放在包的 tmp 目录，可随时删除，下一轮会自动重建
TMP_DIR = Path('/home/zwr185/dev_ws/src/ai_pkg/tmp')
WAV_FILE = TMP_DIR / "ai_pkg_output.wav"
TTS_FILE = TMP_DIR / "ai_pkg_tts.mp3"

SIMPLIFIED_PROMPT = "以下是普通话的句子，请使用简体中文输出。"

SYSTEM_PROMPT = (
    "你是一个语音助手的决策模块。用户会通过语音提出请求，"
    "你必须调用提供的工具来完成任务，不要自行编造结果。"
    "可用工具："
    "robot_nod（控制机械臂点头）；"
    "robot_shake_head（控制机械臂摇头）；"
    "robot_remote_control（开启或停止机械臂遥操作，参数 action 取 start 或 stop）；"
    "robot_grasp_box（视觉定位目标并控制机械臂抓取，放到固定放置点）。"
    "例如用户说“让机械臂点个头”“点个头”“点头”就调用 robot_nod；"
    "用户说“摇个头”“摇摇头”“让机械臂摇头”就调用 robot_shake_head；"
    "用户说“开启遥操作”“开始遥操作”就调用 robot_remote_control，action 填 start；"
    "用户说“停止遥操作”“关闭遥操作”就调用 robot_remote_control，action 填 stop；"
    "遥操作开启后，用户手动拖动领导臂，从动臂会跟随；"
    "此时不要调用其他动作工具，除非用户明确要求先停止遥操作；"
    "用户说“抓取箱子”“抓箱子”“抓取球”“抓球”“抓物体”“抓东西”"
    "“抓一下”“帮我抓一下”“把前面的东西抓起来”“把物体拿起来”"
    "等任何抓取类请求，都调用 robot_grasp_box；"
    "不需要区分目标具体是箱子、球还是其他物体，统一使用视觉抓取工具。"
    "如果用户的请求无法通过工具完成，请用简洁的中文说明。"
    "拿到工具结果后，用简洁的中文告诉用户最终结果。"
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

        WAV_FILE.parent.mkdir(parents=True, exist_ok=True)
        with wave.open(str(WAV_FILE), "wb") as wf:
            wf.setnchannels(CHANNELS)
            wf.setsampwidth(self.audio.get_sample_size(FORMAT))
            wf.setframerate(RATE)
            wf.writeframes(b"".join(self.frames))


def read_key(fd, timeout=0.1):
    """非阻塞读取单个按键，无输入时返回 None。"""
    if select.select([fd], [], [], timeout)[0]:
        return os.read(fd, 1).decode(errors='ignore').lower()
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


async def synthesize_speech(text, voice, output):
    """使用 edge-tts 将文本合成为 mp3 文件。"""
    communicate = edge_tts.Communicate(text, voice)
    await communicate.save(str(output))


def play_audio(path):
    """使用 ffplay 播放音频文件。"""
    if shutil.which("ffplay") is None:
        print(f"未找到 ffplay，可手动播放：{path}")
        return
    subprocess.run(
        ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", str(path)],
        check=False,
    )


class LlmAgentNode(Node):
    """语音指令 Agent 的 ROS 2 节点。"""

    def __init__(self):
        super().__init__('llm_agent_node')
        self.declare_parameter('whisper_model', 'large-v3-turbo')
        self.declare_parameter('llm_model', 'deepseek-chat')
        self.declare_parameter('base_url', 'https://api.deepseek.com')
        # 留空表示自动以 stdio 方式拉起本地 mcp_server_node；
        # 填入 URL（如 http://127.0.0.1:8000/mcp）则连接已启动的 HTTP MCP 服务
        self.declare_parameter('mcp_url', '')
        self.declare_parameter('tts_voice', 'zh-CN-XiaoyiNeural')

        self.whisper_model = self.get_parameter('whisper_model').value
        self.llm_model = self.get_parameter('llm_model').value
        self.base_url = self.get_parameter('base_url').value
        self.mcp_url = self.get_parameter('mcp_url').value
        self.tts_voice = self.get_parameter('tts_voice').value
        self.api_key = os.environ.get('DEEPSEEK_API_KEY')

    @asynccontextmanager
    async def _mcp_connection(self):
        """连接 MCP 服务：mcp_url 非空时走 HTTP，否则自动拉起本地 stdio 服务。"""
        if self.mcp_url:
            async with Client(self.mcp_url) as mcp_client:
                yield mcp_client
        else:
            transport = StdioTransport(
                command='ros2',
                args=['run', 'ai_pkg', 'mcp_server_node'],
                # MCP SDK 默认只传递少量环境变量，需完整传递 ROS 环境
                env=dict(os.environ),
            )
            async with Client(transport) as mcp_client:
                yield mcp_client

    async def ask_agent(self, user_text, messages):
        """把文本提交给 DeepSeek，若模型要求调用工具则通过 MCP 执行并回传结果。"""
        async with self._mcp_connection() as mcp_client:
            mcp_tools = await mcp_client.list_tools()
            tools = to_openai_tools(mcp_tools)
            print(f"已连接本地 MCP Server，可用工具：{[tool.name for tool in mcp_tools]}")

            llm = OpenAI(api_key=self.api_key, base_url=self.base_url)
            messages.append({"role": "user", "content": user_text})

            for _ in range(MAX_TOOL_ROUNDS):
                response = llm.chat.completions.create(
                    model=self.llm_model,
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

    def speak(self, text):
        """用 edge-tts 播报大模型回答。"""
        if not text:
            return
        try:
            TTS_FILE.parent.mkdir(parents=True, exist_ok=True)
            asyncio.run(synthesize_speech(text, self.tts_voice, TTS_FILE))
        except Exception as exc:
            self.get_logger().warn(f'语音播报失败: {exc}')
            return
        play_audio(TTS_FILE)

    def run(self):
        """加载模型并进入交互式主循环，直到用户按 q 或 Ctrl+C 退出。"""
        # stdout 被 ros2 launch 重定向为管道时默认是块缓冲，改为行缓冲保证提示即时显示
        sys.stdout.reconfigure(line_buffering=True)

        if not self.api_key:
            self.get_logger().error(
                '未检测到环境变量 DEEPSEEK_API_KEY，'
                '请先执行：export DEEPSEEK_API_KEY="sk-你的密钥"')
            return

        # 直接用 /dev/tty 读取按键：ros2 launch 启动时子进程的 stdin 不是终端
        try:
            tty_file = open('/dev/tty', 'r')
        except OSError:
            self.get_logger().error(
                '无法打开 /dev/tty，本节点需要在终端中运行。')
            return

        # 优先使用系统完整版 ffmpeg，避免 PATH 中的精简版无法解码 wav
        os.environ["PATH"] = "/usr/bin" + os.pathsep + os.environ.get("PATH", "")

        TMP_DIR.mkdir(parents=True, exist_ok=True)
        print(f"临时文件目录：{TMP_DIR}")

        print(f"正在加载 Whisper 模型：{self.whisper_model} ...")
        model = whisper.load_model(self.whisper_model)
        print("Whisper 模型加载完成。")

        audio = pyaudio.PyAudio()
        recorder = Recorder(audio)

        fd = tty_file.fileno()
        old_settings = termios.tcgetattr(fd)

        messages = [{"role": "system", "content": SYSTEM_PROMPT}]

        print(BANNER)
        print("下一步：按 r 开始录音，按 q 退出。")

        try:
            tty.setcbreak(fd)  # 单键输入，无需回车
            while True:
                key = read_key(fd)
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
                    key = read_key(fd)
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
                    answer = asyncio.run(self.ask_agent(text, messages))
                except Exception as exc:
                    del messages[history_length:]  # 失败时回滚本轮上下文
                    print(f"处理失败：{exc}")
                    print("下一步：按 r 重新开始本轮，按 q 退出。")
                    continue

                print(f"\n最终回答：{answer}\n")
                self.speak(answer)
                print("本轮完成。下一步：按 r 发起新的语音指令，按 q 退出。")
        except KeyboardInterrupt:
            print("\n已退出。")
        finally:
            if recorder.is_recording:
                recorder.stop()
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            tty_file.close()
            audio.terminate()
            print("程序已退出")


def main(args=None):
    rclpy.init(args=args)
    node = LlmAgentNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
