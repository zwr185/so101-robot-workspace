"""语音识别脚本：使用 OpenAI Whisper 识别 output.wav 并输出文本。"""

import argparse
import os
import sys

import whisper

DEFAULT_AUDIO = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output.wav")
# 引导 Whisper 输出简体中文（模型对中文可能默认输出繁体）
SIMPLIFIED_PROMPT = "以下是普通话的句子，请使用简体中文输出。"


def parse_args():
    parser = argparse.ArgumentParser(description="使用 Whisper 将语音文件识别为文本")
    parser.add_argument(
        "audio",
        nargs="?",
        default=DEFAULT_AUDIO,
        help="待识别的音频文件路径，默认脚本同目录的 output.wav",
    )
    parser.add_argument(
        "-m",
        "--model",
        default="large-v3-turbo",
        help="模型大小：tiny / base / small / medium / large-v3-turbo / large-v3-16bit，默认 large-v3-turbo",
    )
    parser.add_argument(
        "-l",
        "--language",
        default="zh",
        help="识别语言，默认 zh（中文）",
    )
    return parser.parse_args()


def main():
    # 优先使用系统完整版 ffmpeg：PATH 中的精简版 ffmpeg 可能不支持 wav 解码
    os.environ["PATH"] = "/usr/bin" + os.pathsep + os.environ.get("PATH", "")

    args = parse_args()

    if not os.path.isfile(args.audio):
        print(f"找不到音频文件：{args.audio}", file=sys.stderr)
        sys.exit(1)

    print(f"正在加载 Whisper 模型：{args.model} ...")
    model = whisper.load_model(args.model)

    print(f"正在识别：{args.audio}")
    # 中文识别时通过提示词引导模型输出简体，而非繁体
    prompt = SIMPLIFIED_PROMPT if args.language.startswith("zh") else None
    result = model.transcribe(args.audio, language=args.language, initial_prompt=prompt)

    print("\n识别结果：")
    print(result["text"].strip())


if __name__ == "__main__":
    main()
