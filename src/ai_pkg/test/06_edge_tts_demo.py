"""文本转语音演示：使用 edge-tts 将文本合成为中文语音（mp3）并播放。"""

import argparse
import asyncio
import os
import shutil
import subprocess

import edge_tts

# ==================== 中文语音配置 ====================
# 修改下面这行 DEFAULT_VOICE 即可切换默认朗读声音，例如：
#   DEFAULT_VOICE = "zh-CN-YunxiNeural"      # 男声、年轻阳光
#   DEFAULT_VOICE = "zh-CN-YunyangNeural"    # 男声、新闻播报
#   DEFAULT_VOICE = "zh-CN-XiaoyiNeural"     # 女声、活泼
# 也可以在运行时临时指定：python3 06_edge_tts_demo.py "你好" -v zh-CN-YunxiNeural
# 完整列表可随时查询：python3 06_edge_tts_demo.py --list-voices
#
# 普通话（zh-CN）
#   zh-CN-XiaoxiaoNeural          女声  晓晓，自然通用（默认）
#   zh-CN-XiaoyiNeural            女声  晓伊，年轻活泼
#   zh-CN-YunxiNeural             男声  云希，年轻阳光
#   zh-CN-YunjianNeural           男声  云健，浑厚有力
#   zh-CN-YunxiaNeural            男声  云夏，少年感
#   zh-CN-YunyangNeural           男声  云扬，新闻播报
#   zh-CN-liaoning-XiaobeiNeural  女声  晓北，辽宁方言
#   zh-CN-shaanxi-XiaoniNeural    女声  晓妮，陕西方言
#
# 粤语（zh-HK）
#   zh-HK-HiuGaaiNeural           女声  晓佳
#   zh-HK-HiuMaanNeural           女声  晓曼
#   zh-HK-WanLungNeural           男声  云龙
#
# 台湾国语（zh-TW）
#   zh-TW-HsiaoChenNeural         女声  晓臻
#   zh-TW-HsiaoYuNeural           女声  晓雨
#   zh-TW-YunJheNeural            男声  云哲
# ======================================================
DEFAULT_VOICE = "zh-CN-XiaoyiNeural"
OUTPUT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "edge_tts_output.mp3")


def parse_args():
    parser = argparse.ArgumentParser(description="edge-tts 文本转语音演示")
    parser.add_argument(
        "text",
        nargs="?",
        default="你好，这是 edge-tts 文本转语音演示。",
        help="要合成的文本",
    )
    parser.add_argument(
        "-v",
        "--voice",
        default=DEFAULT_VOICE,
        help=f"语音名称，默认 {DEFAULT_VOICE}",
    )
    parser.add_argument("-r", "--rate", default="+0%", help="语速变化，如 +20%%、-20%%，默认 +0%%")
    parser.add_argument("--volume", default="+0%", help="音量变化，如 +20%%、-20%%，默认 +0%%")
    parser.add_argument("-o", "--output", default=OUTPUT_FILE, help="输出 mp3 路径")
    parser.add_argument("--list-voices", action="store_true", help="列出所有中文语音后退出")
    parser.add_argument("--no-play", action="store_true", help="只生成文件，不自动播放")
    return parser.parse_args()


async def list_chinese_voices():
    voices = await edge_tts.list_voices()
    for voice in voices:
        if voice["Locale"].startswith("zh-"):
            print(f"{voice['ShortName']:<32} {voice['Gender']:<7} {voice['Locale']}")


async def synthesize(args):
    communicate = edge_tts.Communicate(
        args.text,
        args.voice,
        rate=args.rate,
        volume=args.volume,
    )
    await communicate.save(args.output)
    print(f"合成完成：{args.output}")


def play(path):
    if shutil.which("ffplay"):
        subprocess.run(
            ["ffplay", "-nodisp", "-autoexit", "-loglevel", "quiet", path],
            check=False,
        )
    else:
        print(f"未找到 ffplay，可手动播放：{path}")


def main():
    args = parse_args()

    if args.list_voices:
        asyncio.run(list_chinese_voices())
        return

    asyncio.run(synthesize(args))

    if not args.no_play:
        play(args.output)


if __name__ == "__main__":
    main()
