"""录音脚本：使用 pyaudio 以 16kHz 采集麦克风音频，按键控制，保存为 output.wav。"""

import os
import select
import sys
import termios
import threading
import tty
import wave

import pyaudio

# 音频参数
RATE = 16000          # 采样率 16kHz
CHANNELS = 1          # 单声道
FORMAT = pyaudio.paInt16  # 16 位量化
CHUNK = 1024          # 每次读取的帧数
OUTPUT_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), "output.wav")

MENU = """
=============== 录音程序 ===============
  r : 开始录音
  s : 停止录音并保存为 output.wav
  q : 退出程序
========================================
"""


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
            print("正在录音中，无需重复开始")
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
        print("开始录音...（按 s 停止，按 q 退出）")

    def _capture(self):
        while self.recording:
            self.frames.append(self.stream.read(CHUNK, exception_on_overflow=False))

    def stop(self):
        if not self.recording:
            print("当前没有在录音，请先按 r 开始")
            return
        self.recording = False
        self.thread.join()
        self.stream.stop_stream()
        self.stream.close()
        self.stream = None

        with wave.open(OUTPUT_FILE, "wb") as wf:
            wf.setnchannels(CHANNELS)
            wf.setsampwidth(self.audio.get_sample_size(FORMAT))
            wf.setframerate(RATE)
            wf.writeframes(b"".join(self.frames))

        duration = len(self.frames) * CHUNK / RATE
        print(f"录音结束，已保存到 {OUTPUT_FILE}（约 {duration:.2f} 秒）")


def main():
    audio = pyaudio.PyAudio()
    recorder = Recorder(audio)

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    print(MENU)
    try:
        tty.setcbreak(fd)  # 单键输入，无需回车
        while True:
            # 每 0.1 秒检查一次键盘输入
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1).lower()
                if key == "r":
                    recorder.start()
                elif key == "s":
                    recorder.stop()
                elif key == "q":
                    break
    except KeyboardInterrupt:
        pass
    finally:
        if recorder.is_recording:
            recorder.stop()
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        audio.terminate()
        print("\n程序已退出")


if __name__ == "__main__":
    main()
