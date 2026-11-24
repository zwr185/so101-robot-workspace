"""MCP Server 示例：基于 FastMCP 提供加减乘除与机械臂点头工具。

运行方式：
  1) stdio（默认，供 MCP 客户端以子进程方式拉起）：
       python3 12_mcp_server.py
     在终端中手动运行时，可按 q 键退出。
  2) HTTP（便于调试，可配合 MCP Inspector 或浏览器）：
       python3 12_mcp_server.py --http
     服务地址：http://127.0.0.1:8000/mcp
"""

import argparse
import os
import select
import sys
import termios
import threading
import tty

from fastmcp import FastMCP

# 创建 MCP 服务实例
mcp = FastMCP("计算器服务")


def log(message):
    """日志统一写 stderr，避免污染 stdio 模式的协议输出通道。"""
    print(message, file=sys.stderr, flush=True)


@mcp.tool
def add(a: int, b: int) -> int:
    """将两个整数相加并返回结果。"""
    result = a + b
    log(f"Executing add({a}, {b}) -> {result}")
    return result


@mcp.tool
def subtract(a: int, b: int) -> int:
    """将两个整数相减并返回结果（a - b）。"""
    result = a - b
    log(f"Executing subtract({a}, {b}) -> {result}")
    return result


@mcp.tool
def multiply(a: int, b: int) -> int:
    """将两个整数相乘并返回结果。"""
    result = a * b
    log(f"Executing multiply({a}, {b}) -> {result}")
    return result


@mcp.tool
def divide(a: int, b: int) -> float:
    """将两个整数相除并返回结果，除数不能为 0。"""
    if b == 0:
        log(f"Executing divide({a}, {b}) -> error: division by zero")
        raise ValueError("除数不能为 0")
    result = a / b
    log(f"Executing divide({a}, {b}) -> {result}")
    return result


@mcp.tool
def robot_nod() -> str:
    """控制机械臂执行点头动作（模拟）。"""
    log("Executing robot_nod()...")
    log("LOG: Robot Arm Moving: Position 0 -> Position 1 (Nod Down)")
    log("LOG: Robot Arm Moving: Position 1 -> Position 0 (Nod Up)")
    return "Robot arm nodded successfully."


def listen_quit_key():
    """stdio 模式下监听 q 键退出。

    MCP 协议占用了 stdin/stdout，所以按键从 /dev/tty 读取；
    只有在终端手动运行（stdin 是终端）时才启用，避免干扰客户端连接。
    """
    if not sys.stdin.isatty():
        return

    try:
        tty_fd = os.open("/dev/tty", os.O_RDONLY)
    except OSError:
        return

    old_settings = termios.tcgetattr(tty_fd)
    try:
        tty.setcbreak(tty_fd)  # 单键输入，无需回车
        while True:
            if select.select([tty_fd], [], [], 0.2)[0]:
                key = os.read(tty_fd, 1).decode(errors="ignore").lower()
                if key == "q":
                    termios.tcsetattr(tty_fd, termios.TCSADRAIN, old_settings)
                    # stdout 是 MCP 协议通道，提示信息必须写到 stderr
                    print("\n收到 q 键，MCP 服务已退出", file=sys.stderr, flush=True)
                    os._exit(0)
    finally:
        termios.tcsetattr(tty_fd, termios.TCSADRAIN, old_settings)
        os.close(tty_fd)


def main():
    parser = argparse.ArgumentParser(description="FastMCP 示例服务（加减乘除与机械臂工具）")
    parser.add_argument("--http", action="store_true", help="以 HTTP 方式启动，默认 stdio")
    args = parser.parse_args()

    if args.http:
        mcp.run(transport="http", host="127.0.0.1", port=8000)
    else:
        print("stdio 模式已启动，等待 MCP 客户端连接；在终端中按 q 键可退出。",
              file=sys.stderr, flush=True)
        threading.Thread(target=listen_quit_key, daemon=True).start()
        mcp.run()


if __name__ == "__main__":
    main()
