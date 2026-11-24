"""Terminal key listener: press p to exit the teleoperation launch."""

import os
import select
import termios
import time
import tty


def main():
    """Print the hint and block until the user presses p."""
    print('遥操作已启动：领导臂先运行，1 秒后启动从动臂。', flush=True)
    print('按 p 退出遥操作（Ctrl+C 也可以）', flush=True)

    try:
        tty_fd = os.open('/dev/tty', os.O_RDONLY)
    except OSError as exc:
        print(
            f'无法读取终端键盘（{exc}），请使用 Ctrl+C 退出',
            flush=True,
        )
        # 没有终端时保持存活，避免误触发 launch 关闭
        while True:
            time.sleep(1.0)

    old_settings = termios.tcgetattr(tty_fd)
    try:
        tty.setcbreak(tty_fd)
        while True:
            if select.select([tty_fd], [], [], 0.2)[0]:
                key = os.read(tty_fd, 1).decode(errors='ignore').lower()
                if key == 'p':
                    print('\n收到 p 键，正在退出遥操作...', flush=True)
                    return 0
    except KeyboardInterrupt:
        return 0
    finally:
        termios.tcsetattr(tty_fd, termios.TCSADRAIN, old_settings)
        os.close(tty_fd)


if __name__ == '__main__':
    raise SystemExit(main())
