#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
舵机 PID 参数调节工具
Servo PID Tuning Tool

用于读取/写入 STS/SMS 系列舵机内部闭环 P/D/I 参数（EEPROM，掉电保存）。

寄存器地址 / Register addresses:
    0x15 (21)  P 系数 / P coefficient，出厂默认 32
    0x16 (22)  D 系数 / D coefficient，出厂默认 32
    0x17 (23)  I 系数 / I coefficient，出厂默认 0

用法 / Usage:
    python -m src.tools.servo_pid_tune                    # 交互式选端口，读取 ID1~6
    python -m src.tools.servo_pid_tune /dev/ttyUSB0       # 指定端口，读取 ID1~6
    python -m src.tools.servo_pid_tune --list             # 列出可用串口
    python -m src.tools.servo_pid_tune --id 2 --p 24 --d 32 --i 0
    python -m src.tools.servo_pid_tune --p 24 --d 32 -y   # 批量写入 ID1~6
"""

import sys
import os
import json
import time
import argparse
from datetime import datetime

# 项目根目录加入路径，保证可直接运行本文件
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..'))
sys.path.insert(0, PROJECT_ROOT)

try:
    from scservo_sdk.port_handler import PortHandler
    from scservo_sdk.sms_sts import sms_sts
    from scservo_sdk.scservo_def import COMM_SUCCESS
except ImportError as e:
    print(f"❌ 错误: 无法导入 SCServo SDK: {e}")
    print("   Error: Cannot import SCServo SDK")
    sys.exit(1)

try:
    from src.port_utils import select_port_interactive, list_ports_for_user
except ImportError:
    print("❌ 错误: 未找到 port_utils")
    print("   Error: port_utils not found")
    sys.exit(1)

# === 配置常量 ===
BAUD_RATE = 1000000
SMS_STS_TORQUE_ENABLE = 40

SERVO_P_ADDR = 21   # 0x15 P 系数
SERVO_D_ADDR = 22   # 0x16 D 系数
SERVO_I_ADDR = 23   # 0x17 I 系数

PID_PARAMS = (('p', SERVO_P_ADDR), ('d', SERVO_D_ADDR), ('i', SERVO_I_ADDR))
PID_VALUE_MAX = 254
DEFAULT_SERVO_IDS = [1, 2, 3, 4, 5, 6]
BACKUP_DIR = os.path.join(PROJECT_ROOT, 'json')


def scan_servos(servo_handler) -> list:
    """扫描端口上的所有舵机"""
    found = []
    for servo_id in range(1, 21):
        _, result, _ = servo_handler.ping(servo_id)
        if result == COMM_SUCCESS:
            found.append(servo_id)
    return found


def read_pid(servo_handler, servo_id: int):
    """读取单个舵机的 P/D/I，任一读取失败返回 None"""
    values = {}
    for name, addr in PID_PARAMS:
        value, result, _ = servo_handler.read1ByteTxRx(servo_id, addr)
        if result != COMM_SUCCESS:
            return None
        values[name] = value
    return values


def print_pid_values(title: str, rows: dict):
    """打印 PID 表格"""
    print(title)
    print("-" * 50)
    print(f"{'ID':<5}{'P':<7}{'D':<7}{'I':<7}")
    for servo_id in sorted(rows):
        values = rows[servo_id]
        print(f"{servo_id:<5}{values['p']:<7}{values['d']:<7}{values['i']:<7}")
    print("-" * 50)


def save_backup(port_name: str, snapshot: dict) -> str:
    """写入前备份原始 P/D/I，返回备份文件路径"""
    os.makedirs(BACKUP_DIR, exist_ok=True)
    now = datetime.now()
    path = os.path.join(BACKUP_DIR, f"servo_pid_backup_{now.strftime('%Y%m%d_%H%M%S')}.json")

    data = {
        'timestamp': now.strftime('%Y-%m-%d %H:%M:%S'),
        'port': port_name,
        'baud_rate': BAUD_RATE,
        'servos': {str(servo_id): values for servo_id, values in snapshot.items()},
    }
    with open(path, 'w', encoding='utf-8') as f:
        json.dump(data, f, indent=4, ensure_ascii=False)
    return path


def write_pid(servo_handler, servo_id: int, changes: dict):
    """
    写入指定舵机的 P/D/I

    EEPROM 写入需要先断力矩；若写入失败会尝试解锁 EEPROM 后重试一次。
    返回 (是否成功, 说明)
    """
    torque_state, result, _ = servo_handler.read1ByteTxRx(servo_id, SMS_STS_TORQUE_ENABLE)
    was_enabled = result == COMM_SUCCESS and torque_state == 1

    if was_enabled:
        servo_handler.write1ByteTxRx(servo_id, SMS_STS_TORQUE_ENABLE, 0)
        time.sleep(0.05)

    unlocked = False
    try:
        for name, addr in PID_PARAMS:
            if name not in changes:
                continue

            value = changes[name]
            result, error = servo_handler.write1ByteTxRx(servo_id, addr, value)

            if result != COMM_SUCCESS:
                # EEPROM 可能被锁定，解锁后重试一次
                servo_handler.unLockEprom(servo_id)
                unlocked = True
                time.sleep(0.05)
                result, error = servo_handler.write1ByteTxRx(servo_id, addr, value)

            if result != COMM_SUCCESS:
                return False, f"写入 {name.upper()} 失败: result={result}, error={error}"
    finally:
        if was_enabled:
            servo_handler.write1ByteTxRx(servo_id, SMS_STS_TORQUE_ENABLE, 1)
        if unlocked:
            servo_handler.LockEprom(servo_id)

    # 回读校验
    current = read_pid(servo_handler, servo_id)
    if current is None:
        return True, "写入成功（回读失败，无法校验）"

    mismatch = [
        name.upper() for name, _ in PID_PARAMS
        if name in changes and current[name] != changes[name]
    ]
    if mismatch:
        return False, f"回读不一致: {', '.join(mismatch)}"
    return True, "成功"


def run_pid_tune(port_name: str, servo_ids: list, changes: dict, assume_yes: bool = False) -> bool:
    """读取/写入舵机 PID"""
    print(f"\n{'=' * 50}")
    print("🔧 舵机 PID 参数调节工具 / Servo PID Tuning")
    print(f"{'=' * 50}")
    print(f"端口 / Port: {port_name}")
    print(f"目标舵机 / Target IDs: {servo_ids}")
    if changes:
        desc = "  ".join(f"{name.upper()}={value}" for name, value in changes.items())
        print(f"写入参数 / New values: {desc}")
    else:
        print("模式 / Mode: 仅读取（未指定 --p/--d/--i）")
    print(f"{'=' * 50}\n")

    port_handler = None
    try:
        port_handler = PortHandler(port_name)
        if not port_handler.openPort():
            print(f"❌ 无法打开串口 / Cannot open {port_name}")
            return False
        if not port_handler.setBaudRate(BAUD_RATE):
            print("❌ 无法设置波特率 / Cannot set baud rate")
            return False

        servo_handler = sms_sts(port_handler)

        print("📡 扫描舵机 / Scanning servos...")
        found = scan_servos(servo_handler)
        if not found:
            print("❌ 未发现舵机 / No servos found")
            return False
        print(f"✅ 发现舵机 / Found: {found}\n")

        missing = [servo_id for servo_id in servo_ids if servo_id not in found]
        if missing:
            print(f"⚠️ 以下 ID 未响应，已跳过 / No response, skipped: {missing}\n")

        snapshot = {}
        for servo_id in servo_ids:
            if servo_id not in found:
                continue
            values = read_pid(servo_handler, servo_id)
            if values is None:
                print(f"⚠️ ID{servo_id} P/D/I 读取失败，已跳过 / Read failed, skipped")
                continue
            snapshot[servo_id] = values

        if not snapshot:
            print("❌ 没有可操作的舵机 / No servo available")
            return False

        print_pid_values("📖 当前 PID / Current PID", snapshot)

        if not changes:
            print("\nℹ️ 仅读取模式，未修改任何参数 / Read-only, nothing changed")
            return True

        print("\n⚠️ 写入 EEPROM 时会暂时断力矩，机械臂可能因重力下落，请先托住机械臂！")
        print("   Torque will be disabled during EEPROM write; support the arm first!")
        if not assume_yes:
            try:
                answer = input("继续写入? / Continue? [y/N]: ").strip().lower()
            except EOFError:
                answer = ""
            if answer not in ('y', 'yes'):
                print("⏹️ 已取消 / Cancelled")
                return False
        print()

        backup_path = save_backup(port_name, snapshot)
        print(f"💾 原值已备份 / Backup saved: {backup_path}\n")

        results = []
        for servo_id in snapshot:
            print(f"✏️ ID{servo_id} 写入中... / Writing", end=" ")
            success, message = write_pid(servo_handler, servo_id, changes)
            print("✅" if success else "❌")
            results.append((servo_id, success, message))
            time.sleep(0.05)

        final = {}
        for servo_id in snapshot:
            values = read_pid(servo_handler, servo_id)
            if values is not None:
                final[servo_id] = values
        if final:
            print()
            print_pid_values("📖 写入后 PID / PID after write", final)

        failed = [(servo_id, message) for servo_id, success, message in results if not success]
        print("=" * 50)
        if failed:
            print(f"⚠️ 部分失败 / Partial: {len(results) - len(failed)}/{len(results)} 个舵机成功")
            for servo_id, message in failed:
                print(f"   ID{servo_id}: {message}")
        else:
            print(f"✅ 全部成功 / All Success! {len(results)}/{len(results)} 个舵机已更新")
        print("=" * 50)
        return not failed

    except Exception as e:
        print(f"\n❌ 异常 / Error: {e}")
        return False
    finally:
        if port_handler is not None:
            try:
                port_handler.closePort()
            except Exception:
                pass


def main():
    parser = argparse.ArgumentParser(
        description='舵机 PID 参数调节工具（读写 P/D/I 系数）',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例 / Examples:
  python -m src.tools.servo_pid_tune                         # 读取 ID1~6 当前 PID
  python -m src.tools.servo_pid_tune /dev/ttyUSB0 --id 2     # 读取 ID2
  python -m src.tools.servo_pid_tune --id 2 --p 24 --d 32    # 写入 ID2 的 P/D
  python -m src.tools.servo_pid_tune --id 1 2 3 --p 24       # 写入多个 ID
  python -m src.tools.servo_pid_tune --p 24 --d 32 -y        # 批量写入 ID1~6

说明 / Notes:
  寄存器 0x15=P, 0x16=D, 0x17=I，取值范围 0~254，出厂默认 P=32, D=32, I=0。
  写入前会自动备份原始值到 json/servo_pid_backup_<时间戳>.json。
        """
    )
    parser.add_argument('port', nargs='?', help='串口名称，不指定则交互式选择')
    parser.add_argument('--id', dest='servo_ids', nargs='+', type=int, default=None,
                        help='目标舵机 ID，可多个，如 --id 1 或 --id 1 2 3（默认 1~6）')
    parser.add_argument('--p', type=int, default=None, help='P 系数（0~254）')
    parser.add_argument('--d', type=int, default=None, help='D 系数（0~254）')
    parser.add_argument('--i', type=int, default=None, help='I 系数（0~254）')
    parser.add_argument('-y', '--yes', action='store_true', help='跳过写入前的确认')
    parser.add_argument('--list', action='store_true', help='列出可用串口')

    args = parser.parse_args()

    if args.list:
        print("=== 可用串口 / Available Serial Ports ===")
        print(list_ports_for_user())
        return

    # 校验 ID
    servo_ids = args.servo_ids if args.servo_ids else DEFAULT_SERVO_IDS
    servo_ids = list(dict.fromkeys(servo_ids))
    for servo_id in servo_ids:
        if not 1 <= servo_id <= 253:
            parser.error(f"舵机 ID 超出范围: {servo_id}（应为 1~253）")

    # 校验并收集要写入的参数
    changes = {}
    for name in ('p', 'd', 'i'):
        value = getattr(args, name)
        if value is not None:
            if not 0 <= value <= PID_VALUE_MAX:
                parser.error(f"--{name} 取值范围为 0~{PID_VALUE_MAX}")
            changes[name] = value

    if args.port:
        port_name = args.port
        print(f"🔌 使用指定端口 / Using specified port: {port_name}")
    else:
        port_name = select_port_interactive("选择 PID 调参串口 / Select port")
        if not port_name:
            print("❌ 未选择端口 / No port selected")
            sys.exit(1)

    success = run_pid_tune(port_name, servo_ids, changes, args.yes)
    sys.exit(0 if success else 1)


if __name__ == "__main__":
    main()
