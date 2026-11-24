"""Load LeRobot calibration and convert joint angles to servo raw positions.

校准约定（与 LeRobot 一致）：
- homing_offset: 关节处于 0 度（中位）时的舵机原始位置；
- range_min/range_max: 实测的舵机原始位置边界；
- drive_mode: 0 表示角度正方向对应原始值增大，1 表示反向；
- 飞特 STS 系列舵机 4096 个计数对应 360 度。
所有角度单位为度，舵机位置为 0-4095 的整数。
"""

import json

COUNTS_PER_DEGREE = 4096.0 / 360.0


def load_calibration(path):
    """Load a calibration JSON file keyed by joint name."""
    if not path:
        return {}

    try:
        with open(path, 'r', encoding='utf-8') as file:
            data = json.load(file)
    except (OSError, ValueError) as exc:
        print(f'[remote_control_pkg] 读取校准文件失败 {path}: {exc}')
        return {}

    entries = {}
    for index, (name, item) in enumerate(data.items()):
        entries[name] = {
            'id': int(item.get('id', index + 1)),
            'drive_mode': int(item.get('drive_mode', 0)),
            'homing_offset': float(item.get('homing_offset', 2048)),
            'range_min': float(item.get('range_min', 0)),
            'range_max': float(item.get('range_max', 4095)),
        }
    return entries


def servo_calibrations(entries):
    """Return calibration entries keyed by servo ID."""
    return {entry['id']: entry for entry in entries.values()}


def _direction(entry):
    """Return the sign that maps angles to raw positions."""
    return 1.0 if entry['drive_mode'] == 0 else -1.0


def angle_to_raw(entry, angle):
    """Convert a joint angle in degrees to a clamped raw position."""
    raw = (
        entry['homing_offset']
        + _direction(entry) * float(angle) * COUNTS_PER_DEGREE
    )
    raw = max(entry['range_min'], min(entry['range_max'], raw))
    return int(round(max(0.0, min(4095.0, raw))))


def raw_to_angle(entry, raw):
    """Convert a servo raw position to a joint angle in degrees."""
    return (
        _direction(entry) * (float(raw) - entry['homing_offset'])
        / COUNTS_PER_DEGREE
    )
