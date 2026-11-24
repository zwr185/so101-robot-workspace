#!/usr/bin/env python
# -*- coding: utf-8 -*-
import json
import sys
import time
from pathlib import Path

import serial
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider

# --- 校准文件 / Calibration file ---
# LeRobot 校准文件：homing_offset 为关节零点(中位)的原始编码器值，
# range_min / range_max 为该关节原始编码器的有效范围。
DEFAULT_CALIBRATION_FILE = Path(
    "/home/zwr185/dev_ws/Seeed_RoboController/json/my_awesome_follower_arm.json"
)

# STS3215: 4096 计数对应 360°
COUNTS_PER_DEGREE = 4096.0 / 360.0
POSITION_MIN = 0
POSITION_MAX = 4095


def load_calibration(path: Path) -> dict:
    """读取 LeRobot 校准文件，返回 {servo_id: joint_info}"""
    with open(path, "r", encoding="utf-8") as f:
        data = json.load(f)

    joints = {}
    for joint_name, info in data.items():
        joints[int(info["id"])] = {
            "joint_name": joint_name,
            "homing_offset": int(info["homing_offset"]),
            "range_min": int(info["range_min"]),
            "range_max": int(info["range_max"]),
            "drive_mode": int(info.get("drive_mode", 0)),
        }
    return joints


def position_to_angle(joint: dict, position: float) -> float:
    """原始位置计数 -> 角度（相对校准零点，单位：度）"""
    sign = -1.0 if joint["drive_mode"] else 1.0
    return sign * (position - joint["homing_offset"]) / COUNTS_PER_DEGREE


def angle_to_position(joint: dict, angle: float) -> int:
    """角度（相对校准零点）-> 原始位置计数，并限制在校准范围内"""
    sign = -1.0 if joint["drive_mode"] else 1.0
    position = joint["homing_offset"] + sign * angle * COUNTS_PER_DEGREE
    position = int(round(position))
    position = max(joint["range_min"], min(joint["range_max"], position))
    return max(POSITION_MIN, min(POSITION_MAX, position))


def get_angle_limits(joint: dict) -> tuple:
    """由校准范围计算滑块的角度上下限"""
    a1 = position_to_angle(joint, joint["range_min"])
    a2 = position_to_angle(joint, joint["range_max"])
    low, high = sorted((a1, a2))
    return max(low, -180.0), min(high, 180.0)


# --- ServoController Class (adapted from previous scripts) ---

class ServoController:
    def __init__(self, port="/dev/ttyACM0", baudrate=1000000, timeout=0.1, joints=None):
        self.port_name = port
        self.serial_port = None
        self.joints = joints or {}
        # Constants
        self.ADDR_GOAL_POSITION = 42
        self.ADDR_TORQUE_ENABLE = 40
        self.INST_WRITE = 3
        self.COMM_SUCCESS = 0
        self.COMM_RX_TIMEOUT = -6
        self.COMM_RX_CORRUPT = -7

        try:
            self.serial_port = serial.Serial(port, baudrate=baudrate, timeout=timeout)
            time.sleep(0.1)
            self.serial_port.reset_input_buffer()
            print(f"Successfully opened port {port}.")
        except serial.SerialException as e:
            print(f"Fatal: Could not open port {port}: {e}")
            sys.exit(1)

    def close(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print(f"Port {self.port_name} closed.")

    def _calculate_checksum(self, data):
        return (~sum(data)) & 0xFF

    def _send_packet(self, servo_id, instruction, parameters=None):
        if not self.serial_port or not self.serial_port.is_open:
            return False
        if parameters is None:
            parameters = []
        length = len(parameters) + 2
        packet_core = [servo_id, length, instruction] + parameters
        checksum = self._calculate_checksum(packet_core)
        packet = bytes([0xFF, 0xFF] + packet_core + [checksum])
        try:
            self.serial_port.reset_input_buffer()
            self.serial_port.write(packet)
            self.serial_port.flush()
            return True
        except Exception:
            return False

    def _read_packet(self):
        start_time = time.time()
        packet = []
        while (time.time() - start_time) < self.serial_port.timeout:
            if self.serial_port.in_waiting > 0:
                byte = self.serial_port.read(1)
                if not byte: continue
                byte = byte[0]
                if not packet and byte != 0xFF:
                    continue
                packet.append(byte)
                if len(packet) >= 2 and packet[-2:] == [0xFF, 0xFF]:
                    if len(packet) > 2:
                        packet = [0xFF, 0xFF]
                    continue
                if len(packet) > 4:
                    pkt_len = packet[3]
                    if len(packet) == pkt_len + 4:
                        core_data = packet[2:-1]
                        calculated_checksum = self._calculate_checksum(core_data)
                        if calculated_checksum == packet[-1]:
                            return self.COMM_SUCCESS, packet[4], packet[5:-1]
                        else:
                            return self.COMM_RX_CORRUPT, 0, []
        return self.COMM_RX_TIMEOUT, 0, []

    def _write_register(self, servo_id, address, value, size=2):
        params = [address]
        if size == 1:
            params.append(value & 0xFF)
        elif size == 2:
            params.extend([value & 0xFF, (value >> 8) & 0xFF])
        else:
            return False

        # For write operations, we often don't need to wait for a response
        # unless we need to confirm the write was successful.
        # Removing the read can make operations faster, especially for sliders.
        if not self._send_packet(servo_id, self.INST_WRITE, params):
            return False

        # We assume success if the packet was sent.
        return True

    def enable_torque(self, servo_id):
        # This is a fire-and-forget write. We don't check the response.
        return self._write_register(servo_id, self.ADDR_TORQUE_ENABLE, 1, size=1)

    def disable_torque(self, servo_id):
        # Release the servo so the joint can be moved by hand.
        return self._write_register(servo_id, self.ADDR_TORQUE_ENABLE, 0, size=1)

    def set_servo_angle(self, servo_id, angle):
        """Sets the servo to an angle (degrees) relative to the calibrated zero position."""
        joint = self.joints[servo_id]
        position = angle_to_position(joint, angle)
        ok = self._write_register(servo_id, self.ADDR_GOAL_POSITION, position, size=2)
        return ok, position


# --- Main function with GUI ---

def main():
    """Main function - creates a GUI to control all student servos."""
    # Configuration
    STUDENT_PORT = "/dev/ttyACM0"  # Adjust COM port if necessary
    BAUDRATE = 1000000
    calibration_file = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_CALIBRATION_FILE

    if not calibration_file.is_file():
        print(f"Fatal: calibration file not found: {calibration_file}")
        sys.exit(1)

    joints = load_calibration(calibration_file)
    SERVO_IDS = sorted(joints)

    print(f"Calibration file: {calibration_file}")
    for servo_id in SERVO_IDS:
        joint = joints[servo_id]
        low, high = get_angle_limits(joint)
        print(
            f"  ID{servo_id} {joint['joint_name']:<14} "
            f"zero={joint['homing_offset']:4d} "
            f"range=[{joint['range_min']}, {joint['range_max']}] "
            f"angle=[{low:7.1f}, {high:7.1f}]deg"
        )

    controller = None
    try:
        controller = ServoController(port=STUDENT_PORT, baudrate=BAUDRATE, joints=joints)

        # Enable torque for all servos first
        print("Enabling torque for all servos...")
        for servo_id in SERVO_IDS:
            controller.enable_torque(servo_id)
            time.sleep(0.05) # Small delay between commands
        print("Torque enabled for all servos.")

        # Setup matplotlib GUI
        fig, ax = plt.subplots(figsize=(9, 6.5))
        plt.subplots_adjust(left=0.32, bottom=0.1, top=0.9, right=0.9)
        fig.suptitle('Student Servo Angle Control (calibrated)')

        sliders = {}
        for i, servo_id in enumerate(SERVO_IDS):
            joint = joints[servo_id]
            low, high = get_angle_limits(joint)
            ax_slider = plt.axes([0.32, 0.1 + i * 0.12, 0.58, 0.05])
            slider = Slider(
                ax=ax_slider,
                label=f"ID{servo_id} {joint['joint_name']}",
                valmin=low,
                valmax=high,
                valinit=min(max(0.0, low), high),
                valfmt='%0.1f°'
            )
            sliders[servo_id] = slider

        def update_factory(servo_id):
            def update(val):
                angle = sliders[servo_id].val
                ok, position = controller.set_servo_angle(servo_id, angle)
                if not ok:
                    print(f"\rFailed to set angle for servo {servo_id}", end="")
                else:
                    print(
                        f"\rServo {servo_id} ({joints[servo_id]['joint_name']}) "
                        f"angle={angle:6.1f}°  raw={position:4d}   ",
                        end=""
                    )
            return update

        for servo_id in SERVO_IDS:
            sliders[servo_id].on_changed(update_factory(servo_id))

        print("\nGUI is ready. Drag sliders to control servos. Close the window to exit.")
        plt.show()

    except KeyboardInterrupt:
        print("\nStopping program.")
    except Exception as e:
        print(f"\nAn error occurred: {e}")
    finally:
        if controller:
            print("\nDisabling torque for all servos...")
            for servo_id in SERVO_IDS:
                controller.disable_torque(servo_id)
                time.sleep(0.05)
            print("Torque disabled. The arm can now be moved by hand.")
            controller.close()
        print("\nProgram finished.")


if __name__ == '__main__':
    main()
