"""Direct-write serial controller for follower arm servos.

Based on arm_pkg/arm_joint_node.py, but keeps only the direct motion path:
no 1.2s smoothing thread, suitable for 50Hz teleoperation commands.
"""

import threading
import time

import serial

from remote_control_pkg.arm_calibration import angle_to_raw, raw_to_angle

ADDR_PRESENT_POSITION = 56
ADDR_GOAL_POSITION = 42
ADDR_TORQUE_ENABLE = 40
ADDR_ACC = 41
INST_READ = 2
INST_WRITE = 3

COMM_SUCCESS = 0
COMM_RX_TIMEOUT = -6
COMM_RX_CORRUPT = -7

SERVO_ACC = 30
SERVO_MAX_SPEED = 2400


class ServoController:
    """Raw-protocol STS servo controller with direct position writes."""

    ADDR_PRESENT_POSITION = ADDR_PRESENT_POSITION
    ADDR_GOAL_POSITION = ADDR_GOAL_POSITION
    ADDR_TORQUE_ENABLE = ADDR_TORQUE_ENABLE
    INST_READ = INST_READ
    INST_WRITE = INST_WRITE
    COMM_SUCCESS = COMM_SUCCESS
    COMM_RX_TIMEOUT = COMM_RX_TIMEOUT
    COMM_RX_CORRUPT = COMM_RX_CORRUPT

    def __init__(self, port, baudrate=1000000, timeout=0.02, calibration=None):
        self.port_name = port
        self.serial_port = None
        self.calibration = calibration or {}
        self._io_lock = threading.RLock()

        try:
            self.serial_port = serial.Serial(
                port, baudrate=baudrate, timeout=timeout
            )
            time.sleep(0.1)
            self.serial_port.reset_input_buffer()
        except serial.SerialException:
            self.serial_port = None

    @property
    def is_open(self):
        """Return True when the serial port is open."""
        return bool(self.serial_port and self.serial_port.is_open)

    def close(self):
        """Close the serial port."""
        if self.is_open:
            self.serial_port.close()

    @staticmethod
    def _calculate_checksum(data):
        return (~sum(data)) & 0xFF

    def _send_packet(self, servo_id, instruction, parameters=None):
        with self._io_lock:
            if not self.is_open:
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
        with self._io_lock:
            if not self.is_open:
                return self.COMM_RX_TIMEOUT, 0, []

            start_time = time.time()
            packet = []
            while (time.time() - start_time) < self.serial_port.timeout:
                if self.serial_port.in_waiting > 0:
                    byte = self.serial_port.read(1)
                    if not byte:
                        continue
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
                            checksum = self._calculate_checksum(core_data)
                            if checksum == packet[-1]:
                                return (
                                    self.COMM_SUCCESS,
                                    packet[4],
                                    packet[5:-1],
                                )
                            return self.COMM_RX_CORRUPT, 0, []
            return self.COMM_RX_TIMEOUT, 0, []

    def _write_register(self, servo_id, address, value, size=1):
        params = [address]
        if size == 1:
            params.append(value & 0xFF)
        elif size == 2:
            params.extend([value & 0xFF, (value >> 8) & 0xFF])
        else:
            return False
        return self._send_packet(servo_id, self.INST_WRITE, params)

    def enable_torque(self, servo_id):
        """Enable servo torque."""
        return self._write_register(servo_id, self.ADDR_TORQUE_ENABLE, 1)

    def disable_torque(self, servo_id):
        """Disable servo torque so the joint can be moved by hand."""
        return self._write_register(servo_id, self.ADDR_TORQUE_ENABLE, 0)

    def angle_to_position(self, servo_id, angle):
        """Convert a joint angle to a clamped raw position."""
        entry = self.calibration.get(servo_id)
        if entry is not None:
            return angle_to_raw(entry, angle)

        position = int(
            ((angle + 90.0) / 180.0) * (3072.0 - 1024.0) + 1024.0
        )
        return max(1024, min(3072, position))

    def write_position(self, servo_id, position, speed=SERVO_MAX_SPEED):
        """Write ACC + goal position + speed in one packet (direct, no smoothing)."""
        position = max(0, min(4095, int(position)))
        params = [
            ADDR_ACC,
            SERVO_ACC & 0xFF,
            position & 0xFF,
            (position >> 8) & 0xFF,
            0,
            0,
            speed & 0xFF,
            (speed >> 8) & 0xFF,
        ]
        return self._send_packet(servo_id, self.INST_WRITE, params)

    def set_servo_angle(self, servo_id, angle):
        """Convert the angle and write the goal position immediately."""
        position = self.angle_to_position(servo_id, angle)
        return self.write_position(servo_id, position)

    def read_raw_position(self, servo_id):
        """Read the raw present position, return None on failure."""
        with self._io_lock:
            if not self._send_packet(
                servo_id, self.INST_READ, [ADDR_PRESENT_POSITION, 2]
            ):
                return None
            result, error, data = self._read_packet()

        if result != self.COMM_SUCCESS or error != 0:
            return None
        if data and len(data) >= 2:
            return data[0] | (data[1] << 8)
        return None

    def read_angle(self, servo_id):
        """Read the joint angle in degrees, return None on failure."""
        position = self.read_raw_position(servo_id)
        if position is None:
            return None

        entry = self.calibration.get(servo_id)
        if entry is not None:
            return raw_to_angle(entry, position)

        angle = ((position - 1024.0) / (3072.0 - 1024.0)) * 180.0 - 90.0
        return max(-90.0, min(90.0, angle))
