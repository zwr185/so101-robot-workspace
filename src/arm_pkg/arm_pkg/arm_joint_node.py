import rclpy
from rclpy.node import Node
from arm_msg.msg import ArmJointAngles
from arm_pkg.arm_calibration import (
    angle_to_raw,
    load_calibration,
    raw_to_angle,
    servo_calibrations,
)
import serial
import math
import threading
import time
import sys

# --- 平滑运动参数（与 04_write_angle_stable.py 一致）---
SMOOTH_DURATION = 1.2          # 大角度指令的平滑时长（秒）
SMOOTH_TICK = 0.03             # 轨迹下发周期（约 33Hz）
SMOOTH_MIN_DELTA = int(10 * 4096.0 / 360.0)  # 超过约 10 度才做 1.2s 平滑
SERVO_ACC = 30                 # 舵机加速度参数（0~254）
SERVO_MAX_SPEED = 2400         # 速度上限
SERVO_MIN_SPEED = 60           # 速度下限
SMOOTH_SPEED_MARGIN = 1.8      # 余弦缓动峰值速度约为平均速度的 1.5 倍，留余量
ADDR_ACC = 41                  # ACC / 目标位置 / 时间 / 速度连续写入起始地址

# --- ServoController Class ---
class ServoController:
    def __init__(self, port="/dev/ttyUSB0", baudrate=1000000, timeout=0.1, calibration=None):
        self.port_name = port
        self.serial_port = None
        # {舵机 ID: 校准数据}，为空时使用旧的 -90..90 固定映射
        self.calibration = calibration or {}
        # Constants
        self.ADDR_PRESENT_POSITION = 56
        self.ADDR_GOAL_POSITION = 42
        self.ADDR_TORQUE_ENABLE = 40
        self.INST_READ = 2
        self.INST_WRITE = 3
        self.COMM_SUCCESS = 0
        self.COMM_RX_TIMEOUT = -6
        self.COMM_RX_CORRUPT = -7

        # 串口访问锁：平滑轨迹线程与 ROS 回调可能同时读写串口
        self._io_lock = threading.RLock()
        self._motion_lock = threading.Lock()
        # {舵机 ID: 当前平滑轨迹参数}
        self._motions = {}
        # {舵机 ID: 最近一次下发的目标原始位置}
        self._last_goal = {}
        self._motion_stop = threading.Event()
        self._motion_thread = threading.Thread(
            target=self._motion_loop,
            name='servo-smooth-motion',
            daemon=True,
        )
        
        try:
            self.serial_port = serial.Serial(port, baudrate=baudrate, timeout=timeout)
            time.sleep(0.1)
            self.serial_port.reset_input_buffer()
            # print(f"Successfully opened port {port}.")
        except serial.SerialException as e:
            pass
            # print(f"Fatal: Could not open port {port}: {e}")
            # sys.exit(1)

        self._motion_thread.start()

    def close(self):
        self._motion_stop.set()
        if self._motion_thread.is_alive():
            self._motion_thread.join(timeout=1.0)
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()

    def _calculate_checksum(self, data):
        return (~sum(data)) & 0xFF

    def _send_packet(self, servo_id, instruction, parameters=None):
        with self._io_lock:
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
        with self._io_lock:
            if not self.serial_port or not self.serial_port.is_open:
                return self.COMM_RX_TIMEOUT, 0, []
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
        
        if not self._send_packet(servo_id, self.INST_WRITE, params):
            return False
        
        return True

    def enable_torque(self, servo_id):
        return self._write_register(servo_id, self.ADDR_TORQUE_ENABLE, 1, size=1)

    def disable_torque(self, servo_id):
        return self._write_register(servo_id, self.ADDR_TORQUE_ENABLE, 0, size=1)

    def _angle_to_position(self, servo_id, angle):
        """把关节角度换算成舵机原始位置"""
        entry = self.calibration.get(servo_id)
        if entry is not None:
            return angle_to_raw(entry, angle)
        # 无校准文件时沿用旧映射：-90..90 对应 1024..3072
        position = int(((angle + 90.0) / 180.0) * (3072.0 - 1024.0) + 1024.0)
        return max(1024, min(3072, position))

    def _write_position(self, servo_id, position, speed, acc=SERVO_ACC):
        """一次性写入 ACC + 目标位置 + 时间 + 速度（7 字节）"""
        position = max(0, min(4095, int(position)))
        params = [
            ADDR_ACC,
            acc & 0xFF,
            position & 0xFF,
            (position >> 8) & 0xFF,
            0,
            0,
            speed & 0xFF,
            (speed >> 8) & 0xFF,
        ]
        return self._send_packet(servo_id, self.INST_WRITE, params)

    def _read_raw_position(self, servo_id):
        """读取当前原始位置（0~4095），失败返回 None"""
        with self._io_lock:
            if not self._send_packet(
                servo_id, self.INST_READ, [self.ADDR_PRESENT_POSITION, 2]
            ):
                return None
            result, error, data = self._read_packet()

        if result != self.COMM_SUCCESS or error != 0:
            return None
        if data and len(data) >= 2:
            return data[0] | (data[1] << 8)
        return None

    def _planned_position(self, servo_id, now):
        """当前规划位置：运动中使用插值位置，否则用上次目标位置"""
        motion = self._motions.get(servo_id)
        if motion is not None:
            t = min((now - motion['t0']) / motion['duration'], 1.0)
            s = 0.5 - 0.5 * math.cos(math.pi * t)
            return motion['start'] + (motion['end'] - motion['start']) * s

        if servo_id in self._last_goal:
            return float(self._last_goal[servo_id])

        position = self._read_raw_position(servo_id)
        return float(position) if position is not None else 2048.0

    def _motion_loop(self):
        """后台线程：按固定周期下发平滑轨迹的中间位置"""
        while not self._motion_stop.is_set():
            now = time.monotonic()
            with self._motion_lock:
                finished = []
                for servo_id, motion in list(self._motions.items()):
                    t = (now - motion['t0']) / motion['duration']
                    if t >= 1.0:
                        position = motion['end']
                        finished.append(servo_id)
                    else:
                        s = 0.5 - 0.5 * math.cos(math.pi * t)
                        position = int(round(
                            motion['start']
                            + (motion['end'] - motion['start']) * s
                        ))
                    self._last_goal[servo_id] = position
                    self._write_position(servo_id, position, motion['speed'])

                for servo_id in finished:
                    self._motions.pop(servo_id, None)

            self._motion_stop.wait(SMOOTH_TICK)

    def set_servo_angle(self, servo_id, angle):
        """平滑移动到目标角度。

        大角度指令按 SMOOTH_DURATION（1.2s）余弦缓动插值；
        小角度增量（连续回放/跟踪）直接下发，避免拖慢原有节奏。
        """
        position = self._angle_to_position(servo_id, angle)
        now = time.monotonic()

        with self._motion_lock:
            start = self._planned_position(servo_id, now)
            delta = abs(position - start)

            if delta < SMOOTH_MIN_DELTA:
                self._motions.pop(servo_id, None)
                self._last_goal[servo_id] = position
                return self._write_position(
                    servo_id, position, SERVO_MAX_SPEED
                )

            speed = int(delta / SMOOTH_DURATION * SMOOTH_SPEED_MARGIN) + 20
            speed = max(SERVO_MIN_SPEED, min(SERVO_MAX_SPEED, speed))
            self._motions[servo_id] = {
                'start': start,
                'end': position,
                't0': now,
                'duration': SMOOTH_DURATION,
                'speed': speed,
            }
            return True

    def set_servo_angle_direct(self, servo_id, angle):
        """直接写入目标角度（不做平滑插值），用于轨迹回放等需要原速的场景。"""
        position = self._angle_to_position(servo_id, angle)

        with self._motion_lock:
            # 取消该关节正在进行的平滑轨迹，避免两路指令互相拉扯
            self._motions.pop(servo_id, None)
            self._last_goal[servo_id] = position
            return self._write_position(
                servo_id, position, SERVO_MAX_SPEED
            )

    def angle_to_position(self, servo_id, angle):
        """把关节角度换算成舵机原始位置（公开接口）。"""
        return self._angle_to_position(servo_id, angle)

    def read_raw_position(self, servo_id):
        """读取当前原始位置（公开接口）。"""
        return self._read_raw_position(servo_id)

    def write_position(self, servo_id, position, speed=SERVO_MAX_SPEED):
        """直接写入原始目标位置（公开接口，不做平滑）。"""
        with self._motion_lock:
            self._motions.pop(servo_id, None)
            self._last_goal[servo_id] = int(position)
        return self._write_position(servo_id, position, speed)

    def cancel_all_motions(self):
        """取消所有正在进行的平滑轨迹。"""
        with self._motion_lock:
            self._motions.clear()

    def get_servo_angle(self, servo_id):
        position = self._read_raw_position(servo_id)
        if position is None:
            return None

        entry = self.calibration.get(servo_id)
        if entry is not None:
            return raw_to_angle(entry, position)

        angle = ((position - 1024.0) / (3072.0 - 1024.0)) * 180.0 - 90.0
        return max(-90.0, min(90.0, angle))

class ArmJointNode(Node):
    def __init__(self):
        super().__init__('arm_joint_node')
        self.get_logger().info('Arm Joint Node has been started.')
        self.declare_parameter(
            'calibration_file',
            '/home/zwr185/dev_ws/Seeed_RoboController/json/'
            'my_awesome_follower_arm.json',
        )
        calibration_file = self.get_parameter('calibration_file').value
        entries = load_calibration(calibration_file)

        # Initialize ServoController
        # Using /dev/ttyACM0 as default for Linux
        self.controller = ServoController(
            port="/dev/ttyACM0",
            calibration=servo_calibrations(entries),
        )
        if not self.controller.serial_port or not self.controller.serial_port.is_open:
             self.get_logger().warn('Failed to open serial port /dev/ttyACM0. Running in dummy mode?')
        if entries:
            self.get_logger().info(
                f'已加载校准文件: {calibration_file}，'
                f'关节: {list(entries.keys())}'
            )
        elif calibration_file:
            self.get_logger().warn(f'校准文件无效，沿用默认映射: {calibration_file}')

        # Create a timer to read angles every 100ms
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.servo_ids = list(range(1, 7)) # Servos 1-6
        
        # Disable torque for all servos to allow manual movement
        for servo_id in self.servo_ids:
            self.controller.disable_torque(servo_id)
            time.sleep(0.05)
            
        self.publisher_ = self.create_publisher(ArmJointAngles, 'arm_joint_angles', 10)
        self.subscription_ = self.create_subscription(
            ArmJointAngles,
            'cmd_angles',
            self.listener_callback,
            10)
        self.subscription_  # prevent unused variable warning

    def listener_callback(self, msg):
        if len(msg.angles) != 6:
            self.get_logger().warn(f'Received invalid number of angles: {len(msg.angles)}. Expected 6.')
            return

        for i, angle in enumerate(msg.angles):
            servo_id = self.servo_ids[i]
            # Enable torque before moving
            self.controller.enable_torque(servo_id)
            # Ensure angle is float
            angle_val = float(angle)
            self.controller.set_servo_angle(servo_id, angle_val)

    def timer_callback(self):
        angles = []
        for servo_id in self.servo_ids:
            angle = self.controller.get_servo_angle(servo_id)
            if angle is not None:
                angles.append(angle)
            else:
                angles.append(0.0) # Default or error value
        
        msg = ArmJointAngles()
        msg.angles = [float(a) for a in angles]
        self.publisher_.publish(msg)
        # self.get_logger().info(f'Published Joint Angles: {angles}')

    def destroy_node(self):
        if hasattr(self, 'controller') and self.controller:
            self.controller.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ArmJointNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
