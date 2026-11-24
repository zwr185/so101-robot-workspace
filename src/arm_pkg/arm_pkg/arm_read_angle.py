"""Interactive terminal node for sending and reading arm joint angles."""

import select
import sys
import time

import rclpy
from rclpy.node import Node

from arm_msg.msg import ArmJointAngles

from arm_pkg.arm_calibration import load_calibration, servo_calibrations
from arm_pkg.arm_joint_node import ServoController


class ArmTerminalCtrlNode(Node):
    """Send arm angles from the terminal and print the current angles."""

    def __init__(self):
        """Open the serial port, release torque and create the publisher."""
        super().__init__('arm_terminal_ctrl_node')
        self.declare_parameter('calibration_file', '')
        calibration_file = self.get_parameter('calibration_file').value
        entries = load_calibration(calibration_file)
        self.servo_ids = list(range(1, 7))
        self.controller = ServoController(
            port='/dev/ttyACM0',
            calibration=servo_calibrations(entries),
        )

        if not self.controller.serial_port or not self.controller.serial_port.is_open:
            self.get_logger().warn(
                'Failed to open serial port /dev/ttyACM0. Running in dummy mode?'
            )
        if entries:
            self.get_logger().info(f'已加载校准文件: {calibration_file}')

        # Release torque so the arm can be moved by hand.
        for servo_id in self.servo_ids:
            self.controller.disable_torque(servo_id)
            time.sleep(0.05)

        self.publisher_ = self.create_publisher(
            ArmJointAngles, 'arm_joint_angles', 10
        )
        self.get_logger().info('arm_terminal_ctrl_node 已启动')

    def run(self):
        """Run the interactive loop until the user quits."""
        self._print_help()
        last_read = 0.0
        while True:
            ready, _, _ = select.select([sys.stdin], [], [], 0.1)
            if ready:
                line = sys.stdin.readline()
                if not line:
                    break
                if not self._handle_command(line.strip()):
                    break

            now = time.monotonic()
            if now - last_read >= 1.0:
                self._read_and_print()
                last_read = now

    def _handle_command(self, line):
        """Handle one command line, return False to quit."""
        if not line:
            return True

        command = line.lower()
        if command == 'q':
            return False
        if command == 'r':
            for servo_id in self.servo_ids:
                self.controller.disable_torque(servo_id)
                time.sleep(0.02)
            print('已释放所有舵机扭矩，可手动摆动机械臂。', flush=True)
            return True

        parts = line.replace(',', ' ').split()
        if len(parts) != 6:
            self._print_help()
            return True

        try:
            angles = [float(value) for value in parts]
        except ValueError:
            print('输入格式错误：请输入 6 个数字，例如：0 0 0 0 0 0', flush=True)
            return True

        for servo_id, angle in zip(self.servo_ids, angles):
            self.controller.enable_torque(servo_id)
            self.controller.set_servo_angle(servo_id, angle)
        print(
            '已发送角度：' + ', '.join(f'{angle:.2f}' for angle in angles),
            flush=True,
        )
        return True

    def _read_and_print(self):
        """Read all servo angles, publish them and print them."""
        angles = []
        for servo_id in self.servo_ids:
            angle = self.controller.get_servo_angle(servo_id)
            angles.append(0.0 if angle is None else angle)

        msg = ArmJointAngles()
        msg.angles = [float(angle) for angle in angles]
        self.publisher_.publish(msg)

        values = ' '.join(f'J{i + 1}={angle:6.2f}' for i, angle in enumerate(angles))
        print(f'当前角度(度): {values}', flush=True)

    @staticmethod
    def _print_help():
        print(
            '输入 6 个角度（单位：度，空格或逗号分隔）并回车发送，例如：0 0 0 0 0 0\n'
            '输入 r 释放所有舵机扭矩（可手动摆动机械臂）\n'
            '输入 q 退出程序',
            flush=True,
        )

    def destroy_node(self):
        """Close the serial port before destroying the node."""
        if hasattr(self, 'controller') and self.controller:
            self.controller.close()
        super().destroy_node()


def main(args=None):
    """Initialize ROS 2 and run the interactive node."""
    rclpy.init(args=args)
    node = ArmTerminalCtrlNode()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
