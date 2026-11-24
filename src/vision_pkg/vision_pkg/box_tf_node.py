"""Node converting detected box pixels into arm base frame coordinates."""

import os

from geometry_msgs.msg import Point, PointStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class BoxTfNode(Node):
    """Convert pixel centers to real coordinates in centimeters."""

    def __init__(self):
        """Declare parameters and create the publisher and subscription."""
        super().__init__('box_tf_node')
        self.declare_parameter('input_topic', '/box_center')
        self.declare_parameter('output_topic', '/box_world_pos')
        self.declare_parameter('mode', 'auto')
        self.declare_parameter(
            'homography_file',
            '/home/zwr185/dev_ws/src/vision_pkg/json/'
            'box_tf_homography.json',
        )
        self.declare_parameter('image_width', 1280)
        self.declare_parameter('image_height', 720)
        self.declare_parameter('camera_x', 7.0)
        self.declare_parameter('camera_y', 37.0)
        self.declare_parameter('camera_z', 45.0)
        self.declare_parameter('target_z', 0.0)
        self.declare_parameter('cm_per_pixel', 0.08)
        self.declare_parameter('x_direction', 1)
        self.declare_parameter('y_direction', 1)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.mode = self.get_parameter('mode').value
        self.homography_file = os.path.expanduser(
            self.get_parameter('homography_file').value
        )
        self.camera_x = float(self.get_parameter('camera_x').value)
        self.camera_y = float(self.get_parameter('camera_y').value)
        self.camera_z = float(self.get_parameter('camera_z').value)
        self.target_z = float(self.get_parameter('target_z').value)
        self.scale = float(self.get_parameter('cm_per_pixel').value)
        self.x_direction = int(self.get_parameter('x_direction').value)
        self.y_direction = int(self.get_parameter('y_direction').value)
        self.center_u = float(self.get_parameter('image_width').value) / 2.0
        self.center_v = float(self.get_parameter('image_height').value) / 2.0

        self.homography = self._load_homography()

        self.publisher_ = self.create_publisher(Point, output_topic, 10)
        self.subscription = self.create_subscription(
            PointStamped, input_topic, self._on_center, 10
        )
        if self.homography is not None:
            source = f'透视变换 ({self.homography_file})'
        else:
            source = (
                '线性近似（未找到可用透视变换文件，先运行 '
                'box_tf_calibrate 生成）'
            )
        self.get_logger().info(
            f'box_tf_node 已启动，订阅: {input_topic}，发布: {output_topic}，'
            f'像素→底座坐标方式: {source}，目标平面高度: {self.target_z} cm'
        )

    def _load_homography(self):
        """Load a fitted pixel-to-base homography when available."""
        if self.mode == 'linear':
            return None

        try:
            import json
            with open(self.homography_file, 'r', encoding='utf-8') as file:
                data = json.load(file)
            values = data.get('homography', data)
            matrix = np.asarray(values, dtype=float).reshape(3, 3)
        except (OSError, ValueError, TypeError) as exc:
            if self.mode == 'homography':
                self.get_logger().warn(
                    f'无法读取透视变换文件 {self.homography_file}: {exc}，'
                    f'回退到线性近似'
                )
            return None

        if abs(matrix[2, 2]) < 1e-12:
            self.get_logger().warn('透视变换矩阵无效，回退到线性近似')
            return None
        return matrix / matrix[2, 2]

    def _on_center(self, msg):
        """Convert the pixel center and publish the real coordinate."""
        pixel_u = float(msg.point.x)
        pixel_v = float(msg.point.y)

        if self.homography is not None:
            vector = self.homography @ np.array(
                [pixel_u, pixel_v, 1.0], dtype=float
            )
            if abs(vector[2]) < 1e-9:
                self.get_logger().warn(
                    f'像素坐标 ({pixel_u:.0f}, {pixel_v:.0f}) 落在透视变换的'
                    f'无穷远点，忽略'
                )
                return
            real_x = vector[0] / vector[2]
            real_y = vector[1] / vector[2]
        else:
            offset_u = (
                (pixel_u - self.center_u) * self.scale * self.x_direction
            )
            offset_v = (
                (pixel_v - self.center_v) * self.scale * self.y_direction
            )
            real_x = self.camera_x - offset_u
            real_y = self.camera_y + offset_v
        real_z = self.target_z

        point = Point()
        point.x = float(real_x)
        point.y = float(real_y)
        point.z = float(real_z)
        self.publisher_.publish(point)


def main(args=None):
    """Initialize ROS 2 and run the box TF node."""
    rclpy.init(args=args)
    node = BoxTfNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
