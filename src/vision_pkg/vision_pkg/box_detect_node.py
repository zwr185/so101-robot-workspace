"""Node detecting the largest colored box and publishing its center."""

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class BoxDetectNode(Node):
    """Detect a color box with fixed thresholds and publish its center."""

    def __init__(self):
        """Declare parameters, create the publisher and subscription."""
        super().__init__('box_detect_node')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('output_topic', 'box_center')
        self.declare_parameter('hue_min', 0)
        self.declare_parameter('hue_max', 26)
        self.declare_parameter('sat_min', 52)
        self.declare_parameter('sat_max', 193)
        self.declare_parameter('val_min', 222)
        self.declare_parameter('val_max', 255)
        self.declare_parameter('kernel_size', 5)
        self.declare_parameter('erode_iter', 1)
        self.declare_parameter('dilate_iter', 2)
        self.declare_parameter('min_area', 760.0)
        self.declare_parameter('show_window', True)

        image_topic = self.get_parameter('image_topic').value
        output_topic = self.get_parameter('output_topic').value

        self.lower = np.array([
            self.get_parameter('hue_min').value,
            self.get_parameter('sat_min').value,
            self.get_parameter('val_min').value,
        ])
        self.upper = np.array([
            self.get_parameter('hue_max').value,
            self.get_parameter('sat_max').value,
            self.get_parameter('val_max').value,
        ])
        self.kernel_size = self.get_parameter('kernel_size').value
        self.erode_iter = self.get_parameter('erode_iter').value
        self.dilate_iter = self.get_parameter('dilate_iter').value
        self.min_area = self.get_parameter('min_area').value
        self.show_window = self.get_parameter('show_window').value

        self.bridge = CvBridge()
        self.publisher_ = self.create_publisher(
            PointStamped, output_topic, 10
        )

        if self.show_window:
            cv2.namedWindow('Mask', cv2.WINDOW_NORMAL)
            cv2.namedWindow('Result', cv2.WINDOW_NORMAL)

        self.subscription = self.create_subscription(
            Image, image_topic, self._on_image, 10
        )
        self.get_logger().info(
            f'box_detect_node 已启动，订阅: {image_topic}，发布坐标: '
            f'{output_topic}，HSV 下限: {self.lower.tolist()}，'
            f'HSV 上限: {self.upper.tolist()}'
        )

    def _on_image(self, msg):
        """Detect boxes, publish the largest center and show the result."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'图像转换失败: {exc}')
            return

        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(img_hsv, self.lower, self.upper)

        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
        if self.erode_iter > 0:
            mask = cv2.erode(mask, kernel, iterations=self.erode_iter)
        if self.dilate_iter > 0:
            mask = cv2.dilate(mask, kernel, iterations=self.dilate_iter)

        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        result = frame.copy() if self.show_window else None
        largest = None
        for contour in contours:
            area = cv2.contourArea(contour)
            if area <= self.min_area:
                continue

            x, y, w, h = cv2.boundingRect(contour)
            if largest is None or area > largest[0]:
                largest = (area, x, y, w, h)

            if result is not None:
                cv2.rectangle(result, (x, y), (x + w, y + h), (0, 0, 255), 2)

        if largest is not None:
            _, x, y, w, h = largest
            cx = x + w // 2
            cy = y + h // 2
            self._publish_center(msg.header.frame_id, cx, cy)

            if result is not None:
                cv2.circle(result, (cx, cy), 5, (0, 255, 0), cv2.FILLED)
                cv2.putText(
                    result, f'({cx}, {cy})', (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2
                )

        if result is not None:
            cv2.imshow('Mask', mask)
            cv2.imshow('Result', result)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('按下 q，退出显示')
                rclpy.shutdown()

    def _publish_center(self, frame_id, cx, cy):
        """Publish the pixel center of the detected box."""
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        msg.point.x = float(cx)
        msg.point.y = float(cy)
        msg.point.z = 0.0
        self.publisher_.publish(msg)

    def destroy_node(self):
        """Close all OpenCV windows before destroying the node."""
        if self.show_window:
            cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """Initialize ROS 2 and run the box detection node."""
    rclpy.init(args=args)
    node = BoxDetectNode()
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
