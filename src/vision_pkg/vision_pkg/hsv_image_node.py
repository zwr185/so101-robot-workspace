"""Node applying HSV thresholding and locating objects in camera images."""

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


def _noop(_value):
    """Trackbar callback that ignores its argument."""
    pass


class HsvImageNode(Node):
    """Show an HSV mask and the located object for each camera frame."""

    def __init__(self):
        """Create the trackbar panel and the image subscription."""
        super().__init__('hsv_image_node')
        self.declare_parameter('image_topic', '/image_raw')

        topic = self.get_parameter('image_topic').value
        self.bridge = CvBridge()

        self.params_window = 'Parameters'
        cv2.namedWindow(self.params_window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.params_window, 640, 350)
        cv2.createTrackbar('Hue Min', self.params_window, 0, 179, _noop)
        cv2.createTrackbar('Hue Max', self.params_window, 179, 179, _noop)
        cv2.createTrackbar('Sat Min', self.params_window, 0, 255, _noop)
        cv2.createTrackbar('Sat Max', self.params_window, 255, 255, _noop)
        cv2.createTrackbar('Val Min', self.params_window, 0, 255, _noop)
        cv2.createTrackbar('Val Max', self.params_window, 255, 255, _noop)
        cv2.createTrackbar('Erode Iter', self.params_window, 1, 10, _noop)
        cv2.createTrackbar('Dilate Iter', self.params_window, 1, 10, _noop)
        cv2.createTrackbar('Min Area', self.params_window, 1000, 20000, _noop)

        cv2.namedWindow('Mask', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Result', cv2.WINDOW_NORMAL)

        self.subscription = self.create_subscription(
            Image, topic, self._on_image, 10
        )
        self.get_logger().info(
            f'hsv_image_node 已启动，订阅图像话题: {topic}，按 q 退出'
        )

    def _trackbar(self, name):
        """Read one trackbar value from the parameter panel."""
        return cv2.getTrackbarPos(name, self.params_window)

    def _on_image(self, msg):
        """Threshold the frame with the trackbar values and show results."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'图像转换失败: {exc}')
            return

        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower = np.array(
            [self._trackbar('Hue Min'), self._trackbar('Sat Min'),
             self._trackbar('Val Min')]
        )
        upper = np.array(
            [self._trackbar('Hue Max'), self._trackbar('Sat Max'),
             self._trackbar('Val Max')]
        )
        mask = cv2.inRange(img_hsv, lower, upper)

        kernel = np.ones((5, 5), np.uint8)
        erode_iter = self._trackbar('Erode Iter')
        dilate_iter = self._trackbar('Dilate Iter')
        if erode_iter > 0:
            mask = cv2.erode(mask, kernel, iterations=erode_iter)
        if dilate_iter > 0:
            mask = cv2.dilate(mask, kernel, iterations=dilate_iter)

        min_area = self._trackbar('Min Area')
        result = frame.copy()
        contours, _ = cv2.findContours(
            mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        for contour in contours:
            if cv2.contourArea(contour) <= min_area:
                continue

            perimeter = cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
            x, y, w, h = cv2.boundingRect(approx)
            cv2.rectangle(result, (x, y), (x + w, y + h), (0, 0, 255), 2)

            cx = x + w // 2
            cy = y + h // 2
            cv2.circle(result, (cx, cy), 5, (0, 255, 0), cv2.FILLED)
            cv2.putText(
                result, f'({cx}, {cy})', (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2
            )

        cv2.imshow('Mask', mask)
        cv2.imshow('Result', result)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('按下 q，退出显示')
            rclpy.shutdown()

    def destroy_node(self):
        """Close all OpenCV windows before destroying the node."""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """Initialize ROS 2 and run the HSV image node."""
    rclpy.init(args=args)
    node = HsvImageNode()
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
