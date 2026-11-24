"""Node subscribing to camera images and showing them with OpenCV."""

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class CameraNativeNode(Node):
    """Subscribe to an image topic and display frames in an OpenCV window."""

    def __init__(self):
        """Create the image subscription and the OpenCV bridge."""
        super().__init__('camera_native_node')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('window_name', 'camera_native_node')

        topic = self.get_parameter('image_topic').value
        self.window_name = self.get_parameter('window_name').value

        self.bridge = CvBridge()
        # WINDOW_NORMAL lets the user resize the window by dragging its edges.
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        self.subscription = self.create_subscription(
            Image, topic, self._on_image, 10
        )
        self.get_logger().info(f'camera_native_node 已启动，订阅图像话题: {topic}')

    def _on_image(self, msg):
        """Convert the incoming image to BGR and show it."""
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'图像转换失败: {exc}')
            return

        cv2.imshow(self.window_name, frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('按下 q，退出显示')
            rclpy.shutdown()

    def destroy_node(self):
        """Close all OpenCV windows before destroying the node."""
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """Initialize ROS 2 and run the camera display node."""
    rclpy.init(args=args)
    node = CameraNativeNode()
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
