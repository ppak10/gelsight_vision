#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class GelSightSubscriber(Node):
    def __init__(self):
        super().__init__('gelsight_subscriber')
        self.subscription = self.create_subscription(
            Image, 'gelsight/image_raw', self.listener_callback, 10)
        self.bridge = CvBridge()
        self.get_logger().info('GelSight subscriber started')

    def listener_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.get_logger().info(f'Received frame: {frame.shape}')
        cv2.imshow('GelSight Mini', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = GelSightSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()