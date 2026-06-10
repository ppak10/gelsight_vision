#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class GelSightPublisher(Node):
    def __init__(self):
        super().__init__('gelsight_publisher')

        # Parameters (override at launch: --ros-args -p device_id:=2)
        self.declare_parameter('device_id', 0)
        self.declare_parameter('frame_rate', 25.0)
        self.declare_parameter('width', 320)
        self.declare_parameter('height', 240)

        device_id = self.get_parameter('device_id').value
        frame_rate = self.get_parameter('frame_rate').value
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value

        self.publisher_ = self.create_publisher(Image, 'gelsight/image_raw', 10)
        self.bridge = CvBridge()

        # Open the GelSight Mini (it's a USB camera)
        self.cap = cv2.VideoCapture(device_id)
        if not self.cap.isOpened():
            self.get_logger().error(f'Cannot open GelSight at /dev/video{device_id}')
            raise RuntimeError('GelSight device not found')

        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        self.timer = self.create_timer(1.0 / frame_rate, self.publish_frame)
        self.get_logger().info(f'GelSight publisher started on /dev/video{device_id}')

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to grab frame')
            return

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gelsight_frame'
        self.publisher_.publish(msg)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GelSightPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()