import os
import cv2
import numpy as np
import rclpy
import threading
import time

from datetime import datetime
from action_msgs.msg import GoalStatus
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, Point, Quaternion, TwistStamped, WrenchStamped
from gelsight_vision.gelsight.height_map import HeightMapper, save_height_preview
from gelsight_vision.gelsight.stitch import stitch_session
from gelsight_vision.gelsight.touch import touch
from math import radians
from moveit_msgs.srv import ServoCommandType
from rclpy.action.client import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import Image
from robot_manager_interfaces.action import PoseGoal
from robot_manager_interfaces.srv import Home
from tf_transformations import quaternion_from_euler

class MotionCameraTouch(Node):
    def __init__(self):
        super().__init__('motion_camera_touch')

        # Assigns namespace to ur20
        _ = self.declare_parameter('ns', 'ur20')
        _ = self.declare_parameter('frame_id', 'plate')
        _ = self.declare_parameter('force_max', 4.0)
        _ = self.declare_parameter('mock', False)
        _ = self.declare_parameter('touch_x', 1)
        _ = self.declare_parameter('touch_y', 1)
        _ = self.declare_parameter(
            'capture_root',
            '/home/ppak/ros2_ws/src/gelsight_vision/captures',
        )

        self.ns: str = str(self.get_parameter('ns').value)
        self.frame_id: str = str(self.get_parameter('frame_id').value)
        self.force_max: float = float(self.get_parameter('force_max').value)
        self.mock: bool = bool(self.get_parameter('mock').value)
        self.touch_x: int = int(self.get_parameter('touch_x').value)
        self.touch_y: int = int(self.get_parameter('touch_y').value)

        self.gelsight_frame = None
        self.d555_frame = None

        # Force values
        self.force = 0.0
        self.offset = 0.0

        # Capture image flags
        self.gelsight_capture_image = False
        self.gelsight_calibrate = False
        self.d555_capture_image = False

        # Shared session folder for captured frames
        capture_root = str(self.get_parameter('capture_root').value)
        session = datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        self.capture_dir = os.path.join(capture_root, session)
        os.makedirs(self.capture_dir, exist_ok=True)
        self.gelsight_capture_count = 0
        self.d555_capture_count = 0

        self.height_mapper = HeightMapper()

        self.pose_goal_client: ActionClient = ActionClient(
            self,
            PoseGoal,
            self.ns + "/" + "pose_goal"
        )
        self.pose_goal_client.wait_for_server()

        self.home_client = self.create_client(Home, self.ns + "/" + "home")

        while not self.home_client.wait_for_service():
            sleep(1.0)
            print("waiting for service")
            continue

        # Force Subscriber
        self.force_subscriber = self.create_subscription(
            WrenchStamped, 
            # self.ns + '/force_torque_sensor_broadcaster/wrench_filtered',
            self.ns + '/wrench',
            self.force_callback, 
            10
        )

        self.cv_bridge = CvBridge()

        # Gelsight Subscriber
        self.gelsight_subscriber = self.create_subscription(
            Image,
            'gelsight_capture/image',
            self.gelsight_listener_callback,
            10
        )

        # D555 Subscriber
        self.d555_subscriber = self.create_subscription(
            Image,
            'gelsight_vision_d555/color/image_raw',
            self.d555_listener_callback,
            10
        )

        # Twist Publisher
        self.twist_publisher = self.create_publisher(
            TwistStamped, 
            self.ns + '/servo_node/delta_twist_cmds', 
            10
        )

        # Enable Servo
        self.servo_client = self.create_client(ServoCommandType, self.ns + '/servo_node/switch_command_type')
        while not self.servo_client.wait_for_service(timeout_sec=1.0): 
            self.get_logger().info("Waiting for service: " + self.ns + '/servo_node/switch_command_type')
        req = ServoCommandType.Request()
        req.command_type = ServoCommandType.Request.TWIST
        future = self.servo_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

    def gelsight_listener_callback(self, msg):
        if self.gelsight_calibrate:
            frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            baseline_path = os.path.join(self.capture_dir, "gelsight_baseline.png")
            cv2.imwrite(baseline_path, frame)
            self.height_mapper.calibrate(frame)
            print(f"gelsight baseline -> {baseline_path}")
            self.gelsight_calibrate = False
            return
        if self.gelsight_capture_image:
            self.gelsight_frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.gelsight_capture_count += 1
            stem = f"gelsight_{self.gelsight_capture_count:04d}"
            img_path = os.path.join(self.capture_dir, f"{stem}.png")
            depth_path = os.path.join(self.capture_dir, f"{stem}_depth.npy")
            preview_path = os.path.join(self.capture_dir, f"{stem}_depth_preview.png")
            cv2.imwrite(img_path, self.gelsight_frame)
            height = self.height_mapper.image_to_height_tensor(self.gelsight_frame).numpy()
            np.save(depth_path, height)
            save_height_preview(height, preview_path)
            print(f"gelsight captured frame -> {img_path}")
            print(f"gelsight depth -> {depth_path}")
            print(f"gelsight depth preview -> {preview_path}")
            self.gelsight_capture_image = False

    def d555_listener_callback(self, msg):
        if self.d555_capture_image:
            self.d555_frame = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            self.d555_capture_count += 1
            path = os.path.join(
                self.capture_dir,
                f"d555_{self.d555_capture_count:04d}.png"
            )
            cv2.imwrite(path, self.d555_frame)
            print(f"d555 captured frame -> {path}")
            self.d555_capture_image = False

    #### Movement
    def move_z(self, speed):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.frame_id
        twist_msg.twist.linear.z = speed
        self.twist_publisher.publish(twist_msg)
    def stop(self):
        twist_msg = TwistStamped()
        twist_msg.header.stamp = self.get_clock().now().to_msg()
        twist_msg.header.frame_id = self.frame_id
        twist_msg.twist.linear.x = 0.0
        twist_msg.twist.linear.y = 0.0
        twist_msg.twist.linear.z = 0.0
        twist_msg.twist.angular.x = 0.0
        twist_msg.twist.angular.y = 0.0
        twist_msg.twist.angular.z = 0.0
        self.twist_publisher.publish(twist_msg)

    #### Force Sensor
    def force_callback(self, msg: WrenchStamped):
        force = 0
        force += abs(msg.wrench.force.z)
        self.force = abs(abs(force) - abs(self.offset))
    def calibrate_sensor(self):
        self.get_logger().info("Starting sensor calibration")
        self.offset = 0.0
        data = []
        time.sleep(0.1)

        for _ in range(100):
            data.append(self.force)
            time.sleep(0.01)
        self.offset = sum(data) / len(data)
        time.sleep(0.1)
        self.get_logger().info(f"Calibration finished! Offset: {self.offset}")

    def run_action(
            self,
            action_client: ActionClient,
            goal_msg,
            show_progress: bool = False
        ):

        feedback_callback = lambda msg: self.get_logger().info(
            f'Progress: {msg.feedback.progress:.1f}%'
        )

        if show_progress:
            result = action_client.send_goal(
                goal_msg,
                feedback_callback=feedback_callback
            )
        else:
            result = action_client.send_goal(goal_msg)

        status = result.status

        if status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(result.result.message)
            exit(1)

def main(args=None):
    rclpy.init(args=args)
    node = MotionCameraTouch()

    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node, ),
        daemon=True
    )
    spin_thread.start()

    try:
        # Capture an untouched-gel frame to calibrate the HeightMapper baseline.
        node.get_logger().info("Capturing gelsight baseline frame for calibration...")
        node.gelsight_calibrate = True
        while node.gelsight_calibrate:
            time.sleep(0.1)
        node.get_logger().info("Gelsight baseline captured and calibrated.")

        # Send request to home robot arm
        request = Home.Request()
        request.speed = 0.1
        node.home_client.call(request)

        q = quaternion_from_euler(radians(0.0), radians(0.0), radians(0.0))

        # Create pose goal for moving camera focus
        goal_msg = PoseGoal.Goal()

        goal_msg.target_pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        )
        goal_msg.velocity_scaling = 0.1
        goal_msg.acceleration_scaling = 0.1
        goal_msg.frame_id = node.frame_id
        goal_msg.target_id = "camera_focus" # Can be any child of tool0. If empty -> tool0 used
        goal_msg.method = "PTP" # Point-to-Point

        # Execute pose goal 
        _ = node.get_logger().info("Moving Camera Focus")
        _ = node.run_action(node.pose_goal_client, goal_msg)
        _ = node.get_logger().info("Finished Moving Camera Focus")

        time.sleep(2.0)
        # Realsense picture take
        node.d555_capture_image = True

        time.sleep(2.0)

        # input("Press any key to continue")

        q = quaternion_from_euler(radians(0.0), radians(0.0), radians(0.0))
        orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

        gelsight_width = 0.020
        # gelsight_width = 0.015
        gelsight_height = 0.015
        # gelsight_height = 0.020

        i_mid = (node.touch_x // 2)
        j_mid = (node.touch_y // 2)

        print(f"i_mid: {i_mid}, j_mid: {j_mid}")

        for i in range(node.touch_x, 0, -1):
            x = (-i_mid + i) * gelsight_width 

            for j in range(node.touch_y, 0, -1):
                y = (-j_mid + j) * gelsight_height


                print(f"i: {i}, j: {j}, x: {x}, y: {y}")
                touch(
                    node=node,
                    frame_id=node.frame_id,
                    mock=node.mock,
                    position=Point(x=x, y=y, z=0.02),
                    orientation=orientation,
                )

        _ = node.get_logger().info("Stitching session captures...")
        stitch_session(
            session_dir=node.capture_dir,
            touch_x=node.touch_x,
            touch_y=node.touch_y,
        )

        # input("Press any key to continue")
        # Execute pose goal
        _ = node.get_logger().info("Moving Camera Focus")
        _ = node.run_action(node.pose_goal_client, goal_msg)
        _ = node.get_logger().info("Finished Moving Camera Focus")

        # input("Press any key to continue")
        # Send request to home robot arm
        request = Home.Request()
        request.speed = 0.1
        node.home_client.call(request)

    except KeyboardInterrupt:
        _ = node.get_logger().info("Script interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()

