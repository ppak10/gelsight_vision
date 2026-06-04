import math
import rclpy
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, TwistStamped, WrenchStamped
from moveit_msgs.srv import ServoCommandType
from rclpy.action.client import ActionClient
from rclpy.node import Node
from robot_manager_interfaces.action import PoseGoal
from robot_manager_interfaces.srv import Home
from tf_transformations import quaternion_from_euler


class TouchWithGelsight(Node):
    def __init__(self):
        super().__init__('touch_with_gelsight')

        # Assigns namespace to ur20
        _ = self.declare_parameter('ns', 'ur20')
        _ = self.declare_parameter('frame_id', 'plate')
        _ = self.declare_parameter('force_max', 4.0)
        _ = self.declare_parameter('mock', True)

        self.ns: str = str(self.get_parameter('ns').value)
        self.frame_id: str = str(self.get_parameter('frame_id').value)
        self.force_max: float = float(self.get_parameter('force_max').value)
        self.mock: bool = bool(self.get_parameter('mock').value)

        # Force values
        self.force = 0.0
        self.offset = 0.0

        self.pose_goal_client: ActionClient = ActionClient(
            self,
            PoseGoal,
            self.ns + "/" + "pose_goal"
        )
        self.pose_goal_client.wait_for_server()

        self.home_client = self.create_client(Home, self.ns + "/" + "home")

        while not self.home_client.wait_for_service():
            continue

        # Force Subscriber
        self.force_subscriber = self.create_subscription(
            WrenchStamped, 
            self.ns + '/force_torque_sensor_broadcaster/wrench_filtered',
            self.force_callback, 
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
        for _ in range(500):
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
    node = TouchWithGelsight()

    spin_thread = threading.Thread(
        target=rclpy.spin,
        args=(node, ),
        daemon=True
    )
    spin_thread.start()

    try:
        # Send request to home robot arm
        request = Home.Request()
        request.speed = 0.25
        node.home_client.call(request)

        # Create pose goal for gelsight tooltip 
        goal_msg = PoseGoal.Goal()
        q = quaternion_from_euler(
            math.radians(0.0),
            math.radians(0.0),
            math.radians(0.0)
        )
        goal_msg.target_pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.1),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        )
        goal_msg.velocity_scaling = 0.2
        goal_msg.acceleration_scaling = 0.1
        goal_msg.frame_id = node.frame_id
        goal_msg.target_id = "gelsight_tooltip"
        goal_msg.method = "PTP" # Point-to-Point

        # Execute pose goal 
        _ = node.get_logger().info("Moving Gelsight Tooltip into Position")
        _ = node.run_action(node.pose_goal_client, goal_msg)
        _ = node.get_logger().info("Finished Moving Gelsight Tooltip into Position")

        node.calibrate_sensor()

        if node.mock:
            for i in range(1000):
                node.move_z(-0.02)
                time.sleep(0.005)
        else:
            while True:
                node.move_z(-0.02)
                print(node.force)
                if node.force >= node.force_max:
                    node.stop()
                    return
                time.sleep(0.005)

    except KeyboardInterrupt:
        _ = node.get_logger().info("Script interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()

