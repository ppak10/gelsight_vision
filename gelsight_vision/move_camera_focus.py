import math
import rclpy
import threading

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion
from rclpy.action.client import ActionClient
from rclpy.node import Node
from robot_manager_interfaces.action import PoseGoal
from robot_manager_interfaces.srv import Home
from tf_transformations import quaternion_from_euler

class MoveCameraFocus(Node):
    def __init__(self):
        super().__init__('move_camera_focus')

        # Assigns namespace to ur20
        _ = self.declare_parameter('ns', 'ur20')
        _ = self.declare_parameter('frame_id', 'plate')
        self.ns: str = str(self.get_parameter('ns').value)
        self.frame_id: str = str(self.get_parameter('frame_id').value)

        self.pose_goal_client: ActionClient = ActionClient(
            self,
            PoseGoal,
            self.ns + "/" + "pose_goal"
        )
        self.pose_goal_client.wait_for_server()

        self.home_client = self.create_client(Home, self.ns + "/" + "home")

        while not self.home_client.wait_for_service():
            continue

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
    node = MoveCameraFocus()

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

        # Create pose goal for moving camera focus
        goal_msg = PoseGoal.Goal()
        q = quaternion_from_euler(
            math.radians(0.0),
            math.radians(0.0),
            math.radians(0.0)
        )
        goal_msg.target_pose = Pose(
            position=Point(x=0.0, y=0.0, z=0.0),
            orientation=Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])
        )
        goal_msg.velocity_scaling = 0.2
        goal_msg.acceleration_scaling = 0.1
        goal_msg.frame_id = node.frame_id
        goal_msg.target_id = "camera_focus" # Can be any child of tool0. If empty -> tool0 used
        goal_msg.method = "PTP" # Point-to-Point

        # Execute pose goal 
        _ = node.get_logger().info("Moving Camera Focus")
        _ = node.run_action(node.pose_goal_client, goal_msg)
        _ = node.get_logger().info("Finished Moving Camera Focus")

    except KeyboardInterrupt:
        _ = node.get_logger().info("Script interrupted by user.")
    finally:
        node.destroy_node()
        rclpy.shutdown()
        spin_thread.join()

if __name__ == '__main__':
    main()

