import rclpy
from rclpy.node import Node
from typing import List
from std_msgs.msg import String
# from actions_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Pose, PoseArray
from rclpy.action import ActionClient
# from rclp.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.action import NavigateThroughPoses
# from nav2_msgs.action._navigate_through_poses import (
# NavigateThroughPoses_FeedbackMessage,
# NavigateThroughPoses_GetResult_Response,
# NavigateThroughPoses_SendGoal_Response)


def createPoseStamped(x, y, z, timestamp, frame="map") -> PoseStamped:
    pose: PoseStamped = PoseStamped()
    pose.header.frame_id = frame
    pose.header.stamp = timestamp
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    pose.pose.orientation.z = 1.0
    pose.pose.orientation.w = 0.0
    return pose


class ESDACommander(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.nav_through_poses_client = ActionClient(
            self, NavigateThroughPoses, "navigate_through_poses")
        self.pose_sub = self.create_subscription(
            PoseStamped,
            "odometry/local",
            self._pose_callback, 1)
        # Waypoints may require custom message type to define acceptance criteria
        self.waypoints_sub = self.create_subscription(
            PoseArray,
            "navigation/waypoints",
            self._waypoints_callback,
            1
        )
        self.navigator = BasicNavigator()
        # Await Nav2Setup before completing construction
        self.navigator.lifecycleStartup()
        # Try and retrigger planning every 2 seconds
        self.timer = self.create_timer(5, self.timer_callback)

        self.pose: PoseStamped = None

        self.waypoints: List[Pose] = []
        # TODO: Add a waypoints subscriptions instead of hardcode

        # For now
        self.waypoints = [createPoseStamped(-2, -1, 0, self.get_clock().now().to_msg()), createPoseStamped(-2, 1, 0, self.get_clock().now().to_msg())]

    def _pose_callback(self, msg: PoseStamped):
        self.pose = msg

    def _waypoints_callback(self, msg: PoseArray):
        self.waypoints = msg.poses
        self.navigator.waitUntilNav2Active()
        if not self.navigator.isTaskComplete():
            self.navigator.cancelTask()
            self.get_logger().info("Cancelled task for more recent waypoints")
        self.navigator.goThroughPoses(self.waypoints)
        self.get_logger().info(
            f"Beginning navigate through {len(self.waypoints)} starting from {self.pose.pose.position}")

    def timer_callback(self):
        if len(self.waypoints):
            self.get_logger().info("Not ready, missing waypoints")
        if not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            print(f"In progress{feedback}")
            return

        self.navigator.goThroughPoses(self.waypoints)
        self.get_logger().info(
            f"Beginning navigate through {len(self.waypoints)} starting from {self.pose.pose.position}")


def main(args=None):
    rclpy.init(args=args)

    commander = ESDACommander()

    rclpy.spin(commander)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    commander.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
