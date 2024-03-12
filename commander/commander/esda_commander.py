import rclpy
from rclpy.node import Node
from typing import List

from std_msgs.msg import String
# from actions_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Pose
from rclpy.action import ActionClient
# from rclp.duration import Duration
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.action import NavigateThroughPoses
from nav2_msgs.action._navigate_through_poses import (
        NavigateThroughPoses_FeedbackMessage, 
        NavigateThroughPoses_GetResult_Response,
        NavigateThroughPoses_SendGoal_Response)


class ESDACommander(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, "navigate_through_poses")
        self.pose_sub = self.create_subscription(
                                                 PoseStamped,
                                                 "pose_topic",
                                                 self._pose_callback)
        self.navigator = BasicNavigator()
        self.navigator.lifecycleStartup()

        self.pose: PoseStamped = None
        self.waypoints: List[Pose] = []
        # TODO: Add a waypoints subscription

    def _pose_callback(self, msg: PoseStamped):
        self.pose = msg

    def timer_callback(self):
        if not self.pose:
            self.get_logger().info(f"Not ready, missing initial pose")
            return
        if not self.waypoints:
            self.get_logger().info(f"Not ready, missing waypoints")
        if not self.navigator.isTaskComplete():
            return
        self.navigator.setInitialPose(self.pose)
        self.navigator.goThroughPoses(self.waypoints)
        self.get_logger().info(f"Beginning navigate through {len(self.waypoints)} starting from {self.pose.pose}")

        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            print(f"In progress {feedback}")
            result = self.navigator.getResult()

            match result:
                case TaskResult.SUCCEEDED:
                    self.get_logger().info("Goal succeeded")
                case TaskResult.CANCELED:
                    self.get_logger().info("Goal was cancelled")
                case TaskResult.FAILED:
                    self.get_logger().warn("Goal failed")
                case _:
                    self.get_logger().error(f"Unknown task result, {result}")
        
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


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
