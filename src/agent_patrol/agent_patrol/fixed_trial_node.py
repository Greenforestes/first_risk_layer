import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from std_msgs.msg import Empty
from tf_transformations import quaternion_from_euler


class FixedTrialNode(BasicNavigator):
    def __init__(self, node_name='fixed_trial_node'):
        super().__init__(node_name)

        self.declare_parameter('trial_name', 'trial')
        self.declare_parameter('initial_pose', [0.0, 0.0, 0.0])
        self.declare_parameter('goal_pose', [2.0, 0.0, 0.0])
        self.declare_parameter('start_delay_sec', 1.0)
        self.declare_parameter('post_start_goal_delay_sec', 0.1)
        self.declare_parameter('nav_timeout_sec', 40.0)
        self.declare_parameter('trial_start_topic', '/trial/start')

        self.trial_name = self.get_parameter('trial_name').value
        self.initial_pose_param = self.get_parameter('initial_pose').value
        self.goal_pose_param = self.get_parameter('goal_pose').value
        self.start_delay_sec = float(self.get_parameter('start_delay_sec').value)
        self.post_start_goal_delay_sec = float(
            self.get_parameter('post_start_goal_delay_sec').value
        )
        self.nav_timeout_sec = float(self.get_parameter('nav_timeout_sec').value)
        self.trial_start_topic = self.get_parameter('trial_start_topic').value

        self.start_pub = self.create_publisher(Empty, self.trial_start_topic, 10)

    def build_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)

        q = quaternion_from_euler(0.0, 0.0, float(yaw))
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]
        return pose

    def run_trial(self):
        self.get_logger().info(f'=== Fixed Trial Start: {self.trial_name} ===')

        init_pose = self.build_pose(*self.initial_pose_param)
        goal_pose = self.build_pose(*self.goal_pose_param)

        self.get_logger().info(
            f'Initial pose: x={self.initial_pose_param[0]:.2f}, '
            f'y={self.initial_pose_param[1]:.2f}, yaw={self.initial_pose_param[2]:.2f}'
        )
        self.get_logger().info(
            f'Goal pose: x={self.goal_pose_param[0]:.2f}, '
            f'y={self.goal_pose_param[1]:.2f}, yaw={self.goal_pose_param[2]:.2f}'
        )

        self.setInitialPose(init_pose)
        self.waitUntilNav2Active()

        self.get_logger().info(
            f'Nav2 active. Settling for {self.start_delay_sec:.1f}s before synchronized start...'
        )
        time.sleep(self.start_delay_sec)

        self.get_logger().info(f'Publishing synchronized start on {self.trial_start_topic}')
        self.start_pub.publish(Empty())

        if self.post_start_goal_delay_sec > 0.0:
            time.sleep(self.post_start_goal_delay_sec)

        self.get_logger().info('Sending fixed goal...')
        self.goToPose(goal_pose)

        start_time = time.time()
        while not self.isTaskComplete():
            if time.time() - start_time > self.nav_timeout_sec:
                self.get_logger().warn('Navigation timeout reached, canceling task.')
                self.cancelTask()
                break

            feedback = self.getFeedback()
            if feedback is not None:
                self.get_logger().info('Task running...')

        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Trial result: SUCCEEDED')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Trial result: CANCELED')
        elif result == TaskResult.FAILED:
            self.get_logger().error('Trial result: FAILED')
        else:
            self.get_logger().error('Trial result: UNKNOWN')

        self.get_logger().info(f'=== Fixed Trial End: {self.trial_name} ===')


def main():
    rclpy.init()
    node = FixedTrialNode()
    node.run_trial()
    rclpy.shutdown()


if __name__ == '__main__':
    main()