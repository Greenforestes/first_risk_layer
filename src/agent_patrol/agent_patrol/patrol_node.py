import rclpy
from geometry_msgs.msg import PoseStamped, Pose
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion, quaternion_from_euler
from rclpy.duration import Duration


class PatrolNode(BasicNavigator):
    def __init__(self,node = 'patrol_node'):
        super().__init__(node)
        #导航相关定义
        self.declare_parameter('initial_points', [0.0, 0.0, 0.0])
        self.declare_parameter('target_points', [1.0, 0.0, 0.0, 1.0, 1.0, 1.57])
        self.initial_points_ = self.get_parameter('initial_points').value
        self.target_points_ = self.get_parameter('target_points').value
        # 实时位置获取 TF 相关定义
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def get_pose_by_xyyaw(self, x, y, yaw):
        '''根据给定的x、y和yaw值创建一个PoseStamped消息'''
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        quat = quaternion_from_euler(0.0, 0.0, yaw)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        return pose
    

    def init_robot_pose(self):
        """
        初始化机器人位姿
        """
        # 从参数获取初始化点
        self.initial_points_ = self.get_parameter('initial_points').value
        # 合成位姿并进行初始化
        self.setInitialPose(self.get_pose_by_xyyaw(
            self.initial_points_[0], self.initial_points_[1], self.initial_points_[2]))
        # 等待直到导航激活
        self.waitUntilNav2Active()

    def get_target_point(self):
        """
        通过参数值获取目标点集合
        """
        # 从参数获取目标点列表
        points = []
        self.target_points_ = self.get_parameter('target_points').value
        for index in range(int(len(self.target_points_)/3)):
            x = self.target_points_[index*3]
            y = self.target_points_[index*3 + 1]
            yaw = self.target_points_[index*3 + 2]
            points.append([x, y, yaw])
            self.get_logger().info(f"Get target point: {index}: x={x}, y={y}, yaw={yaw}")    
        return points   
    

    def nav_to_pose(self, target_point):
        """
        导航到指定位姿
        """
        self.waitUntilNav2Active()
        self.result = self.goToPose(target_point)
        while not self.isTaskComplete():
            feedback = self.getFeedback()
            if feedback:
                self.get_logger().info(f'预计: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9} s 后到达')
        # 最终结果判断
        result = self.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('导航结果：成功')
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('导航结果：被取消')
        elif result == TaskResult.FAILED:
            self.get_logger().error('导航结果：失败')
        else:
            self.get_logger().error('导航结果：返回状态无效')

    def get_current_pose(self):
        """
        获取机器人当前位姿
        """
        while rclpy.ok():
            try:
                tf = self.tf_buffer.lookup_transform(
                    'map', 'base_footprint', rclpy.time.Time(seconds=0), rclpy.time.Duration(seconds=1))
                transform = tf.transform
                rotation_euler = euler_from_quaternion([
                    transform.rotation.x,
                    transform.rotation.y,
                    transform.rotation.z,
                    transform.rotation.w
                ])
                self.get_logger().info(
                    f'平移:{transform.translation},旋转四元数:{transform.rotation}:旋转欧拉角:{rotation_euler}')
                return transform
            except Exception as e:
                self.get_logger().warn(f'不能够获取坐标变换，原因: {str(e)}')



def main():
    rclpy.init()
    patrol = PatrolNode()
    patrol.init_robot_pose()

    while rclpy.ok():
        for point in patrol.get_target_point():
            x, y, yaw = point[0], point[1], point[2]
            # 导航到目标点
            target_pose = patrol.get_pose_by_xyyaw(x, y, yaw)
            patrol.nav_to_pose(target_pose)
    rclpy.shutdown()    

if __name__ == '__main__':
    main()















