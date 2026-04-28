import math
import os
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Point
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Empty


def yaw_to_quaternion(yaw: float):
    half = yaw * 0.5
    qx = 0.0
    qy = 0.0
    qz = math.sin(half)
    qw = math.cos(half)
    return qx, qy, qz, qw


class DynamicActorManager(Node):
    def __init__(self):
        super().__init__('dynamic_actor_manager')

        self.declare_parameter('scenario_file', '')
        self.scenario_file = self.get_parameter('scenario_file').get_parameter_value().string_value

        self.get_logger().info('=== Dynamic Actor Manager Started ===')
        self.get_logger().info(f'scenario_file: {self.scenario_file}')

        if not self.scenario_file:
            raise RuntimeError('scenario_file parameter is empty')

        if not os.path.exists(self.scenario_file):
            raise RuntimeError(f'scenario file does not exist: {self.scenario_file}')

        with open(self.scenario_file, 'r', encoding='utf-8') as f:
            self.scenario = yaml.safe_load(f)

        # 当前先只支持一个障碍物
        self.actor = self.scenario['actors'][0]
        self.entity_name = self.actor['entity_name']
        self.speed = float(self.actor.get('speed', 0.1))
        self.loop = bool(self.actor.get('loop', True))
        self.z_value = float(self.actor.get('z', 0.0))
        self.actor_radius = float(self.actor.get('radius', 0.10))
        self.actor_height = float(self.actor.get('height', 0.8))

        p0 = self.actor['waypoints'][0]
        p1 = self.actor['waypoints'][1]
        self.ax = float(p0[0])
        self.ay = float(p0[1])
        self.bx = float(p1[0])
        self.by = float(p1[1])

        self.dx = self.bx - self.ax
        self.dy = self.by - self.ay
        self.segment_length = math.hypot(self.dx, self.dy)

        if self.segment_length < 1e-6:
            raise RuntimeError('waypoints are too close or identical')

        # future 配置
        future_cfg = self.scenario.get('future', {})
        self.future_frame_id = str(future_cfg.get('frame_id', 'map'))
        self.future_horizon_sec = float(future_cfg.get('horizon_sec', 4.8))
        self.future_dt = float(future_cfg.get('dt', 0.2))
        self.future_marker_z = float(future_cfg.get('marker_z', 0.05))

        # 当前沿线段的“路程”
        self.progress = 0.0
        self.direction = 1.0  # +1: A->B, -1: B->A

        # Gazebo service
        self.client = None
        candidate_services = ['/gazebo/set_entity_state', '/set_entity_state']
        for srv_name in candidate_services:
            client = self.create_client(SetEntityState, srv_name)
            self.get_logger().info(f'trying service: {srv_name}')
            if client.wait_for_service(timeout_sec=2.0):
                self.client = client
                self.service_name = srv_name
                self.get_logger().info(f'connected to service: {srv_name}')
                break

        if self.client is None:
            raise RuntimeError('cannot find Gazebo set_entity_state service')

        # QoS：让 RViz 后加入显示时也更稳
        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        normal_qos = QoSProfile(depth=10)
        normal_qos.reliability = ReliabilityPolicy.RELIABLE

        # Publishers
        self.current_pose_pub = self.create_publisher(
            PoseStamped, '/dynamic_agents/current_pose', latched_qos
        )
        self.current_marker_pub = self.create_publisher(
            Marker, '/dynamic_agents/current_marker', latched_qos
        )
        self.future_path_pub = self.create_publisher(
            Path, '/dynamic_agents/future_path', latched_qos
        )
        self.future_markers_pub = self.create_publisher(
            MarkerArray, '/dynamic_agents/future_markers', latched_qos
        )

        # Timer
        self.timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.last_time = self.get_clock().now()
        self.pending_future = None
        self.tick_count = 0

        self.get_logger().info(
            f'controlling entity [{self.entity_name}] from '
            f'({self.ax:.2f}, {self.ay:.2f}) to ({self.bx:.2f}, {self.by:.2f}), '
            f'speed={self.speed:.2f} m/s'
        )
        self.get_logger().info(
            f'future config: frame={self.future_frame_id}, '
            f'horizon={self.future_horizon_sec:.2f}s, dt={self.future_dt:.2f}s'
        )
        
        self.get_logger().info('=== Stage 2.2 Interface Freeze ===')
        self.get_logger().info('risk-layer input topic: /dynamic_agents/future_path (nav_msgs/Path)')
        self.get_logger().info('debug topics:')
        self.get_logger().info('  /dynamic_agents/current_pose')
        self.get_logger().info('  /dynamic_agents/current_marker')
        self.get_logger().info('  /dynamic_agents/future_markers')

    def advance_progress(self, progress: float, direction: float, delta_s: float):
        """沿线段推进 delta_s，并处理端点反弹。"""
        progress += direction * delta_s

        while True:
            if progress > self.segment_length:
                overflow = progress - self.segment_length
                progress = self.segment_length - overflow
                direction = -1.0
            elif progress < 0.0:
                overflow = -progress
                progress = overflow
                direction = 1.0
            else:
                break

        return progress, direction

    def progress_to_pose(self, progress: float, direction: float):
        """根据 progress / direction 还原出 x, y, yaw。"""
        ratio = progress / self.segment_length
        x = self.ax + ratio * self.dx
        y = self.ay + ratio * self.dy

        if direction > 0:
            yaw = math.atan2(self.dy, self.dx)
        else:
            yaw = math.atan2(-self.dy, -self.dx)

        return x, y, yaw

    def build_future_samples(self):
        """生成未来 0~horizon 的路径点。"""
        samples = []

        sim_progress = self.progress
        sim_direction = self.direction

        total_steps = max(1, int(self.future_horizon_sec / self.future_dt))

        for step in range(total_steps + 1):
            t = step * self.future_dt
            x, y, yaw = self.progress_to_pose(sim_progress, sim_direction)
            samples.append((t, x, y, yaw))

            if step < total_steps:
                sim_progress, sim_direction = self.advance_progress(
                    sim_progress, sim_direction, self.speed * self.future_dt
                )

        return samples

    def publish_current_pose_and_marker(self, x: float, y: float, yaw: float):
        now_msg = self.get_clock().now().to_msg()
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        # 1) 当前位姿 PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = self.future_frame_id
        pose_msg.header.stamp = now_msg
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = self.future_marker_z
        pose_msg.pose.orientation.x = qx
        pose_msg.pose.orientation.y = qy
        pose_msg.pose.orientation.z = qz
        pose_msg.pose.orientation.w = qw
        self.current_pose_pub.publish(pose_msg)

        # 2) 当前障碍物 Marker（圆柱）
        marker = Marker()
        marker.header.frame_id = self.future_frame_id
        marker.header.stamp = now_msg
        marker.ns = 'current_actor'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = self.actor_height * 0.5
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = self.actor_radius * 1.1
        marker.scale.y = self.actor_radius * 1.1
        marker.scale.z = self.actor_height

        marker.color.a = 0.85
        marker.color.r = 1.0
        marker.color.g = 0.15
        marker.color.b = 0.15

        self.current_marker_pub.publish(marker)

    def publish_future(self, samples):
        now_msg = self.get_clock().now().to_msg()

        # 1) Path
        path_msg = Path()
        path_msg.header.frame_id = self.future_frame_id
        path_msg.header.stamp = now_msg

        for _, x, y, yaw in samples:
            pose_msg = PoseStamped()
            pose_msg.header.frame_id = self.future_frame_id
            pose_msg.header.stamp = now_msg

            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            pose_msg.pose.position.x = x
            pose_msg.pose.position.y = y
            pose_msg.pose.position.z = self.future_marker_z
            pose_msg.pose.orientation.x = qx
            pose_msg.pose.orientation.y = qy
            pose_msg.pose.orientation.z = qz
            pose_msg.pose.orientation.w = qw

            path_msg.poses.append(pose_msg)

        self.future_path_pub.publish(path_msg)

        # 2) MarkerArray
        marker_array = MarkerArray()

        # future line
        line_marker = Marker()
        line_marker.header.frame_id = self.future_frame_id
        line_marker.header.stamp = now_msg
        line_marker.ns = 'future_line'
        line_marker.id = 0
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.4
        line_marker.color.a = 1.0
        line_marker.color.r = 0.1
        line_marker.color.g = 1.0
        line_marker.color.b = 0.1
        line_marker.pose.orientation.w = 1.0

        for _, x, y, _ in samples:
            p = Point()
            p.x = x
            p.y = y
            p.z = self.future_marker_z
            line_marker.points.append(p)

        marker_array.markers.append(line_marker)

        # future points
        sphere_marker = Marker()
        sphere_marker.header.frame_id = self.future_frame_id
        sphere_marker.header.stamp = now_msg
        sphere_marker.ns = 'future_points'
        sphere_marker.id = 1
        sphere_marker.type = Marker.SPHERE_LIST
        sphere_marker.action = Marker.ADD
        sphere_marker.scale.x = 0.08
        sphere_marker.scale.y = 0.08
        sphere_marker.scale.z = 0.08
        sphere_marker.color.a = 1.0
        sphere_marker.color.r = 1.0
        sphere_marker.color.g = 0.85
        sphere_marker.color.b = 0.15
        sphere_marker.pose.orientation.w = 1.0

        for _, x, y, _ in samples:
            p = Point()
            p.x = x
            p.y = y
            p.z = self.future_marker_z
            sphere_marker.points.append(p)

        marker_array.markers.append(sphere_marker)

        self.future_markers_pub.publish(marker_array)

    def timer_callback(self):
        # 防止 service 堆积
        if self.pending_future is not None and not self.pending_future.done():
            return

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        if dt <= 0.0:
            return

        # 推进当前实体
        self.progress, self.direction = self.advance_progress(
            self.progress, self.direction, self.speed * dt
        )


        x, y, yaw = self.progress_to_pose(self.progress, self.direction)
        qx, qy, qz, qw = yaw_to_quaternion(yaw)

        # 写入 Gazebo
        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = self.entity_name
        req.state.reference_frame = 'world'

        req.state.pose.position.x = x
        req.state.pose.position.y = y
        req.state.pose.position.z = self.z_value

        req.state.pose.orientation.x = qx
        req.state.pose.orientation.y = qy
        req.state.pose.orientation.z = qz
        req.state.pose.orientation.w = qw

        req.state.twist.linear.x = 0.0
        req.state.twist.linear.y = 0.0
        req.state.twist.linear.z = 0.0
        req.state.twist.angular.x = 0.0
        req.state.twist.angular.y = 0.0
        req.state.twist.angular.z = 0.0

        self.pending_future = self.client.call_async(req)

        # 发布当前状态
        self.publish_current_pose_and_marker(x, y, yaw)

        # 发布 future
        samples = self.build_future_samples()
        self.publish_future(samples)

        self.tick_count += 1
        if self.tick_count % 20 == 0:
            self.get_logger().info(
                f'[{self.entity_name}] current=({x:.2f}, {y:.2f}), '
                f'progress={self.progress:.2f}/{self.segment_length:.2f}, dir={self.direction:+.0f}, '
                f'future_points={len(samples)}'
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DynamicActorManager()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info('dynamic_actor_manager interrupted by user')
    except Exception as e:
        print(f'[dynamic_actor_manager] fatal error: {e}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()