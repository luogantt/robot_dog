#!/usr/bin/env python3
"""Map-frame route following with checkpoints and corridor protection."""

import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float32, Int32, String

from .common import (
    checkpoint_stations,
    clamp,
    load_route,
    lookahead_path_index,
    nearest_path_index,
    pure_pursuit_target,
    quaternion_to_yaw,
    wrap_angle,
    yaw_to_quaternion,
)


class RouteFollower(Node):
    def __init__(self) -> None:
        super().__init__('s10_route_follower')
        self.declare_parameter('route_file', '')
        self.declare_parameter('odom_topic', '/lightning/odom')
        self.declare_parameter('command_topic', '/s10_nav/cmd_tracking')
        self.declare_parameter('control_rate_hz', 20.0)
        self.declare_parameter('odom_timeout_sec', 0.30)
        self.declare_parameter('search_back_points', 10)
        self.declare_parameter('search_forward_points', 120)
        self.declare_parameter('missed_checkpoint_margin_m', 0.80)
        self.declare_parameter('goal_tolerance_m', 0.25)

        route_file = str(self.get_parameter('route_file').value)
        if not route_file:
            raise RuntimeError('route_file parameter is required')
        self.route = load_route(route_file)
        self.checkpoint_stations = checkpoint_stations(self.route)

        controller = self.route.controller
        self.lookahead = float(controller.get('lookahead', 0.90))
        self.nominal_speed = float(controller.get('nominal_speed', 0.55))
        self.max_speed = float(controller.get('max_speed', 0.80))
        self.max_yaw_rate = float(controller.get('max_yaw_rate', 0.45))
        self.goal_slow_radius = float(controller.get('goal_slow_radius', 1.20))
        self.curvature_speed_gain = float(controller.get('curvature_speed_gain', 1.5))
        self.heading_gain = float(controller.get('heading_gain', 1.8))
        self.yaw_only_threshold = math.radians(
            float(controller.get('yaw_only_threshold_deg', 60.0)))
        self.heading_slow_threshold = math.radians(
            float(controller.get('heading_slow_threshold_deg', 30.0)))

        self.odom_timeout = float(self.get_parameter('odom_timeout_sec').value)
        self.search_back = int(self.get_parameter('search_back_points').value)
        self.search_forward = int(self.get_parameter('search_forward_points').value)
        self.missed_margin = float(self.get_parameter('missed_checkpoint_margin_m').value)
        self.goal_tolerance = float(self.get_parameter('goal_tolerance_m').value)

        self.pose = None
        self.last_odom_monotonic = None
        self.progress_index = 0
        self.checkpoint_index = 0
        self.route_fault = False
        self.finished = False

        odom_topic = str(self.get_parameter('odom_topic').value)
        command_topic = str(self.get_parameter('command_topic').value)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        self.command_pub = self.create_publisher(TwistStamped, command_topic, 10)
        latched_qos = QoSProfile(depth=1)
        latched_qos.reliability = ReliabilityPolicy.RELIABLE
        latched_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(Path, '/s10_nav/route_path', latched_qos)
        self.checkpoint_pub = self.create_publisher(Int32, '/s10_nav/checkpoint_index', 10)
        self.event_pub = self.create_publisher(String, '/s10_nav/checkpoint_event', 10)
        self.finished_pub = self.create_publisher(Bool, '/s10_nav/finished', latched_qos)
        self.fault_pub = self.create_publisher(Bool, '/s10_nav/route_fault', latched_qos)
        self.corridor_pub = self.create_publisher(Float32, '/s10_nav/corridor_error', 10)

        rate = max(1.0, float(self.get_parameter('control_rate_hz').value))
        self.timer = self.create_timer(1.0 / rate, self.control)
        self.publish_route_path()
        self.publish_status()
        self.get_logger().info(
            f'loaded route {self.route.name}: {len(self.route.path)} points, '
            f'{len(self.route.checkpoints)} checkpoints, length '
            f'{self.route.cumulative_distance[-1]:.1f} m')

    def on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self.pose = (
            float(position.x),
            float(position.y),
            quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w),
        )
        self.last_odom_monotonic = time.monotonic()

    def publish_route_path(self) -> None:
        message = Path()
        message.header.frame_id = self.route.frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        for point in self.route.path:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            qx, qy, qz, qw = yaw_to_quaternion(point.yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            message.poses.append(pose)
        self.path_pub.publish(message)

    def publish_status(self) -> None:
        current = -1 if self.checkpoint_index >= len(self.route.checkpoints) \
            else self.checkpoint_index
        self.checkpoint_pub.publish(Int32(data=current))
        self.finished_pub.publish(Bool(data=self.finished))
        self.fault_pub.publish(Bool(data=self.route_fault))

    def publish_zero(self) -> None:
        message = TwistStamped()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'base_link'
        self.command_pub.publish(message)

    def update_checkpoints(self, x: float, y: float, yaw: float, station: float) -> None:
        while self.checkpoint_index < len(self.route.checkpoints):
            checkpoint = self.route.checkpoints[self.checkpoint_index]
            distance = math.hypot(checkpoint.x - x, checkpoint.y - y)
            yaw_error = abs(math.degrees(wrap_angle(checkpoint.yaw - yaw)))
            if distance <= checkpoint.position_tolerance and \
                    yaw_error <= checkpoint.yaw_tolerance_deg:
                event = (
                    f'REACHED,{checkpoint.checkpoint_id},{self.checkpoint_index},'
                    f'distance={distance:.3f},yaw_error_deg={yaw_error:.1f}')
                self.event_pub.publish(String(data=event))
                self.get_logger().info(event)
                self.checkpoint_index += 1
                continue

            checkpoint_station = self.checkpoint_stations[self.checkpoint_index]
            if checkpoint.required and station > checkpoint_station + self.missed_margin:
                self.route_fault = True
                event = (
                    f'MISSED,{checkpoint.checkpoint_id},{self.checkpoint_index},'
                    f'distance={distance:.3f}')
                self.event_pub.publish(String(data=event))
                self.get_logger().error(event)
            break

    def control(self) -> None:
        if self.route_fault or self.finished:
            self.publish_zero()
            self.publish_status()
            return
        if self.pose is None or self.last_odom_monotonic is None or \
                time.monotonic() - self.last_odom_monotonic > self.odom_timeout:
            self.publish_zero()
            return

        x, y, yaw = self.pose
        search_start = max(0, self.progress_index - self.search_back)
        search_end = min(len(self.route.path), self.progress_index + self.search_forward)
        nearest, corridor_error = nearest_path_index(
            self.route.path, x, y, search_start, search_end)
        self.progress_index = max(self.progress_index, nearest)
        station = self.route.cumulative_distance[self.progress_index]
        self.corridor_pub.publish(Float32(data=float(corridor_error)))

        self.update_checkpoints(x, y, yaw, station)
        if corridor_error > self.route.max_corridor_error:
            self.route_fault = True
            self.event_pub.publish(String(
                data=f'CORRIDOR_FAULT,error={corridor_error:.3f}'))
            self.get_logger().error(
                f'route corridor exceeded: {corridor_error:.2f} m > '
                f'{self.route.max_corridor_error:.2f} m')
            self.publish_zero()
            self.publish_status()
            return

        remaining = self.route.cumulative_distance[-1] - station
        goal = self.route.path[-1]
        goal_distance = math.hypot(goal.x - x, goal.y - y)
        all_required_reached = all(
            not checkpoint.required or index < self.checkpoint_index
            for index, checkpoint in enumerate(self.route.checkpoints))
        if remaining <= self.goal_tolerance and goal_distance <= self.goal_tolerance \
                and all_required_reached:
            self.finished = True
            self.event_pub.publish(String(data='FINISHED'))
            self.get_logger().info('route finished')
            self.publish_zero()
            self.publish_status()
            return

        target_index = lookahead_path_index(
            self.route.cumulative_distance, self.progress_index, self.lookahead)
        target = self.route.path[target_index]
        _, _, curvature, heading_error = pure_pursuit_target(x, y, yaw, target)

        speed = min(self.nominal_speed, self.max_speed)
        speed /= 1.0 + self.curvature_speed_gain * abs(curvature)
        if remaining < self.goal_slow_radius:
            speed *= clamp(remaining / max(self.goal_slow_radius, 1e-3), 0.20, 1.0)
        if corridor_error > self.route.warn_corridor_error:
            interval = self.route.max_corridor_error - self.route.warn_corridor_error
            speed *= clamp((self.route.max_corridor_error - corridor_error) /
                           max(interval, 1e-3), 0.20, 1.0)

        if abs(heading_error) >= self.yaw_only_threshold:
            speed = 0.0
        elif abs(heading_error) > self.heading_slow_threshold:
            speed *= max(0.10, math.cos(heading_error)) ** 2

        if speed == 0.0:
            yaw_rate = self.heading_gain * heading_error
        else:
            yaw_rate = speed * curvature + 0.35 * self.heading_gain * heading_error
        yaw_rate = clamp(yaw_rate, -self.max_yaw_rate, self.max_yaw_rate)

        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        command.twist.linear.x = float(clamp(speed, 0.0, self.max_speed))
        command.twist.angular.z = float(yaw_rate)
        self.command_pub.publish(command)
        self.publish_status()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
