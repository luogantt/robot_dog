#!/usr/bin/env python3
"""Record a map-frame route after the final map has been saved and reloaded."""

import math
from pathlib import Path
import time

import rclpy
from nav_msgs.msg import Odometry, Path as PathMessage
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_srvs.srv import SetBool, Trigger
import yaml

from .common import quaternion_to_yaw, yaw_to_quaternion


class RouteRecorder(Node):
    def __init__(self) -> None:
        super().__init__('s10_route_recorder')
        self.declare_parameter('odom_topic', '/lightning/odom')
        self.declare_parameter('output_file', '/tmp/s10_route.yaml')
        self.declare_parameter('route_name', 'final_route_v01')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('sample_distance_m', 0.15)
        self.declare_parameter('sample_period_sec', 0.50)
        self.declare_parameter('checkpoint_tolerance_m', 0.35)

        self.output_file = str(self.get_parameter('output_file').value)
        self.route_name = str(self.get_parameter('route_name').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.sample_distance = float(self.get_parameter('sample_distance_m').value)
        self.sample_period = float(self.get_parameter('sample_period_sec').value)
        self.checkpoint_tolerance = float(
            self.get_parameter('checkpoint_tolerance_m').value)

        self.recording = False
        self.current_pose = None
        self.path = []
        self.checkpoints = []
        self.last_sample_monotonic = 0.0

        odom_topic = str(self.get_parameter('odom_topic').value)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 10)
        path_qos = QoSProfile(depth=1)
        path_qos.reliability = ReliabilityPolicy.RELIABLE
        path_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.path_pub = self.create_publisher(PathMessage, '/s10_nav/recorded_path', path_qos)
        self.create_service(SetBool, '/s10_route_recorder/set_recording', self.set_recording)
        self.create_service(Trigger, '/s10_route_recorder/mark_checkpoint', self.mark_checkpoint)
        self.create_service(Trigger, '/s10_route_recorder/save', self.save)
        self.create_service(Trigger, '/s10_route_recorder/clear', self.clear)
        self.get_logger().info(f'route recorder output: {self.output_file}')

    def on_odom(self, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        self.current_pose = [
            float(position.x),
            float(position.y),
            quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w),
        ]
        if not self.recording:
            return
        now = time.monotonic()
        distance = math.inf if not self.path else math.hypot(
            self.current_pose[0] - self.path[-1][0],
            self.current_pose[1] - self.path[-1][1],
        )
        yaw_change = math.inf if not self.path else abs(math.atan2(
            math.sin(self.current_pose[2] - self.path[-1][2]),
            math.cos(self.current_pose[2] - self.path[-1][2]),
        ))
        periodic_motion_sample = (
            now - self.last_sample_monotonic >= self.sample_period and
            (distance >= 0.02 or yaw_change >= math.radians(5.0))
        )
        if distance >= self.sample_distance or periodic_motion_sample:
            self.path.append(list(self.current_pose))
            self.last_sample_monotonic = now
            self.publish_path()

    def set_recording(self, request: SetBool.Request,
                      response: SetBool.Response) -> SetBool.Response:
        if request.data and self.current_pose is None:
            response.success = False
            response.message = 'no odometry received'
            return response
        self.recording = bool(request.data)
        if self.recording and not self.path:
            self.path.append(list(self.current_pose))
            self.last_sample_monotonic = time.monotonic()
        response.success = True
        response.message = 'recording' if self.recording else 'paused'
        self.get_logger().info(response.message)
        return response

    def mark_checkpoint(self, _request: Trigger.Request,
                        response: Trigger.Response) -> Trigger.Response:
        if self.current_pose is None:
            response.success = False
            response.message = 'no odometry received'
            return response
        checkpoint_id = f'P{len(self.checkpoints) + 1:02d}'
        self.checkpoints.append({
            'id': checkpoint_id,
            'x': round(self.current_pose[0], 6),
            'y': round(self.current_pose[1], 6),
            'yaw': round(self.current_pose[2], 6),
            'position_tolerance': self.checkpoint_tolerance,
            'yaw_tolerance_deg': 180.0,
            'required': True,
        })
        response.success = True
        response.message = f'marked {checkpoint_id}'
        self.get_logger().info(response.message)
        return response

    def save(self, _request: Trigger.Request,
             response: Trigger.Response) -> Trigger.Response:
        if len(self.path) < 2:
            response.success = False
            response.message = 'need at least two route points'
            return response
        output = Path(self.output_file).expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        document = {
            'frame_id': self.frame_id,
            'route_name': self.route_name,
            'loop': False,
            'warn_corridor_error': 0.40,
            'max_corridor_error': 0.60,
            'controller': {
                'lookahead': 0.90,
                'nominal_speed': 0.55,
                'max_speed': 0.80,
                'max_yaw_rate': 0.45,
                'goal_slow_radius': 1.20,
                'curvature_speed_gain': 1.50,
                'heading_gain': 1.80,
                'heading_slow_threshold_deg': 30.0,
                'yaw_only_threshold_deg': 60.0,
            },
            'checkpoints': self.checkpoints,
            'path': [[round(value, 6) for value in point] for point in self.path],
        }
        with output.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(document, stream, allow_unicode=True, sort_keys=False)
        response.success = True
        response.message = f'saved {len(self.path)} points to {output}'
        self.get_logger().info(response.message)
        return response

    def clear(self, _request: Trigger.Request,
              response: Trigger.Response) -> Trigger.Response:
        self.recording = False
        self.path.clear()
        self.checkpoints.clear()
        self.publish_path()
        response.success = True
        response.message = 'route and checkpoints cleared'
        self.get_logger().warning(response.message)
        return response

    def publish_path(self) -> None:
        message = PathMessage()
        message.header.frame_id = self.frame_id
        message.header.stamp = self.get_clock().now().to_msg()
        for x, y, yaw in self.path:
            pose = PoseStamped()
            pose.header = message.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            qx, qy, qz, qw = yaw_to_quaternion(yaw)
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            message.poses.append(pose)
        self.path_pub.publish(message)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RouteRecorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
