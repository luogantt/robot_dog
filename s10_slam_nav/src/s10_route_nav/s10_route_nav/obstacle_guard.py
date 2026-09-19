#!/usr/bin/env python3
"""Low-complexity dual-lidar obstacle guard for the fixed-route controller."""

from dataclasses import dataclass
import math
import time
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32, UInt8


CLEAR = 0
SLOW = 1
AVOID = 2
STOP = 3


@dataclass
class CloudMetrics:
    received_at: float
    center_distance: float
    left_clearance: float
    right_clearance: float


def xyz_arrays(message: PointCloud2, stride: int) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Create zero-copy strided NumPy views over x/y/z FLOAT32 fields."""
    fields = {field.name: field for field in message.fields}
    for name in ('x', 'y', 'z'):
        if name not in fields or fields[name].datatype != PointField.FLOAT32:
            raise ValueError(f'PointCloud2 field {name} must be FLOAT32')
    count = int(message.width) * int(message.height)
    endian = '>' if message.is_bigendian else '<'
    dtype = np.dtype(endian + 'f4')
    step = max(1, int(stride))

    def view(name: str) -> np.ndarray:
        field = fields[name]
        return np.ndarray(
            shape=(count,), dtype=dtype, buffer=message.data,
            offset=int(field.offset), strides=(int(message.point_step),),
        )[::step]

    return view('x'), view('y'), view('z')


class ObstacleGuard(Node):
    def __init__(self) -> None:
        super().__init__('s10_obstacle_guard')
        self.declare_parameter('front_topic', '/rslidar_front/points')
        self.declare_parameter('rear_topic', '/rslidar_rear/points')
        self.declare_parameter('expected_frame_id', 'lidar_link')
        self.declare_parameter('cloud_timeout_sec', 0.50)
        self.declare_parameter('sample_stride', 4)
        self.declare_parameter('detection_length_m', 2.0)
        self.declare_parameter('corridor_half_width_m', 0.45)
        self.declare_parameter('side_width_m', 1.20)
        self.declare_parameter('min_z_m', -0.30)
        self.declare_parameter('max_z_m', 1.20)
        self.declare_parameter('self_min_x_m', -0.45)
        self.declare_parameter('self_max_x_m', 0.45)
        self.declare_parameter('self_half_width_m', 0.28)
        self.declare_parameter('emergency_distance_m', 0.60)
        self.declare_parameter('avoid_trigger_distance_m', 0.90)
        self.declare_parameter('slow_distance_m', 1.20)
        self.declare_parameter('minimum_side_clearance_m', 1.00)
        self.declare_parameter('enable_avoidance', True)
        self.declare_parameter('avoidance_speed_mps', 0.15)
        self.declare_parameter('avoidance_yaw_rate_rps', 0.30)
        self.declare_parameter('clear_confirm_cycles', 6)
        self.declare_parameter('update_rate_hz', 20.0)

        parameter = lambda name: self.get_parameter(name).value
        self.expected_frame = str(parameter('expected_frame_id'))
        self.timeout = float(parameter('cloud_timeout_sec'))
        self.sample_stride = int(parameter('sample_stride'))
        self.detection_length = float(parameter('detection_length_m'))
        self.corridor_half_width = float(parameter('corridor_half_width_m'))
        self.side_width = float(parameter('side_width_m'))
        self.min_z = float(parameter('min_z_m'))
        self.max_z = float(parameter('max_z_m'))
        self.self_min_x = float(parameter('self_min_x_m'))
        self.self_max_x = float(parameter('self_max_x_m'))
        self.self_half_width = float(parameter('self_half_width_m'))
        self.emergency_distance = float(parameter('emergency_distance_m'))
        self.avoid_trigger_distance = float(parameter('avoid_trigger_distance_m'))
        self.slow_distance = float(parameter('slow_distance_m'))
        self.minimum_side_clearance = float(parameter('minimum_side_clearance_m'))
        self.enable_avoidance = bool(parameter('enable_avoidance'))
        self.avoidance_speed = float(parameter('avoidance_speed_mps'))
        self.avoidance_yaw_rate = float(parameter('avoidance_yaw_rate_rps'))
        self.clear_confirm_cycles = int(parameter('clear_confirm_cycles'))

        if not (0.0 < self.emergency_distance < self.avoid_trigger_distance <
                self.slow_distance <= self.detection_length):
            raise ValueError('require emergency < avoid_trigger < slow <= detection_length')

        self.metrics: Dict[str, CloudMetrics] = {}
        self.current_state = STOP
        self.clear_cycles = 0

        self.create_subscription(
            PointCloud2, str(parameter('front_topic')),
            lambda message: self.on_cloud('front', message), qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, str(parameter('rear_topic')),
            lambda message: self.on_cloud('rear', message), qos_profile_sensor_data)
        self.state_pub = self.create_publisher(UInt8, '/s10_nav/obstacle_state', 10)
        self.scale_pub = self.create_publisher(Float32, '/s10_nav/obstacle_speed_scale', 10)
        self.distance_pub = self.create_publisher(Float32, '/s10_nav/obstacle_distance', 10)
        self.command_pub = self.create_publisher(
            TwistStamped, '/s10_nav/cmd_avoidance', 10)
        rate = max(1.0, float(parameter('update_rate_hz')))
        self.timer = self.create_timer(1.0 / rate, self.evaluate)

    def on_cloud(self, source: str, message: PointCloud2) -> None:
        if self.expected_frame and message.header.frame_id != self.expected_frame:
            self.get_logger().error(
                f'{source} cloud frame is {message.header.frame_id!r}, expected '
                f'{self.expected_frame!r}; cloud ignored',
                throttle_duration_sec=2.0,
            )
            return
        try:
            x, y, z = xyz_arrays(message, self.sample_stride)
        except (ValueError, TypeError, BufferError) as error:
            self.get_logger().error(str(error), throttle_duration_sec=2.0)
            return

        finite = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
        height = (z >= self.min_z) & (z <= self.max_z)
        forward = (x > self.self_max_x) & (x <= self.detection_length)
        not_self = ~((x >= self.self_min_x) & (x <= self.self_max_x) &
                     (np.abs(y) <= self.self_half_width))
        valid = finite & height & forward & not_self

        x_valid = x[valid]
        y_valid = y[valid]
        if x_valid.size == 0:
            self.metrics[source] = CloudMetrics(
                time.monotonic(), self.detection_length,
                self.detection_length, self.detection_length)
            return

        ranges = np.hypot(x_valid, y_valid)
        center = np.abs(y_valid) <= self.corridor_half_width
        left = (y_valid > self.corridor_half_width) & (y_valid <= self.side_width)
        right = (y_valid < -self.corridor_half_width) & (y_valid >= -self.side_width)

        def clearance(mask: np.ndarray) -> float:
            return float(np.min(ranges[mask])) if np.any(mask) else self.detection_length

        self.metrics[source] = CloudMetrics(
            received_at=time.monotonic(),
            center_distance=clearance(center),
            left_clearance=clearance(left),
            right_clearance=clearance(right),
        )

    def aggregate(self) -> Optional[CloudMetrics]:
        now = time.monotonic()
        current = [metric for metric in self.metrics.values()
                   if now - metric.received_at <= self.timeout]
        if not current:
            return None
        return CloudMetrics(
            received_at=now,
            center_distance=min(metric.center_distance for metric in current),
            left_clearance=min(metric.left_clearance for metric in current),
            right_clearance=min(metric.right_clearance for metric in current),
        )

    def desired_state(self, metric: CloudMetrics) -> int:
        if metric.center_distance <= self.emergency_distance:
            return STOP
        if metric.center_distance <= self.avoid_trigger_distance:
            best_side = max(metric.left_clearance, metric.right_clearance)
            if self.enable_avoidance and best_side >= self.minimum_side_clearance:
                return AVOID
            return STOP
        if metric.center_distance <= self.slow_distance:
            return SLOW
        return CLEAR

    def evaluate(self) -> None:
        metric = self.aggregate()
        desired = STOP if metric is None else self.desired_state(metric)

        if desired == CLEAR and self.current_state != CLEAR:
            self.clear_cycles += 1
            if self.clear_cycles >= self.clear_confirm_cycles:
                self.current_state = CLEAR
                self.clear_cycles = 0
        else:
            self.clear_cycles = 0
            self.current_state = desired

        speed_scale = 0.0
        command = TwistStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'

        if metric is not None:
            if self.current_state == CLEAR:
                speed_scale = 1.0
            elif self.current_state == SLOW:
                interval = self.slow_distance - self.avoid_trigger_distance
                speed_scale = max(
                    0.20,
                    min(1.0, (metric.center_distance - self.avoid_trigger_distance) /
                        max(interval, 1e-3)),
                )
            elif self.current_state == AVOID:
                speed_scale = 1.0
                direction = 1.0 if metric.left_clearance >= metric.right_clearance else -1.0
                command.twist.linear.x = self.avoidance_speed
                command.twist.angular.z = direction * self.avoidance_yaw_rate

        nearest = 0.0 if metric is None else metric.center_distance
        self.state_pub.publish(UInt8(data=self.current_state))
        self.scale_pub.publish(Float32(data=float(speed_scale)))
        self.distance_pub.publish(Float32(data=float(nearest)))
        self.command_pub.publish(command)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ObstacleGuard()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
