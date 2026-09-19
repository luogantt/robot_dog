#!/usr/bin/env python3
"""Fail-closed health gate for localization, sensors, route and point schema."""

import math
import time
from typing import Dict, List

from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Imu, PointCloud2, PointField
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .common import quaternion_to_yaw, wrap_angle


EXPECTED_POINT_FIELDS = {
    'x': PointField.FLOAT32,
    'y': PointField.FLOAT32,
    'z': PointField.FLOAT32,
    'intensity': PointField.FLOAT32,
    'ring': PointField.UINT16,
    'timestamp': PointField.FLOAT64,
}


class HealthMonitor(Node):
    def __init__(self) -> None:
        super().__init__('s10_health_monitor')
        self.declare_parameter('odom_topic', '/lightning/odom')
        self.declare_parameter('imu_topic', '/IMU')
        self.declare_parameter('front_cloud_topic', '/rslidar_front/points')
        self.declare_parameter('rear_cloud_topic', '/rslidar_rear/points')
        self.declare_parameter('require_rear_cloud', True)
        self.declare_parameter('odom_timeout_sec', 0.30)
        self.declare_parameter('imu_timeout_sec', 0.20)
        self.declare_parameter('cloud_timeout_sec', 0.50)
        self.declare_parameter('max_position_jump_m', 0.50)
        self.declare_parameter('max_yaw_jump_deg', 15.0)
        self.declare_parameter('stable_samples', 10)
        self.declare_parameter('update_rate_hz', 20.0)

        parameter = lambda name: self.get_parameter(name).value
        self.require_rear = bool(parameter('require_rear_cloud'))
        self.timeouts = {
            'odom': float(parameter('odom_timeout_sec')),
            'imu': float(parameter('imu_timeout_sec')),
            'front_cloud': float(parameter('cloud_timeout_sec')),
            'rear_cloud': float(parameter('cloud_timeout_sec')),
        }
        self.max_position_jump = float(parameter('max_position_jump_m'))
        self.max_yaw_jump = math.radians(float(parameter('max_yaw_jump_deg')))
        self.required_stable_samples = int(parameter('stable_samples'))

        self.last_received: Dict[str, float] = {}
        self.schema_valid = {'front_cloud': False, 'rear_cloud': False}
        self.schema_checked = {'front_cloud': False, 'rear_cloud': False}
        self.last_pose = None
        self.latched_fault = ''
        self.route_fault = False
        self.stable_count = 0
        self.healthy = False

        self.create_subscription(
            Odometry, str(parameter('odom_topic')), self.on_odom, 10)
        # The /IMU stream runs at 200 Hz inside a node that also handles
        # clouds, odometry and a 20 Hz evaluation timer.  A best-effort
        # depth-10 subscription overflows whenever the executor is briefly
        # busy, which reads as spurious "imu stale" faults.  Reliable with a
        # larger history lets the 200 Hz stream queue across those hiccups.
        imu_qos = QoSProfile(depth=200)
        self.create_subscription(
            Imu, str(parameter('imu_topic')),
            lambda message: self.mark_received('imu'), imu_qos)
        self.create_subscription(
            PointCloud2, str(parameter('front_cloud_topic')),
            lambda message: self.on_cloud('front_cloud', message), qos_profile_sensor_data)
        self.create_subscription(
            PointCloud2, str(parameter('rear_cloud_topic')),
            lambda message: self.on_cloud('rear_cloud', message), qos_profile_sensor_data)
        self.create_subscription(Bool, '/s10_nav/route_fault', self.on_route_fault, 10)

        self.health_pub = self.create_publisher(Bool, '/s10_nav/healthy', 10)
        self.diagnostic_pub = self.create_publisher(
            DiagnosticArray, '/s10_nav/diagnostics', 10)
        self.create_service(Trigger, '/s10_health/reset_fault', self.reset_fault)
        rate = max(1.0, float(parameter('update_rate_hz')))
        self.timer = self.create_timer(1.0 / rate, self.evaluate)

    def mark_received(self, name: str) -> None:
        self.last_received[name] = time.monotonic()

    def on_cloud(self, name: str, message: PointCloud2) -> None:
        self.mark_received(name)
        if not self.schema_checked[name]:
            actual = {field.name: field.datatype for field in message.fields}
            invalid = [field_name for field_name, datatype in EXPECTED_POINT_FIELDS.items()
                       if actual.get(field_name) != datatype]
            self.schema_valid[name] = not invalid
            self.schema_checked[name] = True
            if invalid:
                self.latched_fault = f'{name} invalid fields: {", ".join(invalid)}'
                self.get_logger().error(self.latched_fault)
            else:
                self.get_logger().info(f'{name} XYZIRT schema verified')

    def on_odom(self, message: Odometry) -> None:
        self.mark_received('odom')
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        pose = (
            float(position.x),
            float(position.y),
            quaternion_to_yaw(orientation.x, orientation.y, orientation.z, orientation.w),
        )
        if not all(math.isfinite(value) for value in pose):
            self.latched_fault = 'non-finite localization pose'
        elif self.last_pose is not None:
            distance = math.hypot(pose[0] - self.last_pose[0], pose[1] - self.last_pose[1])
            yaw_jump = abs(wrap_angle(pose[2] - self.last_pose[2]))
            if distance > self.max_position_jump:
                self.latched_fault = f'localization position jump: {distance:.3f} m'
            elif yaw_jump > self.max_yaw_jump:
                self.latched_fault = (
                    f'localization yaw jump: {math.degrees(yaw_jump):.1f} deg')
        self.last_pose = pose

    def on_route_fault(self, message: Bool) -> None:
        self.route_fault = bool(message.data)
        if self.route_fault:
            self.latched_fault = 'route follower fault'

    def reset_fault(self, _request: Trigger.Request,
                    response: Trigger.Response) -> Trigger.Response:
        self.latched_fault = ''
        self.route_fault = False
        self.last_pose = None
        self.stable_count = 0
        self.healthy = False
        response.success = True
        response.message = 'health fault reset; waiting for stable samples'
        self.get_logger().warning(response.message)
        return response

    def current_reasons(self) -> List[str]:
        now = time.monotonic()
        reasons = []
        required = ['odom', 'imu', 'front_cloud']
        if self.require_rear:
            required.append('rear_cloud')
        for name in required:
            if name not in self.last_received:
                reasons.append(f'{name} not received')
            elif now - self.last_received[name] > self.timeouts[name]:
                reasons.append(f'{name} stale {now - self.last_received[name]:.3f}s')
        for name in ('front_cloud', 'rear_cloud'):
            if name == 'rear_cloud' and not self.require_rear:
                continue
            if self.schema_checked[name] and not self.schema_valid[name]:
                reasons.append(f'{name} schema invalid')
        if self.latched_fault:
            reasons.append(self.latched_fault)
        return reasons

    def evaluate(self) -> None:
        reasons = self.current_reasons()
        if reasons:
            self.stable_count = 0
            self.healthy = False
        else:
            self.stable_count += 1
            self.healthy = self.stable_count >= self.required_stable_samples

        self.health_pub.publish(Bool(data=self.healthy))
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = 's10_navigation_health'
        status.hardware_id = 'S10'
        status.level = DiagnosticStatus.OK if self.healthy else DiagnosticStatus.ERROR
        status.message = 'ready' if self.healthy else '; '.join(reasons or ['stabilizing'])
        status.values = [
            KeyValue(key='stable_samples', value=str(self.stable_count)),
            KeyValue(key='latched_fault', value=self.latched_fault or 'none'),
            KeyValue(key='front_schema', value=str(self.schema_valid['front_cloud'])),
            KeyValue(key='rear_schema', value=str(self.schema_valid['rear_cloud'])),
        ]
        array.status.append(status)
        self.diagnostic_pub.publish(array)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = HealthMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
