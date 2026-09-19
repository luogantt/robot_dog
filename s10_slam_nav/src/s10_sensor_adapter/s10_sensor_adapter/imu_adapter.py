#!/usr/bin/env python3
"""Scale and rotate Airy IMU vectors into the body-aligned lidar_link frame.

Bench measurements on 2026-09-06 (robot static, front Airy) showed two
rs_driver/RSAIRY quirks that this node compensates:

1. Acceleration is emitted in g units (static reading magnitude ~1.0) while the
   sensor_msgs/Imu convention is m/s^2.  Gyroscope is already converted to
   rad/s.  accel_scale = 9.80665 restores m/s^2.
2. The raw IMU frame's "up" axis is -y (static reading ~ -1.0 g on y), whereas
   the driver-transformed cloud in lidar_link is z-up (verified ground-plane
   normal ~= -z).  Rotating the cloud extrinsic (roll=0, pitch=-90 deg,
   yaw=-180 deg) onto the raw IMU leaves gravity on +y, so the IMU rotation
   does NOT equal the cloud extrinsic rotation.  roll = -90 deg maps raw up
   (-y) to lidar_link +z; the in-plane yaw about vertical is unobservable
   while static and must be confirmed/adjusted in the first motion test.
"""

import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

from .math_utils import rotate_covariance, rotate_vector, rpy_matrix


class ImuAdapter(Node):
    """Publish /IMU in lidar_link with correct units, vertical and orientation validity."""

    def __init__(self) -> None:
        super().__init__('s10_imu_adapter')
        self.declare_parameter('input_topic', '/rslidar_front/imu')
        self.declare_parameter('output_topic', '/IMU')
        self.declare_parameter('output_frame_id', 'lidar_link')
        self.declare_parameter('roll', -1.5707963)
        self.declare_parameter('pitch', 0.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('accel_scale', 9.80665)
        self.declare_parameter('warn_gap_sec', 0.10)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        self.warn_gap_sec = float(self.get_parameter('warn_gap_sec').value)
        self.accel_scale = float(self.get_parameter('accel_scale').value)
        self.rotation = rpy_matrix(
            float(self.get_parameter('roll').value),
            float(self.get_parameter('pitch').value),
            float(self.get_parameter('yaw').value),
        )

        # Lightning subscribes with the default reliable rclcpp::QoS(10); the
        # rslidar driver also publishes reliable.  Sensor-data (best_effort)
        # would make /IMU incompatible with Lightning's reliable subscription.
        self.publisher = self.create_publisher(Imu, output_topic, 10)
        self.subscription = self.create_subscription(
            Imu, input_topic, self.on_imu, 10)
        self.last_stamp_sec = None
        self.last_receive_monotonic = None
        self.received = 0

        self.get_logger().info(
            f'Airy IMU adapter: {input_topic} -> {output_topic}, frame={self.output_frame_id}')

    def on_imu(self, source: Imu) -> None:
        stamp_sec = float(source.header.stamp.sec) + float(source.header.stamp.nanosec) * 1e-9
        now_monotonic = time.monotonic()
        if self.last_stamp_sec is not None:
            if stamp_sec <= self.last_stamp_sec:
                self.get_logger().error(
                    f'non-monotonic IMU stamp: current={stamp_sec:.9f}, '
                    f'previous={self.last_stamp_sec:.9f}',
                    throttle_duration_sec=1.0,
                )
            elif stamp_sec - self.last_stamp_sec > self.warn_gap_sec:
                self.get_logger().warning(
                    f'IMU timestamp gap {(stamp_sec - self.last_stamp_sec) * 1000.0:.1f} ms',
                    throttle_duration_sec=1.0,
                )

        target = Imu()
        target.header.stamp = source.header.stamp
        target.header.frame_id = self.output_frame_id

        angular = rotate_vector(
            self.rotation,
            [source.angular_velocity.x, source.angular_velocity.y, source.angular_velocity.z],
        )
        linear = rotate_vector(
            self.rotation,
            [source.linear_acceleration.x, source.linear_acceleration.y,
             source.linear_acceleration.z],
        )
        # rs_driver/RSAIRY reports acceleration in g; convert to m/s^2.
        linear = [value * self.accel_scale for value in linear]
        target.angular_velocity.x, target.angular_velocity.y, target.angular_velocity.z = angular
        target.linear_acceleration.x, target.linear_acceleration.y, target.linear_acceleration.z = linear

        # Airy messages contain angular velocity and acceleration only.  Identity is
        # used as a harmless value, while covariance[0] = -1 explicitly marks it invalid.
        target.orientation.x = 0.0
        target.orientation.y = 0.0
        target.orientation.z = 0.0
        target.orientation.w = 1.0
        target.orientation_covariance[0] = -1.0
        target.angular_velocity_covariance = rotate_covariance(
            self.rotation, source.angular_velocity_covariance)
        acceleration_covariance = rotate_covariance(
            self.rotation, source.linear_acceleration_covariance)
        if len(acceleration_covariance) == 9 and acceleration_covariance[0] >= 0.0:
            acceleration_covariance = [
                value * self.accel_scale ** 2 for value in acceleration_covariance]
        target.linear_acceleration_covariance = acceleration_covariance

        self.publisher.publish(target)
        self.last_stamp_sec = stamp_sec
        self.last_receive_monotonic = now_monotonic
        self.received += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ImuAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
