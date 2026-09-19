"""Launch sensors, IMU adaptation and Lightning online mapping."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('s10_nav_bringup'))
    sensor_share = Path(get_package_share_directory('s10_sensor_adapter'))

    driver_config = LaunchConfiguration('driver_config')
    lightning_config = LaunchConfiguration('lightning_config')
    start_driver = LaunchConfiguration('start_driver')

    return LaunchDescription([
        DeclareLaunchArgument(
            'driver_config', default_value=str(share / 'config' / 'airy_dual.yaml')),
        DeclareLaunchArgument(
            'lightning_config',
            default_value=str(share / 'config' / 's10_lightning_slam.yaml')),
        DeclareLaunchArgument('start_driver', default_value='true'),
        Node(
            package='rslidar_sdk',
            executable='rslidar_sdk_node',
            name='rslidar_sdk_node',
            output='screen',
            parameters=[{'config_path': driver_config}],
            condition=IfCondition(start_driver),
        ),
        Node(
            package='s10_sensor_adapter',
            executable='imu_adapter',
            name='s10_imu_adapter',
            output='screen',
            parameters=[str(sensor_share / 'config' / 'imu_adapter.yaml')],
        ),
        Node(
            package='lightning',
            executable='run_slam_online',
            output='screen',
            arguments=[['--config=', lightning_config]],
        ),
    ])
