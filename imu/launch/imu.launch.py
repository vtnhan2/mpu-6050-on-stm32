#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for STM32 communication'
    )
    
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='Baud rate for serial communication'
    )
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='imu_link',
        description='Frame ID for IMU data'
    )
    
    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='100.0',
        description='IMU data publish rate in Hz'
    )
    
    # IMU Node
    imu_node = Node(
        package='imu',
        executable='imu_node',
        name='imu_node',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'frame_id': LaunchConfiguration('frame_id'),
            'publish_rate': LaunchConfiguration('publish_rate'),
        }],
        remappings=[
            ('imu/data', 'imu/data_raw'),
        ]
    )
    
    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        frame_id_arg,
        publish_rate_arg,
        imu_node,
    ])
