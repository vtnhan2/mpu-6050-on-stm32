#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('imu')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyUSB0',
        description='Serial port for IMU communication'
    )
    
    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value='921600',
        description='Serial baudrate for IMU communication'
    )
    
    # IMU Node
    imu_node = Node(
        package='imu',
        executable='imu_python_node.py',
        name='imu_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'serial_port': LaunchConfiguration('serial_port'),
            'serial_baudrate': LaunchConfiguration('serial_baudrate'),
        }],
        remappings=[
            ('imu/data', 'imu/data'),
            ('imu/mag', 'imu/mag'),
            ('imu/temp', 'imu/temp'),
        ]
    )
    
    # Static transform publisher for base_link to imu_link
    static_transform_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_imu_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link'],
        output='screen'
    )
    
    # RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(pkg_dir, 'config', 'imu.rviz')],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        serial_port_arg,
        serial_baudrate_arg,
        imu_node,
        static_transform_publisher,
        rviz_node,
    ])
