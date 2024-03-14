#!/usr/bin/python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import OpaqueFunction


def generate_launch_description():
    st_pub1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        namespace='robot1',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'robot1_odom']
    )

    st_pub2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        namespace='robot2',
        output='screen',
        emulate_tty=True,
        arguments=['2', '0', '0', '0', '0', '0', 'world', 'robot2_odom']
    )

    st_pub3 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        namespace='robot3',
        output='screen',
        emulate_tty=True,
        arguments=['0', '2', '0', '0', '0', '0', 'world', 'robot3_odom']
    )

    return LaunchDescription([
        st_pub1,
        st_pub2,
        st_pub3
        ])
