from launch import LaunchDescription
from launch.actions import ExecuteProcess
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    share_dir = get_package_share_directory('hive_gazebo')
    world = os.path.join(share_dir, 'worlds', 'empty_world.world')

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gzserver', world, '-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['gzclient'],
            output='screen'
        ),
    ])
