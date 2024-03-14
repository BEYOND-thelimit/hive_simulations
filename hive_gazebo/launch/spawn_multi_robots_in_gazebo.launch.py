from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
import os
import xacro
from ament_index_python.packages import get_package_share_directory

def robot(share_dir,robot_name,x,y):
    xacro_file1 = os.path.join(share_dir, 'urdf', robot_name, robot_name+'.xacro')
    robot_description_config1 = xacro.process_file(xacro_file1)
    robot_urdf1 = robot_description_config1.toxml()

    rviz_config_file1 = os.path.join(share_dir, 'rviz', robot_name+'.rviz')

    robot_state_publisher_node1 = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name,
        parameters=[
            {'robot_description': robot_urdf1},
            {'use_sim_time': True}
        ]
    )
    joint_state_publisher_node1 = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=robot_name,
        parameters=[
            {'use_sim_time': True}
        ]
    )
    urdf_spawn_node1 = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-topic', robot_name+'/robot_description',
            '-x', x,'-y', y,'-z', '0'
        ],
        output='screen'
    )

    rviz_node1 = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file1],
        output='screen'
    )

    return robot_state_publisher_node1, joint_state_publisher_node1, urdf_spawn_node1, rviz_node1


def generate_launch_description():
    share_dir = get_package_share_directory('hive_gazebo')

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('hive_gazebo'),
                'launch',
                'gz_plugins.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'true'
        }.items()
    )

    camera_rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('hive_gazebo'),
                'launch',
                'camera_description.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'true'
        }.items()
    )
    robot1_robot_state_publisher, \
    robot1_joint_state_publisher, \
    robot1_urdf_node, \
    robot1_rviz_node= robot(share_dir=share_dir,robot_name='robot1', x='0', y='0')

    robot2_robot_state_publisher, \
    robot2_joint_state_publisher, \
    robot2_urdf_node, \
    robot2_rviz_node= robot(share_dir=share_dir,robot_name='robot2', x='2', y='0')

    robot3_robot_state_publisher, \
    robot3_joint_state_publisher, \
    robot3_urdf_node, \
    robot3_rviz_node= robot(share_dir=share_dir,robot_name='robot3', x='0', y='2')


    return LaunchDescription([
        gui_arg,
        gazebo,
        robot1_robot_state_publisher,
        robot1_joint_state_publisher,
        robot1_urdf_node,
        robot1_rviz_node,

        robot2_robot_state_publisher,
        robot2_joint_state_publisher,
        robot2_urdf_node,
        #robot2_rviz_node,

        robot3_robot_state_publisher,
        robot3_joint_state_publisher,
        robot3_urdf_node,
        # robot3_rviz_node
    ])