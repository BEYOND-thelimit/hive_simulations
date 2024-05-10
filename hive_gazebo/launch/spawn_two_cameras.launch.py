import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def camera(share_dir,camera_name):
    urdf_file_path = os.path.join(share_dir, 'urdf', 'simple_cameras', camera_name + '.urdf')

    # URDF 스폰 노드
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', camera_name,
            '-file', urdf_file_path,  # '-file' 옵션으로 변경
        ],
        output='screen'
    )

    # 카메라 상태 발행 노드
    camera_state_publisher = Node(
        name='camera_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=camera_name,
        output='screen',
        arguments=[urdf_file_path]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=camera_name,
        parameters=[
            {'use_sim_time': True}
        ]
    )
    return [urdf_spawn_node, camera_state_publisher, joint_state_publisher_node]


def generate_launch_description():
    share_dir = get_package_share_directory('hive_gazebo')
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
    rviz_config_file = os.path.join(share_dir, 'rviz', 'two_cameras.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )
    cam_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '2.5', '0', '0', '0', 'world', 'ceiling_link']
    )

    camera1_urdf_spawn_node, camera1_state_publisher, \
    camera1_joint_state_publisher = camera(share_dir=share_dir, camera_name='camera1')

    camera2_urdf_spawn_node, camera2_state_publisher, \
    camera2_joint_state_publisher = camera(share_dir=share_dir, camera_name='camera2')
    return LaunchDescription([
        rviz_node,
        gui_arg,
        gazebo,
        cam_pub,
        camera1_urdf_spawn_node,
        camera1_state_publisher,
        camera1_joint_state_publisher,
        camera2_urdf_spawn_node,
        camera2_state_publisher,
        camera2_joint_state_publisher
    ])
