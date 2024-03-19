import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # URDF 파일 경로 설정
    urdf_file_path = os.path.join(get_package_share_directory('hive_gazebo'), 'urdf', 'simple_cameras', 'camera1.urdf')

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
    cam_pub1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '3.5', '0', '0', '0', 'world', 'ceiling_link']
    )

    # URDF 스폰 노드
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'camera1',
            '-file', urdf_file_path,  # '-file' 옵션으로 변경
        ],
        output='screen'
    )

    # 카메라 상태 발행 노드
    camera_state_publisher = Node(
        name='camera_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='camera1',
        output='screen',
        arguments=[urdf_file_path]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace='camera1',
        parameters=[
            {'use_sim_time': True}
        ]
    )
    world = os.path.join(
        get_package_share_directory('hive_gazebo'),
        'worlds',
        'empty_world.world'
    )
    return LaunchDescription([
        gazebo,
        cam_pub1,
        urdf_spawn_node,
        camera_state_publisher,
        joint_state_publisher_node,
        
    ])
