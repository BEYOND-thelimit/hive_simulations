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
    xacro_file = os.path.join(share_dir, 'urdf', robot_name, robot_name+'.xacro')
    robot_description_config1 = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config1.toxml()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace=robot_name,
        parameters=[
            {'robot_description': robot_urdf},
            {'use_sim_time': True}
        ]
    )
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        namespace=robot_name,
        parameters=[
            {'use_sim_time': True}
        ]
    )
    urdf_spawn_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', robot_name,
            '-topic', robot_name+'/robot_description',
            '-x', x ,'-y', y,'-z', '0.0575'
        ],
        output='screen'
    )

    return robot_state_publisher_node, joint_state_publisher_node, urdf_spawn_node

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
    rviz_config_file = os.path.join(share_dir, 'rviz', 'main.rviz')

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

    odom_to_world_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('hive_gazebo'),
                'launch',
                'static_odom_to_world.launch.py'
            ])
        ]),
        launch_arguments={
            'pause': 'true'
        }.items()
    )

    cam_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '3.5', '0', '0', '0', 'world', 'ceiling_link']
    )

    robot1_robot_state_publisher, \
    robot1_joint_state_publisher, \
    robot1_urdf_node = robot(share_dir=share_dir,robot_name='robot1',x='0',y='0')

    robot2_robot_state_publisher, \
    robot2_joint_state_publisher, \
    robot2_urdf_node = robot(share_dir=share_dir,robot_name='robot2',x='0',y='1')

    robot3_robot_state_publisher, \
    robot3_joint_state_publisher, \
    robot3_urdf_node = robot(share_dir=share_dir,robot_name='robot3',x='0',y='2')

    camera1_urdf_spawn_node, camera1_state_publisher, \
    camera1_joint_state_publisher = camera(share_dir=share_dir, camera_name='camera1')

    camera2_urdf_spawn_node, camera2_state_publisher, \
    camera2_joint_state_publisher = camera(share_dir=share_dir, camera_name='camera2')


    return LaunchDescription([
        rviz_node,
        gui_arg,
        gazebo,

        # 로봇 관련 노드
        odom_to_world_publisher,
        robot1_robot_state_publisher,
        robot1_joint_state_publisher,
        robot1_urdf_node,
        robot2_robot_state_publisher,
        robot2_joint_state_publisher,
        robot2_urdf_node,
        robot3_robot_state_publisher,
        robot3_joint_state_publisher,
        robot3_urdf_node,

        # 카메라 관련 노드
        cam_pub,
        camera1_urdf_spawn_node,
        camera1_state_publisher,
        camera1_joint_state_publisher,
        camera2_urdf_spawn_node,
        camera2_state_publisher,
        camera2_joint_state_publisher
    ])