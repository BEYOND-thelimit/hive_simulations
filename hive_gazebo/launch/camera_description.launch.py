# Copyright 2023 Intel Corporation. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# /* Author: Doron Hirshberg */
import os
import launch
from launch_ros.actions import Node, PushRosNamespace
import launch.events
from ament_index_python.packages import get_package_share_directory
import sys
sys.path.append(os.path.dirname(os.path.realpath(__file__)))
from launch_utils import to_urdf


def generate_launch_description():
    camera_model = 'test_d435i_camera.urdf.xacro'

    rviz_config_dir = os.path.join(get_package_share_directory('hive_gazebo'), 'rviz', 'camera.rviz')
    xacro_path = os.path.join(get_package_share_directory('hive_gazebo'), 'urdf', 'camera1', camera_model)
    urdf = to_urdf(xacro_path, {'use_nominal_extrinsics': 'true', 'add_plug': 'true'})
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_dir],
        parameters=[{'use_sim_time': False}]
        )

    camera_state_publisher = Node(
        name='camera_state_publisher',
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace='camera1',
        output='screen',
        arguments=[urdf]
        )

    cam_pub1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        emulate_tty=True,
        arguments=['0', '0', '3.5', '0', '0', '0', 'world', 'ceiling_link']
    )
    return launch.LaunchDescription([rviz_node, camera_state_publisher,cam_pub1])
